/*
 * BR/EDR side of the S31 "JBL speaker":
 *   - dual-mode controller (BLE for Auracast + Classic for phones)
 *   - discoverable as a loudspeaker, A2DP sink (SBC)
 *   - decoded phone audio is resampled to 48 kHz and handed to the Auracast
 *     encoder, so music played to the S31 is re-broadcast, like the Xtreme 5
 *     does when it hosts.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_a2dp_legacy_api.h"

#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "nvs.h"
#include "esp_memory_utils.h"

#include "a2dp_bridge.h"

#define TAG "JBL_BREDR"

#define OUT_RATE        48000
#define MS_BYTES(ms)    (OUT_RATE * 4 / 1000 * (ms))
#define RB_BYTES        MS_BYTES(400)           /* 400 ms of 48 kHz stereo s16 */
#define PRIME_BYTES     MS_BYTES(120)           /* start reading after 120 ms */
#define TARGET_BYTES    MS_BYTES(150)           /* steady-state fill the drift control aims for */

static volatile uint32_t s_underruns;           /* encoder found too little phone audio */
static volatile uint32_t s_drops;               /* phone audio dropped (buffer full) */

static RingbufHandle_t s_rb;
static volatile uint32_t s_src_rate = 44100;
static volatile bool s_streaming;
static volatile bool s_primed;
static volatile uint32_t s_in_pkts;
static volatile bool s_a2dp_connected;
static uint8_t s_peer[6];                   /* the one phone we're attached to */
static volatile bool s_have_peer;
static volatile bool s_retry_after_loss;
static volatile bool s_acl_up;              /* a Classic link to the last phone exists (in or out) */
static volatile uint32_t s_acl_change_ms;   /* when that link last came up or went down */

static void remember_phone(const uint8_t *a);
extern uint8_t *BTM_ReadLocalFeatures(void);    /* Bluedroid: LMP features page 0 */

/* All ACL packet types, EDR (2/3 Mbps) DH1/DH3/DH5 allowed (no "NO_x_DHy" bits). */
#define ACL_PKT_ALL_EDR (ESP_BT_ACL_PKT_TYPES_MASK_DM1 | ESP_BT_ACL_PKT_TYPES_MASK_DH1 |                          ESP_BT_ACL_PKT_TYPES_MASK_DM3 | ESP_BT_ACL_PKT_TYPES_MASK_DH3 |                          ESP_BT_ACL_PKT_TYPES_MASK_DM5 | ESP_BT_ACL_PKT_TYPES_MASK_DH5)
static void (*s_name_cb)(const char *name);

void a2dp_bridge_on_phone_name(void (*cb)(const char *name))
{
    s_name_cb = cb;
}

/* ---- 44.1/48 kHz -> 48 kHz resampler --------------------------------------
 * Windowed-sinc polyphase FIR: 32 taps, 256 phases with linear interpolation
 * between phases, Blackman window, cutoff 20 kHz. Linear interpolation (the
 * old resampler) leaves spectral images only ~20-25 dB down that fold back
 * into the audible band (a 10 kHz cymbal -> a false ~14 kHz tone): a harsh,
 * "low-bitrate" sound the LC3 encoder then wastes bits on. This keeps them
 * ~70 dB down. The ratio can still be nudged +-0.2 % for drift control. */
#define RS_TAPS    32
#define RS_HALF    (RS_TAPS / 2)
#define RS_PHASES  256
#define RS_HIST    4096                        /* staged input frames */
static float s_kern[RS_PHASES + 1][RS_TAPS];
static float s_hist[2][RS_HIST];
static int s_hlen;                             /* valid frames in s_hist */
static double s_rpos;                          /* read position (frames) in s_hist */
static volatile bool s_rs_reset = true;

static void rs_init_kernel(void)
{
    const double fc = 20000.0 / 44100.0;      /* cutoff as a fraction of the input rate */
    for (int p = 0; p <= RS_PHASES; p++) {
        double frac = (double)p / RS_PHASES;
        double sum = 0;
        for (int k = 0; k < RS_TAPS; k++) {
            double t = (double)k - (RS_HALF - 1) - frac;         /* tap offset in input samples */
            double x = 2.0 * fc * t;
            double sinc = fabs(x) < 1e-9 ? 1.0 : sin(M_PI * x) / (M_PI * x);
            double w = 0.42 + 0.5 * cos(M_PI * t / RS_HALF) + 0.08 * cos(2.0 * M_PI * t / RS_HALF);
            if (fabs(t) >= RS_HALF) w = 0;
            s_kern[p][k] = (float)(2.0 * fc * sinc * w);
            sum += s_kern[p][k];
        }
        for (int k = 0; k < RS_TAPS; k++) {
            s_kern[p][k] = (float)(s_kern[p][k] / sum);         /* unity gain at DC for every phase */
        }
    }
}

static uint8_t s_out_buf[4096 * 4];   /* 4096 stereo frames at 48 kHz */

/* ---- audio path --------------------------------------------------------- */

static void a2d_data_cb(const uint8_t *data, uint32_t len)
{
    const int16_t *in = (const int16_t *)data;
    int n = len / 4;                       /* stereo frames */
    if (n <= 0 || s_rb == NULL) {
        return;
    }
    s_in_pkts++;

    int16_t *out = (int16_t *)s_out_buf;
    const int out_max = sizeof(s_out_buf) / 4;
    int o = 0;

    if (s_rs_reset) {
        s_rs_reset = false;
        memset(s_hist, 0, sizeof(s_hist));
        s_hlen = RS_TAPS;                  /* start on a short run of silence */
        s_rpos = RS_HALF - 1;
    }
    if (n > RS_HIST - s_hlen) {
        n = RS_HIST - s_hlen;              /* never happens with SBC packets (< 2k frames) */
    }
    for (int j = 0; j < n; j++) {
        s_hist[0][s_hlen + j] = in[2 * j];
        s_hist[1][s_hlen + j] = in[2 * j + 1];
    }
    s_hlen += n;

    {
        /* Drift control: nudge the resampling ratio (max +-0.2 %) so the
         * buffer hovers around TARGET_BYTES; the phone's clock and ours differ. */
        float fill = (float)(RB_BYTES - xRingbufferGetCurFreeSize(s_rb));
        float e = (fill - (float)TARGET_BYTES) / (float)TARGET_BYTES;
        if (e > 1.0f) e = 1.0f;
        if (e < -1.0f) e = -1.0f;
        const double step = ((double)s_src_rate / (double)OUT_RATE) * (1.0 + 0.002 * e);
        double pos = s_rpos;
        while ((int)pos + RS_HALF < s_hlen && o < out_max) {
            int i = (int)pos;
            float ph = (float)(pos - i) * RS_PHASES;
            int p0 = (int)ph;
            float t = ph - (float)p0;
            const float *k0 = s_kern[p0], *k1 = s_kern[p0 + 1];
            const float *x0 = &s_hist[0][i - (RS_HALF - 1)];
            const float *x1 = &s_hist[1][i - (RS_HALF - 1)];
            float acc0 = 0, acc1 = 0;
            for (int k = 0; k < RS_TAPS; k++) {
                float c = k0[k] + (k1[k] - k0[k]) * t;
                acc0 += c * x0[k];
                acc1 += c * x1[k];
            }
            if (acc0 > 32767.f) acc0 = 32767.f; else if (acc0 < -32768.f) acc0 = -32768.f;
            if (acc1 > 32767.f) acc1 = 32767.f; else if (acc1 < -32768.f) acc1 = -32768.f;
            out[2 * o] = (int16_t)lrintf(acc0);
            out[2 * o + 1] = (int16_t)lrintf(acc1);
            o++;
            pos += step;
        }
        /* drop consumed input, keep the taps' history */
        int keep_from = (int)pos - (RS_HALF - 1);
        if (keep_from > 0) {
            s_hlen -= keep_from;
            memmove(s_hist[0], s_hist[0] + keep_from, s_hlen * sizeof(float));
            memmove(s_hist[1], s_hist[1] + keep_from, s_hlen * sizeof(float));
            pos -= keep_from;
        }
        s_rpos = pos;
    }

    /* Drop if the encoder falls behind (keeps latency bounded). */
    if (o > 0 && xRingbufferSend(s_rb, out, o * 4, 0) != pdTRUE) {
        s_drops++;
    }
}

bool a2dp_bridge_read(int16_t *pcm, int frames)
{
    size_t want = frames * 4;
    if (s_rb == NULL || !s_streaming) {
        s_primed = false;
        if (s_a2dp_connected) {
            memset(pcm, 0, want);          /* phone connected but paused: silence, like a real speaker */
            return true;
        }
        return false;                      /* no phone at all: caller plays the test tone */
    }
    size_t buffered = RB_BYTES - xRingbufferGetCurFreeSize(s_rb);
    if (!s_primed) {
        if (buffered < PRIME_BYTES) {
            memset(pcm, 0, want);          /* streaming but still filling: silence */
            return true;
        }
        s_primed = true;
    }
    if (buffered < want) {
        /* Underrun mid-song: play silence and re-prime; never fall back to the tone. */
        s_primed = false;
        s_underruns++;
        memset(pcm, 0, want);
        return true;
    }

    uint8_t *dst = (uint8_t *)pcm;
    size_t got = 0;
    while (got < want) {
        size_t sz = 0;
        uint8_t *p = xRingbufferReceiveUpTo(s_rb, &sz, 0, want - got);
        if (p == NULL) {
            break;
        }
        memcpy(dst + got, p, sz);
        vRingbufferReturnItem(s_rb, p);
        got += sz;
    }
    if (got < want) {
        memset(dst + got, 0, want - got);
    }
    return true;
}

bool a2dp_bridge_streaming(void)
{
    return s_streaming;
}

bool a2dp_bridge_connected(void)
{
    return s_a2dp_connected;
}

uint32_t a2dp_bridge_packets(void)
{
    return s_in_pkts;
}

void a2dp_bridge_stats(uint32_t *underruns, uint32_t *drops, uint32_t *fill_ms)
{
    *underruns = s_underruns;
    *drops = s_drops;
    *fill_ms = s_rb ? (RB_BYTES - xRingbufferGetCurFreeSize(s_rb)) / MS_BYTES(1) : 0;
}

/* ---- profiles ------------------------------------------------------------ */

static void a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *p)
{
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        static const char *st[] = {"Disconnected", "Connecting", "Connected", "Disconnecting"};
        uint8_t *a = p->conn_stat.remote_bda;
        ESP_LOGW(TAG, "A2DP %s [%02x:%02x:%02x:%02x:%02x:%02x]%s", st[p->conn_stat.state],
                 a[0], a[1], a[2], a[3], a[4], a[5],
                 p->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED
                     ? (p->conn_stat.disc_rsn == ESP_A2D_DISC_RSN_NORMAL ? " (normal)" : " (ABNORMAL - link lost)")
                     : "");
        bool is_current = s_have_peer && memcmp(a, s_peer, 6) == 0;

        /* One audio source at a time, like a real speaker: a second device that
         * shows up while a phone is attached gets dropped straight away. */
        if (s_have_peer && !is_current &&
            (p->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTING ||
             p->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED)) {
            ESP_LOGW(TAG, "  second audio source - refusing it (one phone at a time)");
            esp_a2d_sink_disconnect(a);
            break;
        }
        if (s_have_peer && !is_current) {
            break;                       /* events from the refused device */
        }

        if (p->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            memcpy(s_peer, a, 6);
            s_have_peer = true;
            s_a2dp_connected = true;
            remember_phone(a);
            esp_bt_gap_read_remote_name(a);   /* phone name -> CRC in the JBL beacon */
            esp_bt_gap_set_acl_pkt_types(a, ACL_PKT_ALL_EDR);   /* let the phone use 2/3-DH5 */
            /* stop inquiry/page scanning while a phone is attached: frees radio time */
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
        } else if (p->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            bool lost = p->conn_stat.disc_rsn != ESP_A2D_DISC_RSN_NORMAL;
            s_have_peer = false;
            s_a2dp_connected = false;
            s_streaming = false;
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            if (lost) {
                s_retry_after_loss = true;   /* reconnect_task calls the phone back */
            }
        }
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT:
        s_streaming = (p->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED);
        s_rs_reset = true;
        ESP_LOGW(TAG, "A2DP audio %s", s_streaming ? "STARTED -> re-broadcasting on Auracast"
                                                   : "suspended -> broadcasting silence");
        break;
    case ESP_A2D_AUDIO_CFG_EVT: {
        const esp_a2d_mcc_t *m = &p->audio_cfg.mcc;
        uint32_t r = 44100;
        if (m->type == ESP_A2D_MCT_SBC) {
            if (m->cie.sbc_info.samp_freq & ESP_A2D_SBC_CIE_SF_48K)      r = 48000;
            else if (m->cie.sbc_info.samp_freq & ESP_A2D_SBC_CIE_SF_44K) r = 44100;
            else if (m->cie.sbc_info.samp_freq & ESP_A2D_SBC_CIE_SF_32K) r = 32000;
            else                                                        r = 16000;
        }
        s_src_rate = r;
        ESP_LOGW(TAG, "A2DP codec configured: type %d, %lu Hz -> resampled to 48000 Hz",
                 m->type, (unsigned long)r);
        break;
    }
    default:
        break;
    }
}

static void gap_bt_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *p)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        ESP_LOGW(TAG, "Paired with '%s' (status %d)", p->auth_cmpl.device_name, p->auth_cmpl.stat);
        break;
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "SSP confirm %06lu -> accept", (unsigned long)p->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(p->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_READ_REMOTE_NAME_EVT:
        if (p->read_rmt_name.stat == ESP_BT_STATUS_SUCCESS && s_name_cb) {
            s_name_cb((const char *)p->read_rmt_name.rmt_name);
        }
        break;
    case ESP_BT_GAP_ACL_PKT_TYPE_CHANGED_EVT:
        ESP_LOGW(TAG, "ACL packet types now 0x%04x (status %d)%s", p->set_acl_pkt_types.pkt_types,
                 p->set_acl_pkt_types.status,
                 (p->set_acl_pkt_types.pkt_types & 0x3306) == 0x3306 ? " - NO EDR packets!" : " - EDR allowed");
        break;
    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        if (p->acl_conn_cmpl_stat.stat == ESP_BT_STATUS_SUCCESS) {
            s_acl_up = true;
            s_acl_change_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }
        break;
    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        s_acl_up = false;
        s_acl_change_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        break;
    case ESP_BT_GAP_PIN_REQ_EVT: {
        esp_bt_pin_code_t pin = {'0', '0', '0', '0'};
        esp_bt_gap_pin_reply(p->pin_req.bda, true, 4, pin);
        break;
    }
    default:
        break;
    }
}

/* The last phone that streamed to us, kept in NVS. */
static void remember_phone(const uint8_t *a)
{
    nvs_handle_t h;
    if (nvs_open("jbl", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, "last_phone", a, 6);
        nvs_commit(h);
        nvs_close(h);
    }
}

static bool recall_phone(uint8_t out[6])
{
    nvs_handle_t h;
    size_t len = 6;
    bool ok = false;
    if (nvs_open("jbl", NVS_READONLY, &h) == ESP_OK) {
        ok = nvs_get_blob(h, "last_phone", out, &len) == ESP_OK && len == 6;
        nvs_close(h);
    }
    return ok;
}

/* Like a real speaker: on power-up (and after a lost link) call back the LAST
 * phone only - never cycle through every device ever paired. Kept short and
 * polite: never page while any Classic link is up (the phone may be connecting
 * to us at that very moment - two pages collide and both fail), short page
 * timeout, and long gaps so our page scan stays open for the phone. */
static void call_back_last_phone(const char *why, int tries)
{
    uint8_t a[6];
    if (!recall_phone(a)) {
        ESP_LOGI(TAG, "no previous phone - connect to 'S31 JBL Xtreme 5' from the phone's Bluetooth settings");
        return;
    }
    for (int attempt = 0; attempt < tries && !s_a2dp_connected; attempt++) {
        /* wait until no link is up and nothing changed for 1.5 s */
        for (int w = 0; w < 100; w++) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (s_a2dp_connected) {
                return;
            }
            if (!s_acl_up && now - s_acl_change_ms > 1500) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (s_a2dp_connected || s_acl_up) {
            return;                          /* the phone is (re)connecting on its own */
        }
        ESP_LOGW(TAG, "%s: calling back last phone %02x:%02x:%02x:%02x:%02x:%02x (try %d/%d)",
                 why, a[0], a[1], a[2], a[3], a[4], a[5], attempt + 1, tries);
        esp_a2d_sink_connect(a);
        for (int w = 0; w < 50 && !s_a2dp_connected; w++) {   /* page 2.5 s + setup */
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        for (int w = 0; w < 60 && !s_a2dp_connected; w++) {   /* then 6 s of page-scan only */
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

static void reconnect_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1500));
    call_back_last_phone("power-up", 2);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (s_retry_after_loss && !s_a2dp_connected) {
            s_retry_after_loss = false;
            call_back_last_phone("link lost", 3);
        }
    }
}

/* ---- bring-up ------------------------------------------------------------ */

esp_err_t bt_dual_mode_init(void)
{
    esp_err_t ret;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "controller init failed %d", ret);
        return ret;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret) {
        ESP_LOGE(TAG, "controller enable (dual mode) failed %d", ret);
        return ret;
    }
    esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&cfg);
    if (ret) {
        return ret;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        return ret;
    }

    /* LE security as in the BLE-audio examples (Just Works, SC + bond). */
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(rsp_key));
    return ESP_OK;
}

esp_err_t a2dp_bridge_start(const char *bredr_name)
{
    rs_init_kernel();
    /* Phone-audio buffer lives in PSRAM when there is one (frees internal SRAM
     * for Bluetooth and the LC3 encoder); falls back to internal RAM. */
    s_rb = xRingbufferCreateWithCaps(RB_BYTES, RINGBUF_TYPE_BYTEBUF, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_rb == NULL) {
        s_rb = xRingbufferCreate(RB_BYTES, RINGBUF_TYPE_BYTEBUF);
    }
    if (s_rb == NULL) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGW(TAG, "audio buffer %u KB in %s; free internal %u KB, PSRAM %u KB", RB_BYTES / 1024,
             esp_ptr_external_ram(s_rb) ? "PSRAM" : "internal RAM",
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));

    esp_bt_gap_register_callback(gap_bt_cb);
    esp_bt_sp_param_t sp = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t io = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(sp, &io, sizeof(io));

    esp_bt_gap_set_device_name(bredr_name);

    /* Class of Device: Audio/Video - Loudspeaker, services Audio + Rendering. */
    esp_bt_cod_t cod = {0};
    cod.major = ESP_BT_COD_MAJOR_DEV_AV;
    cod.minor = 0x05;                                  /* Loudspeaker */
    cod.service = ESP_BT_COD_SRVC_AUDIO | ESP_BT_COD_SRVC_RENDERING;
    esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD);

    esp_a2d_register_callback(a2d_cb);
    ESP_ERROR_CHECK(esp_a2d_sink_init());
    esp_a2d_sink_register_data_callback(a2d_data_cb);

    esp_a2d_mcc_t mcc = {0};
    mcc.type = ESP_A2D_MCT_SBC;
    mcc.cie.sbc_info.samp_freq = ESP_A2D_SBC_CIE_SF_32K | ESP_A2D_SBC_CIE_SF_44K | ESP_A2D_SBC_CIE_SF_48K;
    mcc.cie.sbc_info.ch_mode = ESP_A2D_SBC_CIE_CH_MODE_MONO | ESP_A2D_SBC_CIE_CH_MODE_DUAL_CHANNEL |
                               ESP_A2D_SBC_CIE_CH_MODE_STEREO | ESP_A2D_SBC_CIE_CH_MODE_JOINT_STEREO;
    mcc.cie.sbc_info.block_len = ESP_A2D_SBC_CIE_BLOCK_LEN_4 | ESP_A2D_SBC_CIE_BLOCK_LEN_8 |
                                 ESP_A2D_SBC_CIE_BLOCK_LEN_12 | ESP_A2D_SBC_CIE_BLOCK_LEN_16;
    mcc.cie.sbc_info.num_subbands = ESP_A2D_SBC_CIE_NUM_SUBBANDS_4 | ESP_A2D_SBC_CIE_NUM_SUBBANDS_8;
    mcc.cie.sbc_info.alloc_mthd = ESP_A2D_SBC_CIE_ALLOC_MTHD_SRN | ESP_A2D_SBC_CIE_ALLOC_MTHD_LOUDNESS;
    mcc.cie.sbc_info.min_bitpool = 53;  /* pin SBC High Quality: the phone may not drop below 53 */
    mcc.cie.sbc_info.max_bitpool = 53;   /* full SBC High Quality, as before */
    esp_a2d_sink_register_stream_endpoint(0, &mcc);

    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    /* 2.56 s page timeout (default 5.12 s): a call-back that gets no answer
     * gives the radio back to page scan / the BIG sooner. */
    esp_bt_gap_set_page_timeout(0x1000);

    const uint8_t *lf = BTM_ReadLocalFeatures();
    if (lf) {
        ESP_LOGW(TAG, "local LMP features %02x %02x %02x %02x %02x %02x %02x %02x -> EDR 2M %s, 3M %s, 3-slot EDR %s, 5-slot EDR %s",
                 lf[0], lf[1], lf[2], lf[3], lf[4], lf[5], lf[6], lf[7],
                 (lf[3] & 0x02) ? "yes" : "NO", (lf[3] & 0x04) ? "yes" : "NO",
                 (lf[4] & 0x80) ? "yes" : "NO", (lf[5] & 0x01) ? "yes" : "NO");
    }
    const uint8_t *a = esp_bt_dev_get_address();
    ESP_LOGW(TAG, "BR/EDR speaker '%s' discoverable at %02x:%02x:%02x:%02x:%02x:%02x (A2DP sink)",
             bredr_name, a[0], a[1], a[2], a[3], a[4], a[5]);
    xTaskCreatePinnedToCore(reconnect_task, "a2dp_reconn", 3072, NULL, 4, NULL, 1);   /* core 1: keep core 0 for the BT controller */
    return ESP_OK;
}
