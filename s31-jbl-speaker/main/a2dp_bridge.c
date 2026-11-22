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
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "nvs.h"
#include "esp_memory_utils.h"

#include "a2dp_bridge.h"

#define TAG "JBL_BREDR"

#define OUT_RATE        48000
#define MS_BYTES(ms)    (OUT_RATE * 4 / 1000 * (ms))
#define RB_BYTES        MS_BYTES(400)           /* 400 ms of 48 kHz stereo s16 */
#define PRIME_BYTES     MS_BYTES(50)            /* start reading after 50 ms (low latency, S31 is master) */
#define TARGET_BYTES    MS_BYTES(80)            /* steady-state fill the drift control aims for */
/* Latency cap. The +-0.2 % drift control needs minutes to work off a big
 * excess (250 ms at 0.2 % = 125 s), so after a burst (phone catching up after
 * a radio stall) the reader drops the OLDEST audio once, straight back to
 * TARGET: one short skip instead of minutes of extra delay, and the writer
 * (drop-newest, whole packet, only when the ring is completely full) almost
 * never has to fire. 300 ms leaves 150 ms over target for normal jitter
 * (measured fill 137-187 ms while streaming). */
#define HIGH_BYTES      MS_BYTES(160)

static volatile uint32_t s_underruns;           /* encoder found too little phone audio */
static volatile uint32_t s_drops;               /* phone audio dropped (buffer full) */
static volatile uint32_t s_skips;               /* latency cap hit: oldest audio dropped back to TARGET */
/* Set by the A2DP callbacks (BTC task) on start / suspend / codec change /
 * disconnect; the encoder task (the ring's consumer, the only side allowed to
 * move rd) empties the ring on its next read, so audio from before a pause or
 * from the previous phone is never played. */
static volatile bool s_flush_req;

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

/* Controller radio-priority profiles (A2DP vs Auracast): see bt_sched_tune.c
 * and re/controller_sched.md. */
#include "bt_sched_tune.h"

/* Bluedroid internals: desired link role for new connections, current role. */
extern uint8_t L2CA_SetDesireRole(uint8_t new_role);
extern uint8_t BTM_GetRole(uint8_t *remote_bd_addr, uint8_t *p_role);
#define S31_HCI_ROLE_MASTER        0x00
#define S31_L2CAP_ROLE_NO_SWITCH   0x40     /* disallow role switch at create-connection */
#define S31_A2DP_T_POLL            10       /* master poll interval, slots (6.25 ms) */

/* A/B test: while music streams, rotate radio profiles every 20 s and log
 * which one is active, so one listening session compares them all. */
#ifndef SCHED_AB_TEST
#define SCHED_AB_TEST 0
#endif
#define ACL_PKT_DH3_ONLY (ESP_BT_ACL_PKT_TYPES_MASK_DM1 | ESP_BT_ACL_PKT_TYPES_MASK_DH1 |                           ESP_BT_ACL_PKT_TYPES_MASK_DM3 | ESP_BT_ACL_PKT_TYPES_MASK_DH3 |                           ESP_BT_ACL_PKT_TYPES_MASK_NO_2_DH5 | ESP_BT_ACL_PKT_TYPES_MASK_NO_3_DH5)

/* All ACL packet types, EDR (2/3 Mbps) DH1/DH3/DH5 allowed (no "NO_x_DHy" bits). */
#define ACL_PKT_ALL_EDR (ESP_BT_ACL_PKT_TYPES_MASK_DM1 | ESP_BT_ACL_PKT_TYPES_MASK_DH1 |                          ESP_BT_ACL_PKT_TYPES_MASK_DM3 | ESP_BT_ACL_PKT_TYPES_MASK_DH3 |                          ESP_BT_ACL_PKT_TYPES_MASK_DM5 | ESP_BT_ACL_PKT_TYPES_MASK_DH5)
static void (*s_name_cb)(const char *name);

void a2dp_bridge_on_phone_name(void (*cb)(const char *name))
{
    s_name_cb = cb;
}

/* ---- 44.1/48 kHz -> 48 kHz resampler + phone-audio ring ---------------------
 * The DSP itself (windowed-sinc polyphase resampler, SPSC ring) is in
 * a2dp_dsp.h so tools/rs_host_test.c can test the same code on the host.
 *
 * Cost (core 1, which also runs Bluedroid and the LC3 encoder): with
 * A2D_SBC_DECODE_BATCH (idf-patches/0006) the SBC decoder calls a2d_data_cb
 * once per media packet (~5-8 SBC frames) instead of once per SBC frame, so
 * the drift computation and the ring publish happen ~50-70 times/s instead of
 * ~345. The ring is a lock-free SPSC ring (no spinlock / critical section),
 * and the FIR is single precision, fully unrolled, with no libm calls. */
#include "a2dp_dsp.h"

#define RB_FRAMES       (RB_BYTES / 4)
#define PRIME_FRAMES    (PRIME_BYTES / 4)
#define TARGET_FRAMES   (TARGET_BYTES / 4)
#define HIGH_FRAMES     (HIGH_BYTES / 4)

static rs_state_t s_rs;                        /* touched only by the A2DP decoder task */
static uint32_t s_rs_out[RS_OUT_MAX];          /* one FIR pass of output, internal RAM */
static spsc_ring_t s_ring;                     /* data in PSRAM, positions here */
static volatile bool s_rs_reset = true;

/* ---- audio path --------------------------------------------------------- */

static void a2d_data_cb(const uint8_t *data, uint32_t len)
{
    const int16_t *in = (const int16_t *)data;
    int n = len / 4;                       /* stereo frames */
    if (n <= 0 || s_ring.buf == NULL) {
        return;
    }
    s_in_pkts++;

    if (s_rs_reset) {
        s_rs_reset = false;
        rs_reset(&s_rs);
    }

    /* Drift control, once per packet: nudge the resampling ratio (max
     * +-0.2 %) so the buffer hovers around TARGET; the phone's clock and
     * ours differ. */
    const uint32_t src_rate = s_src_rate;
    uint32_t space = ring_space(&s_ring);
    uint32_t fill = RB_FRAMES - space;
    const float e = ((float)fill - (float)TARGET_FRAMES) * (1.0f / (float)TARGET_FRAMES);
    const uint64_t step = rs_step_q32(src_rate, OUT_RATE, e);

    /* Drop the whole packet if the encoder has fallen behind (keeps latency
     * bounded), as the byte ring did when a send did not fit. Bound: n input
     * frames give at most n / step + 1 outputs (+1 after a reset). */
    /* n / step without a 64-bit software divide: out/in rate ratio times
     * (1 - 0.002 e) is the first-order inverse of the drift factor (error
     * < 5e-6, far below the +3 margin). */
    const float inv_step = ((float)OUT_RATE / (float)src_rate) * (1.0f - 0.002f * e);
    uint32_t need = (uint32_t)((float)n * inv_step * 1.0001f) + 3;
    if (need > space) {
        s_drops++;
        return;
    }

    while (n > 0) {
        int m = rs_room(&s_rs);
        if (m > n) {
            m = n;
        }
        rs_push(&s_rs, in, m);
        in += 2 * m;
        n -= m;
        int o = rs_run(&s_rs, step, s_rs_out, RS_OUT_MAX);
        if (o > 0) {
            ring_write(&s_ring, s_rs_out, o);   /* fits: checked against `need` above */
        }
    }
}

bool a2dp_bridge_read(int16_t *pcm, int frames)
{
    size_t want = frames * 4;
    if (s_flush_req && s_ring.buf != NULL) {
        /* consumer-side flush: rd = wr. Anything the writer adds after this
         * (fresh audio, after its resampler reset) is kept. */
        s_flush_req = false;
        ring_skip(&s_ring, ring_fill(&s_ring));
        s_primed = false;
    }
    if (s_ring.buf == NULL || !s_streaming) {
        s_primed = false;
        if (s_a2dp_connected) {
            memset(pcm, 0, want);          /* phone connected but paused: silence, like a real speaker */
            return true;
        }
        return false;                      /* no phone at all: caller plays the test tone */
    }
    uint32_t buffered = ring_fill(&s_ring);
    if (!s_primed) {
        if (buffered < PRIME_FRAMES) {
            memset(pcm, 0, want);          /* streaming but still filling: silence */
            return true;
        }
        s_primed = true;
    }
    if (buffered > HIGH_FRAMES) {
        ring_skip(&s_ring, buffered - TARGET_FRAMES);   /* drop the oldest: latency back to TARGET */
        buffered = TARGET_FRAMES;
        s_skips++;
    }
    if (buffered < (uint32_t)frames) {
        /* Underrun mid-song: play silence and re-prime; never fall back to the tone. */
        s_primed = false;
        s_underruns++;
        memset(pcm, 0, want);
        return true;
    }
    ring_read(&s_ring, (uint32_t *)pcm, frames);
    return true;
}

void a2dp_bridge_discard(int frames)
{
    if (s_ring.buf == NULL || !s_streaming || !s_primed || frames <= 0) {
        return;                            /* nothing being played out yet */
    }
    uint32_t fill = ring_fill(&s_ring);
    ring_skip(&s_ring, fill < (uint32_t)frames ? fill : (uint32_t)frames);
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

void a2dp_bridge_stats(uint32_t *underruns, uint32_t *drops, uint32_t *skips, uint32_t *fill_ms)
{
    *underruns = s_underruns;
    *drops = s_drops;
    *skips = s_skips;
    *fill_ms = s_ring.buf ? ring_fill(&s_ring) * 4 / MS_BYTES(1) : 0;
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
            {
                uint8_t role = 0xFF;
                BTM_GetRole(a, &role);
                if (role == 0) {
                    /* As master we only hear the phone when we poll it: poll every
                     * 10 slots (6.25 ms) instead of the default 40 so A2DP gets its
                     * bitpool-53 throughput from the gaps between BIG events. */
                    esp_bt_gap_set_qos(a, S31_A2DP_T_POLL);
                }
                ESP_LOGW(TAG, "BR/EDR role on this link: %s", role == 0 ? "MASTER (S31 schedules the phone - good)"
                                                          : role == 1 ? "SLAVE (phone schedules blind - expect overlap)"
                                                                      : "unknown");
            }
            /* stop inquiry/page scanning while a phone is attached: frees radio time */
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
        } else if (p->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            bool lost = p->conn_stat.disc_rsn != ESP_A2D_DISC_RSN_NORMAL;
            s_have_peer = false;
            s_a2dp_connected = false;
            s_streaming = false;
            s_rs_reset = true;           /* next phone starts from a clean resampler ... */
            s_flush_req = true;          /* ... and an empty ring */
            bt_sched_tune_streaming(false);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            if (lost) {
                s_retry_after_loss = true;   /* reconnect_task calls the phone back */
            }
        }
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT:
        /* Start and suspend/stop alike: restart the resampler (its history
         * holds the end of the old audio) and empty the PCM ring, so a resume
         * re-primes on fresh audio only. Bluedroid already flushes its own
         * encoded-packet queue (rx_flush) on suspend/stop. Flags first, then
         * s_streaming, so the reader never sees "streaming" with stale data. */
        s_rs_reset = true;
        s_flush_req = true;
        __atomic_thread_fence(__ATOMIC_RELEASE);
        s_streaming = (p->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED);
        bt_sched_tune_streaming(s_streaming);
        if (s_have_peer) {
            uint8_t role = 0xFF;
            BTM_GetRole(s_peer, &role);
            ESP_LOGW(TAG, "BR/EDR role at audio %s: %s", s_streaming ? "start" : "stop",
                     role == 0 ? "MASTER" : role == 1 ? "SLAVE" : "unknown");
            if (role == 0 && s_streaming) {
                esp_bt_gap_set_qos(s_peer, S31_A2DP_T_POLL);
            }
        }
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
        s_rs_reset = true;               /* new source rate: old history/phase is meaningless */
        s_flush_req = true;
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
    case ESP_BT_GAP_QOS_CMPL_EVT:
        ESP_LOGW(TAG, "QoS: poll interval %lu slots (status %d)", (unsigned long)p->qos_cmpl.t_poll, p->qos_cmpl.stat);
        break;
    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        ESP_LOGW(TAG, "ACL disconnected, HCI reason 0x%02x%s", p->acl_disconn_cmpl_stat.reason,
                 p->acl_disconn_cmpl_stat.reason == 0x08 ? " (supervision timeout)" :
                 p->acl_disconn_cmpl_stat.reason == 0x13 ? " (remote user terminated)" :
                 p->acl_disconn_cmpl_stat.reason == 0x16 ? " (local host terminated)" :
                 p->acl_disconn_cmpl_stat.reason == 0x22 ? " (LMP response timeout)" :
                 p->acl_disconn_cmpl_stat.reason == 0x23 ? " (LMP transaction collision)" :
                 p->acl_disconn_cmpl_stat.reason == 0x21 ? " (role change not allowed)" : "");
        bt_sched_tune_streaming(false);
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

#if SCHED_AB_TEST
static void sched_ab_task(void *arg)
{
    /* Round 2: BIS first copies 15, repetitions 7, DH3 only; sweep the ACL
     * base. The controller adds +1 per missed 25 ms poll (and per 4 queued
     * packets), so a base just under 7 lets repetitions win normally and the
     * phone link overtake them only while it is falling behind. */
    static const struct { uint8_t acl, first, rep, pto; bool dh3; const char *name; } prof[] = {
        { 3, 15, 7, 7, true, "Q1 ACL3 (reps7), DH3" },
        { 4, 15, 7, 7, true, "Q2 ACL4 (reps7), DH3" },
        { 5, 15, 7, 7, true, "Q3 ACL5 (reps7), DH3" },
        { 6, 15, 7, 7, true, "Q4 ACL6 (reps7), DH3" },
    };
    int i = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(20000));
        if (!s_streaming || !s_have_peer) {
            continue;
        }
        i = (i + 1) & 3;
        ESP_LOGW(TAG, "===== A/B PROFILE %s =====", prof[i].name);
        bt_sched_tune_apply_custom(prof[i].acl, prof[i].first, prof[i].rep, prof[i].pto, prof[i].name);
        esp_bt_gap_set_acl_pkt_types(s_peer, prof[i].dh3 ? ACL_PKT_DH3_ONLY : ACL_PKT_ALL_EDR);
    }
}
#endif

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
    /* Be the BR/EDR master: accept incoming connections as master (role switch
     * at setup) and never offer a switch on our own outgoing pages. Together
     * with the link-policy patch (no role switch later) our controller decides
     * when the phone may transmit - around the Auracast BIG. */
    L2CA_SetDesireRole(S31_HCI_ROLE_MASTER);
    L2CA_SetDesireRole(S31_L2CAP_ROLE_NO_SWITCH);
    bt_sched_tune_init();
    return ESP_OK;
}

esp_err_t a2dp_bridge_start(const char *bredr_name)
{
    rs_init_kernel();
    /* Phone-audio buffer lives in PSRAM when there is one (frees internal SRAM
     * for Bluetooth and the LC3 encoder); falls back to internal RAM. */
    const uint32_t slots = RB_FRAMES + 1;      /* SPSC ring keeps one slot empty */
    uint32_t *rb = heap_caps_malloc(slots * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (rb == NULL) {
        rb = heap_caps_malloc(slots * 4, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (rb == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_ring.slots = slots;
    s_ring.wr = 0;
    s_ring.rd = 0;
    __atomic_store_n(&s_ring.buf, rb, __ATOMIC_RELEASE);   /* last: a2d_data_cb / read test buf */
    ESP_LOGW(TAG, "audio buffer %u KB in %s; free internal %u KB, PSRAM %u KB", RB_BYTES / 1024,
             esp_ptr_external_ram(rb) ? "PSRAM" : "internal RAM",
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
#if SCHED_AB_TEST
    xTaskCreatePinnedToCore(sched_ab_task, "sched_ab", 3072, NULL, 3, NULL, 1);
#endif
    return ESP_OK;
}
