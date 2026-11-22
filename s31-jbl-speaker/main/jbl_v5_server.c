/*
 * JBL "V5" protocol server, so the JBL One app can drive the S31 like a real
 * JBL Xtreme 5 (device page, EQ, settings). Decoded from JBL One 2.9.4; see
 * apk/notes_v5_server.md.
 *
 * Transport: GATT service excelpoint.com (65786365-...-2e636f6d0000), served
 * over BR/EDR (the app uses connectGatt(transport=BR/EDR)) and LE.
 *   ...0002  app -> speaker (write without response)
 *   ...0001  speaker -> app (notify)
 * Frame: u16 0xDD00 | u16 cmd (1 Get, 2 Set, 3 Notify, 4 heartbeat-ack)
 *        | u8 pktCount | u8 pktIdx | u16 payloadLen | payload
 * Get payload:   u16 featId list
 * Set payload:   {u16 featId, u16 len, value}*
 * Reply/notify:  u16 status (0 = OK), then {u16 featId, u16 len, value}*
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "esp_bt_device.h"
#include "esp_gatts_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"

#include "jbl_v5_server.h"
#include "app_task.h"

#define TAG "JBL_V5"

/* ---- speaker state the app can read / change ------------------------------ */

static struct {
    uint8_t battery;          /* 0x000D: b7 charging, b0-6 % */
    uint8_t feedback_tone;    /* 0x0C40: b7 on */
    uint16_t auto_off;        /* 0x0012: b15 on, b0-14 seconds */
    uint8_t volume;           /* 0x2D42 */
    uint8_t mute;             /* 0x2D41 */
    uint8_t bass_boost;       /* 0x2009 */
    uint8_t playtime_boost;   /* 0x2006 */
    uint8_t smart_eq;         /* 0x0E80 */
    uint8_t auracast_mode;    /* 0x2004 */
    uint8_t eq[8];            /* 0x0E7E: category + 7 gain indices */
    uint8_t light_status;     /* 0x0D00 */
    uint8_t light_show;       /* 0x0D40 */
    uint8_t light_bright;     /* 0x0D41 */
    uint8_t rgb_loop[4];      /* 0x0D80 */
} st = {
    .light_status = 0x80,
    .light_show = 0x81,
    .light_bright = 0xBC,
    .rgb_loop = { 0x01, 0x00, 0x00, 0x00 },
    .battery = 0x50,
    .feedback_tone = 0x80,
    .auto_off = 0x8000 | 1800,
    .volume = 0x32,
    .mute = 0,
    .bass_boost = 0x81,
    .playtime_boost = 0,
    .smart_eq = 0,
    .auracast_mode = 1,
    .eq = { 0x80, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C },   /* JBL Signature */
};

static const char *s_name = "S31 JBL Xtreme 5";
static const char *s_serial = "S31X5000001";
static const uint8_t s_fw[3] = { 0x1a, 0x1d, 0x0c };

/* ---- link ------------------------------------------------------------------ */

static esp_gatt_if_t s_gif = ESP_GATT_IF_NONE;
static uint16_t s_conn_id = 0xFFFF;
static uint16_t s_notify_handle;
static volatile bool s_linked;

void v5_server_set_notify_handle(uint16_t h)
{
    s_notify_handle = h;
}

static volatile uint16_t s_att_mtu = 0;      /* 0 = unknown (BR/EDR links often report none) */

void v5_server_set_mtu(uint16_t mtu)
{
    s_att_mtu = mtu;
}

/* ATT bearer congestion (ESP_GATTS_CONGEST_EVT). Notifications are never
 * refused by Bluedroid: while congested they pile up in the L2CAP queue in
 * internal heap. Replies to the app's Get/Set are few and bounded (<= 3
 * packets per request) so they still go; the periodic heartbeat is optional
 * and is skipped while congested. */
static volatile bool s_congested;
static volatile uint32_t s_hb_skipped;

void v5_server_set_congested(uint16_t conn_id, bool congested)
{
    if (conn_id == s_conn_id && congested != s_congested) {
        s_congested = congested;
        ESP_LOGW(TAG, "JBL One link %s", congested ? "congested - holding heartbeats" : "uncongested");
    }
}

/* send_frame runs in the BTC task (replies) and in v5_hb (heartbeat): one
 * static frame buffer behind a mutex instead of 408 B on each stack (the BTC
 * task has only CONFIG_BT_BTC_TASK_STACK_SIZE = 3 KB). */
static SemaphoreHandle_t s_tx_lock;
static uint8_t s_frame[8 + 400];

/* One V5 message; split into pktCount packets when it doesn't fit one ATT
 * notification (the app reassembles until pktIdx == pktCount-1). */
static void send_frame(uint16_t cmd, const uint8_t *payload, uint16_t len)
{
    uint8_t *f = s_frame;
    if (!s_linked || s_notify_handle == 0 || len > 400) {
        if (len > 400) ESP_LOGE(TAG, "V5 reply too long (%u B) - dropped", len);
        return;
    }
    if (s_tx_lock == NULL || xSemaphoreTake(s_tx_lock, pdMS_TO_TICKS(200)) != pdTRUE) {
        ESP_LOGE(TAG, "V5 frame 0x%04x dropped (tx busy)", cmd);
        return;
    }
    uint16_t room = s_att_mtu > 3 + 8 + 16 ? s_att_mtu - 3 - 8 : 160;   /* payload bytes per packet */
    uint8_t count = (len + room - 1) / room;
    if (count == 0) count = 1;
    for (uint8_t idx = 0; idx < count; idx++) {
        uint16_t o = idx * room;
        uint16_t n = len - o < room ? len - o : room;
        f[0] = 0x00; f[1] = 0xDD;
        f[2] = cmd & 0xFF; f[3] = cmd >> 8;
        f[4] = count; f[5] = idx;
        f[6] = n & 0xFF; f[7] = n >> 8;
        memcpy(f + 8, payload + o, n);
        /* the API copies f into its BTC message before returning */
        esp_err_t e = esp_ble_gatts_send_indicate(s_gif, s_conn_id, s_notify_handle, 8 + n, f, false);
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "V5 notify failed (%d), %u B", e, 8 + n);
            break;                                /* rest of a split frame is useless now */
        }
    }
    xSemaphoreGive(s_tx_lock);
}

/* append {id,len,value} */
static uint16_t put_tlv(uint8_t *p, uint16_t off, uint16_t id, const void *v, uint16_t n)
{
    p[off] = id & 0xFF; p[off + 1] = id >> 8;
    p[off + 2] = n & 0xFF; p[off + 3] = n >> 8;
    if (n) memcpy(p + off + 4, v, n);
    return off + 4 + n;
}

/* Value of one feature; returns false if we don't know it. */
static bool feat_value(uint16_t id, uint8_t *v, uint16_t *n)
{
    const uint8_t *mac;
    uint32_t u32;
    switch (id) {
    case 0x0004: v[0] = 0x69; v[1] = 0x21; *n = 2; return true;               /* ProductID 2169 */
    case 0x0005: v[0] = 0x00; *n = 1; return true;                            /* ColorID */
    case 0x0006: *n = strlen(s_name); memcpy(v, s_name, *n); return true;     /* DeviceName */
    case 0x0008: *n = strlen(s_serial); memcpy(v, s_serial, *n); return true; /* SerialNumber */
    case 0x000A: mac = esp_bt_dev_get_address(); memcpy(v, mac, 6); *n = 6; return true;
    case 0x000C: memcpy(v, s_fw, 3); *n = 3; return true;                     /* FirmwareVersion */
    case 0x000D: v[0] = st.battery; *n = 1; return true;
    case 0x0012: v[0] = st.auto_off & 0xFF; v[1] = st.auto_off >> 8; *n = 2; return true;
    case 0x0013: v[0] = st.auto_off & 0xFF; v[1] = st.auto_off >> 8; *n = 2; return true; /* AutoStandby */
    case 0x0045: v[0] = 0x01; *n = 1; return true;                            /* PowerStatus on */
    case 0x0056: v[0] = 0x00; *n = 1; return true;                            /* RemotePowerEnableStatus */
    case 0x0D45: v[0] = 0x00; *n = 1; return true;                            /* BassBoostLight */
    case 0x0D62: v[0] = 0x00; *n = 1; return true;                            /* ChargingLight */
    case 0x0C40: v[0] = st.feedback_tone; *n = 1; return true;
    case 0x1008: u32 = 600; memcpy(v, &u32, 4); *n = 4; return true;          /* TotalPlaybackTime */
    case 0x0B80: *n = 0; return true;                                         /* Auracast assistant: idle */
    case 0x0B86: v[0] = 0x01; *n = 1; return true;                            /* BroadcasterEnable: we host */
    case 0x0B87: v[0] = 0x01; *n = 1; return true;                            /* BroadcasterStart: on air */
    case 0x0BA0: v[0] = 0x00; *n = 1; return true;
    case 0x0BA1: v[0] = 0x01; *n = 1; return true;                            /* ActiveAudioSource BT */
    case 0x0E7E: memcpy(v, st.eq, 8); *n = 8; return true;
    case 0x0E80: v[0] = st.smart_eq; *n = 1; return true;
    case 0x2004: v[0] = st.auracast_mode; *n = 1; return true;
    case 0x2005: v[0] = 0x00; *n = 1; return true;
    case 0x2006: v[0] = st.playtime_boost; *n = 1; return true;
    case 0x2009: v[0] = st.bass_boost; *n = 1; return true;
    case 0x2C00: v[0] = 0x80; *n = 1; return true;                            /* Auracast party */
    case 0x2C01: v[0] = 0x01; *n = 1; return true;                            /* group: party */
    case 0x2C02: v[0] = 0x02; *n = 1; return true;                            /* connected */
    case 0x2C03: v[0] = 0x02; *n = 1; return true;                            /* role: primary */
    case 0x2C04: v[0] = 0x00; *n = 1; return true;
    case 0x2C06: memset(v, 0, 4); *n = 4; return true;
    case 0x2C80: v[0] = 0x00; *n = 1; return true;
    case 0x2C81: memset(v, 0, 6); *n = 6; return true;
    case 0x2D40: v[0] = 0x00; *n = 1; return true;                            /* AudioOperation */
    case 0x2D41: v[0] = st.mute; *n = 1; return true;
    case 0x2D42: v[0] = st.volume; *n = 1; return true;
    case 0x2D43: *n = 0; return true;                                         /* SongName */
    case 0x2D44: *n = 0; return true;                                         /* ArtistName */
    case 0x2D47: v[0] = 0x00; *n = 1; return true;
    case 0x2D49: v[0] = 0x00; *n = 1; return true;                            /* AudioPlayingStatus */
    /* lights: values the real Xtreme 5 reports */
    case 0x0D00: v[0] = st.light_status; *n = 1; return true;                 /* LightingStatus */
    case 0x0D40: v[0] = st.light_show; *n = 1; return true;                   /* LightShow */
    case 0x0D41: v[0] = st.light_bright; *n = 1; return true;                 /* LightBrightness */
    case 0x0D80: memcpy(v, st.rgb_loop, 4); *n = 4; return true;              /* RGBColorLooping */
    case 0x0C00: v[0] = 0x00; *n = 1; return true;                            /* MicConnectionStatus */
    case 0x200B: v[0] = 0x00; *n = 1; return true;                            /* AIVocalRemoval */
    case 0x2007: v[0] = 0x00; *n = 1; return true;                            /* MoodSound */
    case 0x30C2: memset(v, 0, 4); *n = 4; return true;                        /* PitchShifterValue */
    default: return false;
    }
}

/* ---- EQ: 7 biquads per channel, JBL's own coefficients ---------------------- */

#define EQ_BANDS 7
typedef struct { float b0, b1, b2, na1, na2; } biquad_t;
static biquad_t s_eq_new[EQ_BANDS];
static biquad_t s_eq[EQ_BANDS];
static float s_z[2][EQ_BANDS][4];         /* x1 x2 y1 y2 per channel/band */
static volatile bool s_eq_pending, s_eq_on;

static void eq_from_q29(const uint8_t *c)
{
    for (int b = 0; b < EQ_BANDS; b++) {
        int32_t q[5];
        memcpy(q, c + 20 * b, 20);
        const float k = 1.0f / (float)(1 << 29);
        s_eq_new[b].b0 = q[0] * k;
        s_eq_new[b].b1 = q[1] * k;
        s_eq_new[b].b2 = q[2] * k;
        s_eq_new[b].na1 = q[3] * k;
        s_eq_new[b].na2 = q[4] * k;
    }
    /* flat if every band is b0=1, others 0 */
    bool flat = true;
    for (int b = 0; b < EQ_BANDS; b++) {
        if (fabsf(s_eq_new[b].b0 - 1.0f) > 1e-4f || fabsf(s_eq_new[b].b1) > 1e-4f ||
            fabsf(s_eq_new[b].b2) > 1e-4f || fabsf(s_eq_new[b].na1) > 1e-4f || fabsf(s_eq_new[b].na2) > 1e-4f) {
            flat = false;
        }
    }
    s_eq_on = !flat;
    s_eq_pending = true;
}

void v5_eq_process(int16_t *pcm, int frames)
{
    if (s_eq_pending) {
        memcpy(s_eq, s_eq_new, sizeof(s_eq));
        memset(s_z, 0, sizeof(s_z));
        s_eq_pending = false;
    }
    if (!s_eq_on) {
        return;
    }
    for (int n = 0; n < frames; n++) {
        for (int ch = 0; ch < 2; ch++) {
            float x = pcm[2 * n + ch] * 0.7071f;      /* -3 dB headroom for boosts */
            for (int b = 0; b < EQ_BANDS; b++) {
                float *z = s_z[ch][b];
                const biquad_t *q = &s_eq[b];
                float y = q->b0 * x + q->b1 * z[0] + q->b2 * z[1] + q->na1 * z[2] + q->na2 * z[3];
                z[1] = z[0]; z[0] = x;
                z[3] = z[2]; z[2] = y;
                x = y;
            }
            x *= 1.4142f;
            if (x > 32767.f) x = 32767.f;
            if (x < -32768.f) x = -32768.f;
            pcm[2 * n + ch] = (int16_t)x;
        }
    }
}

/* ---- commands ---------------------------------------------------------------- */

static void notify_feature(uint16_t id)
{
    uint8_t p[2 + 4 + 64], v[64];
    uint16_t n = 0;
    if (!feat_value(id, v, &n)) {
        return;
    }
    p[0] = 0; p[1] = 0;
    uint16_t off = put_tlv(p, 2, id, v, n);
    send_frame(3, p, off);
}

static void handle_get(const uint8_t *pl, uint16_t len)
{
    static uint8_t out[400];                      /* BTC task only: off its 3 KB stack */
    uint8_t v[64];
    uint16_t off = 2;
    uint8_t miss[64];
    uint16_t nmiss = 0;
    out[0] = 0; out[1] = 0;                       /* status OK */
    char dbg[200], mdbg[120]; int dl = 0, ml = 0;
    dbg[0] = mdbg[0] = 0;
    for (uint16_t i = 0; i + 1 < len; i += 2) {
        uint16_t id = pl[i] | (pl[i + 1] << 8);
        uint16_t n = 0;
        if (dl < (int)sizeof(dbg) - 6) dl += snprintf(dbg + dl, sizeof(dbg) - dl, "%04x ", id);
        if (feat_value(id, v, &n) && off + 4 + n <= 380) {
            off = put_tlv(out, off, id, v, n);
        } else if (nmiss + 2 <= sizeof(miss)) {
            miss[nmiss++] = id & 0xFF;
            miss[nmiss++] = id >> 8;
            if (ml < (int)sizeof(mdbg) - 6) ml += snprintf(mdbg + ml, sizeof(mdbg) - ml, "%04x ", id);
        }
    }
    ESP_LOGW(TAG, "app Get [%s] -> %u B%s%s", dbg, off, nmiss ? " | UNSUPPORTED: " : "", mdbg);
    send_frame(1, out, off);
    if (nmiss) {
        /* like the real Xtreme 5: a second Get reply carrying 0x0000
         * UnsupportedFeatureList with the IDs it doesn't have */
        uint8_t u[2 + 4 + sizeof(miss)];
        u[0] = 0; u[1] = 0;
        uint16_t uo = put_tlv(u, 2, 0x0000, miss, nmiss);
        send_frame(1, u, uo);
    }
}

static void handle_set(const uint8_t *pl, uint16_t len)
{
    const uint8_t ok[2] = { 0, 0 };
    bool eq_query = false, eq_changed = false;

    for (uint16_t i = 0; i + 4 <= len;) {
        uint16_t id = pl[i] | (pl[i + 1] << 8);
        uint16_t n = pl[i + 2] | (pl[i + 3] << 8);
        const uint8_t *v = pl + i + 4;
        if (i + 4 + n > len) {
            break;
        }
        switch (id) {
        case 0x0E01: eq_query = true; break;                                  /* EqInfoQuery */
        case 0x0E7F:                                                          /* EqSetPortable */
            if (n >= 1) {
                st.eq[0] = v[0];
                int coef_off = 1;
                if (v[0] == 0xC1 || v[0] == 0x00) {                           /* custom: 7 indices */
                    if (n >= 8) memcpy(st.eq + 1, v + 1, 7);
                    coef_off = 8;
                } else {
                    memset(st.eq + 1, 0x0C, 7);
                }
                if (n >= coef_off + 140) {
                    eq_from_q29(v + coef_off);
                }
                eq_changed = true;
                ESP_LOGW(TAG, "EQ set: category 0x%02x, bands %d %d %d %d %d %d %d (applied to audio)",
                         st.eq[0], st.eq[1], st.eq[2], st.eq[3], st.eq[4], st.eq[5], st.eq[6], st.eq[7]);
            }
            break;
        case 0x0E80: if (n) st.smart_eq = v[0]; break;
        case 0x0C40: if (n) st.feedback_tone = v[0]; break;
        case 0x0012: if (n >= 2) st.auto_off = v[0] | (v[1] << 8); break;
        case 0x2D42: if (n) st.volume = v[0]; break;
        case 0x2D41: if (n) st.mute = v[0]; break;
        case 0x2009: if (n) st.bass_boost = v[0]; break;
        case 0x2006: if (n) st.playtime_boost = v[0]; break;
        case 0x2004: if (n) st.auracast_mode = v[0]; break;
        case 0x0D00: if (n) st.light_status = v[0]; break;
        case 0x0D40: if (n) st.light_show = v[0]; break;
        case 0x0D41: if (n) st.light_bright = v[0]; break;
        case 0x0D80: if (n >= 4) memcpy(st.rgb_loop, v, 4); break;
        case 0x004A:
        case 0x004B:
            ESP_LOGW(TAG, "app sent anti-counterfeit challenge 0x%04x (%u B) - cannot sign it", id, n);
            break;
        default:
            break;
        }
        ESP_LOGW(TAG, "app Set 0x%04x (%u B)", id, n);
        i += 4 + n;
    }
    send_frame(2, ok, 2);                         /* every Set gets "00 00" */
    if (eq_query || eq_changed) {
        notify_feature(0x0E7E);                   /* current EQ back to the app */
    }
}

/* reassembly for multi-packet frames: cold (a few app commands per minute),
 * so it lives in PSRAM, allocated on the first split frame */
#define V5_RX_MAX 600
static uint8_t *s_rx;
static uint16_t s_rx_len;

void v5_server_on_write(const uint8_t *d, uint16_t len)
{
    if (len < 8 || d[0] != 0x00 || d[1] != 0xDD) {
        ESP_LOGI(TAG, "non-V5 write (%u B)", len);
        return;
    }
    uint16_t cmd = d[2] | (d[3] << 8);
    uint8_t cnt = d[4], idx = d[5];
    uint16_t plen = d[6] | (d[7] << 8);
    const uint8_t *pl = d + 8;
    uint16_t have = len - 8 < plen ? len - 8 : plen;

    if (cnt > 1) {                                /* split frame: collect */
        if (s_rx == NULL) {
            s_rx = heap_caps_malloc(V5_RX_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (s_rx == NULL) {
                s_rx = heap_caps_malloc(V5_RX_MAX, MALLOC_CAP_8BIT);
            }
            if (s_rx == NULL) {
                ESP_LOGE(TAG, "no RAM for V5 reassembly - split frame dropped");
                return;
            }
        }
        if (idx == 0) s_rx_len = 0;
        if (s_rx_len + have <= V5_RX_MAX) {
            memcpy(s_rx + s_rx_len, pl, have);
            s_rx_len += have;
        }
        if (idx + 1 < cnt) return;
        pl = s_rx;
        have = s_rx_len;
    }

    switch (cmd) {
    case 1: handle_get(pl, have); break;
    case 2: handle_set(pl, have); break;
    case 4: break;                                /* heartbeat ack from app */
    default: {
        char h[3 * 24 + 1]; int hl = 0;
        for (uint16_t k = 0; k < len && k < 24; k++) hl += snprintf(h + hl, sizeof(h) - hl, "%02x ", d[k]);
        ESP_LOGW(TAG, "app cmd 0x%04x (%u B): %s", cmd, have, h);
        break;
    }
    }
}

void v5_server_on_connect(esp_gatt_if_t gif, uint16_t conn_id)
{
    s_gif = gif;
    s_conn_id = conn_id;
    s_linked = true;
    ESP_LOGW(TAG, "JBL One control link up (conn %u)", conn_id);
}

void v5_server_on_disconnect(uint16_t conn_id)
{
    if (conn_id == s_conn_id) {
        s_linked = false;
        s_att_mtu = 0;
        s_congested = false;
        ESP_LOGW(TAG, "JBL One control link down");
    }
}

/* The real speaker sends this every few seconds. */
static void heartbeat_task(void *arg)
{
    const uint8_t hb[6] = { 0, 0, 0x02, 0x00, 0x00, 0x00 };
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (s_linked) {
            if (s_congested) {
                s_hb_skipped++;                   /* optional traffic: never queue it behind a jam */
                continue;
            }
            send_frame(3, hb, sizeof(hb));
        }
    }
}

void v5_server_start(const char *device_name)
{
    if (device_name) {
        s_name = device_name;
    }
    s_tx_lock = xSemaphoreCreateMutex();
    app_task_cold("v5_hb", heartbeat_task, 3072, 3);   /* core 1, PSRAM stack: only posts GATT notifications */
}
