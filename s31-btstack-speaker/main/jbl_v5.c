/*
 * JBL "V5" protocol server on BTstack (port of the Bluedroid build's
 * jbl_v5_server.c; protocol decoded from JBL One 2.9.4, apk/notes_v5_server.md).
 *
 * Transport: GATT excelpoint.com (65786365-...-2e636f6d0000), over BR/EDR
 * (the app uses connectGatt(transport=BR/EDR)) or LE:
 *   ...0002  app -> speaker (write / write without response / long write)
 *   ...0001  speaker -> app (notify)
 * Everything here runs on the BTstack thread: no locks. Replies go to the
 * bearer the request came on; the heartbeat goes to the current app link.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "btstack.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "nvs.h"

#include "jbl.h"
#include "jbl_gatt.h"
#include "jbl_v5.h"
#include "v5_eq.h"

static const char *TAG = "JBL_V5";

#define V5_MAX_PAYLOAD   400        /* longest reply we build */
#define V5_RX_MAX        600        /* longest split frame we reassemble */
#define V5_HEARTBEAT_MS  5000       /* the real speaker sends one every few seconds */
#define V5_PERSIST_EQ    1          /* keep the app's EQ across reboots (NVS) */
#define V5_PERSIST_MS    5000       /* debounce: write once the slider settles */

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
    .light_status = 0x80,
    .light_show = 0x81,
    .light_bright = 0xBC,
    .rgb_loop = { 0x01, 0x00, 0x00, 0x00 },
};

static const char s_name[] = JBL_DEVICE_NAME;
static const char s_serial[] = "S31X5000001";
static const uint8_t s_fw[3] = { 0x1a, 0x1d, 0x0c };
static bd_addr_t s_local_addr;

/* ---- link ------------------------------------------------------------------ */

static hci_con_handle_t s_link = HCI_CON_HANDLE_INVALID;
static bool s_linked;
static btstack_timer_source_t s_hb_timer;

static uint32_t s_rx_frames, s_rx_get, s_rx_set, s_rx_bad, s_rx_split;
static uint32_t s_tx_frames, s_tx_pkts, s_tx_drop, s_hb_sent, s_hb_skipped, s_eq_sets;

void jbl_v5_set_local_addr(const bd_addr_t addr)
{
    memcpy(s_local_addr, addr, sizeof(bd_addr_t));
}

/* One V5 message on `con`; split into pktCount packets when it doesn't fit one
 * notification (the app reassembles until pktIdx == pktCount - 1). */
static void send_frame(hci_con_handle_t con, uint16_t cmd, const uint8_t *payload, uint16_t len)
{
    static uint8_t f[8 + V5_MAX_PAYLOAD];
    if (con == HCI_CON_HANDLE_INVALID || len > V5_MAX_PAYLOAD) {
        if (len > V5_MAX_PAYLOAD) {
            ESP_LOGE(TAG, "V5 reply too long (%u B) - dropped", len);
        }
        s_tx_drop++;
        return;
    }
    uint16_t mtu = jbl_gatt_mtu(con);
    if (mtu < ATT_DEFAULT_MTU) {
        s_tx_drop++;                                  /* bearer gone */
        return;
    }
    uint16_t room = mtu - 3 - 8;                      /* payload bytes per packet */
    if (room > V5_MAX_PAYLOAD) {
        room = V5_MAX_PAYLOAD;
    }
    uint16_t count = len == 0 ? 1 : (uint16_t)((len + room - 1) / room);
    if (count > 255) {
        s_tx_drop++;
        return;
    }
    uint32_t need = (uint32_t)count * (8 + JBL_GATT_TX_OVERHEAD) + len;
    if (jbl_gatt_tx_busy() && need > jbl_gatt_tx_free()) {
        ESP_LOGE(TAG, "V5 frame 0x%04x dropped (notify queue full)", cmd);
        s_tx_drop++;
        return;
    }
    for (uint16_t idx = 0; idx < count; idx++) {
        uint16_t o = idx * room;
        uint16_t n = len - o < room ? len - o : room;
        f[0] = 0x00; f[1] = 0xDD;
        f[2] = cmd & 0xFF; f[3] = cmd >> 8;
        f[4] = (uint8_t)count; f[5] = (uint8_t)idx;
        f[6] = n & 0xFF; f[7] = n >> 8;
        memcpy(f + 8, payload + o, n);
        if (!jbl_gatt_notify(con, jbl_gatt_v5_notify_handle(), f, 8 + n)) {
            ESP_LOGE(TAG, "V5 notify failed, %u B", 8 + n);
            s_tx_drop++;
            return;                                   /* rest of a split frame is useless now */
        }
        s_tx_pkts++;
    }
    s_tx_frames++;
}

/* append {id,len,value} */
static uint16_t put_tlv(uint8_t *p, uint16_t off, uint16_t id, const void *v, uint16_t n)
{
    p[off] = id & 0xFF; p[off + 1] = id >> 8;
    p[off + 2] = n & 0xFF; p[off + 3] = n >> 8;
    if (n) {
        memcpy(p + off + 4, v, n);
    }
    return off + 4 + n;
}

/* Value of one feature (v has room for 64 B); false if we don't know it. */
static bool feat_value(uint16_t id, uint8_t *v, uint16_t *n)
{
    uint32_t u32;
    switch (id) {
    case 0x0004: v[0] = 0x69; v[1] = 0x21; *n = 2; return true;               /* ProductID 2169 */
    case 0x0005: v[0] = 0x00; *n = 1; return true;                            /* ColorID */
    case 0x0006: *n = sizeof(s_name) - 1; memcpy(v, s_name, *n); return true; /* DeviceName */
    case 0x0008: *n = sizeof(s_serial) - 1; memcpy(v, s_serial, *n); return true; /* SerialNumber */
    case 0x000A: memcpy(v, s_local_addr, 6); *n = 6; return true;             /* DeviceMAC */
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
    /* Report "no party" to the app: with PartyStatus b7 (auracastParty) and
     * group = party, JBL One greys the EQ out ("not available in Auracast
     * mode"). The broadcast and the native join do not depend on these GATT
     * values (beacon + ext adv do that), so EQ stays usable while hosting. */
    case 0x2C00: v[0] = 0x00; *n = 1; return true;                            /* not in a party (EQ enabled) */
    case 0x2C01: v[0] = 0x00; *n = 1; return true;                            /* group: normal */
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

/* ---- EQ persistence ------------------------------------------------------------ */

typedef struct {
    uint8_t ver;
    uint8_t has_coef;
    uint8_t eq[8];
    uint8_t coef[V5_EQ_COEF_BYTES];
} eq_blob_t;

#define EQ_BLOB_VER 1
static eq_blob_t s_eq_blob = { .ver = EQ_BLOB_VER };
static btstack_timer_source_t s_persist_timer;

static void eq_persist_now(btstack_timer_source_t *ts)
{
    (void)ts;
    nvs_handle_t h;
    if (nvs_open("jbl_v5", NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    memcpy(s_eq_blob.eq, st.eq, sizeof(st.eq));
    esp_err_t e = nvs_set_blob(h, "eq", &s_eq_blob, sizeof(s_eq_blob));
    if (e == ESP_OK) {
        e = nvs_commit(h);
    }
    nvs_close(h);
    ESP_LOGI(TAG, "EQ stored (%s)", esp_err_to_name(e));
}

static void eq_persist_later(void)
{
#if V5_PERSIST_EQ
    btstack_run_loop_remove_timer(&s_persist_timer);
    btstack_run_loop_set_timer_handler(&s_persist_timer, &eq_persist_now);
    btstack_run_loop_set_timer(&s_persist_timer, V5_PERSIST_MS);
    btstack_run_loop_add_timer(&s_persist_timer);
#endif
}

static void eq_restore(void)
{
#if V5_PERSIST_EQ
    nvs_handle_t h;
    if (nvs_open("jbl_v5", NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    eq_blob_t b;
    size_t sz = sizeof(b);
    esp_err_t e = nvs_get_blob(h, "eq", &b, &sz);
    nvs_close(h);
    if (e != ESP_OK || sz != sizeof(b) || b.ver != EQ_BLOB_VER) {
        return;
    }
    s_eq_blob = b;
    memcpy(st.eq, b.eq, sizeof(st.eq));
    if (b.has_coef) {
        v5_eq_set_q29(b.coef);
    }
    ESP_LOGW(TAG, "EQ restored: category 0x%02x, bands %d %d %d %d %d %d %d%s", st.eq[0], st.eq[1], st.eq[2],
             st.eq[3], st.eq[4], st.eq[5], st.eq[6], st.eq[7], b.has_coef ? " (applied to audio)" : "");
#endif
}

/* ---- commands ---------------------------------------------------------------- */

static void notify_feature(hci_con_handle_t con, uint16_t id)
{
    uint8_t p[2 + 4 + 64], v[64];
    uint16_t n = 0;
    if (!feat_value(id, v, &n)) {
        return;
    }
    p[0] = 0; p[1] = 0;
    uint16_t off = put_tlv(p, 2, id, v, n);
    send_frame(con, 3, p, off);
}

static void handle_get(hci_con_handle_t con, const uint8_t *pl, uint16_t len)
{
    static uint8_t out[V5_MAX_PAYLOAD];
    uint8_t v[64];
    uint8_t miss[64];
    uint16_t off = 2, nmiss = 0;
    char dbg[200], mdbg[120];
    int dl = 0, ml = 0;
    dbg[0] = mdbg[0] = 0;
    out[0] = 0; out[1] = 0;                           /* status OK */
    for (uint16_t i = 0; i + 1 < len; i += 2) {
        uint16_t id = pl[i] | (pl[i + 1] << 8);
        uint16_t n = 0;
        if (dl < (int)sizeof(dbg) - 6) {
            dl += snprintf(dbg + dl, sizeof(dbg) - (size_t)dl, "%04x ", id);
        }
        if (feat_value(id, v, &n) && off + 4 + n <= 380) {
            off = put_tlv(out, off, id, v, n);
        } else if (nmiss + 2u <= (uint16_t)sizeof(miss)) {
            miss[nmiss++] = id & 0xFF;
            miss[nmiss++] = id >> 8;
            if (ml < (int)sizeof(mdbg) - 6) {
                ml += snprintf(mdbg + ml, sizeof(mdbg) - (size_t)ml, "%04x ", id);
            }
        }
    }
    ESP_LOGI(TAG, "app Get [%s] -> %u B%s%s", dbg, off, nmiss ? " | UNSUPPORTED: " : "", mdbg);
    send_frame(con, 1, out, off);
    if (nmiss) {
        /* like the real Xtreme 5: a second Get reply carrying 0x0000
         * UnsupportedFeatureList with the IDs it doesn't have */
        uint8_t u[2 + 4 + sizeof(miss)];
        u[0] = 0; u[1] = 0;
        uint16_t uo = put_tlv(u, 2, 0x0000, miss, nmiss);
        send_frame(con, 1, u, uo);
    }
}

static void handle_set(hci_con_handle_t con, const uint8_t *pl, uint16_t len)
{
    static const uint8_t ok[2] = { 0, 0 };
    bool eq_query = false, eq_changed = false;

    for (uint16_t i = 0; i + 4 <= len;) {
        uint16_t id = pl[i] | (pl[i + 1] << 8);
        uint16_t n = pl[i + 2] | (pl[i + 3] << 8);
        const uint8_t *v = pl + i + 4;
        if ((uint32_t)i + 4 + n > len) {
            break;
        }
        switch (id) {
        case 0x0E01: eq_query = true; break;                                  /* EqInfoQuery */
        case 0x0E7F:                                                          /* EqSetPortable */
            if (n >= 1) {
                st.eq[0] = v[0];
                uint16_t coef_off = 1;
                if (v[0] == 0xC1 || v[0] == 0x00) {                           /* custom: 7 indices */
                    if (n >= 8) {
                        memcpy(st.eq + 1, v + 1, 7);
                    }
                    coef_off = 8;
                } else {
                    memset(st.eq + 1, 0x0C, 7);
                }
                if (n >= coef_off + V5_EQ_COEF_BYTES) {
                    v5_eq_set_q29(v + coef_off);
                    memcpy(s_eq_blob.coef, v + coef_off, V5_EQ_COEF_BYTES);
                    s_eq_blob.has_coef = 1;
                }
                eq_changed = true;
                s_eq_sets++;
                ESP_LOGW(TAG, "EQ set: category 0x%02x, bands %d %d %d %d %d %d %d (%s)",
                         st.eq[0], st.eq[1], st.eq[2], st.eq[3], st.eq[4], st.eq[5], st.eq[6], st.eq[7],
                         v5_eq_is_on() ? "applied to audio" : "flat");
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
        ESP_LOGI(TAG, "app Set 0x%04x (%u B)", id, n);
        i += 4 + n;
    }
    send_frame(con, 2, ok, sizeof(ok));               /* every Set gets "00 00" */
    if (eq_query || eq_changed) {
        notify_feature(con, 0x0E7E);                  /* current EQ back to the app */
    }
    if (eq_changed) {
        eq_persist_later();
    }
}

/* reassembly for multi-packet frames: cold, PSRAM, allocated on first use */
static uint8_t *s_rx;
static uint16_t s_rx_len;

void jbl_v5_on_write(hci_con_handle_t con, const uint8_t *d, uint16_t len)
{
    if (len < 8 || d[0] != 0x00 || d[1] != 0xDD) {
        ESP_LOGI(TAG, "non-V5 write (%u B)", len);
        s_rx_bad++;
        return;
    }
    if (!s_linked || s_link != con) {
        s_link = con;                                 /* the app talks here now */
        s_linked = true;
        ESP_LOGW(TAG, "JBL One control link on con 0x%04x", con);
    }
    s_rx_frames++;
    uint16_t cmd = d[2] | (d[3] << 8);
    uint8_t cnt = d[4], idx = d[5];
    uint16_t plen = d[6] | (d[7] << 8);
    const uint8_t *pl = d + 8;
    uint16_t have = len - 8 < plen ? len - 8 : plen;

    if (cnt > 1) {                                    /* split frame: collect */
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
        if (idx == 0) {
            s_rx_len = 0;
        }
        if (s_rx_len + have <= V5_RX_MAX) {
            memcpy(s_rx + s_rx_len, pl, have);
            s_rx_len += have;
        }
        if (idx + 1 < cnt) {
            return;
        }
        s_rx_split++;
        pl = s_rx;
        have = s_rx_len;
    }

    switch (cmd) {
    case 1: s_rx_get++; handle_get(con, pl, have); break;
    case 2: s_rx_set++; handle_set(con, pl, have); break;
    case 4: break;                                    /* heartbeat ack from app */
    default: {
        char h[3 * 24 + 1];
        int hl = 0;
        h[0] = 0;
        for (uint16_t k = 0; k < len && k < 24; k++) {
            hl += snprintf(h + hl, sizeof(h) - (size_t)hl, "%02x ", d[k]);
        }
        ESP_LOGW(TAG, "app cmd 0x%04x (%u B): %s -> replied 00 00", cmd, have, h);
        /* Every command needs a reply with the same cmd id: the app's V5
         * queue waits 6 s per unanswered command (that delayed the EQ). */
        static const uint8_t ok[2] = { 0x00, 0x00 };
        send_frame(con, cmd, ok, sizeof(ok));
        break;
    }
    }
}

void jbl_v5_link_up(hci_con_handle_t con)
{
    s_link = con;
    s_linked = true;
    ESP_LOGW(TAG, "JBL One control link up (con 0x%04x, MTU %u)", con, jbl_gatt_mtu(con));
}

void jbl_v5_link_down(hci_con_handle_t con)
{
    if (s_linked && con == s_link) {
        s_linked = false;
        s_link = HCI_CON_HANDLE_INVALID;
        ESP_LOGW(TAG, "JBL One control link down");
    }
}

static void heartbeat(btstack_timer_source_t *ts)
{
    static const uint8_t hb[6] = { 0, 0, 0x02, 0x00, 0x00, 0x00 };
    if (s_linked) {
        if (jbl_gatt_tx_busy()) {
            s_hb_skipped++;                           /* optional traffic: never queue it behind a jam */
        } else {
            send_frame(s_link, 3, hb, sizeof(hb));
            s_hb_sent++;
        }
    }
    btstack_run_loop_set_timer(ts, V5_HEARTBEAT_MS);
    btstack_run_loop_add_timer(ts);
}

void jbl_v5_setup(void)
{
    eq_restore();
    btstack_run_loop_set_timer_handler(&s_hb_timer, &heartbeat);
    btstack_run_loop_set_timer(&s_hb_timer, V5_HEARTBEAT_MS);
    btstack_run_loop_add_timer(&s_hb_timer);
}

void jbl_v5_log_stats(void)
{
    ESP_LOGI(TAG, "V5 link %s (con 0x%04x) | rx %lu frames (Get %lu, Set %lu, split %lu, bad %lu) | "
             "tx %lu frames / %lu pkts, dropped %lu | heartbeat %lu sent, %lu skipped | EQ sets %lu",
             s_linked ? "up" : "down", s_link, (unsigned long)s_rx_frames, (unsigned long)s_rx_get,
             (unsigned long)s_rx_set, (unsigned long)s_rx_split, (unsigned long)s_rx_bad,
             (unsigned long)s_tx_frames, (unsigned long)s_tx_pkts, (unsigned long)s_tx_drop,
             (unsigned long)s_hb_sent, (unsigned long)s_hb_skipped, (unsigned long)s_eq_sets);
}
