/*
 * JBL "V5" protocol server on BTstack (port of the Bluedroid build's
 * jbl_v5_server.c; protocol decoded from JBL One 2.9.4, apk/notes_v5_server.md).
 *
 * Transport: GATT excelpoint.com (65786365-...-2e636f6d0000), over BR/EDR
 * (the app uses connectGatt(transport=BR/EDR)) or LE:
 *   ...0002  app -> speaker (write / write without response / long write)
 *   ...0001  speaker -> app (notify)
 * Frame: 00 DD cmd(2) pktCount pktIdx len(2) payload. Get = list of u16 IDs;
 * Set and every reply / Notify = {u16 id, u16 len, value}, replies start with
 * a u16 status (b15 = error). The app matches a reply to its request by the
 * command id alone, so every request gets exactly ONE reply frame.
 *
 * Values come from the live speaker state (phone volume, party role, AVRCP
 * track info); Sets act on it (volume, mute, play/pause/skip, Auracast on/off,
 * rename, factory reset) and settings persist in NVS. OTA (cmds 0x0101-0x0104)
 * receives the whole image into PSRAM, then stores it in the "jblota" flash
 * partition for read-back, and takes over the image's firmware version so the
 * app's post-update check passes.
 *
 * Everything here runs on the BTstack thread: no locks. Replies go to the
 * bearer the request came on; notifications go to the current app link.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "btstack.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_rom_md5.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "a2dp_in.h"
#include "jbl.h"
#include "jbl_beacon.h"
#include "hci_transport_s31_vhci.h"
#include "jbl_gatt.h"
#include "le_audio.h"
#include "jbl_v5.h"
#include "s31_ident.h"
#include "party.h"
#include "csb.h"
#include "lls.h"
#include "audio_out.h"
#include "jbl_assist.h"
#include "tones.h"
#include "power.h"
#include "v5_eq.h"

static const char *TAG = "JBL_V5";

#define V5_MAX_PAYLOAD   400        /* longest reply we build */
#define V5_RX_MAX        600        /* longest split frame we reassemble */
#define V5_HEARTBEAT_MS  5000       /* the real speaker sends one every few seconds */
#define V5_PERSIST_EQ    1          /* keep the app's EQ across reboots (NVS) */
#define V5_PERSIST_MS    5000       /* debounce: write once the slider settles */
#define V5_NAME_MAX      32
#define V5_VAL_MAX       192

/* Firmware version (000C) reported until an OTA image has been captured: below
 * the cloud's current Xtreme 5 release (26.40.10), so JBL One offers the update. */
#define V5_FW_DEFAULT    { 26, 29, 12 }

/* ---- speaker settings the app can read / change (persisted) ---------------- */

typedef struct {
    uint8_t ver;
    uint8_t feedback_tone;    /* 0x0C40 / 0x200F: b7 on */
    uint16_t auto_off;        /* 0x0012: b15 on, b0-14 seconds */
    uint16_t auto_standby;    /* 0x0013: same layout */
    uint8_t bass_boost;       /* 0x2009: b7 on, b0-6 level */
    uint8_t playtime_boost;   /* 0x2006 */
    uint8_t smart_eq;         /* 0x0E80 */
    uint8_t remote_power;     /* 0x0056 */
    uint8_t le_audio;         /* 0x0BA0 */
    uint8_t mood;             /* 0x2007: 7F = none */
    uint8_t vocal_removal;    /* 0x200B: 7F = none */
    uint8_t light_status;     /* 0x0D00 */
    uint8_t light_show;       /* 0x0D40 */
    uint8_t light_bright;     /* 0x0D41 */
    uint8_t light_temp;       /* 0x0D42 */
    uint8_t light_speed;      /* 0x0D43 */
    uint8_t bass_light;       /* 0x0D45 */
    uint8_t light_preset;     /* 0x0D46 */
    uint8_t light_rgb[3];     /* 0x0D47 */
    uint8_t rgb_loop[4];      /* 0x0D80 */
    uint8_t party_channel;    /* 0x2C04 */
    uint8_t group_id[4];      /* 0x2C06 */
    uint8_t partner[6];       /* 0x2C81 */
    uint8_t fw[3];            /* 0x000C: default, or the version of the captured OTA image */
    float pitch;              /* 0x30C2 */
    char name[V5_NAME_MAX + 1];       /* 0x0006 */
    char group_name[17];              /* (old 0x2C05, unused) */
    /* ---- v3: appended, so a v2 blob loads as a prefix ---- */
    char stereo_name[24];             /* 0x2C05 "<model> Stereo" */
    char stereo_id[9];                /* 0x2C06: 8 ASCII hex chars */
    uint8_t stereo_role;              /* 0x2C03 while paired: 1 secondary, 2 primary */
    uint8_t lls;                      /* 0x2C80: stereo link set up */
    uint8_t bcast_hq;                 /* 0x0B87: 0x80 HQ, 0x00 SQ */
    char bcast_name[33];              /* 0x0B8A */
    uint8_t eq_be;                    /* 0x0E02 floats big-endian (as the app last sent) */
} v5_cfg_t;

#define CFG_VER 3
#define CFG_V2_SIZE offsetof(v5_cfg_t, stereo_name)
static v5_cfg_t cfg = {
    .ver = CFG_VER,
    .feedback_tone = 0x80,
    .auto_standby = 0x8000,
    .bass_boost = 0x81,
    .remote_power = 0x00,
    .mood = 0x7F,
    .vocal_removal = 0x7F,
    .light_status = 0x80,
    .light_show = 0x81,
    .light_bright = 0xBC,
    .light_temp = 0x7F,
    .light_speed = 0x7F,
    .light_preset = 0x01,               /* 0 is no valid preset: the iOS app dropped the link on it */
    .rgb_loop = { 0x01, 0x00, 0x00, 0x00 },
    .fw = V5_FW_DEFAULT,
    .name = JBL_DEVICE_NAME,
    .auto_off = 1200,                   /* off by default (user's choice, 2026-11-24): always on;
                                         * JBL One's switch sets b15, 20 min stays the time */
    .bcast_hq = 0x80,
    .bcast_name = JBL_BROADCAST_NAME,
    .eq_be = 1,
};
static bool s_cfg_loaded;
static bool s_eq_dirty, s_cfg_dirty;              /* persist on the debounce timer */
static uint32_t s_idle_s;                         /* auto power off: seconds without audio */
static uint16_t s_manual_off_s;                   /* JBL One power-off timer */
static void eq_store_coef(const uint8_t *coef);

static uint8_t s_eq[8] = { 0x80, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C };   /* 0x0E7E: JBL Signature */

static bd_addr_t s_local_addr;

/* ---- link ------------------------------------------------------------------ */

static hci_con_handle_t s_link = HCI_CON_HANDLE_INVALID;
static bool s_linked;
static btstack_timer_source_t s_hb_timer;

static uint32_t s_rx_frames, s_rx_get, s_rx_set, s_rx_bad, s_rx_split;
static uint32_t s_tx_frames, s_tx_pkts, s_tx_drop, s_hb_sent, s_hb_skipped, s_eq_sets, s_notifies;

void jbl_v5_set_local_addr(const bd_addr_t addr)
{
    memcpy(s_local_addr, addr, sizeof(bd_addr_t));
}

/* One V5 message on `con`; split into pktCount packets when it doesn't fit one
 * notification (the app reassembles until pktIdx == pktCount - 1). */
/* routing identifier of the frame being sent: 0x00 Direct (us), 0x01 Forward (stereo
 * partner), 0x02 SubWoofer - replies go back on the identifier they came on */
static uint8_t s_tx_ident;

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
        f[0] = s_tx_ident; f[1] = 0xDD;
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

/* ---- live state mapping ------------------------------------------------------ */

static uint8_t to100(int v127) { return (uint8_t)((v127 * 100 + 63) / 127); }
static uint8_t to127(int v100) { return (uint8_t)((v100 > 100 ? 100 : v100) * 127 / 100); }

/* what the S31 plays at: the listen volume while a party member, else the phone's */
static uint8_t heard_volume100(void)
{
    int lv = party_listen_volume();
    return to100(lv >= 0 ? lv : a2dp_in_volume());
}

/* 2D40 AudioOperation as a state: 2 play, 1 pause, 0 stop */
static uint8_t play_op(void)
{
    if (party_listening()) return 2;                  /* the party stream is playing */
    switch (a2dp_in_play_status()) {
    case AVRCP_PLAYBACK_STATUS_PLAYING: return 2;
    case AVRCP_PLAYBACK_STATUS_PAUSED: return 1;
    default: return 0;
    }
}

static size_t str_copy(uint8_t *v, const char *s, size_t max)
{
    size_t n = strlen(s);
    if (n > max) n = max;
    memcpy(v, s, n);
    return n;
}

/* ---- EQ, one neutral copy for every phone -------------------------------------
 * The app speaks two EQ forms: 0x0E7E/0x0E7F (category + 7 gain indices +
 * coefficients) and 0x0E02 EqInfo (7 bands: type + f32 freq/gain/Q, no
 * coefficients). Both are kept in s_eqc, so a custom EQ set from one phone is
 * what every other phone reads back, whichever form it uses.
 * EqInfo on the wire (V5EqInfo.valuePayload): cat, status, count, 4 header f32
 * (calibration, sample rate, L/R gain) then 13 B bands. Android writes the header
 * ALWAYS little-endian and the bands big-endian (cloud flag EQBigEndian for 2169;
 * otherwise LE + a trailing preset name); the iOS app writes everything
 * big-endian. So the band order is read from the bands themselves, and replies
 * to the iOS app's link get a BE header. */
#define EQI_MAX 160
typedef struct {
    uint8_t type;
    float f, g, q;
} eq_band_t;
typedef struct {
    uint8_t ver, cat, status, count;
    eq_band_t b[V5_EQ_BANDS];
    char name[33];
} eq_canon_t;
#define EQC_VER 1
static const uint8_t s_std_type[7] = { 0, 1, 1, 1, 1, 1, 2 };
static const float s_std_f[7] = { 125, 250, 500, 1000, 2000, 4000, 8000 };
static const float s_std_q[7] = { 0.707f, 2, 2, 2, 2, 2, 0.707f };
static eq_canon_t s_eqc;

/* an EQ slider drag sends a Set every ~100 ms: log at most one per second (the UART
 * time is BT-thread time, and every Set is a round trip the app waits on) */
static bool eq_log_ok(void)
{
    static uint32_t last_ms;
    uint32_t now = btstack_run_loop_get_time_ms();
    if (last_ms != 0 && now - last_ms < 1000) return false;
    last_ms = now;
    return true;
}
static hci_con_handle_t s_cur_con = HCI_CON_HANDLE_INVALID;   /* link the current reply goes to */
static hci_con_handle_t s_ios_con = HCI_CON_HANDLE_INVALID;   /* link of the iOS app */

static void eqc_standard(uint8_t cat, const float gains[7])
{
    s_eqc.ver = EQC_VER;
    s_eqc.cat = cat;
    s_eqc.status = 1;
    s_eqc.count = 7;
    for (int i = 0; i < 7; i++) {
        s_eqc.b[i] = (eq_band_t){ s_std_type[i], s_std_f[i], gains ? gains[i] : 0.0f, s_std_q[i] };
    }
    s_eqc.name[0] = 0;
}

static void put_f32(uint8_t *p, float f, bool be)
{
    uint32_t u;
    memcpy(&u, &f, 4);
    if (be) { p[0] = u >> 24; p[1] = u >> 16; p[2] = u >> 8; p[3] = u; }
    else memcpy(p, &u, 4);
}

static float get_f32(const uint8_t *p, bool be)
{
    uint32_t u;
    if (be) u = (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
    else memcpy(&u, p, 4);
    float f;
    memcpy(&f, &u, 4);
    return f;
}

static bool freq_ok(float f) { return f >= 10.0f && f <= 24000.0f; }

/* gain index of the portable form: bands 1-6 = 6 - 0.5 idx; band 0 = 6 - 0.5 idx
 * down to 0 dB (idx 12), then 0.75 dB steps to -9 dB (idx 24) */
static float idx_gain(int band, uint8_t idx)
{
    if (idx > 24) idx = 24;
    if (band == 0 && idx > 12) return -0.75f * (float)(idx - 12);
    return 6.0f - 0.5f * (float)idx;
}

static uint8_t gain_idx(int band, float g)
{
    long k;
    if (band == 0 && g < 0.0f) k = 12 + lrintf(-g / 0.75f);
    else k = lrintf((6.0f - g) * 2.0f);
    return (uint8_t)(k < 0 ? 0 : k > 24 ? 24 : k);
}

/* 0x0E7E view of the neutral copy */
static void eqc_to_portable(void)
{
    s_eq[0] = s_eqc.cat;
    bool custom = s_eqc.cat == 0xC1 || s_eqc.cat == 0x00;
    for (int i = 0; i < 7; i++) {
        s_eq[1 + i] = custom && i < s_eqc.count ? gain_idx(i, s_eqc.b[i].g) : 0x0C;
    }
}

/* Set 0x0E7F (portable form): category + indices -> neutral copy */
static void eqc_from_portable(void)
{
    float g[7];
    bool custom = s_eq[0] == 0xC1 || s_eq[0] == 0x00;
    for (int i = 0; i < 7; i++) g[i] = custom ? idx_gain(i, s_eq[1 + i]) : 0.0f;
    eqc_standard(s_eq[0], g);
}

static uint16_t eq_info_value(uint8_t *v)
{
    if (s_eqc.ver != EQC_VER) eqc_standard(0x80, NULL);
    bool hdr_be = s_cur_con != HCI_CON_HANDLE_INVALID && s_cur_con == s_ios_con;
    bool band_be = cfg.eq_be;
    v[0] = s_eqc.cat; v[1] = s_eqc.status; v[2] = s_eqc.count;
    uint32_t nul = 0x4F800000u;
    float fnul;
    memcpy(&fnul, &nul, 4);
    put_f32(v + 3, fnul, hdr_be);
    put_f32(v + 7, 48000.0f, hdr_be);
    put_f32(v + 11, fnul, hdr_be);
    put_f32(v + 15, fnul, hdr_be);
    uint16_t o = 19;
    for (int i = 0; i < s_eqc.count && i < V5_EQ_BANDS; i++) {
        v[o] = s_eqc.b[i].type;
        put_f32(v + o + 1, s_eqc.b[i].f, band_be);
        put_f32(v + o + 5, s_eqc.b[i].g, band_be);
        put_f32(v + o + 9, s_eqc.b[i].q, band_be);
        o += 13;
    }
    if (!band_be) {                                  /* LE form carries the preset name */
        size_t k = strlen(s_eqc.name);
        memcpy(v + o, s_eqc.name, k);
        o += (uint16_t)k;
    }
    return o;
}

/* coefficients for the audio path from the neutral copy */
static void eqc_apply(void)
{
    uint8_t coef[V5_EQ_COEF_BYTES];
    for (int i = 0; i < V5_EQ_BANDS; i++) {
        if (i == 0 && cfg.playtime_boost) {
            /* Playtime Boost (battery saver on a real Xtreme 5): less bass, the costly
             * part for the amplifier - band 0 as a low shelf 6 dB lower */
            float f = (s_eqc.count > 0 && s_eqc.b[0].type == 0) ? s_eqc.b[0].f : 125.0f;
            float g = (s_eqc.count > 0 && s_eqc.b[0].type == 0) ? s_eqc.b[0].g : 0.0f;
            v5_eq_design_q29(0, f, g - 6.0f, 0.707f, coef);
        } else if (i < s_eqc.count) {
            v5_eq_design_q29(s_eqc.b[i].type, s_eqc.b[i].f, s_eqc.b[i].g, s_eqc.b[i].q, coef + 20 * i);
        } else {
            v5_eq_design_q29(1, 1000.0f, 0.0f, 1.0f, coef + 20 * i);   /* identity */
        }
    }
    v5_eq_set_q29(coef);
    eq_store_coef(coef);
}

/* Set 0x0E02: bands -> neutral copy -> 7 biquads built here */
static bool eq_info_apply(const uint8_t *v, uint16_t n)
{
    if (n < 19 || n > EQI_MAX) return false;
    uint8_t cnt = v[2] > V5_EQ_BANDS ? V5_EQ_BANDS : v[2];
    if (19u + 13u * cnt > n) return false;
    eq_canon_t c = { .ver = EQC_VER, .cat = v[0], .status = v[1], .count = cnt };
    bool be_seen = false, le_seen = false;
    char dbg[220];
    int dl = 0;
    for (int i = 0; i < cnt; i++) {
        const uint8_t *b = v + 19 + 13 * i;
        bool be = freq_ok(get_f32(b + 1, true)) || !freq_ok(get_f32(b + 1, false));
        if (be) be_seen = true; else le_seen = true;
        c.b[i] = (eq_band_t){ b[0], get_f32(b + 1, be), get_f32(b + 5, be), get_f32(b + 9, be) };
        if (!freq_ok(c.b[i].f) || c.b[i].type > 4 || !(c.b[i].g > -30.0f && c.b[i].g < 30.0f)) {
            c.b[i] = (eq_band_t){ 1, 1000.0f, 0.0f, 1.0f };   /* unreadable / unsupported band: flat */
        } else if (!(c.b[i].q >= 0.3f && c.b[i].q <= 10.0f)) {
            c.b[i].q = c.b[i].type >= 3 ? 0.707f : 1.0f;      /* a pass filter never with a silly Q */
        }
        if (dl < (int)sizeof(dbg) - 32) {
            dl += snprintf(dbg + dl, sizeof(dbg) - (size_t)dl, " %u:%.0fHz %+.1fdB Q%.2f", c.b[i].type, c.b[i].f,
                           c.b[i].g, c.b[i].q);
        }
    }
    uint16_t tail = (uint16_t)(19u + 13u * cnt);
    if (n > tail) {                                    /* LE form: preset name */
        size_t k = n - tail > 32 ? 32 : n - tail;
        memcpy(c.name, v + tail, k);
        c.name[k] = 0;
    }
    bool hdr_be = v[7] == 0x47 && v[8] == 0x3B;        /* 48000.0f big-endian */
    /* replies to this link mirror the header order it just used */
    s_ios_con = hdr_be ? s_cur_con : (s_ios_con == s_cur_con ? HCI_CON_HANDLE_INVALID : s_ios_con);
    s_eqc = c;
    eqc_apply();
    eqc_to_portable();
    bool bands_be = be_seen || !le_seen;
    if (cfg.eq_be != bands_be) {
        cfg.eq_be = bands_be;
        s_cfg_dirty = true;
    }
    if (eq_log_ok()) {
        ESP_LOGW(TAG, "EQ (EqInfo, header %s, bands %s): category 0x%02x%s%s%s (%s) | %lu sets",
                 hdr_be ? "BE" : "LE", be_seen ? (le_seen ? "mixed" : "BE") : "LE", c.cat,
                 c.name[0] ? " '" : "", c.name, c.name[0] ? "'" : "",
                 v5_eq_is_on() ? "applied to audio" : "flat", (unsigned long)(s_eq_sets + 1));
        ESP_LOGW(TAG, "EQ bands:%s", dbg);
    }
    return true;
}

/* ---- Broadcast codes: the app sends an ASCII password; LE Audio uses it as a
 * zero-padded 16-byte string, first character in the first HCI octet. BTstack
 * reverses 16-byte fields when it builds the command, so store it reversed. */
static char s_bcast_pw[17];                      /* 0x0B88, "" = open broadcast */
static uint8_t s_bcast_code[16];                 /* BTstack order */

static bool pw_is_none(const uint8_t *v, uint16_t n)
{
    if (n == 0) return true;
    for (uint16_t k = 0; k < n; k++) {
        if (v[k] != 0xFF && v[k] != 0x00) return false;
    }
    return true;
}

static void pw_to_code(const uint8_t *v, uint16_t n, uint8_t code[16])
{
    memset(code, 0, 16);
    for (uint16_t k = 0; k < n && k < 16; k++) code[15 - k] = v[k];
}

/* ---- 0x0B80 Auracast assistant status, from party.c's join state */
static uint16_t assist_info(uint8_t *v)
{
    party_assist_t a;
    party_assist_get(&a);
    uint8_t st = a.joined ? 3 : (a.joining ? 2 : 0);
    if (a.scanning) st = st == 0 ? 1 : st + 2;            /* 1 / 4 / 5 */
    v[0] = st;
    if (st < 2 && st != 1) return 1;
    if (st == 1) return 1;
    v[1] = a.joined ? 0 : 0xFF;                          /* playing subgroup */
    uint8_t *g = v + 2;                                  /* GroupInfo, 41 B */
    memcpy(g, a.addr, 6);
    g[6] = a.addr_type;
    g[7] = a.encrypted ? 0x80 : 0x00;
    memset(g + 8, 0, 33);
    str_copy(g + 8, a.name, 32);
    uint8_t *sg = g + 41;                                /* one LC3 subgroup */
    sg[0] = 0; sg[1] = 2; sg[2] = a.kbps & 0xFF; sg[3] = a.kbps >> 8;
    return 2 + 41 + 4;
}

/* Value of one feature (v has room for V5_VAL_MAX B); false if we don't know it. */
static bool feat_value(uint16_t id, uint8_t *v, uint16_t *n)
{
    uint32_t u32;
    uint8_t role = party_v5_role();
    switch (id) {
    /* device info / power */
    case 0x0001: {                                                            /* MTU: V5 bytes per packet */
        uint16_t m = s_cur_con != HCI_CON_HANDLE_INVALID ? jbl_gatt_mtu(s_cur_con) : 0;
        m = m > 3 ? (uint16_t)(m - 3) : 20;
        v[0] = m & 0xFF; v[1] = m >> 8; *n = 2; return true;
    }
    case 0x0004: v[0] = s31_ident_model() & 0xFF; v[1] = s31_ident_model() >> 8; *n = 2; return true;   /* ProductID */
    case 0x0005: v[0] = 0x00; v[1] = s31_ident_color(); *n = 2; return true;  /* ColorID (set when flashing) */
    case 0x0006: *n = (uint16_t)str_copy(v, cfg.name, V5_NAME_MAX); return true;
    case 0x0008: *n = (uint16_t)str_copy(v, s31_ident_serial(), 32); return true;   /* SerialNumber (per board) */
    case 0x000A: memcpy(v, s_local_addr, 6); *n = 6; return true;             /* DeviceMAC (BR/EDR) */
    case 0x000C: memcpy(v, cfg.fw, 3); *n = 3; return true;                   /* FirmwareVersion */
    case 0x000D: v[0] = 100; *n = 1; return true;                             /* battery: mains, 100 % */
    case 0x0012: v[0] = cfg.auto_off & 0xFF; v[1] = cfg.auto_off >> 8; *n = 2; return true;
    case 0x0013: v[0] = cfg.auto_standby & 0xFF; v[1] = cfg.auto_standby >> 8; *n = 2; return true;
    case 0x0045: v[0] = 0x01; *n = 1; return true;                            /* PowerStatus on */
    case 0x0056: v[0] = cfg.remote_power; *n = 1; return true;
    case 0x1008: u32 = 600; memcpy(v, &u32, 4); *n = 4; return true;          /* TotalPlaybackTime, min */
    case 0x0C40: v[0] = cfg.feedback_tone; *n = 1; return true;
    case 0x200F: v[0] = cfg.feedback_tone; *n = 1; return true;               /* generic feedback tone */
    case 0x3802: v[0] = 0x00; *n = 1; return true;                            /* BackwardScreen */
    case 0x2006: v[0] = cfg.playtime_boost; *n = 1; return true;
    case 0x0BA0: v[0] = cfg.le_audio; *n = 1; return true;
    /* audio */
    case 0x0BA1: v[0] = party_listening() ? 0x10 : 0x01; *n = 1; return true; /* source: Auracast / BT */
    case 0x2D40: v[0] = play_op(); *n = 1; return true;
    case 0x2D41: v[0] = a2dp_in_muted() ? 1 : 0; *n = 1; return true;
    case 0x2D42: v[0] = heard_volume100(); *n = 1; return true;
    case 0x2D43: *n = (uint16_t)str_copy(v, a2dp_in_track_title(), 60); return true;
    case 0x2D44: *n = (uint16_t)str_copy(v, a2dp_in_track_artist(), 60); return true;
    case 0x2D47: v[0] = 0x00; *n = 1; return true;                            /* AudioContentDetected off */
    case 0x2D49: v[0] = play_op() == 2 ? 1 : 0; *n = 1; return true;         /* AudioPlayingStatus */
    case 0x0E7E: memcpy(v, s_eq, 8); *n = 8; return true;
    case 0x0E02: *n = eq_info_value(v); return true;                          /* EqInfo (band form) */
    case 0x0E80: v[0] = cfg.smart_eq; *n = 1; return true;
    case 0x2009: v[0] = cfg.bass_boost; *n = 1; return true;
    case 0x2007: v[0] = cfg.mood; *n = 1; return true;
    case 0x200B: v[0] = cfg.vocal_removal; *n = 1; return true;
    case 0x30C2: memcpy(v, &cfg.pitch, 4); *n = 4; return true;
    case 0x2005: v[0] = 0x00; *n = 1; return true;                            /* standard quality */
    /* Auracast / party. PartyStatus and GroupType stay 0: with them set JBL One
     * greys the EQ out ("not available in Auracast mode") and refuses OTA; the
     * native join does not depend on them (beacon + ext adv do that). */
    case 0x2004: v[0] = party_auracast_enabled() ? 1 : 0; *n = 1; return true; /* AuracastMode */
    case 0x0B80: *n = assist_info(v); return true;                            /* assistant status */
    case 0x0B86: v[0] = party_standard_broadcast_on() ? 0x80 : 0x00; *n = 1; return true; /* Broadcast to Others */
    case 0x0B87: v[0] = cfg.bcast_hq; *n = 1; return true;                    /* broadcast quality */
    case 0x0B88:                                                              /* broadcast password */
        if (s_bcast_pw[0]) { *n = (uint16_t)str_copy(v, s_bcast_pw, 16); }
        else { memset(v, 0xFF, 4); *n = 4; }                                  /* none: FF FF FF FF */
        return true;
    case 0x0B8B:                                                              /* subgroup names {idx, 32 B} */
        v[0] = 0;
        memset(v + 1, 0, 32);
        str_copy(v + 1, "Audio", 32);
        *n = 33;
        return true;
    case 0x0B89: memset(v, 0, 6); *n = 6; return true;
    case 0x0B8A: *n = (uint16_t)str_copy(v, cfg.bcast_name, 32); return true;
    /* the fw image reads 0x80 / 4 here (0x10549a8), but JBL One only showed the stereo card
     * with these (2026-11-24: with 0x80 / 4 the app never switched to the group) */
    case 0x2C00: v[0] = cfg.lls ? 0x40 : 0x00; *n = 1; return true;           /* b6 Auracast stereo */
    case 0x2C01: v[0] = cfg.lls ? 0x02 : role ? 0x01 : 0x00; *n = 1; return true;   /* 2 AuracastStereo, 1 party */
    case 0x2C02: v[0] = (role || cfg.lls) ? 0x02 : 0x00; *n = 1; return true; /* connected while in a party */
    case 0x2C03: v[0] = cfg.lls ? cfg.stereo_role : role; *n = 1; return true; /* 0 normal, 1 secondary, 2 primary */
    case 0x2C04: v[0] = cfg.party_channel; *n = 1; return true;
    case 0x2C05: *n = (uint16_t)str_copy(v, cfg.stereo_name, 23); return true;
    case 0x2C06: *n = (uint16_t)str_copy(v, cfg.stereo_id, 8); return true;
    case 0x2C80: v[0] = cfg.lls ? 0x01 : 0x00; *n = 1; return true;            /* stereo link */
    case 0x2C81: memcpy(v, cfg.partner, 6); *n = 6; return true;
    /* lights (stored for the WS2812B effects) */
    case 0x0D00: v[0] = cfg.light_status & 0xCF; *n = 1; return true;     /* never "syncing" */
    case 0x0D40: v[0] = cfg.light_show; *n = 1; return true;
    case 0x0D41: v[0] = cfg.light_bright; *n = 1; return true;
    case 0x0D42: v[0] = cfg.light_temp; *n = 1; return true;
    case 0x0D43: v[0] = cfg.light_speed; *n = 1; return true;
    case 0x0D45: v[0] = cfg.bass_light; *n = 1; return true;
    case 0x0D46: v[0] = cfg.light_preset ? cfg.light_preset : 0x01; *n = 1; return true;
    case 0x0D47: memcpy(v, cfg.light_rgb, 3); *n = 3; return true;
    case 0x0D48: v[0] = 0x01; *n = 1; return true;                            /* LoopInfo: off */
    case 0x0D62: v[0] = 0x00; *n = 1; return true;                            /* ChargingLight */
    case 0x0D80: memcpy(v, cfg.rgb_loop, 4); *n = 4; return true;
    /* mic (MicSupport is on for 2169): nothing connected */
    case 0x0C00: v[0] = 0x00; *n = 1; return true;
    case 0x0C12: v[0] = 0x00; *n = 1; return true;
    case 0x0C31: v[0] = 0x00; v[1] = 0x00; v[2] = 0x00; *n = 3; return true;
    default: return false;
    }
}

/* ---- persistence --------------------------------------------------------------- */

typedef struct {
    uint8_t ver;
    uint8_t has_coef;
    uint8_t eq[8];
    uint8_t coef[V5_EQ_COEF_BYTES];
} eq_blob_t;

#define EQ_BLOB_VER 1
static eq_blob_t s_eq_blob = { .ver = EQ_BLOB_VER };
static btstack_timer_source_t s_persist_timer;

static void eq_store_coef(const uint8_t *coef)
{
    memcpy(s_eq_blob.coef, coef, V5_EQ_COEF_BYTES);
    s_eq_blob.has_coef = 1;
}

static void cfg_load(void)
{
    if (s_cfg_loaded) return;
    s_cfg_loaded = true;
    nvs_handle_t h;
    if (nvs_open("jbl_v5", NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    v5_cfg_t c;
    size_t sz = sizeof(c);
    esp_err_t ge = nvs_get_blob(h, "cfg", &c, &sz);
    if (ge == ESP_OK && (sz == CFG_V2_SIZE || sz == ((CFG_V2_SIZE + 3u) & ~3u)) && c.ver == 2) {
        /* v2 (its size includes the struct's tail padding): keep it, defaults for the new fields */
        memcpy((uint8_t *)&c + CFG_V2_SIZE, (const uint8_t *)&cfg + CFG_V2_SIZE, sizeof(c) - CFG_V2_SIZE);
        c.ver = CFG_VER;
        sz = sizeof(c);
    }
    if (ge == ESP_OK && sz == sizeof(c) && c.ver == CFG_VER) {
        c.name[V5_NAME_MAX] = 0;
        c.group_name[16] = 0;
        if (c.name[0] == 0) strcpy(c.name, JBL_DEVICE_NAME);
        cfg = c;
        ESP_LOGW(TAG, "settings restored: name '%s', firmware %u.%u.%u, feedback tone %s",
                 cfg.name, cfg.fw[0], cfg.fw[1], cfg.fw[2], (cfg.feedback_tone & 0x80) ? "on" : "off");
    } else if (s31_ident_name()[0]) {
        /* first boot after flashing: the name picked in the flasher */
        strncpy(cfg.name, s31_ident_name(), V5_NAME_MAX);
        cfg.name[V5_NAME_MAX] = 0;
        ESP_LOGW(TAG, "first boot: name '%s' (set when flashing)", cfg.name);
    }
    nvs_close(h);
}

static void persist_now(btstack_timer_source_t *ts)
{
    (void)ts;
    nvs_handle_t h;
    if (nvs_open("jbl_v5", NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    esp_err_t e = ESP_OK;
    if (s_eq_dirty) {
        memcpy(s_eq_blob.eq, s_eq, sizeof(s_eq));
        e = nvs_set_blob(h, "eq", &s_eq_blob, sizeof(s_eq_blob));
    }
    if (e == ESP_OK && s_eq_dirty && s_eqc.ver == EQC_VER) {
        e = nvs_set_blob(h, "eqc", &s_eqc, sizeof(s_eqc));
    }
    if (e == ESP_OK && s_cfg_dirty) {
        e = nvs_set_blob(h, "cfg", &cfg, sizeof(cfg));
    }
    if (e == ESP_OK) {
        e = nvs_commit(h);
    }
    nvs_close(h);
    ESP_LOGI(TAG, "settings stored (%s%s) %s", s_eq_dirty ? "EQ " : "", s_cfg_dirty ? "cfg" : "",
             esp_err_to_name(e));
    s_eq_dirty = s_cfg_dirty = false;
}

static void persist_later(bool eq)
{
    if (eq) {
#if V5_PERSIST_EQ
        s_eq_dirty = true;
#endif
    } else {
        s_cfg_dirty = true;
    }
    if (!s_eq_dirty && !s_cfg_dirty) return;
    btstack_run_loop_remove_timer(&s_persist_timer);
    btstack_run_loop_set_timer_handler(&s_persist_timer, &persist_now);
    btstack_run_loop_set_timer(&s_persist_timer, V5_PERSIST_MS);
    btstack_run_loop_add_timer(&s_persist_timer);
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
    memcpy(s_eq, b.eq, sizeof(s_eq));
    {
        nvs_handle_t h2;
        size_t il = sizeof(s_eqc);
        eq_canon_t c;
        if (nvs_open("jbl_v5", NVS_READONLY, &h2) == ESP_OK) {
            if (nvs_get_blob(h2, "eqc", &c, &il) == ESP_OK && il == sizeof(c) && c.ver == EQC_VER) {
                s_eqc = c;
                eqc_to_portable();
            }
            nvs_close(h2);
        }
    }
    if (b.has_coef) {
        v5_eq_set_q29(b.coef);
    }
    ESP_LOGW(TAG, "EQ restored: category 0x%02x, bands %d %d %d %d %d %d %d%s", s_eq[0], s_eq[1], s_eq[2],
             s_eq[3], s_eq[4], s_eq[5], s_eq[6], s_eq[7], b.has_coef ? " (applied to audio)" : "");
#endif
}

bool jbl_v5_feedback_tone_on(void)
{
    cfg_load();
    return (cfg.feedback_tone & 0x80) != 0;
}

const char *jbl_device_name(void)
{
    cfg_load();
    return cfg.name;
}

/* ---- notifications -------------------------------------------------------------- */

static void notify_features(hci_con_handle_t con, const uint16_t *ids, int count)
{
    s_cur_con = con;
    static uint8_t p[V5_MAX_PAYLOAD];
    uint8_t v[V5_VAL_MAX];
    uint16_t off = 2;
    p[0] = 0; p[1] = 0;
    for (int i = 0; i < count; i++) {
        uint16_t n = 0;
        if (feat_value(ids[i], v, &n) && off + 4 + n <= V5_MAX_PAYLOAD) {
            off = put_tlv(p, off, ids[i], v, n);
        }
    }
    if (off > 2) {
        send_frame(con, 3, p, off);
        s_notifies++;
    }
}

void jbl_v5_notify(uint16_t id);

static void stereo_resume_cb(btstack_timer_source_t *ts)
{
    (void)ts;
    jbl_v5_stereo_resume();
}

static void stereo_linked(bool linked)
{
    bool was = cfg.lls != 0;
    cfg.lls = linked ? 1 : 0;
    if (!linked && was) {   /* the link dropped, the pair is still stored: search again, no timeout (X5) */
        static btstack_timer_source_t t;
        btstack_run_loop_remove_timer(&t);
        btstack_run_loop_set_timer_handler(&t, stereo_resume_cb);
        btstack_run_loop_set_timer(&t, 1000);
        btstack_run_loop_add_timer(&t);
    }
    ESP_LOGW(TAG, "stereo pair: partner link %s - 0x2C80 = %u", linked ? "UP" : "down", cfg.lls);
    if (linked) {
        /* beacon as a linked stereo speaker, exactly as a real Xtreme 5 in a pair advertises it
         * (0xD080 left / 0xD100 right, either role): b15 on, group Stereo, Connected, no role
         * bits, our channel. Without b15 (0x54C0 before) JBL One took the pair as off and only
         * grouped the cards later, over its control link. The group id byte stays (the cards
         * are grouped by it). */
        uint16_t pi = (uint16_t)(0x8000u | 0x4000u | 0x1000u | ((cfg.party_channel & 3u) << 7));
        jbl_set_stereo_wait(pi, (uint8_t)cfg.stereo_id[3]);
        /* the primary splits: it plays its own channel and broadcasts the secondary's as one BIS */
        audio_out_set_channel(cfg.stereo_role == 2 ? cfg.party_channel : 0);
        party_set_stereo_primary(cfg.stereo_role == 2);
        ESP_LOGW(TAG, "stereo pair: beacon partyInfo 0x%04x, playing the %s channel%s", pi,
                 cfg.party_channel == 2 ? "RIGHT" : "LEFT", cfg.stereo_role == 2 ? ", broadcast on" : "");
    } else {
        jbl_set_stereo_wait(0, 0);
        audio_out_set_channel(0);
        party_set_stereo_primary(false);
    }
    jbl_v5_notify(0x2C80);
    jbl_v5_notify(0x2C01);
    jbl_v5_notify(0x2C00);
}

/* the pair is over: settings, beacon, channel, broadcast hold back to normal */
static void stereo_clear(void)
{
    cfg.lls = 0;
    cfg.stereo_role = 0;
    cfg.party_channel = 0;
    cfg.stereo_name[0] = cfg.stereo_id[0] = 0;
    memset(cfg.partner, 0, 6);
    stereo_linked(false);
    /* stored at once on every path: ungrouped by the partner or the ungroup key only cleared RAM,
     * and after a restart the S31 searched for the old partner again for good (directed adverts
     * every 20 ms next to the phone links, 2026-11-25) */
    persist_later(false);
}

void jbl_v5_stereo_partner_ungrouped(void)
{
    stereo_clear();
    lls_stop();
    ESP_LOGW(TAG, "stereo pair: ungrouped by the partner");
}

void jbl_v5_stereo_partner_channel(uint8_t ch)
{
    cfg.party_channel = ch;
    stereo_linked(true);
}

static volatile bool s_stereo_suspended;        /* Auracast party on: the pair waits (Xtreme 5) */

void jbl_v5_stereo_resume(void)
{
    static const uint8_t zero[6] = { 0 };
    if (s_stereo_suspended) return;
    if ((cfg.stereo_role != 1 && cfg.stereo_role != 2) || memcmp(cfg.partner, zero, 6) == 0 || lls_linked()) return;
    lls_group_t g = { .channel = cfg.party_channel };
    memcpy(g.gid, cfg.stereo_id, 4);
    snprintf(g.name, sizeof(g.name), "%s", cfg.stereo_name);
    cfg.lls = 0;
    ESP_LOGW(TAG, "stereo pair: resuming the stored pair (%s, channel %s, partner %02x:%02x:%02x:%02x:%02x:%02x)",
             cfg.stereo_role == 2 ? "PRIMARY" : "SECONDARY", cfg.party_channel == 2 ? "RIGHT" : "LEFT",
             cfg.partner[0], cfg.partner[1], cfg.partner[2], cfg.partner[3], cfg.partner[4], cfg.partner[5]);
    lls_start(cfg.stereo_role, cfg.partner, &g, LLS_FOREVER, stereo_linked);
}

/* ---- Auracast party while paired: the Xtreme 5 drops the pair (0x13) and makes it again after ---- */
static btstack_context_callback_registration_t s_susp_reg;
static volatile bool s_susp_want;

static void suspend_on_bt(void *ctx)
{
    (void)ctx;
    bool on = s_susp_want;
    if (on == s_stereo_suspended) return;
    if (on) {
        if (cfg.stereo_role == 0) return;
        s_stereo_suspended = true;
        ESP_LOGW(TAG, "stereo pair: on hold for the Auracast party (link closed, comes back after)");
        lls_stop();
        cfg.lls = 0;
        stereo_linked(false);
    } else {
        s_stereo_suspended = false;
        ESP_LOGW(TAG, "stereo pair: Auracast party over - making the pair again");
        jbl_v5_stereo_resume();
    }
}

bool jbl_v5_stereo_suspend(bool on)
{
    if (on && cfg.stereo_role == 0) return false;
    if (!on && !s_stereo_suspended) return false;
    s_susp_want = on;
    s_susp_reg.callback = suspend_on_bt;
    btstack_run_loop_execute_on_main_thread(&s_susp_reg);
    return true;
}

bool jbl_v5_stereo_suspended(void) { return s_stereo_suspended; }

/* ---- stereo pair: lights (AA 43), tone setting (AA 60), EQ (AA 93) -------------------------
 * re/fw_jbl/REPORT_X5_STEREO_BEHAVIOR.md. App 0x0D40 theme values -> the X5's local theme ids
 * (fw table 0x115E0F4) -> AA 43 wire codes (0x115E0D4). */
static bool light_custom(void)                    /* a custom colour is set (0x0D80, not FF FF FF / 0) */
{
    const uint8_t *c = cfg.rgb_loop + 1;
    return !(c[0] == 0xFF && c[1] == 0xFF && c[2] == 0xFF) && !(c[0] == 0 && c[1] == 0 && c[2] == 0);
}

static void eq_report_later(hci_con_handle_t con);

static const uint8_t s_theme_app[6] = { 0x81, 0x88, 0x89, 0x8A, 0x8B, 0x8C };
static const uint8_t s_theme_local[6] = { 36, 32, 31, 33, 34, 35 };
static const uint8_t s_theme_wire[6] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06 };

void jbl_v5_light_record(uint8_t r[15])
{
    int t = 0;
    for (int i = 0; i < 6; i++) if (s_theme_app[i] == cfg.light_show) t = i;
    memset(r, 0, 15);
    r[0] = s_theme_wire[t];
    r[4] = (cfg.light_bright & 0x80) ? 1 : 0;              /* light show on (0x0D41 b7) */
    r[6] = cfg.light_bright & 0x7F;                       /* brightness */
    if (light_custom()) memcpy(r + 8, cfg.rgb_loop + 1, 3);
    else memset(r + 8, 0xFF, 3);
    r[12] = s_theme_local[t];
    r[13] = 0xB9;                                         /* "from a primary" */
    r[14] = light_custom() ? 1 : 0;
}

void jbl_v5_light_from_partner(const uint8_t r[15])
{
    uint8_t show = cfg.light_show, bright = cfg.light_bright;
    for (int i = 0; i < 6; i++) if (s_theme_local[i] == r[12]) show = s_theme_app[i];
    bright = (uint8_t)((r[4] ? 0x80 : 0x00) | (r[6] & 0x7F));
    bool col = false;
    if (r[13] == 0xB9 && r[14] && !(r[8] == 0 && r[9] == 0 && r[10] == 0)) {
        col = memcmp(cfg.rgb_loop + 1, r + 8, 3) != 0;
        memcpy(cfg.rgb_loop + 1, r + 8, 3);
        cfg.rgb_loop[0] = 0x01;
    }
    bool chg = show != cfg.light_show || bright != cfg.light_bright || col;
    if (!chg) return;
    ESP_LOGW(TAG, "stereo pair: the primary's light: theme 0x%02x, %s, brightness %u%s", show, r[4] ? "on" : "off",
             r[6] & 0x7F, col ? ", custom colour" : "");
    if (show != cfg.light_show) { cfg.light_show = show; jbl_v5_notify(0x0D40); }
    if (bright != cfg.light_bright) { cfg.light_bright = bright; jbl_v5_notify(0x0D41); }
    if (col) jbl_v5_notify(0x0D80);
    persist_later(false);
}

void jbl_v5_feedback_tone_from_partner(bool on)
{
    uint8_t v = (uint8_t)(on ? (cfg.feedback_tone | 0x80) : (cfg.feedback_tone & 0x7F));
    if (v == cfg.feedback_tone) return;
    cfg.feedback_tone = v;
    ESP_LOGW(TAG, "stereo pair: prompt tones %s (the primary's setting)", on ? "on" : "off");
    jbl_v5_notify(0x0C40);
    persist_later(false);
}

/* AA 93 / 92 payload (115 B): 01 <eq id> <n> n x {f32 freq, f32 gain, f32 Q, u8 type, 3 pad} */
static const uint8_t s_tws_eq_type[5] = { 0x11, 0x0B, 0x10, 0x06, 0x02 };   /* app type 0..4 (fw 0x104f458) */

void jbl_v5_eq_tws_build(uint8_t p[115])
{
    if (s_eqc.ver != EQC_VER) eqc_standard(0x80, NULL);
    memset(p, 0, 115);
    uint8_t nb = s_eqc.count > V5_EQ_BANDS ? V5_EQ_BANDS : s_eqc.count;
    p[0] = 0x01;
    p[1] = s_eqc.cat;
    p[2] = nb;
    for (int i = 0; i < nb; i++) {
        uint8_t *b = p + 3 + 16 * i;
        memcpy(b, &s_eqc.b[i].f, 4);
        memcpy(b + 4, &s_eqc.b[i].g, 4);
        memcpy(b + 8, &s_eqc.b[i].q, 4);
        b[12] = s_tws_eq_type[s_eqc.b[i].type <= 4 ? s_eqc.b[i].type : 0];
    }
}

void jbl_v5_eq_tws_apply(const uint8_t *p, uint16_t n)
{
    if (n < 3 || p[0] != 0x01) return;
    uint8_t nb = p[2];
    if (nb > V5_EQ_BANDS || n < 3u + 16u * nb) return;
    eq_canon_t c = s_eqc;
    c.ver = EQC_VER;
    c.cat = p[1];
    c.count = nb;
    for (int i = 0; i < nb; i++) {
        const uint8_t *b = p + 3 + 16 * i;
        float f, g, q;
        memcpy(&f, b, 4);
        memcpy(&g, b + 4, 4);
        memcpy(&q, b + 8, 4);
        uint8_t t = 0;
        for (uint8_t k = 0; k < 5; k++) if (s_tws_eq_type[k] == b[12]) t = k;
        if (!(f >= 20.0f && f <= 20000.0f) || !(g > -30.0f && g < 30.0f) || !(q > 0.05f && q < 20.0f)) {
            ESP_LOGW(TAG, "stereo pair: the primary's EQ band %d out of range - ignored", i);
            return;
        }
        c.b[i].type = t;
        c.b[i].f = f;
        c.b[i].g = g;
        c.b[i].q = q;
    }
    s_eqc = c;
    eqc_apply();
    eqc_to_portable();
    persist_later(true);
    eq_report_later(s_cur_con);
    ESP_LOGW(TAG, "stereo pair: EQ from the primary: category 0x%02x, %u bands", c.cat, nb);
}

/* power-up: call the phone back first, then make the pair - paging the phone next to the
 * pair's broadcast failed (page timeout) or took 11 s; alone it takes ~1.5 s */
static btstack_timer_source_t s_boot_resume_timer;
static uint32_t s_boot_resume_t0;

static void boot_resume_cb(btstack_timer_source_t *ts)
{
    (void)ts;
    uint32_t now = btstack_run_loop_get_time_ms();
    if (a2dp_in_callback_busy() && now - s_boot_resume_t0 < 8000) {
        btstack_run_loop_set_timer(&s_boot_resume_timer, 200);
        btstack_run_loop_add_timer(&s_boot_resume_timer);
        return;
    }
    ESP_LOGW(TAG, "stereo pair: phone call-back %s after %lu ms - pair now", a2dp_in_callback_busy() ? "still busy" : "done",
             (unsigned long)(now - s_boot_resume_t0));
    jbl_v5_stereo_resume();
}

void jbl_v5_stereo_resume_at_boot(void)
{
    if (cfg.stereo_role != 2) {                   /* secondary: it drops its phone once linked anyway */
        jbl_v5_stereo_resume();
        return;
    }
    s_boot_resume_t0 = btstack_run_loop_get_time_ms();
    btstack_run_loop_remove_timer(&s_boot_resume_timer);
    btstack_run_loop_set_timer_handler(&s_boot_resume_timer, boot_resume_cb);
    btstack_run_loop_set_timer(&s_boot_resume_timer, 500);   /* the call-back is scheduled 300 ms after HCI */
    btstack_run_loop_add_timer(&s_boot_resume_timer);
}

void jbl_v5_stereo_ungroup_local(void)
{
    if (cfg.stereo_role == 0) return;
    lls_ungroup();
    stereo_clear();
    ESP_LOGW(TAG, "stereo pair: ungrouped (the secondary's ungroup key)");
    jbl_v5_notify(0x2C80);
}

static void stereo_wait_end(btstack_timer_source_t *ts)
{
    (void)ts;
    if (cfg.lls) return;                        /* linked: stereo_linked owns the beacon now */
    jbl_set_stereo_wait(0, 0);
    ESP_LOGW(TAG, "stereo pair: setup window over, beacon back to normal");
}

void jbl_v5_notify(uint16_t id)
{
    if (s_linked) {
        notify_features(s_link, &id, 1);
    }
}

void jbl_v5_on_party_changed(void)
{
    static const uint16_t ids[] = { 0x2004, 0x0B86, 0x0B87, 0x2C02, 0x2C03, 0x0BA1, 0x2D40, 0x2D42 };
    if (s_linked) {
        notify_features(s_link, ids, (int)(sizeof(ids) / sizeof(ids[0])));
    }
}

/* ---- OTA capture ------------------------------------------------------------------
 * JBL One's portable OTA task: 0x0101 start {u32 size}, 0x0102 data {u32 offset,
 * <= 500 B}, 0x0103 stop; each answered (same cmd) by a >= 12 B notification
 * {u16 status, u16 percent 0..10000, u32 next offset, u32 length to read}.
 * Done when percent reaches 10000 / length to read is 0. No install command
 * follows: the app waits 40 s, reconnects and expects 000C == its target. */

#define OTA_CHUNK      500
#define OTA_PART_LABEL "jblota"
#define OTA_HDR_SIZE   0x1000            /* header sector, image follows */

typedef struct {
    char magic[8];                       /* "JBLOTA01" */
    uint32_t size;
    uint8_t md5[16];
    uint8_t version[3];                  /* parsed from the image's ota.xml, 0 = unknown */
    uint8_t reported_before[3];          /* what we told the app before the update */
    char version_name[64];
} ota_hdr_t;

static struct {
    uint8_t *img;
    uint32_t size, next, t0_ms;
    uint8_t last_decile;
    bool active, storing;
    uint32_t chunks, resends;
} s_ota;

static void ota_reply(hci_con_handle_t con, uint16_t cmd, uint16_t status)
{
    uint8_t p[12];
    uint32_t next = s_ota.next, size = s_ota.size;
    uint16_t pct = size ? (uint16_t)((uint64_t)next * 10000u / size) : 0;
    uint32_t left = size > next ? size - next : 0;
    uint32_t ltr = left > OTA_CHUNK ? OTA_CHUNK : left;
    if (status != 0) {
        pct = 0;
        ltr = 0;
    }
    p[0] = status & 0xFF; p[1] = status >> 8;
    p[2] = pct & 0xFF; p[3] = pct >> 8;
    memcpy(p + 4, &next, 4);
    memcpy(p + 8, &ltr, 4);
    send_frame(con, cmd, p, sizeof(p));
}

/* "JBL_Xtreme5_v26.40.10_2609281042" in ota.xml. The raw OTA file carries a CRC16
 * after every 32 data bytes, so strip that framing before searching. */
static bool ota_parse_version(const uint8_t *img, uint32_t size, uint8_t ver[3], char *name, size_t name_max)
{
    static const char key[] = "<version_name>";
    uint32_t scan = size < 0x1100 ? size : 0x1100;
    uint8_t *d = heap_caps_malloc(scan, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (d == NULL) return false;
    bool found = false;
    for (int pass = 0; pass < 2 && !found; pass++) {
        uint32_t n = 0;
        if (pass == 0) {                                  /* de-framed */
            for (uint32_t i = 0; i < scan; i += 34) {
                uint32_t k = scan - i < 32 ? scan - i : 32;
                memcpy(d + n, img + i, k);
                n += k;
            }
        } else {                                          /* as is */
            memcpy(d, img, scan);
            n = scan;
        }
        for (uint32_t i = 0; i + sizeof(key) < n && !found; i++) {
            if (memcmp(d + i, key, sizeof(key) - 1) != 0) continue;
            uint32_t s = i + sizeof(key) - 1, e = s;
            while (e < n && e - s < name_max - 1 && d[e] != '<') e++;
            memcpy(name, d + s, e - s);
            name[e - s] = 0;
            const char *v = strstr(name, "_v");
            unsigned a, b, c;
            if (v && sscanf(v + 2, "%u.%u.%u", &a, &b, &c) == 3 && a < 256 && b < 256 && c < 256) {
                ver[0] = (uint8_t)a; ver[1] = (uint8_t)b; ver[2] = (uint8_t)c;
                found = true;
            }
        }
    }
    heap_caps_free(d);
    return found;
}

static void ota_store_task(void *arg)
{
    (void)arg;
    ota_hdr_t hdr;
    memset(&hdr, 0, sizeof(hdr));
    memcpy(hdr.magic, "JBLOTA01", 8);
    hdr.size = s_ota.size;
    memcpy(hdr.reported_before, cfg.fw, 3);
    md5_context_t md5;
    esp_rom_md5_init(&md5);
    esp_rom_md5_update(&md5, s_ota.img, s_ota.size);
    esp_rom_md5_final(hdr.md5, &md5);
    bool have_ver = ota_parse_version(s_ota.img, s_ota.size, hdr.version, hdr.version_name, sizeof(hdr.version_name));
    char md5s[33];
    for (int i = 0; i < 16; i++) snprintf(md5s + 2 * i, 3, "%02x", hdr.md5[i]);
    ESP_LOGW(TAG, "OTA image: %lu B, MD5 %s, version %s (%s)", (unsigned long)s_ota.size, md5s,
             have_ver ? "found" : "NOT found", have_ver ? hdr.version_name : "-");

    const esp_partition_t *p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY,
                                                        OTA_PART_LABEL);
    esp_err_t e = ESP_ERR_NOT_FOUND;
    if (p && OTA_HDR_SIZE + s_ota.size <= p->size) {
        uint32_t total = (OTA_HDR_SIZE + s_ota.size + 0xFFFu) & ~0xFFFu;
        uint8_t *buf = heap_caps_malloc(4096, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        e = buf ? ESP_OK : ESP_ERR_NO_MEM;
        /* sector by sector with a yield in between: audio and Bluetooth keep running */
        for (uint32_t o = 0; e == ESP_OK && o < total; o += 4096) {
            e = esp_partition_erase_range(p, o, 4096);
            if (e != ESP_OK) break;
            memset(buf, 0xFF, 4096);
            if (o == 0) {
                memcpy(buf, &hdr, sizeof(hdr));
            } else {
                uint32_t io = o - OTA_HDR_SIZE, k = s_ota.size - io < 4096 ? s_ota.size - io : 4096;
                memcpy(buf, s_ota.img + io, k);
            }
            e = esp_partition_write(p, o, buf, 4096);
            vTaskDelay(1);
        }
        heap_caps_free(buf);
    }
    if (e == ESP_OK) {
        ESP_LOGW(TAG, "OTA image stored in flash partition '%s' at 0x%lx (header) / 0x%lx (image, %lu B)",
                 OTA_PART_LABEL, (unsigned long)p->address, (unsigned long)(p->address + OTA_HDR_SIZE),
                 (unsigned long)s_ota.size);
    } else {
        ESP_LOGE(TAG, "OTA image NOT stored: %s", esp_err_to_name(e));
    }
    heap_caps_free(s_ota.img);
    s_ota.img = NULL;
    if (have_ver) {
        /* JBL One verifies 000C == its target after the "install": take the image's version */
        memcpy(cfg.fw, hdr.version, 3);
        nvs_handle_t h;                                   /* this task, not the BT thread: write directly */
        if (nvs_open("jbl_v5", NVS_READWRITE, &h) == ESP_OK) {
            v5_cfg_t c = cfg;
            if (nvs_set_blob(h, "cfg", &c, sizeof(c)) == ESP_OK) nvs_commit(h);
            nvs_close(h);
        }
        ESP_LOGW(TAG, "firmware version now reported as %u.%u.%u (was %u.%u.%u)", cfg.fw[0], cfg.fw[1], cfg.fw[2],
                 hdr.reported_before[0], hdr.reported_before[1], hdr.reported_before[2]);
    }
    s_ota.storing = false;
    vTaskDelete(NULL);
}

static void ota_abort(const char *why)
{
    if (s_ota.active) {
        ESP_LOGW(TAG, "OTA %s at %lu / %lu B", why, (unsigned long)s_ota.next, (unsigned long)s_ota.size);
    }
    if (!s_ota.storing) {
        heap_caps_free(s_ota.img);
        s_ota.img = NULL;
    }
    s_ota.active = false;
}

static void handle_ota(hci_con_handle_t con, uint16_t cmd, const uint8_t *pl, uint16_t len)
{
    switch (cmd) {
    case 0x0101: {                                        /* start */
        uint32_t size = 0;
        if (len >= 4) memcpy(&size, pl, 4);
        ota_abort("restarted");
        const esp_partition_t *p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY,
                                                            OTA_PART_LABEL);
        s_ota.size = size;
        s_ota.next = 0;
        if (s_ota.storing || size == 0 || p == NULL || OTA_HDR_SIZE + size > p->size) {
            tones_play("c_err");
            ESP_LOGE(TAG, "OTA start refused: %lu B (%s)", (unsigned long)size,
                     s_ota.storing ? "still storing the last image" : p ? "too big" : "no 'jblota' partition");
            ota_reply(con, cmd, 0x8001);
            return;
        }
        s_ota.img = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_ota.img == NULL) {
            ESP_LOGE(TAG, "OTA start refused: no PSRAM for %lu B", (unsigned long)size);
            ota_reply(con, cmd, 0x8002);
            return;
        }
        s_ota.active = true;
        s_ota.t0_ms = btstack_run_loop_get_time_ms();
        s_ota.last_decile = 0;
        s_ota.chunks = s_ota.resends = 0;
        ESP_LOGW(TAG, "OTA start: %lu B image from JBL One (MTU %u) - capturing", (unsigned long)size,
                 jbl_gatt_mtu(con));
        ota_reply(con, cmd, 0);
        return;
    }
    case 0x0102: {                                        /* data */
        uint32_t off = 0;
        if (!s_ota.active || len < 4) {
            ota_reply(con, cmd, 0x8003);
            return;
        }
        memcpy(&off, pl, 4);
        uint32_t n = len - 4u;
        if (off != s_ota.next || n == 0 || off + n > s_ota.size) {
            s_ota.resends++;                              /* ask again from where we are */
            ESP_LOGW(TAG, "OTA chunk at %lu (%lu B), expected %lu - asking for a resend", (unsigned long)off,
                     (unsigned long)n, (unsigned long)s_ota.next);
            ota_reply(con, cmd, 0);
            return;
        }
        memcpy(s_ota.img + off, pl + 4, n);
        s_ota.next += n;
        s_ota.chunks++;
        uint8_t dec = (uint8_t)((uint64_t)s_ota.next * 10u / s_ota.size);
        if (dec != s_ota.last_decile) {
            s_ota.last_decile = dec;
            uint32_t ms = btstack_run_loop_get_time_ms() - s_ota.t0_ms;
            ESP_LOGW(TAG, "OTA %u0 %% (%lu / %lu B, %lu kB/s, resends %lu)", dec, (unsigned long)s_ota.next,
                     (unsigned long)s_ota.size, (unsigned long)(ms ? s_ota.next / ms : 0),
                     (unsigned long)s_ota.resends);
        }
        ota_reply(con, cmd, 0);
        if (s_ota.next == s_ota.size) {
            s_ota.active = false;
            s_ota.storing = true;
            ESP_LOGW(TAG, "OTA transfer complete (%lu chunks, %lu s) - storing", (unsigned long)s_ota.chunks,
                     (unsigned long)((btstack_run_loop_get_time_ms() - s_ota.t0_ms) / 1000));
            if (xTaskCreatePinnedToCore(ota_store_task, "ota_store", 4096, NULL, 2, NULL, 0) != pdPASS) {
                ESP_LOGE(TAG, "OTA store task not started");
                s_ota.storing = false;
                ota_abort("not stored");
            }
        }
        return;
    }
    case 0x0103:                                          /* stop (user left the OTA screen) */
        if (s_ota.active) ota_abort("stopped by the app");
        ota_reply(con, cmd, 0);
        return;
    default:                                              /* 0x0104 query */
        ota_reply(con, cmd, 0);
        return;
    }
}

/* ---- commands ---------------------------------------------------------------- */

static void factory_reset_cb(btstack_timer_source_t *ts)
{
    (void)ts;
    ESP_LOGW(TAG, "FACTORY RESET: erasing NVS (pairings, EQ, settings, volume) and restarting");
    nvs_flash_erase();
    esp_restart();
}

static void handle_get(hci_con_handle_t con, const uint8_t *pl, uint16_t len)
{
    static uint8_t out[V5_MAX_PAYLOAD];
    uint8_t v[V5_VAL_MAX];
    uint8_t miss[64];
    uint16_t off = 2, nmiss = 0;
    char dbg[200], mdbg[120];
    int dl = 0, ml = 0;
    dbg[0] = mdbg[0] = 0;
    out[0] = 0; out[1] = 0;                           /* status OK */
    s_cur_con = con;
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
    /* IDs we lack ride in the SAME reply as 0x0000 UnsupportedFeatureList: a second
     * cmd-1 frame would complete the app's NEXT Get (replies match by cmd id) */
    if (nmiss && off + 4 + nmiss <= V5_MAX_PAYLOAD) {
        off = put_tlv(out, off, 0x0000, miss, nmiss);
    }
    ESP_LOGI(TAG, "app Get [%s] -> %u B%s%s", dbg, off, nmiss ? " | UNSUPPORTED: " : "", mdbg);
    send_frame(con, 1, out, off);
}

static void handle_set(hci_con_handle_t con, const uint8_t *pl, uint16_t len);

void jbl_v5_debug_set(uint16_t id, const uint8_t *v, uint16_t n)
{
    uint8_t pl[4 + 16];
    if (n > 16) n = 16;
    pl[0] = (uint8_t)id; pl[1] = (uint8_t)(id >> 8); pl[2] = (uint8_t)n; pl[3] = 0;
    memcpy(pl + 4, v, n);
    hci_con_handle_t keep = s_cur_con;
    handle_set(keep, pl, (uint16_t)(4 + n));      /* the reply goes to the app link if one is up */
}


/* After an EQ change the speaker reports its EQ back (EqInfo, then the portable form), as
 * the app's EqInfoQuery would get it. Without it the app's idea of the device EQ stayed at
 * what it read on connect: switching Vocal -> JBL Signature sent nothing (the app thought
 * Signature was still set) and Vocal stayed on. 300 ms after the last Set, so a slider drag
 * (a Set every ~100 ms) is not answered mid-drag. */
static btstack_timer_source_t s_eq_report_timer;
static hci_con_handle_t s_eq_report_con = HCI_CON_HANDLE_INVALID;

static void eq_report_cb(btstack_timer_source_t *t)
{
    (void)t;
    hci_con_handle_t con = s_eq_report_con;
    s_eq_report_con = HCI_CON_HANDLE_INVALID;
    if (con == HCI_CON_HANDLE_INVALID || gap_get_connection_type(con) == GAP_CONNECTION_INVALID) return;
    hci_con_handle_t prev = s_cur_con;
    s_cur_con = con;                                   /* header order as that link uses */
    notify_features(con, (const uint16_t[]){ 0x0E7E }, 1);    /* EqInfo last (see the EqInfoQuery reply) */
    notify_features(con, (const uint16_t[]){ 0x0E02 }, 1);
    s_cur_con = prev;
}

static void eq_report_later(hci_con_handle_t con)
{
    s_eq_report_con = con;
    btstack_run_loop_remove_timer(&s_eq_report_timer);
    btstack_run_loop_set_timer_handler(&s_eq_report_timer, eq_report_cb);
    btstack_run_loop_set_timer(&s_eq_report_timer, 300);
    btstack_run_loop_add_timer(&s_eq_report_timer);
}

static void handle_set(hci_con_handle_t con, const uint8_t *pl, uint16_t len)
{
    s_cur_con = con;
    static const uint8_t ok[2] = { 0, 0 };
    /* JBL One sends one command at a time and waits for this reply: send it before the
     * work, so a slider drag (an EQ Set every few connection events) is not slowed down */
    send_frame(con, 2, ok, sizeof(ok));               /* every Set gets "00 00" */
    bool eq_query = false, eq_changed = false, cfg_changed = false, renamed = false, reset = false;
    uint16_t post[12];                                /* notify these after the "00 00" reply */
    int npost = 0;
#define POST(x) do { if (npost < (int)(sizeof(post) / sizeof(post[0]))) post[npost++] = (x); } while (0)
    bool assist_join = false, assist_leave = false, assist_has_code = false;
    uint8_t assist_type = 0;
    uint8_t assist_code[16];
    bd_addr_t assist_addr;
    uint8_t std_bcast = 0;                            /* 1 on, 2 off, 3 restart with new settings */

    for (uint16_t i = 0; i + 4 <= len;) {
        uint16_t id = pl[i] | (pl[i + 1] << 8);
        uint16_t n = pl[i + 2] | (pl[i + 3] << 8);
        const uint8_t *v = pl + i + 4;
        if ((uint32_t)i + 4 + n > len) {
            break;
        }
        bool c = true;                                /* a persisted setting changed */
        switch (id) {
        case 0x0E01: eq_query = true; c = false; break;                       /* EqInfoQuery */
        case 0x0E02:                                                          /* EqInfo, band form */
            c = false;
            if (eq_info_apply(v, n)) {
                eq_changed = true;
                s_eq_sets++;
            }
            break;
        case 0x0E7F:                                                          /* EqSetPortable */
            c = false;
            if (n >= 1) {
                s_eq[0] = v[0];
                uint16_t coef_off = 1;
                if (v[0] == 0xC1 || v[0] == 0x00) {                           /* custom: 7 indices */
                    if (n >= 8) {
                        memcpy(s_eq + 1, v + 1, 7);
                    }
                    coef_off = 8;
                } else {
                    memset(s_eq + 1, 0x0C, 7);
                }
                if (n >= coef_off + V5_EQ_COEF_BYTES) {
                    v5_eq_set_q29(v + coef_off);
                    memcpy(s_eq_blob.coef, v + coef_off, V5_EQ_COEF_BYTES);
                    s_eq_blob.has_coef = 1;
                }
                eq_changed = true;
                s_eq_sets++;
                eqc_from_portable();
                if (eq_log_ok()) {
                    ESP_LOGW(TAG, "EQ set: category 0x%02x, bands %d %d %d %d %d %d %d (%s)",
                             s_eq[0], s_eq[1], s_eq[2], s_eq[3], s_eq[4], s_eq[5], s_eq[6], s_eq[7],
                             v5_eq_is_on() ? "applied to audio" : "flat");
                }
            }
            break;
        /* settings */
        case 0x0006:                                                          /* rename */
            if (n > 0) {
                size_t k = n > V5_NAME_MAX ? V5_NAME_MAX : n;
                memcpy(cfg.name, v, k);
                cfg.name[k] = 0;
                renamed = true;
            }
            break;
        case 0x0E80: if (n) cfg.smart_eq = v[0]; break;
        case 0x0C40: case 0x200F:
            if (n) {
                cfg.feedback_tone = v[0];
                if (!(v[0] & 0x80)) tones_stop();
                if (cfg.lls && cfg.stereo_role == 2) lls_send_tone_setting();
            }
            break;
        case 0x0012:
            if (n >= 2) {
                cfg.auto_off = v[0] | (v[1] << 8);
                s_idle_s = 0;
                ESP_LOGW(TAG, "auto power off %s%u s", (cfg.auto_off & 0x8000) ? "after " : "OFF (",
                         cfg.auto_off & 0x7FFF);
                POST(0x0012);
            }
            break;
        case 0x0013: if (n >= 2) cfg.auto_standby = v[0] | (v[1] << 8); break;
        case 0x0056: if (n) cfg.remote_power = v[0]; break;
        case 0x0BA0:
            if (n) {
                bool was = (cfg.le_audio & 0x80) != 0, on = (v[0] & 0x80) != 0;
                cfg.le_audio = v[0];
                ESP_LOGW(TAG, "app LE Audio %s", on ? "ON" : "OFF");
                le_audio_set_enabled(on);
                POST(0x0BA0);
                if (on != was) {
                    /* as the Xtreme 5: LE Audio switched on or off -> pairing mode, so the
                     * phone can pair again with the new bond set (CTKD on, Classic only off) */
                    a2dp_in_button(A2DP_BTN_PAIRING_SWITCH);
                }
            }
            break;
        case 0x2009: if (n) cfg.bass_boost = v[0]; break;
        case 0x2006:
            if (n) {
                cfg.playtime_boost = v[0];
                if (s_eqc.ver != EQC_VER) eqc_standard(s_eq[0] ? s_eq[0] : 0x80, NULL);
                eqc_apply();
                ESP_LOGW(TAG, "Playtime Boost %s (bass %s)", v[0] ? "ON" : "off", v[0] ? "-6 dB" : "as the EQ");
            }
            break;
        case 0x2007: if (n) cfg.mood = v[0]; break;
        case 0x200B: if (n) cfg.vocal_removal = v[0]; break;
        case 0x30C2: if (n >= 4) memcpy(&cfg.pitch, v, 4); break;
        case 0x0D00: if (n && v[0] != 0x1F) cfg.light_status = v[0] & 0xCF; POST(0x0D00); lls_light_changed(); break;
        case 0x0D40: if (n) cfg.light_show = v[0]; POST(0x0D40); lls_light_changed(); break;
        case 0x0D41: if (n) cfg.light_bright = v[0]; POST(0x0D41); lls_light_changed(); break;
        case 0x0D42: if (n) cfg.light_temp = v[0]; POST(0x0D42); break;
        case 0x0D43: if (n) cfg.light_speed = v[0]; POST(0x0D43); break;
        case 0x0D45: if (n) cfg.bass_light = v[0]; POST(0x0D45); break;
        case 0x0D46: if (n && v[0]) cfg.light_preset = v[0]; POST(0x0D46); break;
        case 0x0D47: if (n >= 3) memcpy(cfg.light_rgb, v, 3); POST(0x0D47); break;
        case 0x0D80: if (n >= 4) memcpy(cfg.rgb_loop, v, 4); POST(0x0D80); lls_light_changed(); break;
        case 0x2C03: if (n) cfg.stereo_role = v[0]; break;                   /* stereo: our role */
        case 0x2C04:                                                          /* 1 left, 2 right */
            if (n) {
                cfg.party_channel = v[0];
                if (cfg.lls) {
                    stereo_linked(true);                /* paired: new channel now (output, beacon) */
                    if (cfg.stereo_role == 2) lls_send_channel(v[0]);   /* the partner takes the other */
                }
            }
            break;
        case 0x2C05: { size_t k = n > 23 ? 23 : n; memcpy(cfg.stereo_name, v, k); cfg.stereo_name[k] = 0; break; }
        case 0x2C06: { size_t k = n > 8 ? 8 : n; memcpy(cfg.stereo_id, v, k); cfg.stereo_id[k] = 0; break; }
        case 0x2C81:                                                          /* stereo partner: link up */
            if (n >= 6) {
                memcpy(cfg.partner, v, 6);
                ESP_LOGW(TAG, "stereo pair: partner %02x:%02x:%02x:%02x:%02x:%02x, role %u, channel %s, group '%s' %s",
                         v[0], v[1], v[2], v[3], v[4], v[5], cfg.stereo_role,
                         cfg.party_channel == 1 ? "LEFT" : cfg.party_channel == 2 ? "RIGHT" : "?", cfg.stereo_name,
                         cfg.stereo_id);
                tones_play("indicate");
            }
            break;
        case 0x2C07:                                                          /* 1 create, 0 ungroup */
            if (n && v[0] == 0) {
                lls_ungroup();                          /* tell the partner first (AA 13, group id 0) */
                stereo_clear();                         /* beacon, channel and broadcast hold back to normal */
                ESP_LOGW(TAG, "stereo pair: ungrouped");
                POST(0x2C80);
            } else if (n && v[0] == 1) {
                /* The app waits up to 15 s for 0x2C80 = 1 (the speaker-to-speaker stereo link).
                 * lls.c looks for the partner link (transport still being worked out);
                 * 0x2C80 = 1 only once that link is really up. */
                cfg.lls = 0;
                {
                    lls_group_t g = { .channel = cfg.party_channel };
                    memcpy(g.gid, cfg.stereo_id, 4);            /* the speaker keeps the first 4 chars */
                    snprintf(g.name, sizeof(g.name), "%s", cfg.stereo_name);
                    lls_start(cfg.stereo_role == 2 ? 2 : 1, cfg.partner, &g, 20, stereo_linked);
                }
                {   /* advertise as a real speaker waiting for its partner: its channel + stereoGroupId
                     * (the real one sent 0x00c0 / 0x0140 and the 4th char of its 4-char group id) */
                    uint16_t pi = (uint16_t)(((cfg.party_channel & 3u) << 7) | 0x40u);
                    uint8_t gid = (uint8_t)cfg.stereo_id[3];
                    jbl_set_stereo_wait(pi, gid);
                    ESP_LOGW(TAG, "stereo pair: beacon partyInfo 0x%04x, stereoGroupId 0x%02x for 30 s", pi, gid);
                    static btstack_timer_source_t t;
                    btstack_run_loop_remove_timer(&t);
                    btstack_run_loop_set_timer_handler(&t, stereo_wait_end);
                    btstack_run_loop_set_timer(&t, 30000);
                    btstack_run_loop_add_timer(&t);
                }
                ESP_LOGW(TAG, "stereo pair: create (%s, channel %s) - making the LE partner link",
                         cfg.stereo_role == 2 ? "PRIMARY" : "SECONDARY",
                         cfg.party_channel == 1 ? "LEFT" : cfg.party_channel == 2 ? "RIGHT" : "?");
            }
            break;
        /* live controls */
        case 0x2D42:                                                          /* volume 0..100 */
            c = false;
            if (n) {
                if (party_listening()) {
                    party_listen_volume_set(to127(v[0]));
                } else {
                    a2dp_in_set_volume(to127(v[0]), "JBL One");
                }
            }
            break;
        case 0x2D41: c = false; if (n) a2dp_in_set_mute(v[0] != 0); break;   /* mute */
        case 0x2D40:                                                          /* play/pause/skip/volume */
            c = false;
            if (n) {
                ESP_LOGW(TAG, "app AudioOperation %u -> %s", v[0], a2dp_in_media_op(v[0]) ? "sent" : "no AVRCP");
            }
            break;
        case 0x2004:                                                          /* AuracastMode: JBL party */
            c = false;
            if (n) {
                ESP_LOGW(TAG, "app turns the JBL party %s (0x2004 = 0x%02x)", v[0] ? "ON" : "OFF", v[0]);
                party_set_auracast_enabled(v[0] != 0);
                POST(0x2004);
                POST(0x0B86);
            }
            break;
        case 0x0B86:                                                          /* Broadcast to Others: standard */
            c = false;
            if (n) {
                bool on = (v[0] & 0x80) != 0;
                ESP_LOGW(TAG, "app turns Broadcast to Others %s (standard Auracast)", on ? "ON" : "OFF");
                std_bcast = on ? 1 : 2;
                POST(0x0B86);
                POST(0x2004);
            }
            break;
        case 0x0B87:                                                          /* broadcast quality, not on/off */
            if (n) {
                cfg.bcast_hq = v[0] & 0x80;
                ESP_LOGW(TAG, "app broadcast quality: %s", cfg.bcast_hq ? "HIGH" : "standard");
            }
            break;
        case 0x0B8A: {
            size_t k = n > 32 ? 32 : n;
            if (k) { memcpy(cfg.bcast_name, v, k); cfg.bcast_name[k] = 0; }
            if (party_standard_broadcast_on() && !std_bcast) std_bcast = 3;   /* restart with the new name */
            break;
        }
        case 0x0B88:
            c = false;
            if (pw_is_none(v, n)) {
                s_bcast_pw[0] = 0;
            } else {
                uint16_t k = n > 16 ? 16 : n;
                memcpy(s_bcast_pw, v, k);
                s_bcast_pw[k] = 0;
                pw_to_code(v, k, s_bcast_code);
            }
            ESP_LOGW(TAG, "app broadcast password: %s", s_bcast_pw[0] ? "set" : "none (open broadcast)");
            if (party_standard_broadcast_on() && !std_bcast) std_bcast = 3;
            break;
        case 0x0B81:                                                          /* assistant scan {action, s} */
            c = false;
            if (n >= 1) party_assist_scan(v[0] != 0, n >= 2 ? v[1] : 60);
            break;
        case 0x0B85: c = false; break;                                       /* subgroup play: we play subgroup 0 */
        case 0x0B83:                                                          /* join / leave a broadcast */
            c = false;
            if (n >= 8) {                     /* {action, addr type, addr[6], status, password...} */
                assist_type = v[1];
                memcpy(assist_addr, v + 2, 6);
                if (v[0] == 0) assist_leave = true;
                else assist_join = true;                                      /* 01 open, 02 with password */
                if (v[0] == 2 && n > 9 && !pw_is_none(v + 9, (uint16_t)(n - 9))) {
                    pw_to_code(v + 9, (uint16_t)(n - 9), assist_code);
                    assist_has_code = true;
                }
            }
            break;
        case 0x000F: c = false; reset = true; break;                          /* factory reset */
        case 0x0011:                                                          /* remote power off {u16 s} */
            c = false;
            s_manual_off_s = n >= 2 ? (v[0] | (v[1] << 8)) : 0;
            ESP_LOGW(TAG, "app power-off request: in %u s", s_manual_off_s);
            if (s_manual_off_s == 0) power_soft_off("JBL One");
            break;
        case 0x0031:
            c = false;
            if (n >= 8) {
                ESP_LOGI(TAG, "app time sync %04u-%02u-%02u %02u:%02u:%02u", v[6] | (v[7] << 8), v[5], v[4],
                         v[2], v[1], v[0]);
            }
            break;
        case 0x2011:
            c = false;
            ESP_LOGW(TAG, "app identify request 0x%02x", n ? v[0] : 0);
            if (n && v[0] == 0x80) tones_stop();
            else tones_play("indicate");
            break;
        case 0x004A:
        case 0x004B:
            c = false;
            ESP_LOGW(TAG, "app sent anti-counterfeit challenge 0x%04x (%u B) - cannot sign it", id, n);
            break;
        default:
            c = false;
            break;
        }
        cfg_changed |= c;
        {
            char h[3 * 12 + 1];
            int hl = 0;
            h[0] = 0;
            for (uint16_t k = 0; k < n && k < 12; k++) hl += snprintf(h + hl, sizeof(h) - (size_t)hl, "%02x ", v[k]);
            if (id != 0x0E02 && id != 0x0E7F) {         /* EQ: its own (rate-limited) log */
                ESP_LOGI(TAG, "app Set 0x%04x (%u B): %s%s", id, n, h, n > 12 ? "..." : "");
            }
        }
        i += 4 + n;
    }
    if (eq_query) {
        /* both forms, each in its own frame - the portable one FIRST, EqInfo LAST: JBL One
         * rebuilds its preset list from whichever arrives, with that form's ids, and Xtreme
         * 5 (2169) EQ Sets go the EqInfo way. EqInfo last leaves EqInfo ids (JBL_SIGNATURE);
         * portable last left "SIGNATURE", which the EqInfo lookup does not find - tapping
         * JBL Signature then sent nothing (Vocal / Energetic / Chill have equal names). */
        notify_features(con, (const uint16_t[]){ 0x0E7E }, 1);
        notify_features(con, (const uint16_t[]){ 0x0E02 }, 1);
    }
    for (int k = 0; k < npost; k++) {
        notify_features(con, &post[k], 1);
    }
#undef POST
    if (assist_join) {
        party_assist_join(assist_addr, assist_type, assist_has_code ? assist_code : NULL);
    }
    if (std_bcast) {
        party_set_standard_broadcast(std_bcast != 2, cfg.bcast_name, s_bcast_pw[0] ? s_bcast_code : NULL);
    }
    if (assist_leave) {
        party_assist_leave();
        jbl_v5_assist_result(0, assist_addr, assist_type, 1);
    }
    if (eq_changed) {
        persist_later(true);
        eq_report_later(con);
        lls_send_eq();                                /* stereo primary: the partner plays the same EQ */
    }
    if (cfg_changed) {
        persist_later(false);
    }
    if (renamed) {
        ESP_LOGW(TAG, "renamed to '%s' (Bluetooth name, LE beacon and GATT Device Name)", cfg.name);
        gap_set_local_name(cfg.name);
        jbl_beacon_name_changed();
        notify_features(con, (const uint16_t[]){ 0x0006 }, 1);
    }
    if (reset) {
        static btstack_timer_source_t t;
        ESP_LOGW(TAG, "app requested a FACTORY RESET - in 2 s");
        tones_play("poweroff");
        btstack_run_loop_set_timer_handler(&t, factory_reset_cb);
        btstack_run_loop_set_timer(&t, 2000);
        btstack_run_loop_add_timer(&t);
    }
}

/* reassembly for multi-packet frames: cold, PSRAM, allocated on first use */
static uint8_t *s_rx;
static uint16_t s_rx_len;

void jbl_v5_on_write(hci_con_handle_t con, const uint8_t *d, uint16_t len)
{
    if (len >= 8 && (d[0] == 0x01 || d[0] == 0x02) && d[1] == 0xDD) {
        /* Forward / SubWoofer: the iOS app asks our stereo partner (PID, serial,
         * firmware) right after connecting and gives the whole link up after 30 s
         * without an answer. No partner: answer on the same route with an error
         * status - the app parses no partner info from it, and its request ends. */
        static uint32_t s_fwd;
        static const uint8_t no_partner[2] = { 0x01, 0x80 };      /* status 0x8001 */
        uint16_t fcmd = d[2] | (d[3] << 8);
        if (d[0] == 0x01 && lls_linked() && cfg.stereo_role == 2) {
            /* as a real X5 primary: Gets from the partner cache (its AA 12), no status word;
             * Sets dropped except identify (0x2011 = 0x81) */
            uint16_t plen = (uint16_t)(d[6] | (d[7] << 8));
            const uint8_t *p = d + 8;
            if (plen > len - 8) plen = len - 8;
            if (fcmd == 1 || fcmd == 3) {
                static uint8_t out[96];
                uint16_t o = 0;
                for (uint16_t i = 0; i + 1 < plen && o + 4 + 32 <= sizeof(out); i += 2) {
                    uint16_t id = (uint16_t)(p[i] | (p[i + 1] << 8));
                    uint16_t vn = 0;
                    if (!lls_partner_get(id, out + o + 4, &vn) || vn == 0) continue;
                    out[o] = id & 0xFF; out[o + 1] = id >> 8; out[o + 2] = vn & 0xFF; out[o + 3] = vn >> 8;
                    o += 4 + vn;
                }
                s_tx_ident = 0x01;
                send_frame(con, fcmd, out, o);
                s_tx_ident = 0;
            } else if (fcmd == 2) {
                for (uint32_t i = 0; i + 4 <= plen;) {
                    uint16_t id = (uint16_t)(p[i] | (p[i + 1] << 8)), vl = (uint16_t)(p[i + 2] | (p[i + 3] << 8));
                    if (id == 0x2011 && vl >= 1 && i + 4 < plen && p[i + 4] == 0x81) lls_partner_identify();
                    if (4u + vl > plen - i) break;           /* (a 16-bit index wrapped: endless loop) */
                    i += 4u + vl;
                }
            }
            return;
        }
        if (s_fwd++ < 3) {
            ESP_LOGW(TAG, "app %s cmd 0x%04x - no %s, answered with an error status", d[0] == 1 ? "Forward" : "SubWoofer",
                     fcmd, d[0] == 1 ? "stereo partner" : "subwoofer");
        }
        s_tx_ident = d[0];
        send_frame(con, fcmd, no_partner, sizeof(no_partner));
        s_tx_ident = 0;
        return;
    }
    if (len < 8 || d[0] != 0x00 || d[1] != 0xDD) {
        char h[3 * 32 + 1];
        int hl = 0;
        h[0] = 0;
        for (uint16_t k = 0; k < len && k < 32; k++) hl += snprintf(h + hl, sizeof(h) - (size_t)hl, "%02x ", d[k]);
        ESP_LOGW(TAG, "non-V5 write (%u B) on con 0x%04x: %s", len, con, h);
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
    case 0x0101: case 0x0102: case 0x0103: case 0x0104:
        handle_ota(con, cmd, pl, have);
        break;
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
    jbl_beacon_set_app_connected(true);
    ESP_LOGW(TAG, "JBL One control link up (con 0x%04x, MTU %u)", con, jbl_gatt_mtu(con));
}

bool jbl_v5_is_link(hci_con_handle_t con) { return s_linked && con == s_link; }

void jbl_v5_link_down(hci_con_handle_t con)
{
    if (s_linked && con == s_link) {
        s_linked = false;
        s_link = HCI_CON_HANDLE_INVALID;
        jbl_beacon_set_app_connected(false);
        ESP_LOGW(TAG, "JBL One control link down");
        if (s_ota.active) ota_abort("link lost");
    }
}

/* ---- auto power off: no audio (phone or party) for the app's time ------------- */
static btstack_timer_source_t s_idle_timer;

static void idle_tick(btstack_timer_source_t *ts)
{
    bool busy = a2dp_in_streaming() || party_listening() || s_ota.active || s_ota.storing;
    s_idle_s = busy ? 0 : s_idle_s + 1;
    uint32_t limit = cfg.auto_off & 0x7FFF;
    if ((cfg.auto_off & 0x8000) && limit > 0 && !power_is_off()) {
        if (s_idle_s + 60 == limit) {
            ESP_LOGW(TAG, "auto power off in 60 s (no audio for %lu s)", (unsigned long)s_idle_s);
        }
        if (s_idle_s >= limit) {
            power_soft_off("auto power off (no audio)");
        }
    }
    if (s_manual_off_s > 0 && --s_manual_off_s == 0) {
        power_soft_off("JBL One power-off timer");
    }
    btstack_run_loop_set_timer(ts, 1000);
    btstack_run_loop_add_timer(ts);
}

/* ---- Auracast assistant (Join Nearby Broadcast): called by party.c ---------- */
void jbl_v5_assist_found(const bd_addr_t addr, uint8_t addr_type, uint8_t flags, const char *name)
{
    if (!s_linked) return;
    uint8_t g[6 + 1 + 1 + 33];
    memcpy(g, addr, 6);
    g[6] = addr_type;
    g[7] = flags;
    memset(g + 8, 0, 33);
    str_copy(g + 8, name, 32);
    uint8_t p[2 + 4 + sizeof(g)];
    p[0] = 0; p[1] = 0;
    uint16_t off = put_tlv(p, 2, 0x0B82, g, sizeof(g));
    send_frame(s_link, 3, p, off);
    s_notifies++;
}

void jbl_v5_assist_result(uint8_t action, const bd_addr_t addr, uint8_t addr_type, uint8_t status)
{
    le_audio_bass_assist_result(action, status);         /* the phone's "Listen to audio broadcast" */
    if (!s_linked) return;
    uint8_t r[9], p[2 + 4 + 9];
    r[0] = action;
    r[1] = addr_type;
    memcpy(r + 2, addr, 6);
    r[8] = status;                                    /* 1 ok, 2 timeout, 3 bad code, 4 code needed */
    p[0] = 0; p[1] = 0;
    uint16_t off = put_tlv(p, 2, 0x0B83, r, sizeof(r));
    send_frame(s_link, 3, p, off);
    s_notifies++;
    jbl_v5_notify(0x0B80);
}

void jbl_v5_assist_scan_state(bool scanning)
{
    if (!s_linked) return;
    uint8_t r[2] = { scanning ? 1 : 0, 0 }, p[2 + 4 + 2];
    p[0] = 0; p[1] = 0;
    uint16_t off = put_tlv(p, 2, 0x0B81, r, sizeof(r));
    send_frame(s_link, 3, p, off);
    jbl_v5_notify(0x0B80);
}

static void heartbeat(btstack_timer_source_t *ts)
{
    static const uint8_t hb[6] = { 0, 0, 0x02, 0x00, 0x00, 0x00 };
    if (s_linked) {
        if (jbl_gatt_tx_busy() || s_ota.active) {
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
    cfg_load();
    eq_restore();
    le_audio_set_enabled((cfg.le_audio & 0x80) != 0);   /* advertised once HCI is up */
    ESP_LOGW(TAG, "V5 server: name '%s', firmware %u.%u.%u", cfg.name, cfg.fw[0], cfg.fw[1], cfg.fw[2]);
    btstack_run_loop_set_timer_handler(&s_idle_timer, &idle_tick);
    btstack_run_loop_set_timer(&s_idle_timer, 1000);
    btstack_run_loop_add_timer(&s_idle_timer);
    btstack_run_loop_set_timer_handler(&s_hb_timer, &heartbeat);
    btstack_run_loop_set_timer(&s_hb_timer, V5_HEARTBEAT_MS);
    btstack_run_loop_add_timer(&s_hb_timer);
}

void jbl_v5_log_stats(void)
{
    ESP_LOGI(TAG, "V5 link %s (con 0x%04x) | rx %lu frames (Get %lu, Set %lu, split %lu, bad %lu) | "
             "tx %lu frames / %lu pkts, dropped %lu | heartbeat %lu sent, %lu skipped | EQ sets %lu | notifies %lu",
             s_linked ? "up" : "down", s_link, (unsigned long)s_rx_frames, (unsigned long)s_rx_get,
             (unsigned long)s_rx_set, (unsigned long)s_rx_split, (unsigned long)s_rx_bad,
             (unsigned long)s_tx_frames, (unsigned long)s_tx_pkts, (unsigned long)s_tx_drop,
             (unsigned long)s_hb_sent, (unsigned long)s_hb_skipped, (unsigned long)s_eq_sets,
             (unsigned long)s_notifies);
    if (s_ota.active) {
        ESP_LOGW(TAG, "OTA capturing: %lu / %lu B (%lu %%), resends %lu", (unsigned long)s_ota.next,
                 (unsigned long)s_ota.size, (unsigned long)((uint64_t)s_ota.next * 100u / s_ota.size),
                 (unsigned long)s_ota.resends);
    }
}
