/*
 * S31 as its own "JBL One app" on BTstack (port of the Bluedroid build's
 * jbl_assistant.c): make a nearby JBL speaker join the S31's Auracast
 * broadcast, with no phone involved.
 *
 * Protocol (decoded from JBL One 2.9.4, com.harman.v5protocol):
 *   GATT service 65786365-6c70-6f69-6e74-2e636f6d0000 ("excelpoint.com")
 *     ...0002  write    (commands to the speaker)
 *     ...0001  notify   (responses / notifications)
 *   Frame (LE): u16 identifier (0xDD00 Direct) | u16 cmd (1 Get, 2 Set, 3 Notify)
 *               | u8 pktCount | u8 pktIndex | u16 payloadLen | payload
 *   Set payload: repeated { u16 featId, u16 len, value }
 *   Response/notify payload: u16 status, then repeated { u16 featId, u16 len, value }
 *   Auracast-assistant features:
 *     0x0B81 ScanControl     [1=start, timeout s]
 *     0x0B82 GroupInfo       [addr6 MSB first][addrType][flags][name\0]  (one per source found)
 *     0x0B83 GroupSelection  [1=connect][addrType][addr6 MSB first][0xFF]
 *     0x0B85 SubgroupPlay    [1=play][subgroup idx][quality 0=std/1=high]
 *
 * Flow: scan LE for the speaker's 0xFDDF beacon (any V5 speaker, not our own
 *   macCrc) -> connect -> pair (Just Works) -> find service -> enable notify
 *   -> Set ScanControl -> wait for a GroupInfo whose name/address is ours
 *   -> Set SubgroupPlay + GroupSelection (join) -> drop the link.
 *
 * BTstack thread only, except jbl_assist_set_paused / jbl_assist_set_broadcast.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "btstack.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "jbl.h"
#include "jbl_assist.h"
#include "a2dp_in.h"
#include "jbl_util.h"

static const char *TAG = "JBL_ASSIST";

#define JBL_FDDF            0xFDDF
#define SCAN_TIMEOUT_S      60
#define JOIN_FALLBACK_MS    20000          /* join anyway if our source isn't listed */
#define TICK_MS             500
#define NATIVE_GRACE_MS     15000          /* give the speaker time to join us natively first */
#define SPEAKER_COOLDOWN_MS 30000
#define SECURITY_WAIT_MS    5000           /* discover anyway if pairing never reports */
#define MAX_SPEAKERS        32

/* Scan: passive, 30 ms window. The Bluedroid build used a 1 s interval (3 %
 * duty); BTstack's gap_set_scan_params caps the interval at 0x400 (640 ms),
 * so this is 30 ms every 640 ms (4.7 %). The scan is off while the phone
 * streams and while the party role listens. */
#define SCAN_INTERVAL       0x0400
#define SCAN_WINDOW         48

/* 0 = only read the speaker's settings on each state change (learning mode);
 * 1 = also make a speaker that's looking for a master join our broadcast. */
#define ASSIST_JOIN         1
/* 0 = never connect to the speaker, only watch its beacon (native-join test). */
#define ASSIST_CONNECT      1
/* After a join: set AuracastMode (0x2004) to this; 0xFF = leave it alone
 * (3 was ignored and re-applying "On" restarts its search). */
#define JOIN_SET_AURACAST_MODE 0xFF
/* Keep the control link to a joined speaker open (tested: party light goes off
 * either way; closing frees the link so any number of speakers can be brought in). */
#define KEEP_LINK_AFTER_JOIN   0

/* excelpoint.com, big-endian as BTstack's uuid128 */
static const uint8_t com_svc_uuid[16] = { 0x65, 0x78, 0x63, 0x65, 0x6c, 0x70, 0x6f, 0x69,
                                          0x6e, 0x74, 0x2e, 0x63, 0x6f, 0x6d, 0x00, 0x00 };
static const uint8_t com_c1_uuid[16]  = { 0x65, 0x78, 0x63, 0x65, 0x6c, 0x70, 0x6f, 0x69,
                                          0x6e, 0x74, 0x2e, 0x63, 0x6f, 0x6d, 0x00, 0x01 };
static const uint8_t com_c2_uuid[16]  = { 0x65, 0x78, 0x63, 0x65, 0x6c, 0x70, 0x6f, 0x69,
                                          0x6e, 0x74, 0x2e, 0x63, 0x6f, 0x6d, 0x00, 0x02 };

typedef enum {
    ST_IDLE, ST_SCAN_PAUSED, ST_SCANNING, ST_STOPPING, ST_CONNECTING, ST_CONNECTED, ST_PROBING,
    ST_SEARCHING, ST_JOINING, ST_DONE,
    ST_PARTY_OFF,          /* paused by the party role (S31 is a LISTENer, not hosting) */
} state_t;

static const char *const s_state_name[] = {
    "idle", "scan paused", "scanning", "stopping", "connecting", "connected", "probing",
    "searching", "joining", "done", "party off",
};

typedef enum { G_IDLE, G_SVC, G_CHR, G_CCCD, G_WRITE } gatt_step_t;

/* ---- state (BTstack thread) ------------------------------------------------- */

static bool s_started;
static volatile bool s_paused;                    /* party LISTEN: no scan, no joins */
static bool s_streaming;                          /* phone music: no scan */
static bool s_pending_join;
static state_t s_state = ST_IDLE;
static uint32_t s_state_since;
static hci_con_handle_t s_con = HCI_CON_HANDLE_INVALID;
static bd_addr_t s_peer;
static bd_addr_type_t s_peer_type;
static gatt_client_service_t s_svc;
static bool s_have_svc;
static gatt_client_characteristic_t s_c_notify, s_c_write;
static bool s_have_notify, s_have_write, s_write_rsp = true;
static gatt_client_notification_t s_notif;
static bool s_listening;
static gatt_step_t s_gstep = G_IDLE;
static bool s_disc_started;
static uint32_t s_connected_at;
static uint16_t s_own_crc;
static char s_own_name[48] = JBL_BROADCAST_NAME;
static bd_addr_t s_own_addr;
static uint8_t s_own_addr_type;
static uint32_t s_search_start;
static bool s_join_sent;
static bool s_close_sent;
static bool s_pause_close;                        /* link close already requested for this pause */
static uint16_t s_cur_crc;                        /* speaker currently being brought in */
static btstack_timer_source_t s_tick;
static btstack_packet_callback_registration_t s_hci_reg, s_sm_reg;

/* any-thread broadcast identity hand-over */
static portMUX_TYPE s_bc_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_bc_pending;
static char s_bc_name[48];
static bd_addr_t s_bc_addr;
static uint8_t s_bc_addr_type;
static bool s_bc_addr_set;                        /* else: controller public address */

/* stats */
static uint32_t s_reports, s_connects, s_joins_sent, s_joins_ok, s_native_joins;

/* ---- outgoing V5 writes (one GATT request at a time) ------------------------- */

#define WQ_LEN   4
#define WQ_FRAME (8 + 64)
static uint8_t s_wq[WQ_LEN][WQ_FRAME];
static uint16_t s_wq_len[WQ_LEN];
static uint8_t s_wq_head, s_wq_count;

static void gatt_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static void set_state(state_t s)
{
    if (s != s_state) {
        s_state = s;
        s_state_since = btstack_run_loop_get_time_ms();
    }
}

static uint32_t state_age(void)
{
    return btstack_run_loop_get_time_ms() - s_state_since;
}

static void hexs(const uint8_t *d, int n, char *out, int outsz)
{
    int o = 0;
    for (int i = 0; i < n && o + 4 < outsz; i++) {
        o += snprintf(out + o, (size_t)(outsz - o), "%02x ", d[i]);
    }
    out[o] = 0;
}

static void wq_drain(void)
{
    while (s_wq_count && s_gstep == G_IDLE && s_con != HCI_CON_HANDLE_INVALID && s_have_write) {
        uint8_t *f = s_wq[s_wq_head];
        uint16_t len = s_wq_len[s_wq_head];
        uint16_t mtu = ATT_DEFAULT_MTU;
        gatt_client_get_mtu(s_con, &mtu);
        bool fits = len <= mtu - 3;
        uint8_t st;
        if (!s_write_rsp && fits) {
            st = gatt_client_write_value_of_characteristic_without_response(s_con, s_c_write.value_handle, len, f);
            if (st == GATT_CLIENT_BUSY) {
                gatt_client_request_can_write_without_response_event(&gatt_handler, s_con);
                return;
            }
            if (st != ERROR_CODE_SUCCESS) {
                ESP_LOGW(TAG, "V5 write failed (0x%02x)", st);
            }
        } else {
            if (!s_write_rsp && !(s_c_write.properties & ATT_PROPERTY_WRITE)) {
                ESP_LOGW(TAG, "V5 frame %u B > MTU %u and no long write - dropped", len, mtu);
                st = ERROR_CODE_UNSPECIFIED_ERROR;
            } else if (fits) {
                st = gatt_client_write_value_of_characteristic(&gatt_handler, s_con, s_c_write.value_handle, len, f);
            } else {
                st = gatt_client_write_long_value_of_characteristic(&gatt_handler, s_con, s_c_write.value_handle, len, f);
            }
            if (st == ERROR_CODE_SUCCESS) {
                s_gstep = G_WRITE;               /* frame stays queued until the response */
                return;
            }
            ESP_LOGW(TAG, "V5 write failed (0x%02x)", st);
        }
        s_wq_head = (uint8_t)((s_wq_head + 1) % WQ_LEN);
        s_wq_count--;
    }
}

static void wq_reset(void)
{
    s_wq_head = 0;
    s_wq_count = 0;
}

static void v5_write(const uint8_t *frame, uint16_t len)
{
    if (len > WQ_FRAME || s_con == HCI_CON_HANDLE_INVALID || !s_have_write) {
        return;
    }
    if (s_wq_count == WQ_LEN) {
        ESP_LOGW(TAG, "V5 write queue full - frame dropped");
        return;
    }
    uint8_t slot = (uint8_t)((s_wq_head + s_wq_count) % WQ_LEN);
    memcpy(s_wq[slot], frame, len);
    s_wq_len[slot] = len;
    s_wq_count++;
    wq_drain();
}

/* ---- V5 frames ------------------------------------------------------------ */

static void v5_send_set(const uint8_t *payload, uint16_t len)
{
    uint8_t f[8 + 64];
    if (len > 64 || !s_have_write) {
        return;
    }
    f[0] = 0x00; f[1] = 0xDD;          /* identifier Direct */
    f[2] = 0x02; f[3] = 0x00;          /* cmd Set */
    f[4] = 0x01; f[5] = 0x00;          /* 1 packet, index 0 */
    f[6] = len & 0xFF; f[7] = len >> 8;
    memcpy(f + 8, payload, len);
    char h[3 * 72 + 1];
    hexs(f, 8 + len, h, sizeof(h));
    ESP_LOGW(TAG, "-> V5 Set (%u B): %s", 8 + len, h);
    v5_write(f, 8 + len);
}

/* Read-only: ask the speaker for its device / party / Auracast settings. */
static void v5_probe(void)
{
    static const uint16_t ids[] = {
        0x0004, 0x0006, 0x000A, 0x000C,                 /* PID, name, MAC, firmware */
        0x2004, 0x2005,                                 /* AuracastMode, StandardQuality */
        0x2C00, 0x2C01, 0x2C02, 0x2C03, 0x2C04,         /* party status/type/conn/role/channel */
        0x2C05, 0x2C06, 0x2C07, 0x2C80, 0x2C81,         /* group name/ID/action, LLS, partner MAC */
        0x0B80, 0x0B86, 0x0B87, 0x0B89, 0x0B8A,         /* assistant info, broadcaster enable/start/MAC/name */
        0x0BA0, 0x0BA1, 0x2D49,                         /* LE audio status, active source, playing */
    };
    enum { N = sizeof(ids) / sizeof(ids[0]) };
    uint8_t f[8 + 2 * N];
    const uint16_t len = 2 * N;
    f[0] = 0x00; f[1] = 0xDD; f[2] = 0x01; f[3] = 0x00;   /* Direct, Get */
    f[4] = 0x01; f[5] = 0x00; f[6] = len & 0xFF; f[7] = len >> 8;
    for (size_t k = 0; k < N; k++) {
        f[8 + 2 * k] = ids[k] & 0xFF;
        f[9 + 2 * k] = ids[k] >> 8;
    }
    ESP_LOGW(TAG, "-> V5 Get (read-only probe of %u settings)", (unsigned)N);
    v5_write(f, sizeof(f));
}

static void v5_scan_start(void)
{
    const uint8_t p[] = { 0x81, 0x0B, 0x02, 0x00, 0x01, SCAN_TIMEOUT_S };
    ESP_LOGW(TAG, "Asking the speaker to scan for Auracast broadcasts (%d s)", SCAN_TIMEOUT_S);
    v5_send_set(p, sizeof(p));
    s_search_start = btstack_run_loop_get_time_ms();
    s_join_sent = false;
    set_state(ST_SEARCHING);
}

/* addr_msb: 6 bytes, MSB first (as the speaker reports / expects) */
static void v5_join(const uint8_t addr_msb[6], uint8_t addr_type)
{
    uint8_t p[7 + 13];
    int i = 0;
    /* SubgroupPlay: play, subgroup 0, high quality */
    p[i++] = 0x85; p[i++] = 0x0B; p[i++] = 0x03; p[i++] = 0x00;
    p[i++] = 0x01; p[i++] = 0x00; p[i++] = 0x01;
    /* GroupSelection: connect, addrType, addr (MSB first), status 0xFF */
    p[i++] = 0x83; p[i++] = 0x0B; p[i++] = 0x09; p[i++] = 0x00;
    p[i++] = 0x01; p[i++] = addr_type;
    memcpy(p + i, addr_msb, 6); i += 6;
    p[i++] = 0xFF;
    ESP_LOGW(TAG, ">>> JOIN: telling the speaker to play %02x:%02x:%02x:%02x:%02x:%02x (type %u)",
             addr_msb[0], addr_msb[1], addr_msb[2], addr_msb[3], addr_msb[4], addr_msb[5], addr_type);
    v5_send_set(p, (uint16_t)i);
    s_join_sent = true;
    s_joins_sent++;
    set_state(ST_JOINING);
}

static void v5_on_notify(const uint8_t *d, uint16_t n)
{
    char h[3 * 96 + 1];
    hexs(d, n > 96 ? 96 : n, h, sizeof(h));
    ESP_LOGI(TAG, "<- V5 (%u B): %s", n, h);
    if (n < 10) {
        return;
    }
    uint16_t cmd = d[2] | (d[3] << 8);
    uint16_t plen = d[6] | (d[7] << 8);
    const uint8_t *p = d + 8;
    const uint8_t *end = d + (8 + plen <= n ? 8 + plen : n);
    uint16_t status = p[0] | (p[1] << 8);
    p += 2;
    if (cmd == 2 && status != 0) {
        ESP_LOGW(TAG, "<- Set rejected, status 0x%04x", status);
    }
    while (p + 4 <= end) {
        uint16_t fid = p[0] | (p[1] << 8);
        uint16_t flen = p[2] | (p[3] << 8);
        const uint8_t *v = p + 4;
        if (v + flen > end) {
            break;
        }
        /* Log every setting the speaker reports (skip the chatty 0x0Dxx meters). */
        if ((fid & 0xFF00) != 0x0D00) {
            char vh[3 * 40 + 1], va[41];
            int m = flen > 40 ? 40 : flen;
            hexs(v, m, vh, sizeof(vh));
            for (int k = 0; k < m; k++) {
                va[k] = (v[k] >= 0x20 && v[k] < 0x7f) ? (char)v[k] : '.';
            }
            va[m] = 0;
            ESP_LOGW(TAG, "   [%s] feat 0x%04x (%u B): %s |%s|", cmd == 1 ? "GET" : cmd == 3 ? "NTF" : "RSP",
                     fid, flen, vh, va);
        }
        if (fid == 0x0B82 && flen >= 9) {
            char name[40] = { 0 };
            int nl = flen - 8 < 39 ? flen - 8 : 39;
            memcpy(name, v + 8, (size_t)nl);
            ESP_LOGW(TAG, "<- speaker FOUND broadcast '%s' at %02x:%02x:%02x:%02x:%02x:%02x type %u flags 0x%02x",
                     name, v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
            bool ours = s_own_name[0] != 0 && strstr(name, s_own_name) != NULL;
            if (memcmp(v, s_own_addr, 6) == 0) {       /* both MSB first */
                ours = true;
            }
            if (ours && !s_join_sent) {
                ESP_LOGW(TAG, "    ^ that's the S31 - joining it");
                v5_join(v, v[6]);
            }
        } else if (fid == 0x0B83 && flen >= 1) {
            static const char *const r[] = { "?", "SUCCESS", "timeout", "wrong code", "code required" };
            uint8_t res = v[flen - 1];
            if (v[0] != 1) {
                ESP_LOGW(TAG, "<- speaker 0x%04x LEFT the broadcast (action %u, status %u)", s_cur_crc, v[0], res);
            } else {
                ESP_LOGW(TAG, "<- JOIN result: %u (%s)", res, res < 5 ? r[res] : "?");
            }
            if (v[0] == 1 && res == 1 && s_state == ST_JOINING) {
                s_close_sent = false;
                s_joins_ok++;
                set_state(ST_DONE);
                ESP_LOGW(TAG, ">>> speaker 0x%04x joined the S31's Auracast", s_cur_crc);
                if (JOIN_SET_AURACAST_MODE != 0xFF) {
                    const uint8_t m[] = { 0x04, 0x20, 0x01, 0x00, JOIN_SET_AURACAST_MODE };
                    ESP_LOGW(TAG, "    setting its AuracastMode to %d to keep party mode on", JOIN_SET_AURACAST_MODE);
                    v5_send_set(m, sizeof(m));
                    v5_probe();          /* read back party state */
                }
            }
        } else if (fid == 0x0B80 && flen >= 1) {
            ESP_LOGW(TAG, "<- assistant status %u", v[0]);
        }
        p = v + flen;
    }
}

/* ---- LE scan for the speaker ----------------------------------------------- */

static const uint8_t *find_svc16(const uint8_t *ad, int len, uint16_t uuid, int *out_len)
{
    int i = 0;
    while (i + 1 < len) {
        int l = ad[i];
        if (l == 0 || i + 1 + l > len) {
            break;
        }
        if (ad[i + 1] == 0x16 && l >= 3 && (ad[i + 2] | (ad[i + 3] << 8)) == uuid) {
            *out_len = l - 3;
            return ad + i + 4;
        }
        i += l + 1;
    }
    return NULL;
}

/* Offset of the field selected by `bit` inside a V5 beacon payload, or -1.
 * Fields appear in mask order bit15 -> bit1 (sizes from the JBL One parser). */
static int v5_field_offset(const uint8_t *b, int n, int bit, int size)
{
    static const int8_t sz[16] = { 0, 2, 5, 2, 3, 3, 1, 2, 1, 2, 1, 6, 2, 2, 2, 1 };
    /* index = bit: 15 color 1, 14 src1 2, 13 src2 2, 12 macCrc 2, 11 mac 6,
     * 10 battery 1, 9 misc 2, 8 partyMethod 1, 7 partyInfo 2, 6 stereoGrp 1,
     * 5 fw3 3, 4 deviceId 3, 3 partyLight 2, 2 fw5 5, 1 misc2 2 */
    if (n < 4) {
        return -1;
    }
    uint16_t mask = b[2] | (b[3] << 8);
    if (!(mask & (1u << bit))) {
        return -1;
    }
    int off = 4;
    for (int k = 15; k > bit; k--) {
        if (mask & (1u << k)) {
            off += sz[k];
        }
    }
    return off + size <= n ? off : -1;
}

/* partyInfo (u16): b15 on, b13-14 group, b11-12 conn (1 connecting, 2 connected),
 * b9-10 role (1 Secondary, 2 Primary), b7-8 channel, b6 join status */
#define PI_ON(p)    (((p) >> 15) & 1)
#define PI_GROUP(p) (((p) >> 13) & 3)
#define PI_CONN(p)  (((p) >> 11) & 3)
#define PI_ROLE(p)  (((p) >> 9) & 3)

static const char *pi_str(uint16_t p)
{
    static char s[64];
    static const char *const conn[] = { "idle", "connecting", "connected", "?" };
    static const char *const role[] = { "normal", "SECONDARY", "PRIMARY", "sub" };
    if (!PI_ON(p)) {
        return "Auracast OFF";
    }
    snprintf(s, sizeof(s), "Auracast ON, %s, %s", role[PI_ROLE(p)], conn[PI_CONN(p)]);
    return s;
}

typedef struct {
    uint16_t crc;
    uint16_t pi;
    bool want;
    uint32_t want_at;          /* ms */
    uint32_t cooldown;         /* ms */
} spk_t;

static spk_t s_spk[MAX_SPEAKERS];
static int s_nspk;

static spk_t *spk_get(uint16_t crc)
{
    for (int i = 0; i < s_nspk; i++) {
        if (s_spk[i].crc == crc) {
            return &s_spk[i];
        }
    }
    if (s_nspk >= MAX_SPEAKERS) {
        return NULL;
    }
    spk_t *s = &s_spk[s_nspk++];
    memset(s, 0, sizeof(*s));
    s->crc = crc;
    s->pi = 0xFFFF;
    s->cooldown = btstack_run_loop_get_time_ms();
    ESP_LOGW(TAG, "New JBL speaker seen: macCrc 0x%04x (%d known)", crc, s_nspk);
    return s;
}

static bool time_reached(uint32_t now, uint32_t t)
{
    return (int32_t)(now - t) >= 0;
}

static void on_adv_report(const bd_addr_t addr, bd_addr_type_t addr_type, bool connectable,
                          const uint8_t *ad, int ad_len)
{
    int dl = 0;
    const uint8_t *b = find_svc16(ad, ad_len, JBL_FDDF, &dl);
    /* Any JBL V5 speaker (Xtreme 5, Charge 6, Flip 7, ...): 0xFDDF beacon with
     * a btMacAddressCrc and a partyInfo field. */
    if (!b || dl < 6) {
        return;
    }
    int co = v5_field_offset(b, dl, 12, 2);
    int po = v5_field_offset(b, dl, 7, 2);
    if (co < 0 || po < 0) {
        return;
    }
    uint16_t crc = b[co] | (b[co + 1] << 8);
    if (crc == s_own_crc) {
        return;                          /* that's us */
    }
    s_reports++;
    uint16_t pid = b[0] | (b[1] << 8);
    uint16_t pi = b[po] | (b[po + 1] << 8);
    spk_t *sp = spk_get(crc);
    if (sp == NULL) {
        return;
    }
    const uint32_t now = btstack_run_loop_get_time_ms();
    if (sp->pi != pi) {
        bool receiving = PI_ON(pi) && PI_ROLE(pi) == 1 && PI_CONN(pi) == 2;
        ESP_LOGW(TAG, "JBL %04x (macCrc 0x%04x) is now: %s [0x%04x]%s", pid, crc, pi_str(pi), pi,
                 receiving ? "  <- receiving a master's Auracast" : "");
        /* Native JBL behaviour: a speaker whose Auracast button was pressed with
         * nothing to host goes "party on, SECONDARY, connecting" = looking for a
         * master. */
        bool looking_for_master = PI_ON(pi) && PI_ROLE(pi) == 1 && PI_CONN(pi) == 1;
        sp->pi = pi;
        if (looking_for_master && time_reached(now, sp->cooldown)) {
            sp->want = true;
            sp->want_at = now + NATIVE_GRACE_MS;
            ESP_LOGW(TAG, ">>> speaker 0x%04x: Auracast button pressed, looking for a master "
                     "(waiting %d s for it to join natively)", crc, NATIVE_GRACE_MS / 1000);
        } else if (sp->want) {
            sp->want = false;
            if (receiving) {
                s_native_joins++;
            }
            ESP_LOGW(TAG, ">>> speaker 0x%04x left 'looking for master' on its own%s", crc,
                     receiving ? " - NATIVE JOIN!" : "");
        }
    }
    if (!ASSIST_CONNECT || s_state != ST_SCANNING || s_paused || !sp->want || !connectable ||
        !time_reached(now, sp->want_at)) {
        return;
    }
    ESP_LOGW(TAG, "speaker 0x%04x didn't join natively in %d s - bringing it in", crc, NATIVE_GRACE_MS / 1000);
    sp->want = false;
    sp->cooldown = now + SPEAKER_COOLDOWN_MS;
    s_cur_crc = crc;
    s_pending_join = ASSIST_JOIN;
    memcpy(s_peer, addr, sizeof(bd_addr_t));
    s_peer_type = addr_type;
    ESP_LOGW(TAG, ">>> connecting to speaker 0x%04x at %s to bring it into our broadcast", crc, bd_addr_to_str(s_peer));
    /* BTstack disables the scan before it sends LE Create Connection */
    set_state(ST_STOPPING);
    gap_stop_scan();
    uint8_t st = gap_connect(s_peer, s_peer_type);
    if (st != ERROR_CODE_SUCCESS) {
        ESP_LOGW(TAG, "connect not started (0x%02x) - rescanning", st);
        set_state(ST_IDLE);
        return;
    }
    s_connects++;
    set_state(ST_CONNECTING);
}

/* ---- GATT client ------------------------------------------------------------- */

static void link_cleanup(void)
{
    if (s_listening) {
        gatt_client_stop_listening_for_characteristic_value_updates(&s_notif);
        s_listening = false;
    }
    s_con = HCI_CON_HANDLE_INVALID;
    s_have_svc = s_have_notify = s_have_write = false;
    s_write_rsp = true;
    s_gstep = G_IDLE;
    s_disc_started = false;
    wq_reset();
}

static void link_close(void)
{
    if (s_con != HCI_CON_HANDLE_INVALID) {
        gap_disconnect(s_con);
    }
}

static void discover_start(void)
{
    if (s_disc_started || s_con == HCI_CON_HANDLE_INVALID) {
        return;
    }
    s_disc_started = true;
    uint8_t st = gatt_client_discover_primary_services_by_uuid128(&gatt_handler, s_con, com_svc_uuid);
    if (st != ERROR_CODE_SUCCESS) {
        ESP_LOGW(TAG, "service discovery not started (0x%02x) - closing", st);
        link_close();
        return;
    }
    s_gstep = G_SVC;
}

static void on_notify_enabled(uint8_t att_status)
{
    ESP_LOGI(TAG, "notifications on (status 0x%x)", att_status);
    if (s_pending_join) {
        v5_scan_start();            /* straight to: speaker scans -> finds us -> join */
    } else {
        set_state(ST_PROBING);
        v5_probe();
    }
}

static void gatt_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }
    switch (hci_event_packet_get_type(packet)) {
    case GATT_EVENT_SERVICE_QUERY_RESULT:
        if (gatt_event_service_query_result_get_handle(packet) == s_con) {
            gatt_event_service_query_result_get_service(packet, &s_svc);
            s_have_svc = true;
        }
        break;
    case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT: {
        if (gatt_event_characteristic_query_result_get_handle(packet) != s_con) {
            break;
        }
        gatt_client_characteristic_t c;
        gatt_event_characteristic_query_result_get_characteristic(packet, &c);
        if (memcmp(c.uuid128, com_c1_uuid, 16) == 0) {
            s_c_notify = c;
            s_have_notify = true;
        } else if (memcmp(c.uuid128, com_c2_uuid, 16) == 0) {
            s_c_write = c;
            s_have_write = true;
            s_write_rsp = (c.properties & ATT_PROPERTY_WRITE) != 0;
        }
        break;
    }
    case GATT_EVENT_QUERY_COMPLETE: {
        if (gatt_event_query_complete_get_handle(packet) != s_con) {
            break;
        }
        uint8_t att = gatt_event_query_complete_get_att_status(packet);
        gatt_step_t step = s_gstep;
        s_gstep = G_IDLE;
        switch (step) {
        case G_SVC:
            if (!s_have_svc) {
                ESP_LOGW(TAG, "speaker has no excelpoint.com service?! (att 0x%02x)", att);
                link_close();
                break;
            }
            if (gatt_client_discover_characteristics_for_service(&gatt_handler, s_con, &s_svc) == ERROR_CODE_SUCCESS) {
                s_gstep = G_CHR;
            } else {
                link_close();
            }
            break;
        case G_CHR:
            ESP_LOGW(TAG, "excelpoint.com: notify 0x%04x, write 0x%04x (%s)",
                     s_have_notify ? s_c_notify.value_handle : 0, s_have_write ? s_c_write.value_handle : 0,
                     s_write_rsp ? "with response" : "no response");
            if (!s_have_notify) {
                on_notify_enabled(att);          /* nothing to enable: still talk */
                break;
            }
            gatt_client_listen_for_characteristic_value_updates(&s_notif, &gatt_handler, s_con, &s_c_notify);
            s_listening = true;
            if (gatt_client_write_client_characteristic_configuration(&gatt_handler, s_con, &s_c_notify,
                    GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION) == ERROR_CODE_SUCCESS) {
                s_gstep = G_CCCD;
            } else {
                on_notify_enabled(0xFF);
            }
            break;
        case G_CCCD:
            on_notify_enabled(att);
            break;
        case G_WRITE:
            if (att != ATT_ERROR_SUCCESS) {
                ESP_LOGW(TAG, "write failed, ATT status 0x%02x", att);
            }
            if (s_wq_count) {
                s_wq_head = (uint8_t)((s_wq_head + 1) % WQ_LEN);
                s_wq_count--;
            }
            break;
        default:
            break;
        }
        wq_drain();
        break;
    }
    case GATT_EVENT_CAN_WRITE_WITHOUT_RESPONSE:
        wq_drain();
        break;
    case GATT_EVENT_NOTIFICATION:
        if (gatt_event_notification_get_handle(packet) == s_con) {
            v5_on_notify(gatt_event_notification_get_value(packet), gatt_event_notification_get_value_length(packet));
        }
        break;
    default:
        break;
    }
}

/* ---- HCI / SM events ------------------------------------------------------------ */

static void hci_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }
    switch (hci_event_packet_get_type(packet)) {
    case GAP_EVENT_ADVERTISING_REPORT: {
        bd_addr_t a;
        gap_event_advertising_report_get_address(packet, a);
        uint8_t type = gap_event_advertising_report_get_advertising_event_type(packet);
        on_adv_report(a, (bd_addr_type_t)gap_event_advertising_report_get_address_type(packet),
                      type == 0 || type == 1,               /* ADV_IND / ADV_DIRECT_IND */
                      gap_event_advertising_report_get_data(packet),
                      gap_event_advertising_report_get_data_length(packet));
        break;
    }
    case GAP_EVENT_EXTENDED_ADVERTISING_REPORT: {
        bd_addr_t a;
        gap_event_extended_advertising_report_get_address(packet, a);
        uint16_t type = gap_event_extended_advertising_report_get_advertising_event_type(packet);
        on_adv_report(a, (bd_addr_type_t)gap_event_extended_advertising_report_get_address_type(packet),
                      (type & 0x01) != 0,
                      gap_event_extended_advertising_report_get_data(packet),
                      gap_event_extended_advertising_report_get_data_length(packet));
        break;
    }
    case HCI_EVENT_META_GAP:
        if (hci_event_gap_meta_get_subevent_code(packet) != GAP_SUBEVENT_LE_CONNECTION_COMPLETE ||
            gap_subevent_le_connection_complete_get_role(packet) != HCI_ROLE_MASTER) {
            break;
        }
        if (s_state != ST_CONNECTING) {
            /* our cancelled connect completed after all: don't leave it dangling */
            bd_addr_t a;
            gap_subevent_le_connection_complete_get_peer_address(packet, a);
            if (gap_subevent_le_connection_complete_get_status(packet) == ERROR_CODE_SUCCESS &&
                s_con == HCI_CON_HANDLE_INVALID && bd_addr_cmp(a, s_peer) == 0) {
                gap_disconnect(gap_subevent_le_connection_complete_get_connection_handle(packet));
            }
            break;
        }
        {
            uint8_t st = gap_subevent_le_connection_complete_get_status(packet);
            if (st != ERROR_CODE_SUCCESS) {
                ESP_LOGW(TAG, "connect failed (0x%02x) - rescanning", st);
                set_state(ST_IDLE);
                break;
            }
            link_cleanup();
            s_con = gap_subevent_le_connection_complete_get_connection_handle(packet);
            s_connected_at = btstack_run_loop_get_time_ms();
            set_state(ST_CONNECTED);
            ESP_LOGW(TAG, "Connected to speaker (con 0x%04x) - pairing", s_con);
            sm_request_pairing(s_con);           /* Just Works; discovery once it reports */
        }
        break;
    case HCI_EVENT_DISCONNECTION_COMPLETE:
        if (hci_event_disconnection_complete_get_connection_handle(packet) == s_con) {
            ESP_LOGW(TAG, "control link closed (reason 0x%x)%s", hci_event_disconnection_complete_get_reason(packet),
                     s_state == ST_DONE ? " - speaker stays on our Auracast; watching its beacon"
                                        : " - watching for the next button press");
            link_cleanup();
            set_state(ST_IDLE);
        }
        break;
    default:
        break;
    }
}

static void sm_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }
    switch (hci_event_packet_get_type(packet)) {
    case SM_EVENT_PAIRING_COMPLETE:
        if (sm_event_pairing_complete_get_handle(packet) == s_con) {
            uint8_t st = sm_event_pairing_complete_get_status(packet);
            ESP_LOGW(TAG, "LE pairing with speaker: %s (status 0x%x, reason 0x%x)", st == ERROR_CODE_SUCCESS ? "OK" : "FAILED",
                     st, sm_event_pairing_complete_get_reason(packet));
            discover_start();
        }
        break;
    case SM_EVENT_REENCRYPTION_COMPLETE:
        if (sm_event_reencryption_complete_get_handle(packet) == s_con) {
            ESP_LOGW(TAG, "LE link to speaker re-encrypted (status 0x%x)", sm_event_reencryption_complete_get_status(packet));
            discover_start();
        }
        break;
    default:
        break;
    }
}

/* ---- state machine --------------------------------------------------------------- */

static void broadcast_identity_pickup(void)
{
    if (!s_bc_pending) {
        return;
    }
    taskENTER_CRITICAL(&s_bc_mux);
    memcpy(s_own_name, s_bc_name, sizeof(s_own_name));
    if (s_bc_addr_set) {
        memcpy(s_own_addr, s_bc_addr, sizeof(bd_addr_t));
        s_own_addr_type = s_bc_addr_type;
    }
    s_bc_pending = false;
    taskEXIT_CRITICAL(&s_bc_mux);
    ESP_LOGI(TAG, "our broadcast: '%s' at %s (type %u)", s_own_name, bd_addr_to_str(s_own_addr), s_own_addr_type);
}

/* Party role LISTEN: the S31 is not hosting, so there is nothing to bring a
 * speaker into, and the LE scanner belongs to the sink. */
static void pause_step(void)
{
    uint32_t age = state_age();
    switch (s_state) {
    case ST_SCANNING:
        ESP_LOGW(TAG, "party LISTEN -> stopping the JBL speaker scan (S31 is not hosting)");
        gap_stop_scan();
        set_state(ST_PARTY_OFF);
        break;
    case ST_IDLE:
    case ST_SCAN_PAUSED:
        set_state(ST_PARTY_OFF);
        break;
    case ST_STOPPING:
        if (age > 3000) {
            set_state(ST_IDLE);
        }
        break;
    case ST_CONNECTING:
        if (age > 15000) {
            gap_connect_cancel();
            set_state(ST_IDLE);
        }
        break;
    case ST_CONNECTED: case ST_PROBING: case ST_SEARCHING: case ST_JOINING: case ST_DONE:
        if (!s_pause_close) {
            ESP_LOGW(TAG, "party LISTEN -> closing the control link to speaker 0x%04x", s_cur_crc);
            s_pause_close = true;
            link_close();
        }
        break;
    default:
        break;
    }
}

static void tick(btstack_timer_source_t *ts)
{
    btstack_run_loop_set_timer(ts, TICK_MS);
    btstack_run_loop_add_timer(ts);
    broadcast_identity_pickup();

    if (s_paused) {
        pause_step();
        return;
    }
    s_pause_close = false;
    if (s_state == ST_PARTY_OFF) {
        ESP_LOGW(TAG, "party HOST -> JBL speaker assistant resumed");
        set_state(s_streaming ? ST_SCAN_PAUSED : ST_IDLE);
    }
    /* While phone music streams, stop the LE scan: each scan window blocks the
     * phone's A2DP link, which then can't carry bitpool 53. Native join doesn't
     * need it - speakers find us by our own adverts. */
    if (s_state == ST_SCANNING && s_streaming) {
        ESP_LOGW(TAG, "phone music streaming -> pausing the JBL speaker scan (frees airtime for A2DP)");
        set_state(ST_SCAN_PAUSED);
        gap_stop_scan();
        return;
    }
    if (s_state == ST_SCAN_PAUSED) {
        if (!s_streaming) {
            ESP_LOGW(TAG, "phone music stopped -> resuming the JBL speaker scan");
            set_state(ST_IDLE);
        }
        return;
    }
    uint32_t age = state_age();
    switch (s_state) {
    case ST_IDLE:
        if (S31_QUIET_RADIO_TEST) {
            break;                               /* bench: no scan */
        }
        ESP_LOGI(TAG, "Hosting; watching for a JBL speaker whose Auracast button is pressed...");
        gap_set_scan_params(0 /* passive */, SCAN_INTERVAL, SCAN_WINDOW, 0);
        gap_set_scan_duplicate_filter(false);    /* partyInfo changes must be reported */
        gap_start_scan();
        set_state(ST_SCANNING);
        break;
    case ST_CONNECTING:
        if (age > 15000) {
            ESP_LOGW(TAG, "connect timed out - rescanning");
            gap_connect_cancel();
            set_state(ST_IDLE);
        }
        break;
    case ST_CONNECTED:
        if (!s_disc_started && btstack_run_loop_get_time_ms() - s_connected_at > SECURITY_WAIT_MS) {
            ESP_LOGW(TAG, "no pairing result in %d s - discovering anyway", SECURITY_WAIT_MS / 1000);
            discover_start();
        }
        break;
    case ST_SEARCHING:
        if (!s_join_sent && btstack_run_loop_get_time_ms() - s_search_start > JOIN_FALLBACK_MS) {
            ESP_LOGW(TAG, "speaker didn't list our broadcast - sending join with our address anyway");
            v5_join(s_own_addr, s_own_addr_type);
        }
        break;
    case ST_JOINING:
        if (age > 30000) {
            ESP_LOGW(TAG, "no join confirmation - re-scanning on the speaker");
            v5_scan_start();
        }
        break;
    case ST_PROBING:
        if (age > 2500) {
            if (s_pending_join) {
                v5_scan_start();
            } else {
                s_close_sent = false;
                set_state(ST_DONE);              /* drop the link, keep watching */
            }
        }
        break;
    case ST_STOPPING:
        if (age > 3000) {
            set_state(ST_IDLE);
        }
        break;
    case ST_DONE:
        /* Like a real master: hand-off done, drop the control link. */
        if (!s_close_sent && age > 3000) {
            s_close_sent = true;
            if (KEEP_LINK_AFTER_JOIN) {
                ESP_LOGW(TAG, "keeping the link to speaker 0x%04x open; watching for more speakers", s_cur_crc);
                set_state(ST_IDLE);              /* resume scanning; link stays up */
            } else {
                link_close();
            }
        }
        break;
    default:
        break;
    }
}

/* ---- API -------------------------------------------------------------------------- */

/* Stop our scan right away (not on the next tick) when the party role takes the
 * scanner: its own scan commands are posted / issued after this one. */
static void pause_apply(void *ctx)
{
    (void)ctx;
    if (!s_started || !s_paused) {
        return;
    }
    if (s_state == ST_SCANNING || s_state == ST_IDLE || s_state == ST_SCAN_PAUSED) {
        pause_step();
    }
}

static btstack_context_callback_registration_t s_pause_reg = { .callback = &pause_apply };

void jbl_assist_set_paused(bool paused)
{
    s_paused = paused;           /* scan stops now; links close / resume on the next tick */
    if (paused) {
        jbl_run_on_bt(&s_pause_reg);
    }
}

void jbl_assist_set_broadcast(const char *name, const uint8_t addr[6], uint8_t addr_type)
{
    taskENTER_CRITICAL(&s_bc_mux);
    memset(s_bc_name, 0, sizeof(s_bc_name));
    if (name != NULL) {
        strncpy(s_bc_name, name, sizeof(s_bc_name) - 1);
    }
    s_bc_addr_set = addr != NULL;
    if (addr != NULL) {
        memcpy(s_bc_addr, addr, sizeof(bd_addr_t));
        s_bc_addr_type = addr_type;
    }
    s_bc_pending = true;
    taskEXIT_CRITICAL(&s_bc_mux);
}

void jbl_assist_set_streaming(bool streaming)
{
    s_streaming = streaming;
}

bool jbl_assist_is_link(hci_con_handle_t con)
{
    return con != HCI_CON_HANDLE_INVALID && con == s_con;
}

bool jbl_assist_scan_idle(void)
{
    state_t s = s_state;
    return s != ST_SCANNING && s != ST_STOPPING && s != ST_IDLE && s != ST_SCAN_PAUSED;
}

void jbl_assist_setup(void)
{
    s_hci_reg.callback = &hci_handler;
    hci_add_event_handler(&s_hci_reg);
    s_sm_reg.callback = &sm_handler;
    sm_add_event_handler(&s_sm_reg);
}

void jbl_assist_start(uint16_t own_mac_crc, const bd_addr_t own_addr)
{
    s_own_crc = own_mac_crc;
    memcpy(s_own_addr, own_addr, sizeof(bd_addr_t));
    s_own_addr_type = 0;                         /* public, unless the broadcast says otherwise */
    broadcast_identity_pickup();
    if (s_started) {
        return;
    }
    s_started = true;
    set_state(ST_IDLE);
    btstack_run_loop_set_timer_handler(&s_tick, &tick);
    btstack_run_loop_set_timer(&s_tick, TICK_MS);
    btstack_run_loop_add_timer(&s_tick);
    ESP_LOGW(TAG, "JBL master mode: '%s' is hosting; press Auracast on a JBL speaker to join it", s_own_name);
}

void jbl_assist_log_stats(void)
{
    ESP_LOGI(TAG, "assistant %s%s%s | speakers %d | beacon reports %lu | connects %lu | joins sent %lu, ok %lu | "
             "native joins seen %lu",
             s_state_name[s_state], s_paused ? " (party LISTEN)" : "", s_streaming ? " (phone streaming)" : "",
             s_nspk, (unsigned long)s_reports, (unsigned long)s_connects, (unsigned long)s_joins_sent,
             (unsigned long)s_joins_ok, (unsigned long)s_native_joins);
}
