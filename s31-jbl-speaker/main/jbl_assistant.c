/*
 * S31 as its own "JBL One app": make a nearby JBL speaker join the S31's
 * Auracast broadcast, with no phone involved.
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
 * Flow: scan LE for the speaker's 0xFDDF beacon (PID 2169, not our own CRC)
 *   -> connect -> pair (Just Works) -> MTU -> find service -> enable notify
 *   -> Set ScanControl -> wait for a GroupInfo whose name/address is ours
 *   -> Set SubgroupPlay + GroupSelection (join) -> log the result.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_common_api.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "jbl_assistant.h"
#include "a2dp_bridge.h"
#include "app_task.h"

#define TAG "JBL_ASSIST"

#define JBL_FDDF            0xFDDF
#define TARGET_PID_LO       0x69          /* 2169 on air as 69 21 */
#define TARGET_PID_HI       0x21
#define GATTC_APP_ID        0x2B
#define SCAN_TIMEOUT_S      60
#define JOIN_FALLBACK_MS    20000          /* join anyway if our source isn't listed */

static const uint8_t com_svc_uuid[16] = { 0x00, 0x00, 0x6d, 0x6f, 0x63, 0x2e, 0x74, 0x6e,
                                          0x69, 0x6f, 0x70, 0x6c, 0x65, 0x63, 0x78, 0x65 };
static const uint8_t com_c1_uuid[16]  = { 0x01, 0x00, 0x6d, 0x6f, 0x63, 0x2e, 0x74, 0x6e,
                                          0x69, 0x6f, 0x70, 0x6c, 0x65, 0x63, 0x78, 0x65 };
static const uint8_t com_c2_uuid[16]  = { 0x02, 0x00, 0x6d, 0x6f, 0x63, 0x2e, 0x74, 0x6e,
                                          0x69, 0x6f, 0x70, 0x6c, 0x65, 0x63, 0x78, 0x65 };

typedef enum {
    ST_IDLE, ST_SCAN_PAUSED, ST_SCANNING, ST_STOPPING, ST_CONNECTING, ST_CONNECTED, ST_PROBING,
    ST_SEARCHING, ST_JOINING, ST_DONE,
} state_t;

/* 0 = only read the speaker's settings on each state change (learning mode);
 * 1 = also make a speaker that's looking for a master join our broadcast. */
#define ASSIST_JOIN         1
/* 0 = never connect to the speaker, only watch its beacon (native-join test). */
#define ASSIST_CONNECT      1
/* After a join: set AuracastMode (0x2004) to this so the speaker stays in party
 * mode (3 = "OnLinked"); 0xFF = leave it alone. */
#define JOIN_SET_AURACAST_MODE 0xFF   /* 3 was ignored and re-applying "On" restarts its search */
/* Keep the control link to a joined speaker open (it drops out when the link closes). */
#define KEEP_LINK_AFTER_JOIN   0   /* tested: party light goes off either way; closing frees the
                                       link so any number of speakers can be brought in */

static bool s_pending_join;

static volatile state_t s_state = ST_IDLE;
static esp_gatt_if_t s_gattc_if = ESP_GATT_IF_NONE;
static uint16_t s_conn_id;
static esp_bd_addr_t s_peer;
static esp_ble_addr_type_t s_peer_type;
static uint16_t s_svc_start, s_svc_end;
static uint16_t s_h_notify, s_h_write;
static bool s_write_rsp = true;
static uint16_t s_own_crc;
static char s_own_name[48];
static uint8_t s_own_addr[6];
static TickType_t s_search_start;
static bool s_join_sent;
static bool s_close_sent;
static uint16_t s_cur_crc;          /* speaker currently being brought in */

static esp_ble_ext_scan_params_t s_scan_params = {
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
    .cfg_mask = ESP_BLE_GAP_EXT_SCAN_CFG_UNCODE_MASK,
    /* 30 ms window every 300 ms (10% duty, was 50%): enough to see a speaker's
     * Auracast button state, without starving the broadcast of airtime. */
    .uncoded_cfg = {BLE_SCAN_TYPE_PASSIVE, 1600, 48},   /* 30 ms every 1 s (3 %) */
};

static void hexs(const uint8_t *d, int n, char *out, int outsz)
{
    int o = 0;
    for (int i = 0; i < n && o + 4 < outsz; i++) {
        o += snprintf(out + o, outsz - o, "%02x ", d[i]);
    }
    out[o] = 0;
}

/* ---- V5 frames ------------------------------------------------------------ */

static void v5_send_set(const uint8_t *payload, uint16_t len)
{
    uint8_t f[8 + 64];
    if (len > 64 || s_h_write == 0) {
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
    esp_ble_gattc_write_char(s_gattc_if, s_conn_id, s_h_write, 8 + len, f,
                             s_write_rsp ? ESP_GATT_WRITE_TYPE_RSP : ESP_GATT_WRITE_TYPE_NO_RSP,
                             ESP_GATT_AUTH_REQ_NONE);
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
    uint8_t f[8 + 2 * sizeof(ids) / sizeof(ids[0])];
    uint16_t len = 2 * sizeof(ids) / sizeof(ids[0]);
    f[0] = 0x00; f[1] = 0xDD; f[2] = 0x01; f[3] = 0x00;   /* Direct, Get */
    f[4] = 0x01; f[5] = 0x00; f[6] = len & 0xFF; f[7] = len >> 8;
    for (size_t k = 0; k < sizeof(ids) / sizeof(ids[0]); k++) {
        f[8 + 2 * k] = ids[k] & 0xFF;
        f[9 + 2 * k] = ids[k] >> 8;
    }
    ESP_LOGW(TAG, "-> V5 Get (read-only probe of %u settings)", (unsigned)(sizeof(ids) / sizeof(ids[0])));
    esp_ble_gattc_write_char(s_gattc_if, s_conn_id, s_h_write, sizeof(f), f,
                             s_write_rsp ? ESP_GATT_WRITE_TYPE_RSP : ESP_GATT_WRITE_TYPE_NO_RSP,
                             ESP_GATT_AUTH_REQ_NONE);
}

static void v5_scan_start(void)
{
    const uint8_t p[] = { 0x81, 0x0B, 0x02, 0x00, 0x01, SCAN_TIMEOUT_S };
    ESP_LOGW(TAG, "Asking the speaker to scan for Auracast broadcasts (%d s)", SCAN_TIMEOUT_S);
    v5_send_set(p, sizeof(p));
    s_search_start = xTaskGetTickCount();
    s_join_sent = false;
    s_state = ST_SEARCHING;
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
    v5_send_set(p, i);
    s_join_sent = true;
    s_state = ST_JOINING;
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
                va[k] = (v[k] >= 0x20 && v[k] < 0x7f) ? v[k] : '.';
            }
            va[m] = 0;
            ESP_LOGW(TAG, "   [%s] feat 0x%04x (%u B): %s |%s|", cmd == 1 ? "GET" : cmd == 3 ? "NTF" : "RSP",
                     fid, flen, vh, va);
        }
        if (fid == 0x0B82 && flen >= 9) {
            char name[40] = {0};
            int nl = flen - 8 < 39 ? flen - 8 : 39;
            memcpy(name, v + 8, nl);
            ESP_LOGW(TAG, "<- speaker FOUND broadcast '%s' at %02x:%02x:%02x:%02x:%02x:%02x type %u flags 0x%02x",
                     name, v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
            bool ours = strstr(name, s_own_name) != NULL;
            uint8_t own_msb[6];
            for (int k = 0; k < 6; k++) {
                own_msb[k] = s_own_addr[k];          /* esp addr is already display order */
            }
            if (memcmp(v, own_msb, 6) == 0) {
                ours = true;
            }
            if (ours && !s_join_sent) {
                ESP_LOGW(TAG, "    ^ that's the S31 - joining it");
                v5_join(v, v[6]);
            }
        } else if (fid == 0x0B83 && flen >= 1) {
            static const char *r[] = {"?", "SUCCESS", "timeout", "wrong code", "code required"};
            uint8_t st = v[flen - 1];
            if (v[0] != 1) {
                ESP_LOGW(TAG, "<- speaker 0x%04x LEFT the broadcast (action %u, status %u)", s_cur_crc, v[0], st);
            } else {
                ESP_LOGW(TAG, "<- JOIN result: %u (%s)", st, st < 5 ? r[st] : "?");
            }
            if (v[0] == 1 && st == 1 && s_state == ST_JOINING) {
                s_close_sent = false;
                s_state = ST_DONE;
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
    if (n < 4) return -1;
    uint16_t mask = b[2] | (b[3] << 8);
    if (!(mask & (1 << bit))) return -1;
    int off = 4;
    for (int k = 15; k > bit; k--) {
        if (mask & (1 << k)) off += sz[k];
    }
    return off + size <= n ? off : -1;
}

/* partyInfo (u16): b15 on, b13-14 group, b11-12 conn (1 connecting, 2 connected),
 * b9-10 role (1 Secondary, 2 Primary), b7-8 channel, b6 join status */
#define PI_ON(p)    (((p) >> 15) & 1)
#define PI_GROUP(p) (((p) >> 13) & 3)
#define PI_CONN(p)  (((p) >> 11) & 3)
#define PI_ROLE(p)  (((p) >> 9) & 3)

static TickType_t s_cooldown_until;

static const char *pi_str(uint16_t p)
{
    static char s[64];
    static const char *conn[] = {"idle", "connecting", "connected", "?"};
    static const char *role[] = {"normal", "SECONDARY", "PRIMARY", "sub"};
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
    TickType_t want_at;          /* give the speaker time to join us natively first */
    TickType_t cooldown;
} spk_t;
#define NATIVE_GRACE_MS 15000
#define MAX_SPEAKERS 32
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
    ESP_LOGW(TAG, "New JBL speaker seen: macCrc 0x%04x (%d known)", crc, s_nspk);
    return s;
}

static void on_ext_adv_report(const esp_ble_gap_ext_adv_report_t *r)
{
    int dl = 0;
    const uint8_t *b = find_svc16(r->adv_data, r->adv_data_len, JBL_FDDF, &dl);
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
    uint16_t pid = b[0] | (b[1] << 8);
    uint16_t pi = b[po] | (b[po + 1] << 8);
    {
        spk_t *seen = spk_get(crc);
        if (seen && seen->pi != pi) {
            ESP_LOGW(TAG, "JBL %04x (macCrc 0x%04x) is now: %s [0x%04x]%s", pid, crc, pi_str(pi), pi,
                     (PI_ON(pi) && PI_ROLE(pi) == 1 && PI_CONN(pi) == 2) ? "  <- receiving a master's Auracast" : "");
        }
    }

    /* Native JBL behaviour: a speaker whose Auracast button was pressed with
     * nothing to host goes "party on, SECONDARY, connecting" = looking for a
     * master. */
    bool looking_for_master = PI_ON(pi) && PI_ROLE(pi) == 1 && PI_CONN(pi) == 1;

    /* Any number of JBL speakers: one table entry per speaker (keyed by macCrc). */
    spk_t *sp = spk_get(crc);
    if (sp == NULL) {
        return;
    }
    if (sp->pi != pi) {
        sp->pi = pi;
        if (looking_for_master && xTaskGetTickCount() >= sp->cooldown) {
            sp->want = true;
            sp->want_at = xTaskGetTickCount() + pdMS_TO_TICKS(NATIVE_GRACE_MS);
            ESP_LOGW(TAG, ">>> speaker 0x%04x: Auracast button pressed, looking for a master "
                     "(waiting %d s for it to join natively)", crc, NATIVE_GRACE_MS / 1000);
        } else if (sp->want) {
            sp->want = false;
            ESP_LOGW(TAG, ">>> speaker 0x%04x left 'looking for master' on its own%s", crc,
                     (PI_ON(pi) && PI_ROLE(pi) == 1 && PI_CONN(pi) == 2) ? " - NATIVE JOIN!" : "");
        }
    }
    if (!ASSIST_CONNECT || s_state != ST_SCANNING || !sp->want || !(r->event_type & 0x01) ||
        xTaskGetTickCount() < sp->want_at) {
        return;
    }
    ESP_LOGW(TAG, "speaker 0x%04x didn't join natively in %d s - bringing it in", crc, NATIVE_GRACE_MS / 1000);
    sp->want = false;
    sp->cooldown = xTaskGetTickCount() + pdMS_TO_TICKS(30000);
    s_cur_crc = crc;
    s_pending_join = ASSIST_JOIN;
    memcpy(s_peer, r->addr, 6);
    s_peer_type = r->addr_type;
    ESP_LOGW(TAG, ">>> connecting to speaker 0x%04x at %02x:%02x:%02x:%02x:%02x:%02x to bring it into our broadcast",
             crc, s_peer[0], s_peer[1], s_peer[2], s_peer[3], s_peer[4], s_peer[5]);
    s_state = ST_STOPPING;             /* connect only once the scan has really stopped */
    esp_ble_gap_stop_ext_scan();
}

void jbl_assistant_gap_event(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SET_EXT_SCAN_PARAMS_COMPLETE_EVT:
        esp_ble_gap_start_ext_scan(0, 0);
        break;
    case ESP_GAP_BLE_EXT_SCAN_START_COMPLETE_EVT:
        ESP_LOGI(TAG, "LE scan started (status %d)", param->ext_scan_start.status);
        break;
    case ESP_GAP_BLE_EXT_SCAN_STOP_COMPLETE_EVT:
        if (s_state == ST_STOPPING) {
            s_state = ST_CONNECTING;
            esp_ble_gattc_aux_open(s_gattc_if, s_peer, s_peer_type, true);
        }
        break;
    case ESP_GAP_BLE_EXT_ADV_REPORT_EVT:
        on_ext_adv_report(&param->ext_adv_report.params);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        ESP_LOGW(TAG, "LE pairing with speaker: %s (reason 0x%x)",
                 param->ble_security.auth_cmpl.success ? "OK" : "FAILED",
                 param->ble_security.auth_cmpl.fail_reason);
        break;
    default:
        break;
    }
}

/* ---- GATT client ------------------------------------------------------------- */

static void gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *p)
{
    if (event == ESP_GATTC_REG_EVT) {
        if (p->reg.app_id == GATTC_APP_ID) {
            s_gattc_if = gattc_if;
        }
        return;
    }
    if (gattc_if != s_gattc_if) {
        return;
    }
    switch (event) {
    case ESP_GATTC_OPEN_EVT:
        if (p->open.status != ESP_GATT_OK) {
            ESP_LOGW(TAG, "connect failed (0x%x) - rescanning", p->open.status);
            s_state = ST_IDLE;
            break;
        }
        s_conn_id = p->open.conn_id;
        s_state = ST_CONNECTED;
        ESP_LOGW(TAG, "Connected to speaker (conn %u) - pairing + MTU", s_conn_id);
        esp_ble_set_encryption(s_peer, ESP_BLE_SEC_ENCRYPT_NO_MITM);
        esp_ble_gattc_send_mtu_req(gattc_if, s_conn_id);
        break;
    case ESP_GATTC_CFG_MTU_EVT: {
        ESP_LOGI(TAG, "MTU %u", p->cfg_mtu.mtu);
        esp_bt_uuid_t f = { .len = ESP_UUID_LEN_128 };
        memcpy(f.uuid.uuid128, com_svc_uuid, 16);
        esp_ble_gattc_search_service(gattc_if, s_conn_id, &f);
        break;
    }
    case ESP_GATTC_SEARCH_RES_EVT:
        if (p->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128 &&
            memcmp(p->search_res.srvc_id.uuid.uuid.uuid128, com_svc_uuid, 16) == 0) {
            s_svc_start = p->search_res.start_handle;
            s_svc_end = p->search_res.end_handle;
        }
        break;
    case ESP_GATTC_SEARCH_CMPL_EVT: {
        if (s_svc_start == 0) {
            ESP_LOGW(TAG, "speaker has no excelpoint.com service?!");
            esp_ble_gattc_close(gattc_if, s_conn_id);
            break;
        }
        esp_gattc_char_elem_t ch;
        uint16_t cnt = 1;
        esp_bt_uuid_t u = { .len = ESP_UUID_LEN_128 };
        memcpy(u.uuid.uuid128, com_c1_uuid, 16);
        if (esp_ble_gattc_get_char_by_uuid(gattc_if, s_conn_id, s_svc_start, s_svc_end, u, &ch, &cnt) == ESP_GATT_OK && cnt) {
            s_h_notify = ch.char_handle;
        }
        cnt = 1;
        memcpy(u.uuid.uuid128, com_c2_uuid, 16);
        if (esp_ble_gattc_get_char_by_uuid(gattc_if, s_conn_id, s_svc_start, s_svc_end, u, &ch, &cnt) == ESP_GATT_OK && cnt) {
            s_h_write = ch.char_handle;
            s_write_rsp = (ch.properties & ESP_GATT_CHAR_PROP_BIT_WRITE) != 0;
        }
        ESP_LOGW(TAG, "excelpoint.com: notify 0x%04x, write 0x%04x (%s)", s_h_notify, s_h_write,
                 s_write_rsp ? "with response" : "no response");
        if (s_h_notify) {
            esp_ble_gattc_register_for_notify(gattc_if, s_peer, s_h_notify);
        }
        break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        esp_gattc_descr_elem_t d;
        uint16_t cnt = 1;
        esp_bt_uuid_t cccd = { .len = ESP_UUID_LEN_16, .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG };
        if (esp_ble_gattc_get_descr_by_char_handle(gattc_if, s_conn_id, p->reg_for_notify.handle, cccd, &d, &cnt) == ESP_GATT_OK && cnt) {
            uint8_t on[2] = {0x01, 0x00};
            esp_ble_gattc_write_char_descr(gattc_if, s_conn_id, d.handle, 2, on,
                                           ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        }
        break;
    }
    case ESP_GATTC_WRITE_DESCR_EVT:
        ESP_LOGI(TAG, "notifications on (status 0x%x)", p->write.status);
        if (s_pending_join) {
            v5_scan_start();            /* straight to: speaker scans -> finds us -> join */
        } else {
            s_state = ST_PROBING;
            v5_probe();
        }
        break;
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p->write.status != ESP_GATT_OK) {
            ESP_LOGW(TAG, "write failed, status 0x%x (insufficient auth? retrying after pairing)", p->write.status);
        }
        break;
    case ESP_GATTC_NOTIFY_EVT:
        v5_on_notify(p->notify.value, p->notify.value_len);
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGW(TAG, "control link closed (reason 0x%x)%s", p->disconnect.reason,
                 s_state == ST_DONE ? " - speaker stays on our Auracast; watching its beacon"
                                    : " - watching for the next button press");
        s_svc_start = s_svc_end = s_h_notify = s_h_write = 0;
        s_state = ST_IDLE;
        break;
    default:
        break;
    }
}

static void assistant_task(void *arg)
{
    TickType_t state_since = xTaskGetTickCount();
    state_t last = s_state;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (s_state != last) {
            last = s_state;
            state_since = xTaskGetTickCount();
        }
        TickType_t age = xTaskGetTickCount() - state_since;
        /* While phone music streams, stop the LE scan: each 30 ms scan window
         * blocks the phone's A2DP link, which then can't carry bitpool 53.
         * Native join doesn't need it - speakers find us by our own adverts. */
        if (s_state == ST_SCANNING && a2dp_bridge_streaming()) {
            ESP_LOGW(TAG, "phone music streaming -> pausing the JBL speaker scan (frees airtime for A2DP)");
            s_state = ST_SCAN_PAUSED;
            esp_ble_gap_stop_ext_scan();
            continue;
        }
        if (s_state == ST_SCAN_PAUSED) {
            if (!a2dp_bridge_streaming()) {
                ESP_LOGW(TAG, "phone music stopped -> resuming the JBL speaker scan");
                s_state = ST_IDLE;
            }
            continue;
        }
        switch (s_state) {
        case ST_IDLE:
            if (s_gattc_if != ESP_GATT_IF_NONE) {
                ESP_LOGI(TAG, "Hosting; watching for an Xtreme 5 whose Auracast button is pressed...");
                s_state = ST_SCANNING;
                esp_ble_gap_set_ext_scan_params(&s_scan_params);
            }
            break;
        case ST_CONNECTING:
            if (age > pdMS_TO_TICKS(15000)) {
                ESP_LOGW(TAG, "connect timed out - rescanning");
                esp_ble_gattc_close(s_gattc_if, s_conn_id);
                s_state = ST_IDLE;
            }
            break;
        case ST_SEARCHING:
            if (!s_join_sent && xTaskGetTickCount() - s_search_start > pdMS_TO_TICKS(JOIN_FALLBACK_MS)) {
                ESP_LOGW(TAG, "speaker didn't list our broadcast - sending join with our address anyway");
                v5_join(s_own_addr, 0 /* public */);
            }
            break;
        case ST_JOINING:
            if (age > pdMS_TO_TICKS(30000)) {
                ESP_LOGW(TAG, "no join confirmation - re-scanning on the speaker");
                v5_scan_start();
            }
            break;
        case ST_PROBING:
            if (age > pdMS_TO_TICKS(2500)) {
                if (s_pending_join) {
                    v5_scan_start();
                } else {
                    s_close_sent = false;
                    s_state = ST_DONE;          /* drop the link, keep watching */
                }
            }
            break;
        case ST_STOPPING:
            if (age > pdMS_TO_TICKS(3000)) {
                s_state = ST_IDLE;              /* scan-stop event never came */
            }
            break;
        case ST_DONE:
            /* Like a real master: hand-off done, drop the control link. */
            if (!s_close_sent && age > pdMS_TO_TICKS(3000)) {
                s_close_sent = true;
                s_cooldown_until = xTaskGetTickCount() + pdMS_TO_TICKS(30000);
                if (KEEP_LINK_AFTER_JOIN) {
                    ESP_LOGW(TAG, "keeping the link to speaker 0x%04x open; watching for more speakers", s_cur_crc);
                    s_state = ST_IDLE;          /* resume scanning; link stays up */
                } else {
                    esp_ble_gattc_close(s_gattc_if, s_conn_id);
                }
            }
            break;
        default:
            break;
        }
    }
}

esp_err_t jbl_assistant_start(uint16_t own_mac_crc, const char *own_broadcast_name)
{
    s_own_crc = own_mac_crc;
    strncpy(s_own_name, own_broadcast_name, sizeof(s_own_name) - 1);
    memcpy(s_own_addr, esp_bt_dev_get_address(), 6);
    esp_ble_gatt_set_local_mtu(247);
    esp_ble_gattc_register_callback(gattc_cb);
    esp_ble_gattc_app_register(GATTC_APP_ID);
    app_task_cold("jbl_assist", assistant_task, 4096, 5);   /* core 1, PSRAM stack: state machine + GATTC calls only, no flash */
    ESP_LOGW(TAG, "JBL master mode: '%s' is hosting; press Auracast on an Xtreme 5 to join it",
             own_broadcast_name);
    return ESP_OK;
}
