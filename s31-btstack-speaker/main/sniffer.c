/*
 * Auracast / JBL party sniffer (S31_SNIFFER build, BT thread).
 *
 * Logs, for a JBL party master (Harman 0x0057 manufacturer data or a name with
 * "JBL") or, with SNIF_ANY_TARGET, any Auracast source:
 *   ADV   every distinct extended advertising payload, raw hex, with SID, PHYs,
 *         TX power, RSSI, PA interval and the time since the previous report
 *   PA    every distinct periodic advertising payload (fragments reassembled),
 *         raw hex, with ms timestamp and the number of identical reports between
 *   BIGINFO the raw BIGInfo report plus decoded fields
 *   ISO   after BIG sync: per BIS, every 2 s, packets valid / possibly invalid /
 *         lost (Packet_Status_Flag), sequence-number gaps, SDU lengths,
 *         timestamp deltas; and 30 s of LC3 frames of BIS 1 as
 *         "LC3F <seq> <ts> <ps> <len> <base64>" (tools: extract_rx.py)
 */
#include "sniffer.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "btstack.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "SNIF";

#ifndef SNIF_ANY_TARGET
#define SNIF_ANY_TARGET 0               /* 1 = also lock onto non-JBL Auracast sources */
#endif
#define SNIF_LC3_DUMP_FRAMES 3000       /* 30 s of BIS 1 */
#define MAX_BIS 4

static btstack_packet_callback_registration_t s_reg;
static enum { S_SCAN, S_W4_PA, S_W4_BIGINFO, S_W4_BIG, S_STREAM } s_state;
static bd_addr_t s_addr;
static bd_addr_type_t s_addr_type;
static uint8_t s_sid;
static hci_con_handle_t s_sync_handle = HCI_CON_HANDLE_INVALID;

static le_audio_big_sync_t s_big;
static le_audio_big_sync_params_t s_big_params;
static hci_con_handle_t s_bis[MAX_BIS];
static uint8_t s_num_bis;

/* adv / PA change tracking */
static uint8_t s_adv_last[2][255];
static uint16_t s_adv_last_len[2];
static int64_t s_adv_last_t[2];
static uint8_t s_pa_buf[1650], s_pa_last[1650];
static uint16_t s_pa_len, s_pa_last_len;
static uint32_t s_pa_same, s_pa_reports, s_pa_changes;
static int64_t s_pa_last_t;

/* ISO stats */
typedef struct {
    uint32_t valid, ps1, lost, gaps, zero_len, pkts;
    uint16_t last_seq;
    bool have_seq;
    uint32_t last_ts;
    uint32_t ts_dmin, ts_dmax;
    uint16_t len_min, len_max;
} bis_stat_t;
static bis_stat_t s_st[MAX_BIS];
static uint32_t s_dump_n;
static uint32_t s_n_ext, s_n_legacy, s_n_legacy_jbl;

static int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

static void log_hex(const char *what, const uint8_t *d, uint16_t n)
{
    static char line[3 * 64 + 1];
    for (uint16_t off = 0; off < n; off += 64) {
        uint16_t m = (uint16_t)(n - off > 64 ? 64 : n - off);
        for (uint16_t i = 0; i < m; i++) {
            snprintf(&line[3 * i], 4, "%02x ", d[off + i]);
        }
        line[3 * m] = 0;
        printf("%s +%03u: %s\n", what, off, line);
    }
}

static void b64(const uint8_t *in, int n, char *out)
{
    static const char t[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    int o = 0;
    for (int i = 0; i < n; i += 3) {
        uint32_t v = (uint32_t)in[i] << 16 | (i + 1 < n ? (uint32_t)in[i + 1] << 8 : 0) | (i + 2 < n ? in[i + 2] : 0);
        out[o++] = t[(v >> 18) & 63];
        out[o++] = t[(v >> 12) & 63];
        out[o++] = i + 1 < n ? t[(v >> 6) & 63] : '=';
        out[o++] = i + 2 < n ? t[v & 63] : '=';
    }
    out[o] = 0;
}

static void start_scan(void)
{
    s_state = S_SCAN;
    gap_set_scan_params(1 /* active: get scan responses too */, 0x30, 0x30, 0);
    gap_start_scan();
    ESP_LOGW(TAG, "scanning for a JBL party master%s...", SNIF_ANY_TARGET ? " (or any Auracast source)" : "");
}

static void on_ext_adv(const uint8_t *packet)
{
    bd_addr_t a;
    gap_event_extended_advertising_report_get_address(packet, a);
    uint8_t len = gap_event_extended_advertising_report_get_data_length(packet);
    const uint8_t *data = gap_event_extended_advertising_report_get_data(packet);
    uint8_t sid = gap_event_extended_advertising_report_get_advertising_sid(packet);
    uint16_t pa_itvl = gap_event_extended_advertising_report_get_periodic_advertising_interval(packet);

    bool baa = false, harman = false, jbl_name = false;
    uint32_t bid = 0;
    char name[40] = { 0 };
    ad_context_t c;
    for (ad_iterator_init(&c, len, data); ad_iterator_has_more(&c); ad_iterator_next(&c)) {
        uint8_t t = ad_iterator_get_data_type(&c), n = ad_iterator_get_data_len(&c);
        const uint8_t *d = ad_iterator_get_data(&c);
        if (t == BLUETOOTH_DATA_TYPE_SERVICE_DATA_16_BIT_UUID && n >= 5 && little_endian_read_16(d, 0) == 0x1852) {
            baa = true;
            bid = little_endian_read_24(d, 2);
        } else if (t == BLUETOOTH_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA && n >= 2 && little_endian_read_16(d, 0) == 0x0057) {
            harman = true;
        } else if (t == BLUETOOTH_DATA_TYPE_SERVICE_DATA_16_BIT_UUID && n >= 2 && little_endian_read_16(d, 0) == 0xFDDF) {
            harman = true;                       /* JBL V5 beacon */
        } else if (t == BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME || t == BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME || t == 0x30) {
            memcpy(name, d, n < sizeof(name) - 1 ? n : sizeof(name) - 1);
            if (strstr(name, "JBL") != NULL) {
                jbl_name = true;
            }
        }
    }
    bool target = (s_state != S_SCAN) ? (bd_addr_cmp(a, s_addr) == 0) : (baa && (harman || jbl_name || SNIF_ANY_TARGET));
    bool same_dev = s_state != S_SCAN && bd_addr_cmp(a, s_addr) == 0;
    if (!target && !same_dev && !(baa && s_state == S_SCAN)) {
        if (harman && s_state == S_SCAN) {
            /* other JBL adverts (e.g. the party beacon set) - log distinct ones */
            static uint8_t seen[16][32];
            static int n_seen;
            uint8_t key[32] = { 0 };
            memcpy(key, a, 6);
            memcpy(key + 6, data, len < 26 ? len : 26);
            for (int i = 0; i < n_seen; i++) {
                if (memcmp(seen[i], key, 32) == 0) {
                    return;
                }
            }
            if (n_seen < 16) {
                memcpy(seen[n_seen++], key, 32);
            }
            ESP_LOGW(TAG, "JBL adv (not a broadcast) %s sid %u evt 0x%02x rssi %d len %u", bd_addr_to_str(a), sid,
                     gap_event_extended_advertising_report_get_advertising_event_type(packet),
                     (int8_t)gap_event_extended_advertising_report_get_rssi(packet), len);
            log_hex("ADVX", data, len);
        }
        return;
    }
    if (baa && !target && s_state == S_SCAN) {
        static uint32_t logged[8];
        static int n_logged;
        for (int i = 0; i < n_logged; i++) {
            if (logged[i] == bid) {
                return;
            }
        }
        if (n_logged < 8) {
            logged[n_logged++] = bid;
        }
        ESP_LOGW(TAG, "other Auracast source (ignored): %s '%s' Broadcast_ID 0x%06" PRIx32, bd_addr_to_str(a), name, bid);
        return;
    }
    /* target device: log each distinct payload per scan/aux (index by SID parity) */
    int k = sid & 1;
    int64_t t = now_ms();
    if (len != s_adv_last_len[k] || memcmp(data, s_adv_last[k], len) != 0) {
        ESP_LOGW(TAG, "ADV %s type %u sid %u evt 0x%02x phy %u/%u txp %d rssi %d PA itvl %u (%u.%02u ms) bid 0x%06" PRIx32
                 " name '%s' len %u (+%lld ms since prev report)",
                 bd_addr_to_str(a), gap_event_extended_advertising_report_get_address_type(packet), sid,
                 gap_event_extended_advertising_report_get_advertising_event_type(packet),
                 gap_event_extended_advertising_report_get_primary_phy(packet),
                 gap_event_extended_advertising_report_get_secondary_phy(packet),
                 (int8_t)gap_event_extended_advertising_report_get_tx_power(packet),
                 (int8_t)gap_event_extended_advertising_report_get_rssi(packet), pa_itvl, pa_itvl * 5 / 4,
                 (pa_itvl * 125) % 100, bid, name, len, (long long)(t - s_adv_last_t[k]));
        log_hex("ADV", data, len);
        memcpy(s_adv_last[k], data, len);
        s_adv_last_len[k] = len;
    }
    s_adv_last_t[k] = t;

    if (s_state == S_SCAN && baa && pa_itvl != 0) {
        bd_addr_copy(s_addr, a);
        s_addr_type = (bd_addr_type_t)gap_event_extended_advertising_report_get_address_type(packet);
        s_sid = sid;
        s_state = S_W4_PA;
        ESP_LOGW(TAG, "TARGET %s '%s' Broadcast_ID 0x%06" PRIx32 " SID %u -> PA sync", bd_addr_to_str(a), name, bid, sid);
        gap_periodic_advertiser_list_clear();
        gap_periodic_advertiser_list_add(s_addr_type, s_addr, s_sid);
        gap_periodic_advertising_create_sync(0x01, s_sid, s_addr_type, s_addr, 0, 1000, 0);
    }
}

static void on_pa_report(const uint8_t *packet)
{
    uint8_t st = hci_subevent_le_periodic_advertising_report_get_data_status(packet);
    uint8_t n = hci_subevent_le_periodic_advertising_report_get_data_length(packet);
    const uint8_t *d = hci_subevent_le_periodic_advertising_report_get_data(packet);
    if (s_pa_len + n <= sizeof(s_pa_buf)) {
        memcpy(s_pa_buf + s_pa_len, d, n);
        s_pa_len += n;
    }
    if (st == 1) {
        return;                                  /* more fragments follow */
    }
    s_pa_reports++;
    int64_t t = now_ms();
    if (st == 2) {
        ESP_LOGW(TAG, "PA report truncated (%u B so far)", s_pa_len);
    }
    if (s_pa_len != s_pa_last_len || memcmp(s_pa_buf, s_pa_last, s_pa_len) != 0) {
        s_pa_changes++;
        ESP_LOGW(TAG, "PA t=%lld ms len %u rssi %d txp %d | %lu identical reports before, last change %lld ms ago",
                 (long long)t, s_pa_len, (int8_t)hci_subevent_le_periodic_advertising_report_get_rssi(packet),
                 (int8_t)hci_subevent_le_periodic_advertising_report_get_tx_power(packet), (unsigned long)s_pa_same,
                 (long long)(t - s_pa_last_t));
        log_hex("PA", s_pa_buf, s_pa_len);
        memcpy(s_pa_last, s_pa_buf, s_pa_len);
        s_pa_last_len = s_pa_len;
        s_pa_same = 0;
        s_pa_last_t = t;
    } else {
        s_pa_same++;
    }
    s_pa_len = 0;
}

static void on_biginfo(const uint8_t *packet, uint16_t size)
{
    if (s_state != S_W4_BIGINFO) {
        return;
    }
    log_hex("BIGINFO raw", packet, size);
    uint8_t nb = hci_subevent_le_biginfo_advertising_report_get_num_bis(packet);
    ESP_LOGW(TAG, "BIGINFO: num_bis %u, NSE %u, ISO interval %u (%u.%02u ms), BN %u, PTO %u, IRC %u, max_pdu %u, "
             "sdu_interval %lu us, max_sdu %u, PHY %u, framing %u, encryption %u",
             nb, hci_subevent_le_biginfo_advertising_report_get_nse(packet),
             hci_subevent_le_biginfo_advertising_report_get_iso_interval(packet),
             hci_subevent_le_biginfo_advertising_report_get_iso_interval(packet) * 5 / 4,
             (hci_subevent_le_biginfo_advertising_report_get_iso_interval(packet) * 125) % 100,
             hci_subevent_le_biginfo_advertising_report_get_bn(packet),
             hci_subevent_le_biginfo_advertising_report_get_pto(packet),
             hci_subevent_le_biginfo_advertising_report_get_irc(packet),
             hci_subevent_le_biginfo_advertising_report_get_max_pdu(packet),
             (unsigned long)hci_subevent_le_biginfo_advertising_report_get_sdu_interval(packet),
             hci_subevent_le_biginfo_advertising_report_get_max_sdu(packet),
             hci_subevent_le_biginfo_advertising_report_get_phy(packet),
             hci_subevent_le_biginfo_advertising_report_get_framing(packet),
             hci_subevent_le_biginfo_advertising_report_get_encryption(packet));
    if (hci_subevent_le_biginfo_advertising_report_get_encryption(packet)) {
        ESP_LOGE(TAG, "encrypted BIG - not syncing");
        return;
    }
    s_num_bis = nb > MAX_BIS ? MAX_BIS : nb;
    memset(&s_big_params, 0, sizeof(s_big_params));
    s_big_params.big_handle = 0;
    s_big_params.sync_handle = s_sync_handle;
    s_big_params.big_sync_timeout_10ms = 200;
    s_big_params.num_bis = s_num_bis;
    for (uint8_t i = 0; i < s_num_bis; i++) {
        s_big_params.bis_indices[i] = (uint8_t)(i + 1);
    }
    s_state = S_W4_BIG;
    uint8_t rc = gap_big_sync_create(&s_big, &s_big_params);
    ESP_LOGW(TAG, "BIG sync create (all %u BIS): 0x%02x", s_num_bis, rc);
}

static void hci_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }
    switch (hci_event_packet_get_type(packet)) {
    case GAP_EVENT_EXTENDED_ADVERTISING_REPORT:
        s_n_ext++;
        on_ext_adv(packet);
        break;
    case GAP_EVENT_ADVERTISING_REPORT: {
        s_n_legacy++;
        uint8_t len = gap_event_advertising_report_get_data_length(packet);
        const uint8_t *data = gap_event_advertising_report_get_data(packet);
        ad_context_t c;
        for (ad_iterator_init(&c, len, data); ad_iterator_has_more(&c); ad_iterator_next(&c)) {
            uint8_t ty = ad_iterator_get_data_type(&c);
            uint16_t id = ad_iterator_get_data_len(&c) >= 2 ? little_endian_read_16(ad_iterator_get_data(&c), 0) : 0;
            if ((ty == BLUETOOTH_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA && id == 0x0057) ||
                (ty == BLUETOOTH_DATA_TYPE_SERVICE_DATA_16_BIT_UUID && id == 0xFDDF)) {
                s_n_legacy_jbl++;
                static uint8_t last[31];
                static uint8_t last_len;
                if (len != last_len || memcmp(last, data, len) != 0) {
                    bd_addr_t a;
                    gap_event_advertising_report_get_address(packet, a);
                    ESP_LOGW(TAG, "JBL legacy adv %s type %u rssi %d len %u", bd_addr_to_str(a),
                             gap_event_advertising_report_get_advertising_event_type(packet),
                             (int8_t)gap_event_advertising_report_get_rssi(packet), len);
                    log_hex("ADVL", data, len);
                    memcpy(last, data, len);
                    last_len = len;
                }
            }
        }
        break;
    }
    case HCI_EVENT_LE_META:
        switch (hci_event_le_meta_get_subevent_code(packet)) {
        case HCI_SUBEVENT_LE_PERIODIC_ADVERTISING_SYNC_ESTABLISHMENT:
            if (hci_subevent_le_periodic_advertising_sync_establishment_get_status(packet) != 0) {
                ESP_LOGW(TAG, "PA sync failed 0x%02x, rescanning",
                         hci_subevent_le_periodic_advertising_sync_establishment_get_status(packet));
                start_scan();
                break;
            }
            s_sync_handle = hci_subevent_le_periodic_advertising_sync_establishment_get_sync_handle(packet);
            ESP_LOGW(TAG, "PA synced: handle 0x%04x, interval %u (%u.%02u ms), advertiser PHY %u, clock accuracy %u",
                     s_sync_handle, hci_subevent_le_periodic_advertising_sync_establishment_get_periodic_advertising_interval(packet),
                     hci_subevent_le_periodic_advertising_sync_establishment_get_periodic_advertising_interval(packet) * 5 / 4,
                     (hci_subevent_le_periodic_advertising_sync_establishment_get_periodic_advertising_interval(packet) * 125) % 100,
                     hci_subevent_le_periodic_advertising_sync_establishment_get_advertiser_phy(packet),
                     hci_subevent_le_periodic_advertising_sync_establishment_get_advertiser_clock_accuracy(packet));
            s_state = S_W4_BIGINFO;
            break;
        case HCI_SUBEVENT_LE_PERIODIC_ADVERTISING_REPORT:
            on_pa_report(packet);
            break;
        case HCI_SUBEVENT_LE_BIGINFO_ADVERTISING_REPORT:
            on_biginfo(packet, size);
            break;
        case HCI_SUBEVENT_LE_PERIODIC_ADVERTISING_SYNC_LOST:
            ESP_LOGW(TAG, "PA sync LOST -> rescanning");
            s_sync_handle = HCI_CON_HANDLE_INVALID;
            memset(s_adv_last_len, 0, sizeof(s_adv_last_len));
            s_pa_last_len = 0;
            start_scan();
            break;
        case HCI_SUBEVENT_LE_BIG_SYNC_LOST:
            ESP_LOGW(TAG, "BIG sync LOST, reason 0x%02x", packet[4]);
            s_state = S_W4_BIGINFO;
            break;
        default:
            break;
        }
        break;
    case HCI_EVENT_META_GAP:
        if (hci_event_gap_meta_get_subevent_code(packet) == GAP_SUBEVENT_BIG_SYNC_CREATED) {
            if (gap_subevent_big_sync_created_get_status(packet) != 0) {
                ESP_LOGW(TAG, "BIG sync failed 0x%02x", gap_subevent_big_sync_created_get_status(packet));
                s_state = S_W4_BIGINFO;
                break;
            }
            uint8_t n = gap_subevent_big_sync_created_get_num_bis(packet);
            for (uint8_t i = 0; i < n && i < MAX_BIS; i++) {
                s_bis[i] = gap_subevent_big_sync_created_get_bis_con_handles(packet, i);
            }
            memset(s_st, 0, sizeof(s_st));
            s_dump_n = 0;
            s_state = S_STREAM;
            ESP_LOGW(TAG, "BIG synced: %u BIS, ISO interval %u us -> recording", n,
                     gap_subevent_big_sync_created_get_iso_interval_1250us(packet) * 1250u);
        }
        break;
    default:
        break;
    }
}

static void iso_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    if (packet_type != HCI_ISO_DATA_PACKET || size < 4 || s_state != S_STREAM) {
        return;
    }
    uint16_t hdr = little_endian_read_16(packet, 0);
    hci_con_handle_t h = hdr & 0x0fff;
    bool ts_flag = (hdr >> 14) & 1;
    int b = -1;
    for (int i = 0; i < s_num_bis; i++) {
        if (s_bis[i] == h) {
            b = i;
        }
    }
    if (b < 0) {
        return;
    }
    uint16_t off = 4;
    uint32_t ts = 0;
    if (ts_flag) {
        ts = little_endian_read_32(packet, off);
        off += 4;
    }
    if (size < off + 4) {
        return;
    }
    uint16_t seq = little_endian_read_16(packet, off);
    uint16_t lp = little_endian_read_16(packet, off + 2);
    uint16_t sdu_len = lp & 0x0fff;
    uint8_t ps = (uint8_t)(lp >> 14);
    off += 4;
    bis_stat_t *s = &s_st[b];
    s->pkts++;
    if (ps == 0) s->valid++;
    else if (ps == 1) s->ps1++;
    else s->lost++;
    if (sdu_len == 0) s->zero_len++;
    if (s->have_seq && (uint16_t)(seq - s->last_seq) != 1) s->gaps++;
    if (sdu_len) {
        if (s->len_min == 0 || sdu_len < s->len_min) s->len_min = sdu_len;
        if (sdu_len > s->len_max) s->len_max = sdu_len;
    }
    if (ts_flag && s->have_seq) {
        uint32_t d = ts - s->last_ts;
        if (s->ts_dmin == 0 || d < s->ts_dmin) s->ts_dmin = d;
        if (d > s->ts_dmax) s->ts_dmax = d;
    }
    s->last_ts = ts;
    s->last_seq = seq;
    s->have_seq = true;
    if (b == 0 && s_dump_n < SNIF_LC3_DUMP_FRAMES) {
        static char line[400];
        uint16_t n = (uint16_t)(size - off < sdu_len ? size - off : sdu_len);
        b64(packet + off, n, line);
        printf("LC3F %u %lu %u %u %s\n", seq, (unsigned long)ts, ps, n, line);
        if (++s_dump_n == SNIF_LC3_DUMP_FRAMES) {
            ESP_LOGW(TAG, "LC3 dump of BIS 1 complete (%u frames)", SNIF_LC3_DUMP_FRAMES);
        }
    }
}

void sniffer_log_stats(void)
{
    if (s_state != S_STREAM) {
        ESP_LOGW(TAG, "state %d, adv reports ext %lu legacy %lu (JBL legacy %lu), PA reports %lu (changes %lu)", s_state,
                 (unsigned long)s_n_ext, (unsigned long)s_n_legacy, (unsigned long)s_n_legacy_jbl,
                 (unsigned long)s_pa_reports, (unsigned long)s_pa_changes);
        return;
    }
    for (int i = 0; i < s_num_bis; i++) {
        bis_stat_t *s = &s_st[i];
        ESP_LOGW(TAG, "ISO BIS%d 2s: pkts %lu valid %lu maybe-bad %lu LOST %lu seq gaps %lu empty %lu | SDU %u-%u B | ts delta %lu-%lu us",
                 i + 1, (unsigned long)s->pkts, (unsigned long)s->valid, (unsigned long)s->ps1, (unsigned long)s->lost,
                 (unsigned long)s->gaps, (unsigned long)s->zero_len, s->len_min, s->len_max,
                 (unsigned long)s->ts_dmin, (unsigned long)s->ts_dmax);
        uint16_t ls = s->last_seq;
        uint32_t lt = s->last_ts;
        bool hs = s->have_seq;
        memset(s, 0, sizeof(*s));
        s->last_seq = ls;
        s->last_ts = lt;
        s->have_seq = hs;
    }
    ESP_LOGI(TAG, "PA reports %lu (changes %lu)", (unsigned long)s_pa_reports, (unsigned long)s_pa_changes);
}

void sniffer_setup(void)
{
    s_reg.callback = &hci_handler;
    hci_add_event_handler(&s_reg);
    hci_register_iso_packet_handler(&iso_handler);
}

void sniffer_start(void)
{
    start_scan();
}
