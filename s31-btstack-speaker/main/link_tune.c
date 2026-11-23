/*
 * Radio link tuning (BT thread, core 1).
 *
 * The phone's A2DP stream, the Auracast BIG, the JBL ext/periodic adv and the
 * JBL One control link all share one radio. Everything here frees airtime for
 * the BIG and makes the phone's delivery steadier:
 *
 *  BR/EDR (phone):
 *   - all usable ACL packet types incl. EDR 3 Mbps (3-DH5, 1021 B) are allowed
 *     on the link; the S31 is master, so its LM sets the 3 Mbps packet table.
 *     With AVDTP MTU 1005 Android packs 8 SBC frames (bitpool 53) per 3-DH5
 *     instead of 5 per 2-DH5: ~40 % fewer media packets for the same audio.
 *   - QoS guaranteed, poll interval 24 slots (15 ms; was 10 as on the Bluedroid
 *     build: the phone is polled often enough to empty its queue before the
 *     next BIG event instead of bursting late.
 *   - inquiry scan (discoverable) off while the phone is connected; page scan
 *     stays on (connectable) so a second phone / JBL One can still connect.
 *
 *  LE (JBL One / peer GATT link, S31 peripheral):
 *   - 6 s after the connection (service discovery and the V5 handshake are
 *     done by then) the S31 asks for a 60-75 ms interval instead of the
 *     7.5-15 ms phones use while discovering.
 */
#include "link_tune.h"
#include "a2dp_in.h"
#include "auracast.h"
#include "le_audio.h"
#include "jbl_v5.h"
#include "hci_transport_s31_vhci.h"

#include <stdio.h>
#include <string.h>

#include "btstack.h"
#include "esp_log.h"

static const char *TAG = "LINK";

/* Tpoll 24 slots = 15 ms: room for ~67 3-DH5 exchanges/s (A2DP needs ~50).
 * Each exchange reserves 3.56 ms as master (re/controller_sched.md s7), so
 * 10 slots reserved ~57 % of the air and overlapped ~6 BIS subevents each. */
/* The phone can send one packet per poll. At 32 slots (20 ms = 50 polls/s) a phone
 * needing 43 (8-frame packets) has no room for retries - the iPhone's 3-DH5
 * retries then pile up into flushed packets - and one sending 5-frame packets
 * (69/s, Galaxy S26 at a 663 B MTU) can't keep up at all. So: poll fast while no
 * BIG needs the air, and leave the BIG its share while hosting. */
#define POLL_SLOTS_IDLE     12                    /* 7.5 ms: 133 polls/s - any phone, any packet size */
#define POLL_SLOTS_HOSTING  24                    /* 15 ms: 67 polls/s - 43 needed + retries, BIG keeps its gap */
#define POLL_SLOTS_OTHER    80                    /* 50 ms: a second phone (JBL One only, or not playing) */
#define POLL_CHECK_MS       500
#define MAX_ACL             3
#define LE_UPDATE_DELAY_MS  6000
#define LE_ITVL_MIN         48                    /* 60 ms in 1.25 ms */
#define LE_ITVL_MAX         60                    /* 75 ms */
#define LE_LATENCY          0
#define LE_TIMEOUT          500                   /* 5 s in 10 ms */

static btstack_packet_callback_registration_t s_reg;
/* Every BR/EDR link is tuned on its own: the phone with A2DP gets the audio poll,
 * any other connected phone a slow best-effort one. With two phones up (one
 * playing, the other only running JBL One) the idle iPhone had asked for an
 * 8.75 ms poll and the controller granted it - more air than the phone that was
 * playing, which then lost packet after packet. */
static struct {
    hci_con_handle_t h;
    uint16_t applied;                             /* poll slots set, 0 = redo */
    bool had_a2dp;                                /* A2DP ran on it (and may be gone now) */
} s_links[MAX_ACL];
static hci_con_handle_t s_acl = HCI_CON_HANDLE_INVALID;   /* newest link */
static hci_con_handle_t s_le = HCI_CON_HANDLE_INVALID;
static btstack_timer_source_t s_le_timer;
static uint16_t s_pkt_types;
static btstack_timer_source_t s_poll_timer;

static int link_count(void)
{
    int n = 0;
    for (int i = 0; i < MAX_ACL; i++) n += s_links[i].h != HCI_CON_HANDLE_INVALID;
    return n;
}

int link_tune_classic_links(void) { return link_count(); }

static uint16_t poll_wanted(hci_con_handle_t h)
{
    hci_con_handle_t audio = a2dp_in_con_handle();
    if (link_count() > 1 && audio != HCI_CON_HANDLE_INVALID && h != audio) return POLL_SLOTS_OTHER;
    if (h != audio) {
        for (int i = 0; i < MAX_ACL; i++) {
            /* the phone dropped A2DP but kept the link (switching to LE Audio): at a
             * 7.5 ms guaranteed poll the controller had no air left to answer the
             * phone's LE connect (0x3E). A link setting A2DP up stays fast. */
            if (s_links[i].h == h && s_links[i].had_a2dp) return POLL_SLOTS_OTHER;
        }
    }
    if (h != audio && le_audio_phone_linked() &&
        (le_audio_starting() || (a2dp_in_ext_active() && auracast_state() != AURACAST_STATE_OFF))) {
        /* a classic link without A2DP (JBL One over BR/EDR GATT) while the phone sets up
         * its LE Audio stream, or while that stream feeds our broadcast: 7.5-15 ms
         * guaranteed polls took the air the CIS (and the BIG) need */
        return POLL_SLOTS_OTHER;
    }
    return auracast_state() == AURACAST_STATE_OFF ? POLL_SLOTS_IDLE : POLL_SLOTS_HOSTING;
}

static void poll_apply(const char *why);

void link_tune_refresh(const char *why)
{
    hci_con_handle_t audio = a2dp_in_con_handle();
    for (int i = 0; i < MAX_ACL; i++) {
        if (audio != HCI_CON_HANDLE_INVALID && s_links[i].h == audio) s_links[i].had_a2dp = true;
    }
    poll_apply(why);
}

static void poll_apply(const char *why)
{
    for (int i = 0; i < MAX_ACL; i++) {
        hci_con_handle_t h = s_links[i].h;
        if (h == HCI_CON_HANDLE_INVALID) continue;
        uint16_t want = poll_wanted(h);
        if (want == s_links[i].applied) continue;
        bool other = want == POLL_SLOTS_OTHER;
        gap_qos_set(h, other ? HCI_SERVICE_TYPE_BEST_EFFORT : HCI_SERVICE_TYPE_GUARANTEED,
                    other ? 4000 : 48000 /* B/s */, 0, (uint32_t)want * 625u, 0xFFFFFFFFu);
        s_links[i].applied = want;
        ESP_LOGW(TAG, "ACL 0x%04x: poll every %u slots (%.1f ms, %u polls/s) - %s", h, want, want * 0.625,
                 (unsigned)(1600u / want), other ? (link_count() > 1 ? "second phone, not the A2DP one" : "no A2DP on it now") : why);
    }
}

static void poll_tick(btstack_timer_source_t *ts)
{
    poll_apply(auracast_state() == AURACAST_STATE_OFF ? "no broadcast" : "broadcasting");
    btstack_run_loop_set_timer(ts, POLL_CHECK_MS);
    btstack_run_loop_add_timer(ts);
}

static const char *pkt_types_str(uint16_t t, char *buf, size_t n)
{
    /* HCI packet type field: bits 1,2,8,9,12,13 are "may NOT be used" (EDR) */
    snprintf(buf, n, "%s%s%s%s%s%s%s%s%s%s",
             (t & 0x0008) ? "DM1 " : "", (t & 0x0010) ? "DH1 " : "",
             (t & 0x0400) ? "DM3 " : "", (t & 0x0800) ? "DH3 " : "",
             (t & 0x4000) ? "DM5 " : "", (t & 0x8000) ? "DH5 " : "",
             (t & 0x0002) ? "" : "2-DHx ", (t & 0x0004) ? "" : "3-DHx ",
             (t & 0x1000) ? "" : "2-DH5 ", (t & 0x2000) ? "" : "3-DH5");
    return buf;
}

static void log_features(const char *who, const uint8_t *f)
{
    ESP_LOGW(TAG, "%s features %02x %02x %02x %02x %02x %02x %02x %02x | EDR 2M %s, 3M %s, 3-slot EDR %s, 5-slot EDR %s",
             who, f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7],
             (f[3] & 0x02) ? "yes" : "NO", (f[3] & 0x04) ? "yes" : "NO",
             (f[4] & 0x80) ? "yes" : "NO", (f[5] & 0x01) ? "yes" : "NO");
}

static uint16_t s_le_itvl, s_le_lat, s_le_to;     /* current LE link parameters (HCI units) */

static void le_update_cb(btstack_timer_source_t *ts)
{
    (void)ts;
    if (s_le == HCI_CON_HANDLE_INVALID) {
        return;
    }
    bool jbl = jbl_v5_is_link(s_le);
    if (le_audio_owns_link(s_le) || jbl) {
        /* an LE Audio phone: its own short interval keeps the link alive next to the CIS
         * and the JBL party BIG (at 60 ms it timed out, 0x08). JBL One's control link:
         * at 60 ms every Get/Set round trip (EQ, volume, ...) waited several intervals. */
        ESP_LOGW(TAG, "LE 0x%04x is %s: interval left as the phone set it (latency %u)", s_le,
                 jbl ? "the JBL One link" : "an LE Audio phone", s_le_lat);
        if (jbl && s_le_lat > 0) {
            /* peripheral latency lets the S31 sleep through events: every app command
             * (it waits for each reply) then waits too - same interval, latency 0 */
            int rc = gap_request_connection_parameter_update(s_le, s_le_itvl, s_le_itvl, 0, s_le_to);
            ESP_LOGW(TAG, "LE 0x%04x: JBL One link - asking latency 0 -> %d", s_le, rc);
        }
        return;
    }
    int rc = gap_request_connection_parameter_update(s_le, LE_ITVL_MIN, LE_ITVL_MAX, LE_LATENCY, LE_TIMEOUT);
    ESP_LOGW(TAG, "LE 0x%04x: asking for %u-%u ms interval (rc %d)", s_le,
             LE_ITVL_MIN * 5 / 4, LE_ITVL_MAX * 5 / 4, rc);
}

static void hci_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }
    char buf[96];
    switch (hci_event_packet_get_type(packet)) {
    case HCI_EVENT_COMMAND_COMPLETE:
        if (size >= 14 && hci_event_command_complete_get_command_opcode(packet) == 0x1003 && packet[5] == 0) {
            log_features("local", &packet[6]);
            ESP_LOGW(TAG, "usable ACL packet types 0x%04x: %s", hci_usable_acl_packet_types(),
                     pkt_types_str(hci_usable_acl_packet_types(), buf, sizeof(buf)));
        }
        break;
    case HCI_EVENT_CONNECTION_COMPLETE:
        if (hci_event_connection_complete_get_status(packet) != ERROR_CODE_SUCCESS ||
            hci_event_connection_complete_get_link_type(packet) != 0x01) {
            break;
        }
        s_acl = hci_event_connection_complete_get_connection_handle(packet);
        for (int i = 0; i < MAX_ACL; i++) {
            if (s_links[i].h == HCI_CON_HANDLE_INVALID || s_links[i].h == s_acl) {
                s_links[i].h = s_acl;
                s_links[i].applied = 0;
                s_links[i].had_a2dp = false;
                break;
            }
        }
        s_pkt_types = hci_usable_acl_packet_types();
        hci_send_cmd(&hci_change_connection_packet_type, s_acl, s_pkt_types);
        poll_apply("phone connected");
        a2dp_in_refresh_discoverable();
        ESP_LOGW(TAG, "ACL 0x%04x: packet types 0x%04x (%s), inquiry scan off",
                 s_acl, s_pkt_types, pkt_types_str(s_pkt_types, buf, sizeof(buf)));
        break;
    case HCI_EVENT_READ_REMOTE_SUPPORTED_FEATURES_COMPLETE:
        if (size >= 13 && packet[2] == 0) {
            log_features("phone", &packet[5]);
        }
        break;
    case HCI_EVENT_CONNECTION_PACKET_TYPE_CHANGED:
        ESP_LOGW(TAG, "ACL 0x%04x packet types changed (status 0x%02x): 0x%04x %s",
                 hci_event_connection_packet_type_changed_get_handle(packet),
                 hci_event_connection_packet_type_changed_get_status(packet),
                 hci_event_connection_packet_type_changed_get_packet_types(packet),
                 pkt_types_str(hci_event_connection_packet_type_changed_get_packet_types(packet), buf, sizeof(buf)));
        break;
    case HCI_EVENT_MAX_SLOTS_CHANGED:
        ESP_LOGW(TAG, "ACL 0x%04x max slots now %u", hci_event_max_slots_changed_get_handle(packet),
                 hci_event_max_slots_changed_get_lmp_max_slots(packet));
        break;
    case HCI_EVENT_QOS_SETUP_COMPLETE:
        if (size >= 21) {
            hci_con_handle_t qh = little_endian_read_16(packet, 3);
            uint32_t lat = little_endian_read_32(packet, 15);
            ESP_LOGW(TAG, "ACL 0x%04x QoS setup status 0x%02x: service %u, token %lu B/s, latency %lu us",
                     qh, packet[2], packet[6], (unsigned long)little_endian_read_32(packet, 7), (unsigned long)lat);
            for (int i = 0; i < MAX_ACL; i++) {
                if (s_links[i].h == qh && s_links[i].applied == POLL_SLOTS_OTHER &&
                    lat < (uint32_t)POLL_SLOTS_OTHER * 625u) {
                    s_links[i].applied = 0;           /* the idle phone asked for a fast poll: slow again next tick */
                }
            }
        }
        break;
    case HCI_EVENT_DISCONNECTION_COMPLETE: {
        hci_con_handle_t h = hci_event_disconnection_complete_get_connection_handle(packet);
        bool was_acl = false;
        for (int i = 0; i < MAX_ACL; i++) {
            if (s_links[i].h == h) {
                s_links[i].h = HCI_CON_HANDLE_INVALID;
                was_acl = true;
            }
        }
        if (was_acl) {
            for (int i = 0; i < MAX_ACL; i++) s_links[i].applied = 0;   /* the other phone may be the audio one now */
            if (link_count() == 0) {
                s_acl = HCI_CON_HANDLE_INVALID;
                a2dp_in_refresh_discoverable();      /* back on unless an LE Audio phone is linked */
                ESP_LOGW(TAG, "ACL gone");
            }
        }
        if (h == s_le) {
            s_le = HCI_CON_HANDLE_INVALID;
            btstack_run_loop_remove_timer(&s_le_timer);
        }
        break;
    }
    case HCI_EVENT_META_GAP:
        switch (hci_event_gap_meta_get_subevent_code(packet)) {
        case GAP_SUBEVENT_LE_CONNECTION_COMPLETE:
            if (gap_subevent_le_connection_complete_get_status(packet) == ERROR_CODE_SUCCESS &&
                gap_subevent_le_connection_complete_get_role(packet) == HCI_ROLE_SLAVE) {
                s_le = gap_subevent_le_connection_complete_get_connection_handle(packet);
                s_le_itvl = gap_subevent_le_connection_complete_get_conn_interval(packet);
                s_le_lat = gap_subevent_le_connection_complete_get_conn_latency(packet);
                s_le_to = gap_subevent_le_connection_complete_get_supervision_timeout(packet);
                ESP_LOGW(TAG, "LE 0x%04x up, interval %u.%02u ms", s_le,
                         gap_subevent_le_connection_complete_get_conn_interval(packet) * 5 / 4,
                         (gap_subevent_le_connection_complete_get_conn_interval(packet) * 125) % 100);
                btstack_run_loop_remove_timer(&s_le_timer);
                btstack_run_loop_set_timer_handler(&s_le_timer, le_update_cb);
                btstack_run_loop_set_timer(&s_le_timer, LE_UPDATE_DELAY_MS);
                btstack_run_loop_add_timer(&s_le_timer);
            }
            break;
        default:
            break;
        }
        break;
    case HCI_EVENT_LE_META:
        if (hci_event_le_meta_get_subevent_code(packet) == HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE &&
            hci_subevent_le_connection_update_complete_get_connection_handle(packet) == s_le) {
            uint16_t i = hci_subevent_le_connection_update_complete_get_conn_interval(packet);
            uint16_t lat = hci_subevent_le_connection_update_complete_get_conn_latency(packet);
            uint16_t to = hci_subevent_le_connection_update_complete_get_supervision_timeout(packet);
            ESP_LOGW(TAG, "LE 0x%04x interval now %u.%02u ms, latency %u, timeout %u ms (status 0x%02x)", s_le,
                     i * 5 / 4, (i * 125) % 100, lat, to * 10u,
                     hci_subevent_le_connection_update_complete_get_status(packet));
            if (hci_subevent_le_connection_update_complete_get_status(packet) == ERROR_CODE_SUCCESS) {
                s_le_itvl = i;
                s_le_lat = lat;
                s_le_to = to;
            }
            if (lat > 0 && jbl_v5_is_link(s_le)) {
                /* JBL One link: with peripheral latency the S31 may sleep through events and
                 * every app reply waits - ask for latency 0, same interval */
                int rc = gap_request_connection_parameter_update(s_le, i, i, 0, to);
                ESP_LOGW(TAG, "LE 0x%04x: JBL One on the LE Audio link - asking latency 0 -> %d", s_le, rc);
            }
        }
        break;
    default:
        break;
    }
}

void link_tune_setup(void)
{
    for (int i = 0; i < MAX_ACL; i++) s_links[i].h = HCI_CON_HANDLE_INVALID;
    s_reg.callback = &hci_handler;
    hci_add_event_handler(&s_reg);
    btstack_run_loop_set_timer_handler(&s_poll_timer, &poll_tick);
    btstack_run_loop_set_timer(&s_poll_timer, POLL_CHECK_MS);
    btstack_run_loop_add_timer(&s_poll_timer);
}
