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
    uint16_t types;                               /* packet types set (0 = not yet) */
    bool had_a2dp;                                /* A2DP ran on it (and may be gone now) */
    bool ready;                                   /* encrypted (or 4 s up): our tuning may start */
    uint32_t up_ms;
    bool sniff_req;                               /* we asked for sniff (standby phone) */
    uint8_t mode;                                 /* 0 active, 2 sniff (HCI Mode Change) */
    uint8_t sniff_fails;
    uint32_t sniff_hold_until;                    /* no new sniff request before this */
    uint32_t sniff_since;                         /* entered sniff at */
} s_links[MAX_ACL];

/* The standby phone (multipoint: linked, not the one playing) sniffs while the other
 * phone streams. Its 50 ms poll was cheap alone, but next to a broadcast with the S31 as
 * the streaming phone's slave (no role switch while broadcasting) it left the streaming
 * link 60-124 ms delivery gaps and ~1 underrun/s; in sniff it costs one anchor per
 * ~0.5 s. The Xtreme 5 sniffs idle links too. The phone leaves sniff by itself when it
 * has something to send (play, AVRCP); we leave it when that phone becomes the audio one
 * or the other stops streaming. */
#define SNIFF_MIN_SLOTS     640                   /* 400 ms */
#define SNIFF_MAX_SLOTS     800                   /* 500 ms */
#define SNIFF_ATTEMPT       4
#define SNIFF_TIMEOUT       2
/* after the phone left sniff itself: it may have had nothing to send (seen: an Android phone left
 * sniff and stayed silent; 5 s of 50 ms polls there gave the streaming phone 6 underflows).
 * Play / AVRCP from it arrive within that second and make it the audio phone anyway. */
#define SNIFF_HOLD_MS       1000
#define SNIFF_HOLD_LONG_MS  4000                  /* it left again within 3 s of entering */
#define SNIFF_MAX_FAILS     3

/* Our tuning (packet types, QoS poll) waits until a new link is encrypted: sent right at
 * the connect, on top of a phone's Secure Connections pairing, it overflowed the
 * controller's per-link procedure list ("OLM_CONN: pend list full ... refuse") and the
 * phone gave up pairing (0x13). A link that never encrypts gets it after 4 s. */
#define TUNE_WAIT_MS 4000
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
static const char *pkt_types_str(uint16_t t, char *buf, size_t n);
static int s_qos_retries;                         /* QoS Setup refusals retried (reset per new link) */

void link_tune_refresh(const char *why)
{
    hci_con_handle_t audio = a2dp_in_con_handle();
    for (int i = 0; i < MAX_ACL; i++) {
        if (audio != HCI_CON_HANDLE_INVALID && s_links[i].h == audio) s_links[i].had_a2dp = true;
    }
    poll_apply(why);
}

/* The standby phone (not the playing one) never streams to us: no 5-slot packets on its link.
 * A 5-slot exchange (~3.7 ms) cannot be cut by the radio once started and took BIS slots of
 * the broadcast while that phone set up its profiles (~30 SDUs per connect). It gets every
 * packet type back when it becomes the playing phone. */
#define PKT_NO_5SLOT(t) (uint16_t)(((t) & ~0xC000u) | 0x3000u)    /* -DM5 -DH5, 2-DH5/3-DH5 "may not" */
static uint16_t pkt_types_wanted(int i)
{
    hci_con_handle_t audio = a2dp_in_con_handle();
    bool standby = link_count() > 1 && audio != HCI_CON_HANDLE_INVALID && s_links[i].h != audio;
    return standby ? PKT_NO_5SLOT(s_pkt_types) : s_pkt_types;
}

static void pkt_types_apply(int i, const char *why)
{
    uint16_t t = pkt_types_wanted(i);
    if (t == s_links[i].types || !hci_can_send_command_packet_now()) return;
    char buf[96];
    s_links[i].types = t;
    hci_send_cmd(&hci_change_connection_packet_type, s_links[i].h, t);
    ESP_LOGW(TAG, "ACL 0x%04x: packet types 0x%04x (%s) - %s", s_links[i].h, t, pkt_types_str(t, buf, sizeof(buf)), why);
}

static void link_ready(int i, const char *why)
{
    s_links[i].ready = true;
    s_links[i].types = 0;
    pkt_types_apply(i, why);
    poll_apply(why);
}

static bool sniff_wanted(int i)
{
    hci_con_handle_t audio = a2dp_in_con_handle();
    return link_count() > 1 && audio != HCI_CON_HANDLE_INVALID && s_links[i].h != audio &&
           a2dp_in_streaming() && !a2dp_in_link_streams_held(s_links[i].h) && s_links[i].sniff_fails < SNIFF_MAX_FAILS &&
           (int32_t)(btstack_run_loop_get_time_ms() - s_links[i].sniff_hold_until) >= 0;
}

/* returns true while the link is (going) in sniff: no QoS on it then */
static bool sniff_apply(int i)
{
    hci_con_handle_t h = s_links[i].h;
    bool want = sniff_wanted(i);
    if (want && !s_links[i].sniff_req) {
        if (!hci_can_send_command_packet_now()) return true;          /* next tick */
        hci_send_cmd(&hci_write_link_policy_settings, h, 0x0004);     /* sniff allowed on this link */
        gap_sniff_mode_enter(h, SNIFF_MIN_SLOTS, SNIFF_MAX_SLOTS, SNIFF_ATTEMPT, SNIFF_TIMEOUT);
        s_links[i].sniff_req = true;
        ESP_LOGW(TAG, "ACL 0x%04x: standby phone -> sniff (%u-%u ms) while the other phone streams", h,
                 SNIFF_MIN_SLOTS * 5 / 8, SNIFF_MAX_SLOTS * 5 / 8);
    } else if (!want && s_links[i].sniff_req) {
        s_links[i].sniff_req = false;
        if (s_links[i].mode == 2) gap_sniff_mode_exit(h);
        s_links[i].applied = 0;                                       /* its poll again */
        ESP_LOGW(TAG, "ACL 0x%04x: leaving sniff (%s)", h,
                 h == a2dp_in_con_handle() ? "now the audio phone" : "the other phone stopped");
    }
    return s_links[i].sniff_req || s_links[i].mode == 2;
}

static void poll_apply(const char *why)
{
    for (int i = 0; i < MAX_ACL; i++) {
        hci_con_handle_t h = s_links[i].h;
        if (h == HCI_CON_HANDLE_INVALID) continue;
        if (!s_links[i].ready) {
            if (btstack_run_loop_get_time_ms() - s_links[i].up_ms < TUNE_WAIT_MS) continue;
            link_ready(i, "link up 4 s, not encrypted");   /* calls poll_apply again: done */
            return;
        }
        pkt_types_apply(i, link_count() > 1 ? "multipoint" : why);
        if (sniff_apply(i)) continue;
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
        s_qos_retries = 0;
        s_acl = hci_event_connection_complete_get_connection_handle(packet);
        for (int i = 0; i < MAX_ACL; i++) {
            if (s_links[i].h == HCI_CON_HANDLE_INVALID || s_links[i].h == s_acl) {
                s_links[i].h = s_acl;
                s_links[i].applied = 0;
                s_links[i].had_a2dp = false;
                s_links[i].ready = false;
                s_links[i].up_ms = btstack_run_loop_get_time_ms();
                s_links[i].sniff_req = false;
                s_links[i].types = 0;
                s_links[i].mode = 0;
                s_links[i].sniff_fails = 0;
                s_links[i].sniff_hold_until = 0;
                break;
            }
        }
        s_pkt_types = hci_usable_acl_packet_types();
        a2dp_in_refresh_discoverable();
        break;
    case HCI_EVENT_ENCRYPTION_CHANGE:
    case HCI_EVENT_ENCRYPTION_CHANGE_V2:
        if (packet[2] == ERROR_CODE_SUCCESS && packet[5] != 0) {
            hci_con_handle_t eh = little_endian_read_16(packet, 3);
            for (int i = 0; i < MAX_ACL; i++) {
                if (s_links[i].h == eh && !s_links[i].ready) link_ready(i, "phone connected");
            }
        }
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
    case HCI_EVENT_COMMAND_STATUS:
        /* QoS Setup refused (0x12 right after a connect, before the link is set up): it was marked
         * applied - make every link ask again on the next tick */
        if (hci_event_command_status_get_command_opcode(packet) == 0x0807 &&
            hci_event_command_status_get_status(packet) != ERROR_CODE_SUCCESS) {
            /* 3 more tries in all, then the controller's default poll stays. The controller
             * refuses (0x12) or takes the same request depending on the link's state - not the
             * role, not the interval asked for (seen 2026-11-24 over ~200 links); audio is fine
             * either way, so this is a note, not an error */
            if (s_qos_retries < 3) {
                s_qos_retries++;
                for (int i = 0; i < MAX_ACL; i++) s_links[i].applied = 0;
            } else if (s_qos_retries == 3) {
                s_qos_retries++;
                ESP_LOGI(TAG, "QoS Setup refused 4 times (0x%02x): this link keeps the controller's poll interval",
                         hci_event_command_status_get_status(packet));
            }
        }
        break;
    case HCI_EVENT_MODE_CHANGE: {
        hci_con_handle_t mh = hci_event_mode_change_get_handle(packet);
        uint8_t st = hci_event_mode_change_get_status(packet);
        uint8_t mode = hci_event_mode_change_get_mode(packet);
        for (int i = 0; i < MAX_ACL; i++) {
            if (s_links[i].h != mh) continue;
            if (st != ERROR_CODE_SUCCESS) {
                s_links[i].sniff_req = false;
                s_links[i].sniff_fails++;
                s_links[i].sniff_hold_until = btstack_run_loop_get_time_ms() + 10000;
                s_links[i].applied = 0;
                ESP_LOGW(TAG, "ACL 0x%04x: sniff refused (0x%02x, %u/%u)", mh, st, s_links[i].sniff_fails,
                         SNIFF_MAX_FAILS);
                break;
            }
            s_links[i].mode = mode;
            if (mode == 2) {
                s_links[i].sniff_since = btstack_run_loop_get_time_ms();
                ESP_LOGW(TAG, "ACL 0x%04x: in sniff, anchor every %.1f ms", mh,
                         hci_event_mode_change_get_interval(packet) * 0.625);
            } else {
                if (s_links[i].sniff_req) {                  /* the phone left sniff itself */
                    s_links[i].sniff_req = false;
                    uint32_t now = btstack_run_loop_get_time_ms();
                    s_links[i].sniff_hold_until = now + (now - s_links[i].sniff_since < 3000 ? SNIFF_HOLD_LONG_MS
                                                                                            : SNIFF_HOLD_MS);
                }
                s_links[i].applied = 0;
                ESP_LOGW(TAG, "ACL 0x%04x: active mode again", mh);
            }
        }
        break;
    }
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
