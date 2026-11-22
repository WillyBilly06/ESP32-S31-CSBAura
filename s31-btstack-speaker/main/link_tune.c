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
 *   - QoS guaranteed, poll interval 10 slots (6.25 ms) as on the Bluedroid
 *     build: the phone is polled often enough to empty its queue before the
 *     next BIG event instead of bursting late.
 *   - inquiry scan (discoverable) off while the phone is connected; page scan
 *     stays on (connectable) so a second phone / JBL One can still connect.
 *
 *  LE (JBL One / peer GATT link, S31 peripheral):
 *   - 6 s after the connection (service discovery and the V5 handshake are
 *     done by then) the S31 asks for a 30-50 ms interval instead of the
 *     7.5-15 ms phones use while discovering.
 */
#include "link_tune.h"

#include <stdio.h>
#include <string.h>

#include "btstack.h"
#include "esp_log.h"

static const char *TAG = "LINK";

#define POLL_SLOTS          10                    /* Tpoll, 0.625 ms slots */
#define LE_UPDATE_DELAY_MS  6000
#define LE_ITVL_MIN         24                    /* 30 ms in 1.25 ms */
#define LE_ITVL_MAX         40                    /* 50 ms */
#define LE_LATENCY          0
#define LE_TIMEOUT          500                   /* 5 s in 10 ms */

static btstack_packet_callback_registration_t s_reg;
static hci_con_handle_t s_acl = HCI_CON_HANDLE_INVALID;
static hci_con_handle_t s_le = HCI_CON_HANDLE_INVALID;
static btstack_timer_source_t s_le_timer;
static uint16_t s_pkt_types;

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

static void le_update_cb(btstack_timer_source_t *ts)
{
    (void)ts;
    if (s_le == HCI_CON_HANDLE_INVALID) {
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
        s_pkt_types = hci_usable_acl_packet_types();
        hci_send_cmd(&hci_change_connection_packet_type, s_acl, s_pkt_types);
        gap_qos_set(s_acl, HCI_SERVICE_TYPE_GUARANTEED, 48000 /* B/s */, 0, POLL_SLOTS * 625, 0xFFFFFFFFu);
        gap_discoverable_control(0);
        ESP_LOGW(TAG, "ACL 0x%04x: packet types 0x%04x (%s), QoS poll %u slots, inquiry scan off",
                 s_acl, s_pkt_types, pkt_types_str(s_pkt_types, buf, sizeof(buf)), POLL_SLOTS);
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
            ESP_LOGW(TAG, "ACL 0x%04x QoS setup status 0x%02x: service %u, token %lu B/s, latency %lu us",
                     little_endian_read_16(packet, 3), packet[2], packet[6],
                     (unsigned long)little_endian_read_32(packet, 7), (unsigned long)little_endian_read_32(packet, 15));
        }
        break;
    case HCI_EVENT_DISCONNECTION_COMPLETE: {
        hci_con_handle_t h = hci_event_disconnection_complete_get_connection_handle(packet);
        if (h == s_acl) {
            s_acl = HCI_CON_HANDLE_INVALID;
            gap_discoverable_control(1);
            ESP_LOGW(TAG, "ACL gone: inquiry scan back on");
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
            ESP_LOGW(TAG, "LE 0x%04x interval now %u.%02u ms (status 0x%02x)", s_le, i * 5 / 4, (i * 125) % 100,
                     hci_subevent_le_connection_update_complete_get_status(packet));
        }
        break;
    default:
        break;
    }
}

void link_tune_setup(void)
{
    s_reg.callback = &hci_handler;
    hci_add_event_handler(&s_reg);
}
