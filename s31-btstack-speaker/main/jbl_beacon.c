/*
 * JBL host beacon on BTstack (port of jbl_beacon_* in the Bluedroid build's
 * main.c and beacon_* in its bluedroid/adv.c).
 *
 * Copy of the Xtreme 5's beacon on its second address, own values:
 *   PDU       legacy ADV_IND (connectable + scannable) on its own extended set,
 *             SID 1, 1M, all channels, no TX power preference, scan request
 *             notifications on (new scanners are logged)
 *   address   random static, per speaker (s31_ident_beacon_addr: from the Bluetooth address)
 *   interval  20 ms (as the Xtreme 5), 100 ms while phone music streams so the
 *             BIG and the A2DP link get the airtime
 *   adv       Service Data 0xFDDF, V5 layout (com/harman/v5protocol/discover/a.java):
 *     [0-1] PID 2169 (Xtreme 5), byte-reversed on air   69 21
 *     [2-3] field-presence mask (LE)                     0x97C0 (+ b14 with the phone name)
 *           b15 colorID, b14 srcName1Crc, b12 btMacAddressCrc, b10 battery,
 *           b9 deviceMiscInfo, b8 partyMethodInfo, b7 partyInfo, b6 stereoGroupId
 *     colorID (s31_ident_color) | [srcName1Crc] | btMacAddressCrc = CRC-16/ARC of OUR Classic MAC
 *     battery 0x50 (80 %) | misc 0x9080 (connectable | LED | BR/EDR; b15 SET: iOS JBL One needs it -
 *     without it the iOS app never talks to us; the old Bluedroid build cleared it for Android)
 *     partyMethod 0xC0 (Auracast | long-lasting stereo) | partyInfo | stereoGroupId 00
 *   scan rsp  Complete Local Name "S31 JBL Xtreme 5"
 * A peer that connects stops the set (legacy connectable); it is re-armed when
 * that peer disconnects, as in the Bluedroid build.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "btstack.h"

#include "esp_log.h"

#include "jbl.h"
#include "jbl_beacon.h"
#include "jbl_v5.h"
#include "s31_ident.h"

static const char *TAG = "JBL_BEACON";

#define JBL_PID           (s31_ident_model())   /* the model this board is (V5 family) */
#define JBL_ADV_MASK      0x97C0
#define JBL_MASK_SRCNAME  (1u << 14)
#define JBL_BATTERY       0x50
/* misc: b15 connectable (over LE), b12 LED, b7 BR/EDR. Experiment for iOS JBL One,
 * which can only reach a speaker over LE: b15 SET (the Android app ignores it for
 * portables, where it uses BR/EDR GATT). 0x1080 = the old value with b15 clear. */
#define JBL_MISC_INFO     0x9080
#define JBL_PARTY_METHOD  0xC0

#define BEACON_SID           1
#define BEACON_IDLE_UNITS    32      /* 20 ms / 0.625 ms */
#define BEACON_STREAM_UNITS  160     /* 100 ms */
/* 1 = advertise again as soon as a peer connected (the controller allows a
 * second connection); 0 = as Bluedroid: re-arm when that peer leaves. */
#define BEACON_REARM_WHILE_CONNECTED 0

static bd_addr_t s_addr = { 0xC4, 0x58, 0x34, 0x53, 0x31, 0x04 };

static le_advertising_set_t s_set;
static le_extended_advertising_parameters_t s_params = {
    .advertising_event_properties = 0x13,          /* legacy | scannable | connectable = ADV_IND */
    .primary_advertising_interval_min = BEACON_IDLE_UNITS,
    .primary_advertising_interval_max = BEACON_IDLE_UNITS,
    .primary_advertising_channel_map = 0x07,
    .own_address_type = BD_ADDR_TYPE_LE_RANDOM,
    .peer_address_type = BD_ADDR_TYPE_LE_PUBLIC,
    .peer_address = { 0 },
    .advertising_filter_policy = 0,
    .advertising_tx_power = 127,                   /* no preference */
    .primary_advertising_phy = 1,                  /* LE 1M */
    .secondary_advertising_max_skip = 0,
    .secondary_advertising_phy = 1,
    .advertising_sid = BEACON_SID,
    .scan_request_notification_enable = 1,
};

static uint8_t s_handle = 0xFF;
static bool s_started;
static bool s_installed;
static bool s_stopped_by_con;                      /* set terminated by a connection */
static hci_con_handle_t s_beacon_con = HCI_CON_HANDLE_INVALID;
static uint16_t s_mac_crc;
static jbl_beacon_state_t s_st;
static uint8_t s_adv[31];                          /* BTstack keeps the pointers */
static uint8_t s_adv_len;
static uint8_t s_rsp[31];
static uint8_t s_rsp_len;
static btstack_packet_callback_registration_t s_hci_reg;

static uint32_t s_data_updates, s_interval_changes, s_rearms, s_peer_cons, s_scanners_seen;
static bool s_app_connected;                       /* misc b6: JBL One link up */

/* Service Data AD for the current state; returns its length (<= 31). */
static uint8_t build_adv(uint8_t *adv)
{
    uint8_t *p = adv + 4;
    int i = 0;
    uint16_t mask = JBL_ADV_MASK | (s_st.name_known ? JBL_MASK_SRCNAME : 0);
    p[i++] = JBL_PID & 0xFF;          p[i++] = JBL_PID >> 8;            /* 69 21 */
    p[i++] = mask & 0xFF;             p[i++] = mask >> 8;               /* c0 97 / c0 d7 */
    p[i++] = s31_ident_color();                                        /* colorID (set when flashing) */
    if (s_st.name_known) {
        p[i++] = s_st.name_crc & 0xFF; p[i++] = s_st.name_crc >> 8;     /* srcName1Crc */
    }
    p[i++] = s_mac_crc & 0xFF;        p[i++] = s_mac_crc >> 8;          /* MAC CRC */
    p[i++] = JBL_BATTERY;
    uint16_t misc = JBL_MISC_INFO | (s_app_connected ? 0x0040 : 0);
    p[i++] = misc & 0xFF;             p[i++] = misc >> 8;
    p[i++] = JBL_PARTY_METHOD;
    p[i++] = s_st.party_info & 0xFF;  p[i++] = s_st.party_info >> 8;
    p[i++] = s_st.stereo_gid;                                          /* stereoGroupId */
    adv[0] = (uint8_t)(3 + i);
    adv[1] = BLUETOOTH_DATA_TYPE_SERVICE_DATA_16_BIT_UUID;
    adv[2] = 0xDF;
    adv[3] = 0xFD;
    return (uint8_t)(4 + i);
}

/* scan rsp: Complete Local Name = the speaker name (JBL One rename, persisted) */
static void build_rsp(void)
{
    const char *nm = jbl_device_name();
    size_t nl = strlen(nm);
    if (nl > sizeof(s_rsp) - 2) nl = sizeof(s_rsp) - 2;
    s_rsp_len = (uint8_t)(2 + nl);
    s_rsp[0] = (uint8_t)(nl + 1);                      /* len incl. type byte */
    s_rsp[1] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
    memcpy(s_rsp + 2, nm, nl);
}

void jbl_beacon_name_changed(void)
{
    build_rsp();
    if (s_installed) {
        gap_extended_advertising_set_scan_response_data(s_handle, s_rsp_len, s_rsp);
        ESP_LOGW(TAG, "beacon name now '%.*s'", s_rsp_len - 2, (const char *)s_rsp + 2);
    }
}

static uint16_t s_force_units;                     /* console 'beacon_ms': 0 = automatic */
static uint32_t s_scan_reqs, s_scan_reqs_logged;   /* every scan request on the beacon */

static uint16_t interval_units(bool streaming)
{
    if (s_force_units) return s_force_units;
    return streaming ? BEACON_STREAM_UNITS : BEACON_IDLE_UNITS;
}

static void apply_interval(void)
{
    s_params.primary_advertising_interval_min = interval_units(s_st.streaming);
    s_params.primary_advertising_interval_max = interval_units(s_st.streaming);
    gap_extended_advertising_set_params(s_handle, &s_params);   /* BTstack stops / restarts the set */
    s_interval_changes++;
    ESP_LOGI(TAG, "JBL beacon interval now %u ms", (unsigned)(interval_units(s_st.streaming) * 5 / 8));
}

void jbl_beacon_test_props(uint8_t props)
{
    s_params.advertising_event_properties = props;
    if (s_started) apply_interval();
    ESP_LOGW(TAG, "beacon PDU properties now 0x%02x", props);
}

void jbl_beacon_force_interval_ms(unsigned ms)
{
    s_force_units = ms ? (uint16_t)(ms * 8 / 5) : 0;
    if (s_force_units && s_force_units < 32) s_force_units = 32;
    if (s_started) apply_interval();
}

static void log_payload(void)
{
    char h[3 * 31 + 1];
    int o = 0;
    for (int k = 4; k < s_adv_len; k++) {
        o += snprintf(h + o, sizeof(h) - (size_t)o, "%02x ", s_adv[k]);
    }
    h[o] = 0;
    ESP_LOGW(TAG, "JBL beacon 0xFDDF payload: %s", h);
}

static void rearm(void)
{
    if (!s_installed) {
        return;
    }
    gap_extended_advertising_start(s_handle, 0, 0);
    s_stopped_by_con = false;
    s_rearms++;
    ESP_LOGI(TAG, "JBL host beacon re-armed");
}

static void hci_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET || !s_started) {
        return;
    }
    switch (hci_event_packet_get_type(packet)) {
    case HCI_EVENT_META_GAP:
        switch (hci_event_gap_meta_get_subevent_code(packet)) {
        case GAP_SUBEVENT_ADVERTISING_SET_INSTALLED:
            if (gap_subevent_advertising_set_installed_get_advertisement_handle(packet) == s_handle) {
                uint8_t st = gap_subevent_advertising_set_installed_get_status(packet);
                if (st != ERROR_CODE_SUCCESS) {
                    ESP_LOGE(TAG, "beacon set (handle %u) rejected by the controller: 0x%02x", s_handle, st);
                } else {
                    ESP_LOGW(TAG, "JBL host beacon started (handle %u) at %s, %u ms", s_handle,
                             bd_addr_to_str(s_addr), (unsigned)(interval_units(s_st.streaming) * 5 / 8));
                }
            }
            break;
        case GAP_SUBEVENT_LE_CONNECTION_COMPLETE:
            if (gap_subevent_le_connection_complete_get_status(packet) != ERROR_CODE_SUCCESS) {
                ESP_LOGW(TAG, ">>> LE connection attempt FAILED, status 0x%02x (adv handle %u)",
                         gap_subevent_le_connection_complete_get_status(packet),
                         gap_subevent_le_connection_complete_get_advertising_handle(packet));
            } else {
                bd_addr_t pa;
                gap_subevent_le_connection_complete_get_peer_address(packet, pa);
                ESP_LOGW(TAG, ">>> LE connection from %s (adv handle %u, ours %u)", bd_addr_to_str(pa),
                         gap_subevent_le_connection_complete_get_advertising_handle(packet), s_handle);
            }
            if (gap_subevent_le_connection_complete_get_status(packet) == ERROR_CODE_SUCCESS &&
                gap_subevent_le_connection_complete_get_advertising_handle(packet) == s_handle) {
                bd_addr_t a;
                gap_subevent_le_connection_complete_get_peer_address(packet, a);
                s_beacon_con = gap_subevent_le_connection_complete_get_connection_handle(packet);
                s_peer_cons++;
                ESP_LOGW(TAG, ">>> PEER CONNECTED TO JBL BEACON: %s (con 0x%04x)", bd_addr_to_str(a), s_beacon_con);
                if (BEACON_REARM_WHILE_CONNECTED || s_st.stereo_gid) rearm();   /* stereo setup: the partner looks for it */
            }
            break;
        default:
            break;
        }
        break;
    case HCI_EVENT_LE_META:
        switch (hci_event_le_meta_get_subevent_code(packet)) {
        case HCI_SUBEVENT_LE_ADVERTISING_SET_TERMINATED:
            if (hci_subevent_le_advertising_set_terminated_get_advertising_handle(packet) != s_handle) {
                break;
            }
            if (hci_subevent_le_advertising_set_terminated_get_status(packet) == ERROR_CODE_SUCCESS) {
                s_beacon_con = hci_subevent_le_advertising_set_terminated_get_connection_handle(packet);
                s_stopped_by_con = true;     /* re-armed when the peer leaves */
                ESP_LOGI(TAG, "beacon paused: connection 0x%04x came in on it", s_beacon_con);
                if (BEACON_REARM_WHILE_CONNECTED || s_st.stereo_gid) rearm();   /* stereo setup: the partner looks for it */
            } else {
                ESP_LOGW(TAG, "beacon set terminated, status 0x%02x - re-arming",
                         hci_subevent_le_advertising_set_terminated_get_status(packet));
                rearm();
            }
            break;
        case HCI_SUBEVENT_LE_SCAN_REQUEST_RECEIVED: {
            /* Log each scanner address the first time it is seen. */
            static bd_addr_t seen[32];
            static int n_seen;
            if (hci_subevent_le_scan_request_received_get_advertising_handle(packet) != s_handle) {
                break;
            }
            bd_addr_t a;
            hci_subevent_le_scan_request_received_get_scanner_address(packet, a);
            s_scan_reqs++;
            int k;
            for (k = 0; k < n_seen; k++) {
                if (bd_addr_cmp(seen[k], a) == 0) {
                    break;
                }
            }
            if (k == n_seen) {
                memcpy(seen[s_scanners_seen % 32], a, sizeof(bd_addr_t));
                if (n_seen < 32) {
                    n_seen++;
                }
                s_scanners_seen++;
                ESP_LOGW(TAG, ">>> NEW SCANNER on the beacon: %s (type %u)", bd_addr_to_str(a),
                         hci_subevent_le_scan_request_received_get_scanner_address_type(packet));
            }
            break;
        }
        default:
            break;
        }
        break;
    case HCI_EVENT_DISCONNECTION_COMPLETE:
        if (hci_event_disconnection_complete_get_connection_handle(packet) == s_beacon_con) {
            s_beacon_con = HCI_CON_HANDLE_INVALID;
            if (s_stopped_by_con) {
                rearm();
            }
        }
        break;
    default:
        break;
    }
}

void jbl_beacon_setup(void)
{
    s31_ident_beacon_addr(s_addr);                 /* per speaker, not one address for all boards */
    s_hci_reg.callback = &hci_handler;
    hci_add_event_handler(&s_hci_reg);
}

void jbl_beacon_start(uint16_t mac_crc, const jbl_beacon_state_t *st)
{
    if (s_started) {
        return;
    }
    s_mac_crc = mac_crc;
    s_st = *st;
    if (!hci_le_extended_advertising_supported()) {
        ESP_LOGE(TAG, "controller has no LE extended advertising - JBL beacon not started");
        return;
    }
    s_adv_len = build_adv(s_adv);
    build_rsp();

    s_params.primary_advertising_interval_min = interval_units(s_st.streaming);
    s_params.primary_advertising_interval_max = interval_units(s_st.streaming);
    uint8_t err = gap_extended_advertising_setup(&s_set, &s_params, &s_handle);
    if (err != ERROR_CODE_SUCCESS) {
        ESP_LOGE(TAG, "no advertising set for the JBL beacon (0x%02x)", err);
        return;
    }
    s_started = true;
    s_installed = true;
    gap_extended_advertising_set_random_address(s_handle, s_addr);
    gap_extended_advertising_set_adv_data(s_handle, s_adv_len, s_adv);
    gap_extended_advertising_set_scan_response_data(s_handle, s_rsp_len, s_rsp);
    gap_extended_advertising_start(s_handle, 0, 0);
    log_payload();
}

void jbl_beacon_set_app_connected(bool on)
{
    if (on == s_app_connected) return;
    s_app_connected = on;
    if (s_started) {
        jbl_beacon_state_t st = s_st;
        s_adv_len = 0;                             /* force the data update */
        jbl_beacon_update(&st);
    }
}

void jbl_beacon_update(const jbl_beacon_state_t *st)
{
    if (!s_started) {
        s_st = *st;                                    /* used by jbl_beacon_start */
        return;
    }
    bool interval_changed = st->streaming != s_st.streaming;
    s_st = *st;
    if (s_st.stereo_gid && s_stopped_by_con) rearm();   /* stereo setup: back on air although connected */

    uint8_t adv[31];
    uint8_t len = build_adv(adv);
    if (len != s_adv_len || memcmp(adv, s_adv, len) != 0) {
        memcpy(s_adv, adv, len);
        s_adv_len = len;
        gap_extended_advertising_set_adv_data(s_handle, s_adv_len, s_adv);
        s_data_updates++;
        log_payload();
    }
    if (interval_changed && !s_force_units) {
        apply_interval();
    }
}

void jbl_beacon_log_stats(void)
{
    ESP_LOGI(TAG, "beacon %s handle %u, %u ms, partyInfo 0x%04x%s | data updates %lu, interval changes %lu, "
             "peer connects %lu, re-arms %lu, scanners %lu, scan requests %lu (+%lu)",
             !s_started ? "OFF" : s_stopped_by_con ? "PAUSED (peer connected)" : "on", s_handle,
             (unsigned)(interval_units(s_st.streaming) * 5 / 8), s_st.party_info,
             s_st.name_known ? " +srcName1Crc" : "", (unsigned long)s_data_updates,
             (unsigned long)s_interval_changes, (unsigned long)s_peer_cons, (unsigned long)s_rearms,
             (unsigned long)s_scanners_seen, (unsigned long)s_scan_reqs,
             (unsigned long)(s_scan_reqs - s_scan_reqs_logged));
    s_scan_reqs_logged = s_scan_reqs;
}

void jbl_beacon_pause(bool pause)
{
    if (s_handle == 0xFF) {
        return;
    }
    if (pause) {
        gap_extended_advertising_stop(s_handle);
    } else {
        gap_extended_advertising_start(s_handle, 0, 0);
    }
}
