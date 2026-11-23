/*
 * JBL personality glue: setup order, identity (macAddressCrc), LE security,
 * and the any-thread setters that feed the beacon / assistant on the BTstack
 * thread. See jbl.h.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "btstack.h"

#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "jbl.h"
#include "a2dp_in.h"
#include "jbl_assist.h"
#include "le_audio.h"
#include "jbl_beacon.h"
#include "jbl_gatt.h"
#include "jbl_util.h"
#include "jbl_v5.h"
#include "v5_eq.h"

static const char *TAG = "JBL";

static TaskHandle_t s_bt_task;
static volatile uint16_t s_mac_crc;
static volatile uint16_t s_party_info = JBL_PARTY_INFO_HOST;
static volatile bool s_streaming;
static volatile uint32_t s_name_word;              /* 0 = unknown, else 0x10000 | srcName1Crc */
static bool s_working;
static btstack_context_callback_registration_t s_apply_reg;
static btstack_packet_callback_registration_t s_sm_reg, s_hci_reg;

/* 1 = read the name of each phone that connects over BR/EDR and put its CRC in
 * the beacon (the Bluedroid build got it from its A2DP bridge). jbl_set_phone_name()
 * still overrides it. */
#define JBL_AUTO_PHONE_NAME 1
static bd_addr_t s_name_req_addr;
static bool s_name_req_active;

/* ---- helpers ------------------------------------------------------------------ */

uint16_t jbl_crc16_arc(const uint8_t *d, size_t n)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

bool jbl_on_bt_thread(void)
{
    return s_bt_task != NULL && xTaskGetCurrentTaskHandle() == s_bt_task;
}

void jbl_run_on_bt(btstack_context_callback_registration_t *reg)
{
    if (jbl_on_bt_thread()) {
        reg->callback(reg->context);
    } else if (s_bt_task != NULL) {
        btstack_run_loop_execute_on_main_thread(reg);
    }
    /* before jbl_setup: the stored values are picked up at start */
}

static void beacon_state(jbl_beacon_state_t *st)
{
    uint32_t nw = s_name_word;
    st->party_info = s_party_info;
    st->name_known = (nw & 0x10000u) != 0;
    st->name_crc = (uint16_t)nw;
    st->streaming = s_streaming;
}

/* BTstack thread: push the latest requested state (coalesced). */
static void apply(void *ctx)
{
    (void)ctx;
    jbl_beacon_state_t st;
    beacon_state(&st);
    jbl_beacon_update(&st);
    jbl_assist_set_streaming(st.streaming);
}

/* ---- LE security: Just Works, SC + bond (as the Bluedroid build) ------------------- */

static void sm_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }
    switch (hci_event_packet_get_type(packet)) {
    case SM_EVENT_JUST_WORKS_REQUEST:
        sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
        break;
    case SM_EVENT_NUMERIC_COMPARISON_REQUEST:
        sm_numeric_comparison_confirm(sm_event_numeric_comparison_request_get_handle(packet));
        break;
    case SM_EVENT_PAIRING_COMPLETE: {
        hci_con_handle_t con = sm_event_pairing_complete_get_handle(packet);
        uint8_t st = sm_event_pairing_complete_get_status(packet);
        ESP_LOGI(TAG, "LE pairing con 0x%04x: status 0x%02x, reason 0x%02x", con, st,
                 sm_event_pairing_complete_get_reason(packet));
        if (st != ERROR_CODE_SUCCESS || jbl_assist_is_link(con)) break;   /* not our own speaker links */
        le_audio_note_pairing(con);
        /* the phone's identity address (public = its classic address) */
        bd_addr_t id;
        int type = -1;
        int idx = sm_le_device_index(con);
        if (idx >= 0) le_device_db_info(idx, &type, id, NULL);
        if (type != BD_ADDR_TYPE_LE_PUBLIC) {
            sm_event_pairing_complete_get_address(packet, id);
            type = sm_event_pairing_complete_get_addr_type(packet);
        }
        if (type == BD_ADDR_TYPE_LE_PUBLIC) {
            a2dp_in_call_after_le_pairing(id);
        }
        break;
    }
    default:
        break;
    }
}

/* ---- phone name (BR/EDR remote name request) ------------------------------------- */

static void hci_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }
    switch (hci_event_packet_get_type(packet)) {
    case HCI_EVENT_CONNECTION_COMPLETE:
        if (JBL_AUTO_PHONE_NAME && hci_event_connection_complete_get_status(packet) == ERROR_CODE_SUCCESS &&
            hci_event_connection_complete_get_link_type(packet) == 0x01 /* ACL */) {
            hci_event_connection_complete_get_bd_addr(packet, s_name_req_addr);
            s_name_req_active = gap_remote_name_request(s_name_req_addr, 0x01 /* R1 */, 0) == ERROR_CODE_SUCCESS;
        }
        break;
    case HCI_EVENT_REMOTE_NAME_REQUEST_COMPLETE: {
        bd_addr_t a;
        hci_event_remote_name_request_complete_get_bd_addr(packet, a);
        if (!s_name_req_active || bd_addr_cmp(a, s_name_req_addr) != 0) {
            break;
        }
        s_name_req_active = false;
        if (hci_event_remote_name_request_complete_get_status(packet) == ERROR_CODE_SUCCESS) {
            char name[249];
            const char *n = hci_event_remote_name_request_complete_get_remote_name(packet);
            size_t len = strnlen(n, sizeof(name) - 1);
            memcpy(name, n, len);
            name[len] = 0;
            jbl_set_phone_name(name);
        }
        break;
    }
    default:
        break;
    }
}

/* ---- API ---------------------------------------------------------------------- */

void jbl_setup(void)
{
    s_bt_task = xTaskGetCurrentTaskHandle();

    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_BT) == ESP_OK) {    /* = controller public address */
        s_mac_crc = jbl_crc16_arc(mac, sizeof(mac));
        jbl_v5_set_local_addr(mac);
    }

    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    sm_set_authentication_requirements(SM_AUTHREQ_SECURE_CONNECTION | SM_AUTHREQ_BONDING);
    s_sm_reg.callback = &sm_handler;
    sm_add_event_handler(&s_sm_reg);
    s_hci_reg.callback = &hci_handler;
    hci_add_event_handler(&s_hci_reg);

    gatt_client_init();
    jbl_gatt_setup();
    jbl_v5_setup();
    jbl_beacon_setup();
    jbl_assist_setup();

    s_apply_reg.callback = &apply;
    s_apply_reg.context = NULL;
}

void jbl_on_hci_working(void)
{
    bd_addr_t a;
    gap_local_bd_addr(a);
    uint16_t crc = jbl_crc16_arc(a, sizeof(bd_addr_t));
    if (crc != s_mac_crc) {
        ESP_LOGW(TAG, "controller address differs from the eFuse BT MAC - using the controller's");
        s_mac_crc = crc;
    }
    jbl_v5_set_local_addr(a);
    ESP_LOGW(TAG, "JBL identity: BT MAC %s -> macAddressCrc 0x%04x (the user's Xtreme 5 is 0x10cd)",
             bd_addr_to_str(a), crc);

    s_working = true;
    jbl_beacon_state_t st;
    beacon_state(&st);
#ifndef AURA_GENERIC_JBL_OFF
#define AURA_GENERIC_JBL_OFF 0     /* plain-Auracast test: no JBL host beacon / assistant */
#endif
    if (AURA_GENERIC_JBL_OFF) {
        ESP_LOGW(TAG, "generic Auracast test: JBL beacon and assistant OFF (GATT/V5 still up)");
        return;
    }
    jbl_beacon_start(crc, &st);
    jbl_assist_set_streaming(st.streaming);
    jbl_assist_start(crc, a);
}

void jbl_set_party_info(uint16_t party_info)
{
    if (s_party_info == party_info) {
        return;
    }
    s_party_info = party_info;
    jbl_run_on_bt(&s_apply_reg);
}

void jbl_set_streaming(bool streaming)
{
    if (s_streaming == streaming) {
        return;                                   /* cheap to poll */
    }
    s_streaming = streaming;
    jbl_run_on_bt(&s_apply_reg);
}

void jbl_set_phone_name(const char *name)
{
    uint32_t nw = 0;
    if (name != NULL && name[0] != 0) {
        uint16_t crc = jbl_crc16_arc((const uint8_t *)name, strlen(name));
        nw = 0x10000u | crc;
        ESP_LOGW(TAG, "phone name '%s' -> srcName1Crc 0x%04x added to the JBL beacon", name, crc);
    }
    if (s_name_word == nw) {
        return;
    }
    s_name_word = nw;
    jbl_run_on_bt(&s_apply_reg);
}

bool jbl_request_phone_name(const bd_addr_t addr)
{
    if (s_name_req_active) {
        return false;                             /* one at a time (BTstack: one pending) */
    }
    bd_addr_copy(s_name_req_addr, addr);
    s_name_req_active = gap_remote_name_request(s_name_req_addr, 0x01 /* R1 */, 0) == ERROR_CODE_SUCCESS;
    return s_name_req_active;
}

uint16_t jbl_mac_crc(void)
{
    return s_mac_crc;
}

void jbl_log_stats(void)
{
    if (!s_working) {
        return;
    }
    jbl_gatt_log_stats();
    jbl_v5_log_stats();
    v5_eq_log_stats();
    jbl_beacon_log_stats();
    jbl_assist_log_stats();
}
