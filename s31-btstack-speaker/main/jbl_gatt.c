/*
 * JBL-speaker GATT server on BTstack (port of the Bluedroid build's
 * jbl_gatt.c + gatts_cb in its main.c).
 *
 *   GAP     Device Name "S31 JBL Xtreme 5", Appearance 0x0885 (speaker)
 *   "excelpoint.cnl"  65786365-6c70-6f69-6e74-2e636e6c0000
 *        ...0001  read + notify  (speaker -> peer)
 *        ...0002  write          (peer -> speaker; logged)
 *   "excelpoint.com"  65786365-6c70-6f69-6e74-2e636f6d0000
 *        ...0001  read + notify  (V5 replies to JBL One)
 *        ...0002  write          (V5 commands; long ones as prepared writes)
 *   Device Information 0x180A: manufacturer / model / firmware
 *
 * The same att_server serves LE and BR/EDR (ENABLE_GATT_OVER_CLASSIC: L2CAP
 * PSM 0x1F); JBL One opens the BR/EDR one. Notifications work on both
 * bearers natively (the Bluedroid build needed idf-patches/0004 for that).
 */
#include <stdio.h>
#include <string.h>

#include "btstack.h"
#include "classic/gatt_sdp.h"

#include "esp_heap_caps.h"
#include "esp_log.h"

#include "jbl_assist.h"
#include "jbl_gatt.h"
#include "jbl_gatt_db.h"
#include "jbl_v5.h"

static const char *TAG = "JBL_GATT";

#define H_SC_CCCD    ATT_CHARACTERISTIC_GATT_SERVICE_CHANGED_01_CLIENT_CONFIGURATION_HANDLE
#define H_CNL_NTF    ATT_CHARACTERISTIC_65786365_6c70_6f69_6e74_2e636e6c0001_01_VALUE_HANDLE
#define H_CNL_CCCD   ATT_CHARACTERISTIC_65786365_6c70_6f69_6e74_2e636e6c0001_01_CLIENT_CONFIGURATION_HANDLE
#define H_CNL_WR     ATT_CHARACTERISTIC_65786365_6c70_6f69_6e74_2e636e6c0002_01_VALUE_HANDLE
#define H_COM_NTF    ATT_CHARACTERISTIC_65786365_6c70_6f69_6e74_2e636f6d0001_01_VALUE_HANDLE
#define H_COM_CCCD   ATT_CHARACTERISTIC_65786365_6c70_6f69_6e74_2e636f6d0001_01_CLIENT_CONFIGURATION_HANDLE
#define H_COM_WR     ATT_CHARACTERISTIC_65786365_6c70_6f69_6e74_2e636f6d0002_01_VALUE_HANDLE

/* ---- per-connection CCCDs ----------------------------------------------------- */

#define MAX_LINKS 4
enum { CCCD_SC, CCCD_CNL, CCCD_COM, CCCD_N };

typedef struct {
    hci_con_handle_t con;               /* HCI_CON_HANDLE_INVALID = free */
    uint16_t cccd[CCCD_N];
} link_t;

static link_t s_links[MAX_LINKS];

static link_t *link_find(hci_con_handle_t con)
{
    for (int i = 0; i < MAX_LINKS; i++) {
        if (s_links[i].con == con) {
            return &s_links[i];
        }
    }
    return NULL;
}

static link_t *link_add(hci_con_handle_t con)
{
    link_t *l = link_find(con);
    if (l == NULL) {
        l = link_find(HCI_CON_HANDLE_INVALID);
    }
    if (l != NULL) {
        l->con = con;
        memset(l->cccd, 0, sizeof(l->cccd));
    }
    return l;
}

static int cccd_index(uint16_t handle)
{
    switch (handle) {
    case H_SC_CCCD:  return CCCD_SC;
    case H_CNL_CCCD: return CCCD_CNL;
    case H_COM_CCCD: return CCCD_COM;
    default:         return -1;
    }
}

static const char *handle_name(uint16_t handle)
{
    switch (handle) {
    case H_CNL_WR:   return "cnl.0002 (write)";
    case H_CNL_CCCD: return "cnl.0001 CCCD";
    case H_COM_WR:   return "com.0002 (write)";
    case H_COM_CCCD: return "com.0001 CCCD";
    case H_SC_CCCD:  return "Service Changed CCCD";
    default:         return "other";
    }
}

/* ---- prepared (long) writes to the V5 channel ------------------------------------ */

/* cold (a few app commands per minute): PSRAM, allocated on first use */
#define V5_PREP_MAX 600
static uint8_t *s_prep;
static uint16_t s_prep_len;
static hci_con_handle_t s_prep_con = HCI_CON_HANDLE_INVALID;

static void prep_reset(void)
{
    s_prep_len = 0;
    s_prep_con = HCI_CON_HANDLE_INVALID;
}

static int prep_write(hci_con_handle_t con, uint16_t handle, uint16_t offset, const uint8_t *buf, uint16_t len)
{
    if (handle != H_COM_WR) {
        ESP_LOGI(TAG, "prepared write to %s 0x%04x off %u len %u (ignored)", handle_name(handle), handle, offset, len);
        return 0;
    }
    if (s_prep_con != HCI_CON_HANDLE_INVALID && s_prep_con != con) {
        return ATT_ERROR_PREPARE_QUEUE_FULL;           /* another peer's long write in progress */
    }
    if (s_prep == NULL) {
        s_prep = heap_caps_malloc(V5_PREP_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_prep == NULL) {
            s_prep = heap_caps_malloc(V5_PREP_MAX, MALLOC_CAP_8BIT);
        }
        if (s_prep == NULL) {
            ESP_LOGE(TAG, "no RAM for V5 prepared writes");
            return ATT_ERROR_INSUFFICIENT_RESOURCES;
        }
    }
    if ((uint32_t)offset + len > V5_PREP_MAX) {
        return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;   /* reported on execute */
    }
    s_prep_con = con;
    memcpy(s_prep + offset, buf, len);
    if (offset + len > s_prep_len) {
        s_prep_len = offset + len;
    }
    return 0;
}

/* ---- notification queue --------------------------------------------------------- */

#define TX_RING_SIZE 3072
static btstack_ring_buffer_t s_tx_ring;
static uint8_t *s_tx_storage;
static uint8_t s_stage[JBL_GATT_TX_OVERHEAD + JBL_GATT_NOTIFY_MAX];
static bool s_staged;                       /* s_stage holds the next notification */
static bool s_tx_wait;                      /* can-send-now request outstanding */
static hci_con_handle_t s_tx_wait_con = HCI_CON_HANDLE_INVALID;
static btstack_context_callback_registration_t s_tx_reg;

static uint32_t s_tx_sent, s_tx_queued, s_tx_dropped, s_tx_stale, s_tx_waits, s_tx_peak;
static uint32_t s_rx_writes, s_rx_v5, s_rx_prep_frames;

static uint8_t *s_tx_tmp;                   /* purge scratch, after the ring storage */

static void tx_drain(void);

/* Drop everything queued for a connection that went away: its can-send request
 * died with it, and it may still look alive while its disconnect is handled. */
static void tx_purge(hci_con_handle_t con)
{
    if (s_staged && little_endian_read_16(s_stage, 0) == con) {
        s_staged = false;
        s_tx_stale++;
    }
    if (s_tx_wait && s_tx_wait_con == con) {
        s_tx_wait = false;
        s_tx_wait_con = HCI_CON_HANDLE_INVALID;
    }
    if (s_tx_storage == NULL) {
        return;
    }
    /* rotate once through the ring, writing back what is not for `con` */
    uint32_t left = btstack_ring_buffer_bytes_available(&s_tx_ring);
    while (left >= JBL_GATT_TX_OVERHEAD) {
        uint32_t got = 0;
        btstack_ring_buffer_read(&s_tx_ring, s_tx_tmp, JBL_GATT_TX_OVERHEAD, &got);
        uint16_t len = little_endian_read_16(s_tx_tmp, 4);
        btstack_ring_buffer_read(&s_tx_ring, s_tx_tmp + JBL_GATT_TX_OVERHEAD, len, &got);
        left -= JBL_GATT_TX_OVERHEAD + len;
        if (little_endian_read_16(s_tx_tmp, 0) == con) {
            s_tx_stale++;
        } else {
            btstack_ring_buffer_write(&s_tx_ring, s_tx_tmp, JBL_GATT_TX_OVERHEAD + len);
        }
    }
}

static void tx_can_send(void *context)
{
    (void)context;
    s_tx_wait = false;
    s_tx_wait_con = HCI_CON_HANDLE_INVALID;
    tx_drain();
}

static void tx_drain(void)
{
    while (!s_tx_wait) {
        if (!s_staged) {
            if (btstack_ring_buffer_bytes_available(&s_tx_ring) < JBL_GATT_TX_OVERHEAD) {
                return;
            }
            uint32_t got = 0;
            btstack_ring_buffer_read(&s_tx_ring, s_stage, JBL_GATT_TX_OVERHEAD, &got);
            uint16_t len = little_endian_read_16(s_stage, 4);
            btstack_ring_buffer_read(&s_tx_ring, s_stage + JBL_GATT_TX_OVERHEAD, len, &got);
            s_staged = true;
        }
        hci_con_handle_t con = little_endian_read_16(s_stage, 0);
        uint16_t attr = little_endian_read_16(s_stage, 2);
        uint16_t len = little_endian_read_16(s_stage, 4);
        uint8_t st = att_server_notify(con, attr, s_stage + JBL_GATT_TX_OVERHEAD, len);
        if (st == ERROR_CODE_SUCCESS) {
            s_staged = false;
            s_tx_sent++;
        } else if (st == BTSTACK_ACL_BUFFERS_FULL) {
            s_tx_reg.callback = &tx_can_send;
            s_tx_reg.context = NULL;
            if (att_server_request_to_send_notification(&s_tx_reg, con) == ERROR_CODE_SUCCESS) {
                s_tx_wait = true;
                s_tx_wait_con = con;
                s_tx_waits++;
            } else {
                s_staged = false;               /* link vanished meanwhile */
                s_tx_stale++;
            }
        } else {
            s_staged = false;                   /* connection gone: drop its leftovers */
            s_tx_stale++;
        }
    }
}

uint32_t jbl_gatt_tx_free(void)
{
    return s_tx_storage ? btstack_ring_buffer_bytes_free(&s_tx_ring) : 0;
}

bool jbl_gatt_tx_busy(void)
{
    return s_staged || s_tx_wait || (s_tx_storage && !btstack_ring_buffer_empty(&s_tx_ring));
}

bool jbl_gatt_notify(hci_con_handle_t con, uint16_t attr_handle, const uint8_t *data, uint16_t len)
{
    if (len > JBL_GATT_NOTIFY_MAX) {
        s_tx_dropped++;
        return false;
    }
    /* fast path: nothing queued and the bearer has room */
    if (!jbl_gatt_tx_busy()) {
        uint8_t st = att_server_notify(con, attr_handle, data, len);
        if (st == ERROR_CODE_SUCCESS) {
            s_tx_sent++;
            return true;
        }
        if (st != BTSTACK_ACL_BUFFERS_FULL) {
            s_tx_stale++;
            return false;
        }
    }
    if (s_tx_storage == NULL || btstack_ring_buffer_bytes_free(&s_tx_ring) < JBL_GATT_TX_OVERHEAD + (uint32_t)len) {
        s_tx_dropped++;
        return false;
    }
    uint8_t hdr[JBL_GATT_TX_OVERHEAD];
    little_endian_store_16(hdr, 0, con);
    little_endian_store_16(hdr, 2, attr_handle);
    little_endian_store_16(hdr, 4, len);
    btstack_ring_buffer_write(&s_tx_ring, hdr, sizeof(hdr));
    btstack_ring_buffer_write(&s_tx_ring, (uint8_t *)(uintptr_t)data, len);
    s_tx_queued++;
    uint32_t used = TX_RING_SIZE - btstack_ring_buffer_bytes_free(&s_tx_ring);
    if (used > s_tx_peak) {
        s_tx_peak = used;
    }
    tx_drain();
    return true;
}

uint16_t jbl_gatt_v5_notify_handle(void)
{
    return H_COM_NTF;
}

uint16_t jbl_gatt_mtu(hci_con_handle_t con)
{
    return att_server_get_mtu(con);
}

/* ---- ATT callbacks ------------------------------------------------------------- */

static uint16_t att_read_cb(hci_con_handle_t con, uint16_t handle, uint16_t offset, uint8_t *buffer, uint16_t size)
{
    int ci = cccd_index(handle);
    if (ci >= 0) {
        link_t *l = link_find(con);
        return att_read_callback_handle_little_endian_16(l ? l->cccd[ci] : 0, offset, buffer, size);
    }
    return 0;
}

static void log_write(uint16_t handle, const uint8_t *buf, uint16_t len)
{
    char h[3 * 32 + 1];
    int n = len > 32 ? 32 : len, o = 0;
    for (int i = 0; i < n; i++) {
        o += snprintf(h + o, sizeof(h) - (size_t)o, "%02x ", buf[i]);
    }
    h[o] = 0;
    ESP_LOGW(TAG, ">>> peer WRITE %s handle 0x%04x len %u: %s", handle_name(handle), handle, len, h);
}

static int att_write_cb(hci_con_handle_t con, uint16_t handle, uint16_t mode, uint16_t offset,
                        uint8_t *buf, uint16_t len)
{
    switch (mode) {
    case ATT_TRANSACTION_MODE_ACTIVE:
        return prep_write(con, handle, offset, buf, len);
    case ATT_TRANSACTION_MODE_VALIDATE:
        return 0;
    case ATT_TRANSACTION_MODE_EXECUTE:
        if (con == s_prep_con && s_prep_len != 0) {
            s_rx_prep_frames++;
            uint16_t n = s_prep_len;
            prep_reset();
            jbl_v5_on_write(con, s_prep, n);
        } else if (con == s_prep_con) {
            prep_reset();
        }
        return 0;
    case ATT_TRANSACTION_MODE_CANCEL:
        if (con == s_prep_con) {
            prep_reset();
        }
        return 0;
    default:
        break;
    }

    s_rx_writes++;
    if (handle == H_COM_WR) {
        s_rx_v5++;
        jbl_v5_on_write(con, buf, len);         /* JBL One command channel */
        return 0;
    }
    int ci = cccd_index(handle);
    if (ci >= 0) {
        link_t *l = link_find(con);
        if (l == NULL) {
            l = link_add(con);
        }
        if (l != NULL && len >= 2) {
            l->cccd[ci] = little_endian_read_16(buf, 0);
        }
    }
    log_write(handle, buf, len);
    return 0;
}

static void att_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }
    switch (hci_event_packet_get_type(packet)) {
    case ATT_EVENT_CONNECTED: {
        hci_con_handle_t con = att_event_connected_get_handle(packet);
        bd_addr_t a;
        att_event_connected_get_address(packet, a);
        bool classic = gap_get_connection_type(con) == GAP_CONNECTION_ACL;
        link_add(con);
        if (!classic && jbl_assist_is_link(con)) {
            break;                              /* our own GATT-client link to a speaker */
        }
        if (!classic && gap_get_role(con) == HCI_ROLE_MASTER) {
            break;                              /* outgoing LE link of another module */
        }
        ESP_LOGW(TAG, ">>> GATT CONNECT from %s over %s (con 0x%04x, MTU %u)", bd_addr_to_str(a),
                 classic ? "BR/EDR" : "LE", con, att_server_get_mtu(con));
        jbl_v5_link_up(con);
        break;
    }
    case ATT_EVENT_DISCONNECTED: {
        hci_con_handle_t con = att_event_disconnected_get_handle(packet);
        link_t *l = link_find(con);
        if (l != NULL) {
            l->con = HCI_CON_HANDLE_INVALID;
        }
        if (con == s_prep_con) {
            prep_reset();
        }
        jbl_v5_link_down(con);
        tx_purge(con);
        tx_drain();                             /* the other links' notifications */
        break;
    }
    case ATT_EVENT_MTU_EXCHANGE_COMPLETE:
        ESP_LOGI(TAG, "peer MTU %u (con 0x%04x)", att_event_mtu_exchange_complete_get_MTU(packet),
                 att_event_mtu_exchange_complete_get_handle(packet));
        break;
    default:
        break;
    }
}

/* ---- SDP: one record per primary service, as Bluedroid's gatt_add_sdp_record --- */

#define SDP_REC_SIZE 80
static uint8_t s_sdp_gatt[SDP_REC_SIZE];
static uint8_t s_sdp_gap[SDP_REC_SIZE];
static uint8_t s_sdp_cnl[SDP_REC_SIZE];
static uint8_t s_sdp_com[SDP_REC_SIZE];
static uint8_t s_sdp_dis[SDP_REC_SIZE];

/* ServiceClassIDList = the service UUID; ProtocolDescriptorList = L2CAP(0x1F) +
 * ATT(start, end); BrowseGroupList = public root. uuid128 big-endian or NULL. */
static void sdp_gatt_service_record(uint8_t *rec, uint16_t uuid16, const uint8_t *uuid128,
                                    uint16_t start, uint16_t end)
{
    de_create_sequence(rec);
    de_add_number(rec, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_SERVICE_RECORD_HANDLE);
    de_add_number(rec, DE_UINT, DE_SIZE_32, sdp_create_service_record_handle());

    de_add_number(rec, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_SERVICE_CLASS_ID_LIST);
    uint8_t *cls = de_push_sequence(rec);
    if (uuid128 != NULL) {
        uint8_t u[16];
        memcpy(u, uuid128, sizeof(u));
        de_add_uuid128(cls, u);
    } else {
        de_add_number(cls, DE_UUID, DE_SIZE_16, uuid16);
    }
    de_pop_sequence(rec, cls);

    de_add_number(rec, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_PROTOCOL_DESCRIPTOR_LIST);
    uint8_t *pdl = de_push_sequence(rec);
    {
        uint8_t *l2cap = de_push_sequence(pdl);
        de_add_number(l2cap, DE_UUID, DE_SIZE_16, BLUETOOTH_PROTOCOL_L2CAP);
        de_add_number(l2cap, DE_UINT, DE_SIZE_16, BLUETOOTH_PSM_ATT);
        de_pop_sequence(pdl, l2cap);
        uint8_t *att = de_push_sequence(pdl);
        de_add_number(att, DE_UUID, DE_SIZE_16, BLUETOOTH_PROTOCOL_ATT);
        de_add_number(att, DE_UINT, DE_SIZE_16, start);
        de_add_number(att, DE_UINT, DE_SIZE_16, end);
        de_pop_sequence(pdl, att);
    }
    de_pop_sequence(rec, pdl);

    de_add_number(rec, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_BROWSE_GROUP_LIST);
    uint8_t *bg = de_push_sequence(rec);
    de_add_number(bg, DE_UUID, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_PUBLIC_BROWSE_ROOT);
    de_pop_sequence(rec, bg);
}

static void sdp_register(uint8_t *rec, const char *what)
{
    if (de_get_len(rec) > SDP_REC_SIZE) {
        ESP_LOGE(TAG, "SDP record %s overflows (%lu B)", what, (unsigned long)de_get_len(rec));
        abort();
    }
    uint8_t st = sdp_register_service(rec);
    if (st != ERROR_CODE_SUCCESS) {
        ESP_LOGE(TAG, "SDP record %s not registered (0x%02x)", what, st);
    }
}

static void sdp_setup(void)
{
    static const uint8_t cnl_uuid[16] = { 0x65, 0x78, 0x63, 0x65, 0x6c, 0x70, 0x6f, 0x69,
                                          0x6e, 0x74, 0x2e, 0x63, 0x6e, 0x6c, 0x00, 0x00 };
    static const uint8_t com_uuid[16] = { 0x65, 0x78, 0x63, 0x65, 0x6c, 0x70, 0x6f, 0x69,
                                          0x6e, 0x74, 0x2e, 0x63, 0x6f, 0x6d, 0x00, 0x00 };

    /* GATT service itself: BTstack's own record (class 0x1801) */
    gatt_create_sdp_record(s_sdp_gatt, sdp_create_service_record_handle(),
                           ATT_SERVICE_GATT_SERVICE_START_HANDLE, ATT_SERVICE_GATT_SERVICE_END_HANDLE);
    sdp_register(s_sdp_gatt, "GATT");
    sdp_gatt_service_record(s_sdp_gap, ORG_BLUETOOTH_SERVICE_GENERIC_ACCESS, NULL,
                            ATT_SERVICE_GAP_SERVICE_START_HANDLE, ATT_SERVICE_GAP_SERVICE_END_HANDLE);
    sdp_register(s_sdp_gap, "GAP");
    sdp_gatt_service_record(s_sdp_cnl, 0, cnl_uuid,
                            ATT_SERVICE_65786365_6c70_6f69_6e74_2e636e6c0000_START_HANDLE,
                            ATT_SERVICE_65786365_6c70_6f69_6e74_2e636e6c0000_END_HANDLE);
    sdp_register(s_sdp_cnl, "excelpoint.cnl");
    sdp_gatt_service_record(s_sdp_com, 0, com_uuid,
                            ATT_SERVICE_65786365_6c70_6f69_6e74_2e636f6d0000_START_HANDLE,
                            ATT_SERVICE_65786365_6c70_6f69_6e74_2e636f6d0000_END_HANDLE);
    sdp_register(s_sdp_com, "excelpoint.com");
    sdp_gatt_service_record(s_sdp_dis, ORG_BLUETOOTH_SERVICE_DEVICE_INFORMATION, NULL,
                            ATT_SERVICE_ORG_BLUETOOTH_SERVICE_DEVICE_INFORMATION_START_HANDLE,
                            ATT_SERVICE_ORG_BLUETOOTH_SERVICE_DEVICE_INFORMATION_END_HANDLE);
    sdp_register(s_sdp_dis, "Device Information");
}

/* ---- setup ------------------------------------------------------------------ */

void jbl_gatt_setup(void)
{
    for (int i = 0; i < MAX_LINKS; i++) {
        s_links[i].con = HCI_CON_HANDLE_INVALID;
    }
    /* cold (a few notifications per second at most): PSRAM */
    const size_t sz = TX_RING_SIZE + JBL_GATT_TX_OVERHEAD + JBL_GATT_NOTIFY_MAX;
    s_tx_storage = heap_caps_malloc(sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_tx_storage == NULL) {
        s_tx_storage = heap_caps_malloc(sz, MALLOC_CAP_8BIT);
    }
    if (s_tx_storage == NULL) {
        ESP_LOGE(TAG, "no RAM for the notification queue - only direct sends");
    } else {
        btstack_ring_buffer_init(&s_tx_ring, s_tx_storage, TX_RING_SIZE);
        s_tx_tmp = s_tx_storage + TX_RING_SIZE;
    }

    att_server_init(profile_data, &att_read_cb, &att_write_cb);
    att_server_register_packet_handler(&att_event_handler);
    sdp_setup();

    ESP_LOGW(TAG, "GATT excelpoint.cnl: notify 0x%04x, write 0x%04x | excelpoint.com: notify 0x%04x, "
             "write 0x%04x | DIS 0x%04x | LE + BR/EDR (%u B database)",
             H_CNL_NTF, H_CNL_WR, H_COM_NTF, H_COM_WR,
             ATT_SERVICE_ORG_BLUETOOTH_SERVICE_DEVICE_INFORMATION_START_HANDLE, (unsigned)sizeof(profile_data));
}

void jbl_gatt_log_stats(void)
{
    int links = 0;
    for (int i = 0; i < MAX_LINKS; i++) {
        links += s_links[i].con != HCI_CON_HANDLE_INVALID;
    }
    ESP_LOGI(TAG, "GATT links %d | rx writes %lu (V5 %lu, long %lu) | notify sent %lu, queued %lu, "
             "dropped %lu, stale %lu, waits %lu, queue peak %lu B%s",
             links, (unsigned long)s_rx_writes, (unsigned long)s_rx_v5, (unsigned long)s_rx_prep_frames,
             (unsigned long)s_tx_sent, (unsigned long)s_tx_queued, (unsigned long)s_tx_dropped,
             (unsigned long)s_tx_stale, (unsigned long)s_tx_waits, (unsigned long)s_tx_peak,
             s_tx_wait ? " (waiting)" : "");
}
