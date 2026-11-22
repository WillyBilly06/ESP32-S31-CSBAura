/*
 * BTstack HCI transport over the ESP32-S31 controller's VHCI.
 *
 * Adapted from BTstack port/esp32/components/btstack/hci_transport_esp32_vhci.c:
 *   - the S31 controller runs dual mode (BR/EDR for the phone + LE for
 *     Auracast/JBL), enabled with ESP_BT_MODE_BTDM
 *   - synchronous VHCI: like the C6/H2 generation the S31 controller is paced
 *     by regular HCI host->controller flow control (Number Of Completed
 *     Packets), so can_send_packet_now stays NULL
 *   - the receive ring also holds ISO SDUs (BIG sync while the party role
 *     listens to another JBL)
 *
 * The VHCI receive callback runs in the controller task (core 1); packets are
 * queued in a ring and delivered on the BTstack run loop thread (also core 1).
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "btstack_config.h"
#include "btstack_debug.h"
#include "btstack_run_loop.h"
#include "btstack_ring_buffer.h"
#include "btstack_util.h"
#include "hci.h"
#include "hci_transport.h"
#include "hci_transport_s31_vhci.h"

#include "esp_bt.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "vhci";

static void (*transport_packet_handler)(uint8_t packet_type, uint8_t *packet, uint16_t size);

#define MAX_NR_HOST_EVENT_PACKETS 8
static uint8_t s_ring_storage[HCI_HOST_ACL_PACKET_NUM * (2 + 1 + HCI_ACL_HEADER_SIZE + HCI_HOST_ACL_PACKET_LEN) +
                              S31_HOST_ISO_PACKET_NUM * (2 + 1 + S31_HOST_ISO_PACKET_LEN) +
                              MAX_NR_HOST_EVENT_PACKETS * (2 + 1 + HCI_EVENT_BUFFER_SIZE)];
static btstack_ring_buffer_t s_ring;
static uint8_t s_rx_with_pre[HCI_INCOMING_PRE_BUFFER_SIZE + HCI_INCOMING_PACKET_BUFFER_SIZE + 1];
static uint8_t *const s_rx = &s_rx_with_pre[HCI_INCOMING_PRE_BUFFER_SIZE];
static SemaphoreHandle_t s_ring_mutex;
static volatile uint32_t s_rx_dropped;
static volatile uint32_t s_rx_type[6];

static void deliver_packets(void *context);
static btstack_context_callback_registration_t s_deliver_cb = { .callback = deliver_packets };

uint32_t hal_time_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* controller task */
static void host_send_pkt_available_cb(void)
{
}

/* controller task */
static int host_recv_pkt_cb(uint8_t *data, uint16_t len)
{
    if (len > 0 && data[0] < 6) {
        s_rx_type[data[0]]++;
        if (data[0] == 5 && s_rx_type[5] <= 3) {
            esp_rom_printf("VHCI ISO rx #%u len %u: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                           (unsigned)s_rx_type[5], len, data[1], data[2], data[3], data[4], data[5], data[6],
                           data[7], data[8], data[9], data[10], data[11], data[12]);
        }
    }
    xSemaphoreTake(s_ring_mutex, portMAX_DELAY);
    if (btstack_ring_buffer_bytes_free(&s_ring) < (uint32_t)len + 2) {
        xSemaphoreGive(s_ring_mutex);
        s_rx_dropped++;
        return 0;
    }
    uint8_t tag[2];
    little_endian_store_16(tag, 0, len);
    btstack_ring_buffer_write(&s_ring, tag, sizeof(tag));
    btstack_ring_buffer_write(&s_ring, data, len);
    xSemaphoreGive(s_ring_mutex);
    btstack_run_loop_execute_on_main_thread(&s_deliver_cb);
    return 0;
}

/* controller task: received ISO data (BIS sync) does not go through VHCI; the
 * controller hands it here (idf-patches/0010) starting at the ISO handle and
 * frees it after we return. Queue it like the VHCI packets, with the H4 type. */
static void iso_rx_cb(uint8_t *data, uint16_t len)
{
    s_rx_type[5]++;
    xSemaphoreTake(s_ring_mutex, portMAX_DELAY);
    if (btstack_ring_buffer_bytes_free(&s_ring) < (uint32_t)len + 3) {
        xSemaphoreGive(s_ring_mutex);
        s_rx_dropped++;
        return;
    }
    uint8_t tag[3];
    little_endian_store_16(tag, 0, (uint16_t)(len + 1));
    tag[2] = HCI_ISO_DATA_PACKET;
    btstack_ring_buffer_write(&s_ring, tag, sizeof(tag));
    btstack_ring_buffer_write(&s_ring, data, len);
    xSemaphoreGive(s_ring_mutex);
    btstack_run_loop_execute_on_main_thread(&s_deliver_cb);
}
extern void ble_host_register_rx_iso_data_cb(void *cb);

static const esp_vhci_host_callback_t s_vhci_cb = {
    .notify_host_send_available = host_send_pkt_available_cb,
    .notify_host_recv = host_recv_pkt_cb,
};

/* run loop thread */
static void deliver_packets(void *context)
{
    (void)context;
    xSemaphoreTake(s_ring_mutex, portMAX_DELAY);
    while (btstack_ring_buffer_bytes_available(&s_ring)) {
        uint32_t n;
        uint8_t tag[2];
        btstack_ring_buffer_read(&s_ring, tag, 2, &n);
        uint32_t len = little_endian_read_16(tag, 0);
        if (len > HCI_INCOMING_PACKET_BUFFER_SIZE + 1) {
            /* cannot happen (writer checks), but never overrun s_rx */
            ESP_LOGE(TAG, "rx packet %u too long, ring reset", (unsigned)len);
            btstack_ring_buffer_init(&s_ring, s_ring_storage, sizeof(s_ring_storage));
            break;
        }
        btstack_ring_buffer_read(&s_ring, s_rx, len, &n);
        xSemaphoreGive(s_ring_mutex);
        /* The S31 controller reports LMP feature bit 37 "BR/EDR Not Supported"
         * (features byte 4 = 0xF8) although it runs BR/EDR fine in BTDM mode
         * (Bluedroid ignored the bit). BTstack honours it and would skip all
         * Classic setup (scan enable, CoD, name, SSP) -> the phone cannot see
         * us. Clear it in HCI Read Local Supported Features' response. */
        if (len >= 15 && s_rx[0] == HCI_EVENT_PACKET && s_rx[1] == HCI_EVENT_COMMAND_COMPLETE &&
            little_endian_read_16(s_rx, 4) == 0x1003 && s_rx[6] == 0) {
            s_rx[7 + 4] &= (uint8_t)~0x20;     /* byte 4 bit 5 (feature bit 37) */
        }
        transport_packet_handler(s_rx[0], &s_rx[1], (uint16_t)(len - 1));
        xSemaphoreTake(s_ring_mutex, portMAX_DELAY);
    }
    xSemaphoreGive(s_ring_mutex);
}

static void transport_init(const void *transport_config)
{
    (void)transport_config;
    s_ring_mutex = xSemaphoreCreateMutex();
}

static int transport_open(void)
{
    static bool controller_up;
    btstack_ring_buffer_init(&s_ring, s_ring_storage, sizeof(s_ring_storage));
    if (!controller_up) {
        esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        esp_err_t err = esp_bt_controller_init(&cfg);
        if (err) {
            ESP_LOGE(TAG, "esp_bt_controller_init: %s", esp_err_to_name(err));
            return -1;
        }
        err = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
        if (err) {
            ESP_LOGE(TAG, "esp_bt_controller_enable(BTDM): %s", esp_err_to_name(err));
            return -1;
        }
        controller_up = true;
    }
    esp_vhci_host_register_callback(&s_vhci_cb);
    ble_host_register_rx_iso_data_cb((void *)iso_rx_cb);
    ESP_LOGI(TAG, "controller up (BR/EDR + LE), VHCI ring %u B", (unsigned)sizeof(s_ring_storage));
    return 0;
}

static int transport_close(void)
{
    return 0;
}

static void transport_register_packet_handler(void (*handler)(uint8_t, uint8_t *, uint16_t))
{
    transport_packet_handler = handler;
}

static volatile uint32_t s_iso_alloc_fail, s_iso_tx_err;
static volatile int s_iso_tx_last_err;
extern int r_ble_hci_trans_hs_iso_tx(const uint8_t *data, uint16_t length, void *arg);

/* ISO TX errors from the controller (-1 no BIS, -2 bad packet, -3 cannot send,
 * -4 not ready / data path not set up, -5 fragment failed). */
uint32_t hci_transport_s31_iso_tx_errors(int *last)
{
    if (last) {
        *last = s_iso_tx_last_err;
    }
    return s_iso_tx_err + s_iso_alloc_fail;
}

static int transport_send_packet(uint8_t packet_type, uint8_t *packet, int size)
{
    if (packet_type == HCI_ISO_DATA_PACKET) {
        /* ISO goes straight to the controller like Espressif's own ISO host
         * (esp_ble_iso iso.c): r_ble_hci_trans_hs_iso_tx(pkt starting at the
         * handle). Through VHCI the H4 type byte stays in front of the handle
         * and the controller drops the SDU. The controller owns and free()s
         * the buffer in every case, so it gets a heap copy. */
        uint8_t *buf = heap_caps_malloc((size_t)size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (buf == NULL) {
            s_iso_alloc_fail++;
            return -1;
        }
        memcpy(buf, packet, (size_t)size);
        int rc = r_ble_hci_trans_hs_iso_tx(buf, (uint16_t)size, NULL);
        if (rc != 0) {
            s_iso_tx_err++;
            s_iso_tx_last_err = rc;
        }
        return 0;
    }
    /* HCI_OUTGOING_PRE_BUFFER_SIZE (1) leaves room for the H4 type byte */
    packet--;
    *packet = packet_type;
    esp_vhci_host_send_packet(packet, (uint16_t)(size + 1));
    return 0;
}

static const hci_transport_t s_transport = {
    .name = "s31-vhci",
    .init = &transport_init,
    .open = &transport_open,
    .close = &transport_close,
    .register_packet_handler = &transport_register_packet_handler,
    .send_packet = &transport_send_packet,
};

const hci_transport_t *hci_transport_s31_vhci_get_instance(void)
{
    return &s_transport;
}

void hci_transport_s31_rx_counts(uint32_t *evt, uint32_t *acl, uint32_t *iso)
{
    *evt = s_rx_type[4];
    *acl = s_rx_type[2];
    *iso = s_rx_type[5];
}

uint32_t hci_transport_s31_vhci_rx_dropped(void)
{
    return s_rx_dropped;
}
