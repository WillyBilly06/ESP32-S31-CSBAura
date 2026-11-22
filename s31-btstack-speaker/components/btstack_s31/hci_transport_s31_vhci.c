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
#include "esp_log.h"
#include "esp_timer.h"
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

static int transport_send_packet(uint8_t packet_type, uint8_t *packet, int size)
{
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

uint32_t hci_transport_s31_vhci_rx_dropped(void)
{
    return s_rx_dropped;
}
