/*
 * ESP32-S31 JBL speaker on BTstack.
 *
 * Core layout:
 *   core 1: BT controller (its own task, prio 23) + BTstack run loop ("btstack",
 *           prio 20): HCI, L2CAP, AVDTP/A2DP, AVRCP, SDP, ATT/GATT, ISO
 *   core 0: audio DSP: SBC decode + resampler ("sbc_dsp"), LC3 encode / decode
 */
#include <stdio.h>
#include <string.h>

#include "btstack.h"
#include "btstack_run_loop_freertos.h"
#include "btstack_tlv_esp32.h"
#include "ble/le_device_db_tlv.h"
#include "classic/btstack_link_key_db_tlv.h"
#include "hci_transport_s31_vhci.h"
#include "hci_dump.h"
#include "hci_dump_embedded_stdout.h"

#include "esp_bt.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "a2dp_in.h"
#include "auracast.h"
#include "jbl.h"
#include "link_tune.h"
#include "sniffer.h"

/* 1 = JBL party / Auracast sniffer only (no A2DP, no broadcast, no JBL GATT) */
#ifndef S31_SNIFFER
#define S31_SNIFFER 0
#endif

static const char *TAG = "S31";
/* #define S31_HCI_DUMP 1   bring-up: print every HCI packet */

#define LOCAL_NAME "S31 JBL Xtreme 5"

extern uint32_t hci_transport_s31_vhci_rx_dropped(void);
extern uint32_t hci_transport_s31_iso_tx_errors(int *last);
extern void hci_transport_s31_rx_counts(uint32_t *evt, uint32_t *acl, uint32_t *iso);
static btstack_packet_callback_registration_t s_hci_reg;

static void log_heap(void)
{
    int iso_last = 0;
    uint32_t iso_err = hci_transport_s31_iso_tx_errors(&iso_last);
    ESP_LOGW(TAG, "HEAP: internal free %u KB (largest %u KB, lowest %u KB) | PSRAM free %u KB | VHCI rx drops %lu | ISO tx errors %lu (last %d)",
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) >> 10),
             (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) >> 10),
             (unsigned)(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) >> 10),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) >> 10),
             (unsigned long)hci_transport_s31_vhci_rx_dropped(), (unsigned long)iso_err, iso_last);
}

static void stats_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(2000));
        if (S31_SNIFFER) {
            uint32_t ev, acl, iso;
            hci_transport_s31_rx_counts(&ev, &acl, &iso);
            ESP_LOGW(TAG, "VHCI rx: events %lu, ACL %lu, ISO %lu", (unsigned long)ev, (unsigned long)acl, (unsigned long)iso);
            sniffer_log_stats();
            log_heap();
            continue;
        }
        a2dp_in_log_stats();
        auracast_log_stats();
        jbl_set_streaming(a2dp_in_streaming());
        jbl_log_stats();
        log_heap();
    }
}

static void hci_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET || hci_event_packet_get_type(packet) != BTSTACK_EVENT_STATE) {
        return;
    }
    if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
        bd_addr_t a;
        gap_local_bd_addr(a);
        ESP_LOGW(TAG, "BTstack up: '%s' at %s (BR/EDR A2DP sink, LE)", LOCAL_NAME, bd_addr_to_str(a));
        if (S31_SNIFFER) {
            sniffer_start();
            return;
        }
        {   /* LE TX power: probe the controller ROM setter (value units unknown: enum
             * index or dBm) from high to low and keep the highest it accepts */
            extern int r_ble_txpwr_set(int type, uint16_t handle, int level);
            extern int r_ble_txpwr_get(int type, uint16_t handle);
            int before = r_ble_txpwr_get(0, 0);
            static const int tries[] = { 20, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9 };
            char res[160];
            int n = 0, chosen = -1;
            for (unsigned i = 0; i < sizeof(tries) / sizeof(tries[0]); i++) {
                int rc = r_ble_txpwr_set(0, 0, tries[i]);
                n += snprintf(res + n, sizeof(res) - n, "%d:%d ", tries[i], rc);
                if (rc == 0) {
                    chosen = tries[i];
                    break;
                }
            }
            int rc_adv = chosen >= 0 ? r_ble_txpwr_set(1, 2, chosen) : -99;   /* adv set 2 (Auracast) */
            ESP_LOGW(TAG, "LE TX power: default was %d; tried %s-> using %d (adv set 2: %d), now default %d",
                     before, res, chosen, rc_adv, r_ble_txpwr_get(0, 0));
        }
        a2dp_in_on_hci_working();
        auracast_on_hci_working();
        jbl_on_hci_working();
        auracast_host_start();
    }
}

static void btstack_task(void *arg)
{
    (void)arg;
    btstack_memory_init();
    btstack_run_loop_init(btstack_run_loop_freertos_get_instance());
    hci_init(hci_transport_s31_vhci_get_instance(), NULL);
#ifdef S31_HCI_DUMP
    hci_dump_init(hci_dump_embedded_stdout_get_instance());
#endif

    const btstack_tlv_t *tlv = btstack_tlv_esp32_get_instance();
    btstack_tlv_set_instance(tlv, NULL);
    hci_set_link_key_db(btstack_link_key_db_tlv_get_instance(tlv, NULL));
    le_device_db_tlv_configure(tlv, NULL);

    s_hci_reg.callback = &hci_handler;
    hci_add_event_handler(&s_hci_reg);

    gap_set_local_name(LOCAL_NAME);
    if (S31_SNIFFER) {
        sniffer_setup();
    } else {
        link_tune_setup();
        a2dp_in_setup();
        auracast_setup();
        jbl_setup();
    }

    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();                 /* never returns */
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    log_heap();
    /* core 1 = Bluetooth core: the BTstack run loop sits below the controller (23) */
    xTaskCreatePinnedToCore(btstack_task, "btstack", 8192, NULL, configMAX_PRIORITIES - 5, NULL, 1);
    xTaskCreatePinnedToCore(stats_task, "stats", 4096, NULL, 2, NULL, 1);
}
