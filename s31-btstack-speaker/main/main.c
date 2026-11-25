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
#include "console_cmd.h"
#include "hci_dump_embedded_stdout.h"

#include "esp_bt.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_private/esp_clk.h"

#include "a2dp_in.h"
#include "auracast.h"
#include "jbl.h"
#include "csb.h"
#include "lls.h"
#include "jbl_v5.h"
#include "s31_ident.h"
#include "link_tune.h"
#include "party.h"
#include "le_audio.h"
#include "audio_out.h"
#include "buttons.h"
#include "tones.h"
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

/* CPU speed probe: a fixed busy loop, timed (us); ~constant unless the clock changes */
static uint32_t speed_probe_us(void)
{
    int64_t t0 = esp_timer_get_time();
    volatile uint32_t x = 0;
    for (uint32_t i = 0; i < 200000; i++) {
        x += i;
    }
    return (uint32_t)(esp_timer_get_time() - t0);
}

static void log_heap(void)
{
    ESP_LOGW(TAG, "CPU: %lu MHz (esp_clk), speed probe %lu us on core %d",
             (unsigned long)(esp_clk_cpu_freq() / 1000000), (unsigned long)speed_probe_us(), xPortGetCoreID());
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
#if S31_DEV
        {   /* who uses each core (run-time stats, delta over this 2 s window) */
            static TaskStatus_t st[40];
            static uint32_t last_rt[40];
            static TaskHandle_t last_h[40];
            static uint32_t last_total;
            uint32_t total = 0;
            UBaseType_t n = uxTaskGetSystemState(st, 40, &total);
            uint32_t dt = total - last_total;
            char line[400];
            int len = 0;
            for (UBaseType_t i = 0; i < n && dt; i++) {
                uint32_t prev = 0;
                for (int j = 0; j < 40; j++) if (last_h[j] == st[i].xHandle) prev = last_rt[j];
                uint32_t pct = (uint32_t)((uint64_t)(st[i].ulRunTimeCounter - prev) * 100 / dt);
                if (pct >= 3 && len < (int)sizeof(line) - 40) {
                    BaseType_t core = xTaskGetCoreID(st[i].xHandle);
                    len += snprintf(line + len, sizeof(line) - len, "%s/%d %lu%% ", st[i].pcTaskName,
                                    core == tskNO_AFFINITY ? -1 : (int)core, (unsigned long)pct);
                }
            }
            for (UBaseType_t i = 0; i < n && i < 40; i++) { last_h[i] = st[i].xHandle; last_rt[i] = st[i].ulRunTimeCounter; }
            last_total = total;
            if (len) ESP_LOGW(TAG, "CPU use (2 s, %% of one core): %s", line);
            static int hwm_tick;
            if (++hwm_tick % 5 == 0) {                 /* every 10 s: unused stack per task (bytes) */
                len = 0;
                for (UBaseType_t i = 0; i < n && len < (int)sizeof(line) - 30; i++) {
                    len += snprintf(line + len, sizeof(line) - len, "%s %u ", st[i].pcTaskName,
                                    (unsigned)(st[i].usStackHighWaterMark * sizeof(StackType_t)));
                }
                ESP_LOGW(TAG, "stack free (min ever, B): %s", line);
            }
        }
#endif
        a2dp_in_log_stats();
        auracast_log_stats();
        party_log_stats();
        le_audio_log_stats();
        csb_log_stats();
        lls_log_stats();
        audio_out_log_stats();
        /* slow beacon (100 ms) while streaming OR listening: at 20 ms it collides with the master's 20 ms BIG */
        /* slow beacon (100 ms) only while a BIG needs the air: broadcasting or listening.
         * Phone audio alone keeps 20 ms, so JBL One finds the speaker quickly. */
        jbl_set_streaming(auracast_state() != AURACAST_STATE_OFF || party_listening() || le_audio_phone_linked());
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
        ESP_LOGW(TAG, "BTstack up: '%s' at %s (BR/EDR A2DP sink, LE)", jbl_device_name(), bd_addr_to_str(a));
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
        csb_on_hci_working();
        /* no broadcast at boot: nothing plays yet, and the radio is better spent
         * paging the last phone (party starts LISTEN once it is connected) */
        party_on_hci_working();
        le_audio_on_hci_working();
    }
}

/* LE pairing (SMP, L2CAP CID 6) messages on the console: which side asks for what and who
 * fails - everything else of the packet log stays off */
static const hci_dump_t *s_dump_std;
static void smp_dump_packet(uint8_t type, uint8_t in, uint8_t *p, uint16_t len)
{
    /* link encryption around a pairing: LTK request, our reply, the result */
    if (type == HCI_COMMAND_DATA_PACKET && len >= 3) {
        uint16_t op = little_endian_read_16(p, 0);
        if (op == HCI_OPCODE_HCI_LE_LONG_TERM_KEY_REQUEST_REPLY && len >= 21)
            ESP_LOGW("SMP", "LTK reply con 0x%04x", little_endian_read_16(p, 3));   /* no key bytes in logs */
        else if (op == HCI_OPCODE_HCI_LE_LONG_TERM_KEY_NEGATIVE_REPLY)
            ESP_LOGW("SMP", "LTK negative reply con 0x%04x", little_endian_read_16(p, 3));
        return;
    }
    if (type == HCI_EVENT_PACKET && len >= 2) {
        uint8_t ev = p[0];
        if (ev == HCI_EVENT_LE_META && len >= 15 && p[2] == HCI_SUBEVENT_LE_LONG_TERM_KEY_REQUEST)
            ESP_LOGW("SMP", "controller asks for the LTK, con 0x%04x ediv 0x%04x", little_endian_read_16(p, 3),
                     little_endian_read_16(p, 13));
        else if ((ev == HCI_EVENT_ENCRYPTION_CHANGE || ev == HCI_EVENT_ENCRYPTION_CHANGE_V2) && len >= 6)
            ESP_LOGW("SMP", "encryption change con 0x%04x: status 0x%02x, %s", little_endian_read_16(p, 3), p[2],
                     p[5] ? "on" : "off");
        else if (ev == HCI_EVENT_ENCRYPTION_KEY_REFRESH_COMPLETE && len >= 5)
            ESP_LOGW("SMP", "key refresh con 0x%04x: status 0x%02x", little_endian_read_16(p, 3), p[2]);
        else if (ev == HCI_EVENT_COMMAND_COMPLETE && len >= 6) {
            uint16_t op = little_endian_read_16(p, 3);
            if (op == HCI_OPCODE_HCI_LE_LONG_TERM_KEY_REQUEST_REPLY || op == HCI_OPCODE_HCI_LE_ENCRYPT)
                ESP_LOGW("SMP", "cmd 0x%04x done: status 0x%02x", op, p[5]);
        }
        return;
    }
    if (type != HCI_ACL_DATA_PACKET || len < 9 || little_endian_read_16(p, 6) != 0x0006) return;
    static const char *names[] = { "?", "PairingRequest", "PairingResponse", "Confirm", "Random", "PairingFailed",
                                   "EncryptionInfo", "CentralIdent", "IdentityInfo", "IdentityAddr", "SigningInfo",
                                   "SecurityRequest", "PublicKey", "DHKeyCheck", "Keypress" };
    uint8_t code = p[8];
    char hex[40] = { 0 };
    int n = len - 9 > 12 ? 12 : len - 9;
    /* keys and pairing secrets (Confirm, Random, LTK, EDIV/Rand, IRK, CSRK, public key, DHKey
     * check) never go to the log: their name only */
    if (code == 3 || code == 4 || code == 6 || code == 7 || code == 8 || code == 10 || code == 12 || code == 13) n = 0;
    for (int i = 0; i < n; i++) snprintf(hex + i * 3, 4, "%02x ", p[9 + i]);
    ESP_LOGW("SMP", "%s %s (0x%02x) %s", in ? "phone ->" : "-> phone", code < 15 ? names[code] : "?", code, hex);
}
static void smp_dump_message(int level, const char *fmt, va_list ap)
{
    if (s_dump_std && s_dump_std->log_message) s_dump_std->log_message(level, fmt, ap);
}
static const hci_dump_t s_smp_dump = { NULL, smp_dump_packet, smp_dump_message };

static void btstack_task(void *arg)
{
    (void)arg;
    btstack_memory_init();
    btstack_run_loop_init(btstack_run_loop_freertos_get_instance());
    hci_init(hci_transport_s31_vhci_get_instance(), NULL);
    /* BTstack log_error lines on the console (no packet dump) */
#ifdef S31_HCI_DUMP
    hci_dump_init(hci_dump_embedded_stdout_get_instance());
    hci_dump_enable_packet_log(true);
#else
    s_dump_std = hci_dump_embedded_stdout_get_instance();
    hci_dump_init(&s_smp_dump);                   /* log_error lines + the LE pairing messages */
    hci_dump_enable_packet_log(true);
#endif

    const btstack_tlv_t *tlv = btstack_tlv_esp32_get_instance();
    btstack_tlv_set_instance(tlv, NULL);
    hci_set_link_key_db(btstack_link_key_db_tlv_get_instance(tlv, NULL));
    le_device_db_tlv_configure(tlv, NULL);

    s_hci_reg.callback = &hci_handler;
    hci_add_event_handler(&s_hci_reg);

    gap_set_local_name(jbl_device_name());   /* JBL One can rename it (NVS) */
    if (S31_SNIFFER) {
        sniffer_setup();
    } else {
        link_tune_setup();
        a2dp_in_setup();
        audio_out_setup();
        tones_setup();
        tones_play("poweron");
        auracast_setup();
        party_setup();
        le_audio_setup();
        csb_setup();
        lls_setup();
        buttons_setup();
#if S31_DEV
        console_cmd_setup();                 /* serial commands: development builds only */
#endif
        gap_set_security_level(LEVEL_0);     /* incoming ATT-over-BR/EDR (JBL One): the phone authenticates */
        jbl_setup();
        gap_set_security_level(LEVEL_2);     /* channels we open */
    }

    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();                 /* never returns */
}

void app_main(void)
{
#if S31_DEV
    esp_log_level_set("*", ESP_LOG_INFO);    /* development: every line (default level is WARN) */
#else
    esp_log_level_set("*", ESP_LOG_ERROR);   /* release: errors only (the arguments still run) */
#endif
    /* LMP opcode 54 (LMP_page_scan_mode_req, deprecated) from phones: the closed
     * controller rejects it and continues - logged at ERROR on every connect */
    esp_log_level_set("OLM_LMP", ESP_LOG_NONE);
    /* the controller's scheduler logs every page scan / page window at WARN ("OLC: pscan ...",
     * "OLC: page tle ...") - debug noise; its errors stay */
    esp_log_level_set("OLC", ESP_LOG_ERROR);
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    log_heap();
    s31_ident_init();                        /* this board's serial, color, Broadcast_ID */
    /* core 1 = Bluetooth core: the BTstack run loop sits below the controller (23) */
    xTaskCreatePinnedToCore(btstack_task, "btstack", 8192, NULL, configMAX_PRIORITIES - 5, NULL, 1);
    /* the 2 s stats lines: stack in PSRAM (internal RAM is what BT and the audio DMA need) */
    xTaskCreatePinnedToCoreWithCaps(stats_task, "stats", 4096, NULL, 2, NULL, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}
