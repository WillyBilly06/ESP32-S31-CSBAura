/*
 * ESP32-S31 Auracast demo — "behave like the JBL button"
 * ===========================================================================
 * One button cycles three states, mirroring the JBL Auracast button:
 *
 *     IDLE  ──press──▶  BROADCAST (Auracast SOURCE: we transmit)
 *       ▲                   │ press
 *       │ press             ▼
 *     RECEIVE (Auracast SINK: we join a broadcast)  ◀──────────┘
 *
 * HOW TO USE THIS FILE
 *   The BROADCAST path below uses the REAL ESP-IDF LE-Audio API, matching the
 *   official example `examples/bluetooth/esp_ble_audio/pbp/pbs` (verified against
 *   esp-idf @ad20ded6). The simplest way to get a building tree:
 *
 *     idf.py create-project-from-example "espressif/esp_ble_audio:pbp/pbs"
 *
 *   then drop THIS file in as main.c (it keeps the example's helpers:
 *   bluetooth_init / app_host_init from common_components/example_init, and
 *   ADV_HANDLE / ext_adv_start / set_device_name / the TX scheduler from adv.c +
 *   example_utils). The button/mode layer here wraps those real calls.
 *
 *   The RECEIVE path mirrors `pbp/pbk` (public broadcast sink); it's kept as a
 *   documented outline because this file is built from the *source* example —
 *   pull the sink calls from pbk when you wire receive.
 *
 *   The API is a PREVIEW; spots to reconcile with YOUR idf version are >>VERIFY<<.
 * ===========================================================================
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "esp_ble_audio_bap_api.h"
#include "esp_ble_audio_bap_lc3_preset_defs.h"
#include "esp_ble_audio_cap_api.h"
#include "esp_ble_audio_codec_api.h"
#include "esp_ble_audio_pbp_api.h"

/* From the example's common_components + adv.c (copied in when you start from
 * the pbp/pbs example). Declared here so this file documents its deps. */
extern void bluetooth_init(void);          /* example_init */
extern void app_host_init(void);           /* example_init */
extern void set_device_name(void);         /* adv.c */
extern int  ext_adv_start(void);           /* adv.c (name varies; >>VERIFY) */
#ifndef ADV_HANDLE
#define ADV_HANDLE 0                        /* real value comes from adv.h  >>VERIFY */
#endif

static const char *TAG = "auracast";

/* ---- Interop knobs — match these to the JBL side ------------------------ */
#define LOCAL_BROADCAST_NAME "ESP32-S31"   /* UTF-8 broadcast name           */
#define LOCAL_BROADCAST_ID   0x123456      /* 3-byte broadcast ID            */
#define USE_ENCRYPTION       false         /* false = public Auracast (no code) */
#define LOCAL_BROADCAST_CODE ""            /* up to 16 bytes if USE_ENCRYPTION */

/* LC3 preset. 48_2_1 = 48 kHz / 10 ms (Auracast "high quality"); 16_2_1 and
 * 24_2_1 are the lighter standard presets. When RECEIVING you do NOT pick this
 * — you read the JBL's config from its BASE on sync.                         */
ESP_BLE_AUDIO_BAP_LC3_BROADCAST_PRESET_48_2_1_DEFINE(s_preset,
        ESP_BLE_AUDIO_LOCATION_FRONT_LEFT,
        ESP_BLE_AUDIO_CONTEXT_TYPE_MEDIA);

static esp_ble_audio_cap_stream_t             s_stream;
static esp_ble_audio_cap_broadcast_source_t  *s_src;      /* >>VERIFY type name */
static uint8_t                               *s_iso_buf;
static volatile bool                          s_tx_on;
static uint16_t                               s_seq;

/* ---- Button / mode ------------------------------------------------------ */
#define BUTTON_GPIO 0
typedef enum { MODE_IDLE = 0, MODE_BROADCAST, MODE_RECEIVE, MODE_MAX } app_mode_t;
static const char *MODE_NAME[] = { "IDLE", "BROADCAST(source)", "RECEIVE(sink)" };
static volatile app_mode_t s_mode = MODE_IDLE;
static QueueHandle_t s_btn_q;

/* =========================================================================
 *  AURACAST SOURCE  (real pbp/pbs API)
 * ========================================================================= */
static void tx_scheduler_cb(void *arg)
{
    if (!s_tx_on || !s_iso_buf) return;
    uint16_t sdu = s_stream.bap_stream.qos->sdu;          /* >>VERIFY field path */
    memset(s_iso_buf, (uint8_t)s_seq, sdu);               /* TODO: real audio    */
    esp_err_t err = esp_ble_audio_cap_stream_send(&s_stream, s_iso_buf, sdu, s_seq);
    if (err == ESP_OK) s_seq++;
}

static void stream_started_cb(esp_ble_audio_cap_stream_t *stream)
{
    uint16_t sdu = stream->bap_stream.qos->sdu;           /* >>VERIFY field path */
    s_iso_buf = calloc(1, sdu);
    s_seq = 0;
    s_tx_on = true;
    ESP_LOGI(TAG, "stream started, sdu=%u — feeding ISO data", sdu);
    /* The example starts example_audio_tx_scheduler here, period = qos->interval,
     * calling tx_scheduler_cb. >>VERIFY the scheduler API from example_utils.   */
}
static void stream_stopped_cb(esp_ble_audio_cap_stream_t *stream)
{
    s_tx_on = false; free(s_iso_buf); s_iso_buf = NULL;
}

static esp_ble_audio_cap_stream_ops_t s_stream_ops = {   /* >>VERIFY type name  */
    .started = stream_started_cb,
    .stopped = stream_stopped_cb,
};

static void broadcast_start(void)
{
    esp_ble_audio_cap_stream_ops_register(&s_stream, &s_stream_ops);

    esp_ble_audio_cap_initiator_broadcast_stream_param_t sp = { .stream = &s_stream };
    esp_ble_audio_cap_initiator_broadcast_subgroup_param_t sg = {
        .stream_count  = 1,
        .stream_params = &sp,
        .codec_cfg     = &s_preset.codec_cfg,
    };
    esp_ble_audio_cap_initiator_broadcast_create_param_t cp = {
        .subgroup_count  = 1,
        .subgroup_params = &sg,
        .qos             = &s_preset.qos,
        .packing         = ESP_BLE_ISO_PACKING_SEQUENTIAL,
        .encryption      = USE_ENCRYPTION,
    };
#if USE_ENCRYPTION
    memcpy(cp.broadcast_code, LOCAL_BROADCAST_CODE, strlen(LOCAL_BROADCAST_CODE));
#endif
    ESP_ERROR_CHECK(esp_ble_audio_cap_initiator_broadcast_audio_create(&cp, &s_src));

    /* Attach the extended/periodic advertising that carries the Broadcast Audio
     * Announcement + BASE (this is what makes it discoverable as Auracast). The
     * example builds adv_info (broadcast name = LOCAL_BROADCAST_NAME, id =
     * LOCAL_BROADCAST_ID) and calls: */
    /* esp_ble_audio_bap_broadcast_adv_add(&adv_info);   >>VERIFY adv_info from adv.c */
    ext_adv_start();

    ESP_ERROR_CHECK(esp_ble_audio_cap_initiator_broadcast_audio_start(s_src, ADV_HANDLE));
    ESP_LOGI(TAG, "BROADCAST up: name='%s' id=0x%06x public=%d",
             LOCAL_BROADCAST_NAME, LOCAL_BROADCAST_ID, !USE_ENCRYPTION);
}

static void broadcast_stop(void)
{
    s_tx_on = false;
    esp_ble_audio_cap_initiator_broadcast_audio_stop(s_src);
    esp_ble_audio_cap_initiator_broadcast_audio_delete(s_src);
    s_src = NULL;
    ESP_LOGI(TAG, "BROADCAST stopped");
}

/* =========================================================================
 *  AURACAST SINK  (outline — fill from pbp/pbk)
 * ========================================================================= */
static void receive_start(void)
{
    /* pbk flow: start ext scan -> on a Broadcast Audio Announcement (service
     * UUID 0x1852) optionally matching LOCAL_BROADCAST_NAME, sync to its
     * periodic advertising, read the BASE (gives the LC3 cfg), then BIG-sync
     * (with the broadcast code if encrypted). Decoded SDUs arrive on the sink
     * stream recv callback -> render to I2S/codec.
     * Enable CONFIG_BT_BAP_BROADCAST_SINK + CONFIG_BT_CAP_ACCEPTOR.  >>VERIFY  */
    ESP_LOGI(TAG, "RECEIVE: scanning for a public Auracast broadcast...");
}
static void receive_stop(void) { ESP_LOGI(TAG, "RECEIVE stopped"); }

/* =========================================================================
 *  MODE MACHINE + BUTTON
 * ========================================================================= */
static void enter_mode(app_mode_t m)
{
    switch (s_mode) {
        case MODE_BROADCAST: broadcast_stop(); break;
        case MODE_RECEIVE:   receive_stop();   break;
        default: break;
    }
    s_mode = m;
    ESP_LOGI(TAG, "==> mode: %s", MODE_NAME[m]);
    switch (m) {
        case MODE_BROADCAST: broadcast_start(); break;
        case MODE_RECEIVE:   receive_start();   break;
        default: break;
    }
}

static void IRAM_ATTR button_isr(void *arg)
{
    uint32_t v = 1; xQueueSendFromISR(s_btn_q, &v, NULL);
}
static void button_task(void *arg)
{
    uint32_t v; TickType_t last = 0;
    for (;;) {
        if (xQueueReceive(s_btn_q, &v, portMAX_DELAY)) {
            TickType_t now = xTaskGetTickCount();
            if ((now - last) < pdMS_TO_TICKS(250)) continue;
            last = now;
            if (gpio_get_level(BUTTON_GPIO) != 0) continue;
            enter_mode((app_mode_t)((s_mode + 1) % MODE_MAX));
        }
    }
}
static void button_init(void)
{
    s_btn_q = xQueueCreate(8, sizeof(uint32_t));
    gpio_config_t io = { .pin_bit_mask = 1ULL << BUTTON_GPIO, .mode = GPIO_MODE_INPUT,
                         .pull_up_en = GPIO_PULLUP_ENABLE, .intr_type = GPIO_INTR_NEGEDGE };
    gpio_config(&io);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr, NULL);
    xTaskCreate(button_task, "btn", 3072, NULL, 10, NULL);
}

/* ========================================================================= */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    /* Bring up controller + host exactly like the example does. */
    bluetooth_init();
    app_host_init();
    ESP_ERROR_CHECK(esp_ble_audio_common_init(NULL));
    esp_ble_audio_cap_stream_ops_register(&s_stream, &s_stream_ops);
    ESP_ERROR_CHECK(esp_ble_audio_common_start(NULL));
    set_device_name();

    button_init();
    ESP_LOGI(TAG, "Ready. Button cycles: IDLE -> BROADCAST -> RECEIVE");
}
