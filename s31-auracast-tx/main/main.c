/*
 * SPDX-FileCopyrightText: 2021-2022 Nordic Semiconductor ASA
 * SPDX-FileContributor: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP32-S31 Auracast TEST-TONE source, built from USB_Audio_to_Auracast.
 *
 * Broadcasts a public (unencrypted) Auracast stream shaped like the one the
 * JBL Xtreme 5 itself sends in button mode:
 *   - 48 kHz / 10 ms LC3 (preset 48_2_1), 2 BIS (Front-Left + Front-Right)
 *   - Broadcast Audio Announcement 0x1852 + Broadcast_ID
 *   - Public Broadcast Announcement 0x1856, features 0x04 (High Quality, no enc)
 *   - Broadcast Name (AD 0x30)
 * Audio is a generated beep pattern (L 440 Hz, R 660 Hz, 400 ms on / 600 ms off)
 * so you can tell by ear that the Xtreme 5 is playing the S31's stream.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_ble_audio_lc3_defs.h"
#include "esp_ble_audio_bap_api.h"
#include "esp_ble_audio_pacs_api.h"
#include "esp_ble_audio_bap_lc3_preset_defs.h"

#include "ble_audio_example_init.h"
#include "ble_audio_example_utils.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lc3.h"

#include "esp_gatts_api.h"

#include "adv.h"
#include "jbl_gatt.h"

/* 48 kHz / 10 ms stereo — the "High Quality" Auracast format. */
ESP_BLE_AUDIO_BAP_LC3_BROADCAST_PRESET_48_2_1_DEFINE(preset_active,
                                                     ESP_BLE_AUDIO_LOCATION_FRONT_LEFT |
                                                     ESP_BLE_AUDIO_LOCATION_FRONT_RIGHT,
                                                     ESP_BLE_AUDIO_CONTEXT_TYPE_UNSPECIFIED |
                                                     ESP_BLE_AUDIO_CONTEXT_TYPE_MEDIA);   /* 0x0005, as the Xtreme 5 */

#define LOCAL_DEVICE_NAME      "ESP32-S31 Auracast"
#define LOCAL_DEVICE_NAME_LEN  (sizeof(LOCAL_DEVICE_NAME) - 1)
/* Same "<owner>'s JBL Xtreme 5" shape as the real speaker's Broadcast Name. */
#define LOCAL_BROADCAST_NAME   "ESP32-S31's JBL Xtreme 4"
#define LOCAL_BROADCAST_NAME_LEN (sizeof(LOCAL_BROADCAST_NAME) - 1)

#define LOCAL_BROADCAST_CODE   ""        /* empty -> OPEN broadcast, like the Xtreme 5 */
/* The Xtreme 5 always hosts with Broadcast_ID 0x112233, and the Xtreme 4's
 * receive matcher locks onto a broadcast by its 3-byte Broadcast_ID. Use the
 * same ID so a JBL joiner looking for a JBL party matches us. (Keep the real
 * Xtreme 5's own Auracast OFF while testing so the IDs don't clash.) */
/* NOTE: 0x112233 is the Xtreme 5's OWN fixed ID; copying it made the speaker
 * treat us as itself (hidden from its scan). Use a distinct ID. */
#define LOCAL_BROADCAST_ID     0x4A5834          /* "JX4": a fresh ID, not the Xtreme 5's */

/* ---- JBL host beacon (second advertiser) --------------------------------
 * Copy of the Xtreme 5's beacon on its second address: Service Data under
 * 16-bit UUID 0xFDDF, 16 bytes, with the "hosting" flag (byte 14) = 0xAA.
 * Bytes captured from the Xtreme 5 with Auracast off were
 *   69 21 c0 d7 00 db 96 cd 10 36 80 90 c0 40 00 00
 * and nRF showed the tail become 40 AA 00 with Auracast on. Replace with the
 * exact "on" capture if it differs elsewhere. */
#define JBL_BEACON_UUID        0xFDDF
/* Captured from the Xtreme 5 WHILE its 0x112233 broadcast was on air.
 * Byte 14 states seen: 00 off, AA on-but-idle, AC searching, B4 hosting+streaming. */
/* Emulating a SECOND Xtreme 5: same layout as the real one, own values.
 *   [0-1]  69 21   model (Xtreme 5)          - kept
 *   [5-6]  5a 31   per-device/source tag     - was 60 90 / db 96 on the real one
 *   [9]    50      battery (80%)             - was 35
 *   [14]   B4      hosting + streaming       - kept */
static const uint8_t jbl_beacon_payload[16] = {
    0x69, 0x21, 0xc0, 0xd7, 0x00, 0x4a, 0x34, 0xcd,
    0x10, 0x5a, 0x80, 0x90, 0xc0, 0x40, 0xB4, 0x00,
};

/* JBL party block the Xtreme 5 puts in its PERIODIC advertising, right after
 * the BASE: Manufacturer Specific Data, company 0xFDDF, TLV records
 *   [02 02 1f]            tag 0x02 = mode/role byte (same tag the Xtreme 4 parses)
 *   [03 f0 11 NN]         tag 0xF0, NN = counter
 *   [12 11 <17 bytes>]    tag 0x11, status block
 * Copied verbatim from a capture (NN=0x02, YY=0x49). */
static const uint8_t jbl_pa_mfg_ad[] = {
    0x1d, 0xff, 0xdf, 0xfd,
    0x02, 0x02, 0x1f,
    0x03, 0xf0, 0x11, 0x02,
    0x12, 0x11, 0x01, 0x01, 0x01, 0x00, 0x49, 0x00, 0x01, 0x01, 0x3c,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x4a, 0x34, 0x01,  /* tail was 24 b9 01 */
};
#define JBL_BEACON_NAME        "S31 JBL Xtreme 4"
/* Random static address for the beacon (top two bits 11). */
static const uint8_t jbl_beacon_addr[6] = { 0xC4, 0x58, 0x34, 0x53, 0x31, 0x04 };

/* Xtreme 4 firmware (FUN_00022f5c / FUN_0002a78c) puts a Manufacturer Specific
 * AD in the EXTENDED advertising next to 0x1852 + Broadcast Name, and its scan
 * classifier (FUN_000223bc -> FUN_00022370) reads the FIRST 0xFF AD of an
 * extended scan report: company 0xFDDF/0x0057, nested tag 0x02 = role byte.
 * So we also carry the JBL block in the extended advert, not only in the PA. */
#define JBL_EXT_MFG_AD     1

/* Public Broadcast Announcement (Auracast/PBP) so finders/assistants list it. */
#define PBA_UUID_VAL           0x1856
#define PBA_FEATURES_HQ        0x04      /* bit0 enc=0, bit2 High-Quality present */
#define PBA_PROGRAM_INFO       "Test Tone"

#define AD_TYPE_BROADCAST_NAME 0x30

/* Tone generator */
#define TONE_SR_HZ             48000
#define TONE_DT_US             10000
#define TONE_HZ_L              440.0f
#define TONE_HZ_R              660.0f
#define TONE_AMPL              0.25f     /* ~ -12 dBFS */
#define TONE_ON_MS             400
#define TONE_PERIOD_MS         1000

static lc3_encoder_t     g_enc[2];
static void             *g_enc_mem[2];
static int               g_frame_samples;
static SemaphoreHandle_t g_encode_sem;   /* scheduler tick -> encode_task */
static volatile uint32_t g_frames_sent;

#define STREAM_COUNT            CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT
#define SUBGROUP_COUNT          CONFIG_BT_BAP_BROADCAST_SRC_SUBGROUP_COUNT

static struct broadcast_source_stream {
    esp_ble_audio_bap_stream_t stream;
    uint16_t seq_num;
    uint8_t *data;
    example_audio_tx_scheduler_t scheduler;
} streams[STREAM_COUNT];

static esp_ble_audio_bap_broadcast_source_t *broadcast_source;

static int stream_index(const esp_ble_audio_bap_stream_t *stream)
{
    for (size_t i = 0; i < ARRAY_SIZE(streams); i++) {
        if (&streams[i].stream == stream) {
            return (int)i;
        }
    }
    return -1;
}

static void stream_started_cb(esp_ble_audio_bap_stream_t *stream)
{
    struct broadcast_source_stream *source_stream = CONTAINER_OF(stream,
                                                                 struct broadcast_source_stream,
                                                                 stream);
    int idx = stream_index(stream);
    esp_err_t err;

    ESP_LOGI(TAG, "[SRC #%d] Stream started", idx);

    if (source_stream->stream.qos == NULL || source_stream->stream.qos->sdu == 0) {
        ESP_LOGE(TAG, "[SRC #%d] Invalid QoS", idx);
        return;
    }

    if (source_stream->data == NULL) {
        source_stream->data = calloc(1, source_stream->stream.qos->sdu);
        if (source_stream->data == NULL) {
            ESP_LOGE(TAG, "[SRC #%d] Failed to alloc TX buffer (sdu %u)",
                     idx, source_stream->stream.qos->sdu);
            return;
        }
    }

    source_stream->seq_num = 0;
    /* Stream 0's drift-compensated scheduler paces BOTH channels. */
    if (idx == 0) {
        example_audio_tx_scheduler_reset(&source_stream->scheduler);
        err = example_audio_tx_scheduler_start(&source_stream->scheduler,
                                               preset_active.qos.interval);
        if (err) {
            ESP_LOGE(TAG, "[SRC #%d] Scheduler start failed, err %d", idx, err);
        }
    }
}

static void stream_stopped_cb(esp_ble_audio_bap_stream_t *stream, uint8_t reason)
{
    struct broadcast_source_stream *source_stream = CONTAINER_OF(stream,
                                                                 struct broadcast_source_stream,
                                                                 stream);
    ESP_LOGI(TAG, "[SRC #%d] Stream stopped, reason 0x%02x", stream_index(stream), reason);
    example_audio_tx_scheduler_stop(&source_stream->scheduler);
}

static void stream_disconnected_cb(esp_ble_audio_bap_stream_t *stream, uint8_t reason)
{
    struct broadcast_source_stream *source_stream = CONTAINER_OF(stream,
                                                                 struct broadcast_source_stream,
                                                                 stream);
    ESP_LOGI(TAG, "[SRC #%d] ISO disconnected, reason 0x%02x", stream_index(stream), reason);
    example_audio_tx_scheduler_stop(&source_stream->scheduler);
}

static void stream_sent_cb(esp_ble_audio_bap_stream_t *stream, void *user_data)
{
    struct broadcast_source_stream *source_stream = CONTAINER_OF(stream,
                                                                 struct broadcast_source_stream,
                                                                 stream);
    char name[24];

    snprintf(name, sizeof(name), "SRC #%d", stream_index(stream));
    example_audio_tx_scheduler_on_sent(&source_stream->scheduler, user_data, TAG, name);
}

static esp_ble_audio_bap_stream_ops_t stream_ops = {
    .started      = stream_started_cb,
    .stopped      = stream_stopped_cb,
    .sent         = stream_sent_cb,
    .disconnected = stream_disconnected_cb,
};

static bool stream_is_streaming(const esp_ble_audio_bap_stream_t *stream)
{
    esp_ble_audio_bap_ep_info_t ep_info = {0};

    if (stream->ep == NULL) {
        return false;
    }
    if (esp_ble_audio_bap_ep_get_info(stream->ep, &ep_info)) {
        return false;
    }
    return (ep_info.state == ESP_BLE_AUDIO_BAP_EP_STATE_STREAMING);
}

/* Fill one 10 ms interleaved stereo frame with the beep pattern. */
static void tone_fill(int16_t *pcm, int fs)
{
    static float    ph[2];
    static uint32_t clk;   /* running sample count */
    const float     two_pi = 6.28318530718f;
    const float     inc[2] = { two_pi * TONE_HZ_L / TONE_SR_HZ, two_pi * TONE_HZ_R / TONE_SR_HZ };
    const uint32_t  period = TONE_SR_HZ * TONE_PERIOD_MS / 1000;
    const uint32_t  on     = TONE_SR_HZ * TONE_ON_MS / 1000;
    const uint32_t  ramp   = TONE_SR_HZ / 200;   /* 5 ms fade in/out, no clicks */

    for (int n = 0; n < fs; n++) {
        uint32_t pos = clk % period;
        float env = 0.0f;
        if (pos < on) {
            env = 1.0f;
            if (pos < ramp) {
                env = (float)pos / ramp;
            } else if (pos > on - ramp) {
                env = (float)(on - pos) / ramp;
            }
        }
        for (int ch = 0; ch < 2; ch++) {
            pcm[2 * n + ch] = (int16_t)(TONE_AMPL * env * 32767.0f * sinf(ph[ch]));
            ph[ch] += inc[ch];
            if (ph[ch] > two_pi) {
                ph[ch] -= two_pi;
            }
        }
        clk++;
    }
}

/* One SDU interval: generate a stereo frame, encode L+R, send both BIS with the
 * same sequence number. Runs in encode_task (large stack). */
static void broadcast_source_tx(void)
{
    static int16_t pcm[480 * 2];

    if (!stream_is_streaming(&streams[0].stream)) {
        return;
    }

    int fs = g_frame_samples;
    if (fs > 480) {
        fs = 480;
    }
    tone_fill(pcm, fs);

    for (int ch = 0; ch < 2 && ch < (int)ARRAY_SIZE(streams); ch++) {
        struct broadcast_source_stream *s = &streams[ch];
        if (g_enc[ch] == NULL || s->data == NULL ||
            s->stream.qos == NULL || s->stream.qos->sdu == 0) {
            continue;
        }
        int opf = s->stream.qos->sdu;
        if (lc3_encode(g_enc[ch], LC3_PCM_FORMAT_S16, pcm + ch, 2, opf, s->data) != 0) {
            continue;
        }
        esp_ble_audio_bap_stream_send(&s->stream, s->data, opf, streams[0].seq_num);
    }
    streams[0].seq_num++;
    g_frames_sent++;
}

static void tx_scheduler_cb(void *arg)
{
    (void)arg;
    if (g_encode_sem != NULL) {
        xSemaphoreGive(g_encode_sem);
    }
}

static void source_started_cb(esp_ble_audio_bap_broadcast_source_t *source)
{
    ESP_LOGW(TAG, "Broadcast source started: '%s' Broadcast_ID 0x%06x (open, 48 kHz stereo)",
             LOCAL_BROADCAST_NAME, LOCAL_BROADCAST_ID);
}

static void source_stopped_cb(esp_ble_audio_bap_broadcast_source_t *source, uint8_t reason)
{
    ESP_LOGI(TAG, "Broadcast source stopped, reason 0x%02x", reason);

    for (size_t i = 0; i < ARRAY_SIZE(streams); i++) {
        if (streams[i].data != NULL) {
            free(streams[i].data);
            streams[i].data = NULL;
        }
    }
}

static esp_err_t broadcast_source_setup(void)
{
    esp_ble_audio_bap_broadcast_source_subgroup_param_t subgroup_param[SUBGROUP_COUNT];
    esp_ble_audio_bap_broadcast_source_stream_param_t stream_params[STREAM_COUNT];
    esp_ble_audio_bap_broadcast_source_param_t create_param = {0};
    size_t streams_per_subgroup;
    uint8_t left[] = {
        ESP_BLE_AUDIO_CODEC_DATA(ESP_BLE_AUDIO_CODEC_CFG_CHAN_ALLOC,
                                 EXAMPLE_BYTES_LIST_LE32(ESP_BLE_AUDIO_LOCATION_FRONT_LEFT))
    };
    uint8_t right[] = {
        ESP_BLE_AUDIO_CODEC_DATA(ESP_BLE_AUDIO_CODEC_CFG_CHAN_ALLOC,
                                 EXAMPLE_BYTES_LIST_LE32(ESP_BLE_AUDIO_LOCATION_FRONT_RIGHT))
    };
    static esp_ble_audio_bap_broadcast_source_cb_t broadcast_source_cb = {
        .started = source_started_cb,
        .stopped = source_stopped_cb,
    };
    esp_err_t err;

    err = esp_ble_audio_bap_broadcast_source_register_cb(&broadcast_source_cb);
    if (err) {
        ESP_LOGE(TAG, "Failed to register broadcast source callbacks, err %d", err);
        return err;
    }

    streams_per_subgroup = ARRAY_SIZE(stream_params) / ARRAY_SIZE(subgroup_param);

    for (size_t i = 0; i < ARRAY_SIZE(subgroup_param); i++) {
        subgroup_param[i].params_count = streams_per_subgroup;
        subgroup_param[i].params = stream_params + i * streams_per_subgroup;
        subgroup_param[i].codec_cfg = &preset_active.codec_cfg;
    }

    for (size_t i = 0; i < ARRAY_SIZE(stream_params); i++) {
        stream_params[i].stream = &streams[i].stream;
        stream_params[i].data = (i == 0 ? left : right);
        stream_params[i].data_len = (i == 0 ? sizeof(left) : sizeof(right));
        esp_ble_audio_bap_stream_cb_register(stream_params[i].stream, &stream_ops);
    }

    /* The Xtreme 5's BASE uses an 80 ms presentation delay. */
    preset_active.qos.pd = 80000;

    create_param.params_count = ARRAY_SIZE(subgroup_param);
    create_param.params = subgroup_param;
    create_param.qos = &preset_active.qos;
    create_param.encryption = (strlen(LOCAL_BROADCAST_CODE) > 0);
    create_param.packing = ESP_BLE_ISO_PACKING_SEQUENTIAL;
    if (create_param.encryption) {
        memcpy(create_param.broadcast_code, LOCAL_BROADCAST_CODE, strlen(LOCAL_BROADCAST_CODE));
    }

    ESP_LOGI(TAG, "Creating broadcast source: %u subgroup(s), %u stream(s)/subgroup",
             ARRAY_SIZE(subgroup_param), streams_per_subgroup);

    err = esp_ble_audio_bap_broadcast_source_create(&create_param, &broadcast_source);
    if (err) {
        ESP_LOGE(TAG, "Failed to create broadcast source, err %d", err);
        return err;
    }
    return 0;
}

/* Extended advertising, field-for-field like the Xtreme 5's broadcast:
 *   0x1852 Broadcast Audio Announcement + Broadcast_ID
 *   0x1856 Public Broadcast Announcement = 04 00 (High Quality, no metadata)
 *   Appearance 0x0885
 *   Broadcast Name (AD 0x30)
 * (No Complete Local Name - the Xtreme 5's broadcast advert has none.) */
#define AD_TYPE_APPEARANCE     0x19
#define JBL_APPEARANCE         0x0885

static uint8_t *ext_adv_data_get(uint8_t *data_len)
{
    const uint32_t broadcast_id = LOCAL_BROADCAST_ID;
    uint8_t *data;
    uint8_t i = 0;

    *data_len = 7 + 6 + 4 + (2 + LOCAL_BROADCAST_NAME_LEN) +
                (JBL_EXT_MFG_AD ? sizeof(jbl_pa_mfg_ad) : 0);

    data = calloc(1, *data_len);
    if (data == NULL) {
        return NULL;
    }

    data[i++] = 0x06; /* 1 + 2 + 3 */
    data[i++] = EXAMPLE_AD_TYPE_SERVICE_DATA16;
    data[i++] = (ESP_BLE_AUDIO_UUID_BROADCAST_AUDIO_VAL & 0xFF);
    data[i++] = ((ESP_BLE_AUDIO_UUID_BROADCAST_AUDIO_VAL >> 8) & 0xFF);
    data[i++] = (broadcast_id & 0xFF);
    data[i++] = ((broadcast_id >> 8) & 0xFF);
    data[i++] = ((broadcast_id >> 16) & 0xFF);

    data[i++] = 0x05; /* 1 + 2 + 2 */
    data[i++] = EXAMPLE_AD_TYPE_SERVICE_DATA16;
    data[i++] = (PBA_UUID_VAL & 0xFF);
    data[i++] = ((PBA_UUID_VAL >> 8) & 0xFF);
    data[i++] = PBA_FEATURES_HQ;   /* 0x04 */
    data[i++] = 0x00;              /* no PBA metadata */

    data[i++] = 0x03;
    data[i++] = AD_TYPE_APPEARANCE;
    data[i++] = JBL_APPEARANCE & 0xFF;
    data[i++] = (JBL_APPEARANCE >> 8) & 0xFF;

    data[i++] = LOCAL_BROADCAST_NAME_LEN + 1;
    data[i++] = AD_TYPE_BROADCAST_NAME;
    memcpy(data + i, LOCAL_BROADCAST_NAME, LOCAL_BROADCAST_NAME_LEN);
    i += LOCAL_BROADCAST_NAME_LEN;

    if (JBL_EXT_MFG_AD) {
        memcpy(data + i, jbl_pa_mfg_ad, sizeof(jbl_pa_mfg_ad));
        i += sizeof(jbl_pa_mfg_ad);
    }

    char h[3 * 80 + 1] = {0};
    for (int k = 0; k < i && k < 80; k++) {
        sprintf(h + 3 * k, "%02x ", data[k]);
    }
    ESP_LOGW(TAG, "EXT ADV (%u B): %s", i, h);

    return data;
}

/* Periodic data = BASE (cached once) + JBL party block. The Xtreme 5 advances
 * the block's counter (tag 0xF0, NN) and a changing byte in the tag-0x11 block
 * (YY) while it hosts, so we do the same. */
/* 1 = append the JBL 0xFDDF party block to the periodic data; 0 = BASE only. */
#define JBL_PA_BLOCK       1

#define JBL_PA_NN_OFFSET   10      /* index of NN in jbl_pa_mfg_ad */
#define JBL_PA_YY_OFFSET   17      /* index of YY in jbl_pa_mfg_ad */

static uint8_t s_base_ad[128];
static uint8_t s_base_ad_len;
static uint8_t s_pa_nn = 0x02;

static uint8_t *per_adv_data_build(uint8_t *data_len)
{
    uint8_t *data;

    if (s_base_ad_len == 0) {
        NET_BUF_SIMPLE_DEFINE(base_buf, 120);
        esp_err_t err = esp_ble_audio_bap_broadcast_source_get_base(broadcast_source, &base_buf);
        if (err) {
            ESP_LOGE(TAG, "Failed to get encoded BASE, err %d", err);
            return NULL;
        }
        /* base_buf.len already includes the 2-octet UUID */
        s_base_ad[0] = 1 + base_buf.len;
        s_base_ad[1] = EXAMPLE_AD_TYPE_SERVICE_DATA16;
        memcpy(s_base_ad + 2, base_buf.data, base_buf.len);
        s_base_ad_len = 2 + base_buf.len;
    }

    const size_t blk = JBL_PA_BLOCK ? sizeof(jbl_pa_mfg_ad) : 0;
    *data_len = s_base_ad_len + blk;
    data = calloc(1, *data_len);
    if (data == NULL) {
        return NULL;
    }
    memcpy(data, s_base_ad, s_base_ad_len);
    if (blk) {
        memcpy(data + s_base_ad_len, jbl_pa_mfg_ad, sizeof(jbl_pa_mfg_ad));
        data[s_base_ad_len + JBL_PA_NN_OFFSET] = s_pa_nn;
        data[s_base_ad_len + JBL_PA_YY_OFFSET] = (uint8_t)(xTaskGetTickCount() / pdMS_TO_TICKS(10));
    }
    return data;
}

static uint8_t *per_adv_data_get(uint8_t *data_len)
{
    return per_adv_data_build(data_len);
}

/* Keep the JBL party block alive, like a real host. */
static void pa_update_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        s_pa_nn++;
        uint8_t len = 0;
        uint8_t *d = per_adv_data_build(&len);
        if (d != NULL) {
            per_adv_data_update(d, len);
            free(d);
        }
    }
}

static int broadcast_start(void)
{
    esp_ble_audio_bap_broadcast_adv_info_t info = {
        .adv_handle = ADV_HANDLE,
    };
    uint8_t *ext_data = NULL;
    uint8_t *per_data = NULL;
    uint8_t ext_len = 0;
    uint8_t per_len = 0;
    int err = 0;

    ext_data = ext_adv_data_get(&ext_len);
    if (ext_data == NULL) {
        err = -ENOMEM;
        goto end;
    }
    per_data = per_adv_data_get(&per_len);
    if (per_data == NULL) {
        err = -ENOMEM;
        goto end;
    }

    err = ext_adv_start(ext_data, ext_len, per_data, per_len);
    if (err) {
        goto end;
    }
    err = esp_ble_audio_bap_broadcast_adv_add(&info);
    if (err) {
        ESP_LOGE(TAG, "Failed to add adv for broadcast source, err %d", err);
        goto end;
    }
    err = esp_ble_audio_bap_broadcast_source_start(broadcast_source, ADV_HANDLE);
    if (err) {
        ESP_LOGE(TAG, "Failed to start broadcast source, err %d", err);
    }

end:
    free(ext_data);
    free(per_data);
    return err;
}

/* Log anything a speaker does over a connection to the beacon, then re-arm it. */
static void gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                     esp_ble_gatts_cb_param_t *p)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "GATTS app registered (if %d)", gatts_if);
        jbl_gatt_register(gatts_if);
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        jbl_gatt_on_attr_tab(p);
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGW(TAG, ">>> GATT CONNECT from %02x:%02x:%02x:%02x:%02x:%02x (conn_id %u)",
                 p->connect.remote_bda[0], p->connect.remote_bda[1], p->connect.remote_bda[2],
                 p->connect.remote_bda[3], p->connect.remote_bda[4], p->connect.remote_bda[5],
                 p->connect.conn_id);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGW(TAG, ">>> GATT DISCONNECT (conn_id %u, reason 0x%02x)",
                 p->disconnect.conn_id, p->disconnect.reason);
        beacon_adv_restart();
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "peer MTU %u", p->mtu.mtu);
        break;
    case ESP_GATTS_READ_EVT:
        ESP_LOGW(TAG, ">>> peer READ handle 0x%04x", p->read.handle);
        break;
    case ESP_GATTS_WRITE_EVT: {
        char h[3 * 32 + 1] = {0};
        int n = p->write.len > 32 ? 32 : p->write.len;
        for (int i = 0; i < n; i++) {
            sprintf(h + 3 * i, "%02x ", p->write.value[i]);
        }
        ESP_LOGW(TAG, ">>> peer WRITE %s handle 0x%04x len %u: %s",
                 jbl_gatt_handle_name(p->write.handle), p->write.handle, p->write.len, h);
        break;
    }
    default:
        break;
    }
}

/* Start the JBL-style host beacon next to the broadcast. */
static int jbl_beacon_start(void)
{
    uint8_t adv[4 + sizeof(jbl_beacon_payload)];
    uint8_t rsp[2 + sizeof(JBL_BEACON_NAME) - 1];
    uint8_t i = 0;

    adv[i++] = 3 + sizeof(jbl_beacon_payload);         /* 0x13 */
    adv[i++] = EXAMPLE_AD_TYPE_SERVICE_DATA16;          /* 0x16 */
    adv[i++] = JBL_BEACON_UUID & 0xFF;                  /* DF */
    adv[i++] = (JBL_BEACON_UUID >> 8) & 0xFF;           /* FD */
    memcpy(adv + i, jbl_beacon_payload, sizeof(jbl_beacon_payload));

    rsp[0] = sizeof(JBL_BEACON_NAME);                   /* len incl. type byte */
    rsp[1] = EXAMPLE_AD_TYPE_NAME_COMPLETE;
    memcpy(rsp + 2, JBL_BEACON_NAME, sizeof(JBL_BEACON_NAME) - 1);

    char h[3 * sizeof(jbl_beacon_payload) + 1] = {0};
    for (size_t k = 0; k < sizeof(jbl_beacon_payload); k++) {
        sprintf(h + 3 * k, "%02x ", jbl_beacon_payload[k]);
    }
    ESP_LOGW(TAG, "JBL beacon 0xFDDF payload: %s", h);

    return beacon_adv_start(adv, sizeof(adv), rsp, sizeof(rsp), jbl_beacon_addr);
}

static esp_err_t tone_lc3_init(void)
{
    g_frame_samples = lc3_frame_samples(TONE_DT_US, TONE_SR_HZ);   /* 480 @ 48k/10ms */
    for (int ch = 0; ch < 2; ch++) {
        g_enc_mem[ch] = malloc(lc3_encoder_size(TONE_DT_US, TONE_SR_HZ));
        if (g_enc_mem[ch] == NULL) {
            return ESP_ERR_NO_MEM;
        }
        g_enc[ch] = lc3_setup_encoder(TONE_DT_US, TONE_SR_HZ, TONE_SR_HZ, g_enc_mem[ch]);
        if (g_enc[ch] == NULL) {
            return ESP_FAIL;
        }
    }
    ESP_LOGI(TAG, "LC3 encoders ready: %d Hz, %d us, %d samples/frame",
             TONE_SR_HZ, TONE_DT_US, g_frame_samples);
    return ESP_OK;
}

/* Encoder task (lc3_encode needs several KB of stack), paced by the
 * drift-compensated scheduler via the semaphore. */
static void encode_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    uint32_t   last_sent = 0;

    for (;;) {
        if (xSemaphoreTake(g_encode_sem, pdMS_TO_TICKS(500)) == pdTRUE) {
            broadcast_source_tx();
        }
        if (xTaskGetTickCount() - last >= pdMS_TO_TICKS(2000)) {
            uint32_t now = g_frames_sent;
            ESP_LOGW(TAG, "TX STATS 2s: %lu stereo LC3 frames sent (expect ~200)",
                     (unsigned long)(now - last_sent));
            last_sent = now;
            last = xTaskGetTickCount();
        }
    }
}

void app_main(void)
{
    esp_err_t err;

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "Failed to initialize BLE, err %d", err);
        return;
    }
    err = app_host_init();
    if (err) {
        ESP_LOGE(TAG, "Failed to init host, err %d", err);
        return;
    }
    if (esp_ble_gatts_register_callback(gatts_cb) == ESP_OK) {
        esp_ble_gatts_app_register(0x0A);
    }
    err = esp_ble_audio_common_init(NULL);
    if (err) {
        ESP_LOGE(TAG, "Failed to initialize audio, err %d", err);
        return;
    }
    err = broadcast_source_setup();
    if (err) {
        return;
    }
    err = esp_ble_audio_common_start(NULL);
    if (err) {
        ESP_LOGE(TAG, "Failed to start audio, err %d", err);
        return;
    }

    err = tone_lc3_init();
    if (err) {
        ESP_LOGE(TAG, "Failed to init LC3 encoders, err %d", err);
        return;
    }

    for (size_t i = 0; i < ARRAY_SIZE(streams); i++) {
        err = example_audio_tx_scheduler_init(&streams[i].scheduler, tx_scheduler_cb, &streams[i]);
        if (err) {
            ESP_LOGE(TAG, "[SRC #%zu] Scheduler init failed, err %d", i, err);
            return;
        }
    }

    g_encode_sem = xSemaphoreCreateCounting(16, 0);
    if (g_encode_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create encode semaphore");
        return;
    }
    xTaskCreatePinnedToCore(encode_task, "lc3_enc", 12288, NULL,
                            configMAX_PRIORITIES - 4, NULL, 1);

    err = broadcast_start();
    if (err) {
        ESP_LOGE(TAG, "Failed to start broadcast, err %d", err);
        return;
    }

    err = jbl_beacon_start();
    if (err) {
        ESP_LOGE(TAG, "Failed to start JBL host beacon, err %d", err);
    }

    if (JBL_PA_BLOCK) {
        xTaskCreate(pa_update_task, "jbl_pa", 4096, NULL, 5, NULL);
    }
    ESP_LOGW(TAG, "JBL party block in periodic data: %s", JBL_PA_BLOCK ? "ON" : "OFF");
}
