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
#include "esp_timer.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lc3.h"

#include "esp_gatts_api.h"

#include "adv.h"
#include "jbl_gatt.h"
#include "a2dp_bridge.h"
#include "esp_heap_caps.h"
#include "esp_bt_device.h"
#include "jbl_assistant.h"
#include "jbl_v5_server.h"
#include "app_task.h"

/* 48 kHz / 10 ms stereo â€” the "High Quality" Auracast format. */
/* 48_2_2 = high-reliability variant: same codec (48 kHz/10 ms/100 B), RTN 4 but
 * 65 ms max transport latency (vs 20 ms) so retransmissions survive BR/EDR and
 * scan traffic sharing the radio. */
ESP_BLE_AUDIO_BAP_LC3_BROADCAST_PRESET_48_2_2_DEFINE(preset_active,
                                                     ESP_BLE_AUDIO_LOCATION_FRONT_LEFT |
                                                     ESP_BLE_AUDIO_LOCATION_FRONT_RIGHT,
                                                     ESP_BLE_AUDIO_CONTEXT_TYPE_UNSPECIFIED |
                                                     ESP_BLE_AUDIO_CONTEXT_TYPE_MEDIA);   /* 0x0005, as the Xtreme 5 */

#define LOCAL_DEVICE_NAME      "ESP32-S31 Auracast"
#define LOCAL_DEVICE_NAME_LEN  (sizeof(LOCAL_DEVICE_NAME) - 1)
/* Same "<owner>'s JBL Xtreme 5" shape as the real speaker's Broadcast Name. */
#define LOCAL_BROADCAST_NAME   "ESP32-S31's JBL Xtreme 5"
/* Classic (BR/EDR) name phones see when pairing for music. */
#define JBL_BREDR_NAME         "S31 JBL Xtreme 5"
#define LOCAL_BROADCAST_NAME_LEN (sizeof(LOCAL_BROADCAST_NAME) - 1)

#define LOCAL_BROADCAST_CODE   ""        /* empty -> OPEN broadcast, like the Xtreme 5 */
/* The Xtreme 5 always hosts with Broadcast_ID 0x112233, and the Xtreme 4's
 * receive matcher locks onto a broadcast by its 3-byte Broadcast_ID. Use the
 * same ID so a JBL joiner looking for a JBL party matches us. (Keep the real
 * Xtreme 5's own Auracast OFF while testing so the IDs don't clash.) */
/* NOTE: 0x112233 is the Xtreme 5's OWN fixed ID; copying it made the speaker
 * treat us as itself (hidden from its scan). Use a distinct ID. */
/* Every JBL Xtreme 5 master seen so far broadcasts with this fixed ID; with our
 * own identity (macCrc) it should read as "another JBL master", not as a copy. */
#define LOCAL_BROADCAST_ID     0x112233

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
/* V5 legacy beacon layout, decoded from the JBL One app
 * (com/harman/v5protocol/discover/a.java):
 *   [0-1] PID 2169 (Xtreme 5), byte-reversed on air   69 21
 *   [2-3] field-presence mask (LE)                     0x97C0
 *         b15 colorID, b12 btMacAddressCrc, b10 battery, b9 deviceMiscInfo,
 *         b8 partyMethodInfo, b7 partyInfo, b6 stereoGroupId
 *         (b14 srcName1Crc = CRC of the connected phone's name: left out)
 *   colorID 00
 *   btMacAddressCrc = CRC-16/ARC of OUR Classic BT MAC (the app's device key,
 *                     "V5_"+crc; the real Xtreme 5's is 0x10CD = CRC of 78:66:F3:AB:11:CE)
 *   battery 0x50 = 80 %, not charging
 *   deviceMiscInfo 0x9080 = connectable | LED | BR/EDR supported
 *   partyMethodInfo 0xC0 = Auracast supported | long-lasting stereo supported
 *   partyInfo 0xB440 = party on, group Party, Connected, role PRIMARY (host)
 *   stereoGroupId 00 */
#define JBL_PID                0x2169
#define JBL_ADV_MASK           0x97C0   /* + bit 14 (srcName1Crc) once the phone's name is known */
#define JBL_BATTERY            0x50
/* misc: LED | BR/EDR supported, connectable bit (15) CLEAR - with it set JBL One
 * takes an LE-only path and never opens the device. */
#define JBL_MISC_INFO          0x1080
#define JBL_PARTY_METHOD       0xC0
#define JBL_PARTY_INFO_HOST    0xB440
static uint8_t jbl_beacon_payload[16];
static uint8_t jbl_beacon_len;
/* CRC-16/ARC of the connected phone's Bluetooth name: JBL One only treats a
 * speaker as "connected to this phone" when its beacon carries it. */
static volatile bool s_src_name_known;
static volatile uint16_t s_src_name_crc;
static volatile bool s_beacon_dirty;

static uint16_t crc16_arc(const uint8_t *d, size_t n)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
        }
    }
    return crc;
}

static uint16_t jbl_beacon_build(void)
{
    const uint8_t *mac = esp_bt_dev_get_address();     /* BR/EDR MAC, display order */
    uint16_t crc = crc16_arc(mac, 6);
    uint8_t *p = jbl_beacon_payload;
    int i = 0;
    uint16_t mask = JBL_ADV_MASK | (s_src_name_known ? (1 << 14) : 0);
    p[i++] = JBL_PID & 0xFF;         p[i++] = JBL_PID >> 8;            /* 69 21 */
    p[i++] = mask & 0xFF;            p[i++] = mask >> 8;               /* c0 97 / c0 d7 */
    p[i++] = 0x00;                                                    /* colorID */
    if (s_src_name_known) {
        p[i++] = s_src_name_crc & 0xFF; p[i++] = s_src_name_crc >> 8;  /* srcName1Crc */
    }
    p[i++] = crc & 0xFF;             p[i++] = crc >> 8;                /* MAC CRC */
    p[i++] = JBL_BATTERY;
    p[i++] = JBL_MISC_INFO & 0xFF;   p[i++] = JBL_MISC_INFO >> 8;
    p[i++] = JBL_PARTY_METHOD;
    p[i++] = JBL_PARTY_INFO_HOST & 0xFF; p[i++] = JBL_PARTY_INFO_HOST >> 8;
    p[i++] = 0x00;                                                    /* stereoGroupId */
    jbl_beacon_len = i;
    return crc;
}

/* Full AD for the beacon: Service Data 0xFDDF + payload. Returns length. */
static uint8_t jbl_beacon_ad(uint8_t *adv)
{
    jbl_beacon_build();
    adv[0] = 3 + jbl_beacon_len;
    adv[1] = EXAMPLE_AD_TYPE_SERVICE_DATA16;
    adv[2] = 0xDF;
    adv[3] = 0xFD;
    memcpy(adv + 4, jbl_beacon_payload, jbl_beacon_len);
    return 4 + jbl_beacon_len;
}

/* Called (from the Classic stack) with the connected phone's Bluetooth name. */
static void on_phone_name(const char *name)
{
    s_src_name_crc = crc16_arc((const uint8_t *)name, strlen(name));
    s_src_name_known = true;
    s_beacon_dirty = true;                       /* pa_update_task pushes it */
    ESP_LOGW(TAG, "phone name '%s' -> srcName1Crc 0x%04x added to the JBL beacon", name, s_src_name_crc);
}

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
                0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0xb9, 0x01,  /* as the real master */
};
#define JBL_BEACON_NAME        "S31 JBL Xtreme 5"
/* Random static address for the beacon (top two bits 11). */
static const uint8_t jbl_beacon_addr[6] = { 0xC4, 0x58, 0x34, 0x53, 0x31, 0x04 };

/* Xtreme 4 firmware (FUN_00022f5c / FUN_0002a78c) puts a Manufacturer Specific
 * AD in the EXTENDED advertising next to 0x1852 + Broadcast Name, and its scan
 * classifier (FUN_000223bc -> FUN_00022370) reads the FIRST 0xFF AD of an
 * extended scan report: company 0xFDDF/0x0057, nested tag 0x02 = role byte.
 * So we also carry the JBL block in the extended advert, not only in the PA. */
#define JBL_EXT_MFG_AD     0   /* the real Xtreme 5 master has no mfg AD in its ext adv */

/* Public Broadcast Announcement (Auracast/PBP) so finders/assistants list it. */
#define PBA_UUID_VAL           0x1856
#define PBA_FEATURES_HQ        0x04      /* bit0 enc=0, bit2 High-Quality present */
#define PBA_PROGRAM_INFO       "Test Tone"

#define AD_TYPE_BROADCAST_NAME 0x30

/* 0 = silence when no phone is streaming; 1 = test beeps. */
#define TEST_TONE              0
/* Garble isolation: while the phone is paused, play S31-generated sweeps
 * (A: 48 kHz direct, B: 44.1 kHz through the resampler), no SBC. */
#ifndef SWEEP_TEST
#define SWEEP_TEST             1
#endif
/* A/B test for the garble: record the last REPLAY_SECONDS of phone music (as
 * encoded) into PSRAM; when the phone pauses, loop it over Auracast while the
 * Classic link is idle. Clean replay => BR/EDR coexistence breaks the BIG.
 * Debug only, off by default: it costs ~5.8 MB of PSRAM (30 s of PCM, taken on
 * the first frame) plus a 1.9 KB memcpy every 10 ms on the encode path. Build
 * with -DREPLAY_TEST=1 (or flip the default) to bring it back. */
#ifndef REPLAY_TEST
#define REPLAY_TEST            0
#endif
#define REPLAY_SECONDS         30
static volatile bool g_replaying;   /* replay loop is on air (beacon stays slow, like streaming) */

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
static volatile uint32_t g_send_errors;     /* ISO SDUs the stack refused (radio/buffer busy) */
static volatile uint32_t g_glitches;        /* PCM discontinuities before encoding */
static volatile uint32_t g_clipped;         /* full-scale samples before encoding */
extern volatile uint8_t btc_a2dp_sink_bitpool_last, btc_a2dp_sink_bitpool_min, btc_a2dp_sink_bitpool_max;
extern volatile uint32_t btc_a2dp_sink_media_pkts, btc_a2dp_sink_media_seq_gaps;
static volatile uint32_t g_iso_done;        /* SDUs the controller reports as sent */
static volatile uint32_t g_iso_not_txd;     /* SDUs the controller could NOT put on air */
static volatile uint32_t g_iso_drift;       /* SDUs flagged as drifted vs the BIG anchor */
/* Controller-reported per-SDU timing (vendor fields, us): how long an SDU sat
 * inside the controller between arriving (sdu_rec_time) and going on air
 * (tx_time_stamp), plus tx vs its SDU reference time. */
static volatile uint32_t g_q_sum, g_q_max, g_q_n, g_ref_sum, g_ref_max;
static volatile uint32_t g_q_raw[4];
static volatile uint32_t g_late_ticks;      /* encode ticks > 15 ms after the previous one */
static volatile uint32_t g_max_gap_us;      /* worst gap between encode ticks */

/* Radio-stall probes (stream 0). The controller stamps each SDU with the BIG
 * anchor it went out on (sdu_ref_time) and its transmit time; with a 20 ms
 * ISO interval and BN 2 consecutive anchors must be 0 or 20 ms apart. A larger
 * step means BIG events went out without our audio (the radio was busy, or
 * the SDU was late) - receivers then hear PLC. Completion callback gaps show
 * stalls on the host side of HCI. */
static volatile uint32_t g_ref_prev, g_tx_prev;
static volatile uint32_t g_ref_jumps, g_ref_step_max;   /* anchor steps > 20 ms (missed BIG events) */
static volatile uint32_t g_tx_step_max;                 /* worst tx timestamp step */
static volatile int64_t  g_cb_prev_us;
static volatile uint32_t g_cb_gap_max, g_cb_gaps_30;    /* completion callback gaps */
static volatile uint32_t g_ref_hist[4];                 /* anchor step: 0, 20 ms, 40 ms, >40 ms */
volatile uint32_t lc3_enc_stat[5];                       /* liblc3: frames, bw<FB, bw changes, TNS on, attack */
static volatile uint32_t g_enc_us_sum, g_enc_us_max, g_enc_n;
/* LC3 loopback check (left channel): decode every encoded frame on the S31
 * and compare with the delayed input -> proves/disproves encoder bitstream
 * problems independently of the radio and the speaker. */
#define LC3_LOOPBACK 1
static lc3_decoder_t g_lb_dec;
static int g_lb_delay;
static int16_t g_lb_hist[480 + 512];
static volatile uint32_t g_lb_frames, g_lb_dec_err, g_lb_bad;
static volatile float g_lb_sig, g_lb_err, g_lb_worst = 999.0f;   /* lc3_encode time, both channels */
static volatile uint32_t g_iso_backlog_drops; /* SDU pairs not handed over: too many already in flight */
static volatile uint32_t g_send_backoffs;     /* SDU pairs not attempted: backing off after repeated send errors */
static volatile uint32_t g_tick_skips;        /* encode ticks dropped because the encoder fell behind */

#define STREAM_COUNT            CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT
#define SUBGROUP_COUNT          CONFIG_BT_BAP_BROADCAST_SRC_SUBGROUP_COUNT

/* ---- ISO TX queue policy ----------------------------------------------------
 * The controller holds CONFIG_BT_LE_ISO_NSFC_NUM (12) SDUs for all BIS
 * together (non-standard flow control). When it is full, the LE Audio host
 * (esp_ble_iso common/iso.c, iso_tx_sdu_insert) does not refuse the SDU: it
 * mallocs it into a list with NO length limit and drains it on TX-complete.
 * If the BIG stalls (controller starved, radio taken by BR/EDR) that list
 * grows by 2 x 128 B of internal heap every 10 ms, and every queued SDU is
 * stale audio anyway. So we count our own SDUs in flight per BIS (handed over
 * minus completed/flushed in the sent callback) and drop a whole L+R frame
 * (seq_num still advances, receivers run PLC) once ISO_TX_INFLIGHT_MAX are
 * outstanding: 8 x 10 ms = 80 ms, above the normal 2-4 of the BN 2 / 20 ms
 * BIG and above the 65 ms max transport latency, and 2 x 8 > 12 so the host
 * list is already in use when it trips. If a completion is ever lost
 * (iso_tx_comp_cb drops it when its calloc fails) the count would stick high:
 * after ISO_TX_INFLIGHT_MAX dropped frames with completions still arriving,
 * the estimate is clamped back to half the cap. */
#define ISO_TX_INFLIGHT_MAX     8
/* ISO-clocked pacing: the next SDU is encoded when the controller reports one
 * sent, so the queue stays at a constant depth locked to the BIG's own clock
 * (no CPU-timer drift, no double sends, no skipped ticks). ISO_TX_PREROLL SDUs
 * are queued at stream start; if completions stop for ISO_TX_WDOG_MS the
 * encoder falls back to its own 10 ms pace so audio never freezes. */
#define ISO_CLOCKED_PACING      1
#define ISO_TX_PREROLL          2
#define ISO_TX_WDOG_MS          30
static volatile bool g_iso_paced;              /* stream 0 started, completion pacing active */
static volatile uint32_t g_iso_wdog_ticks;     /* encodes forced by the watchdog (no completions) */
/* After this many consecutive refused sends, skip ISO_TX_BACKOFF_TICKS ticks
 * before trying again: the host logs every refusal (ChanNotConn, IsoTxNoMem)
 * on the UART, 200 lines/s would be a log storm on core 0. */
#define ISO_TX_FAIL_BURST       10
#define ISO_TX_BACKOFF_TICKS    20
/* Encoder catch-up: at most this many ticks may be waiting behind the one
 * being encoded (30 ms). Older ones are dropped (PCM consumed, seq_num
 * advanced) instead of bursting 16 late SDUs into the queue above. */
#define ENC_MAX_BACKLOG         3

static struct broadcast_source_stream {
    esp_ble_audio_bap_stream_t stream;
    uint16_t seq_num;
    uint8_t *data;
    example_audio_tx_scheduler_t scheduler;
    uint32_t sent;                      /* SDUs handed to the host (lc3_enc task) */
    volatile uint32_t done;             /* completed + not-transmitted (iso_task, sent cb) */
} streams[STREAM_COUNT];

static uint32_t g_tx_fail_run;          /* consecutive refused sends (lc3_enc task only) */
static uint32_t g_tx_backoff;           /* ticks left to skip */
static uint32_t g_tx_blocked;           /* consecutive frames dropped by the in-flight cap */
static uint32_t g_tx_blocked_done;      /* stream 0 'done' when that run started */

/* lc3_enc task only. If completions overtake our count (SDUs from before a
 * stream restart; the 2 s stats show on-air + not-sent a few counts above
 * frames sent), re-base 'sent' so the estimate never goes negative and the
 * cap does not silently drift open. */
static uint32_t iso_inflight(struct broadcast_source_stream *s)
{
    uint32_t d = __atomic_load_n(&s->done, __ATOMIC_ACQUIRE);
    int32_t n = (int32_t)(s->sent - d);
    if (n < 0) {
        s->sent = d;
        n = 0;
    }
    return (uint32_t)n;
}

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
    source_stream->sent = 0;
    __atomic_store_n(&source_stream->done, 0, __ATOMIC_RELEASE);
    g_tx_fail_run = g_tx_backoff = g_tx_blocked = 0;
    /* Stream 0 paces BOTH channels. */
    if (idx == 0) {
#if ISO_CLOCKED_PACING
        (void)err;
        xQueueReset(g_encode_sem);                    /* drop stale ticks from a previous run */
        g_iso_paced = true;
        for (int i = 0; i < ISO_TX_PREROLL; i++) {
            xSemaphoreGive(g_encode_sem);
        }
        ESP_LOGW(TAG, "[SRC #0] ISO-clocked TX pacing: pre-roll %d SDUs, then one encode per completed SDU",
                 ISO_TX_PREROLL);
#else
        example_audio_tx_scheduler_reset(&source_stream->scheduler);
        err = example_audio_tx_scheduler_start(&source_stream->scheduler,
                                               preset_active.qos.interval);
        if (err) {
            ESP_LOGE(TAG, "[SRC #%d] Scheduler start failed, err %d", idx, err);
        }
#endif
    }
}

static void stream_stopped_cb(esp_ble_audio_bap_stream_t *stream, uint8_t reason)
{
    struct broadcast_source_stream *source_stream = CONTAINER_OF(stream,
                                                                 struct broadcast_source_stream,
                                                                 stream);
    ESP_LOGI(TAG, "[SRC #%d] Stream stopped, reason 0x%02x", stream_index(stream), reason);
    if (stream_index(stream) == 0) {
        g_iso_paced = false;
        xQueueReset(g_encode_sem);
    }
    example_audio_tx_scheduler_stop(&source_stream->scheduler);
}

static void stream_disconnected_cb(esp_ble_audio_bap_stream_t *stream, uint8_t reason)
{
    struct broadcast_source_stream *source_stream = CONTAINER_OF(stream,
                                                                 struct broadcast_source_stream,
                                                                 stream);
    ESP_LOGI(TAG, "[SRC #%d] ISO disconnected, reason 0x%02x", stream_index(stream), reason);
    if (stream_index(stream) == 0) {
        g_iso_paced = false;
        xQueueReset(g_encode_sem);
    }
    example_audio_tx_scheduler_stop(&source_stream->scheduler);
}

static void stream_sent_cb(esp_ble_audio_bap_stream_t *stream, void *user_data)
{
    struct broadcast_source_stream *source_stream = CONTAINER_OF(stream,
                                                                 struct broadcast_source_stream,
                                                                 stream);
    char name[24];

    const esp_ble_iso_tx_cb_info_t *info = user_data;
    if (info != NULL) {
        __atomic_fetch_add(&source_stream->done, (uint32_t)info->completed_pkts + info->not_txd_pkts,
                           __ATOMIC_RELEASE);
    }
#if ISO_CLOCKED_PACING
    if (info != NULL && stream_index(stream) == 0 && g_iso_paced) {
        uint32_t c = (uint32_t)info->completed_pkts + info->not_txd_pkts;
        while (c--) {
            xSemaphoreGive(g_encode_sem);             /* one SDU left the queue -> make the next */
        }
    }
#endif
    if (info != NULL && stream_index(stream) == 0) {
        int64_t now_us = esp_timer_get_time();
        if (g_cb_prev_us) {
            uint32_t gap = (uint32_t)(now_us - g_cb_prev_us);
            if (gap > g_cb_gap_max) g_cb_gap_max = gap;
            if (gap > 30000) g_cb_gaps_30++;
        }
        g_cb_prev_us = now_us;
        for (size_t i = 0; i < info->pkt_cnt && i < 2; i++) {
            uint32_t ref = info->pkt[i].sdu_ref_time, tx = info->pkt[i].tx_time_stamp;
            if (g_ref_prev) {
                uint32_t step = ref - g_ref_prev, tstep = tx - g_tx_prev;
                if (step < 5000000u) {
                    if (step > g_ref_step_max) g_ref_step_max = step;
                    g_ref_hist[step < 10000 ? 0 : step < 30000 ? 1 : step < 50000 ? 2 : 3]++;
                    if (step >= 30000) g_ref_jumps++;
                }
                if (tstep < 5000000u && tstep > g_tx_step_max) g_tx_step_max = tstep;
            }
            g_ref_prev = ref;
            g_tx_prev = tx;
        }
        g_iso_done += info->completed_pkts;
        g_iso_not_txd += info->not_txd_pkts;
        for (size_t i = 0; i < info->pkt_cnt && i < 2; i++) {
            if (info->pkt[i].drift) {
                g_iso_drift++;
            }
            uint32_t q = info->pkt[i].tx_time_stamp - info->pkt[i].sdu_rec_time;
            uint32_t r = info->pkt[i].tx_time_stamp - info->pkt[i].sdu_ref_time;
            if (q < 5000000u) { g_q_sum += q; g_q_n++; if (q > g_q_max) g_q_max = q; }
            if (r < 5000000u) { g_ref_sum += r; if (r > g_ref_max) g_ref_max = r; }
            g_q_raw[0] = info->pkt[i].tx_time_stamp; g_q_raw[1] = info->pkt[i].sdu_rec_time;
            g_q_raw[2] = info->pkt[i].sdu_ref_time;  g_q_raw[3] = info->pkt[i].sdu_hdl_time;
        }
    }
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
    {
        static int64_t last_us;
        int64_t now_us = esp_timer_get_time();
        if (last_us != 0) {
            uint32_t gap = (uint32_t)(now_us - last_us);
            if (gap > g_max_gap_us) g_max_gap_us = gap;
            if (gap > 15000) g_late_ticks++;
        }
        last_us = now_us;
    }

    int fs = g_frame_samples;
    if (fs > 480) {
        fs = 480;
    }
    /* Phone music over A2DP when it streams; otherwise silence (set
     * TEST_TONE 1 to get the 440/660 Hz beeps back for testing). */
    if (SWEEP_TEST && !a2dp_bridge_streaming()) {
        a2dp_bridge_test_sweep(pcm, fs);       /* phone paused: S31 sweep A/B test */
    } else if (a2dp_bridge_read(pcm, fs)) {
        /* Glitch detector on the PCM we are about to encode: a jump between
         * frames far larger than the frame's own sample-to-sample motion means
         * the audio was damaged before Auracast (A2DP/resampler/buffer). */
        static int16_t prev_last[2];
        for (int ch = 0; ch < 2; ch++) {
            uint32_t mad = 0;
            for (int n = 1; n < fs; n++) {
                int d = pcm[2 * n + ch] - pcm[2 * (n - 1) + ch];
                mad += d < 0 ? -d : d;
                if (pcm[2 * n + ch] >= 32767 || pcm[2 * n + ch] <= -32767) {
                    g_clipped++;
                }
            }
            mad = (uint32_t)((float)mad * (1.0f / 479.0f));   /* fs is always 480: no divide */
            int jump = pcm[ch] - prev_last[ch];
            if (jump < 0) jump = -jump;
            if (jump > 2000 && (uint32_t)jump > 6 * mad + 500) {
                g_glitches++;
            }
            prev_last[ch] = pcm[2 * (fs - 1) + ch];
        }
    } else {
        if (TEST_TONE) {
            tone_fill(pcm, fs);
        } else {
            memset(pcm, 0, sizeof(int16_t) * 2 * fs);
        }
    }

#if REPLAY_TEST
    {
        static int16_t *rec;               /* PSRAM ring of 10 ms stereo frames */
        static uint32_t rec_frames, rec_w, rec_r, rec_n;
        bool replaying = g_replaying;
        if (rec == NULL && rec_frames == 0) {
            rec_frames = REPLAY_SECONDS * 100;
            rec = heap_caps_malloc((size_t)rec_frames * 480 * 2 * sizeof(int16_t), MALLOC_CAP_SPIRAM);
            if (rec == NULL) {
                ESP_LOGE(TAG, "REPLAY: no PSRAM for the test buffer");
            }
        }
        if (rec != NULL && fs == 480) {
            if (a2dp_bridge_streaming()) {
                if (replaying) {
                    g_replaying = false;
                    ESP_LOGW(TAG, "REPLAY: phone streaming again -> live audio");
                }
                memcpy(rec + (size_t)rec_w * 960, pcm, 960 * sizeof(int16_t));
                if (++rec_w >= rec_frames) rec_w = 0;
                if (rec_n < rec_frames) rec_n++;
            } else if (rec_n >= 300) {     /* >= 3 s recorded: loop it */
                if (!replaying) {
                    g_replaying = true;
                    rec_r = rec_w + rec_frames - rec_n; if (rec_r >= rec_frames) rec_r -= rec_frames;
                    ESP_LOGW(TAG, "REPLAY: phone paused -> looping the last %lu s of its music over Auracast "
                             "(Classic link idle). Listen: clean now = coexistence problem.",
                             (unsigned long)(rec_n / 100));
                }
                memcpy(pcm, rec + (size_t)rec_r * 960, 960 * sizeof(int16_t));
                if (++rec_r >= rec_frames) rec_r = 0;
                if (rec_r == rec_w) {
                    rec_r = rec_w + rec_frames - rec_n; if (rec_r >= rec_frames) rec_r -= rec_frames;
                }
            }
        }
    }
#endif

    /* ISO queue policy: decide before spending CPU on the encode. */
    bool send = true;
    if (g_tx_backoff > 0) {
        g_tx_backoff--;
        g_send_backoffs++;
        send = false;
    } else {
        uint32_t worst = 0;
        for (int ch = 0; ch < 2 && ch < (int)ARRAY_SIZE(streams); ch++) {
            uint32_t n = iso_inflight(&streams[ch]);
            if (n > worst) worst = n;
        }
        if (worst >= ISO_TX_INFLIGHT_MAX) {
            uint32_t d0 = __atomic_load_n(&streams[0].done, __ATOMIC_ACQUIRE);
            if (g_tx_blocked == 0) {
                g_tx_blocked_done = d0;
            }
            if (++g_tx_blocked >= ISO_TX_INFLIGHT_MAX && d0 != g_tx_blocked_done) {
                /* the BIG is moving but our count stays high: a completion
                 * was lost; clamp the estimate instead of staying muted */
                for (int ch = 0; ch < 2 && ch < (int)ARRAY_SIZE(streams); ch++) {
                    streams[ch].sent = __atomic_load_n(&streams[ch].done, __ATOMIC_ACQUIRE) +
                                       ISO_TX_INFLIGHT_MAX / 2;
                }
                g_tx_blocked = 0;
            }
            g_iso_backlog_drops++;
            send = false;
        } else {
            g_tx_blocked = 0;
        }
    }

    /* JBL One EQ (7 JBL biquads), applied before encoding like the real speaker.
     * Runs even for a dropped frame so its filter state stays continuous. */
    v5_eq_process(pcm, fs);

    int64_t enc_t0 = esp_timer_get_time();
    for (int ch = 0; send && ch < 2 && ch < (int)ARRAY_SIZE(streams); ch++) {
        struct broadcast_source_stream *s = &streams[ch];
        if (g_enc[ch] == NULL || s->data == NULL ||
            s->stream.qos == NULL || s->stream.qos->sdu == 0) {
            continue;
        }
        int opf = s->stream.qos->sdu;
        if (lc3_encode(g_enc[ch], LC3_PCM_FORMAT_S16, pcm + ch, 2, opf, s->data) != 0) {
            continue;
        }
#if LC3_LOOPBACK
        if (ch == 0 && fs == 480) {
            if (g_lb_dec == NULL) {
                void *m = heap_caps_malloc(lc3_decoder_size(TONE_DT_US, TONE_SR_HZ), MALLOC_CAP_SPIRAM);
                g_lb_dec = m ? lc3_setup_decoder(TONE_DT_US, TONE_SR_HZ, TONE_SR_HZ, m) : NULL;
                g_lb_delay = lc3_delay_samples(TONE_DT_US, TONE_SR_HZ);
                if (g_lb_delay < 0 || g_lb_delay > 512) g_lb_delay = 0;
                ESP_LOGW(TAG, "LC3 loopback check on (decoder delay %d samples)", g_lb_delay);
            }
            if (g_lb_dec != NULL) {
                static int16_t y[480];
                /* history: [delay older samples][this frame] */
                for (int n = 0; n < 480; n++) g_lb_hist[g_lb_delay + n] = pcm[2 * n];
                int r = lc3_decode(g_lb_dec, s->data, opf, LC3_PCM_FORMAT_S16, y, 1);
                if (r != 0) g_lb_dec_err++;
                float fsig = 0.0f, ferr = 0.0f;
                for (int n = 0; n < 480; n++) {
                    float x = (float)g_lb_hist[n], d = (float)y[n] - x;
                    fsig += x * x;
                    ferr += d * d;
                }
                memmove(g_lb_hist, g_lb_hist + 480, (size_t)g_lb_delay * sizeof(int16_t));
                if (fsig > 480.0f * 100.0f * 100.0f) {          /* skip near-silence */
                    float snr = 10.0f * log10f(fsig / (ferr + 1.0f));
                    g_lb_sig += fsig;
                    g_lb_err += ferr;
                    g_lb_frames++;
                    if (snr < g_lb_worst) g_lb_worst = snr;
                    if (snr < 10.0f) g_lb_bad++;
                }
            }
        }
#endif
        /* One attempt per SDU, never retried: a late SDU is worthless, and the
         * host copies s->data, so the buffer is free again on return. */
        if (esp_ble_audio_bap_stream_send(&s->stream, s->data, opf, streams[0].seq_num) != 0) {
            g_send_errors++;
            if (++g_tx_fail_run >= ISO_TX_FAIL_BURST) {
                g_tx_fail_run = 0;
                g_tx_backoff = ISO_TX_BACKOFF_TICKS;
            }
        } else {
            s->sent++;
            g_tx_fail_run = 0;
        }
    }
    {
        uint32_t us = (uint32_t)(esp_timer_get_time() - enc_t0);   /* encode + hand-off, 2 channels */
        g_enc_us_sum += us;
        g_enc_n++;
        if (us > g_enc_us_max) g_enc_us_max = us;
    }
    streams[0].seq_num++;
    g_frames_sent++;
}

/* A tick the encoder is too late for: keep the PCM ring and the sequence
 * number in step with the BIG clock, but encode and send nothing. */
static void broadcast_source_skip_tick(void)
{
    if (!stream_is_streaming(&streams[0].stream)) {
        return;
    }
    a2dp_bridge_discard(g_frame_samples);
    streams[0].seq_num++;
    g_tick_skips++;
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
    /* BIG layout copied from the real Xtreme 5's BIGInfo (read 2026-11-21):
     *   2 BIS, ISO interval 16 (20 ms), BN 2, NSE 8, IRC 4, PTO 0,
     *   max PDU 100, SDU interval 10 ms, 2M PHY, unframed.
     * One radio burst every 20 ms carrying two frames, each sent 4 times,
     * leaves a long gap each cycle for the phone's A2DP link.
     * Applied through LE Create BIG Test (CONFIG_BT_ISO_TEST_PARAMS). */
    /* IRC 4 -> 2 (NSE 8 -> 4): same 20 ms interval, BN 2, 100 B PDUs, 2M PHY,
     * but each frame goes out twice instead of 4 times. Halves the BIG's
     * airtime (~9.6 -> ~4.8 ms per 20 ms) so the phone's A2DP link has room
     * for SBC bitpool 53. Set BIG_IRC 4 to go back to the Xtreme 5's numbers. */
#define BIG_IRC 4   /* Xtreme 5 layout (4 copies); A2DP fits the gaps via MTU 675 = one 2-DH5 per media packet */
    preset_active.qos.rtn = 2;
    preset_active.qos.num_subevents = 2 * BIG_IRC;   /* NSE = BN x IRC */
    preset_active.qos.burst_number = 2;
    preset_active.qos.max_pdu = 100;
    create_param.irc = BIG_IRC;
    create_param.pto = 0;
    create_param.iso_interval = 16;

    create_param.params_count = ARRAY_SIZE(subgroup_param);
    create_param.params = subgroup_param;
    create_param.qos = &preset_active.qos;
    create_param.encryption = (strlen(LOCAL_BROADCAST_CODE) > 0);
    /* Interleaved: L0 R0 L1 R1 | L2 R2 ... -> both channels' first copies (prio 9)
     * go out back-to-back at the start of each BIG event; the A2DP link (prio 8)
     * can only take later repetitions (prio 7), never a first copy. With
     * sequential packing a 5-slot EDR packet won over L's repetitions and ran
     * into R's first copies (26% of SDUs never sent). */
    create_param.packing = ESP_BLE_ISO_PACKING_INTERLEAVED;
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

/* Captured from the real Xtreme 5 while hosting (its ext adv ends with this):
 * Manufacturer Specific Data, company 0x0057 (Harman), 16 x 00, then df fd. */
static const uint8_t jbl_master_mfg_ad[] = {
    0x15, 0xff, 0x57, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xdf, 0xfd,
};

static uint8_t *ext_adv_data_get(uint8_t *data_len)
{
    const uint32_t broadcast_id = LOCAL_BROADCAST_ID;
    uint8_t *data;
    uint8_t i = 0;

    *data_len = 7 + 6 + 4 + (2 + LOCAL_BROADCAST_NAME_LEN) +
                (2 + LOCAL_BROADCAST_NAME_LEN) + sizeof(jbl_master_mfg_ad) +
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

    /* Complete Local Name (same text), as the real JBL master sends */
    data[i++] = LOCAL_BROADCAST_NAME_LEN + 1;
    data[i++] = EXAMPLE_AD_TYPE_NAME_COMPLETE;
    memcpy(data + i, LOCAL_BROADCAST_NAME, LOCAL_BROADCAST_NAME_LEN);
    i += LOCAL_BROADCAST_NAME_LEN;

    /* Harman (0x0057) manufacturer block - the JBL master marker */
    memcpy(data + i, jbl_master_mfg_ad, sizeof(jbl_master_mfg_ad));
    i += sizeof(jbl_master_mfg_ad);

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

/* Keep the JBL party block alive, like a real host; also the one place that
 * touches the running beacon (payload when the phone name arrives, and the
 * interval: 20 ms like the Xtreme 5 when idle, 100 ms while phone audio
 * streams so the BIG and the A2DP link get the airtime). */
#define BEACON_STREAMING_MS 100
static void pa_update_task(void *arg)
{
    bool slow = false;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (s_beacon_dirty) {
            uint8_t adv[4 + sizeof(jbl_beacon_payload)];
            uint8_t n = jbl_beacon_ad(adv);
            s_beacon_dirty = false;
            beacon_adv_data_update(adv, n);
        }
        bool want_slow = a2dp_bridge_streaming() || g_replaying;
        if (want_slow != slow) {
            slow = want_slow;
            beacon_set_interval(slow ? BEACON_STREAMING_MS : BEACON_INTERVAL_MS);
        }
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

/* reassembly of long (prepared) writes: cold, so PSRAM, allocated on the
 * first prepared write */
#define V5_PREP_MAX 600
static uint8_t *s_v5_prep;
static uint16_t s_v5_prep_len;

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
        v5_server_on_connect(gatts_if, p->connect.conn_id);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGW(TAG, ">>> GATT DISCONNECT (conn_id %u, reason 0x%02x)",
                 p->disconnect.conn_id, p->disconnect.reason);
        v5_server_on_disconnect(p->disconnect.conn_id);
        beacon_adv_restart();
        break;
    case ESP_GATTS_CONGEST_EVT:
        v5_server_set_congested(p->congest.conn_id, p->congest.congested);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGW(TAG, "peer MTU %u (conn %u)", p->mtu.mtu, p->mtu.conn_id);
        v5_server_set_mtu(p->mtu.mtu);
        break;
    case ESP_GATTS_EXEC_WRITE_EVT:
        if (p->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && s_v5_prep_len) {
            v5_server_on_write(s_v5_prep, s_v5_prep_len);
        }
        s_v5_prep_len = 0;
        break;
    case ESP_GATTS_READ_EVT:
        ESP_LOGW(TAG, ">>> peer READ handle 0x%04x", p->read.handle);
        break;
    case ESP_GATTS_WRITE_EVT: {
        if (jbl_gatt_is_v5_write(p->write.handle)) {
            /* JBL One command channel (long writes arrive as prepared writes) */
            if (p->write.is_prep) {
                if (s_v5_prep == NULL) {
                    s_v5_prep = heap_caps_malloc(V5_PREP_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                    if (s_v5_prep == NULL) {
                        s_v5_prep = heap_caps_malloc(V5_PREP_MAX, MALLOC_CAP_8BIT);
                    }
                }
                if (s_v5_prep != NULL && p->write.offset + p->write.len <= V5_PREP_MAX) {
                    memcpy(s_v5_prep + p->write.offset, p->write.value, p->write.len);
                    s_v5_prep_len = p->write.offset + p->write.len;
                }
            } else {
                v5_server_on_write(p->write.value, p->write.len);
            }
            break;
        }
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

    uint8_t adv_len = jbl_beacon_ad(adv);
    uint16_t crc = jbl_beacon_build();
    const uint8_t *mac = esp_bt_dev_get_address();
    ESP_LOGW(TAG, "JBL identity: BT MAC %02x:%02x:%02x:%02x:%02x:%02x -> macAddressCrc 0x%04x "
             "(the user's Xtreme 5 is 0x10cd)", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], crc);

    rsp[0] = sizeof(JBL_BEACON_NAME);                   /* len incl. type byte */
    rsp[1] = EXAMPLE_AD_TYPE_NAME_COMPLETE;
    memcpy(rsp + 2, JBL_BEACON_NAME, sizeof(JBL_BEACON_NAME) - 1);

    char h[3 * sizeof(jbl_beacon_payload) + 1] = {0};
    for (size_t k = 0; k < jbl_beacon_len; k++) {
        sprintf(h + 3 * k, "%02x ", jbl_beacon_payload[k]);
    }
    ESP_LOGW(TAG, "JBL beacon 0xFDDF payload: %s", h);

    return beacon_adv_start(adv, adv_len, rsp, sizeof(rsp), jbl_beacon_addr);
}

static esp_err_t tone_lc3_init(void)
{
    g_frame_samples = lc3_frame_samples(TONE_DT_US, TONE_SR_HZ);   /* 480 @ 48k/10ms */
    for (int ch = 0; ch < 2; ch++) {
        /* hot (touched every 10 ms by the encoder): pin it to internal RAM
         * explicitly instead of relying on the 16 KB malloc threshold */
        g_enc_mem[ch] = heap_caps_malloc(lc3_encoder_size(TONE_DT_US, TONE_SR_HZ),
                                         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
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
/* 2 s stats, printed from its own low-priority task so UART logging never
 * delays an LC3 encode tick (the encoder must hand an SDU over every 10 ms). */
static void stats_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    uint32_t   last_sent = 0;

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(2000));

        uint32_t now = g_frames_sent;
        uint32_t ur, dr, sk, fill;
        a2dp_bridge_stats(&ur, &dr, &sk, &fill);
        ESP_LOGW(TAG, "TX 2s: %lu frames (expect 200), send_err %lu | %s, a2dp pkts %lu, "
                 "buf %lu ms, underruns %lu, drops %lu, latency skips %lu, PCM glitches %lu, clipped %lu",
                 (unsigned long)(now - last_sent), (unsigned long)g_send_errors,
                 a2dp_bridge_streaming() ? "PHONE MUSIC" :
                 a2dp_bridge_connected() ? "phone paused (silence)" :
                 TEST_TONE ? "test tone" : "no phone (silence)",
                 (unsigned long)a2dp_bridge_packets(), (unsigned long)fill,
                 (unsigned long)ur, (unsigned long)dr, (unsigned long)sk,
                 (unsigned long)g_glitches, (unsigned long)g_clipped);
        ESP_LOGW(TAG, "ISO 2s: on-air %lu, NOT SENT %lu, drift %lu | encode ticks late %lu, worst gap %lu us | "
                 "in flight %ld/%d, totals: backlog drops %lu, backoff %lu, ticks skipped %lu, watchdog encodes %lu",
                 (unsigned long)g_iso_done, (unsigned long)g_iso_not_txd, (unsigned long)g_iso_drift,
                 (unsigned long)g_late_ticks, (unsigned long)g_max_gap_us,
                 (long)(int32_t)(streams[0].sent - streams[0].done), ISO_TX_INFLIGHT_MAX,
                 (unsigned long)g_iso_backlog_drops, (unsigned long)g_send_backoffs,
                 (unsigned long)g_tick_skips, (unsigned long)g_iso_wdog_ticks);
        if (a2dp_bridge_streaming()) {
            ESP_LOGW(TAG, "SBC 2s: clock corr %ld ppm | bitpool now %u (min %u, max %u), media pkts %lu, lost pkts %lu",
                     (long)a2dp_bridge_clock_ppm(),
                     btc_a2dp_sink_bitpool_last,
                     btc_a2dp_sink_bitpool_min == 0xFF ? 0 : btc_a2dp_sink_bitpool_min,
                     btc_a2dp_sink_bitpool_max, (unsigned long)btc_a2dp_sink_media_pkts,
                     (unsigned long)btc_a2dp_sink_media_seq_gaps);
        }
        btc_a2dp_sink_bitpool_min = 0xFF;
        btc_a2dp_sink_bitpool_max = 0;
        btc_a2dp_sink_media_pkts = 0;
        btc_a2dp_sink_media_seq_gaps = 0;
#if LC3_LOOPBACK
        if (g_lb_frames || g_lb_dec_err) {
            int avg10 = (int)(100.0f * log10f(g_lb_sig / (g_lb_err + 1.0f)));
            int worst10 = (int)(10.0f * g_lb_worst);
            ESP_LOGW(TAG, "LC3 loopback 2s: SNR avg %d.%d dB, worst frame %d.%d dB, frames <10 dB: %lu/%lu, decode errors %lu",
                     avg10 / 10, abs(avg10 % 10), worst10 / 10, abs(worst10 % 10),
                     (unsigned long)g_lb_bad, (unsigned long)g_lb_frames, (unsigned long)g_lb_dec_err);
        }
        g_lb_frames = g_lb_dec_err = g_lb_bad = 0;
        g_lb_sig = g_lb_err = 0.0f;
        g_lb_worst = 999.0f;
#endif
        {
            uint32_t n = g_q_n;
            float inv = n ? 1.0f / (float)n : 0.0f;
            ESP_LOGW(TAG, "ISO timing 2s: SDU wait in controller avg %lu us max %lu us | tx-ref avg %lu us max %lu us (n %lu) raw tx %lu rec %lu ref %lu hdl %lu",
                     (unsigned long)((float)g_q_sum * inv), (unsigned long)g_q_max,
                     (unsigned long)((float)g_ref_sum * inv), (unsigned long)g_ref_max, (unsigned long)n,
                     (unsigned long)g_q_raw[0], (unsigned long)g_q_raw[1], (unsigned long)g_q_raw[2], (unsigned long)g_q_raw[3]);
            g_q_sum = g_q_max = g_q_n = g_ref_sum = g_ref_max = 0;
        }
        ESP_LOGW(TAG, "RADIO 2s: BIG anchor steps 0ms %lu / 20ms %lu / 40ms %lu / >40ms %lu, missed-event jumps %lu, "
                 "worst anchor step %lu us, worst tx step %lu us | completion cb worst gap %lu us, gaps>30ms %lu",
                 (unsigned long)g_ref_hist[0], (unsigned long)g_ref_hist[1], (unsigned long)g_ref_hist[2],
                 (unsigned long)g_ref_hist[3], (unsigned long)g_ref_jumps, (unsigned long)g_ref_step_max,
                 (unsigned long)g_tx_step_max, (unsigned long)g_cb_gap_max, (unsigned long)g_cb_gaps_30);
        g_ref_hist[0] = g_ref_hist[1] = g_ref_hist[2] = g_ref_hist[3] = 0;
        g_ref_jumps = g_ref_step_max = g_tx_step_max = g_cb_gap_max = g_cb_gaps_30 = 0;
        ESP_LOGW(TAG, "ENC 2s: %lu ch-frames, bandwidth below full %lu, bandwidth changes %lu, TNS on %lu, attack %lu",
                 (unsigned long)lc3_enc_stat[0], (unsigned long)lc3_enc_stat[1], (unsigned long)lc3_enc_stat[2],
                 (unsigned long)lc3_enc_stat[3], (unsigned long)lc3_enc_stat[4]);
        memset((void *)lc3_enc_stat, 0, sizeof(lc3_enc_stat));
        {
            /* 200 encodes per 2 s: avg = sum / n via the reciprocal of n */
            uint32_t n = g_enc_n, avg = n ? (uint32_t)((float)g_enc_us_sum * (1.0f / (float)n)) : 0;
            ESP_LOGW(TAG, "LC3 2s: encode L+R avg %lu us, max %lu us per 10 ms frame (~%lu%% of one core)",
                     (unsigned long)avg, (unsigned long)g_enc_us_max, (unsigned long)((avg * 41) >> 12));
            g_enc_us_sum = g_enc_us_max = g_enc_n = 0;
        }
        g_iso_done = g_iso_not_txd = g_iso_drift = g_late_ticks = g_max_gap_us = 0;
        last_sent = now;
    }
}

static void encode_task(void *arg)
{
    for (;;) {
#if ISO_CLOCKED_PACING
        if (g_iso_paced && xSemaphoreTake(g_encode_sem, pdMS_TO_TICKS(ISO_TX_WDOG_MS)) != pdTRUE) {
            if (g_iso_paced) {                        /* no completions: keep audio flowing */
                g_iso_wdog_ticks++;
                broadcast_source_tx();
            }
            continue;
        }
        if (!g_iso_paced && xSemaphoreTake(g_encode_sem, pdMS_TO_TICKS(500)) != pdTRUE) {
            continue;
        }
        {
#else
        if (xSemaphoreTake(g_encode_sem, pdMS_TO_TICKS(500)) == pdTRUE) {
#endif
            /* Late by more than ENC_MAX_BACKLOG ticks (core 0 overloaded):
             * drop the oldest ticks instead of bursting them all out late.
             * Sheds load exactly when there is none to spare, and keeps the
             * ISO queue short. */
            UBaseType_t behind = uxSemaphoreGetCount(g_encode_sem);
            while (behind > ENC_MAX_BACKLOG && xSemaphoreTake(g_encode_sem, 0) == pdTRUE) {
                broadcast_source_skip_tick();
                behind--;
            }
            broadcast_source_tx();
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

    err = bt_dual_mode_init();             /* BLE (Auracast) + BR/EDR (phones) */
    if (err) {
        ESP_LOGE(TAG, "Failed to initialize Bluetooth, err %d", err);
        return;
    }
    a2dp_bridge_on_phone_name(on_phone_name);
    v5_server_start(JBL_BREDR_NAME);
    err = a2dp_bridge_start(JBL_BREDR_NAME);
    if (err) {
        ESP_LOGE(TAG, "Failed to start A2DP sink, err %d", err);
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
    /* Core 0 with the Bluedroid host + iso_task (CONFIG_BT_BLUEDROID_PINNED_TO_CORE_0):
     * core 1 is left to A2DP_DECODER (SBC + resampler, hard-pinned to core 1 in
     * btc_a2dp_sink.c). Prio 21 = below btdm (23), esp_timer/hciT (22), above
     * BTU/iso_task (20), so the controller is never delayed and the iso_task
     * k_work tick that gives g_encode_sem is followed at once by the encode. */
    xTaskCreatePinnedToCore(encode_task, "lc3_enc", 12288, NULL,
                            configMAX_PRIORITIES - 4, NULL, 0);
    /* Cold tasks (log / adv-data housekeeping, never touch flash) get their
     * stacks in PSRAM: frees internal RAM for Bluetooth. A PSRAM stack must
     * not be used while the cache is off, i.e. no NVS/flash calls in them. */
    app_task_cold("tx_stats", stats_task, 4096, 2);

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
        app_task_cold("jbl_pa", pa_update_task, 4096, 5);   /* core 1: keep core 0 for the BT controller */
    }
    ESP_LOGW(TAG, "JBL party block in periodic data: %s", JBL_PA_BLOCK ? "ON" : "OFF");

    /* No app needed: the S31 itself tells a nearby Xtreme 5 to join us. */
    jbl_assistant_start(jbl_beacon_build(), LOCAL_BROADCAST_NAME);
}
