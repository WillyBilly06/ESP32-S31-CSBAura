/*
 * A2DP sink on BTstack.
 *
 * BT thread (core 1):  AVDTP media packet -> copy into s_pkt_q (no decoding)
 * DSP task  (core 0):  SBC decode (Bluedroid SBC, bundled with BTstack)
 *                      -> 32-tap windowed-sinc resampler to 48 kHz with smoothed
 *                         PI clock recovery on the PCM ring fill
 *                      -> lock-free SPSC PCM ring in PSRAM (read by the encoder)
 *
 * Link policy as on the Bluedroid build: the S31 is BR/EDR master (accepts the
 * phone as master, never switches), no sniff, AVDTP MTU 1005 (one 3-DH5 per
 * media packet), SBC bitpool pinned at 53.
 */
#include "a2dp_in.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "btstack.h"
#include "classic/btstack_sbc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"

#include "a2dp_dsp.h"

static const char *TAG = "A2DP";

#define OUT_RATE        48000
#define PKT_MAX         (AVDTP_L2CAP_MTU + 16)
#define PKT_Q_LEN       24                    /* ~550 ms of 8-frame packets */
#define RING_MS         160                   /* max S31-added delay: beyond it the OLDEST audio is dropped */
#define TARGET_MS       80                    /* PCM fill the clock recovery aims at */
#define PRIME_MS        50                    /* start output once this much is buffered */
#define MS_FRAMES(ms)   ((OUT_RATE / 1000) * (ms))

/* ---- AVDTP endpoint / SDP --------------------------------------------------- */
/* All SBC configurations, bitpool pinned at 53 (min = max): the phone cannot
 * fall back to its adaptive low bitpools. */
static uint8_t s_sbc_caps[] = { 0xFF, 0xFF, 53, 53 };
static uint8_t s_sbc_cfg[4];
static uint8_t s_sdp_a2dp[150];
static uint8_t s_sdp_avrcp_ct[200];
static uint8_t s_sdp_avrcp_tg[200];
static uint8_t s_sdp_did[100];
static uint8_t s_local_seid;
static uint16_t s_a2dp_cid;
static bd_addr_t s_peer;
static volatile bool s_connected, s_streaming;
static btstack_packet_callback_registration_t s_hci_reg;

/* ---- media hand-off BT -> DSP ---------------------------------------------- */
typedef struct {
    uint16_t len;
    uint8_t data[PKT_MAX];
} pkt_t;
static QueueHandle_t s_pkt_q;
static volatile uint32_t s_pkts, s_pkt_drops, s_seq_gaps, s_bitpool, s_bp_min = 0xff, s_bp_max;
static volatile uint32_t s_cfg_rate = 44100;
static volatile bool s_reset_dsp;

/* ---- DSP state (core 0) ------------------------------------------------------ */
static btstack_sbc_decoder_bluedroid_t s_sbc_ctx;
static const btstack_sbc_decoder_t *s_sbc;
static rs_state_t s_rs;
static spsc_ring_t s_ring;
static float s_fill_avg, s_integ;
static volatile int32_t s_corr_ppm;
static volatile uint32_t s_underruns, s_ring_drops;
static volatile bool s_primed;
static uint32_t s_out_tmp[RS_OUT_MAX];

static void on_pcm(int16_t *data, int num_frames, int num_channels, int sample_rate, void *context)
{
    (void)context;
    if (num_channels != 2) {
        return;
    }
    if (sample_rate > 0) {
        s_cfg_rate = (uint32_t)sample_rate;
    }
    /* clock recovery: hold the PCM ring at TARGET_MS by nudging the ratio
     * (smoothed PI, +-1500 ppm) - the phone's 44.1 kHz clock vs our BIG clock */
    float fill = (float)ring_fill(&s_ring);
    s_fill_avg += (fill - s_fill_avg) * 0.002f;
    float err = (s_fill_avg - (float)MS_FRAMES(TARGET_MS)) * (1.0f / (float)MS_FRAMES(TARGET_MS));
    s_integ += err * 0.0005f;
    /* +-1500 ppm: at +-500 the loop sat on the limit with the S26 Ultra (its
     * A2DP clock vs our BIG clock), so the fill was not regulated and wandered
     * 40-104 ms -> occasional underrun clicks */
    if (s_integ > 0.75f) s_integ = 0.75f;
    if (s_integ < -0.75f) s_integ = -0.75f;
    float e = 0.25f * err + s_integ;               /* +-1 = +-0.2 % */
    if (e > 0.75f) e = 0.75f;
    if (e < -0.75f) e = -0.75f;
    s_corr_ppm = (int32_t)(e * 2000.0f);
    uint64_t step = rs_step_q32(s_cfg_rate, OUT_RATE, e);

    int done = 0;
    while (done < num_frames) {
        int m = rs_room(&s_rs);
        if (m > num_frames - done) m = num_frames - done;
        rs_push(&s_rs, data + 2 * done, m);
        done += m;
        int o = rs_run(&s_rs, step, s_out_tmp, RS_OUT_MAX);
        if (o > 0) {
            /* full: throw the OLDEST audio away, always keep the newest */
            uint32_t space = ring_space(&s_ring);
            if ((uint32_t)o > space) {
                s_ring_drops += ring_drop_oldest(&s_ring, (uint32_t)o - space);
            }
            ring_write(&s_ring, s_out_tmp, (uint32_t)o);
        }
    }
    if (!s_primed && ring_fill(&s_ring) >= MS_FRAMES(PRIME_MS)) {
        s_primed = true;
    }
}

static void dsp_task(void *arg)
{
    (void)arg;
    static pkt_t p;
    for (;;) {
        if (xQueueReceive(s_pkt_q, &p, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (s_reset_dsp) {
            s_reset_dsp = false;
            rs_reset(&s_rs);
            ring_skip(&s_ring, ring_fill(&s_ring));
            s_fill_avg = (float)MS_FRAMES(TARGET_MS);
            s_integ = 0;
            s_primed = false;
            s_sbc->configure(&s_sbc_ctx, SBC_MODE_STANDARD, on_pcm, NULL);
        }
        /* RTP header (12 + 4*CC) then the 1-byte SBC media payload header */
        if (p.len < 13) {
            continue;
        }
        int pos = 12 + 4 * (p.data[0] & 0x0f);
        if (pos + 1 >= p.len) {
            continue;
        }
        pos += 1;
        if (pos + 3 <= p.len && p.data[pos] == 0x9c) {
            uint32_t bp = p.data[pos + 2];
            s_bitpool = bp;
            if (bp < s_bp_min) s_bp_min = bp;
            if (bp > s_bp_max) s_bp_max = bp;
        }
        s_sbc->decode_signed_16(&s_sbc_ctx, 0, p.data + pos, p.len - pos);
    }
}

static volatile uint32_t s_frames;
static volatile uint16_t s_pkt_max;

/* BT thread */
static void media_handler(uint8_t seid, uint8_t *packet, uint16_t size)
{
    (void)seid;
    static uint16_t last_seq;
    static bool have_seq;
    if (size < 13 || size > PKT_MAX) {
        return;
    }
    uint16_t seq = big_endian_read_16(packet, 2);
    if (have_seq && (uint16_t)(seq - last_seq) != 1) {
        s_seq_gaps++;
    }
    last_seq = seq;
    have_seq = true;
    s_pkts++;
    s_frames += packet[12] & 0x0F;          /* SBC media payload header: frame count */
    if (size > s_pkt_max) {
        s_pkt_max = size;
    }
    static pkt_t p;
    p.len = size;
    memcpy(p.data, packet, size);
    if (xQueueSend(s_pkt_q, &p, 0) != pdTRUE) {
        s_pkt_drops++;
    }
}

/* ---- bench test signal (encoder task, core 0) ----------------------------------- */
#if S31_TEST_SIGNAL
#define TS_F0       50.0f
#define TS_F1       18000.0f
#define TS_SWEEP_S  9
#define TS_CYCLE_S  20
#define TS_AMP      8192.0f                  /* -12 dBFS */
static rs_state_t s_trs;
static uint32_t s_tfifo[480 + RS_OUT_MAX + 16];
static int s_tfill;
static uint32_t s_tout;                      /* 48 kHz frames into the cycle */
static uint32_t s_tin;                       /* 44.1 kHz frames into the down-sweep */
static float s_tph, s_tf;
static int s_tphase = -1;

static inline int16_t ts_sample(float fs)
{
    float v = sinf(6.2831853f * s_tph) * TS_AMP;
    s_tph += s_tf / fs;
    s_tph -= (float)(int)s_tph;
    return (int16_t)v;
}

static void test_fill(int16_t *pcm, int n)
{
    static const float up = 1.0f, dn = -1.0f;
    for (int k = 0; k < n;) {
        uint32_t t = s_tout;
        int ph = t < TS_SWEEP_S * 48000u ? 0 : t < 10 * 48000u ? 1 : t < (10 + TS_SWEEP_S) * 48000u ? 2 : 3;
        if (ph != s_tphase) {
            s_tphase = ph;
            s_tph = 0;
            if (ph == 0) {
                s_tf = TS_F0;
                ESP_LOGW(TAG, "TEST: UP-sweep 50 Hz -> 18 kHz, made at 48 kHz (encoder only)");
            } else if (ph == 2) {
                s_tf = TS_F1;
                s_tin = 0;
                s_tfill = 0;
                rs_reset(&s_trs);
                ESP_LOGW(TAG, "TEST: DOWN-sweep 18 kHz -> 50 Hz, made at 44.1 kHz through the resampler");
            }
        }
        if (ph == 0) {
            const float r = powf(TS_F1 / TS_F0, up / (TS_SWEEP_S * 48000.0f));
            int16_t v = ts_sample(48000.0f);
            s_tf *= r;
            pcm[2 * k] = v;
            pcm[2 * k + 1] = v;
            k++;
            s_tout++;
        } else if (ph == 2) {
            const float r = powf(TS_F1 / TS_F0, dn / (TS_SWEEP_S * 44100.0f));
            while (s_tfill < n - k) {
                int16_t in[2 * 128];
                for (int j = 0; j < 128; j++) {
                    int16_t v = s_tin < TS_SWEEP_S * 44100u ? ts_sample(44100.0f) : 0;
                    if (s_tin < TS_SWEEP_S * 44100u) {
                        s_tf *= r;
                    }
                    s_tin++;
                    in[2 * j] = v;
                    in[2 * j + 1] = v;
                }
                rs_push(&s_trs, in, 128);
                s_tfill += rs_run(&s_trs, rs_step_q32(44100, OUT_RATE, 0), &s_tfifo[s_tfill], RS_OUT_MAX);
            }
            int take = n - k;
            memcpy(pcm + 2 * k, s_tfifo, (size_t)take * 4);
            memmove(s_tfifo, s_tfifo + take, (size_t)(s_tfill - take) * 4);
            s_tfill -= take;
            k += take;
            s_tout += (uint32_t)take;
        } else {
            pcm[2 * k] = 0;
            pcm[2 * k + 1] = 0;
            k++;
            s_tout++;
        }
        if (s_tout >= TS_CYCLE_S * 48000u) {
            s_tout = 0;
        }
    }
}
#endif

int a2dp_in_test_phase(void)
{
#if S31_TEST_SIGNAL
    return s_tphase;
#else
    return -1;
#endif
}

/* ---- encoder side --------------------------------------------------------------- */
int a2dp_in_read(int16_t *pcm, int n)
{
#if S31_TEST_SIGNAL
    if (s_streaming && s_primed) {
        ring_read_upto(&s_ring, (uint32_t *)pcm, (uint32_t)n);   /* keep A2DP draining, discard */
    }
    test_fill(pcm, n);
    return n;
#endif
    if (!s_streaming || !s_primed) {
        memset(pcm, 0, (size_t)n * 4);
        return 0;
    }
    uint32_t take = ring_read_upto(&s_ring, (uint32_t *)pcm, (uint32_t)n);
    if (take < (uint32_t)n) {
        memset(pcm + 2 * take, 0, (size_t)(n - take) * 4);
        s_underruns++;
    }
    return (int)take;
}

bool a2dp_in_streaming(void) { return s_streaming; }
bool a2dp_in_connected(void) { return s_connected; }

void a2dp_in_log_stats(void)
{
    static uint32_t last_pkts, last_frames;
    uint32_t pk = s_pkts, fr = s_frames;
    uint32_t dp = pk - last_pkts;
    ESP_LOGW(TAG, "A2DP 2s: %s, media pkts %lu (+%lu, %lu.%lu SBC frames/pkt, max %u B), q drops %lu, seq gaps %lu | bitpool %lu (min %lu max %lu) | "
             "PCM %lu ms, underruns %lu, oldest dropped %lu, clock corr %ld ppm",
             s_streaming ? "STREAMING" : s_connected ? "connected" : "no phone",
             (unsigned long)pk, (unsigned long)dp,
             (unsigned long)(dp ? (fr - last_frames) / dp : 0), (unsigned long)(dp ? ((fr - last_frames) * 10 / dp) % 10 : 0),
             (unsigned)s_pkt_max, (unsigned long)s_pkt_drops,
             (unsigned long)s_seq_gaps, (unsigned long)s_bitpool,
             (unsigned long)(s_bp_min == 0xff ? 0 : s_bp_min), (unsigned long)s_bp_max,
             (unsigned long)(ring_fill(&s_ring) / (OUT_RATE / 1000)), (unsigned long)s_underruns,
             (unsigned long)s_ring_drops, (long)s_corr_ppm);
    last_pkts = pk;
    last_frames = fr;
    s_pkt_max = 0;
    s_bp_min = 0xff;
    s_bp_max = 0;
}

/* ---- BTstack events (BT thread) --------------------------------------------- */
static void a2dp_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET || hci_event_packet_get_type(packet) != HCI_EVENT_A2DP_META) {
        return;
    }
    switch (packet[2]) {
    case A2DP_SUBEVENT_SIGNALING_CONNECTION_ESTABLISHED: {
        uint8_t st = a2dp_subevent_signaling_connection_established_get_status(packet);
        a2dp_subevent_signaling_connection_established_get_bd_addr(packet, s_peer);
        if (st != ERROR_CODE_SUCCESS) {
            ESP_LOGW(TAG, "A2DP connect to %s failed, status 0x%02x", bd_addr_to_str(s_peer), st);
            break;
        }
        s_a2dp_cid = a2dp_subevent_signaling_connection_established_get_a2dp_cid(packet);
        s_connected = true;
        {   /* remember the phone: called back at power-up */
            nvs_handle_t h;
            if (nvs_open("s31", NVS_READWRITE, &h) == ESP_OK) {
                nvs_set_blob(h, "last_phone", s_peer, 6);
                nvs_commit(h);
                nvs_close(h);
            }
        }
        hci_con_handle_t h = a2dp_subevent_signaling_connection_established_get_con_handle(packet);
        hci_role_t role = gap_get_role(h);
        ESP_LOGW(TAG, "A2DP connected %s, role %s", bd_addr_to_str(s_peer),
                 role == HCI_ROLE_MASTER ? "MASTER (S31 schedules the phone)" : "SLAVE");
        break;
    }
    case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION: {
        uint16_t sr = a2dp_subevent_signaling_media_codec_sbc_configuration_get_sampling_frequency(packet);
        s_cfg_rate = sr;
        ESP_LOGW(TAG, "SBC configured: %u Hz, %u ch, block %u, subbands %u, bitpool %u-%u", sr,
                 a2dp_subevent_signaling_media_codec_sbc_configuration_get_num_channels(packet),
                 a2dp_subevent_signaling_media_codec_sbc_configuration_get_block_length(packet),
                 a2dp_subevent_signaling_media_codec_sbc_configuration_get_subbands(packet),
                 a2dp_subevent_signaling_media_codec_sbc_configuration_get_min_bitpool_value(packet),
                 a2dp_subevent_signaling_media_codec_sbc_configuration_get_max_bitpool_value(packet));
        break;
    }
    case A2DP_SUBEVENT_STREAM_STARTED:
        s_reset_dsp = true;
        s_streaming = true;
        ESP_LOGW(TAG, "A2DP audio STARTED");
        break;
    case A2DP_SUBEVENT_STREAM_SUSPENDED:
    case A2DP_SUBEVENT_STREAM_STOPPED:
        s_streaming = false;
        ESP_LOGW(TAG, "A2DP audio %s", packet[2] == A2DP_SUBEVENT_STREAM_SUSPENDED ? "suspended" : "stopped");
        break;
    case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
        s_streaming = false;
        s_connected = false;
        ESP_LOGW(TAG, "A2DP disconnected");
        break;
    default:
        break;
    }
}

static void hci_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }
    switch (hci_event_packet_get_type(packet)) {
    case HCI_EVENT_PIN_CODE_REQUEST: {
        bd_addr_t a;
        hci_event_pin_code_request_get_bd_addr(packet, a);
        gap_pin_code_response(a, "0000");
        break;
    }
    case HCI_EVENT_SIMPLE_PAIRING_COMPLETE:
        ESP_LOGW(TAG, "pairing %s (status 0x%02x)",
                 hci_event_simple_pairing_complete_get_status(packet) ? "FAILED" : "complete",
                 hci_event_simple_pairing_complete_get_status(packet));
        break;
    case HCI_EVENT_AUTHENTICATION_COMPLETE:
        ESP_LOGW(TAG, "authentication status 0x%02x", hci_event_authentication_complete_get_status(packet));
        break;
    case HCI_EVENT_DISCONNECTION_COMPLETE:
        ESP_LOGW(TAG, "ACL disconnected, reason 0x%02x",
                 hci_event_disconnection_complete_get_reason(packet));
        break;
    case HCI_EVENT_ROLE_CHANGE:
        ESP_LOGW(TAG, "role change: now %s", hci_event_role_change_get_role(packet) == 0 ? "MASTER" : "SLAVE");
        break;
    default:
        break;
    }
}

static void avrcp_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)packet_type; (void)channel; (void)packet; (void)size;
}

void a2dp_in_setup(void)
{
    /* DSP side */
    rs_init_kernel();
    rs_reset(&s_rs);
    uint32_t slots = MS_FRAMES(RING_MS) + 1;
    s_ring.buf = heap_caps_malloc(slots * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_ring.slots = slots;
    s_pkt_q = xQueueCreateWithCaps(PKT_Q_LEN, sizeof(pkt_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_sbc = btstack_sbc_decoder_bluedroid_init_instance(&s_sbc_ctx);
    s_sbc->configure(&s_sbc_ctx, SBC_MODE_STANDARD, on_pcm, NULL);
    /* core 0 = DSP core, below the LC3 encoder (21) */
    xTaskCreatePinnedToCore(dsp_task, "sbc_dsp", 12288, NULL, configMAX_PRIORITIES - 6, NULL, 0);

    /* Bluetooth side */
    l2cap_init();
    sdp_init();
    a2dp_sink_init();
    avrcp_init();
    avrcp_controller_init();
    avrcp_target_init();
    a2dp_sink_register_packet_handler(&a2dp_handler);
    a2dp_sink_register_media_handler(&media_handler);
    avrcp_register_packet_handler(&avrcp_handler);
    avrcp_controller_register_packet_handler(&avrcp_handler);
    avrcp_target_register_packet_handler(&avrcp_handler);
    avdtp_stream_endpoint_t *ep = a2dp_sink_create_stream_endpoint(AVDTP_AUDIO, AVDTP_CODEC_SBC,
                                                                  s_sbc_caps, sizeof(s_sbc_caps),
                                                                  s_sbc_cfg, sizeof(s_sbc_cfg));
    s_local_seid = avdtp_local_seid(ep);

    a2dp_sink_create_sdp_record(s_sdp_a2dp, sdp_create_service_record_handle(),
                                AVDTP_SINK_FEATURE_MASK_SPEAKER, NULL, NULL);
    sdp_register_service(s_sdp_a2dp);
    avrcp_controller_create_sdp_record(s_sdp_avrcp_ct, sdp_create_service_record_handle(),
                                       AVRCP_FEATURE_MASK_CATEGORY_PLAYER_OR_RECORDER, NULL, NULL);
    sdp_register_service(s_sdp_avrcp_ct);
    avrcp_target_create_sdp_record(s_sdp_avrcp_tg, sdp_create_service_record_handle(),
                                   AVRCP_FEATURE_MASK_CATEGORY_MONITOR_OR_AMPLIFIER, NULL, NULL);
    sdp_register_service(s_sdp_avrcp_tg);
    device_id_create_sdp_record(s_sdp_did, sdp_create_service_record_handle(),
                                DEVICE_ID_VENDOR_ID_SOURCE_BLUETOOTH, BLUETOOTH_COMPANY_ID_BLUEKITCHEN_GMBH, 1, 1);
    sdp_register_service(s_sdp_did);

    /* BR/EDR master, never switch, no sniff (sniff anchors starve against the BIG) */
    gap_set_default_link_policy_settings(0);
    gap_set_allow_role_switch(false);
    hci_set_master_slave_policy(HCI_ROLE_MASTER);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    /* Just Works like a speaker: BTstack's default leaves the phone's User
     * Confirmation Request unanswered, so pairing timed out (reason 0x15). */
    gap_ssp_set_auto_accept(1);
    gap_set_bondable_mode(1);
    gap_set_class_of_device(0x240414);            /* audio, loudspeaker */
    gap_discoverable_control(!S31_QUIET_RADIO_TEST);
    gap_connectable_control(!S31_QUIET_RADIO_TEST);

    s_hci_reg.callback = &hci_handler;
    hci_add_event_handler(&s_hci_reg);
}

/* Power-up: call the last phone back (like a real speaker), twice at most. */
static btstack_timer_source_t s_reconn_timer;
static int s_reconn_tries;
static bd_addr_t s_last_phone;

static void reconnect_cb(btstack_timer_source_t *ts)
{
    (void)ts;
    if (s_connected || s_reconn_tries >= 2) {
        return;
    }
    s_reconn_tries++;
    ESP_LOGW(TAG, "power-up: calling back last phone %s (try %d/2)", bd_addr_to_str(s_last_phone), s_reconn_tries);
    uint8_t st = a2dp_sink_establish_stream(s_last_phone, &s_a2dp_cid);
    if (st != ERROR_CODE_SUCCESS) {
        ESP_LOGW(TAG, "a2dp_sink_establish_stream: 0x%02x", st);
    }
    btstack_run_loop_set_timer(&s_reconn_timer, 10000);
    btstack_run_loop_add_timer(&s_reconn_timer);
}

void a2dp_in_on_hci_working(void)
{
    if (S31_QUIET_RADIO_TEST) {
        ESP_LOGW(TAG, "QUIET RADIO TEST: BR/EDR off air (no scan, no call-back)");
        return;
    }
    nvs_handle_t h;
    size_t n = 6;
    if (nvs_open("s31", NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    esp_err_t err = nvs_get_blob(h, "last_phone", s_last_phone, &n);
    nvs_close(h);
    if (err != ESP_OK || n != 6) {
        return;
    }
    btstack_run_loop_set_timer_handler(&s_reconn_timer, reconnect_cb);
    btstack_run_loop_set_timer(&s_reconn_timer, 2000);
    btstack_run_loop_add_timer(&s_reconn_timer);
}
