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
#include "le_audio.h"
#include "link_tune.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "btstack.h"
#include "classic/btstack_sbc.h"
#include "aac_decoder.h"
#include "latm_parser.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"

#include "a2dp_dsp.h"
#include "bt_sched_tune.h"
#include "auracast.h"
#include "party.h"
#include "jbl_v5.h"
#include "tones.h"
#include "hci_dump.h"

static const char *TAG = "A2DP";

#define OUT_RATE        48000
#define PKT_MAX         (AVDTP_L2CAP_MTU + 16)
#define PKT_Q_LEN       24                    /* ~550 ms of 8-frame packets */
#define RING_MS         500                   /* max S31-added delay: beyond it the OLDEST audio is dropped */
/* Average PCM fill (= added latency). While the S31 broadcasts, phones deliver in
 * bursts (~75 ms peak to peak against the encoder's 30 ms reads), with 60-77 ms
 * late packets and stretches ~3 % below realtime on top: 130 ms ran dry a few
 * times a minute, and every underrun costs a re-prime (silence) + a splice.
 * The delay report tells the phone (video lip sync). */
#define A2DP_TARGET_MS  180
/* LE Audio: SDUs come on the controller's fixed ISO schedule, no phone bursts -
 * one ISO interval + 20 ms is enough (set per stream, a2dp_in_ext_start) */
static volatile uint32_t s_target_ms = A2DP_TARGET_MS;
#define TARGET_MS       (s_target_ms)
#define PRIME_MS        TARGET_MS             /* start output at the target fill: no start-up underruns / integrator wind-up */
#define MS_FRAMES(ms)   ((OUT_RATE / 1000) * (ms))

/* ---- AVDTP endpoint / SDP --------------------------------------------------- */
/* All SBC configurations, bitpool pinned at 53 (min = max): the phone cannot
 * fall back to its adaptive low bitpools. */
static uint8_t s_sbc_caps[] = { 0xFF, 0xFF, 53, 53 };
static uint8_t s_sbc_cfg[4];
/* AAC-LC (A2DP 1.4 layout): MPEG-2 + MPEG-4 AAC LC (no HE-AAC: no SBR in the
 * decoder), 44.1 + 48 kHz, 1 or 2 channels, VBR, up to 320 kbit/s. iPhones and
 * most Android phones prefer it over SBC. */
static uint8_t s_aac_caps[6] = { 0xC0, 0x01, 0x8C, 0x84, 0xE2, 0x00 };
static uint8_t s_aac_cfg[6];
static uint8_t s_sbc_seid, s_aac_seid;
enum { CODEC_SBC = 0, CODEC_AAC = 1 };
static volatile bool s_ext_on;                       /* LE Audio feeds the pipeline */
static a2dp_in_ext_decode_t s_ext_dec;
static const char *s_ext_what = "";
static volatile uint8_t s_codec = CODEC_SBC;           /* configured by the phone */
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
/* Stale-audio flush: 500 ms after the last media packet (pause, suspend, stop
 * or disconnect) the packet queue and the PCM ring are emptied, so a resume
 * never starts with audio from before the pause. */
#define FLUSH_IDLE_MS 500
static volatile bool s_flush_req;            /* BT -> reader: drop the whole ring */
static volatile int64_t s_last_media_us;     /* BT: last media packet, 0 = flushed */
static volatile uint32_t s_flushes;
static btstack_timer_source_t s_flush_timer;

/* ---- DSP state (core 0) ------------------------------------------------------ */
static btstack_sbc_decoder_bluedroid_t s_sbc_ctx;
static const btstack_sbc_decoder_t *s_sbc;
static rs_state_t s_rs;
static spsc_ring_t s_ring;
static float s_fill_avg, s_integ;
static volatile int32_t s_corr_ppm;
static volatile uint32_t s_underruns, s_ring_drops;
/* Constant latency: audio played as silence during a delivery stall is owed.
 * When the phone's late packets then arrive, that much is skipped (crossfaded),
 * so the delay returns to TARGET_MS at once instead of staying target + stall
 * (clock correction alone needs minutes to drain it, +-1500 ppm). */
static uint32_t s_deficit;                /* frames, encoder task only */
static uint32_t s_deficit_reads;          /* reads since the deficit last grew */
static volatile uint32_t s_catchups, s_catchup_frames, s_deficit_expired;
static volatile bool s_reset_deficit;
static float s_rd_fill_ema = -1.0f;          /* reader-side fill average (~200 ms), frames */
static volatile uint32_t s_win_min = 0xffffffffu, s_win_max;   /* reader-side fill, this stats window */
static volatile bool s_reset_loop;           /* reader -> DSP: restart clock recovery at the target */
/* Clock tracking, deadband + sustained trend (as the S31_A2DP_Sink fork's
 * audio_sink_srv_track_source_clock): never react to the phone's delivery
 * bursts, only to a fill that STAYS outside the band, in small steps. */
#define TRACK_PERIOD_READS 50                /* encoder reads 10 ms each: sample every 500 ms */
#define TRACK_TREND        2                 /* 2 samples in a row (1 s) outside the band */
#define TRACK_LO_MS        (TARGET_MS - 10)
#define TRACK_HI_MS        (TARGET_MS + 10)
#define TRACK_STEP         0.01f             /* 20 ppm per ms of error beyond the band, max 200 ppm (e: +-1 = +-0.2 %) */
#define TRACK_CLAMP        1.5f              /* +-3000 ppm: the phone clock AND our I2S APLL (measured 0.25 % in LOCAL) */
static volatile float s_track_e;             /* reader -> DSP resampler ratio */
static uint32_t s_track_reads, s_track_lo, s_track_hi;
static float s_track_acc;
static uint32_t s_track_n;
/* Underflow forensics (as the fork): input as % of realtime since the last one */
static volatile uint32_t s_in_frames;        /* DSP: frames written into the ring */
static uint32_t s_uf_frames0;
static int64_t s_uf_t0;
static bool s_reprime;                       /* reader: ran dry, silence until the target is back */
static int64_t s_last_read_us;               /* reader: a pause > 50 ms (start, BIG coming up) -> align */
static uint32_t s_ceiling_hits;
static volatile uint32_t s_aligns, s_ceilings, s_acl_boosts;
#define CEILING_MS (TARGET_MS + 20)          /* 1 s average above this: splice to the target */
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
    float e = s_track_e;                           /* deadband clock tracking (reader side) */
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
            s_in_frames += (uint32_t)o;
        }
    }
    if (!s_primed && ring_fill(&s_ring) >= MS_FRAMES(PRIME_MS)) {
        s_primed = true;
    }
}

/* ---- AAC (core 0, DSP task) ----------------------------------------------------- */
static aac_decoder_t s_aac;
static latm_parser_t s_latm;
static bool s_aac_ready, s_aac_cfg_done;
static uint8_t s_aac_raw[2048];
static int16_t s_aac_pcm[2 * 1024];
static int16_t s_aac_st[2 * 1024];
static volatile uint32_t s_aac_frames, s_aac_errs;

/* the decoder workspace: AAC (Helix ~20.7 KB) or 2 x LC3 decoder (~18 KB), static in
 * main SRAM. Heap copies landed in the slow RTC RAM once internal RAM ran low and the
 * decoder then ran 10-15x slower (stutter, then the task watchdog). */
#define CODEC_WS_SIZE (24 * 1024)
static uint8_t s_codec_ws[CODEC_WS_SIZE] __attribute__((aligned(16)));
static int s_ws_owner;

void *a2dp_in_codec_ws(unsigned *size)
{
    if (size) *size = sizeof(s_codec_ws);
    return s_codec_ws;
}

bool a2dp_in_codec_ws_take(int owner)
{
    if (s_ws_owner == owner) return true;
    s_ws_owner = owner;
    return false;
}

/* probes: is the core slow, or the memory? (logged with the decode times) */
static uint32_t probe_loop_us(void)
{
    int64_t t0 = esp_timer_get_time();
    volatile uint32_t acc = 1;
    for (uint32_t k = 0; k < 20000; k++) acc = acc * 1664525u + 1013904223u;
    return (uint32_t)(esp_timer_get_time() - t0);
}

static uint32_t probe_mem_us(const void *p, unsigned bytes)
{
    int64_t t0 = esp_timer_get_time();
    uint32_t sum = 0;
    const volatile uint32_t *w = (const volatile uint32_t *)p;
    for (int r = 0; r < 4; r++) for (unsigned k = 0; k < bytes / 4; k += 8) sum += w[k];
    (void)sum;
    return (uint32_t)(esp_timer_get_time() - t0);
}

void a2dp_in_probe_log(const char *what)
{
    ESP_LOGI(TAG, "%s probes: loop %lu us, workspace %lu us (ws at %p) | internal free %u B", what,
             (unsigned long)probe_loop_us(), (unsigned long)probe_mem_us(s_codec_ws, sizeof(s_codec_ws)),
             (void *)s_codec_ws, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

static void aac_reset(void)
{
    latm_parser_reset(&s_latm);
    latm_parser_init(&s_latm);
    s_aac_cfg_done = false;
    s_aac_ready = aac_decoder_init(&s_aac) == AAC_OK;
    if (!s_aac_ready) ESP_LOGE(TAG, "AAC decoder init failed");
}

/* RTP header, then one LATM AudioMuxElement (StreamMuxConfig in band) */
static void aac_decode_packet(const uint8_t *d, int len)
{
    if (!a2dp_in_codec_ws_take(1)) aac_reset();          /* LE Audio had the workspace */
    if (!s_aac_ready || len < 13) return;
    int pos = 12 + 4 * (d[0] & 0x0f);
    if (pos >= len) return;
    size_t raw = 0;
    latm_error_t le = latm_parse_frame(&s_latm, d + pos, (size_t)(len - pos), s_aac_raw, sizeof(s_aac_raw), &raw, NULL);
    if (le != LATM_OK || raw == 0) {
        if (le != LATM_ERR_NOT_ENOUGH_DATA) s_aac_errs++;
        return;
    }
    const latm_audio_config_t *ac = latm_get_audio_config(&s_latm);
    if (!s_aac_cfg_done) {
        if (ac == NULL || !ac->config_valid) return;
        aac_decoder_configure(&s_aac, (int)ac->sample_rate, ac->channel_config);
        s_aac_cfg_done = true;
        ESP_LOGW(TAG, "AAC stream: object type %u, %lu Hz, %u ch", ac->object_type,
                 (unsigned long)ac->sample_rate, ac->channel_config);
    }
    int n = 0;
    int64_t t0 = esp_timer_get_time();
    int ar = (int)aac_decoder_decode(&s_aac, s_aac_raw, (int)raw, s_aac_pcm, &n);
    uint32_t us = (uint32_t)(esp_timer_get_time() - t0);
    static uint32_t us_sum, us_max, us_n;
    us_sum += us; us_n++;
    if (us > us_max) us_max = us;
    if (us_n == 430) {                                  /* ~10 s at 44.1 kHz */
        ESP_LOGI(TAG, "AAC decode: avg %lu us, max %lu us per frame (%lu B in, budget %lu us) | errs %lu",
                 (unsigned long)(us_sum / us_n), (unsigned long)us_max, (unsigned long)raw,
                 (unsigned long)(1024u * 1000000u / (ac->sample_rate ? ac->sample_rate : 44100u)),
                 (unsigned long)s_aac_errs);
        a2dp_in_probe_log("AAC");
        us_sum = us_max = us_n = 0;
    }
    if (ar != AAC_OK || n <= 0) {
        s_aac_errs++;
        return;
    }
    s_aac_frames++;
    if (n > 1024) n = 1024;
    int16_t *pcm = s_aac_pcm;
    if (ac->channel_config == 1) {                     /* mono: both sides */
        for (int i = 0; i < n; i++) s_aac_st[2 * i] = s_aac_st[2 * i + 1] = s_aac_pcm[i];
        pcm = s_aac_st;
    }
    on_pcm(pcm, n, 2, (int)ac->sample_rate, NULL);
}

static void dsp_task(void *arg)
{
    (void)arg;
    static pkt_t p;
    int64_t busy_since = 0;
    for (;;) {
        /* any codec: never hold the core for long - after 50 ms of back-to-back
         * decoding, give it up for a tick (the idle task / watchdog, other tasks) */
        if (uxQueueMessagesWaiting(s_pkt_q) == 0) {
            busy_since = 0;                        /* about to block: not busy */
        } else {
            int64_t now = esp_timer_get_time();
            if (busy_since == 0) {
                busy_since = now;
            } else if (now - busy_since > 50000) {
                vTaskDelay(1);
                busy_since = 0;
            }
        }
        if (xQueueReceive(s_pkt_q, &p, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (s_reset_dsp) {
            s_reset_dsp = false;
            s_flush_req = false;                   /* emptied here: the reader must not drop the new stream */
            rs_reset(&s_rs);
            ring_skip(&s_ring, ring_fill(&s_ring));
            s_fill_avg = (float)MS_FRAMES(TARGET_MS);
            s_integ = 0;
            s_primed = false;
            s_reset_deficit = true;
            s_sbc->configure(&s_sbc_ctx, SBC_MODE_STANDARD, on_pcm, NULL);
            if (s_codec == CODEC_AAC && !s_ext_on) aac_reset();
        }
        if (s_ext_on) {
            if (s_ext_dec) s_ext_dec(p.data, p.len);
            continue;
        }
        if (s_codec == CODEC_AAC) {
            aac_decode_packet(p.data, p.len);
            continue;
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
/* Delivery timing (BT thread): gaps between media packets as they reach the
 * host. A gap with no seq gap = the phone's packets were late on air (or held
 * by the phone), not lost. Logged as they happen so they line up with the
 * other logs. */
static volatile uint32_t s_gap_max_us, s_gap_n60, s_gap_n100, s_gap_n200;
static int64_t s_last_arrival;

static uint32_t s_conn_ms, s_start_ms, s_media_first_ms;   /* A2DP settle window (BT thread) */

bool a2dp_in_settling(void)
{
    if (!s_connected || s_ext_on) return false;
    uint32_t now = btstack_run_loop_get_time_ms();
    if (now - s_conn_ms < 4000) return true;
    if (s_streaming && (s_media_first_ms == 0 || now - s_media_first_ms < 1000)) return now - s_start_ms < 8000;
    return false;
}

static void media_handler(uint8_t seid, uint8_t *packet, uint16_t size)
{
    (void)seid;
    static uint16_t last_seq;
    static uint32_t last_rtp;
    static bool have_seq;
    if (size < 13 || size > PKT_MAX || s_ext_on) {
        return;                                    /* LE Audio owns the pipeline meanwhile */
    }
    if (s_media_first_ms == 0) s_media_first_ms = btstack_run_loop_get_time_ms();
    uint16_t seq = big_endian_read_16(packet, 2);
    uint32_t rtp = big_endian_read_32(packet, 4);
    int64_t now = esp_timer_get_time();
    if (have_seq && s_last_arrival != 0) {
        uint32_t gap = (uint32_t)(now - s_last_arrival);
        if (gap > s_gap_max_us) s_gap_max_us = gap;
        if (gap > 60000) {
            s_gap_n60++;
            if (gap > 100000) s_gap_n100++;
            if (gap > 200000) s_gap_n200++;
            uint32_t audio_ms = (uint32_t)(rtp - last_rtp) * 1000u / (s_cfg_rate ? s_cfg_rate : 44100u);
            ESP_LOGW(TAG, "DELIVERY GAP %lu ms before seq %u (seq step %u, RTP step %lu ms of audio) | PCM %lu ms",
                     (unsigned long)(gap / 1000), seq, (uint16_t)(seq - last_seq), (unsigned long)audio_ms,
                     (unsigned long)(ring_fill(&s_ring) / (OUT_RATE / 1000)));
        }
    }
    s_last_arrival = now;
    s_last_media_us = now;
    last_rtp = rtp;
    if (have_seq && (uint16_t)(seq - last_seq) != 1) {
        s_seq_gaps++;
    }
    last_seq = seq;
    have_seq = true;
    s_pkts++;
    s_frames += s_codec == CODEC_SBC ? (packet[12] & 0x0F) : 1;   /* SBC payload header: frame count; AAC: 1 per packet */
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

/* AVRCP absolute volume 0..127 -> gain (cubic: about -40 dB at 22 %, 0 dB at 127) */
static volatile uint8_t s_volume = 100;
static volatile bool s_muted;
static float s_gain_cur = -1.0f;

static float volume_gain(uint8_t v)
{
    float x = (float)v / 127.0f;
    return x * x * x;
}

/* Local speaker only (audio_out): the Auracast broadcast stays full scale like
 * a real JBL master, so party members keep their own volume. */
void a2dp_in_apply_volume(int16_t *pcm, int n)
{
    float target = s_muted ? 0.0f : volume_gain(s_volume);
    if (s_gain_cur < 0.0f) {
        s_gain_cur = target;
    }
    if (target == 1.0f && s_gain_cur == 1.0f) {
        return;
    }
    /* ramp over this frame: no zipper noise on volume steps */
    float g = s_gain_cur, dg = (target - g) / (float)n;
    for (int i = 0; i < n; i++, g += dg) {
        pcm[2 * i] = (int16_t)lrintf((float)pcm[2 * i] * g);
        pcm[2 * i + 1] = (int16_t)lrintf((float)pcm[2 * i + 1] * g);
    }
    s_gain_cur = target;
}

/* Late audio arrived after an underrun: skip what was already played as silence,
 * crossfading the old position into the new one over this frame. */
static bool catch_up(int16_t *pcm, int n, bool excess)
{
    if (!excess && s_deficit_reads++ > 300) {      /* 3 s: the phone flushed it, nothing is coming */
        s_deficit = 0;
        s_deficit_expired++;
        return false;
    }
    uint32_t fill = ring_fill(&s_ring), tgt = MS_FRAMES(TARGET_MS);
    if (n > 480 || fill < tgt + 3u * (uint32_t)n) {
        return false;
    }
    uint32_t skip = fill - tgt - 2u * (uint32_t)n;
    if (!excess && skip > s_deficit) skip = s_deficit;
    if (skip < (uint32_t)n) {                      /* not worth a splice yet */
        return false;
    }
    static int16_t b[2 * 480];
    ring_read(&s_ring, (uint32_t *)pcm, (uint32_t)n);   /* continues what just played */
    ring_skip(&s_ring, skip);
    ring_read(&s_ring, (uint32_t *)b, (uint32_t)n);     /* the audio we jump to */
    for (int i = 0; i < n; i++) {
        float t = ((float)i + 0.5f) / (float)n;
        pcm[2 * i] = (int16_t)lrintf((float)pcm[2 * i] * (1.0f - t) + (float)b[2 * i] * t);
        pcm[2 * i + 1] = (int16_t)lrintf((float)pcm[2 * i + 1] * (1.0f - t) + (float)b[2 * i + 1] * t);
    }
    s_deficit = skip >= s_deficit ? 0 : s_deficit - skip;
    s_catchups++;
    s_catchup_frames += skip;
    return true;
}

/* ---- encoder side --------------------------------------------------------------- */
static void ext_target_follow(void);

int a2dp_in_read(int16_t *pcm, int n)
{
#if S31_TEST_SIGNAL
    if (s_streaming && s_primed) {
        ring_read_upto(&s_ring, (uint32_t *)pcm, (uint32_t)n);   /* keep A2DP draining, discard */
    }
    test_fill(pcm, n);
    return n;
#endif
    ext_target_follow();
    if (s_flush_req) {
        s_flush_req = false;
        ring_skip(&s_ring, ring_fill(&s_ring));
        s_primed = false;                          /* re-prime on the next stream */
        s_reset_deficit = true;
    }
    if (s_reset_deficit) {
        s_reset_deficit = false;
        s_deficit = 0;
        s_rd_fill_ema = -1.0f;
        s_reprime = false;
        s_track_reads = s_track_lo = s_track_hi = 0;
        s_track_acc = 0;
        s_track_n = 0;
        s_uf_t0 = 0;
        s_track_e = 0;                             /* new stream: re-learn the offset (~100 ppm, a few s) */
        s_last_read_us = 0;
        s_ceiling_hits = 0;
    }
    if (!s_streaming || !s_primed) {
        memset(pcm, 0, (size_t)n * 4);
        return 0;
    }
    {   /* (re)start of reading: the backlog that built up while nobody read is
         * not "needed" audio - start exactly at the target */
        int64_t now = esp_timer_get_time();
        bool resumed = s_last_read_us == 0 || now - s_last_read_us > 50000;
        s_last_read_us = now;
        uint32_t fill0 = ring_fill(&s_ring);
        if (resumed && fill0 > MS_FRAMES(TARGET_MS)) {
            ring_skip(&s_ring, fill0 - MS_FRAMES(TARGET_MS));
            s_aligns++;
            s_deficit = 0;
            s_reprime = false;
            s_track_reads = s_track_lo = s_track_hi = 0;
            s_track_acc = 0;
            s_track_n = 0;
        }
    }
    float f = (float)ring_fill(&s_ring);
    {   /* deep fix for delivery gaps: while the cushion is low, raise the phone
         * link's controller priority (base 2 -> 9: with missed-poll escalation it
         * beats the BIS repetitions (12) after ~30 ms instead of ~100 ms) */
        static bool boosted;
        if (!boosted && !s_ext_on && f < (float)(OUT_RATE / 1000) * ((float)TARGET_MS - 60.0f) && s_streaming) {
            boosted = true;
            s_acl_boosts++;
            bt_sched_tune_apply_custom(9, 15, 12, 11, "A2DP cushion low: phone link boosted");
        } else if (boosted && (f > (float)(OUT_RATE / 1000) * ((float)TARGET_MS - 20.0f) || !s_streaming || s_ext_on)) {
            boosted = false;
            bt_sched_tune_apply_custom(2, 15, 12, 11, "A2DP cushion back: BIS copies first");
        }
    }
    if ((uint32_t)f < s_win_min) s_win_min = (uint32_t)f;
    if ((uint32_t)f > s_win_max) s_win_max = (uint32_t)f;
    s_rd_fill_ema = s_rd_fill_ema < 0.0f ? f : s_rd_fill_ema + (f - s_rd_fill_ema) * 0.05f;
    /* tracker: averaged fill, sampled every 500 ms */
    s_track_acc += f;
    s_track_n++;
    if (++s_track_reads >= TRACK_PERIOD_READS && !s_reprime) {
        float avg = s_track_acc / (float)s_track_n;
        s_track_reads = 0;
        s_track_acc = 0;
        s_track_n = 0;
        /* hard ceiling: 2 samples (1 s) averaging above the ceiling is not a burst */
        if (avg > (float)MS_FRAMES(CEILING_MS)) {
            if (++s_ceiling_hits >= 2 && catch_up(pcm, n, true)) {
                s_ceiling_hits = 0;
                s_ceilings++;
                s_track_lo = s_track_hi = 0;
                /* full scale out: the phone volume is applied at the local speaker only */
                return n;
            }
        } else {
            s_ceiling_hits = 0;
        }
        int dir = 0;
        /* anti-windup: step only while the error is NOT already shrinking -
         * the fill answers a rate change slowly (1000 ppm = 1 ms/s) */
        static float prev_avg = -1.0f;
        bool rising = prev_avg >= 0.0f && avg > prev_avg + (float)MS_FRAMES(1);
        bool falling = prev_avg >= 0.0f && avg < prev_avg - (float)MS_FRAMES(1);
        prev_avg = avg;
        if (avg < (float)MS_FRAMES(TRACK_LO_MS)) {
            s_track_hi = 0;
            if (++s_track_lo >= TRACK_TREND && !rising) dir = -1;   /* draining: consume slower */
        } else if (avg > (float)MS_FRAMES(TRACK_HI_MS)) {
            s_track_lo = 0;
            if (++s_track_hi >= TRACK_TREND && !falling) dir = +1;  /* filling: consume faster */
        } else {
            s_track_lo = s_track_hi = 0;
        }
        if (dir != 0) {
            s_track_lo = s_track_hi = 0;
            float off_ms = (avg - (float)MS_FRAMES(TARGET_MS)) / (float)(OUT_RATE / 1000);
            float mag = fabsf(off_ms) - 10.0f;            /* ms beyond the band edge */
            if (mag < 1.0f) mag = 1.0f;
            if (mag > 5.0f) mag = 5.0f;              /* max 100 ppm per step */
            float e = s_track_e + (float)dir * TRACK_STEP * mag;
            if (e > TRACK_CLAMP) e = TRACK_CLAMP;
            if (e < -TRACK_CLAMP) e = -TRACK_CLAMP;
            s_track_e = e;
        }
    }
    if (s_reprime) {                               /* PREFETCHING: silence until the target is back */
        if (ring_fill(&s_ring) < MS_FRAMES(TARGET_MS)) {
            memset(pcm, 0, (size_t)n * 4);
            s_deficit += (uint32_t)n;              /* time passes: late audio arriving later is owed */
            if (s_deficit > MS_FRAMES(RING_MS)) s_deficit = MS_FRAMES(RING_MS);
            s_deficit_reads = 0;
            /* full scale out: the phone volume is applied at the local speaker only */
            return 0;
        }
        s_reprime = false;
    }
    if (s_deficit > 0 && catch_up(pcm, n, false)) {
        /* full scale out: the phone volume is applied at the local speaker only */
        return n;
    }
    uint32_t take = ring_read_upto(&s_ring, (uint32_t *)pcm, (uint32_t)n);
    if (take < (uint32_t)n) {
        memset(pcm + 2 * take, 0, (size_t)(n - take) * 4);
        s_underruns++;
        s_reprime = true;
        {   /* forensics: <100 % = starved (phone/link), ~100 % = bursts vs cushion */
            int64_t now = esp_timer_get_time();
            uint32_t in = s_in_frames;
            if (s_uf_t0 != 0 && now > s_uf_t0) {
                uint64_t dt = (uint64_t)(now - s_uf_t0);
                uint32_t pct = (uint32_t)((uint64_t)(in - s_uf_frames0) * 1000000000ull / ((uint64_t)OUT_RATE * dt));   /* per mille */
                ESP_LOGW(TAG, "UNDERFLOW: input %lu.%lu%% of realtime over %lu ms, longest delivery gap %lu ms, corr %ld ppm",
                         (unsigned long)(pct / 10), (unsigned long)(pct % 10), (unsigned long)(dt / 1000),
                         (unsigned long)(s_gap_max_us / 1000), (long)s_corr_ppm);
            }
            s_uf_t0 = now;
            s_uf_frames0 = in;
        }
        s_deficit += (uint32_t)n - take;
        if (s_deficit > MS_FRAMES(RING_MS)) s_deficit = MS_FRAMES(RING_MS);
        s_deficit_reads = 0;
    }
    /* full scale out: the phone volume is applied at the local speaker only */
    return (int)take;
}

bool a2dp_in_streaming(void) { return s_streaming; }

/* LE Audio: the base buffer, plus one more broadcast interval while we host (the
 * encoder pulls ISO-interval bursts) - follows Auracast going on / off mid-stream */
static volatile uint32_t s_ext_base_ms;

static void ext_target_follow(void)
{
    if (!s_ext_on) return;
    uint32_t t = s_ext_base_ms + (auracast_state() != AURACAST_STATE_OFF ? 30u : 0u);
    if (t > A2DP_TARGET_MS) t = A2DP_TARGET_MS;
    if (t != s_target_ms) {
        ESP_LOGW(TAG, "%s buffer %lu -> %lu ms (broadcast %s)", s_ext_what, (unsigned long)s_target_ms,
                 (unsigned long)t, auracast_state() != AURACAST_STATE_OFF ? "on" : "off");
        s_target_ms = t;
    }
}

void a2dp_in_ext_start(a2dp_in_ext_decode_t decode, const char *what, uint32_t target_ms)
{
    s_ext_dec = decode;
    s_target_ms = target_ms < 30 ? 30 : target_ms > A2DP_TARGET_MS ? A2DP_TARGET_MS : target_ms;
    s_ext_base_ms = s_target_ms;
    if (auracast_state() != AURACAST_STATE_OFF && s_target_ms + 30u <= A2DP_TARGET_MS) s_target_ms += 30u;
    s_ext_what = what ? what : "";
    xQueueReset(s_pkt_q);
    s_last_arrival = 0;
    s_reset_dsp = true;
    s_ext_on = true;
    s_streaming = true;
    bt_sched_tune_le_input(true);
    ESP_LOGW(TAG, "audio source: %s (phone A2DP set aside), buffer %lu ms", s_ext_what, (unsigned long)s_target_ms);
}

void a2dp_in_ext_stop(void)
{
    if (!s_ext_on) return;
    s_ext_on = false;
    s_streaming = false;
    bt_sched_tune_le_input(false);
    s_target_ms = A2DP_TARGET_MS;
    xQueueReset(s_pkt_q);
    s_reset_dsp = true;
    s_flush_req = true;
    ESP_LOGW(TAG, "audio source: %s ended", s_ext_what);
}

bool a2dp_in_ext_active(void) { return s_ext_on; }

void a2dp_in_ext_push(const uint8_t *data, uint16_t len)
{
    if (!s_ext_on || len > PKT_MAX) return;
    static pkt_t p;
    p.len = len;
    if (len) memcpy(p.data, data, len);
    int64_t now = esp_timer_get_time();
    s_last_arrival = now;
    s_last_media_us = now;
    s_pkts++;
    s_frames++;
    if (xQueueSend(s_pkt_q, &p, 0) != pdTRUE) {
        s_pkt_drops++;
    }
}

void a2dp_in_ext_pcm(int16_t *lr, int frames, int rate)
{
    int64_t t0 = esp_timer_get_time();
    on_pcm(lr, frames, 2, rate, NULL);
    static uint32_t us_sum, us_max, n;
    uint32_t us = (uint32_t)(esp_timer_get_time() - t0);
    us_sum += us; n++;
    if (us > us_max) us_max = us;
    if (n == 1000) {
        ESP_LOGI(TAG, "ext PCM path (resample + ring): avg %lu us, max %lu us per %d frames",
                 (unsigned long)(us_sum / n), (unsigned long)us_max, frames);
        us_sum = us_max = n = 0;
    }
}
bool a2dp_in_connected(void) { return s_connected; }
static hci_con_handle_t s_a2dp_con = HCI_CON_HANDLE_INVALID;
uint16_t a2dp_in_con_handle(void) { return s_connected ? s_a2dp_con : HCI_CON_HANDLE_INVALID; }

/* ---- AVRCP / delay report (BT thread) ------------------------------------------ */
static uint16_t s_avrcp_cid;
static btstack_timer_source_t s_avrcp_timer, s_dr_timer;
static int s_dr_tries;

static void volume_store(void)
{
    nvs_handle_t h;
    if (nvs_open("s31", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "volume", s_volume);
        nvs_commit(h);
        nvs_close(h);
    }
}

uint8_t a2dp_in_volume(void) { return s_volume; }

static void volume_set(uint8_t v, const char *why)
{
    if (v > 127) v = 127;
    if (v == s_volume) return;
    s_volume = v;
    auracast_volume_changed();                /* party members follow (JBL block volume) */
    ESP_LOGW(TAG, "volume %u/127 (%d%%, %.1f dB) <- %s", v, v * 100 / 127,
             v ? 20.0 * log10((double)volume_gain(v)) : -999.0, why);
    volume_store();
    jbl_v5_notify(0x2D42);                    /* JBL One slider follows */
}

/* A2DP buffer + encoder pipeline (~5 SDUs) + BIG transport (~30 ms) + the
 * Auracast presentation delay (80 ms) */
#define DELAY_REPORT_MS (TARGET_MS + 50 + 30 + 80)

static void delay_report_cb(btstack_timer_source_t *ts)
{
    (void)ts;
    if (!s_connected) return;
    avdtp_stream_endpoint_t *ep = avdtp_get_stream_endpoint_for_seid(s_local_seid);
    avdtp_connection_t *c = avdtp_get_connection_for_avdtp_cid(s_a2dp_cid);
    bool dr = ep && (ep->remote_sep.configured_service_categories & (1 << AVDTP_DELAY_REPORTING));
    uint16_t d = (uint16_t)(DELAY_REPORT_MS * 10);
    uint8_t st = a2dp_sink_delay_report(s_a2dp_cid, s_local_seid, d);
    ESP_LOGW(TAG, "delay report %u ms (try %d) -> 0x%02x | ep state %d, phone configured DR %s, signaling %d/%d",
             DELAY_REPORT_MS, s_dr_tries + 1, st, ep ? (int)ep->state : -1, dr ? "yes" : "NO",
             c ? (int)c->state : -1, c ? (int)c->initiator_connection_state : -1);
    if (st == ERROR_CODE_COMMAND_DISALLOWED && dr && ++s_dr_tries < 5) {
        btstack_run_loop_set_timer(&s_dr_timer, 300);     /* signaling busy: try again */
        btstack_run_loop_add_timer(&s_dr_timer);
    }
}

static void delay_report_soon(uint32_t ms)
{
    s_dr_tries = 0;
    btstack_run_loop_remove_timer(&s_dr_timer);
    btstack_run_loop_set_timer_handler(&s_dr_timer, delay_report_cb);
    btstack_run_loop_set_timer(&s_dr_timer, ms);
    btstack_run_loop_add_timer(&s_dr_timer);
}

/* The phone normally opens AVRCP itself; if it has not after 0.4 s, we do. */
/* Our AVRCP connect can hang with no result event at all (seen with a REDMAGIC
 * 11 Pro): without AVRCP the phone keeps its volume to itself. Watch it: no
 * link 4 s after the attempt -> drop the pending one and try again (3 tries). */
#define AVRCP_TRIES      3
#define AVRCP_WATCH_MS   4000
static uint16_t s_avrcp_pending_cid;
static int s_avrcp_tries;
/* diagnosis: raw HCI packets for 5 s around our first two attempts per boot */
#define AVRCP_DUMP_ATTEMPTS 0               /* off: the dump floods the console and stalls the BT thread */
#define AVRCP_DUMP_MS       5000
static int s_avrcp_dumps;
/* Our AVRCP channel at security level 0 on odd tries: with a phone that never
 * encrypted the link (we accept its A2DP at level 0), a level-2 channel makes
 * BTstack authenticate and then ask for encryption, which the controller refuses
 * (0x12) - and BTstack waits for that encryption forever (seen in an HCI trace with
 * the REDMAGIC 11 Pro). The level is taken when AVRCP creates its channel, after
 * its SDP query, so it stays lowered until AVRCP reports or for 2 s. */
static btstack_timer_source_t s_level_timer;
static bool s_level_lowered;

static void avrcp_level_restore(btstack_timer_source_t *ts)
{
    (void)ts;
    if (s_level_lowered) {
        s_level_lowered = false;
        gap_set_security_level(LEVEL_2);
    }
}

static void avrcp_level_lower(void)
{
    s_level_lowered = true;
    gap_set_security_level(LEVEL_0);
    btstack_run_loop_remove_timer(&s_level_timer);
    btstack_run_loop_set_timer_handler(&s_level_timer, avrcp_level_restore);
    btstack_run_loop_set_timer(&s_level_timer, 2000);
    btstack_run_loop_add_timer(&s_level_timer);
}
static btstack_timer_source_t s_dump_timer;

static void dump_off(btstack_timer_source_t *ts)
{
    (void)ts;
    hci_dump_enable_packet_log(false);
    ESP_LOGW(TAG, "AVRCP diagnosis: HCI packet log OFF");
}

static void dump_window(void)
{
    if (s_avrcp_dumps >= AVRCP_DUMP_ATTEMPTS) return;
    s_avrcp_dumps++;
    ESP_LOGW(TAG, "AVRCP diagnosis: HCI packet log ON for %d ms (%d/%d)", AVRCP_DUMP_MS, s_avrcp_dumps,
             AVRCP_DUMP_ATTEMPTS);
    hci_dump_enable_packet_log(true);
    btstack_run_loop_remove_timer(&s_dump_timer);
    btstack_run_loop_set_timer_handler(&s_dump_timer, dump_off);
    btstack_run_loop_set_timer(&s_dump_timer, AVRCP_DUMP_MS);
    btstack_run_loop_add_timer(&s_dump_timer);
}

/* The S31 must be MASTER of the phone link (our scheduler tune assumes it). A phone
 * that connects while our own page is still running (power-up call-back) keeps the
 * S31 as SLAVE - the controller cannot switch then ("r_olc_acl_rsw_sched sched
 * failed") - and audio then barely gets through. Ask again once things settle. */
#define ROLE_TRIES 4
static btstack_timer_source_t s_role_timer;
static int s_role_tries;
static hci_con_handle_t s_role_con = HCI_CON_HANDLE_INVALID;

static void role_retry(btstack_timer_source_t *ts)
{
    if (!s_connected || s_role_con == HCI_CON_HANDLE_INVALID || gap_get_role(s_role_con) == HCI_ROLE_MASTER) {
        return;
    }
    if (s_role_tries >= ROLE_TRIES) {
        ESP_LOGE(TAG, "role: still SLAVE after %d requests - phone audio may stutter", ROLE_TRIES);
        return;
    }
    s_role_tries++;
    if (hci_can_send_command_packet_now()) {          /* our policy forbids switches: allow it on this link */
        hci_send_cmd(&hci_write_link_policy_settings, s_role_con, 0x0001);
    }
    uint8_t st = gap_request_role(s_peer, HCI_ROLE_MASTER);
    ESP_LOGW(TAG, "role: S31 is SLAVE - asking for MASTER (try %d/%d) -> 0x%02x", s_role_tries, ROLE_TRIES, st);
    btstack_run_loop_set_timer_handler(ts, role_retry);
    btstack_run_loop_set_timer(ts, 2000);
    btstack_run_loop_add_timer(ts);
}

static void role_check_soon(hci_con_handle_t h)
{
    s_role_con = h;
    s_role_tries = 0;
    btstack_run_loop_remove_timer(&s_role_timer);
    btstack_run_loop_set_timer_handler(&s_role_timer, role_retry);
    btstack_run_loop_set_timer(&s_role_timer, 1000);
    btstack_run_loop_add_timer(&s_role_timer);
}

static void avrcp_connect_cb(btstack_timer_source_t *ts)
{
    if (!s_connected || s_avrcp_cid != 0) {
        s_avrcp_tries = 0;
        return;
    }
    if (s_avrcp_pending_cid != 0) {               /* the last attempt never finished */
        ESP_LOGW(TAG, "AVRCP: attempt %d gave no result in %d ms - dropping it, the phone gets %d ms to open it",
                 s_avrcp_tries, AVRCP_WATCH_MS, AVRCP_WATCH_MS);
        avrcp_disconnect(s_avrcp_pending_cid);
        s_avrcp_pending_cid = 0;
        /* our pending request may be what keeps the phone from opening its own */
        btstack_run_loop_set_timer_handler(ts, avrcp_connect_cb);
        btstack_run_loop_set_timer(ts, AVRCP_WATCH_MS);
        btstack_run_loop_add_timer(ts);
        return;
    }
    if (s_avrcp_tries >= AVRCP_TRIES) {
        ESP_LOGE(TAG, "AVRCP: no link after %d tries - phone volume will not reach the speaker", AVRCP_TRIES);
        s_avrcp_tries = 0;
        return;
    }
    uint16_t cid = 0;
    dump_window();
    bool open = (s_avrcp_tries % 2) == 0;          /* tries 1, 3: no security demanded; 2: level 2 */
    if (open) avrcp_level_lower();
    uint8_t st = avrcp_connect(s_peer, &cid);
    s_avrcp_tries++;
    s_avrcp_pending_cid = st == ERROR_CODE_SUCCESS ? cid : 0;
    ESP_LOGW(TAG, "AVRCP: phone did not connect - connecting ourselves (try %d/%d, %s) -> 0x%02x", s_avrcp_tries,
             AVRCP_TRIES, open ? "security level 0" : "security level 2", st);
    btstack_run_loop_set_timer_handler(ts, avrcp_connect_cb);
    btstack_run_loop_set_timer(ts, AVRCP_WATCH_MS);
    btstack_run_loop_add_timer(ts);
}

static bool s_play_after_connect;

void a2dp_in_avrcp_play_after_switch(void)
{
    if (s_avrcp_cid != 0) {
        uint8_t st = avrcp_controller_play(s_avrcp_cid);
        ESP_LOGW(TAG, "AVRCP Play to the phone (now on LE Audio) -> 0x%02x", st);
        return;
    }
    bd_addr_t none = { 0 };
    if (memcmp(s_peer, none, 6) == 0) return;
    uint16_t cid = 0;
    uint8_t st = avrcp_connect(s_peer, &cid);
    s_play_after_connect = st == ERROR_CODE_SUCCESS;
    ESP_LOGW(TAG, "AVRCP Play for the phone on LE Audio: opening AVRCP to %s first -> 0x%02x",
             bd_addr_to_str(s_peer), st);
}

/* ---- buttons (posted to the BT thread) ---------------------------------------- */
static volatile uint32_t s_btn_pending;          /* bit per a2dp_btn_t */
static btstack_context_callback_registration_t s_btn_reg;
static btstack_timer_source_t s_pair_timer;
static bool s_pairing;
static bool s_new_bond;                          /* a phone bonded in this pairing mode */
static btstack_timer_source_t s_pair_call_timer;
static bd_addr_t s_pair_call_addr;
static uint8_t s_pair_call_tries;
static bool s_pair_call_active;                  /* our A2DP connect after an LE pairing is in flight */
static btstack_timer_source_t s_drop_all_timer;
static void drop_all_phone_links(btstack_timer_source_t *ts);
static bool s_pair_incoming;                     /* the phone's own classic connect reached us */

bool a2dp_in_pairing(void) { return s_pairing; }

void a2dp_in_refresh_discoverable(void)
{
    bool linked = link_tune_classic_links() > 0 || le_audio_phone_linked();
    /* only in pairing mode (button, JBL One LE Audio on, or a speaker with no phone
     * paired yet); bonded phones reconnect by page scan / the LE Audio advert */
    bool disc = !S31_QUIET_RADIO_TEST && s_pairing;
    static int last = -1;
    gap_discoverable_control(disc);
    le_audio_set_discoverable(disc);
    /* page scan: wide while nothing is linked (fast reconnect); short while an LE Audio
     * phone is linked - 22.5 ms windows ate its connection events and its CIS setup
     * took 2 s - still enough for it to page back for A2DP */
    static int last_ps = -1;
    int ps = le_audio_phone_linked() ? 1 : 0;
    if (ps != last_ps) {
        last_ps = ps;
        if (ps) gap_set_page_scan_activity(0x0200, 0x0012);
        else gap_set_page_scan_activity(0x0100, 0x0024);
    }
    if (last != (int)disc) {
        ESP_LOGW(TAG, "discoverable %s (%s)", disc ? "ON" : "off",
                 s_pairing ? "pairing mode" : linked ? "a phone is linked" : "not in pairing mode");
        last = disc;
    }
}

static void pairing_stop(const char *why)
{
    btstack_run_loop_remove_timer(&s_pair_timer);
    s_pairing = false;
    s_new_bond = false;
    a2dp_in_refresh_discoverable();
    gap_connectable_control(!S31_QUIET_RADIO_TEST && !s_streaming);
    ESP_LOGW(TAG, "pairing mode ended (%s)", why);
}

static void pairing_end(btstack_timer_source_t *ts)
{
    (void)ts;
    /* a phone that is still pairing / connecting keeps it open: until it is done */
    if (s_new_bond || link_tune_classic_links() > 0 || le_audio_phone_linked()) {
        static uint8_t ext;
        if (++ext <= 6) {
            ESP_LOGW(TAG, "pairing mode: a phone is still connecting - 30 s more");
            btstack_run_loop_set_timer(&s_pair_timer, 30000);
            btstack_run_loop_add_timer(&s_pair_timer);
            return;
        }
        ext = 0;
    }
    pairing_stop("3 min");
}

void a2dp_in_pairing_done(const char *why)
{
    if (!s_pairing || !s_new_bond) return;        /* a bonded phone reconnecting: keep pairing */
    pairing_stop(why);
}

static void btn_run(void *ctx)
{
    (void)ctx;
    uint32_t b = __atomic_exchange_n(&s_btn_pending, 0, __ATOMIC_ACQ_REL);
    if ((b & (1u << A2DP_BTN_VOL_UP | 1u << A2DP_BTN_VOL_DOWN)) &&
        party_listen_volume_step(((b & (1u << A2DP_BTN_VOL_UP)) ? 8 : 0) - ((b & (1u << A2DP_BTN_VOL_DOWN)) ? 8 : 0))) {
        b &= ~(1u << A2DP_BTN_VOL_UP | 1u << A2DP_BTN_VOL_DOWN);   /* listening: our output, not the phone */
    }
    if (b & (1u << A2DP_BTN_VOL_UP | 1u << A2DP_BTN_VOL_DOWN)) {
        int v = s_volume + ((b & (1u << A2DP_BTN_VOL_UP)) ? 8 : 0) - ((b & (1u << A2DP_BTN_VOL_DOWN)) ? 8 : 0);
        uint8_t was = s_volume;
        volume_set((uint8_t)(v < 0 ? 0 : v > 127 ? 127 : v), "button");
        if (s_avrcp_cid) avrcp_target_volume_changed(s_avrcp_cid, s_volume);   /* phone slider follows */
        static uint32_t last_limit_ms;
        uint32_t now_ms = btstack_run_loop_get_time_ms();
        if ((s_volume == 127 || s_volume == 0) && (was != s_volume || now_ms - last_limit_ms > 800)) {
            last_limit_ms = now_ms;
            tones_play("vol_max");                /* at the end stop, as the Xtreme 5 */
        }
    }
    if (b & (1u << A2DP_BTN_PLAY_PAUSE)) {
        if (s_avrcp_cid == 0) {
            ESP_LOGW(TAG, "play/pause: no AVRCP link");
        } else {
            uint8_t st = s_streaming ? avrcp_controller_pause(s_avrcp_cid) : avrcp_controller_play(s_avrcp_cid);
            ESP_LOGW(TAG, "play/pause button -> %s (0x%02x)", s_streaming ? "PAUSE" : "PLAY", st);
        }
    }
    if (b & (1u << A2DP_BTN_PAIRING)) {
        /* as the Xtreme 5: pairing (button, or LE Audio switched in JBL One) starts clean -
         * every phone link goes, the phone pairs / connects again from scratch */
        btstack_run_loop_remove_timer(&s_drop_all_timer);
        btstack_run_loop_set_timer_handler(&s_drop_all_timer, drop_all_phone_links);
        btstack_run_loop_set_timer(&s_drop_all_timer, 400);  /* the app's reply + the tone first */
        btstack_run_loop_add_timer(&s_drop_all_timer);
        s_pairing = true;
        a2dp_in_refresh_discoverable();
        gap_connectable_control(1);
        btstack_run_loop_remove_timer(&s_pair_timer);
        btstack_run_loop_set_timer_handler(&s_pair_timer, pairing_end);
        btstack_run_loop_set_timer(&s_pair_timer, 180000);
        btstack_run_loop_add_timer(&s_pair_timer);
        ESP_LOGW(TAG, "pairing mode: discoverable + connectable for 3 min");
        tones_play("bt_pair");
    }
}

/* ---- JBL One: volume / mute / transport / track info (BT thread) ---------------- */
static volatile uint32_t s_cnt_tone_ms;            /* A2DP "connected" tone */
static bool s_conn_toned;                        /* played for this A2DP connection */
static bool s_play_at_release;                   /* A2DP was playing when it went away */
static uint32_t s_stream_stop_ms;                /* the A2DP stream last stopped / suspended */
static btstack_timer_source_t s_drop_timer;
static hci_con_handle_t s_drop_con = HCI_CON_HANDLE_INVALID;

static void drop_bare_link(btstack_timer_source_t *ts)
{
    (void)ts;
    hci_con_handle_t h = s_drop_con;
    s_drop_con = HCI_CON_HANDLE_INVALID;
    if (h == HCI_CON_HANDLE_INVALID || s_connected) return;            /* A2DP is back */
    if (gap_get_connection_type(h) != GAP_CONNECTION_ACL) return;        /* gone already */
    if (jbl_v5_is_link(h)) return;                                       /* JBL One on it */
    ESP_LOGW(TAG, "A2DP gone, classic link 0x%04x left bare: closing it (the phone goes LE Audio)", h);
    gap_disconnect(h);
}
static uint32_t s_release_ms;

uint32_t a2dp_in_ms_since_connect_tone(void)
{
    return s_cnt_tone_ms ? btstack_run_loop_get_time_ms() - s_cnt_tone_ms : 0;
}

bool a2dp_in_was_playing(uint32_t within_ms)
{
    return s_play_at_release && btstack_run_loop_get_time_ms() - s_release_ms < within_ms;
}

static uint8_t s_play_status = 0xFE;             /* AVRCP_PLAYBACK_STATUS_*, 0xFE = not reported */
static char s_title[64], s_artist[64];

void a2dp_in_set_volume(uint8_t v, const char *why)
{
    volume_set(v > 127 ? 127 : v, why);
    if (s_avrcp_cid) avrcp_target_volume_changed(s_avrcp_cid, s_volume);   /* phone slider follows */
}

void a2dp_in_set_mute(bool mute)
{
    if (mute == s_muted) return;
    s_muted = mute;
    ESP_LOGW(TAG, "speaker %s", mute ? "MUTED" : "unmuted");
    jbl_v5_notify(0x2D41);
}

bool a2dp_in_muted(void) { return s_muted; }

bool a2dp_in_media_op(uint8_t op)
{
    if (op == 5 || op == 6) {
        a2dp_in_button(op == 5 ? A2DP_BTN_VOL_UP : A2DP_BTN_VOL_DOWN);
        return true;
    }
    if (s_avrcp_cid == 0) return false;
    switch (op) {
    case 0: avrcp_controller_stop(s_avrcp_cid); break;
    case 1: avrcp_controller_pause(s_avrcp_cid); break;
    case 2: avrcp_controller_play(s_avrcp_cid); break;
    case 3: avrcp_controller_backward(s_avrcp_cid); break;
    case 4: avrcp_controller_forward(s_avrcp_cid); break;
    default: return false;
    }
    return true;
}

uint8_t a2dp_in_play_status(void)
{
    if (s_play_status != 0xFE) return s_play_status;
    return s_streaming ? AVRCP_PLAYBACK_STATUS_PLAYING
                       : (s_connected ? AVRCP_PLAYBACK_STATUS_PAUSED : AVRCP_PLAYBACK_STATUS_STOPPED);
}

const char *a2dp_in_track_title(void) { return s_title; }
const char *a2dp_in_track_artist(void) { return s_artist; }

static void meta_copy(char *dst, const uint8_t *v, uint8_t n)
{
    if (n > 63) n = 63;
    memcpy(dst, v, n);
    dst[n] = 0;
}

void a2dp_in_button(a2dp_btn_t btn)
{
    uint32_t was = __atomic_fetch_or(&s_btn_pending, 1u << btn, __ATOMIC_ACQ_REL);
    if (was == 0) {                              /* one post at a time: the registration is a list node */
        s_btn_reg.callback = &btn_run;
        btstack_run_loop_execute_on_main_thread(&s_btn_reg);
    }
}

void a2dp_in_log_stats(void)
{
    static uint32_t last_pkts, last_frames;
    uint32_t pk = s_pkts, fr = s_frames;
    uint32_t dp = pk - last_pkts;
    ESP_LOGW(TAG, "A2DP 2s: %s %s, media pkts %lu (+%lu, %lu.%lu frames/pkt, max %u B), q drops %lu, seq gaps %lu | bitpool %lu (min %lu max %lu) | "
             "PCM %lu ms (avg %lu, min %lu, max %lu), aligns %lu, ceilings %lu, ACL boosts %lu, underruns %lu, oldest dropped %lu, clock corr %ld ppm | catch-ups %lu (%lu ms skipped, %lu expired) | volume %u/127%s | gaps max %lu ms, >60 %lu >100 %lu >200 %lu",
             s_streaming ? "STREAMING" : s_connected ? "connected" : "no phone",
             s_ext_on ? "LE Audio LC3" : s_codec == CODEC_AAC ? "AAC" : "SBC",
             (unsigned long)pk, (unsigned long)dp,
             (unsigned long)(dp ? (fr - last_frames) / dp : 0), (unsigned long)(dp ? ((fr - last_frames) * 10 / dp) % 10 : 0),
             (unsigned)s_pkt_max, (unsigned long)s_pkt_drops,
             (unsigned long)s_seq_gaps, (unsigned long)s_bitpool,
             (unsigned long)(s_bp_min == 0xff ? 0 : s_bp_min), (unsigned long)s_bp_max,
             (unsigned long)(ring_fill(&s_ring) / (OUT_RATE / 1000)),
             (unsigned long)(s_rd_fill_ema > 0 ? (uint32_t)s_rd_fill_ema / (OUT_RATE / 1000) : 0),
             (unsigned long)(s_win_min == 0xffffffffu ? 0 : s_win_min / (OUT_RATE / 1000)),
             (unsigned long)(s_win_max / (OUT_RATE / 1000)), (unsigned long)s_aligns, (unsigned long)s_ceilings, (unsigned long)s_acl_boosts,
             (unsigned long)s_underruns,
             (unsigned long)s_ring_drops, (long)s_corr_ppm,
             (unsigned long)s_catchups, (unsigned long)(s_catchup_frames / (OUT_RATE / 1000)),
             (unsigned long)s_deficit_expired, s_volume, s_avrcp_cid ? "" : " (no AVRCP)",
             (unsigned long)(s_gap_max_us / 1000), (unsigned long)s_gap_n60, (unsigned long)s_gap_n100,
             (unsigned long)s_gap_n200);
    s_gap_max_us = 0;
    s_win_min = 0xffffffffu;
    s_win_max = 0;
    last_pkts = pk;
    last_frames = fr;
    s_pkt_max = 0;
    s_bp_min = 0xff;
    s_bp_max = 0;
}

/* ---- BTstack events (BT thread) --------------------------------------------- */
static void reconnect_failed(void);
static void reconn_abort_on_pairing(const uint8_t *addr);

/* the phone's pending Classic connect request (see HCI_EVENT_CONNECTION_REQUEST) */
static bd_addr_t s_conn_req_addr;
static bool s_conn_req_pending;
static int s_accept_retries;
static btstack_timer_source_t s_accept_timer;

static void accept_retry(btstack_timer_source_t *ts)
{
    (void)ts;
    if (!s_conn_req_pending) return;
    if (!hci_can_send_command_packet_now()) {
        btstack_run_loop_set_timer(&s_accept_timer, 5);
        btstack_run_loop_add_timer(&s_accept_timer);
        return;
    }
    s_accept_retries++;
    ESP_LOGW(TAG, "classic connect from %s: accepting again without a role switch (try %d)",
             bd_addr_to_str(s_conn_req_addr), s_accept_retries);
    hci_send_cmd(&hci_accept_connection_request, s_conn_req_addr, 0x01);   /* remain peripheral */
}
static void reconnect_succeeded(void);

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
            reconnect_failed();
            if (s_pair_call_active) {                 /* after an LE pairing: try again shortly */
                s_pair_call_active = false;
                btstack_run_loop_remove_timer(&s_pair_call_timer);
                btstack_run_loop_set_timer(&s_pair_call_timer, 1500);
                btstack_run_loop_add_timer(&s_pair_call_timer);
            }
            break;
        }
        s_pair_call_active = false;
        reconnect_succeeded();
        s_conn_ms = btstack_run_loop_get_time_ms();
        s_media_first_ms = 0;
        s_a2dp_cid = a2dp_subevent_signaling_connection_established_get_a2dp_cid(packet);
        s_connected = true;
        link_tune_refresh("A2DP connected");
        {   /* remember the phone: called back at power-up */
            nvs_handle_t h;
            if (nvs_open("s31", NVS_READWRITE, &h) == ESP_OK) {
                nvs_set_blob(h, "last_phone", s_peer, 6);
                nvs_commit(h);
                nvs_close(h);
            }
        }
        hci_con_handle_t h = a2dp_subevent_signaling_connection_established_get_con_handle(packet);
        s_a2dp_con = h;
        hci_role_t role = gap_get_role(h);
        s_conn_toned = false;                     /* tone once the stream channel is open */
        ESP_LOGW(TAG, "A2DP connected %s, role %s", bd_addr_to_str(s_peer),
                 role == HCI_ROLE_MASTER ? "MASTER (S31 schedules the phone)" : "SLAVE");
        if (role != HCI_ROLE_MASTER) {
            role_check_soon(h);
        }
        btstack_run_loop_remove_timer(&s_avrcp_timer);
        s_avrcp_tries = 0;
        s_avrcp_pending_cid = 0;
        btstack_run_loop_set_timer_handler(&s_avrcp_timer, avrcp_connect_cb);
        btstack_run_loop_set_timer(&s_avrcp_timer, 400);    /* the phone holds its stream until AVRCP is up: 2 s here = 2 s reconnect delay */
        btstack_run_loop_add_timer(&s_avrcp_timer);
        break;
    }
    case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_MPEG_AAC_CAPABILITY: {
        /* We opened the stream (power-up call-back): BTstack would pick SBC on its
         * own - choose AAC-LC when the phone has it. Bitmaps as BTstack reports them:
         * object type b6 MPEG-2 LC, b5 MPEG-4 LC; rate 0x010 44.1 kHz, 0x008 48 kHz;
         * channels 0x4 two, 0x8 one. */
        uint8_t ot = a2dp_subevent_signaling_media_codec_mpeg_aac_capability_get_object_type_bitmap(packet);
        uint16_t sf = a2dp_subevent_signaling_media_codec_mpeg_aac_capability_get_sampling_frequency_bitmap(packet);
        uint8_t ch = a2dp_subevent_signaling_media_codec_mpeg_aac_capability_get_channels_bitmap(packet);
        if (s_aac_seid == 0 || !(ot & 0x60) || !(sf & 0x018)) break;
        avdtp_configuration_mpeg_aac_t c;
        memset(&c, 0, sizeof(c));
        c.object_type = (ot & 0x40) ? AVDTP_AAC_MPEG2_LC : AVDTP_AAC_MPEG4_LC;
        c.sampling_frequency = (sf & 0x010) ? 44100 : 48000;
        c.channels = (ch & 0x4) ? 2 : 1;
        uint32_t br = a2dp_subevent_signaling_media_codec_mpeg_aac_capability_get_bit_rate(packet);
        c.bit_rate = (br == 0 || br > 320000) ? 320000 : br;
        c.vbr = a2dp_subevent_signaling_media_codec_mpeg_aac_capability_get_vbr(packet);
        c.drc = false;
        uint8_t st = a2dp_sink_set_config_mpeg_aac(a2dp_subevent_signaling_media_codec_mpeg_aac_capability_get_a2dp_cid(packet),
                                                   s_aac_seid,
                                                   a2dp_subevent_signaling_media_codec_mpeg_aac_capability_get_remote_seid(packet), &c);
        ESP_LOGW(TAG, "phone offers AAC: choosing it (%lu Hz, %u ch) -> 0x%02x", (unsigned long)c.sampling_frequency, c.channels, st);
        break;
    }
    case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_MPEG_AAC_CONFIGURATION: {
        uint32_t sr = a2dp_subevent_signaling_media_codec_mpeg_aac_configuration_get_sampling_frequency(packet);
        s_codec = CODEC_AAC;
        s_local_seid = a2dp_subevent_signaling_media_codec_mpeg_aac_configuration_get_local_seid(packet);
        s_cfg_rate = sr;
        s_reset_dsp = true;
        ESP_LOGW(TAG, "AAC configured: object type %u, %lu Hz, %u ch, %lu bit/s%s",
                 a2dp_subevent_signaling_media_codec_mpeg_aac_configuration_get_object_type(packet), (unsigned long)sr,
                 a2dp_subevent_signaling_media_codec_mpeg_aac_configuration_get_num_channels(packet),
                 (unsigned long)a2dp_subevent_signaling_media_codec_mpeg_aac_configuration_get_bit_rate(packet),
                 a2dp_subevent_signaling_media_codec_mpeg_aac_configuration_get_vbr(packet) ? " VBR" : "");
        break;
    }
    case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION: {
        uint16_t sr = a2dp_subevent_signaling_media_codec_sbc_configuration_get_sampling_frequency(packet);
        s_codec = CODEC_SBC;
        s_local_seid = a2dp_subevent_signaling_media_codec_sbc_configuration_get_local_seid(packet);
        s_cfg_rate = sr;
        ESP_LOGW(TAG, "SBC configured: %u Hz, %u ch, block %u, subbands %u, bitpool %u-%u", sr,
                 a2dp_subevent_signaling_media_codec_sbc_configuration_get_num_channels(packet),
                 a2dp_subevent_signaling_media_codec_sbc_configuration_get_block_length(packet),
                 a2dp_subevent_signaling_media_codec_sbc_configuration_get_subbands(packet),
                 a2dp_subevent_signaling_media_codec_sbc_configuration_get_min_bitpool_value(packet),
                 a2dp_subevent_signaling_media_codec_sbc_configuration_get_max_bitpool_value(packet));
        break;
    }
    case A2DP_SUBEVENT_STREAM_ESTABLISHED:
        /* AVDTP 1.3: the sink reports its delay once the stream is configured */
        ESP_LOGW(TAG, "A2DP stream established, status 0x%02x", a2dp_subevent_stream_established_get_status(packet));
        if (a2dp_subevent_stream_established_get_status(packet) == ERROR_CODE_SUCCESS) {
            delay_report_soon(50);
            a2dp_in_pairing_done("phone connected");
            if (!s_conn_toned) {
                /* the phone shows "Connected" when the media channel opens (Android A2DP
                 * CONNECTED), not at the signaling channel 1-2 s earlier */
                s_conn_toned = true;
                s_cnt_tone_ms = btstack_run_loop_get_time_ms();
                tones_play("bt_cnt");
            }
        }
        break;
    case A2DP_SUBEVENT_SIGNALING_DELAY_REPORTING_CAPABILITY:
        ESP_LOGW(TAG, "phone supports delay reporting");
        break;
    case A2DP_SUBEVENT_STREAM_STARTED:
        s_start_ms = btstack_run_loop_get_time_ms();
        s_media_first_ms = 0;
        s_last_arrival = 0;
        s_reset_dsp = true;
        s_streaming = true;
        gap_connectable_control(s_pairing);       /* no page scan while streaming (airtime for the BIS), unless pairing */
        bt_sched_tune_streaming(true);            /* streaming profile (MASTER: same as idle, BIS copies above the ACL) */
        delay_report_soon(100);                   /* again while streaming (allowed in STREAMING too) */
        ESP_LOGW(TAG, "A2DP audio STARTED");
        break;
    case A2DP_SUBEVENT_STREAM_SUSPENDED:
    case A2DP_SUBEVENT_STREAM_STOPPED:
        if (s_streaming) s_stream_stop_ms = btstack_run_loop_get_time_ms();
        s_streaming = false;
        bt_sched_tune_streaming(false);
        gap_connectable_control(!S31_QUIET_RADIO_TEST);
        ESP_LOGW(TAG, "A2DP audio %s", packet[2] == A2DP_SUBEVENT_STREAM_SUSPENDED ? "suspended" : "stopped");
        break;
    case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
        s_release_ms = btstack_run_loop_get_time_ms();
        /* a phone switching to LE Audio pauses the stream itself ~0.7 s before it drops A2DP */
        s_play_at_release = s_streaming || s_play_status == AVRCP_PLAYBACK_STATUS_PLAYING ||
                            (s_stream_stop_ms != 0 && s_release_ms - s_stream_stop_ms < 3000);
        if (le_audio_enabled() && s_a2dp_con != HCI_CON_HANDLE_INVALID) {
            /* the phone keeps the bare classic link ~4 s while it connects LE Audio; meanwhile
             * its first LE connect fails (0x3E). Close the link (unless JBL One uses it). */
            s_drop_con = s_a2dp_con;
            btstack_run_loop_remove_timer(&s_drop_timer);
            btstack_run_loop_set_timer_handler(&s_drop_timer, drop_bare_link);
            btstack_run_loop_set_timer(&s_drop_timer, 300);
            btstack_run_loop_add_timer(&s_drop_timer);
        }
        s_streaming = false;
        s_connected = false;
        s_a2dp_con = HCI_CON_HANDLE_INVALID;
        bt_sched_tune_streaming(false);
        link_tune_refresh("A2DP gone, link kept");
        gap_connectable_control(!S31_QUIET_RADIO_TEST);
        ESP_LOGW(TAG, "A2DP disconnected");
        break;
    default:
        break;
    }
}

/* Encryption on our own outgoing link: after authenticating, BTstack sends HCI Set
 * Connection Encryption once and does not handle the controller refusing it (seen:
 * 0x12) - the link then waits forever and the phone hangs up (REDMAGIC 11 Pro
 * call-back). Refusals are retried here a few times. */
#define ENC_RETRIES     6
#define ENC_RETRY_MS    150
static hci_con_handle_t s_enc_con = HCI_CON_HANDLE_INVALID;
static int s_enc_tries;
static btstack_timer_source_t s_enc_timer;

static void enc_retry(btstack_timer_source_t *ts)
{
    (void)ts;
    if (s_enc_con == HCI_CON_HANDLE_INVALID) return;
    if (!hci_can_send_command_packet_now()) {
        btstack_run_loop_set_timer(&s_enc_timer, 20);
        btstack_run_loop_add_timer(&s_enc_timer);
        return;
    }
    s_enc_tries++;
    ESP_LOGW(TAG, "encryption: asking again on con 0x%04x (try %d/%d)", s_enc_con, s_enc_tries, ENC_RETRIES);
    hci_send_cmd(&hci_set_connection_encryption, s_enc_con, 1);
}

static void hci_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }
    switch (hci_event_packet_get_type(packet)) {
    case HCI_EVENT_CONNECTION_REQUEST: {
        bd_addr_t a;
        hci_event_connection_request_get_bd_addr(packet, a);
        ESP_LOGW(TAG, "classic connect request from %s", bd_addr_to_str(a));
        if (bd_addr_cmp(a, s_pair_call_addr) == 0) s_pair_incoming = true;
        bd_addr_copy(s_conn_req_addr, a);
        s_conn_req_pending = true;
        s_accept_retries = 0;
        break;
    }
    case HCI_EVENT_CONNECTION_COMPLETE:
        s_conn_req_pending = false;
        btstack_run_loop_remove_timer(&s_accept_timer);
        break;
    case HCI_EVENT_COMMAND_COMPLETE:
        if (hci_event_command_complete_get_command_opcode(packet) == 0x1408 && size >= 9) {   /* Read Encryption Key Size */
            ESP_LOGW(TAG, "encryption key size: status 0x%02x, con 0x%04x, %u bytes", packet[5],
                     little_endian_read_16(packet, 6), packet[8]);
        }
        break;
    case HCI_EVENT_ENCRYPTION_CHANGE_V2:
        ESP_LOGW(TAG, "encryption change v2: status 0x%02x, con 0x%04x, enabled %u, key size %u", packet[2],
                 little_endian_read_16(packet, 3), packet[5], size > 6 ? packet[6] : 0);
        if (little_endian_read_16(packet, 3) == s_enc_con) {
            s_enc_con = HCI_CON_HANDLE_INVALID;
            btstack_run_loop_remove_timer(&s_enc_timer);
        }
        break;
    case HCI_EVENT_COMMAND_STATUS:
        if (hci_event_command_status_get_status(packet) != ERROR_CODE_SUCCESS &&
            hci_event_command_status_get_command_opcode(packet) != 0x0413) {
            ESP_LOGW(TAG, "HCI command 0x%04x refused: status 0x%02x", hci_event_command_status_get_command_opcode(packet),
                     hci_event_command_status_get_status(packet));
            /* Accept Connection Request (as master, with a role switch) refused: right after
             * the phone's LE pairing the controller can disallow it (0x0C) and the phone's
             * Classic link never came up. Accept again staying peripheral; the role is
             * switched to master once the link is up (s_role_timer). */
            if (hci_event_command_status_get_command_opcode(packet) == 0x0409 && s_conn_req_pending &&
                s_accept_retries < 3) {
                btstack_run_loop_remove_timer(&s_accept_timer);
                btstack_run_loop_set_timer_handler(&s_accept_timer, accept_retry);
                btstack_run_loop_set_timer(&s_accept_timer, s_accept_retries == 0 ? 10 : 200);
                btstack_run_loop_add_timer(&s_accept_timer);
            }
        }
        if (hci_event_command_status_get_command_opcode(packet) == 0x0413) {      /* Set Connection Encryption */
            uint8_t st = hci_event_command_status_get_status(packet);
            ESP_LOGW(TAG, "encryption request: controller status 0x%02x", st);
            if (st != ERROR_CODE_SUCCESS && s_enc_con != HCI_CON_HANDLE_INVALID && s_enc_tries < ENC_RETRIES) {
                btstack_run_loop_remove_timer(&s_enc_timer);
                btstack_run_loop_set_timer_handler(&s_enc_timer, enc_retry);
                btstack_run_loop_set_timer(&s_enc_timer, ENC_RETRY_MS);
                btstack_run_loop_add_timer(&s_enc_timer);
            }
        }
        break;
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
        if (hci_event_authentication_complete_get_status(packet) == 0) {
            s_enc_con = hci_event_authentication_complete_get_connection_handle(packet);   /* BTstack asks for encryption next */
            s_enc_tries = 0;
        }
        ESP_LOGW(TAG, "authentication status 0x%02x", hci_event_authentication_complete_get_status(packet));
        break;
    case HCI_EVENT_ENCRYPTION_CHANGE:
        if (hci_event_encryption_change_get_connection_handle(packet) == s_enc_con) {
            s_enc_con = HCI_CON_HANDLE_INVALID;
            btstack_run_loop_remove_timer(&s_enc_timer);
        }
        /* enabled: 1 = E0, 2 = AES-CCM (Secure Connections) */
        ESP_LOGW(TAG, "encryption change: status 0x%02x, con 0x%04x, enabled %u",
                 hci_event_encryption_change_get_status(packet),
                 hci_event_encryption_change_get_connection_handle(packet),
                 hci_event_encryption_change_get_encryption_enabled(packet));
        break;
    case HCI_EVENT_LINK_KEY_REQUEST:
        ESP_LOGW(TAG, "phone asks for our link key");
        break;
    case HCI_EVENT_LINK_KEY_NOTIFICATION:
        ESP_LOGW(TAG, "new link key, type %u", packet[2 + 6 + 16]);   /* event: bd_addr(6) key(16) type(1) */
        s_new_bond = true;                        /* a new classic bond (pairing mode ends on its connect) */
        break;
    case HCI_EVENT_IO_CAPABILITY_REQUEST: {
        ESP_LOGW(TAG, "phone starts (re)pairing (IO capability request)");
        bd_addr_t a;
        hci_event_io_capability_request_get_bd_addr(packet, a);
        reconn_abort_on_pairing(a);
        break;
    }
    case GAP_EVENT_PAIRING_COMPLETE:
        ESP_LOGW(TAG, "GAP pairing complete, status 0x%02x", gap_event_pairing_complete_get_status(packet));
        break;
    case HCI_EVENT_DISCONNECTION_COMPLETE:
        if (hci_event_disconnection_complete_get_connection_handle(packet) == s_enc_con) {
            s_enc_con = HCI_CON_HANDLE_INVALID;
            btstack_run_loop_remove_timer(&s_enc_timer);
        }
        ESP_LOGW(TAG, "ACL disconnected, reason 0x%02x",
                 hci_event_disconnection_complete_get_reason(packet));
        break;
    case HCI_EVENT_ROLE_CHANGE:
        ESP_LOGW(TAG, "role change: now %s (status 0x%02x)", hci_event_role_change_get_role(packet) == 0 ? "MASTER" : "SLAVE",
                 hci_event_role_change_get_status(packet));
        if (hci_event_role_change_get_role(packet) == 0 && s_role_con != HCI_CON_HANDLE_INVALID &&
            hci_can_send_command_packet_now()) {
            hci_send_cmd(&hci_write_link_policy_settings, s_role_con, 0x0000);   /* no switches again */
            s_role_con = HCI_CON_HANDLE_INVALID;
        }
        break;
    default:
        break;
    }
}

static void avrcp_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET || hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) {
        return;
    }
    switch (packet[2]) {
    case AVRCP_SUBEVENT_CONNECTION_ESTABLISHED: {
        uint8_t st = avrcp_subevent_connection_established_get_status(packet);
        s_avrcp_pending_cid = 0;
        avrcp_level_restore(NULL);
        if (st != ERROR_CODE_SUCCESS) {
            ESP_LOGW(TAG, "AVRCP connect failed 0x%02x", st);
            s_play_after_connect = false;
            break;
        }
        if (s_play_after_connect) {
            s_play_after_connect = false;
            uint8_t ps = avrcp_controller_play(avrcp_subevent_connection_established_get_avrcp_cid(packet));
            ESP_LOGW(TAG, "AVRCP open: Play to the phone (now on LE Audio) -> 0x%02x", ps);
        }
        s_avrcp_tries = 0;
        btstack_run_loop_remove_timer(&s_avrcp_timer);
        if (s_avrcp_cid == avrcp_subevent_connection_established_get_avrcp_cid(packet)) {
            break;                                       /* same event via the CT/TG handlers */
        }
        s_avrcp_cid = avrcp_subevent_connection_established_get_avrcp_cid(packet);
        /* absolute volume: the phone's slider drives our gain, and we tell it ours -
         * capped so a reconnect never jumps the phone (and the party) to full */
        if (s_volume > 76) s_volume = 76;            /* ~60 % */
        avrcp_target_support_event(s_avrcp_cid, AVRCP_NOTIFICATION_EVENT_VOLUME_CHANGED);
        avrcp_target_volume_changed(s_avrcp_cid, s_volume);
        ESP_LOGW(TAG, "AVRCP connected (cid 0x%02x), absolute volume %u/127", s_avrcp_cid, s_volume);
        /* play state + track changes for JBL One's player */
        avrcp_controller_enable_notification(s_avrcp_cid, AVRCP_NOTIFICATION_EVENT_PLAYBACK_STATUS_CHANGED);
        avrcp_controller_enable_notification(s_avrcp_cid, AVRCP_NOTIFICATION_EVENT_TRACK_CHANGED);
        break;
    }
    case AVRCP_SUBEVENT_CONNECTION_RELEASED:
        if (s_avrcp_cid != 0) ESP_LOGW(TAG, "AVRCP disconnected");
        s_avrcp_cid = 0;
        s_play_status = 0xFE;
        s_title[0] = s_artist[0] = 0;
        break;
    case AVRCP_SUBEVENT_NOTIFICATION_PLAYBACK_STATUS_CHANGED: {
        uint8_t ps = avrcp_subevent_notification_playback_status_changed_get_play_status(packet);
        if (ps != s_play_status) {
            s_play_status = ps;
            ESP_LOGI(TAG, "phone playback status %u", ps);
            jbl_v5_notify(0x2D40);
        }
        break;
    }
    case AVRCP_SUBEVENT_NOTIFICATION_TRACK_CHANGED:
        if (s_avrcp_cid) avrcp_controller_get_now_playing_info(s_avrcp_cid);
        break;
    case AVRCP_SUBEVENT_NOW_PLAYING_TITLE_INFO:
        meta_copy(s_title, avrcp_subevent_now_playing_title_info_get_value(packet),
                  avrcp_subevent_now_playing_title_info_get_value_len(packet));
        break;
    case AVRCP_SUBEVENT_NOW_PLAYING_ARTIST_INFO:
        meta_copy(s_artist, avrcp_subevent_now_playing_artist_info_get_value(packet),
                  avrcp_subevent_now_playing_artist_info_get_value_len(packet));
        break;
    case AVRCP_SUBEVENT_NOW_PLAYING_INFO_DONE:
        ESP_LOGI(TAG, "now playing: '%s' - '%s'", s_title, s_artist);
        jbl_v5_notify(0x2D43);
        jbl_v5_notify(0x2D44);
        break;
    case AVRCP_SUBEVENT_NOTIFICATION_VOLUME_CHANGED:      /* target: the phone set absolute volume */
    {
        uint8_t was = s_volume;
        volume_set(avrcp_subevent_notification_volume_changed_get_absolute_volume(packet) & 0x7f, "phone");
        if ((s_volume == 127 || s_volume == 0) && was != s_volume) {
            tones_play("vol_max");                /* phone slider reached max / min */
        }
    }
        break;
    case AVRCP_SUBEVENT_OPERATION: {                     /* target: volume keys (no absolute volume) */
        if (avrcp_subevent_operation_get_button_pressed(packet) == 0) break;
        uint8_t op = avrcp_subevent_operation_get_operation_id(packet);
        int v = s_volume;
        if (op == AVRCP_OPERATION_ID_VOLUME_UP) v += 8;
        else if (op == AVRCP_OPERATION_ID_VOLUME_DOWN) v -= 8;
        else break;
        uint8_t was = s_volume;
        volume_set((uint8_t)(v < 0 ? 0 : v > 127 ? 127 : v), "volume key");
        if (s_avrcp_cid) avrcp_target_volume_changed(s_avrcp_cid, s_volume);
        if ((s_volume == 127 || s_volume == 0) && (was != s_volume || v > 127 || v < 0)) {
            tones_play("vol_max");
        }
        break;
    }
    default:
        break;
    }
}

static void flush_check(btstack_timer_source_t *ts)
{
    int64_t last = s_last_media_us;
    if (last != 0 && esp_timer_get_time() - last > (int64_t)FLUSH_IDLE_MS * 1000) {
        s_last_media_us = 0;
        uint32_t fill_ms = ring_fill(&s_ring) / (OUT_RATE / 1000);
        xQueueReset(s_pkt_q);                      /* undecoded packets */
        s_reset_dsp = true;                        /* SBC + resampler fresh for the next packet */
        s_flush_req = true;                        /* the reader empties the PCM ring */
        s_last_arrival = 0;
        s_flushes++;
        ESP_LOGW(TAG, "no audio for %d ms (%s): buffers cleared (%lu ms of PCM dropped)", FLUSH_IDLE_MS,
                 !s_connected ? "disconnected" : s_streaming ? "phone stopped sending" : "stopped",
                 (unsigned long)fill_ms);
    }
    btstack_run_loop_set_timer(ts, 100);
    btstack_run_loop_add_timer(ts);
}

void a2dp_in_setup(void)
{
    btstack_run_loop_set_timer_handler(&s_flush_timer, &flush_check);
    btstack_run_loop_set_timer(&s_flush_timer, 100);
    btstack_run_loop_add_timer(&s_flush_timer);
    {   /* last volume */
        nvs_handle_t h;
        uint8_t v;
        if (nvs_open("s31", NVS_READONLY, &h) == ESP_OK) {
            if (nvs_get_u8(h, "volume", &v) == ESP_OK && v <= 127) s_volume = v;
            nvs_close(h);
        }
    }
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
    /* core 1: core 0 carries the broadcast LC3 encoder, I2S and the party decoder; core 1
     * only BTstack (higher priority, preempts this) and the controller. With both decoders
     * on core 0, 320 kbps AAC + hosting pinned core 0 at 100% (task watchdog). */
    if (aac_decoder_workspace_need() > sizeof(s_codec_ws)) {
        ESP_LOGE(TAG, "AAC needs %u B > workspace %u B", aac_decoder_workspace_need(), (unsigned)sizeof(s_codec_ws));
    } else {
        aac_decoder_set_workspace(s_codec_ws, sizeof(s_codec_ws));
    }
    xTaskCreatePinnedToCore(dsp_task, "sbc_dsp", 6144,   /* SBC ~0.8 KB peak; Helix AAC uses more */ NULL, configMAX_PRIORITIES - 6, NULL, 1);

    /* Bluetooth side */
    l2cap_init();
    /* Incoming AVDTP/AVCTP channels: do NOT start our own authentication. The
     * phone authenticates + encrypts the link itself right after connecting;
     * BTstack's level-2 check on its AVDTP channel started a second, colliding
     * authentication and the controller dropped the link (0x16, ~80 ms after
     * the link key reply) - every phone-initiated reconnect failed. Services
     * registered below read this level (gap_get_security_level()). */
    gap_set_security_level(LEVEL_0);
    sdp_init();
    a2dp_sink_init();
    avrcp_init();
    avrcp_controller_init();
    avrcp_target_init();
    /* services above registered at LEVEL_0 (incoming: the phone authenticates);
     * channels WE open (power-up call-back) need LEVEL_2 - the phone closes an
     * unauthenticated AVDTP channel */
    gap_set_security_level(LEVEL_2);
    a2dp_sink_register_packet_handler(&a2dp_handler);
    a2dp_sink_register_media_handler(&media_handler);
    avrcp_register_packet_handler(&avrcp_handler);
    avrcp_controller_register_packet_handler(&avrcp_handler);
    avrcp_target_register_packet_handler(&avrcp_handler);
    avdtp_stream_endpoint_t *aep = a2dp_sink_create_stream_endpoint(AVDTP_AUDIO, AVDTP_CODEC_MPEG_2_4_AAC,
                                                                   s_aac_caps, sizeof(s_aac_caps),
                                                                   s_aac_cfg, sizeof(s_aac_cfg));
    avdtp_stream_endpoint_t *ep = a2dp_sink_create_stream_endpoint(AVDTP_AUDIO, AVDTP_CODEC_SBC,
                                                                  s_sbc_caps, sizeof(s_sbc_caps),
                                                                  s_sbc_cfg, sizeof(s_sbc_cfg));
    s_sbc_seid = avdtp_local_seid(ep);
    s_local_seid = s_sbc_seid;
    avdtp_sink_register_delay_reporting_category(s_sbc_seid);   /* tell the phone our latency (video lip sync) */
    if (aep != NULL) {
        s_aac_seid = avdtp_local_seid(aep);
        avdtp_sink_register_delay_reporting_category(s_aac_seid);
    }
    ESP_LOGW(TAG, "A2DP sink endpoints: AAC-LC seid %u, SBC seid %u", s_aac_seid, s_sbc_seid);

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
    /* BR/EDR Secure Connections off: the S31 controller brings a reconnect up
     * with E0 encryption even when the bond used SC, and BTstack then aborts
     * the link ("SC during pairing, but only E0 now" -> host disconnect, 0x16
     * ~300 ms after the phone connects). Legacy SSP (P-192) like most speakers. */
    gap_secure_connections_enable(false);
    gap_set_bondable_mode(1);
    gap_set_class_of_device(0x240414);            /* audio, loudspeaker */
    /* Page scan: the controller default (11.25 ms every 1.28 s, standard) let a
     * phone's reconnect miss window after window behind the 20 ms LE beacon and
     * the assistant's LE scan. Interlaced every 320 ms catches both page trains
     * in one window; off anyway while a phone streams (gap_connectable_control). */
    gap_set_page_scan_activity(0x0100, 0x0024);  /* 22.5 ms every 160 ms (interlaced: 2 per 160 ms) */
    gap_set_page_scan_type(PAGE_SCAN_MODE_INTERLACED);
    gap_discoverable_control(0);                  /* a2dp_in_on_hci_working: pairing mode or not */
    gap_connectable_control(!S31_QUIET_RADIO_TEST);

    s_hci_reg.callback = &hci_handler;
    hci_add_event_handler(&s_hci_reg);
}

/* Power-up: call the last phone back (like a real speaker), twice at most. */
static btstack_timer_source_t s_reconn_timer;
static int s_reconn_tries;
/* Power-up call-back security. At level 2 BTstack authenticates our outgoing
 * link and then asks for encryption; with the REDMAGIC 11 Pro that never
 * completes (no encryption event), the phone hangs up after ~4 s (0x13) and
 * every call-back failed. At level 0 the phone authenticates and encrypts the
 * link itself, as when it connects to us - most phones. Tries alternate between
 * the two, starting with the one that worked last time for this phone (NVS).
 * Known exception: the REDMAGIC 11 Pro - our encryption request is accepted
 * (status 0) but no encryption change ever comes, at either level; it reconnects
 * itself when picked in its Bluetooth list. */
#define RECONN_TRIES        2           /* one per security level */
static uint8_t s_reconn_first_level = 2;          /* 0 or 2 (level 0 alone: the REDMAGIC waits for us) */
static uint8_t s_reconn_level;                    /* level of the try in flight */
static bool s_reconn_pending;
static btstack_timer_source_t s_reconn_level_timer;
/* While the S31 pages out it cannot be paged: a phone connecting meanwhile fails.
 * A phone whose call-back failed every try is remembered and not called back on
 * the next boot (it reconnects itself in ~0.3 s); one success clears it. */
static bd_addr_t s_last_phone;                    /* tentative: defined again below */
static bd_addr_t s_reconn_bad;
static bool s_reconn_bad_valid;

static void reconn_bad_store(bool bad)
{
    nvs_handle_t h;
    if (nvs_open("s31", NVS_READWRITE, &h) != ESP_OK) return;
    if (bad) nvs_set_blob(h, "cb_bad", s_last_phone, 6);
    else nvs_erase_key(h, "cb_bad");
    nvs_commit(h);
    nvs_close(h);
}

static void reconn_level_restore(btstack_timer_source_t *ts)
{
    (void)ts;
    gap_set_security_level(LEVEL_2);              /* our other outgoing channels */
}
static bd_addr_t s_last_phone;

static void reconnect_cb(btstack_timer_source_t *ts)
{
    (void)ts;
    if (s_connected) {
        s_reconn_pending = false;
        return;
    }
    if (s_reconn_tries >= RECONN_TRIES) {
        s_reconn_pending = false;
        ESP_LOGW(TAG, "power-up call-back failed %d times: next boot leaves %s to reconnect itself",
                 RECONN_TRIES, bd_addr_to_str(s_last_phone));
        reconn_bad_store(true);
        return;
    }
    /* A Secure Connections key (CTKD, from the phone's LE pairing) cannot authenticate a
     * link WE start: with BR/EDR SC off the controller tries legacy authentication, the
     * phone refuses it and a new pairing starts that hangs ~9 s and blocks the phone's own
     * reconnect. Such a phone reconnects itself (LE first, then Classic) - leave it to it. */
    {
        link_key_t k;
        link_key_type_t t = INVALID_LINK_KEY;
        if (gap_get_link_key_for_bd_addr(s_last_phone, k, &t) && gap_secure_connection_for_link_key_type(t)) {
            s_reconn_pending = false;
            ESP_LOGW(TAG, "power-up: %s has an LE-paired (SC) key - not calling it back, it reconnects itself",
                     bd_addr_to_str(s_last_phone));
            return;
        }
    }
    s_reconn_tries++;
    s_reconn_level = (s_reconn_tries & 1) ? s_reconn_first_level : (uint8_t)(2 - s_reconn_first_level);
    /* AVDTP takes the level when it opens its channel (after its SDP query) */
    gap_set_security_level(s_reconn_level == 0 ? LEVEL_0 : LEVEL_2);
    btstack_run_loop_remove_timer(&s_reconn_level_timer);
    btstack_run_loop_set_timer_handler(&s_reconn_level_timer, reconn_level_restore);
    btstack_run_loop_set_timer(&s_reconn_level_timer, 8000);   /* paging alone can take 3 s */
    btstack_run_loop_add_timer(&s_reconn_level_timer);
    s_reconn_pending = true;
    ESP_LOGW(TAG, "power-up: calling back last phone %s (try %d/%d, security level %u)", bd_addr_to_str(s_last_phone),
             s_reconn_tries, RECONN_TRIES, s_reconn_level);
    uint8_t st = a2dp_sink_establish_stream(s_last_phone, &s_a2dp_cid);
    if (st != ERROR_CODE_SUCCESS) {
        ESP_LOGW(TAG, "a2dp_sink_establish_stream: 0x%02x", st);
    }
    btstack_run_loop_set_timer(&s_reconn_timer, 12000);   /* fallback: a failure moves on at once */
    btstack_run_loop_add_timer(&s_reconn_timer);
}


static void pair_call(btstack_timer_source_t *ts)
{
    (void)ts;
    if (s_connected || link_tune_classic_links() > 0) {
        ESP_LOGI(TAG, "after LE pairing: the phone's classic link is up by itself");
        return;
    }
    if (s_reconn_pending) return;                 /* the power-up call-back is paging already */
    if (++s_pair_call_tries > 5) {
        ESP_LOGW(TAG, "after LE pairing: gave up calling %s", bd_addr_to_str(s_pair_call_addr));
        return;
    }
    /* the phone's own page arrived (or a link object exists): creating ours now makes the
     * controller refuse it (0x0B) and drops the phone's too - wait for it instead */
    if (s_pair_incoming || hci_connection_for_bd_addr_and_type(s_pair_call_addr, BD_ADDR_TYPE_ACL) != NULL) {
        ESP_LOGI(TAG, "after LE pairing: the phone's classic connect is under way - waiting");
        s_pair_incoming = false;
        btstack_run_loop_set_timer(&s_pair_call_timer, 1500);
        btstack_run_loop_add_timer(&s_pair_call_timer);
        return;
    }
    gap_set_security_level(LEVEL_2);              /* the CTKD key authenticates the link */
    s_pair_call_active = true;
    uint8_t st = a2dp_sink_establish_stream(s_pair_call_addr, &s_a2dp_cid);
    ESP_LOGW(TAG, "after LE pairing: no classic link from %s yet - connecting A2DP ourselves (try %u) -> 0x%02x",
             bd_addr_to_str(s_pair_call_addr), s_pair_call_tries, st);
}

void a2dp_in_call_after_le_pairing(const uint8_t identity[6])
{
    if (S31_QUIET_RADIO_TEST) return;
    bd_addr_copy(s_pair_call_addr, identity);
    s_pair_call_tries = 0;
    s_pair_incoming = false;
    s_pair_call_active = false;
    s_new_bond = true;                            /* pairing mode ends once this phone is connected */
    btstack_run_loop_remove_timer(&s_pair_call_timer);
    btstack_run_loop_set_timer_handler(&s_pair_call_timer, pair_call);
    btstack_run_loop_set_timer(&s_pair_call_timer, 3000);   /* the phone pages 0-1.5 s after the keys: our own page at 1.2 s collided with it (pend list full, the phone left) - only if it never comes */
    btstack_run_loop_add_timer(&s_pair_call_timer);
}

/* our call-back fell into a new pairing (the phone refused our stored key): end that link
 * now instead of letting it hang until the phone fails it, and stop calling back */
static void reconn_abort_on_pairing(const uint8_t *addr)
{
    if (!s_reconn_pending || s_pairing) return;
    hci_connection_t *c = hci_connection_for_bd_addr_and_type((uint8_t *)addr, BD_ADDR_TYPE_ACL);
    ESP_LOGW(TAG, "power-up call-back: %s wants a new pairing - dropping it, the phone reconnects itself",
             bd_addr_to_str(addr));
    s_reconn_tries = RECONN_TRIES;
    if (c) gap_disconnect(c->con_handle);
}

static void reconnect_failed(void)
{
    if (!s_reconn_pending) return;
    s_reconn_pending = false;
    btstack_run_loop_remove_timer(&s_reconn_timer);
    btstack_run_loop_set_timer(&s_reconn_timer, 500);     /* next try (other security level) */
    btstack_run_loop_add_timer(&s_reconn_timer);
}

static void reconnect_succeeded(void)
{
    if (!s_reconn_pending) return;
    s_reconn_pending = false;
    btstack_run_loop_remove_timer(&s_reconn_timer);
    if (s_reconn_level != s_reconn_first_level) {
        nvs_handle_t h;
        if (nvs_open("s31", NVS_READWRITE, &h) == ESP_OK) {
            nvs_set_u8(h, "cb_level", s_reconn_level);
            nvs_commit(h);
            nvs_close(h);
        }
        s_reconn_first_level = s_reconn_level;
    }
    ESP_LOGW(TAG, "power-up call-back worked at security level %u (try %d)", s_reconn_level, s_reconn_tries);
    if (s_reconn_bad_valid) {
        s_reconn_bad_valid = false;
        reconn_bad_store(false);
    }
}

void a2dp_in_on_hci_working(void)
{
    if (S31_QUIET_RADIO_TEST) {
        ESP_LOGW(TAG, "QUIET RADIO TEST: BR/EDR off air (no scan, no call-back)");
        return;
    }
    nvs_handle_t h;
    size_t n = 6;
    btstack_link_key_iterator_t it;
    bool any_key = false;
    if (gap_link_key_iterator_init(&it)) {
        bd_addr_t a; link_key_t k; link_key_type_t t;
        any_key = gap_link_key_iterator_get_next(&it, a, k, &t);
        gap_link_key_iterator_done(&it);
    }
    if (!any_key && le_device_db_count() == 0) {
        ESP_LOGW(TAG, "no phone paired yet: pairing mode");
        a2dp_in_button(A2DP_BTN_PAIRING);             /* as a new JBL speaker */
    } else {
        a2dp_in_refresh_discoverable();               /* not discoverable until the button */
    }
    if (nvs_open("s31", NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    esp_err_t err = nvs_get_blob(h, "last_phone", s_last_phone, &n);
    uint8_t lvl;
    if (nvs_get_u8(h, "cb_level", &lvl) == ESP_OK && (lvl == 0 || lvl == 2)) s_reconn_first_level = lvl;
    size_t bn = 6;
    s_reconn_bad_valid = nvs_get_blob(h, "cb_bad", s_reconn_bad, &bn) == ESP_OK && bn == 6;
    nvs_close(h);
    if (err != ESP_OK || n != 6) {
        return;
    }
    if (s_reconn_bad_valid && bd_addr_cmp(s_reconn_bad, s_last_phone) == 0) {
        ESP_LOGW(TAG, "power-up: not calling back %s (its call-back failed last time) - connectable, it reconnects itself",
                 bd_addr_to_str(s_last_phone));
        return;
    }
    btstack_run_loop_set_timer_handler(&s_reconn_timer, reconnect_cb);
    btstack_run_loop_set_timer(&s_reconn_timer, 300);
    btstack_run_loop_add_timer(&s_reconn_timer);
}

/* pairing mode: close every phone link (Classic, and LE links where we are the
 * peripheral); our own central links to JBL speakers stay */
static void drop_all_phone_links(btstack_timer_source_t *ts)
{
    (void)ts;
    s_reconn_pending = false;                       /* no power-up call-back now */
    btstack_run_loop_remove_timer(&s_reconn_timer);
    s_pair_call_active = false;
    btstack_run_loop_remove_timer(&s_pair_call_timer);
    btstack_linked_list_iterator_t it;
    hci_connections_get_iterator(&it);
    int n = 0;
    while (btstack_linked_list_iterator_has_next(&it)) {
        hci_connection_t *c = (hci_connection_t *)btstack_linked_list_iterator_next(&it);
        hci_con_handle_t h = c->con_handle;
        gap_connection_type_t t = gap_get_connection_type(h);
        if (t == GAP_CONNECTION_LE && gap_get_role(h) != HCI_ROLE_SLAVE) continue;   /* ours (JBL speakers) */
        if (t != GAP_CONNECTION_ACL && t != GAP_CONNECTION_LE) continue;
        if (gap_disconnect(h) == ERROR_CODE_SUCCESS) n++;
    }
    ESP_LOGW(TAG, "pairing mode: dropped %d phone link(s) - phones pair / connect again", n);
}
