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
#include "bt_sched_tune.h"
#include "auracast.h"

static const char *TAG = "A2DP";

#define OUT_RATE        48000
#define PKT_MAX         (AVDTP_L2CAP_MTU + 16)
#define PKT_Q_LEN       24                    /* ~550 ms of 8-frame packets */
#define RING_MS         500                   /* max S31-added delay: beyond it the OLDEST audio is dropped */
#define TARGET_MS       130                    /* average PCM fill (= added latency); the fill swings below it by the read bursts + delivery gaps */
#define PRIME_MS        TARGET_MS             /* start output at the target fill: no start-up underruns / integrator wind-up */
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
#define CEILING_MS 150                       /* 1 s average above this: splice to the target */
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
            s_reset_deficit = true;
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
/* Delivery timing (BT thread): gaps between media packets as they reach the
 * host. A gap with no seq gap = the phone's packets were late on air (or held
 * by the phone), not lost. Logged as they happen so they line up with the
 * other logs. */
static volatile uint32_t s_gap_max_us, s_gap_n60, s_gap_n100, s_gap_n200;
static int64_t s_last_arrival;

static void media_handler(uint8_t seid, uint8_t *packet, uint16_t size)
{
    (void)seid;
    static uint16_t last_seq;
    static uint32_t last_rtp;
    static bool have_seq;
    if (size < 13 || size > PKT_MAX) {
        return;
    }
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
    last_rtp = rtp;
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

/* AVRCP absolute volume 0..127 -> gain (cubic: about -40 dB at 22 %, 0 dB at 127) */
static volatile uint8_t s_volume = 100;
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
    float target = volume_gain(s_volume);
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
int a2dp_in_read(int16_t *pcm, int n)
{
#if S31_TEST_SIGNAL
    if (s_streaming && s_primed) {
        ring_read_upto(&s_ring, (uint32_t *)pcm, (uint32_t)n);   /* keep A2DP draining, discard */
    }
    test_fill(pcm, n);
    return n;
#endif
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
        if (!boosted && f < (float)MS_FRAMES(70) && s_streaming) {
            boosted = true;
            s_acl_boosts++;
            bt_sched_tune_apply_custom(9, 15, 12, 11, "A2DP cushion low: phone link boosted");
        } else if (boosted && (f > (float)MS_FRAMES(110) || !s_streaming)) {
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
        /* hard ceiling: 2 samples (1 s) averaging above 150 ms is not a burst */
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
bool a2dp_in_connected(void) { return s_connected; }

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
static void avrcp_connect_cb(btstack_timer_source_t *ts)
{
    (void)ts;
    if (s_connected && s_avrcp_cid == 0) {
        uint16_t cid = 0;
        uint8_t st = avrcp_connect(s_peer, &cid);
        ESP_LOGW(TAG, "AVRCP: phone did not connect - connecting ourselves -> 0x%02x", st);
    }
}

/* ---- buttons (posted to the BT thread) ---------------------------------------- */
static volatile uint32_t s_btn_pending;          /* bit per a2dp_btn_t */
static btstack_context_callback_registration_t s_btn_reg;
static btstack_timer_source_t s_pair_timer;
static bool s_pairing;

static void pairing_end(btstack_timer_source_t *ts)
{
    (void)ts;
    s_pairing = false;
    gap_discoverable_control(!S31_QUIET_RADIO_TEST);
    gap_connectable_control(!S31_QUIET_RADIO_TEST && !s_streaming);
    ESP_LOGW(TAG, "pairing mode ended");
}

static void btn_run(void *ctx)
{
    (void)ctx;
    uint32_t b = __atomic_exchange_n(&s_btn_pending, 0, __ATOMIC_ACQ_REL);
    if (b & (1u << A2DP_BTN_VOL_UP | 1u << A2DP_BTN_VOL_DOWN)) {
        int v = s_volume + ((b & (1u << A2DP_BTN_VOL_UP)) ? 8 : 0) - ((b & (1u << A2DP_BTN_VOL_DOWN)) ? 8 : 0);
        volume_set((uint8_t)(v < 0 ? 0 : v > 127 ? 127 : v), "button");
        if (s_avrcp_cid) avrcp_target_volume_changed(s_avrcp_cid, s_volume);   /* phone slider follows */
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
        s_pairing = true;
        gap_discoverable_control(1);
        gap_connectable_control(1);
        btstack_run_loop_remove_timer(&s_pair_timer);
        btstack_run_loop_set_timer_handler(&s_pair_timer, pairing_end);
        btstack_run_loop_set_timer(&s_pair_timer, 180000);
        btstack_run_loop_add_timer(&s_pair_timer);
        ESP_LOGW(TAG, "pairing mode: discoverable + connectable for 3 min");
    }
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
    ESP_LOGW(TAG, "A2DP 2s: %s, media pkts %lu (+%lu, %lu.%lu SBC frames/pkt, max %u B), q drops %lu, seq gaps %lu | bitpool %lu (min %lu max %lu) | "
             "PCM %lu ms (avg %lu, min %lu, max %lu), aligns %lu, ceilings %lu, ACL boosts %lu, underruns %lu, oldest dropped %lu, clock corr %ld ppm | catch-ups %lu (%lu ms skipped, %lu expired) | volume %u/127%s | gaps max %lu ms, >60 %lu >100 %lu >200 %lu",
             s_streaming ? "STREAMING" : s_connected ? "connected" : "no phone",
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
        btstack_run_loop_remove_timer(&s_avrcp_timer);
        btstack_run_loop_set_timer_handler(&s_avrcp_timer, avrcp_connect_cb);
        btstack_run_loop_set_timer(&s_avrcp_timer, 400);    /* the phone holds its stream until AVRCP is up: 2 s here = 2 s reconnect delay */
        btstack_run_loop_add_timer(&s_avrcp_timer);
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
    case A2DP_SUBEVENT_STREAM_ESTABLISHED:
        /* AVDTP 1.3: the sink reports its delay once the stream is configured */
        ESP_LOGW(TAG, "A2DP stream established, status 0x%02x", a2dp_subevent_stream_established_get_status(packet));
        if (a2dp_subevent_stream_established_get_status(packet) == ERROR_CODE_SUCCESS) {
            delay_report_soon(50);
        }
        break;
    case A2DP_SUBEVENT_SIGNALING_DELAY_REPORTING_CAPABILITY:
        ESP_LOGW(TAG, "phone supports delay reporting");
        break;
    case A2DP_SUBEVENT_STREAM_STARTED:
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
        s_streaming = false;
        bt_sched_tune_streaming(false);
        gap_connectable_control(!S31_QUIET_RADIO_TEST);
        ESP_LOGW(TAG, "A2DP audio %s", packet[2] == A2DP_SUBEVENT_STREAM_SUSPENDED ? "suspended" : "stopped");
        break;
    case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
        s_streaming = false;
        s_connected = false;
        bt_sched_tune_streaming(false);
        gap_connectable_control(!S31_QUIET_RADIO_TEST);
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
    case HCI_EVENT_ENCRYPTION_CHANGE:
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
        break;
    case HCI_EVENT_IO_CAPABILITY_REQUEST:
        ESP_LOGW(TAG, "phone starts (re)pairing (IO capability request)");
        break;
    case GAP_EVENT_PAIRING_COMPLETE:
        ESP_LOGW(TAG, "GAP pairing complete, status 0x%02x", gap_event_pairing_complete_get_status(packet));
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
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET || hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) {
        return;
    }
    switch (packet[2]) {
    case AVRCP_SUBEVENT_CONNECTION_ESTABLISHED: {
        uint8_t st = avrcp_subevent_connection_established_get_status(packet);
        if (st != ERROR_CODE_SUCCESS) {
            ESP_LOGW(TAG, "AVRCP connect failed 0x%02x", st);
            break;
        }
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
        break;
    }
    case AVRCP_SUBEVENT_CONNECTION_RELEASED:
        if (s_avrcp_cid != 0) ESP_LOGW(TAG, "AVRCP disconnected");
        s_avrcp_cid = 0;
        break;
    case AVRCP_SUBEVENT_NOTIFICATION_VOLUME_CHANGED:      /* target: the phone set absolute volume */
        volume_set(avrcp_subevent_notification_volume_changed_get_absolute_volume(packet) & 0x7f, "phone");
        break;
    case AVRCP_SUBEVENT_OPERATION: {                     /* target: volume keys (no absolute volume) */
        if (avrcp_subevent_operation_get_button_pressed(packet) == 0) break;
        uint8_t op = avrcp_subevent_operation_get_operation_id(packet);
        int v = s_volume;
        if (op == AVRCP_OPERATION_ID_VOLUME_UP) v += 8;
        else if (op == AVRCP_OPERATION_ID_VOLUME_DOWN) v -= 8;
        else break;
        volume_set((uint8_t)(v < 0 ? 0 : v > 127 ? 127 : v), "volume key");
        if (s_avrcp_cid) avrcp_target_volume_changed(s_avrcp_cid, s_volume);
        break;
    }
    default:
        break;
    }
}

void a2dp_in_setup(void)
{
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
    xTaskCreatePinnedToCore(dsp_task, "sbc_dsp", 4096,   /* measured peak ~0.8 KB */ NULL, configMAX_PRIORITIES - 6, NULL, 0);

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
    avdtp_stream_endpoint_t *ep = a2dp_sink_create_stream_endpoint(AVDTP_AUDIO, AVDTP_CODEC_SBC,
                                                                  s_sbc_caps, sizeof(s_sbc_caps),
                                                                  s_sbc_cfg, sizeof(s_sbc_cfg));
    s_local_seid = avdtp_local_seid(ep);
    avdtp_sink_register_delay_reporting_category(s_local_seid);   /* tell the phone our latency (video lip sync) */

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
    btstack_run_loop_set_timer(&s_reconn_timer, 300);
    btstack_run_loop_add_timer(&s_reconn_timer);
}
