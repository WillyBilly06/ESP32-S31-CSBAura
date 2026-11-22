/*
 * Phone-audio DSP for the A2DP bridge, kept free of ESP-IDF dependencies so the
 * host test (tools/rs_host_test.c) compiles the very same code:
 *
 *   - 44.1/48 kHz -> 48 kHz windowed-sinc polyphase resampler
 *   - single-producer / single-consumer frame ring (lock-free)
 *
 * Resampler: 32 taps, 256 phases with linear interpolation between adjacent
 * phases, Blackman window, cutoff 20 kHz (same kernel as before). Linear
 * interpolation of the samples (the first resampler) left images only
 * ~20-25 dB down; this keeps them below -80 dB. The ratio can be nudged
 * +-0.2 % per call for drift control.
 *
 * Cost per 48 kHz output frame (both channels): 32 x (4 loads + 4 fmadd), fully
 * unrolled with constant offsets, single precision only, no libm calls. The
 * history is interleaved float, processed in chunks of RS_CHUNK input frames;
 * only the <= 31-frame filter tail is moved between chunks.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#define RS_TAPS     32
#define RS_HALF     (RS_TAPS / 2)
#define RS_PHASES   256                         /* 2^RS_PHASE_BITS */
#define RS_PHASE_BITS 8
#define RS_CHUNK    256                         /* input frames per FIR pass */
#define RS_HIST_FRAMES (RS_TAPS + RS_CHUNK)
/* worst-case outputs of one pass: 256 frames of 16 kHz input -> ~770 */
#define RS_OUT_MAX  (RS_CHUNK * 3 + 16)

typedef struct {
    float hist[RS_HIST_FRAMES * 2];             /* interleaved L,R */
    int hlen;                                   /* valid frames in hist */
    int pos_i;                                  /* read position: integer frame in hist ... */
    uint32_t pos_f;                             /* ... and Q32 fraction */
} rs_state_t;

/* [phase][tap]; row p+1 is the next phase, so one base pointer serves both */
static float rs_kern[RS_PHASES + 1][RS_TAPS];

static void rs_init_kernel(void)
{
    /* once at start-up: double is fine here */
    const double fc = 20000.0 / 44100.0;       /* cutoff as a fraction of the input rate */
    for (int p = 0; p <= RS_PHASES; p++) {
        double frac = (double)p / RS_PHASES;
        double sum = 0;
        for (int k = 0; k < RS_TAPS; k++) {
            double t = (double)k - (RS_HALF - 1) - frac;         /* tap offset in input samples */
            double x = 2.0 * fc * t;
            double sinc = fabs(x) < 1e-9 ? 1.0 : sin(M_PI * x) / (M_PI * x);
            double w = 0.42 + 0.5 * cos(M_PI * t / RS_HALF) + 0.08 * cos(2.0 * M_PI * t / RS_HALF);
            if (fabs(t) >= RS_HALF) w = 0;
            rs_kern[p][k] = (float)(2.0 * fc * sinc * w);
            sum += rs_kern[p][k];
        }
        for (int k = 0; k < RS_TAPS; k++) {
            rs_kern[p][k] = (float)(rs_kern[p][k] / sum);       /* unity gain at DC for every phase */
        }
    }
}

static void rs_reset(rs_state_t *s)
{
    memset(s->hist, 0, sizeof(float) * 2 * RS_TAPS);
    s->hlen = RS_TAPS;                          /* start on a short run of silence */
    s->pos_i = RS_HALF - 1;
    s->pos_f = 0;
}

/* Q32 step (input frames per output frame) for a source rate and a fill error
 * e in [-1, 1] (+-0.2 % max). Once per call; float only. */
static inline uint64_t rs_step_q32(uint32_t src_rate, uint32_t out_rate, float e)
{
    if (e > 1.0f) e = 1.0f;
    if (e < -1.0f) e = -1.0f;
    /* out_rate is a power-of-ten constant (48000): multiply by its reciprocal */
    float stepf = (float)src_rate * (1.0f / (float)out_rate) * (1.0f + 0.002f * e);
    return (uint64_t)(stepf * 4294967296.0f);
}

/* round-to-nearest-even float -> int (what lrintf does, minus the libm call) */
static inline int32_t rs_f2i(float x)
{
#if defined(__riscv) && defined(__riscv_flen)
    int32_t r;
    __asm__ ("fcvt.w.s %0, %1, rne" : "=r"(r) : "f"(x));
    return r;
#else
    return (int32_t)lrintf(x);
#endif
}

/* clamp to the s16 range. Not fminf/fmaxf: picolibc's versions call
 * __issignalingf (from flash) for every argument. */
static inline float rs_clamp16(float x)
{
#if defined(__riscv) && defined(__riscv_flen)
    float r;
    __asm__ ("fmax.s %0, %1, %2" : "=f"(r) : "f"(x), "f"(-32768.f));
    __asm__ ("fmin.s %0, %1, %2" : "=f"(r) : "f"(r), "f"(32767.f));
    return r;
#else
    return x > 32767.f ? 32767.f : (x < -32768.f ? -32768.f : x);
#endif
}

static inline uint32_t rs_pack(float l, float r)
{
    l = rs_clamp16(l);
    r = rs_clamp16(r);
    return (uint16_t)(int16_t)rs_f2i(l) | ((uint32_t)(uint16_t)(int16_t)rs_f2i(r) << 16);
}

/* Room for this many more input frames before rs_run() must be called. */
static inline int rs_room(const rs_state_t *s)
{
    return RS_HIST_FRAMES - s->hlen;
}

/* Append n (<= rs_room) interleaved s16 stereo frames. */
static inline void rs_push(rs_state_t *s, const int16_t *in, int n)
{
    float *h = &s->hist[2 * s->hlen];
    for (int j = 0; j < 2 * n; j++) {
        h[j] = (float)in[j];
    }
    s->hlen += n;
}

/* Produce output frames (packed L | R << 16) until the input is used up or
 * out_max is reached; then drop consumed input, keeping the filter tail.
 * With out_max >= RS_OUT_MAX all pushed input is always consumed. */
static int rs_run(rs_state_t *s, uint64_t step, uint32_t *out, int out_max)
{
    const int si = (int)(step >> 32);
    const uint32_t sf = (uint32_t)step;
    int i = s->pos_i;
    uint32_t fr = s->pos_f;
    const int hlen = s->hlen;
    int o = 0;

    while (i + RS_HALF < hlen && o < out_max) {
        const float *k0 = rs_kern[fr >> (32 - RS_PHASE_BITS)];
        const float t = (float)(fr & ((1u << (32 - RS_PHASE_BITS)) - 1)) *
                        (1.0f / (float)(1u << (32 - RS_PHASE_BITS)));
        const float *x = &s->hist[2 * (i - (RS_HALF - 1))];
        float a0 = 0, a1 = 0, b0 = 0, b1 = 0;  /* both phases, then interpolate the outputs */
#pragma GCC unroll 32
        for (int k = 0; k < RS_TAPS; k++) {
            const float xl = x[2 * k], xr = x[2 * k + 1];
            const float c0 = k0[k], c1 = k0[k + RS_TAPS];
            a0 += c0 * xl;
            a1 += c0 * xr;
            b0 += c1 * xl;
            b1 += c1 * xr;
        }
        out[o++] = rs_pack(a0 + (b0 - a0) * t, a1 + (b1 - a1) * t);
        const uint32_t f2 = fr + sf;
        i += si + (f2 < fr);
        fr = f2;
    }

    int keep_from = i - (RS_HALF - 1);
    if (keep_from > 0) {
        int keep = hlen - keep_from;            /* <= RS_TAPS frames when all input was used */
        memmove(s->hist, s->hist + 2 * keep_from, sizeof(float) * 2 * keep);
        s->hlen = keep;
        i -= keep_from;
    }
    s->pos_i = i;
    s->pos_f = fr;
    return o;
}

/* ---- SPSC ring of stereo s16 frames (stored as uint32) ------------------------
 * One writer (A2DP decoder task) and one reader (encoder task); positions are
 * 0..slots-1 with one slot kept empty, so it never needs a lock and survives
 * counter wrap. Data may live in PSRAM; the positions are plain words. */
typedef struct {
    uint32_t *buf;
    uint32_t slots;                             /* capacity + 1 */
    uint32_t wr;                                /* written only by the producer */
    uint32_t rd;                                /* written only by the consumer */
} spsc_ring_t;

static inline uint32_t ring_fill(const spsc_ring_t *r)
{
    uint32_t w = __atomic_load_n(&r->wr, __ATOMIC_ACQUIRE);
    uint32_t d = __atomic_load_n(&r->rd, __ATOMIC_ACQUIRE);
    return w >= d ? w - d : w + r->slots - d;
}

static inline uint32_t ring_space(const spsc_ring_t *r)
{
    return r->slots - 1 - ring_fill(r);
}

/* Producer: n must be <= ring_space(). */
static inline void ring_write(spsc_ring_t *r, const uint32_t *src, uint32_t n)
{
    uint32_t w = r->wr;
    uint32_t first = r->slots - w;
    if (first > n) first = n;
    memcpy(r->buf + w, src, first * 4);
    memcpy(r->buf, src + first, (n - first) * 4);
    w += n;
    if (w >= r->slots) w -= r->slots;
    __atomic_store_n(&r->wr, w, __ATOMIC_RELEASE);
}

/* Overflow policy: drop the OLDEST audio, keep the newest. The read position
 * is therefore moved by both sides, always with compare-and-swap:
 *   producer  ring_drop_oldest()  advances rd past the oldest frames
 *   consumer  ring_read_upto()    copies, then commits rd only if nobody moved
 *                                 it meanwhile (else the copy may contain
 *                                 frames the producer has just overwritten,
 *                                 and it retries on the new position)
 * The write position stays producer-only. */
static inline uint32_t ring_fill_at(const spsc_ring_t *r, uint32_t d)
{
    uint32_t w = __atomic_load_n(&r->wr, __ATOMIC_ACQUIRE);
    return w >= d ? w - d : w + r->slots - d;
}

/* Consumer: copy up to n frames; returns how many. */
static inline uint32_t ring_read_upto(spsc_ring_t *r, uint32_t *dst, uint32_t n)
{
    for (;;) {
        uint32_t d = __atomic_load_n(&r->rd, __ATOMIC_ACQUIRE);
        uint32_t have = ring_fill_at(r, d);
        uint32_t take = have < n ? have : n;
        uint32_t first = r->slots - d;
        if (first > take) first = take;
        memcpy(dst, r->buf + d, first * 4);
        memcpy(dst + first, r->buf, (take - first) * 4);
        uint32_t nd = d + take;
        if (nd >= r->slots) nd -= r->slots;
        if (__atomic_compare_exchange_n(&r->rd, &d, nd, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
            return take;
        }
    }
}

/* Consumer: n must be <= ring_fill(). */
static inline void ring_read(spsc_ring_t *r, uint32_t *dst, uint32_t n)
{
    (void)ring_read_upto(r, dst, n);
}

/* Either side: drop up to n of the oldest frames; returns how many. */
static inline uint32_t ring_drop_oldest(spsc_ring_t *r, uint32_t n)
{
    for (;;) {
        uint32_t d = __atomic_load_n(&r->rd, __ATOMIC_ACQUIRE);
        uint32_t have = ring_fill_at(r, d);
        uint32_t k = have < n ? have : n;
        uint32_t nd = d + k;
        if (nd >= r->slots) nd -= r->slots;
        if (__atomic_compare_exchange_n(&r->rd, &d, nd, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
            return k;
        }
    }
}

/* Consumer: drop the n oldest frames (flush = ring_skip(r, ring_fill(r))). */
static inline void ring_skip(spsc_ring_t *r, uint32_t n)
{
    (void)ring_drop_oldest(r, n);
}
