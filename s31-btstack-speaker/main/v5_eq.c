/*
 * JBL One app EQ (port of v5_eq_process / eq_from_q29 in the Bluedroid build's
 * jbl_v5_server.c), single-precision, lock-free coefficient hand-over.
 *
 * Filter per band and channel (direct form I, the app's own coefficients):
 *   y = b0 x + b1 x1 + b2 x2 + na1 y1 + na2 y2      (na = -a, as sent)
 * The Bluedroid version scaled by 0.7071 before and 1.4142 after the cascade;
 * in float that is a no-op (x0.99998), so it is left out. Output is rounded
 * and clipped to s16.
 *
 * Hand-over (one writer on the BTstack thread, one reader on core 0):
 *   writer: seq = odd -> copy bank -> seq = even (release)
 *   reader: at the start of a call, if seq changed and is even, copy the bank
 *           and re-check seq; a torn copy is dropped and retried next call.
 * Bands whose coefficients are identity are skipped; a band that becomes
 * active starts from zero state, bands active before and after keep theirs
 * (no click when the app moves a slider).
 */
#include <math.h>
#include <string.h>

#include "esp_cpu.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "v5_eq.h"

static const char *TAG = "V5_EQ";

typedef struct {
    float b0, b1, b2, na1, na2;
} biquad_t;

typedef struct {
    biquad_t bq[V5_EQ_BANDS];
    uint8_t act[V5_EQ_BANDS];       /* non-identity bands, cascade order */
    uint8_t nact;
} eq_bank_t;

/* ---- writer -> reader -------------------------------------------------------- */
static eq_bank_t s_pub;
static uint32_t s_seq;              /* even: stable, odd: being written */
static volatile bool s_pub_on;
static volatile uint32_t s_published;

/* ---- reader private (core 0) ------------------------------------------------- */
#define EQ_CHUNK 240                /* frames per pass: 1.9 KB float scratch */
static eq_bank_t s_cur;
static uint32_t s_cur_seq;
static float s_z[V5_EQ_BANDS][8];   /* L: x1 x2 y1 y2, R: x1 x2 y1 y2 */
static float s_buf[2 * EQ_CHUNK];

/* ---- stats (reader writes) --------------------------------------------------- */
static volatile uint32_t s_calls, s_switches, s_deferred, s_cyc_max, s_cyc_last;

#define EQ_IDENT_EPS 1e-4f
#define EQ_DENORM    1e-15f

void v5_eq_set_q29(const uint8_t coef[V5_EQ_COEF_BYTES])
{
    const float k = 1.0f / 536870912.0f;          /* 2^-29 */
    eq_bank_t b;
    memset(&b, 0, sizeof(b));
    for (int i = 0; i < V5_EQ_BANDS; i++) {
        int32_t q[5];
        memcpy(q, coef + 20 * i, sizeof(q));        /* LE on air, LE target */
        biquad_t *f = &b.bq[i];
        f->b0 = (float)q[0] * k;
        f->b1 = (float)q[1] * k;
        f->b2 = (float)q[2] * k;
        f->na1 = (float)q[3] * k;
        f->na2 = (float)q[4] * k;
        bool ident = fabsf(f->b0 - 1.0f) <= EQ_IDENT_EPS && fabsf(f->b1) <= EQ_IDENT_EPS &&
                     fabsf(f->b2) <= EQ_IDENT_EPS && fabsf(f->na1) <= EQ_IDENT_EPS &&
                     fabsf(f->na2) <= EQ_IDENT_EPS;
        if (!ident) {
            b.act[b.nact++] = (uint8_t)i;
        }
    }

    uint32_t s = __atomic_load_n(&s_seq, __ATOMIC_RELAXED);
    __atomic_store_n(&s_seq, s + 1u, __ATOMIC_RELAXED);
    __atomic_thread_fence(__ATOMIC_RELEASE);
    memcpy(&s_pub, &b, sizeof(b));
    __atomic_store_n(&s_seq, s + 2u, __ATOMIC_RELEASE);

    s_pub_on = b.nact != 0;
    s_published++;
}

void v5_eq_design_q29(uint8_t type, float f, float g, float q, uint8_t out[20])
{
    float c[5] = { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f };   /* b0 b1 b2 -a1 -a2, normalized */
    if (f < 10.0f) f = 10.0f;
    if (f > 23000.0f) f = 23000.0f;
    if (q < 0.1f) q = 0.1f;
    if (g > 15.0f) g = 15.0f;
    if (g < -15.0f) g = -15.0f;
    bool flat = type <= 2 && fabsf(g) < 0.05f;
    if (!flat && type <= 4) {
        const float w0 = 2.0f * (float)M_PI * f / 48000.0f, cw = cosf(w0), al = sinf(w0) / (2.0f * q);
        const float A = powf(10.0f, g / 40.0f), sa = 2.0f * sqrtf(A) * al;
        float b0, b1, b2, a0, a1, a2;
        switch (type) {
        case 0:  /* low shelf */
            b0 = A * ((A + 1) - (A - 1) * cw + sa); b1 = 2 * A * ((A - 1) - (A + 1) * cw);
            b2 = A * ((A + 1) - (A - 1) * cw - sa);
            a0 = (A + 1) + (A - 1) * cw + sa; a1 = -2 * ((A - 1) + (A + 1) * cw); a2 = (A + 1) + (A - 1) * cw - sa;
            break;
        case 1:  /* peaking */
            b0 = 1 + al * A; b1 = -2 * cw; b2 = 1 - al * A; a0 = 1 + al / A; a1 = -2 * cw; a2 = 1 - al / A;
            break;
        case 2:  /* high shelf */
            b0 = A * ((A + 1) + (A - 1) * cw + sa); b1 = -2 * A * ((A - 1) + (A + 1) * cw);
            b2 = A * ((A + 1) + (A - 1) * cw - sa);
            a0 = (A + 1) - (A - 1) * cw + sa; a1 = 2 * ((A - 1) - (A + 1) * cw); a2 = (A + 1) - (A - 1) * cw - sa;
            break;
        case 3:  /* low pass */
            b0 = (1 - cw) / 2; b1 = 1 - cw; b2 = (1 - cw) / 2; a0 = 1 + al; a1 = -2 * cw; a2 = 1 - al;
            break;
        default: /* high pass */
            b0 = (1 + cw) / 2; b1 = -(1 + cw); b2 = (1 + cw) / 2; a0 = 1 + al; a1 = -2 * cw; a2 = 1 - al;
            break;
        }
        c[0] = b0 / a0; c[1] = b1 / a0; c[2] = b2 / a0; c[3] = -a1 / a0; c[4] = -a2 / a0;
    }
    for (int i = 0; i < 5; i++) {
        float v = c[i] * 536870912.0f;               /* Q29: |coef| < 4 */
        if (v > 2147483000.0f) v = 2147483000.0f;
        if (v < -2147483000.0f) v = -2147483000.0f;
        int32_t x = (int32_t)lrintf(v);
        memcpy(out + 4 * i, &x, 4);
    }
}

bool v5_eq_is_on(void)
{
    return s_pub_on;
}

static void eq_pick_up(void)
{
    uint32_t s1 = __atomic_load_n(&s_seq, __ATOMIC_ACQUIRE);
    if (s1 == s_cur_seq) {
        return;
    }
    if (s1 & 1u) {
        s_deferred++;                               /* writer busy: next call */
        return;
    }
    eq_bank_t b;
    memcpy(&b, &s_pub, sizeof(b));
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    if (__atomic_load_n(&s_seq, __ATOMIC_RELAXED) != s1 || b.nact > V5_EQ_BANDS) {
        s_deferred++;                               /* torn copy: next call */
        return;
    }
    bool was_active[V5_EQ_BANDS] = { false };
    for (int i = 0; i < s_cur.nact; i++) {
        was_active[s_cur.act[i]] = true;
    }
    for (int i = 0; i < b.nact; i++) {
        if (b.act[i] >= V5_EQ_BANDS) {
            s_deferred++;
            return;
        }
        if (!was_active[b.act[i]]) {
            memset(s_z[b.act[i]], 0, sizeof(s_z[0]));
        }
    }
    s_cur = b;
    s_cur_seq = s1;
    s_switches++;
}

static inline float flush_denorm(float v)
{
    return fabsf(v) < EQ_DENORM ? 0.0f : v;
}

static void eq_band(const biquad_t *q, float *z, float *buf, int n)
{
    const float b0 = q->b0, b1 = q->b1, b2 = q->b2, na1 = q->na1, na2 = q->na2;
    float xl1 = z[0], xl2 = z[1], yl1 = z[2], yl2 = z[3];
    float xr1 = z[4], xr2 = z[5], yr1 = z[6], yr2 = z[7];
    for (int i = 0; i < n; i++) {
        const float xl = buf[2 * i];
        const float xr = buf[2 * i + 1];
        const float yl = b0 * xl + b1 * xl1 + b2 * xl2 + na1 * yl1 + na2 * yl2;
        const float yr = b0 * xr + b1 * xr1 + b2 * xr2 + na1 * yr1 + na2 * yr2;
        xl2 = xl1; xl1 = xl; yl2 = yl1; yl1 = yl;
        xr2 = xr1; xr1 = xr; yr2 = yr1; yr1 = yr;
        buf[2 * i] = yl;
        buf[2 * i + 1] = yr;
    }
    z[0] = xl1; z[1] = xl2; z[2] = flush_denorm(yl1); z[3] = flush_denorm(yl2);
    z[4] = xr1; z[5] = xr2; z[6] = flush_denorm(yr1); z[7] = flush_denorm(yr2);
}

void v5_eq_process(int16_t *pcm, int frames)
{
    s_calls++;
    eq_pick_up();
    if (s_cur.nact == 0 || pcm == NULL || frames <= 0) {
        return;                                     /* flat: untouched */
    }
    const uint32_t t0 = esp_cpu_get_cycle_count();
    for (int done = 0; done < frames;) {
        int n = frames - done;
        if (n > EQ_CHUNK) {
            n = EQ_CHUNK;
        }
        int16_t *p = pcm + 2 * done;
        for (int i = 0; i < 2 * n; i++) {
            s_buf[i] = (float)p[i];
        }
        for (int a = 0; a < s_cur.nact; a++) {
            const int k = s_cur.act[a];
            eq_band(&s_cur.bq[k], s_z[k], s_buf, n);
        }
        for (int i = 0; i < 2 * n; i++) {
            float v = s_buf[i];
            v = v > 32767.0f ? 32767.0f : (v < -32768.0f ? -32768.0f : v);
            p[i] = (int16_t)(int32_t)(v + copysignf(0.5f, v));
        }
        done += n;
    }
    const uint32_t cyc = esp_cpu_get_cycle_count() - t0;
    s_cyc_last = cyc;
    if (cyc > s_cyc_max) {
        s_cyc_max = cyc;
    }
}

void v5_eq_log_stats(void)
{
    const uint32_t mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    ESP_LOGI(TAG, "EQ %s (%u bands) | updates %lu published, %lu applied, %lu deferred | calls %lu | "
             "last %lu us, max %lu us",
             s_pub_on ? "ON" : "flat", (unsigned)s_cur.nact,
             (unsigned long)s_published, (unsigned long)s_switches, (unsigned long)s_deferred,
             (unsigned long)s_calls, (unsigned long)(s_cyc_last / mhz), (unsigned long)(s_cyc_max / mhz));
}
