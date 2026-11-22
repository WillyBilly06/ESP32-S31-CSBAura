/*
 * Host test of the A2DP bridge DSP (s31-jbl-speaker/main/a2dp_dsp.h): the same
 * resampler + SPSC ring code the firmware runs, fed the way a2d_data_cb feeds
 * it (random packet sizes, chunked FIR passes, ring writes, 480-frame reads).
 *
 *   clang -O2 -I../s31-jbl-speaker/main rs_host_test.c -o rs_host_test
 *   rs_host_test <outdir>        (tools/rs_host_check.py builds, runs, analyses)
 *
 * Checks done here (pass/fail printed):
 *   1. packetization invariance: random packet sizes == one huge packet, bit-exact
 *   2. new vs old resampler (the previous per-SBC-frame float implementation,
 *      ported below): max difference in LSB on the same input
 *   3. drift continuity: ratio changes every packet (+-0.2 %), output compared
 *      with the ideal sine at the exact positions the steps imply
 *   4. ring integrity: everything written comes out in order across wraps
 * Writes raw s16 stereo files for the spectral checks (spur levels) in Python.
 */
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "a2dp_dsp.h"

#define OUT_RATE   48000
#define IN_RATE    44100
#define RB_FRAMES  (OUT_RATE / 1000 * 400)
#define TARGET_FRAMES (OUT_RATE / 1000 * 150)

static uint32_t rnd_state = 12345;
static uint32_t rnd(void) { rnd_state = rnd_state * 1664525u + 1013904223u; return rnd_state >> 8; }

/* ---- firmware path (mirror of a2d_data_cb, minus the ESP bits) ---------- */
static rs_state_t S;
static uint32_t rs_out[RS_OUT_MAX];
static spsc_ring_t R;

/* returns frames produced; e = fill error override (NAN: use the real fill) */
static int feed(const int16_t *in, int n, float e_override, uint64_t *step_used)
{
    uint32_t space = ring_space(&R);
    uint32_t fill = RB_FRAMES - space;
    float e = isnan(e_override) ? ((float)fill - TARGET_FRAMES) / TARGET_FRAMES : e_override;
    uint64_t step = rs_step_q32(IN_RATE, OUT_RATE, e);
    if (step_used) *step_used = step;
    uint32_t need = (uint32_t)(((uint64_t)n << 32) / step) + 3;
    if (need > space) {
        fprintf(stderr, "unexpected drop\n");
        exit(2);
    }
    int total = 0;
    while (n > 0) {
        int m = rs_room(&S);
        if (m > n) m = n;
        rs_push(&S, in, m);
        in += 2 * m;
        n -= m;
        int o = rs_run(&S, step, rs_out, RS_OUT_MAX);
        if (o > 0) ring_write(&R, rs_out, o);
        total += o;
    }
    if ((uint32_t)total > need) {
        fprintf(stderr, "need bound violated: %d > %u\n", total, need);
        exit(2);
    }
    return total;
}

/* ---- the previous implementation (a2dp_bridge.c before this change) ------ */
#define OLD_HIST 4096
static float o_hist[2][OLD_HIST];
static int o_hlen, o_rpos_i;
static float o_rpos_f;
static void old_reset(void) { memset(o_hist, 0, sizeof o_hist); o_hlen = RS_TAPS; o_rpos_i = RS_HALF - 1; o_rpos_f = 0; }
static int old_feed(const int16_t *in, int n, float stepf, int16_t *out)
{
    int o = 0;
    for (int j = 0; j < n; j++) { o_hist[0][o_hlen + j] = in[2 * j]; o_hist[1][o_hlen + j] = in[2 * j + 1]; }
    o_hlen += n;
    const int step_i = (int)stepf;
    const float step_f = stepf - (float)step_i;
    int i = o_rpos_i;
    float fr = o_rpos_f;
    while (i + RS_HALF < o_hlen) {
        float ph = fr * RS_PHASES;
        int p0 = (int)ph;
        float t = ph - (float)p0;
        const float *k0 = rs_kern[p0], *k1 = rs_kern[p0 + 1];
        const float *x0 = &o_hist[0][i - (RS_HALF - 1)], *x1 = &o_hist[1][i - (RS_HALF - 1)];
        float a0 = 0, a1 = 0, b0 = 0, b1 = 0;
        for (int k = 0; k < RS_TAPS; k++) { a0 += k0[k] * x0[k]; a1 += k0[k] * x1[k]; b0 += k1[k] * x0[k]; b1 += k1[k] * x1[k]; }
        float acc0 = a0 + (b0 - a0) * t, acc1 = a1 + (b1 - a1) * t;
        if (acc0 > 32767.f) acc0 = 32767.f; else if (acc0 < -32768.f) acc0 = -32768.f;
        if (acc1 > 32767.f) acc1 = 32767.f; else if (acc1 < -32768.f) acc1 = -32768.f;
        out[2 * o] = (int16_t)lrintf(acc0);
        out[2 * o + 1] = (int16_t)lrintf(acc1);
        o++;
        i += step_i;
        fr += step_f;
        if (fr >= 1.0f) { fr -= 1.0f; i++; }
    }
    int keep_from = i - (RS_HALF - 1);
    if (keep_from > 0) {
        o_hlen -= keep_from;
        memmove(o_hist[0], o_hist[0] + keep_from, o_hlen * sizeof(float));
        memmove(o_hist[1], o_hist[1] + keep_from, o_hlen * sizeof(float));
        i -= keep_from;
    }
    o_rpos_i = i;
    o_rpos_f = fr;
    return o;
}

/* ---- helpers -------------------------------------------------------------- */
static void make_input(int16_t *x, int n, double fl1, double fl2, double fr1, double fr2)
{
    for (int j = 0; j < n; j++) {
        double t = (double)j / IN_RATE;
        x[2 * j]     = (int16_t)lrint(32767.0 * (0.4 * sin(2 * M_PI * fl1 * t) + 0.4 * sin(2 * M_PI * fl2 * t)));
        x[2 * j + 1] = (int16_t)lrint(32767.0 * (0.4 * sin(2 * M_PI * fr1 * t) + 0.4 * sin(2 * M_PI * fr2 * t)));
    }
}

static int rand_pkt(void)
{
    /* mostly whole SBC packets (1..15 frames of 128), sometimes odd sizes */
    return (rnd() % 4) ? (int)(1 + rnd() % 15) * 128 : (int)(1 + rnd() % 2000);
}

static void ring_init(void)
{
    static uint32_t *buf;
    if (!buf) buf = malloc((RB_FRAMES + 1) * 4);
    R.buf = buf; R.slots = RB_FRAMES + 1; R.wr = R.rd = 0;
}

/* Run the firmware path over n input frames; reader drains 480-frame blocks.
 * pkt_mode 0: one packet; 1: random packets. Returns output frames. */
static int run_path(const int16_t *x, int n, int pkt_mode, float e_fixed, int16_t *y, int ymax)
{
    rs_reset(&S);
    ring_init();
    int got = 0, pos = 0;
    while (pos < n) {
        int m = pkt_mode ? rand_pkt() : (n - pos);
        if (m > n - pos) m = n - pos;
        if (m > 8000) m = 8000;                /* + reader hold stays below the ring size */
        feed(x + 2 * pos, m, e_fixed, NULL);
        pos += m;
        /* reader: 480-frame blocks, leave a random amount buffered */
        uint32_t hold = rnd() % 6000;
        while (ring_fill(&R) >= 480 + hold && got + 480 <= ymax) {
            ring_read(&R, (uint32_t *)(y + 2 * got), 480);
            got += 480;
        }
    }
    uint32_t rest = ring_fill(&R);
    if (got + (int)rest > ymax) rest = ymax - got;
    ring_read(&R, (uint32_t *)(y + 2 * got), rest);
    return got + rest;
}

static void write_raw(const char *dir, const char *name, const int16_t *y, int frames)
{
    char p[512];
    snprintf(p, sizeof p, "%s/%s", dir, name);
    FILE *f = fopen(p, "wb");
    if (!f) { perror(p); exit(1); }
    fwrite(y, 4, frames, f);
    fclose(f);
}

int main(int argc, char **argv)
{
    const char *dir = argc > 1 ? argv[1] : ".";
    int fails = 0;
    rs_init_kernel();

    const int N = IN_RATE * 3;                 /* 3 s */
    const int YMAX = N * 2;
    int16_t *x = malloc(N * 4), *y1 = malloc(YMAX * 4), *y2 = malloc(YMAX * 4), *yo = malloc(YMAX * 4);

    /* ---- spur test signals: L 1k+10k, R 5k+15k, nominal ratio -------------- */
    make_input(x, N, 1000, 10000, 5000, 15000);
    int n_rand = run_path(x, N, 1, 0.0f, y1, YMAX);
    rnd_state = 999;
    int n_one = run_path(x, N, 0, 0.0f, y2, YMAX);
    write_raw(dir, "spur_new.raw", y1, n_rand);

    /* 1. packetization invariance */
    int same = n_rand == n_one && memcmp(y1, y2, (size_t)n_one * 4) == 0;
    printf("[%s] packetization invariance: random packets vs 8000-frame packets: %d vs %d frames, %s\n",
           same ? "PASS" : "FAIL", n_rand, n_one, same ? "bit-exact" : "DIFFERENT");
    fails += !same;

    /* 2. old implementation, fed per SBC frame (128) as before */
    old_reset();
    int no = 0;
    const float stepf = (float)IN_RATE / OUT_RATE;
    for (int pos = 0; pos < N; pos += 128) {
        int m = N - pos < 128 ? N - pos : 128;
        no += old_feed(x + 2 * pos, m, stepf, yo + 2 * no);
    }
    write_raw(dir, "spur_old.raw", yo, no);
    /* The old code stepped its fraction in float (fr += step_f), which drifts
     * ~1e-8 sample per output (a 3e-8 relative rate error); the new Q32 phase is
     * exact. So compare where that drift is still far below an LSB: the
     * first 256 frames. (Accuracy of the full run is checked in test 3.) */
    int cmpn = no < n_rand ? no : n_rand, maxd = 0, ndiff = 0, maxd_all = 0;
    for (int j = 0; j < 2 * cmpn; j++) {
        int d = abs(yo[j] - y1[j]);
        if (d > maxd_all) maxd_all = d;
        if (j >= 2 * 256) continue;
        if (d) ndiff++;
        if (d > maxd) maxd = d;
    }
    printf("[%s] new vs old resampler, first 256 frames: max diff %d LSB, %d of %d samples differ "
           "(whole 3 s: max %d LSB, from the old float phase accumulator's drift)\n",
           maxd <= 1 ? "PASS" : "FAIL", maxd, ndiff, 2 * 256, maxd_all);
    fails += maxd > 1;

    /* ---- tone sweep for the spectral check (nominal ratio) -------------------- */
    for (int f = 1000; f <= 19000; f += 1000) {
        make_input(x, N, f, f, f + 500, f + 500);   /* L: one tone at 0.8 FS, R: f+500 */
        int n = run_path(x, N, 1, 0.0f, y1, YMAX);
        char name[64];
        snprintf(name, sizeof name, "sweep_%05d.raw", f);
        write_raw(dir, name, y1, n);
    }

    /* 3. drift continuity: ratio changes every packet; every output is compared
     *    with (a) the same windowed-sinc kernel evaluated exactly in double at
     *    the exact position the Q32 steps imply (implementation error: table
     *    interpolation, float, chunking, ring) and (b) the ideal sine. */
    {
        const double fsig[2] = {1000.0, 10000.0};
        const double fc = 20000.0 / 44100.0;
        for (int c = 0; c < 2; c++) {
            make_input(x, N, fsig[c], fsig[c], fsig[c], fsig[c]);   /* 0.8 FS single tone */
            rs_reset(&S);
            ring_init();
            double P = (RS_HALF - 1) - RS_TAPS;    /* input position of output 0 */
            double se = 0, maxe = 0, se2 = 0, maxe2 = 0;
            long cnt = 0, outn = 0;
            int pos = 0;
            while (pos < N) {
                int m = rand_pkt();
                if (m > N - pos) m = N - pos;
                float e = (float)((int)(rnd() % 2001) - 1000) / 1000.0f;   /* -1..1 each packet */
                uint64_t step;
                int o = feed(x + 2 * pos, m, e, &step);
                pos += m;
                static uint32_t tmp[RS_OUT_MAX * 8];
                while (o > 0) {
                    int k = o > RS_OUT_MAX * 8 ? RS_OUT_MAX * 8 : o;
                    ring_read(&R, tmp, k);
                    for (int j = 0; j < k; j++) {
                        int16_t l = (int16_t)(tmp[j] & 0xFFFF);
                        if (outn > 200 && P < N - 64) {      /* skip start-up and the tail */
                            int ii = (int)floor(P);
                            double frac = P - ii, acc = 0, wsum = 0;
                            for (int kk = 0; kk < RS_TAPS; kk++) {
                                double t = (double)kk - (RS_HALF - 1) - frac;
                                double xx = 2.0 * fc * t;
                                double sn = fabs(xx) < 1e-9 ? 1.0 : sin(M_PI * xx) / (M_PI * xx);
                                double w = 0.42 + 0.5 * cos(M_PI * t / RS_HALF) + 0.08 * cos(2.0 * M_PI * t / RS_HALF);
                                if (fabs(t) >= RS_HALF) w = 0;
                                int ji = ii - (RS_HALF - 1) + kk;
                                double xv = (ji >= 0 && ji < N) ? x[2 * ji] : 0.0;
                                acc += sn * w * xv;
                                wsum += sn * w;
                            }
                            double ref = acc / wsum;
                            double ideal = 32767.0 * 0.8 * sin(2 * M_PI * fsig[c] * P / IN_RATE);
                            double err = l - ref, err2 = l - ideal;
                            se += err * err;
                            se2 += err2 * err2;
                            if (fabs(err) > maxe) maxe = fabs(err);
                            if (fabs(err2) > maxe2) maxe2 = fabs(err2);
                            cnt++;
                        }
                        P += (double)step / 4294967296.0;
                        outn++;
                    }
                    o -= k;
                }
            }
            const double arms = 32767.0 * 0.8 / sqrt(2), apk = 32767.0 * 0.8;
            double db_rms = 20 * log10(sqrt(se / cnt) / arms), db_max = 20 * log10(maxe / apk);
            double db_rms2 = 20 * log10(sqrt(se2 / cnt) / arms), db_max2 = 20 * log10(maxe2 / apk);
            int ok = db_max < -80.0 && db_rms < -85.0;
            printf("[%s] drift continuity %5.0f Hz, ratio re-chosen every packet (+-0.2 %%), %ld samples:\n"
                   "       vs exact-kernel reference: rms %.1f dB, peak %.1f dB (%.2f LSB)\n"
                   "       vs ideal sine:             rms %.1f dB, peak %.1f dB (%.2f LSB)\n",
                   ok ? "PASS" : "FAIL", fsig[c], cnt, db_rms, db_max, maxe, db_rms2, db_max2, maxe2);
            fails += !ok;
        }
    }

    /* 4. ring integrity across wraps with uneven writes/reads */
    {
        ring_init();
        uint32_t wseq = 0, rseq = 0, buf[5000];
        int bad = 0;
        for (int it = 0; it < 200000; it++) {
            uint32_t n = rnd() % 3000;
            if (n <= ring_space(&R)) {
                for (uint32_t j = 0; j < n; j++) buf[j] = wseq++;
                ring_write(&R, buf, n);
            }
            n = rnd() % 3000;
            if (n <= ring_fill(&R)) {
                ring_read(&R, buf, n);
                for (uint32_t j = 0; j < n; j++) bad += buf[j] != rseq++;
            }
        }
        printf("[%s] SPSC ring: %u frames through a %u-frame ring (%u wraps), %d out of order\n",
               bad ? "FAIL" : "PASS", rseq, RB_FRAMES, rseq / (RB_FRAMES + 1), bad);
        fails += bad != 0;
    }

    printf("%s\n", fails ? "SOME CHECKS FAILED" : "all C checks passed");
    return fails ? 1 : 0;
}
