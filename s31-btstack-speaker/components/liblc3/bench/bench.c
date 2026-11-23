/*
 * liblc3 host benchmark / correctness harness (bench only, never in firmware)
 *
 *   bench gen  <out.pcm> [seconds]        Generate the deterministic test signal
 *                                         (48 kHz stereo s16 interleaved)
 *   bench enc  <in.pcm> <out.lc3> [reps]  Encode as the firmware does
 *                                         (2 mono encoders, 10 ms, 100 B), time it
 *   bench cmp  <in.pcm> <a.lc3> <b.lc3>   Decode both streams with the decoder
 *                                         linked in this binary and compare
 *
 * Build the binary twice: once against bench/orig (pristine liblc3) and once
 * against ../src (optimized).  Use the *orig* binary for `cmp` so that both
 * streams are decoded by the reference decoder.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <lc3.h>

#define SR      48000
#define NS      480
#define NBYTES  100

/* ---------------------------------------------------------------------- */

static uint32_t rng = 0x12345678;
static float frand(void)               /* uniform [-1, 1) */
{
    rng = rng * 1664525u + 1013904223u;
    return (int32_t)rng * (1.f / 2147483648.f);
}

static int gen(const char *path, double seconds)
{
    int n = (int)(seconds * SR);
    int16_t *pcm = malloc(n * 2 * sizeof(int16_t));

    /* note sequences (Hz) for left / right */
    static const double notes_l[] = { 220, 261.63, 329.63, 392, 440, 523.25, 196, 293.66 };
    static const double notes_r[] = { 110, 146.83, 164.81, 98, 130.81, 174.61, 123.47, 82.41 };

    double ph_l[8] = { 0 }, ph_r[8] = { 0 };
    float pink_l = 0, pink_r = 0, drum = 0, hat = 0;

    for (int i = 0; i < n; i++) {
        double t = (double)i / SR;
        int note = (int)(t / 0.5) % 8;
        double tn = fmod(t, 0.5);
        double env = (tn < 0.01 ? tn / 0.01 : exp(-(tn - 0.01) * 4));

        /* global level: quiet intro, normal, loud (clipping), silence gap */
        double lvl = t < 0.5 ? 0.05 : t < 3.5 ? 0.35 : t < 4.5 ? 1.1 : 0.4;
        if (t > 5.0 && t < 5.3) lvl = 0;

        double vib = 1 + 0.004 * sin(2 * M_PI * 5.5 * t);
        double l = 0, r = 0;

        for (int h = 0; h < 8; h++) {
            double a = 1.0 / (1 + h * h * 0.6);
            ph_l[h] += 2 * M_PI * notes_l[note] * (h + 1) * vib / SR;
            ph_r[h] += 2 * M_PI * notes_r[note] * (h + 1) * 1.0007 / SR;
            l += a * sin(ph_l[h]);
            r += a * sin(ph_r[h] + 0.3 * h);
        }
        l *= 0.3 * env; r *= 0.3 * (0.4 + 0.6 * env);

        /* chirp in the right channel between 2s and 3s */
        if (t > 2 && t < 3)
            r += 0.15 * sin(2 * M_PI * (200 * (t - 2) + 0.5 * 9000 * (t - 2) * (t - 2)));

        /* pinkish noise floor */
        pink_l = 0.98f * pink_l + 0.02f * frand();
        pink_r = 0.98f * pink_r + 0.02f * frand();
        l += 0.6 * pink_l + 0.004 * frand();
        r += 0.6 * pink_r + 0.004 * frand();

        /* transients: kick every 375 ms, hi-hat (white burst) every 187.5 ms */
        if (i % (SR * 3 / 8) == 0) drum = 1;
        if (i % (SR * 3 / 16) == 0) hat = 0.5f;
        double kick = drum * sin(2 * M_PI * 60 * fmod(t, 0.375) * (1 + 3 * drum));
        drum *= 0.9995f; hat *= 0.997f;
        double hh = hat * frand();
        l += 0.5 * kick + 0.3 * hh;
        r += 0.45 * kick + 0.35 * hh;

        /* a few clicks */
        if (i % 33331 == 0) { l += 0.8; r -= 0.8; }

        double sl = lvl * l * 32767, sr = lvl * r * 32767;
        pcm[2*i+0] = sl > 32767 ? 32767 : sl < -32768 ? -32768 : (int16_t)lrint(sl);
        pcm[2*i+1] = sr > 32767 ? 32767 : sr < -32768 ? -32768 : (int16_t)lrint(sr);
    }

    FILE *fp = fopen(path, "wb");
    if (!fp) { perror(path); return 1; }
    fwrite(pcm, sizeof(int16_t), n * 2, fp);
    fclose(fp);
    printf("generated %s: %d frames (%.1f s stereo)\n", path, n / NS, seconds);
    return 0;
}

/* ---------------------------------------------------------------------- */

static void *load(const char *path, long *size)
{
    FILE *fp = fopen(path, "rb");
    if (!fp) { perror(path); exit(1); }
    fseek(fp, 0, SEEK_END); *size = ftell(fp); fseek(fp, 0, SEEK_SET);
    void *p = malloc(*size);
    fread(p, 1, *size, fp); fclose(fp);
    return p;
}

static double now(void)
{
    struct timespec ts; timespec_get(&ts, TIME_UTC);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static int enc(const char *in, const char *out, int reps)
{
    long size;
    int16_t *pcm = load(in, &size);
    int nframes = size / (2 * NS * sizeof(int16_t));
    uint8_t *bs = malloc(nframes * 2 * NBYTES);

    static lc3_encoder_mem_48k_t mem[2];
    double best = 1e9;

    for (int r = 0; r < reps; r++) {
        lc3_encoder_t e[2];
        for (int ch = 0; ch < 2; ch++)
            e[ch] = lc3_setup_encoder(10000, SR, SR, &mem[ch]);

        double t0 = now();
        for (int f = 0; f < nframes; f++)
            for (int ch = 0; ch < 2; ch++)
                lc3_encode(e[ch], LC3_PCM_FORMAT_S16, pcm + f * 2 * NS + ch, 2,
                           NBYTES, bs + (f * 2 + ch) * NBYTES);
        double t = now() - t0;
        if (t < best) best = t;
    }

    FILE *fp = fopen(out, "wb");
    fwrite(bs, 1, nframes * 2 * NBYTES, fp); fclose(fp);
    printf("encoded %d stereo frames: best %.3f ms total, %.2f us / stereo frame\n",
        nframes, best * 1e3, best * 1e6 / nframes);
    return 0;
}

/* ---------------------------------------------------------------------- */

static int16_t *decode(const uint8_t *bs, int nframes)
{
    static lc3_decoder_mem_48k_t mem[2];
    int16_t *pcm = calloc(nframes * 2 * NS, sizeof(int16_t));
    lc3_decoder_t d[2];
    for (int ch = 0; ch < 2; ch++)
        d[ch] = lc3_setup_decoder(10000, SR, SR, &mem[ch]);
    for (int f = 0; f < nframes; f++)
        for (int ch = 0; ch < 2; ch++)
            lc3_decode(d[ch], bs + (f * 2 + ch) * NBYTES, NBYTES,
                LC3_PCM_FORMAT_S16, pcm + f * 2 * NS + ch, 2);
    return pcm;
}

static double snr_db(const int16_t *ref, const int16_t *x, long n, int delay)
{
    double s = 0, e = 0;
    for (long i = 0; i + delay < n; i++) {
        double r = ref[i], d = (double)x[i + delay] - ref[i];
        s += r * r; e += d * d;
    }
    return e == 0 ? INFINITY : 10 * log10(s / e);
}

static int cmp(const char *in, const char *a, const char *b)
{
    long sa, sb, sp;
    uint8_t *ba = load(a, &sa), *bb = load(b, &sb);
    int16_t *src = load(in, &sp);
    if (sa != sb) { printf("size mismatch %ld / %ld\n", sa, sb); return 1; }

    int nframes = sa / (2 * NBYTES), ndiff = 0;
    for (int f = 0; f < 2 * nframes; f++)
        ndiff += memcmp(ba + f * NBYTES, bb + f * NBYTES, NBYTES) != 0;

    printf("%d / %d channel-frames differ (%s)\n", ndiff, 2 * nframes,
        ndiff ? "NOT bit-exact" : "bit-exact");

    int16_t *pa = decode(ba, nframes), *pb = decode(bb, nframes);
    long n = (long)nframes * NS * 2;

    int delay = 2 * lc3_delay_samples(10000, SR);   /* interleaved samples */
    double snr_a = snr_db(src, pa, n, delay), snr_b = snr_db(src, pb, n, delay);
    double snr_ab = snr_db(pa, pb, n, 0);

    int maxd = 0;
    for (long i = 0; i < n; i++) {
        int d = abs(pa[i] - pb[i]);
        if (d > maxd) maxd = d;
    }

    printf("codec SNR vs source : A %.2f dB, B %.2f dB\n", snr_a, snr_b);
    printf("decoded A vs B      : SNR %.2f dB, max |diff| %d LSB\n", snr_ab, maxd);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc >= 3 && !strcmp(argv[1], "gen"))
        return gen(argv[2], argc > 3 ? atof(argv[3]) : 6.0);
    if (argc >= 4 && !strcmp(argv[1], "enc"))
        return enc(argv[2], argv[3], argc > 4 ? atoi(argv[4]) : 5);
    if (argc >= 5 && !strcmp(argv[1], "cmp"))
        return cmp(argv[2], argv[3], argv[4]);

    fprintf(stderr, "usage: bench gen|enc|cmp ...\n");
    return 1;
}
