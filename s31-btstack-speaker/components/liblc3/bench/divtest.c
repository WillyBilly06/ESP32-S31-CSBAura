/*
 * Exhaustive check of the division-free quotient used in energy.c
 * (bench only, never in firmware):
 *
 *   y = RN(1/n), q = RN(x y), r = fma(-q, n, x), q' = fma(r, y, q)
 *
 * must equal RN(x / n) for every float `x >= 2^-100` and n = [1..80]
 * (below, subnormal residuals break the correction: energy.c falls back
 * to a true division).  Scaling `x` by 2 scales both sides exactly, so
 * testing every mantissa of a few binades covers all such inputs; the
 * lowest and highest binades are tested too.
 *
 *   clang -O2 -mfma divtest.c -o divtest && ./divtest
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

static float f_of(uint32_t u) { float f; memcpy(&f, &u, 4); return f; }
static uint32_t u_of(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }

int main(void)
{
    long long bad = 0, tested = 0;

    for (int n = 1; n <= 80; n++) {
        volatile float vn = (float)n;
        float fn = vn, y = 1.f / fn;

        /* binades: lowest supported, [1,4), and the top ones */
        static const uint32_t ranges[][2] = {
            { 0x0d800000u, 0x0e800000u },     /* [2^-100, 2^-98) */
            { 0x3f800000u, 0x40800000u },     /* [1, 4) */
            { 0x7e800000u, 0x7f800000u },     /* [2^126, inf) */
        };

        for (int r = 0; r < 3; r++) {
            for (uint32_t u = ranges[r][0]; u < ranges[r][1]; u++) {
                float x = f_of(u);
                float ref = x / fn;
                float q = x * y;
                float e = fmaf(-q, fn, x);
                float q1 = fmaf(e, y, q);
                tested++;
                if (u_of(q1) != u_of(ref)) {
                    if (bad < 10)
                        printf("n=%d x=%a: ref %a got %a\n", n, x, ref, q1);
                    bad++;
                }
            }
        }
    }

    printf("%lld / %lld mismatches\n", bad, tested);
    return bad != 0;
}
