/******************************************************************************
 *
 *  Copyright 2022 Google LLC
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#include "energy.h"
#include "tables.h"


/**
 * Division of a positive float by a small integer, without division
 * x, n            Dividend (>= 0), and divisor (>= 1)
 * return          x / n, correctly rounded (bit-exact with `x / n`)
 *
 * The reciprocal `y = RN(1/n)` comes from a table, then one correction
 * step (Markstein) gives the correctly rounded quotient :
 *   q = RN(x y),  r = x - q n  (exact with a fused multiply-add),
 *   q' = RN(q + r y)
 * Checked exhaustively for n = [1..80] and x >= 2^-100 (bench/divtest.c).
 * Below 2^-100 the residual may be subnormal, so fall back to a division
 * (except for 0, exact: q = r = 0).
 */
static inline float div_by_int(float x, int n)
{
    static const float inv[81] = {
        0, 1.f/ 1, 1.f/ 2, 1.f/ 3, 1.f/ 4, 1.f/ 5, 1.f/ 6, 1.f/ 7,
        1.f/ 8, 1.f/ 9, 1.f/10, 1.f/11, 1.f/12, 1.f/13, 1.f/14, 1.f/15,
        1.f/16, 1.f/17, 1.f/18, 1.f/19, 1.f/20, 1.f/21, 1.f/22, 1.f/23,
        1.f/24, 1.f/25, 1.f/26, 1.f/27, 1.f/28, 1.f/29, 1.f/30, 1.f/31,
        1.f/32, 1.f/33, 1.f/34, 1.f/35, 1.f/36, 1.f/37, 1.f/38, 1.f/39,
        1.f/40, 1.f/41, 1.f/42, 1.f/43, 1.f/44, 1.f/45, 1.f/46, 1.f/47,
        1.f/48, 1.f/49, 1.f/50, 1.f/51, 1.f/52, 1.f/53, 1.f/54, 1.f/55,
        1.f/56, 1.f/57, 1.f/58, 1.f/59, 1.f/60, 1.f/61, 1.f/62, 1.f/63,
        1.f/64, 1.f/65, 1.f/66, 1.f/67, 1.f/68, 1.f/69, 1.f/70, 1.f/71,
        1.f/72, 1.f/73, 1.f/74, 1.f/75, 1.f/76, 1.f/77, 1.f/78, 1.f/79,
        1.f/80,
    };

    if (n > 80 || (x < 0x1p-100f && x != 0))
        return x / n;

    float y = inv[n], fn = (float)n;
    float q = x * y;
    float r = fmaf(-q, fn, x);
    return fmaf(r, y, q);
}

/**
 * Energy estimation per band
 */
bool lc3_energy_compute(
    enum lc3_dt dt, enum lc3_srate sr, const float *x, float *e)
{
    int nb = lc3_num_bands[dt][sr];
    const int *lim = lc3_band_lim[dt][sr];

    /* Mean the square of coefficients within each band */

    float e_sum[2] = { 0, 0 };
    int iband_h = nb - (const int []){
        [LC3_DT_2M5] = 2, [LC3_DT_5M ] = 3,
        [LC3_DT_7M5] = 4, [LC3_DT_10M] = 2 }[dt];

    for (int iband = 0, i = lim[iband]; iband < nb; iband++) {
        int ie = lim[iband+1];
        int n = ie - i;

        float sx2 = x[i] * x[i];
        for (i++; i < ie; i++)
            sx2 += x[i] * x[i];

        *e = div_by_int(sx2, n);
        e_sum[iband >= iband_h] += *(e++);
    }

    /* Return the near nyquist flag */

    return e_sum[1] > 30 * e_sum[0];
}
