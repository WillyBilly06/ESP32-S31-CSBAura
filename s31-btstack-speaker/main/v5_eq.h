/*
 * JBL One app EQ: 7 cascaded biquads per channel with the coefficients the
 * app sends (EqSetPortable 0x0E7F, Q29), applied to the 48 kHz stereo PCM the
 * Auracast encoder broadcasts.
 *
 * One writer (BTstack thread) publishes a coefficient bank through a seqlock;
 * the single reader (v5_eq_process on core 0) picks it up at the start of a
 * call when the bank is stable, otherwise keeps the previous bank for that
 * call. Neither side ever waits.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define V5_EQ_BANDS      7
#define V5_EQ_COEF_BYTES (V5_EQ_BANDS * 5 * 4)   /* 7 x {b0,b1,b2,-a1,-a2} int32 LE Q29 = 140 */

/* BTstack thread only (single writer). An all-identity set turns the EQ off. */
void v5_eq_set_q29(const uint8_t coef[V5_EQ_COEF_BYTES]);

/* Any thread: true if the last published bank is not flat. */
bool v5_eq_is_on(void);

/* Core 0, encoder only (single reader): in place, 48 kHz stereo interleaved. */
void v5_eq_process(int16_t *pcm, int frames);

/* Any thread. */
void v5_eq_log_stats(void);

#ifdef __cplusplus
}
#endif
