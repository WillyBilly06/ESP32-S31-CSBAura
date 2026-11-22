#pragma once
/*
 * Local speaker output: one I2S channel (APLL, 48 kHz, 32-bit stereo Philips;
 * BCK GPIO 35, LRCK GPIO 39, DOUT GPIO 37) fed from a PCM delay line.
 *
 * Exactly one producer owns the output at a time:
 *   AO_HOST    the Auracast encoder (same PCM as goes on air), delayed so it
 *              plays when the receivers render it (SDU reference + presentation
 *              delay): the S31 and the Xtreme 5 sound together
 *   AO_LOCAL   phone audio with Auracast off (feeder task reads the A2DP ring)
 *   AO_LISTEN  party LISTEN: the decoded master broadcast
 *
 * The writer task (core 0) holds the line at the requested delay: it starts
 * (primes) once that much is buffered, plays silence while not primed, and
 * re-primes after an underrun, so every (re)start comes back in sync. The I2S
 * clock (APLL) and the BT controller both run off the 40 MHz XTAL: no drift
 * between what goes on air and what plays here.
 */
#include <stdbool.h>
#include <stdint.h>

typedef enum { AO_NONE = 0, AO_HOST, AO_LOCAL, AO_LISTEN } ao_owner_t;

void audio_out_setup(void);

/* Any thread. Hand the output to `owner`, flush, play silence until `delay_ms`
 * of audio is buffered. Same owner again: only the delay changes. */
void audio_out_begin(ao_owner_t owner, uint32_t delay_ms);
/* Any thread: let go (if `owner` still has it); output falls back to silence. */
void audio_out_end(ao_owner_t owner);
/* Producer: interleaved L/R int16, n frames. Ignored unless `owner` has it. */
void audio_out_push(ao_owner_t owner, const int16_t *lr, int n);
/* Any thread: target delay in ms while running (e.g. refined lead estimate). */
void audio_out_set_delay(ao_owner_t owner, uint32_t delay_ms);
/* Any thread: sample rate for AO_LISTEN (BASE); host/local are always 48 kHz. */
void audio_out_set_rate(uint32_t hz);

ao_owner_t audio_out_owner(void);
void audio_out_log_stats(void);
