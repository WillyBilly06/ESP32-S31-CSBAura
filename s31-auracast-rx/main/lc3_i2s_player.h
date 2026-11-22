/*
 * LC3 -> I2S player for the LE Audio broadcast sink.
 *
 * Decoded PCM (S16) is pushed per channel; an internal feeder task interleaves
 * L/R and writes 32-bit stereo frames to I2S. Pins match the A2DP project:
 *   BCLK = GPIO2, WS/LRCK = GPIO3, DOUT = GPIO4, MCLK unused.
 */
#pragma once

#include <stdint.h>

#define LC3_I2S_BCLK_PIN   2   /* same as A2DP CONFIG_EXAMPLE_I2S_BCK_PIN  */
#define LC3_I2S_WS_PIN     3   /* same as A2DP CONFIG_EXAMPLE_I2S_LRCK_PIN */
#define LC3_I2S_DOUT_PIN   4   /* same as A2DP CONFIG_EXAMPLE_I2S_DATA_PIN */

#define LC3_I2S_SIDE_LEFT  0
#define LC3_I2S_SIDE_RIGHT 1

/* Create the I2S channel (clock configured later) and start the feeder task. */
void lc3_i2s_player_init(void);

/* Configure the output format once the broadcast codec config is known.
 * sr_hz: PCM sample rate; frame_samples: samples/channel per LC3 frame;
 * stereo: true if two channels (L+R) will be fed, false for mono. Idempotent. */
void lc3_i2s_player_set_format(int sr_hz, int frame_samples, int stereo);

/* Push one decoded LC3 frame worth of S16 PCM for a channel (LEFT/RIGHT). */
void lc3_i2s_player_push(int side, const int16_t *pcm, int nsamples);

/* Stop output and flush buffers (on stream stop / PA sync lost). */
void lc3_i2s_player_flush(void);
