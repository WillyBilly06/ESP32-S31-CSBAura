/*
 * LC3 -> I2S player.  Decoded S16 PCM is pushed per channel into stream buffers;
 * a feeder task interleaves L/R, expands to 32-bit slots and writes to I2S.
 * I2S pin config matches the A2DP project (BCLK=2, WS=3, DOUT=4, MCLK unused).
 */
#include "lc3_i2s_player.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "driver/i2s_std.h"
#include "esp_log.h"

#define TAG_P              "LC3_I2S"
#define MAX_FRAME_SAMPLES  480          /* 10 ms @ 48 kHz (worst case) */
#define SB_FRAMES          10           /* stream-buffer depth in frames */

static i2s_chan_handle_t   s_tx;
static StreamBufferHandle_t s_sb_l, s_sb_r;
static volatile int        s_sr_hz, s_frame_samples, s_stereo, s_ready;

static void feeder_task(void *arg)
{
    static int16_t lbuf[MAX_FRAME_SAMPLES];
    static int16_t rbuf[MAX_FRAME_SAMPLES];
    static int32_t out[MAX_FRAME_SAMPLES * 2];

    for (;;) {
        if (!s_ready) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        int n = s_frame_samples;
        size_t bytes = (size_t)n * sizeof(int16_t);

        /* Block on the left channel; it always exists (mono maps to left). */
        if (xStreamBufferReceive(s_sb_l, lbuf, bytes, pdMS_TO_TICKS(100)) < bytes) {
            continue;   /* underrun / not streaming yet */
        }

        if (s_stereo) {
            if (xStreamBufferReceive(s_sb_r, rbuf, bytes, pdMS_TO_TICKS(20)) < bytes) {
                memcpy(rbuf, lbuf, bytes);   /* right lagged this frame: duplicate left */
            }
        } else {
            memcpy(rbuf, lbuf, bytes);       /* mono -> both channels */
        }

        for (int i = 0; i < n; i++) {
            out[2 * i]     = ((int32_t)lbuf[i]) << 16;
            out[2 * i + 1] = ((int32_t)rbuf[i]) << 16;
        }

        size_t wr;
        i2s_channel_write(s_tx, out, (size_t)n * 2 * sizeof(int32_t), &wr, portMAX_DELAY);
    }
}

void lc3_i2s_player_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = 6;
    chan_cfg.dma_frame_num = 240;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_tx, NULL));

    i2s_std_config_t std = {
        .clk_cfg  = {   /* placeholder rate, set in set_format; APLL = exact MCLK, no fractional divider jitter */
            .sample_rate_hz = 48000, .clk_src = I2S_CLK_SRC_APLL, .mclk_multiple = I2S_MCLK_MULTIPLE_256, .bclk_div = 8,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = LC3_I2S_BCLK_PIN,
            .ws   = LC3_I2S_WS_PIN,
            .dout = LC3_I2S_DOUT_PIN,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx, &std));

    s_sb_l = xStreamBufferCreate(MAX_FRAME_SAMPLES * SB_FRAMES * sizeof(int16_t), 1);
    s_sb_r = xStreamBufferCreate(MAX_FRAME_SAMPLES * SB_FRAMES * sizeof(int16_t), 1);

    xTaskCreatePinnedToCore(feeder_task, "lc3_i2s", 4096, NULL,
                            configMAX_PRIORITIES - 3, NULL, 0);

    ESP_LOGI(TAG_P, "I2S out ready: BCLK=%d WS=%d DOUT=%d (32-bit stereo)",
             LC3_I2S_BCLK_PIN, LC3_I2S_WS_PIN, LC3_I2S_DOUT_PIN);
}

void lc3_i2s_player_set_format(int sr_hz, int frame_samples, int stereo)
{
    if (s_ready && sr_hz == s_sr_hz && frame_samples == s_frame_samples && stereo == s_stereo) {
        return;
    }
    if (frame_samples > MAX_FRAME_SAMPLES) {
        frame_samples = MAX_FRAME_SAMPLES;
    }

    if (s_ready) {
        i2s_channel_disable(s_tx);
        s_ready = 0;
    }

    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(sr_hz);
    clk.clk_src = I2S_CLK_SRC_APLL;               /* exact MCLK from the audio PLL */
    ESP_ERROR_CHECK(i2s_channel_reconfig_std_clock(s_tx, &clk));

    s_sr_hz         = sr_hz;
    s_frame_samples = frame_samples;
    s_stereo        = stereo;

    ESP_ERROR_CHECK(i2s_channel_enable(s_tx));
    s_ready = 1;

    ESP_LOGI(TAG_P, "format: %d Hz, %d samples/frame, %s",
             sr_hz, frame_samples, stereo ? "stereo" : "mono");
}

void lc3_i2s_player_push(int side, const int16_t *pcm, int nsamples)
{
    if (!s_ready) {
        return;
    }
    StreamBufferHandle_t sb = (side == LC3_I2S_SIDE_RIGHT) ? s_sb_r : s_sb_l;
    xStreamBufferSend(sb, pcm, (size_t)nsamples * sizeof(int16_t), 0);
}

void lc3_i2s_player_flush(void)
{
    s_ready = 0;   /* feeder idles; buffers drain naturally on next format set */
}
