/* Local I2S speaker output (see audio_out.h). */
#include "audio_out.h"

#include <stdlib.h>
#include <string.h>

#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "a2dp_dsp.h"
#include "a2dp_in.h"
#include "auracast.h"
#include "party.h"
#include "v5_eq.h"

static const char *TAG = "I2S_OUT";

#define I2S_BCK_PIN      35
#define I2S_LRCK_PIN     39
#define I2S_DOUT_PIN     37
#define OUT_RATE         48000
#define CHUNK            240                     /* frames per I2S write (5 ms) */
#define DMA_DESC         4
#define DMA_FRAMES       240                     /* 4 x 5 ms = 20 ms in DMA */
#define RING_MS          400
#define LOCAL_DELAY_MS   60                      /* Auracast off: A2DP ring already absorbs the jitter */
#define LOCAL_DELAY_LE_MS 30                     /* LE Audio: steady ISO delivery, low latency */

static i2s_chan_handle_t s_i2s;
static spsc_ring_t s_ring;
static volatile ao_owner_t s_owner;
static volatile uint32_t s_delay_frames;         /* prime level: target delay minus what DMA holds */
static volatile uint32_t s_gen, s_gen_seen;      /* begin() -> writer flushes */
static volatile uint32_t s_rate = OUT_RATE, s_rate_cur = OUT_RATE;
static bool s_primed;
/* delay servo: fill error vs target, averaged over the producer's bursts
 * (the encoder pushes 3 x 10 ms per 30 ms BIG event) */
static float s_err_ema;                         /* frames */
static uint32_t s_since_prime, s_hold;
static volatile int32_t st_err_frames;
static volatile uint32_t st_trims, st_coarse;
#define SERVO_SETTLE   200                      /* chunks (1 s) before the one-off coarse fix */
#define COARSE_FRAMES  (5 * 48)                 /* > 5 ms off after settling: jump once */
#define FINE_FRAMES    48                       /* > 1 ms: drop / repeat 1 frame per 5 ms chunk */
static volatile uint32_t st_underruns, st_primes, st_pushed, st_dropped, st_written;
static volatile int64_t s_last_push_us;          /* producer alive? (a stop is not an underrun) */

/* prompt tone, mixed in by the writer (lock-free hand-over by sequence number) */
#define TONE_GAIN 0.7f
static const int16_t *volatile s_tone_pcm;
static volatile uint32_t s_tone_len, s_tone_rate, s_tone_seq;
static volatile bool s_tone_cut;
static volatile uint32_t st_tones;

void audio_out_tone(const int16_t *pcm, uint32_t samples, uint32_t rate, bool cut)
{
    s_tone_pcm = NULL;                            /* writer stops reading the old one first */
    s_tone_cut = cut;
    s_tone_len = samples;
    s_tone_rate = rate;
    s_tone_pcm = pcm;
    s_tone_seq++;
}

/* writer task: mix the prompt into one chunk at the line rate (linear interpolation) */
static void tone_mix(int32_t *out, int frames, uint32_t line_rate)
{
    static uint32_t seq;
    static const int16_t *pcm;
    static uint32_t len;
    static uint64_t pos, step;                    /* Q16 position / step in tone samples */
    static float music;                           /* 0 = cut, 0.5 = ducked */
    if (seq != s_tone_seq) {
        seq = s_tone_seq;
        music = s_tone_cut ? 0.0f : 0.5f;
        pcm = s_tone_pcm;
        len = s_tone_len;
        step = ((uint64_t)s_tone_rate << 16) / (line_rate ? line_rate : 48000);
        pos = 0;
        if (pcm) st_tones++;
    }
    if (pcm == NULL) return;
    for (int i = 0; i < frames; i++) {
        uint32_t k = (uint32_t)(pos >> 16);
        if (k + 1 >= len) {
            pcm = NULL;                           /* done */
            return;
        }
        int32_t fr = (int32_t)(pos & 0xFFFF);
        int32_t t = pcm[k] + (((int32_t)pcm[k + 1] - pcm[k]) * fr >> 16);
        float tv = (float)t * TONE_GAIN;
        for (int c = 0; c < 2; c++) {
            float v = (float)(out[2 * i + c] >> 16) * music + tv;
            if (v > 32767.0f) v = 32767.0f;
            if (v < -32768.0f) v = -32768.0f;
            out[2 * i + c] = ((int32_t)v) << 16;
        }
        pos += step;
    }
}

static uint32_t delay_to_frames(uint32_t ms)
{
    uint32_t dma_ms = DMA_DESC * DMA_FRAMES * 1000u / OUT_RATE;
    uint32_t f = (ms > dma_ms ? ms - dma_ms : 0) * (s_rate / 1000u);
    uint32_t max = s_ring.slots - 1 - CHUNK;
    return f > max ? max : f;
}

void audio_out_begin(ao_owner_t owner, uint32_t delay_ms)
{
    if (s_owner != owner) {
        s_owner = owner;
        s_gen++;                                  /* writer flushes the line on its side */
        ESP_LOGW(TAG, "output -> %s, delay %lu ms", owner == AO_HOST ? "HOST (in sync with the broadcast)" :
                 owner == AO_LOCAL ? "LOCAL (Auracast off)" : owner == AO_LISTEN ? "LISTEN (party master)" : "none",
                 (unsigned long)delay_ms);
    }
    s_delay_frames = delay_to_frames(delay_ms);
}

void audio_out_set_delay(ao_owner_t owner, uint32_t delay_ms)
{
    if (s_owner == owner) {
        s_delay_frames = delay_to_frames(delay_ms);
    }
}

void audio_out_end(ao_owner_t owner)
{
    if (s_owner == owner) {
        s_owner = AO_NONE;
        s_gen++;
    }
}

ao_owner_t audio_out_owner(void) { return s_owner; }

void audio_out_set_rate(uint32_t hz)
{
    if (hz == 8000 || hz == 16000 || hz == 24000 || hz == 32000 || hz == 44100 || hz == 48000) {
        s_rate = hz;
    }
}

void audio_out_push(ao_owner_t owner, const int16_t *lr, int n)
{
    if (owner != s_owner || n <= 0 || s_gen != s_gen_seen) {
        return;                                   /* not ours, or the writer has not flushed yet */
    }
    static int16_t vol_buf[480 * 2];
    if ((owner == AO_HOST || owner == AO_LOCAL) && n <= 480) {
        memcpy(vol_buf, lr, (size_t)n * 4);       /* phone volume: here, not on the broadcast */
        a2dp_in_apply_volume(vol_buf, n);         /* EQ: in the writer (HOST: in the encoder path) */
        lr = vol_buf;
    }
    uint32_t space = ring_space(&s_ring);
    if ((uint32_t)n > space) {                    /* producer ahead (should not happen): keep the newest */
        st_dropped += ring_drop_oldest(&s_ring, (uint32_t)n - space);
    }
    ring_write(&s_ring, (const uint32_t *)lr, (uint32_t)n);
    st_pushed += (uint32_t)n;
    s_last_push_us = esp_timer_get_time();
}

static void writer_task(void *arg)
{
    (void)arg;
    static int32_t out[CHUNK * 2];
    static uint32_t in[CHUNK + 1];
    for (;;) {
        if (s_gen != s_gen_seen) {                /* new owner: flush, re-prime */
            ring_skip(&s_ring, ring_fill(&s_ring));
            s_primed = false;
            s_gen_seen = s_gen;
        }
        if (s_rate != s_rate_cur) {
            i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(s_rate);
            clk.clk_src = I2S_CLK_SRC_APLL;
            clk.mclk_multiple = I2S_MCLK_MULTIPLE_256;
            i2s_channel_disable(s_i2s);
            esp_err_t e = i2s_channel_reconfig_std_clock(s_i2s, &clk);
            i2s_channel_enable(s_i2s);
            ESP_LOGW(TAG, "sample rate %lu Hz: %s", (unsigned long)s_rate, esp_err_to_name(e));
            s_rate_cur = s_rate;
        }
        uint32_t fill = ring_fill(&s_ring);
        if (!s_primed && s_owner != AO_NONE && fill >= s_delay_frames + CHUNK) {
            if (fill > s_delay_frames + CHUNK) {   /* start exactly at the target delay */
                ring_skip(&s_ring, fill - s_delay_frames - CHUNK);
            }
            s_primed = true;
            st_primes++;
            s_err_ema = 0.0f;
            s_since_prime = 0;
            s_hold = 0;
        }
        if (s_primed) {
            float err = (float)ring_fill(&s_ring) - (float)(s_delay_frames + CHUNK);
            s_err_ema += (err - s_err_ema) * 0.02f;   /* ~250 ms */
            st_err_frames = (int32_t)s_err_ema;
            /* AO_LOCAL is pulled on demand by this line (no second clock): no servo -
             * trimming it only made the A2DP ring drain 0.4 % fast */
            /* once after settling, and again whenever the line stays > 20 ms off for 1 s
             * (a skipped BIG event moved it more than the 1-frame trims fix in seconds) */
            static uint16_t far_chunks;
            bool far = s_err_ema > 4 * COARSE_FRAMES || s_err_ema < -4 * COARSE_FRAMES;
            far_chunks = (far && s_since_prime > SERVO_SETTLE) ? far_chunks + 1 : 0;
            bool refix = far_chunks >= SERVO_SETTLE;
            if (refix) far_chunks = 0;
            if ((++s_since_prime == SERVO_SETTLE || refix) && s_owner != AO_LOCAL && (s_err_ema > COARSE_FRAMES || s_err_ema < -COARSE_FRAMES)) {
                if (s_err_ema > 0) {                  /* too late: jump ahead once */
                    uint32_t k = (uint32_t)s_err_ema;
                    if (k > ring_fill(&s_ring) - CHUNK) k = ring_fill(&s_ring) - CHUNK;
                    ring_skip(&s_ring, k);
                    s_err_ema -= (float)k;
                } else {                              /* too early: hold (silence) once */
                    s_hold = (uint32_t)(-s_err_ema);
                    s_err_ema = 0.0f;
                }
                st_coarse++;
            }
        }
        int adj = 0;                                  /* +1 drop a frame, -1 repeat one */
        if (s_primed && s_since_prime > SERVO_SETTLE && s_owner != AO_LOCAL) {
            if (s_err_ema > FINE_FRAMES) adj = 1;
            else if (s_err_ema < -FINE_FRAMES) adj = -1;
        }
        if (s_primed && s_hold >= CHUNK) {
            s_hold -= CHUNK;
            memset(out, 0, sizeof(out));
        } else if (s_primed && ring_fill(&s_ring) >= CHUNK + 1) {
            uint32_t take = (uint32_t)(CHUNK + adj);
            ring_read(&s_ring, in, take);
            /* JBL One EQ here, after the delay line: a change is heard within one DMA
             * period instead of after delay + DMA (~80 ms). HOST has it in the encoder
             * path (the broadcast carries it); LISTEN: our EQ on the party audio. */
            ao_owner_t ow = s_owner;
            if (ow == AO_LOCAL || (ow == AO_LISTEN && s_rate_cur == OUT_RATE)) {
                v5_eq_process((int16_t *)in, take < CHUNK ? (int)take : CHUNK);
            }
            const int16_t *p = (const int16_t *)in;
            for (int i = 0; i < CHUNK * 2; i++) {
                out[i] = ((int32_t)p[i]) << 16;           /* drop: the extra frame is simply skipped */
            }
            if (adj < 0) {                                /* repeat: last frame twice */
                out[2 * CHUNK - 2] = out[2 * CHUNK - 4];
                out[2 * CHUNK - 1] = out[2 * CHUNK - 3];
            }
            if (adj != 0) st_trims++;
            s_hold = 0;
        } else {
            if (s_primed) {                       /* ran dry: re-prime, which re-syncs */
                s_primed = false;
                if (s_owner != AO_NONE && esp_timer_get_time() - s_last_push_us < 50000) {
                    st_underruns++;               /* producer still feeding: a real underrun */
                }
            }
            memset(out, 0, sizeof(out));
        }
        tone_mix(out, CHUNK, s_rate_cur);
        size_t wr = 0;
        i2s_channel_write(s_i2s, out, sizeof(out), &wr, portMAX_DELAY);   /* DMA paces this task */
        st_written += CHUNK;
    }
}

/* Auracast off and the phone is playing: nobody else reads the A2DP ring, so
 * this does, in real time (paced by the delay line). */
static void local_feed_task(void *arg)
{
    (void)arg;
    static int16_t pcm[480 * 2];
    for (;;) {
        bool want = a2dp_in_streaming() && auracast_state() == AURACAST_STATE_OFF &&
                    (s_owner == AO_NONE || s_owner == AO_LOCAL) &&
                    (!party_auracast_enabled() || party_host_failed()) &&
                    !party_assist_active();       /* a joined broadcast wins over the phone */
        if (!want) {
            if (s_owner == AO_LOCAL) {
                audio_out_end(AO_LOCAL);
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (s_owner != AO_LOCAL) {
            audio_out_begin(AO_LOCAL, a2dp_in_ext_active() ? LOCAL_DELAY_LE_MS : LOCAL_DELAY_MS);
            vTaskDelay(pdMS_TO_TICKS(5));         /* writer flushes first */
        }
        /* keep the line at its delay: read 10 ms whenever it is below delay + 10 ms */
        while (s_owner == AO_LOCAL && s_gen == s_gen_seen &&
               ring_fill(&s_ring) < s_delay_frames + CHUNK + 480) {
            a2dp_in_read(pcm, 480);
            audio_out_push(AO_LOCAL, pcm, 480);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void audio_out_setup(void)
{
    uint32_t slots = (OUT_RATE / 1000) * RING_MS + 1;
    s_ring.buf = heap_caps_malloc(slots * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_ring.slots = slots;

    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    cc.dma_desc_num = DMA_DESC;
    cc.dma_frame_num = DMA_FRAMES;
    cc.auto_clear_after_cb = true;                /* underrun -> silence, never a repeated buffer */
    i2s_std_config_t std = {
        .clk_cfg = {                              /* APLL: exact 48 kHz off the XTAL, like the BT clock */
            .sample_rate_hz = OUT_RATE, .clk_src = I2S_CLK_SRC_APLL, .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = { .mclk = I2S_GPIO_UNUSED, .bclk = I2S_BCK_PIN, .ws = I2S_LRCK_PIN,
                      .dout = I2S_DOUT_PIN, .din = I2S_GPIO_UNUSED },
    };
    esp_err_t err = i2s_new_channel(&cc, &s_i2s, NULL);
    if (err == ESP_OK) err = i2s_channel_init_std_mode(s_i2s, &std);
    if (err == ESP_OK) err = i2s_channel_enable(s_i2s);
    ESP_LOGW(TAG, "I2S out %s: 48 kHz APLL, 32-bit stereo, BCK %d LRCK %d DOUT %d",
             err == ESP_OK ? "ready" : esp_err_to_name(err), I2S_BCK_PIN, I2S_LRCK_PIN, I2S_DOUT_PIN);
    if (err != ESP_OK) {
        return;
    }
    /* core 0, above the LC3 encoder: DMA must never starve */
    xTaskCreatePinnedToCore(writer_task, "i2s_out", 2560, NULL, configMAX_PRIORITIES - 3, NULL, 0);
    xTaskCreatePinnedToCore(local_feed_task, "i2s_local", 3072, NULL, configMAX_PRIORITIES - 7, NULL, 0);
}

void audio_out_log_stats(void)
{
    static const char *const names[] = { "idle", "HOST", "LOCAL", "LISTEN" };
    ESP_LOGI(TAG, "I2S 2s: %s, %s, line %lu ms (target %lu ms + 20 DMA), sync error %ld.%ld ms | trims %lu, coarse %lu | underruns %lu, primes %lu, dropped %lu | tones %lu",
             names[s_owner], s_primed ? "playing" : "priming/silent",
             (unsigned long)(ring_fill(&s_ring) / (s_rate_cur / 1000u)),
             (unsigned long)(s_delay_frames / (s_rate_cur / 1000u)),
             (long)(st_err_frames * 10 / (int32_t)(s_rate_cur / 1000u)) / 10,
             labs((long)(st_err_frames * 10 / (int32_t)(s_rate_cur / 1000u)) % 10),
             (unsigned long)st_trims, (unsigned long)st_coarse,
             (unsigned long)st_underruns, (unsigned long)st_primes, (unsigned long)st_dropped,
             (unsigned long)st_tones);
}
