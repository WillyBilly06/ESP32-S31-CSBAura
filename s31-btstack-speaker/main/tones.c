/* Prompt tones from the "tones" flash partition (see tones.h). */
#include "lls.h"
#include "tones.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_partition.h"

#include "audio_out.h"
#include "jbl_v5.h"

static const char *TAG = "TONES";

typedef struct {
    char name[12];
    uint32_t offset;
    uint32_t samples;
} tone_entry_t;

static const uint8_t *s_map;        /* partition mapped into the data cache: played in place */
static const tone_entry_t *s_idx;
static uint32_t s_count, s_rate, s_size;

void tones_setup(void)
{
    const esp_partition_t *p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "tones");
    if (p == NULL) {
        ESP_LOGW(TAG, "no 'tones' partition - prompts off");
        return;
    }
    const void *ptr;
    esp_partition_mmap_handle_t h;
    if (esp_partition_mmap(p, 0, p->size, ESP_PARTITION_MMAP_DATA, &ptr, &h) != ESP_OK) {
        ESP_LOGE(TAG, "cannot map the 'tones' partition");
        return;
    }
    const uint8_t *m = ptr;
    uint32_t count, rate;
    memcpy(&count, m + 8, 4);
    memcpy(&rate, m + 12, 4);
    if (memcmp(m, "S31TONE1", 8) != 0 || count == 0 || count > 64 || rate < 8000 || rate > 48000) {
        ESP_LOGW(TAG, "'tones' partition is empty - write it with tools/tones_pack.py (prompts off)");
        esp_partition_munmap(h);
        return;
    }
    s_map = m;
    s_idx = (const tone_entry_t *)(m + 16);
    s_count = count;
    s_rate = rate;
    s_size = p->size;
    ESP_LOGW(TAG, "%lu prompt tones at %lu Hz in flash (0x%lx)", (unsigned long)count, (unsigned long)rate,
             (unsigned long)p->address);
}

/* The power-on sound plays to its end, as on the Xtreme 5: a tone asked for meanwhile (the
 * phone's "connected", back ~0.5 s after power-up since the call-back got fast) waits for it.
 * One waits; a later one replaces it. */
static int64_t s_poweron_end_us;
static const tone_entry_t *s_after;
static bool s_after_cut;
static esp_timer_handle_t s_after_timer;

static void after_poweron(void *arg)
{
    (void)arg;
    const tone_entry_t *e = s_after;
    s_after = NULL;
    if (e != NULL && jbl_v5_feedback_tone_on()) {
        audio_out_tone((const int16_t *)(s_map + e->offset), e->samples, s_rate, s_after_cut);
        ESP_LOGI(TAG, "tone '%.12s' (after the power-on sound)", e->name);
    }
}

bool tones_play(const char *name)
{
    lls_on_local_tone(name);                      /* stereo primary: mirror pairing / max volume */
    if (s_map == NULL || !jbl_v5_feedback_tone_on()) {
        return false;
    }
    for (uint32_t i = 0; i < s_count; i++) {
        const tone_entry_t *e = &s_idx[i];
        if (strncmp(e->name, name, sizeof(e->name)) == 0) {
            if (e->offset + e->samples * 2u > s_size) break;
            /* Auracast on/off cut the music until they end (as the Xtreme 5); the
             * others (volume end stop ...) duck it */
            bool cut = strcmp(name, "ent_aura") == 0 || strcmp(name, "exi_aura") == 0;
            int64_t now = esp_timer_get_time();
            if (strcmp(name, "poweron") == 0) {
                s_poweron_end_us = now + (int64_t)e->samples * 1000000 / s_rate + 150000;   /* + a short gap */
            } else if (now < s_poweron_end_us) {
                if (s_after_timer == NULL) {
                    const esp_timer_create_args_t a = { .callback = after_poweron, .name = "tone_after" };
                    if (esp_timer_create(&a, &s_after_timer) != ESP_OK) s_after_timer = NULL;
                }
                if (s_after_timer != NULL) {
                    s_after = e;
                    s_after_cut = cut;
                    esp_timer_stop(s_after_timer);
                    esp_timer_start_once(s_after_timer, (uint64_t)(s_poweron_end_us - now));
                    ESP_LOGI(TAG, "tone '%s' waits %lld ms for the power-on sound", name,
                             (long long)((s_poweron_end_us - now) / 1000));
                    return true;
                }
            }
            audio_out_tone((const int16_t *)(s_map + e->offset), e->samples, s_rate, cut);
            ESP_LOGI(TAG, "tone '%s' (%lu ms, %s)", name, (unsigned long)(e->samples * 1000u / s_rate),
                     cut ? "music cut" : "music ducked");
            return true;
        }
    }
    ESP_LOGW(TAG, "no tone '%s'", name);
    return false;
}

void tones_stop(void)
{
    s_after = NULL;
    if (s_after_timer != NULL) esp_timer_stop(s_after_timer);
    audio_out_tone(NULL, 0, 0, false);
}
