/* Prompt tones from the "tones" flash partition (see tones.h). */
#include "tones.h"

#include <string.h>

#include "esp_log.h"
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

bool tones_play(const char *name)
{
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
    audio_out_tone(NULL, 0, 0, false);
}
