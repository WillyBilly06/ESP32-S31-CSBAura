#pragma once

/*
 * Start a COLD app task (housekeeping: logs, adv data refresh, heartbeats,
 * the speaker-assistant state machine) on core 1 with its stack in PSRAM, so
 * internal RAM stays free for Bluetooth, the LC3 encoder and the resampler.
 *
 * Rule: a task with a PSRAM stack must never run while the cache is disabled,
 * so it must not call NVS / esp_partition / spi_flash (the IDF asserts on it).
 * Hot tasks (lc3_enc) and tasks that read NVS (a2dp_reconn) keep internal
 * stacks. Falls back to an internal stack if PSRAM is missing or full.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "esp_heap_caps.h"

static inline void app_task_cold(const char *name, TaskFunction_t fn, uint32_t stack, UBaseType_t prio)
{
    if (xTaskCreatePinnedToCoreWithCaps(fn, name, stack, NULL, prio, NULL, 1,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        xTaskCreatePinnedToCore(fn, name, stack, NULL, prio, NULL, 1);
    }
}
