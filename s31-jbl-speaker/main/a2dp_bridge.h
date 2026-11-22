#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Bring up controller (BLE + BR/EDR dual mode) and Bluedroid. */
esp_err_t bt_dual_mode_init(void);

/* Make the S31 a discoverable BR/EDR loudspeaker with an A2DP sink. */
esp_err_t a2dp_bridge_start(const char *bredr_name);

/* Fill `frames` stereo 48 kHz s16 frames of phone audio. Returns false when
 * no phone is streaming (or the buffer is still priming). */
bool a2dp_bridge_read(int16_t *pcm, int frames);

bool a2dp_bridge_streaming(void);
bool a2dp_bridge_connected(void);
uint32_t a2dp_bridge_packets(void);

/* Called with the connected phone's Bluetooth name (for the JBL beacon). */
void a2dp_bridge_on_phone_name(void (*cb)(const char *name));

/* Totals since boot, plus the current buffer fill. */
void a2dp_bridge_stats(uint32_t *underruns, uint32_t *drops, uint32_t *fill_ms);
