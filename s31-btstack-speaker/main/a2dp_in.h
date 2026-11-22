#pragma once
#include <stdbool.h>
#include <stdint.h>

/* A2DP sink on BTstack: SBC media arrives on the BT run loop (core 1) and is
 * queued; a DSP task on core 0 decodes SBC, resamples 44.1 -> 48 kHz with
 * clock recovery and fills a PCM ring that the Auracast encoder reads. */
void a2dp_in_setup(void);               /* BT thread, before hci_power_control(ON) */
void a2dp_in_on_hci_working(void);      /* BT thread: HCI up (reconnect to last phone) */

bool a2dp_in_streaming(void);
bool a2dp_in_connected(void);

/* Encoder side (core 0): pop n 48 kHz stereo frames; returns frames copied,
 * the rest is zero-filled (underrun). */
int a2dp_in_read(int16_t *pcm, int n);

void a2dp_in_log_stats(void);
