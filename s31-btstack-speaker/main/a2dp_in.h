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

/* Any thread (button task): volume +-8/127 (phone slider follows via AVRCP),
 * play/pause (AVRCP to the phone), pairing (discoverable + connectable 3 min). */
typedef enum { A2DP_BTN_VOL_UP = 0, A2DP_BTN_VOL_DOWN, A2DP_BTN_PLAY_PAUSE, A2DP_BTN_PAIRING } a2dp_btn_t;
void a2dp_in_button(a2dp_btn_t btn);

/* Local speaker path: apply the phone (AVRCP) volume, ramped. One caller at a time. */
void a2dp_in_apply_volume(int16_t *pcm, int n);
/* Any thread: current phone (AVRCP absolute) volume, 0..127. */
uint8_t a2dp_in_volume(void);

/* BT thread (JBL One): set the phone volume 0..127, the phone slider follows. */
void a2dp_in_set_volume(uint8_t v, const char *why);
/* BT thread: mute the local speaker (the broadcast stays full scale). */
void a2dp_in_set_mute(bool mute);
bool a2dp_in_muted(void);
/* BT thread: JBL AudioOperation 0 stop, 1 pause, 2 play, 3 previous, 4 next,
 * 5 volume up, 6 volume down. False without an AVRCP link (0-4). */
bool a2dp_in_media_op(uint8_t op);
/* BT thread: AVRCP playback status (AVRCP_PLAYBACK_STATUS_*) and track info. */
uint8_t a2dp_in_play_status(void);
const char *a2dp_in_track_title(void);
const char *a2dp_in_track_artist(void);

/* Bench test: replace the phone audio with a 20 s cycle of
 *   0-9 s   up-sweep 50 Hz -> 18 kHz made at 48 kHz (straight to the encoder)
 *   10-19 s down-sweep 18 kHz -> 50 Hz made at 44.1 kHz and sent through the
 *           A2DP resampler (fixed ratio, no clock recovery)
 * at -12 dBFS, both channels, EQ bypassed. The phone stream is read and
 * discarded so A2DP keeps running. 0 = normal. */
#ifndef S31_TEST_SIGNAL
#define S31_TEST_SIGNAL 0
#endif
/* Bench: radio as quiet as possible - BR/EDR not discoverable / connectable
 * and no call-back of the phone, no JBL assistant scan. Only the broadcast,
 * the PA and the JBL beacon stay on air. 0 = normal. */
#ifndef S31_QUIET_RADIO_TEST
#define S31_QUIET_RADIO_TEST 0
#endif
/* test cycle phase of the last frame read: 0 up-sweep, 1 gap, 2 down-sweep, 3 gap */
int a2dp_in_test_phase(void);
