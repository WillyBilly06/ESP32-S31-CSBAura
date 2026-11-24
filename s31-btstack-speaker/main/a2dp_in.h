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
/* BT thread: ACL of the phone with the A2DP link, HCI_CON_HANDLE_INVALID if none */
uint16_t a2dp_in_con_handle(void);

/* Another audio source feeding the same pipeline as phone A2DP (resampler, PCM
 * ring, local output / broadcast): LE Audio unicast. While it is active the
 * phone's A2DP packets are dropped and a2dp_in_streaming() is true.
 * BT thread: start / stop / push (one SDU, len 0 = lost).
 * DSP task: the decode callback gets each pushed SDU and hands its PCM
 * (interleaved stereo) to a2dp_in_ext_pcm(). */
typedef void (*a2dp_in_ext_decode_t)(const uint8_t *data, uint16_t len);
void a2dp_in_ext_start(a2dp_in_ext_decode_t decode, const char *what, uint32_t target_ms);
void a2dp_in_ext_stop(void);

/* DSP task: the one decoder workspace in internal SRAM (AAC or LE Audio LC3, one at a
 * time - only the DSP task decodes). owner: 1 AAC, 2 LC3. Returns false if the other
 * decoder had it: the caller sets its decoder up again. */
void *a2dp_in_codec_ws(unsigned *size);
bool a2dp_in_codec_ws_take(int owner);
void a2dp_in_probe_log(const char *what);    /* DSP task: core + workspace speed */

/* BT thread: the phone is fully connected (A2DP media channel open / LE Audio in use):
 * a pairing mode started for a new phone ends */
void a2dp_in_pairing_done(const char *why);
/* end pairing mode now, whatever state it is in (BT thread) */
void a2dp_in_pairing_cancel(const char *why);
/* pairing mode was opened by an LE Audio switch (Auracast paused, not off) */
bool a2dp_in_pairing_is_switch(void);
/* forget every phone: Classic link keys + LE bonds (BT thread); returns LE bonds removed */
int a2dp_in_forget_all(void);
bool a2dp_in_ext_active(void);
void a2dp_in_ext_push(const uint8_t *data, uint16_t len);
void a2dp_in_ext_pcm(int16_t *lr, int frames, int rate);

/* Encoder side (core 0): pop n 48 kHz stereo frames; returns frames copied,
 * the rest is zero-filled (underrun). */
int a2dp_in_read(int16_t *pcm, int n);

void a2dp_in_log_stats(void);

/* Any thread (button task): volume +-8/127 (phone slider follows via AVRCP),
 * play/pause (AVRCP to the phone), pairing (discoverable + connectable 3 min). */
/* A2DP_BTN_PAIRING_SWITCH: LE Audio switched on/off in JBL One - as the Xtreme 5 the speaker
 * forgets every phone (they pair again with the new bond set) and Auracast only pauses */
typedef enum { A2DP_BTN_VOL_UP = 0, A2DP_BTN_VOL_DOWN, A2DP_BTN_PLAY_PAUSE, A2DP_BTN_PAIRING,
               A2DP_BTN_PAIRING_SWITCH } a2dp_btn_t;
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

/* BT thread: A2DP was playing when it went away less than within_ms ago (the phone
 * moved to LE Audio mid-song: le_audio asks it to play on) */
bool a2dp_in_was_playing(uint32_t within_ms);

/* BT thread: the phone is settling into A2DP - connected < 4 s ago, or streaming but
 * without 1 s of media yet (capped at 8 s after Start). The party holds its scan and
 * its broadcast meanwhile: a BIG created right at Start left the stream empty. */
bool a2dp_in_settling(void);

/* BT thread: the phone moved A2DP -> LE Audio mid-song and did not resume: AVRCP Play
 * (opens AVRCP to the last A2DP phone first if it is closed) */
void a2dp_in_avrcp_play_after_switch(void);
/* any thread: ms since the A2DP "connected" tone (0 = never) */
uint32_t a2dp_in_ms_since_connect_tone(void);
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

/* BT thread: discoverable (classic inquiry scan + "general discoverable" in the LE Audio
 * advert) only in pairing mode or with no phone at all - classic or LE Audio. As a JBL
 * speaker: a linked phone takes it out of pairing; the pairing button puts it back. */
void a2dp_in_refresh_discoverable(void);
/* BT thread: stereo pair secondary - drop every phone link, stay unconnectable while on */
void a2dp_in_set_stereo_secondary(bool on);
/* any thread (console "callback"): clear a failed call-back and call the last phone back now */
void a2dp_in_callback_now(void);
/* the power-up call-back of the last phone is scheduled or paging (the stereo pair waits for it) */
bool a2dp_in_callback_busy(void);
/* console: how long the phone gets to open AVRCP itself before we do */
void a2dp_in_set_avrcp_wait(uint32_t ms);
bool a2dp_in_pairing(void);

/* BT thread: a phone just bonded over LE (its classic key derived, CTKD). Samsung pages
 * us for SDP right then, its own controller answers CONTROLLER_BUSY (still on the LE
 * pairing) and Android waits 60 s before trying again - "Pairing..." for a minute. If no
 * classic link is up 2.5 s later, the S31 opens it (A2DP), and the phone's SDP + profiles
 * run on that. Only after a new bond, never on reconnects. */
void a2dp_in_call_after_le_pairing(const uint8_t identity[6]);
#endif
/* test cycle phase of the last frame read: 0 up-sweep, 1 gap, 2 down-sweep, 3 gap */
int a2dp_in_test_phase(void);
