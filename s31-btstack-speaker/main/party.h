#pragma once
#include <stdbool.h>
#include <stdint.h>

/* JBL party roles, as an Xtreme 5:
 *   HOST   (Auracast broadcast of the phone's audio) as soon as A2DP audio plays;
 *   LISTEN (sync to another JBL party master's broadcast, play it on I2S) once
 *          no audio has played for PARTY_IDLE_TO_LISTEN_MS, phone connected or not.
 * Playing always wins: audio starting while listening drops the listen at once.
 * At power-up the S31 hosts first, then falls back to LISTEN when idle. */

void party_setup(void);            /* BT thread, before HCI power on */
void party_on_hci_working(void);   /* BT thread, after auracast_host_start() */
bool party_listening(void);        /* any thread */
/* any thread: a JBL One join is active (joining, playing or re-syncing) - the
 * joined broadcast owns the speaker, the phone's A2DP audio is not played */
bool party_assist_active(void);
/* any thread: HOST role but the broadcast could not start (retrying) - play locally */
bool party_host_failed(void);

/* any thread: the broadcast is held (pairing mode, or the phone still settling its
 * stream) - the phone plays on the local speaker meanwhile */
bool party_host_held(void);
bool party_auracast_enabled(void); /* any thread: Auracast button state (default on) */
void party_set_auracast_enabled(bool on); /* any thread; applied on the next 100 ms tick */
/* pairing mode started (button or LE Audio switch): Auracast off, no exit tone (BT thread) */
void party_auracast_off_for_pairing(void);
void party_log_stats(void);
/* any thread: stereo pair primary - keep hosting the broadcast even without music */
void party_set_stereo_primary(bool on);
/* any thread: stereo pair secondary - take the primary's broadcast handed over by PAST */
void party_set_stereo_secondary(bool on);
/* any thread: listening and the broadcast's audio is flowing */
bool party_listen_streaming(void);        /* any thread, every 2 s */
/* BT thread: S31 volume buttons while listening. Steps our own output (0..127);
 * the next master volume change takes over again. Returns false when not listening. */
bool party_listen_volume_step(int delta);
/* any thread: what we play at while listening (0..127), -1 when not listening */
int party_listen_volume(void);
/* BT thread (JBL One slider): set what we play at while listening, as an offset to the master */
void party_listen_volume_set(uint8_t v);
/* any thread: JBL PartyRole for the app - 0 normal (Auracast off), 1 secondary (listening), 2 primary */
uint8_t party_v5_role(void);

/* JBL One Auracast assistant ("Join Nearby Broadcast"), BT thread. The scan
 * reports every Auracast broadcast nearby (jbl_v5_assist_found); a join makes
 * the S31 listen to that broadcast (any source, phone audio stops playing here)
 * until the app leaves it; results go back through jbl_v5_assist_result. */
typedef struct {
    bool scanning, joining, joined, encrypted;
    uint8_t addr[6], addr_type;
    char name[33];
    uint16_t kbps;
    uint32_t bis_mask;                 /* joined: BIS indices synced (bit i-1 = BIS index i) */
} party_assist_t;
void party_assist_scan(bool on, uint8_t seconds);
/* code: 16-byte broadcast code for an encrypted broadcast, NULL = open */
void party_assist_join(const uint8_t addr[6], uint8_t addr_type, const uint8_t *code);
/* the phone's "Listen to audio broadcast" (BASS): party_assist_leave() restores the mode */
void party_bass_join(const uint8_t addr[6], uint8_t addr_type, const uint8_t *code);
void party_assist_leave(void);
void party_assist_get(party_assist_t *out);

/* BT thread: JBL One "Broadcast to Others" - a standard Auracast broadcast (any
 * Auracast receiver can join; not the JBL party), with this name and an optional
 * 16-byte broadcast code. Turning it on turns the JBL party broadcast off. */
void party_set_standard_broadcast(bool on, const char *name, const uint8_t *code);
bool party_standard_broadcast_on(void);
