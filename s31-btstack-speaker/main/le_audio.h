#pragma once
/* LE Audio unicast sink (BAP unicast server): PACS, ASCS (one stereo sink ASE),
 * VCS, CAS, TMAS (Unicast Media Receiver). A phone's LC3 stream on a CIS feeds the
 * same pipeline as phone A2DP (a2dp_in ext source): local speaker + broadcast.
 * Switched by JBL One's "LE Audio Connection" (0x0BA0). BT thread unless noted. */
#include <stdbool.h>
#include <stdint.h>

#include "btstack.h"

typedef enum {
    LA_PAC, LA_LOC, LA_AVAIL, LA_SUPP,        /* PACS */
    LA_ASE, LA_ASE_CP,                        /* ASCS */
    LA_VOL_STATE, LA_VOL_CP, LA_VOL_FLAGS,    /* VCS */
    LA_TMAP,                                  /* TMAS */
    LA_N
} le_audio_attr_t;

void le_audio_setup(void);
void le_audio_on_hci_working(void);

/* JBL One "LE Audio Connection": advertise + accept LE Audio streams */
void le_audio_set_enabled(bool on);
bool le_audio_enabled(void);

/* GATT (jbl_gatt.c): value or CCCD of one of the attributes above */
uint16_t le_audio_att_read(hci_con_handle_t con, le_audio_attr_t a, bool ccc, uint16_t offset,
                           uint8_t *buf, uint16_t size);
int le_audio_att_write(hci_con_handle_t con, le_audio_attr_t a, bool ccc, const uint8_t *buf, uint16_t len);

/* ISO RX (forwarded HCI ISO packet): true if it was our CIS */
bool le_audio_iso_rx(const uint8_t *packet, uint16_t size);

void le_audio_log_stats(void);
/* SDUs received / lost on the phone's CIS so far; true while it streams */
bool le_audio_cis_counts(uint32_t *rx, uint32_t *lost);

/* the LE link a phone uses for LE Audio (link_tune leaves its interval alone) */
bool le_audio_owns_link(hci_con_handle_t con);

/* any thread: an LE Audio phone is linked (the JBL beacon slows to 100 ms) */
bool le_audio_phone_linked(void);

/* any thread: the phone enabled its stream (CIS on the way) - a listen should
 * end now, the phone's CIS setup crawls while we receive a BIG */
bool le_audio_starting(void);

/* the phone is still setting LE Audio up: connected but the handshake (PAC/ASE/volume
 * subscriptions + 1.5 s quiet) is not done, or a stream is being configured/enabled */
bool le_audio_setting_up(void);

/* BT thread (jbl.c): this LE link just paired (a new bond) - Samsung then goes A2DP, so
 * no LE "connected" tone for it */
void le_audio_note_pairing(hci_con_handle_t con);

/* BT thread: "general discoverable" flag in the LE Audio advert (a2dp_in decides) */
void le_audio_set_discoverable(bool on);
