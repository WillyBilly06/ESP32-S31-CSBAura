/*
 * JBL V5 protocol server (port of the Bluedroid build's jbl_v5_server.c), so
 * the JBL One app drives the S31 like a real JBL Xtreme 5: device page,
 * EQ, settings, heartbeat. Transport: GATT excelpoint.com on LE or BR/EDR.
 *
 * Frame: u16 0xDD00 | u16 cmd (1 Get, 2 Set, 3 Notify, 4 heartbeat ack)
 *        | u8 pktCount | u8 pktIdx | u16 payloadLen | payload
 *   Get payload:  u16 featId list
 *   Set payload:  {u16 featId, u16 len, value}*
 *   Reply/notify: u16 status (0 = OK), then {u16 featId, u16 len, value}*
 * BTstack thread only.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "btstack.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Restore the stored EQ (NVS) and arm the heartbeat; before HCI power on. */
void jbl_v5_setup(void);

/* Controller public address (feature 0x000A DeviceMAC), display order. */
void jbl_v5_set_local_addr(const bd_addr_t addr);

/* ATT bearer up / down (LE peripheral or BR/EDR). The last one up, or the
 * last one that sent a V5 command, is the app link. */
void jbl_v5_link_up(hci_con_handle_t con);
void jbl_v5_link_down(hci_con_handle_t con);
bool jbl_v5_is_link(hci_con_handle_t con);

/* A complete write to excelpoint.com ...0002 (prepared writes already joined). */
void jbl_v5_on_write(hci_con_handle_t con, const uint8_t *data, uint16_t len);

void jbl_v5_log_stats(void);
/* BT thread, at HCI working: a stored stereo pair (0x2C03/0x2C81) reconnects, as a real speaker */
void jbl_v5_stereo_resume(void);
/* BT thread, from lls.c: the stereo partner ungrouped the pair / set our channel (AA 13) */
void jbl_v5_stereo_partner_ungrouped(void);
/* BT thread: ungroup from here (a key), as the app's 0x2C07 = 0 */
void jbl_v5_stereo_ungroup_local(void);
/* any thread: Auracast party on (true) / off (false) while paired - the pair is closed and made
 * again after, as the Xtreme 5. false = nothing to do */
bool jbl_v5_stereo_suspend(bool on);
bool jbl_v5_stereo_suspended(void);
/* stereo pair, BT thread (lls.c): */
void jbl_v5_light_record(uint8_t r[15]);                 /* our light state as the AA 43 record */
void jbl_v5_light_from_partner(const uint8_t r[15]);     /* the primary's AA 43 */
void jbl_v5_feedback_tone_from_partner(bool on);         /* the primary's AA 60 */
void jbl_v5_eq_tws_build(uint8_t p[115]);                /* AA 93 / 92 payload */
void jbl_v5_eq_tws_apply(const uint8_t *p, uint16_t n);  /* the primary's AA 93 */
void jbl_v5_stereo_partner_channel(uint8_t ch);

/* BT thread: push the current value of one feature to the JBL One app (no-op
 * without an app link), e.g. 0x2D42 volume, 0x2D41 mute, 0x2D40 play state. */
void jbl_v5_notify(uint16_t id);
/* BT thread: the party role / Auracast state changed (2004, 0B86/87, 2C02/03 ...). */
void jbl_v5_on_party_changed(void);
/* console: handle a V5 Set (id, value) exactly as if JBL One sent it (BT thread) */
void jbl_v5_debug_set(uint16_t id, const uint8_t *v, uint16_t n);
/* Any thread after NVS init: the speaker name (JBL One rename, persisted). */
const char *jbl_device_name(void);
/* Any thread: JBL One's feedback tone switch (0x0C40 b7) - prompt tones on. */
bool jbl_v5_feedback_tone_on(void);

/* BT thread, from party.c (Auracast assistant / Join Nearby Broadcast): a
 * broadcast seen during the app's scan, a join/leave result (status 1 ok,
 * 2 timeout, 3 wrong code, 4 code needed) and the scan state. */
void jbl_v5_assist_found(const bd_addr_t addr, uint8_t addr_type, uint8_t flags, const char *name);
void jbl_v5_assist_result(uint8_t action, const bd_addr_t addr, uint8_t addr_type, uint8_t status);
void jbl_v5_assist_scan_state(bool scanning);

#ifdef __cplusplus
}
#endif
