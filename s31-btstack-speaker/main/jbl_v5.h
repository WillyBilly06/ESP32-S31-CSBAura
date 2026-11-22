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

/* A complete write to excelpoint.com ...0002 (prepared writes already joined). */
void jbl_v5_on_write(hci_con_handle_t con, const uint8_t *data, uint16_t len);

void jbl_v5_log_stats(void);

#ifdef __cplusplus
}
#endif
