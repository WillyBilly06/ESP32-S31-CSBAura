/*
 * S31 as its own "JBL One app" (port of the Bluedroid build's jbl_assistant.c):
 * watch the 0xFDDF beacons of nearby JBL V5 speakers; when one has its
 * Auracast button pressed ("party on, SECONDARY, connecting" = looking for a
 * master) and has not joined us natively within 15 s, connect over LE, pair
 * (Just Works), enable notifications on excelpoint.com ...0001 and make it
 * join our broadcast (ScanControl 0x0B81, then SubgroupPlay 0x0B85 +
 * GroupSelection 0x0B83). State machine ticks every 500 ms.
 * BTstack thread only, except the setters declared in jbl.h.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "btstack.h"

#ifdef __cplusplus
extern "C" {
#endif

void jbl_assist_setup(void);

/* HCI working: own identity (beacon macCrc, public address) and go. */
void jbl_assist_start(uint16_t own_mac_crc, const bd_addr_t own_addr);

/* Phone music streaming: the LE scan pauses (A2DP airtime). */
void jbl_assist_set_streaming(bool streaming);

/* True for the assistant's own (central) LE link to a speaker. */
bool jbl_assist_is_link(hci_con_handle_t con);

/* True once the assistant is not scanning and will not start a scan. */
bool jbl_assist_scan_idle(void);

void jbl_assist_log_stats(void);

#ifdef __cplusplus
}
#endif
