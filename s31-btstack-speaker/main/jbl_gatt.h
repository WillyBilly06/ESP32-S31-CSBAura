/*
 * JBL-speaker GATT server (BTstack att_server) on LE and BR/EDR: the database
 * compiled from jbl.gatt, one SDP record per primary service (as Bluedroid's
 * GATTS_StartService on LE_BR_EDR), CCCD storage, prepared-write reassembly
 * for the V5 command channel and a notification queue with flow control.
 * BTstack thread only.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "btstack.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Largest notification payload the queue takes (V5 frames are <= 8 + 400). */
#define JBL_GATT_NOTIFY_MAX   512u
/* Queue bytes one notification costs besides its payload. */
#define JBL_GATT_TX_OVERHEAD  6u

void jbl_gatt_setup(void);

/* excelpoint.com ...0001 value handle (V5 replies / notifications). */
uint16_t jbl_gatt_v5_notify_handle(void);

/* ATT MTU of a connection, 0 if it has no ATT bearer. */
uint16_t jbl_gatt_mtu(hci_con_handle_t con);

/* value handle of an LE Audio attribute (le_audio_attr_t) */
uint16_t jbl_gatt_le_audio_handle(int attr);

/* Free queue bytes; a notification of n bytes needs n + JBL_GATT_TX_OVERHEAD. */
uint32_t jbl_gatt_tx_free(void);

/* Anything queued or waiting for the controller (the link is "congested"). */
bool jbl_gatt_tx_busy(void);

/* Queue (or send right away) a notification. Sent regardless of the CCCD, as
 * the Bluedroid build did (JBL One relies on it). False: dropped (no room /
 * too long). The data is copied. */
bool jbl_gatt_notify(hci_con_handle_t con, uint16_t attr_handle, const uint8_t *data, uint16_t len);

void jbl_gatt_log_stats(void);

#ifdef __cplusplus
}
#endif
