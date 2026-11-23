#pragma once
/*
 * s31_ident.h - what makes each flashed S31 its own Xtreme 5.
 *
 * Every board is its own speaker: Classic + LE addresses, the beacon address and macCrc,
 * the serial number and the standard Auracast Broadcast_ID all come from the chip's
 * factory-burned MAC (unique per chip; the ESP32-S31 controller fails BR/EDR
 * re-authentication with any other address). The JBL party Broadcast_ID
 * (0x112233) stays shared on purpose: every Xtreme 5 uses it.
 *
 * Chosen per board when flashing (tools/flasher writes the "s31cfg" partition): the
 * color the JBL One app shows and an optional default name. No partition (or a bad
 * one): Black, "S31 JBL Xtreme 5".
 */
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* JBL color IDs of the Xtreme 5 (beacon colorID / V5 ColorID), from JBL's model config
 * for product 2169: 00 Black, 01 White, 02 Blue, 05 Purple, 1A Camo */
#define S31_COLOR_BLACK 0x00

/* call once, early (before the beacon / JBL One settings are built) */
void s31_ident_init(void);

uint8_t s31_ident_color(void);
/* the name chosen at flash time, or "" when none was set */
const char *s31_ident_name(void);
/* "S31X5" + the last 3 MAC bytes in hex, e.g. "S31X5A1B2C3" */
const char *s31_ident_serial(void);
/* 24-bit standard Auracast Broadcast_ID of this board (never 0 or 0x112233) */
uint32_t s31_ident_std_broadcast_id(void);
/* the JBL beacon's random static address: per speaker (it used to be one fixed address
 * on every board) */
void s31_ident_beacon_addr(uint8_t out[6]);

#ifdef __cplusplus
}
#endif
