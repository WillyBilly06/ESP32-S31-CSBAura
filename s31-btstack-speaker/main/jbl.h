/*
 * JBL personality of the S31 on BTstack: the parts that make it look and act
 * like a JBL Xtreme 5 (ported from the Bluedroid build s31-jbl-speaker).
 *
 *   jbl_gatt    GATT database (GAP, excelpoint.cnl/.com, DIS) on LE + BR/EDR
 *   jbl_v5      JBL V5 protocol server for the JBL One app (+ heartbeat)
 *   v5_eq       the app's 7-band EQ, applied to the broadcast PCM (core 0)
 *   jbl_beacon  0xFDDF host beacon on its own extended advertising set
 *   jbl_assist  scan for JBL speakers looking for a master and bring them in
 *
 * Threading: every function here runs on the BTstack thread (core 1) unless
 * marked "any thread". The "any thread" setters only store the new value and
 * post one coalesced callback to the BTstack run loop.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Names the S31 uses as a JBL speaker. */
#define JBL_DEVICE_NAME         "S31 JBL Xtreme 5"          /* GAP / V5 DeviceName / beacon scan rsp */
#define JBL_BROADCAST_NAME      "ESP32-S31's JBL Xtreme 5"  /* Auracast Broadcast Name (assistant matches it) */

/* partyInfo values for the beacon (b15 on, b13-14 group, b11-12 conn, b9-10 role). */
#define JBL_PARTY_INFO_HOST      0xB440u   /* on, Party, connected, PRIMARY */
#define JBL_PARTY_INFO_LOOKING   0xAA40u   /* on, Party, connecting, SECONDARY */
#define JBL_PARTY_INFO_SECONDARY 0xB240u   /* on, Party, connected, SECONDARY */
#define JBL_PARTY_INFO_OFF       0x0040u   /* Auracast off (b15 clear), as a real speaker advertises */

/* BTstack thread, after a2dp_in_setup() (needs l2cap_init + sdp_init) and
 * before hci_power_control(HCI_POWER_ON). Owns sm_init(), att_server_init()
 * and gatt_client_init(): nobody else may call them. */
void jbl_setup(void);

/* BTstack thread, on BTSTACK_EVENT_STATE == HCI_STATE_WORKING: starts the
 * beacon and the assistant. */
void jbl_on_hci_working(void);

/* Any thread. Beacon partyInfo (JBL_PARTY_INFO_*). Default HOST. */
void jbl_set_party_info(uint16_t party_info);

/* Any thread. Phone music streaming: beacon 100 ms (else 20 ms) and the
 * assistant's LE scan is paused, so A2DP keeps its airtime. */
void jbl_set_streaming(bool streaming);

/* Any thread. Bluetooth name of the connected phone (NULL / "" = none): its
 * CRC-16/ARC goes into the beacon as srcName1Crc. The string is not kept. */
void jbl_set_phone_name(const char *name);
/* BT thread: ask a phone for its name (BR/EDR remote name request, no link needed);
 * the answer goes to jbl_set_phone_name. For an LE-only (LE Audio) phone. */
bool jbl_request_phone_name(const bd_addr_t addr);

/* Any thread. Party role LISTEN: stop the assistant's scan / joins (and close
 * its control link) so the LE scanner belongs to the sink; false = resume. */
void jbl_assist_set_paused(bool paused);

/* Any thread. Where our broadcast is, for the assistant's join fallback.
 * Default: JBL_BROADCAST_NAME at the controller's public address (type 0).
 * addr is in BTstack bd_addr_t order (as printed). name is copied. */
void jbl_assist_set_broadcast(const char *name, const uint8_t addr[6], uint8_t addr_type);

/* Any thread. macAddressCrc = CRC-16/ARC of the S31's own Classic MAC
 * (display order). Valid from jbl_setup() on (eFuse BT MAC). */
uint16_t jbl_mac_crc(void);

/* Any thread. One compact log line per module (counters are approximate). */
void jbl_log_stats(void);

/* Core 0 (Auracast encoder): app EQ on 48 kHz stereo interleaved s16, in place.
 * Lock-free, float only; see v5_eq.h. */
void v5_eq_process(int16_t *pcm, int frames);

#ifdef __cplusplus
}
#endif
