/* lls.h - JBL Xtreme 5 stereo pair ("letws") link between two speakers, see lls.c */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "btstack.h"

/* result: true = partner linked, false = timed out / failed / link ended */
typedef void (*lls_cb_t)(bool linked);

typedef struct {
    uint8_t channel;        /* 0x2C04: 1 left, 2 right */
    uint8_t gid[4];         /* 0x2C06 group id (first 4 bytes) */
    char name[24];          /* 0x2C05 group name */
} lls_group_t;

void lls_setup(void);   /* BT thread, before HCI power on */

/* BT thread. role 1 = secondary (directed advert), 2 = primary (scans, connects);
 * partner = the other speaker's BD_ADDR (display order, as JBL One's 0x2C81) */
void lls_start(uint8_t role, const uint8_t partner[6], const lls_group_t *group, uint16_t timeout_s, lls_cb_t cb);
void lls_stop(void);
bool lls_linked(void);
bool lls_is_link(hci_con_handle_t con);
/* the stereo partner's address (set by lls_start, kept after) */
bool lls_is_partner_addr(const bd_addr_t a);

/* from jbl_gatt (our server = secondary role): the primary wrote cnl.0001's CCCD / cnl.0002 */
void lls_on_cnl_cccd(hci_con_handle_t con, uint16_t value);
void lls_on_cnl_write(hci_con_handle_t con, const uint8_t *data, uint16_t len);

void lls_log_stats(void);

/* stereo pair, as the Xtreme 5 runs it (re/fw_jbl/REPORT_X5_STEREO.md):
 * the app's Forward Gets are answered from the partner's AA 12 (serial 0x0008/9, firmware
 * 0x000C, battery 0x000D); false = not known */
bool lls_partner_get(uint16_t feat, uint8_t *v, uint16_t *n);
/* app Forward Set 0x2011 = 0x81 (identify the partner): AA F2 01 4B to it after 950 ms */
void lls_partner_identify(void);
/* primary: our channel changed (Direct Set 0x2C04) - the partner gets the other one (AA 13) */
void lls_send_channel(uint8_t own_channel);
/* app ungroup: tell the partner (AA 13 with group id 0), close once it acks (2 s at most) */
void lls_ungroup(void);
/* lls_start timeout: search until found (a stored pair, as the Xtreme 5) */
#define LLS_FOREVER 0xFFFF
/* any thread: we are the linked secondary - send the key to the primary (AA 31 01 k: 1 play,
 * 3 vol+, 4 vol-, 8 BT, 10 next, 11 prev); false = not forwarded, handle it locally */
bool lls_forward_key(uint8_t k);
/* any thread (tones.c): a prompt played here; the primary mirrors pairing / max volume /
 * connected to the secondary (AA F2 01 50/52/51) */
void lls_on_local_tone(const char *name);
/* primary -> secondary (re/fw_jbl/REPORT_X5_STEREO_BEHAVIOR.md); each a no-op unless we are the
 * linked primary. BT thread: */
void lls_send_tone_setting(void);        /* AA 60: our prompt-tone setting */
void lls_send_eq(void);                     /* AA 93: after the partner's AA 12 and on every EQ change */
void lls_send_bt_led(uint8_t state);        /* AA 41: 2 pairing, 3 connected */
/* any thread: */
void lls_light_changed(void);               /* AA 43: our light state changed */
void lls_volume_ring(uint8_t vol_0_127);    /* AA 45: volume ring on the partner (display only) */
/* console: log every JBL (0xFDDF) beacon nearby, on change, for a while */
void lls_beacon_dump(uint16_t seconds);
