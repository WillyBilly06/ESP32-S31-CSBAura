/*
 * JBL host beacon (port of jbl_beacon_* in the Bluedroid build's main.c and
 * beacon_* in bluedroid/adv.c): legacy connectable ADV_IND on its own
 * extended advertising set, random static address c4:58:34:53:31:04,
 * Service Data 0xFDDF with the V5 beacon layout, scan response = Complete
 * Local Name. 20 ms idle, 100 ms while phone music streams.
 * BTstack thread only.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t party_info;
    uint16_t name_crc;          /* srcName1Crc, valid if name_known */
    bool name_known;
    bool streaming;             /* 100 ms instead of 20 ms */
} jbl_beacon_state_t;

void jbl_beacon_setup(void);

/* HCI working: install the set and start advertising. */
void jbl_beacon_start(uint16_t mac_crc, const jbl_beacon_state_t *st);

/* Push a new state; only what changed is sent to the controller. */
void jbl_beacon_update(const jbl_beacon_state_t *st);

/* Bench: take the beacon off air / back on (BT thread). */
void jbl_beacon_pause(bool pause);

void jbl_beacon_log_stats(void);

#ifdef __cplusplus
}
#endif
