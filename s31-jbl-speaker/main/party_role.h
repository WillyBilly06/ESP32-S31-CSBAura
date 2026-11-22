/*
 * JBL Xtreme 5-style Auracast party role switching.
 *
 *   HOST   - phone audio (A2DP) streams: the S31 is the Auracast broadcast
 *            source and JBL party Primary (beacon partyInfo 0xB440).
 *   LISTEN - no phone audio for PARTY_HOLDOFF_MS: our broadcast is stopped,
 *            the beacon says "Secondary, looking for a master" (0xAA40), and
 *            the S31 scans for another JBL master's Auracast, PA-syncs, reads
 *            the BASE, BIG-syncs to its BIS, LC3-decodes and plays it on I2S
 *            (GPIO2 BCLK / GPIO3 WS / GPIO4 DOUT); once synced the beacon says
 *            "Secondary, connected" (0xB240).
 * Phone audio starting again switches straight back to HOST.
 *
 * One task ("party") owns every transition; stack callbacks (iso task, BTC
 * task) only post events to it. Every transition is logged as
 * "PARTY: <from> -> <to> (<why>)".
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_gap_ble_api.h"
#include "esp_ble_audio_common_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 0 = always HOST (the behaviour before the party role existed). */
#ifndef PARTY_AUTO_SWITCH
#define PARTY_AUTO_SWITCH       1
#endif

/* No phone audio for this long -> stop hosting and look for another master. */
#define PARTY_HOLDOFF_MS        3000

/* JBL V5 beacon partyInfo values (see jbl_assistant.c for the bit layout). */
#define PARTY_INFO_HOST         0xB440   /* on, Party, connected, PRIMARY */
#define PARTY_INFO_LOOKING      0xAA40   /* on, Party, connecting, SECONDARY */
#define PARTY_INFO_SECONDARY    0xB240   /* on, Party, connected, SECONDARY */

/* Host side, implemented in main.c (they own the broadcast source). Both are
 * blocking and called from the party task only. */
typedef struct {
    /* ext + periodic adv on ADV_HANDLE, then BAP broadcast source start (BIG). */
    int (*host_start)(void);
    /* BAP broadcast source stop (BIG terminate, waits for the stopped
     * callback), then periodic + ext adv off. Safe to call when not hosting. */
    int (*host_stop)(void);
    /* Put a new partyInfo into the JBL 0xFDDF beacon (pushed at once). */
    void (*set_party_info)(uint16_t party_info);
} party_role_ops_t;

/* Register the sink side (PACS sink capability, scan delegator, broadcast sink
 * callbacks). Call between esp_ble_audio_common_init() and _common_start(). */
esp_err_t party_role_init(void);

/* LE Audio GAP app callback: pass in esp_ble_audio_init_info_t.gap_cb. Gets ext
 * scan reports and PA sync / sync lost events on the iso task. */
void party_role_iso_gap_cb(esp_ble_audio_gap_app_event_t *event);

/* Start the state machine. `hosting` = the broadcast is already up (boot path). */
esp_err_t party_role_start(const party_role_ops_t *ops, uint16_t own_mac_crc, bool hosting);

/* Bluedroid GAP events (scan params/start/stop complete); fed from adv.c. */
void party_role_gap_event(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

/* A2DP audio state changed (a2dp_bridge callback, BTC task). Just a wake-up:
 * the task re-reads a2dp_bridge_streaming() itself. */
void party_role_on_audio_state(bool streaming);

/* Broadcast source callbacks from main.c (iso task). */
void party_role_on_source_started(void);
void party_role_on_source_stopped(void);

/* True while the S31 is (or is becoming) the Auracast host. */
bool party_role_is_hosting(void);

/* One 2 s status line for the stats task while not hosting. */
void party_role_log_stats(void);

#ifdef __cplusplus
}
#endif
