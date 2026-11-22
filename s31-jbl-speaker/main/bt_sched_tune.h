/*
 * bt_sched_tune - BR/EDR (A2DP) vs LE Audio (BIG) radio-priority tuning for
 * the ESP32-S31 controller blob. Background: re/controller_sched.md.
 *
 * Two knobs, both plain data in controller RAM, found and verified at run time
 * (a write only happens after every check passes):
 *   - BR/EDR: ACL base priority = byte 0 of the 36-byte {dft,inc} table read by
 *     r_olc_sched_pti_dft_get() (olc_sched.c.o .dram1.38). Effective ACL prio =
 *     min(14, base + min(14, queued pkts / 4) + min(14, missed poll intervals)).
 *   - LE: the 64-byte ISO config block returned by r_sym_ble_kBC0rRwtjATsFkriXVDL():
 *     [33] BIS first copy (group 0), [34] repetitions (groups 1..IRC-1),
 *     [35] pre-transmission groups (>= IRC), [36] BIG control subevent.
 *     The controller re-applies its defaults (9/7/8/12) on every HCI_Reset.
 *
 * Usage:
 *   bt_sched_tune_init();              after the controller + both host stacks
 *                                      are up (after the last HCI_Reset), before
 *                                      or after Create BIG.
 *   bt_sched_tune_streaming(true);     A2DP audio state STARTED
 *   bt_sched_tune_streaming(false);    A2DP audio SUSPEND/STOPPED, A2DP or ACL
 *                                      disconnect
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    /* ACL highest (14 = the controller's ACL cap) while A2DP streams; BIS first
     * copies 13 > repetitions 12 > pre-tx 11 > periodic adv (10) > ext/legacy
     * adv (1/5, escalating to 9/10) > scan (1/4). Costs BIG airtime whenever
     * the phone link is busy. */
    BT_SCHED_MODE_A2DP_FIRST = 0,
    /* BIS first copies 15 (unreachable by the ACL, whose cap is 14) so every
     * SDU gets on air at least once; ACL base 8 beats repetitions 7 so A2DP
     * keeps its throughput (repetition diversity is what is sacrificed). */
    BT_SCHED_MODE_AURACAST_FIRST = 1,
} bt_sched_mode_t;

#ifndef BT_SCHED_MODE
#define BT_SCHED_MODE BT_SCHED_MODE_A2DP_FIRST
#endif

/* Locate + verify both tables and apply the "not streaming" profile.
 * Safe to call more than once (e.g. again after a controller reset). */
void bt_sched_tune_init(void);

/* Switch between the streaming and idle profiles of the active mode.
 * Re-verifies and re-applies everything (also repairs an HCI_Reset re-default). */
void bt_sched_tune_streaming(bool on);

/* Change the mode at run time (A/B testing); re-applies the current profile. */
void bt_sched_tune_set_mode(bt_sched_mode_t mode);

#ifdef __cplusplus
}
#endif
