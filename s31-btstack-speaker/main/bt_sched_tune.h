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
    /* S31 is BR/EDR master (role switch refused): stock ACL priority 2 and
     * every BIS copy above it - the S31 just polls the phone in the gaps
     * between BIG events, so nothing has to be dropped and nothing moves. */
    BT_SCHED_MODE_MASTER = 2,
} bt_sched_mode_t;

/* BTstack build: the S31 stays BR/EDR master (a2dp_in.c refuses role switch),
 * so MASTER (= the idle profile, BIS above the ACL) as in s31-jbl-speaker,
 * which set it with -DBT_SCHED_MODE=BT_SCHED_MODE_MASTER. */
#ifndef BT_SCHED_MODE
#define BT_SCHED_MODE BT_SCHED_MODE_MASTER
#endif

/* Locate + verify both tables and apply the "not streaming" profile.
 * Safe to call more than once (e.g. again after a controller reset). */
void bt_sched_tune_init(void);

/* Switch between the streaming and idle profiles of the active mode.
 * Re-verifies and re-applies everything (also repairs an HCI_Reset re-default). */
void bt_sched_tune_streaming(bool on);

/* Change the mode at run time (A/B testing); re-applies the current profile. */
void bt_sched_tune_set_mode(bt_sched_mode_t mode);

/* A/B testing: explicit ACL base / BIS first / repetition / pre-tx priorities. */
void bt_sched_tune_apply_custom(uint8_t acl, uint8_t first, uint8_t rep, uint8_t pto, const char *why);

/* The phone streams to us over LE Audio (CIS): the CIS goes above every BIS copy
 * (it has no spare copies, the BIS has IRC 4) and the BIS first copies come down to
 * 13 so the LE links can win their slots too. Off: CIS 14, BIS first 15. */
void bt_sched_tune_le_input(bool on);

#ifdef __cplusplus
}
#endif
