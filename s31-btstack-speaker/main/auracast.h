#pragma once
/*
 * Auracast (BAP broadcast source) on BTstack, on-air identical to the
 * Bluedroid s31-jbl-speaker build that a JBL Xtreme 5 joins natively:
 *
 *   ext adv   own extended set, non-connectable / non-scannable, public address,
 *             SID 2, 80 ms, 1M primary / 2M secondary:
 *             0x1852 Broadcast_ID 0x112233, 0x1856 PBA 04 00, appearance 0x0885,
 *             Broadcast Name + Complete Local Name "ESP32-S31's JBL Xtreme 5",
 *             Harman 0x0057 manufacturer AD (16 x 00, df fd)
 *   PA        100 ms: BASE (LC3 48 kHz / 10 ms / 100 octets, 1 subgroup, BIS 1 FL +
 *             BIS 2 FR, presentation delay 80 ms) + JBL 0xFDDF party block whose
 *             counter and status byte tick every 500 ms
 *   BIG       HCI LE Create BIG Test, Xtreme 5 layout: 2 BIS, SDU 10 ms, ISO
 *             interval 20 ms, NSE 8, BN 2, IRC 4, PTO 0, max PDU 100, 2M,
 *             interleaved, unframed, unencrypted
 *
 * Threads: every function except auracast_log_stats(), auracast_hosting() and
 * auracast_state() must run on the BTstack run loop (task "btstack", core 1).
 * LC3 encoding runs in task "lc3_enc" on core 0; the controller's completion of
 * each SDU pair paces it (ISO-clocked).
 */
#include <stdbool.h>
#include <stdint.h>

#include "btstack_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AURACAST_STATE_OFF = 0,       /* nothing on air */
    AURACAST_STATE_STARTING,      /* ext + periodic adv coming up, BIG being created */
    AURACAST_STATE_ON,            /* BIG active, audio streaming */
    AURACAST_STATE_STOPPING,      /* BIG terminating, then periodic + ext adv off */
} auracast_state_t;

/* Called on the BT thread on every settled transition:
 *   ON  (status 0)             host_start completed, BIG up
 *   OFF (status 0)             host_stop completed, BIG + broadcast adv off
 *   OFF (status != 0)          host_start failed (HCI error code / ERROR_CODE_*),
 *                              everything is off again
 * A start requested while stopping (or a stop while starting) is queued and
 * reported when it settles. */
typedef void (*auracast_state_cb_t)(auracast_state_t state, uint8_t status);

/* BT thread, before hci_power_control(HCI_POWER_ON): LC3 encoders (internal
 * RAM), encoder task on core 0, HCI/ISO handlers, ISO queue depth. */
void auracast_setup(void);

/* BT thread, on BTSTACK_EVENT_STATE == HCI_STATE_WORKING: controller
 * scheduling priorities (bt_sched_tune, MASTER profile). */
void auracast_on_hci_working(void);

/* BT thread. Asynchronous: returns 0 when accepted (or already on / starting),
 * -ENODEV if HCI is not working, -ENOMEM if internal RAM is too low for the
 * controller to build the BIG (it asserts on allocation failure; retry later),
 * -EIO if BTstack refused. Completion via the state callback / auracast_state(). */
int auracast_host_start(void);

/* BT thread. Asynchronous: terminate BIG, then stop periodic + ext adv.
 * Returns 0 (also when already off/stopping: safe to call twice). */
int auracast_host_stop(void);

/* Any thread: true while hosting is requested and not stopped, i.e. state
 * STARTING or ON. */
bool auracast_hosting(void);
auracast_state_t auracast_state(void);

/* BT thread: register for state transitions (NULL to unregister). */
void auracast_set_state_callback(auracast_state_cb_t cb);

/* BT thread: BTstack has a single ISO packet handler, which Auracast owns
 * (HCI_EVENT_BIS_CAN_SEND_NOW). Everything else it receives (ISO data of a
 * BIG sync, CIS events) is forwarded here, e.g. for a party-role sink. */
void auracast_set_iso_forward_handler(btstack_packet_handler_t handler);

/* Any thread: one log line (2 s window): SDUs sent / completed per BIS, not
 * sent, late (silence inserted), latency trims, ready-queue depth, in flight,
 * encoder avg/max us, PCM underruns. */
void auracast_log_stats(void);

/* BT thread, only while Auracast is OFF: the shared LC3 workspace (internal
 * RAM, allocated at setup) for the party decoders; the encoders are set up in
 * it again on the next host start. Returns NULL while hosting. `half` = bytes
 * per codec instance (2 instances fit). */
#include <stddef.h>
void *auracast_lc3_workspace_take(size_t *half);

/* BT thread: the phone volume changed - refresh the JBL party block now so
 * every joined speaker follows it (no-op while not broadcasting). */
void auracast_volume_changed(void);

#ifdef __cplusplus
}
#endif
