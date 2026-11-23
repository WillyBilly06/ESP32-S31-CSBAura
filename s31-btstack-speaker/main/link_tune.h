#pragma once
#include <stdbool.h>

/* Radio link tuning: EDR packet types, QoS poll interval, scan/LE interval
 * trimming. Call before hci_power_control(HCI_POWER_ON), BT thread. */
void link_tune_setup(void);

/* Any thread: the A2DP cushion ran low - poll the phone at the fast (no-BIG) rate
 * until it is refilled, so a phone hitting a bad patch has room for retries. */
void link_tune_poll_boost(bool on);
