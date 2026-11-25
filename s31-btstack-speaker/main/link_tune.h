#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Radio link tuning: EDR packet types, QoS poll interval, scan/LE interval
 * trimming. Call before hci_power_control(HCI_POWER_ON), BT thread. */
void link_tune_setup(void);
/* BT thread: classic (BR/EDR) phone links up */
int link_tune_classic_links(void);
/* BT thread: A2DP came up or went away on a link - re-pick its poll rate */
void link_tune_refresh(const char *why);
/* test: the streaming phone's poll in slots (0 = automatic) */
void link_tune_test_poll(uint16_t slots);
/* test: the idle phone's sniff (slots, attempt, timeout) */
void link_tune_test_idle_sniff(uint16_t mn, uint16_t mx, uint16_t att, uint16_t to);
/* BT thread: a phone link in idle sniff (or the phone's own sniff) - a BIG must not start next to it */
bool link_tune_any_sniff(void);
/* BT thread: re-tune now (a broadcast is starting: idle sniff ends) */
void link_tune_wake(void);
