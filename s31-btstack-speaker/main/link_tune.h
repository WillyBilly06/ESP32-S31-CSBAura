#pragma once

/* Radio link tuning: EDR packet types, QoS poll interval, scan/LE interval
 * trimming. Call before hci_power_control(HCI_POWER_ON), BT thread. */
void link_tune_setup(void);
/* BT thread: classic (BR/EDR) phone links up */
int link_tune_classic_links(void);
/* BT thread: A2DP came up or went away on a link - re-pick its poll rate */
void link_tune_refresh(const char *why);
