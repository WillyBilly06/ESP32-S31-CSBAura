#pragma once

/* Radio link tuning: EDR packet types, QoS poll interval, scan/LE interval
 * trimming. Call before hci_power_control(HCI_POWER_ON), BT thread. */
void link_tune_setup(void);
