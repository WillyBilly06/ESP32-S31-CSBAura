/* csb.h - BR/EDR Connectionless Peripheral Broadcast (CPB / CSB) on the S31, see csb.c */
#pragma once

#include <stdint.h>

#include "btstack.h"

typedef void (*csb_rx_cb_t)(uint32_t clk, uint8_t fragment, const uint8_t *data, uint8_t len);

void csb_setup(void);           /* BT thread, before HCI power on */
void csb_on_hci_working(void);  /* BT thread: event mask page 2 */

/* receive: find addr's synchronization train, then its broadcast */
void csb_rx_start(const bd_addr_t addr);
void csb_rx_stop(void);
/* as a CSB receiver that lost (or never had) sync: Truncated Page to the broadcaster (it
 * starts its synchronization train), then receive it */
void csb_rx_join(const bd_addr_t addr);
/* called when we were truncated-paged (a receiver wants our train) */
void csb_set_tpage_callback(void (*cb)(void));
void csb_rx_set_callback(csb_rx_cb_t cb);

/* transmit: CPB every interval_slots (0.625 ms) + synchronization train */
void csb_tx_start(uint16_t interval_slots, const uint8_t *data, uint8_t len);
void csb_tx_set_params(uint16_t ptype, uint8_t lpo, uint16_t supervision);
void csb_tx_data(const uint8_t *data, uint8_t len);
void csb_tx_stop(void);

void csb_log_stats(void);
