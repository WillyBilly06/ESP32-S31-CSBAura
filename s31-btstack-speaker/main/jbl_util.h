/* Internal helpers shared by the jbl_* modules. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "btstack.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CRC-16/ARC (poly 0xA001 reflected, init 0): the V5 macAddressCrc / srcName1Crc. */
uint16_t jbl_crc16_arc(const uint8_t *d, size_t n);

/* True on the BTstack run-loop task (valid after jbl_setup). */
bool jbl_on_bt_thread(void);

/* Run reg->callback(reg->context) on the BTstack thread: now if already there,
 * else posted (FIFO, coalesced while pending). reg must stay valid. */
void jbl_run_on_bt(btstack_context_callback_registration_t *reg);

#ifdef __cplusplus
}
#endif
