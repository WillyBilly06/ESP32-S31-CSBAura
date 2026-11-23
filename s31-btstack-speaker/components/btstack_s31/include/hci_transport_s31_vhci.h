#pragma once
#include <stdint.h>
/* diagnosis: log short ACL packets (<= 72 B) both ways for ms milliseconds */
void hci_transport_s31_acl_dump(uint32_t ms);
#include "hci_transport.h"
#ifdef __cplusplus
extern "C" {
#endif
/* HCI transport over the ESP32-S31 controller's VHCI (dual mode BR/EDR + LE). */
const hci_transport_t *hci_transport_s31_vhci_get_instance(void);
#ifdef __cplusplus
}
#endif
