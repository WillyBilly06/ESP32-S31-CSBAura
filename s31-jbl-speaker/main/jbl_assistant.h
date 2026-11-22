#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "esp_gap_ble_api.h"

/* Start the auto-join assistant: find a JBL Xtreme 5 over LE, connect to its
 * V5 control channel and make it join our Auracast broadcast. */
esp_err_t jbl_assistant_start(uint16_t own_mac_crc, const char *own_broadcast_name);

/* GAP events the assistant needs (scan reports, pairing); fed from adv.c. */
void jbl_assistant_gap_event(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
