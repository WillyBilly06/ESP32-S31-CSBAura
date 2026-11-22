#pragma once

#include <stdint.h>
#include "esp_gatts_api.h"

/* Build the JBL-speaker GATT database (call on ESP_GATTS_REG_EVT). */
void jbl_gatt_register(esp_gatt_if_t gatts_if);

/* Start each service once its table is created (ESP_GATTS_CREAT_ATTR_TAB_EVT). */
void jbl_gatt_on_attr_tab(const esp_ble_gatts_cb_param_t *p);

/* Human name for a written handle, for logging. */
const char *jbl_gatt_handle_name(uint16_t handle);
