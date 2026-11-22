#pragma once

#include <stdint.h>
#include "esp_gatts_api.h"

/* Build the JBL-speaker GATT database (call on ESP_GATTS_REG_EVT). */
void jbl_gatt_register(esp_gatt_if_t gatts_if);

/* Start each service once its table is created (ESP_GATTS_CREAT_ATTR_TAB_EVT). */
void jbl_gatt_on_attr_tab(const esp_ble_gatts_cb_param_t *p);

/* True for the excelpoint.com ...0002 value (JBL One command channel). */
#include <stdbool.h>
bool jbl_gatt_is_v5_write(uint16_t handle);

/* Human name for a written handle, for logging. */
const char *jbl_gatt_handle_name(uint16_t handle);
