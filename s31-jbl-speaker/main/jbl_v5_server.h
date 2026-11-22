#pragma once

#include <stdint.h>
#include "esp_gatts_api.h"

/* JBL V5 protocol server for the JBL One app (device page, EQ, settings). */
void v5_server_start(const char *device_name);
void v5_server_set_notify_handle(uint16_t handle);
void v5_server_on_connect(esp_gatt_if_t gif, uint16_t conn_id);
void v5_server_on_disconnect(uint16_t conn_id);
void v5_server_on_write(const uint8_t *data, uint16_t len);

/* Apply the EQ the app set (7 JBL biquads) to stereo interleaved 48 kHz PCM. */
void v5_server_set_mtu(uint16_t mtu);
void v5_eq_process(int16_t *pcm, int frames);
