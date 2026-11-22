/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

#include "ble_audio_example_utils.h"

#define TAG "BAP_BSRC"

#define ADV_HANDLE              0
#define ADV_SID                 2      /* the Xtreme 5 broadcasts on advertising set ID 2 */
#define ADV_TX_POWER            127
#define ADV_INTERVAL_MS         80     /* Xtreme 5: ~73-114 ms gaps seen -> ~80 ms */
#define PER_ADV_INTERVAL_MS     100

/* Second advertiser: JBL-style host beacon (legacy, connectable, own address). */
#define BEACON_HANDLE           1
#define BEACON_SID              1
#define BEACON_INTERVAL_MS      20     /* as the real Xtreme 5 beacon */

int app_host_init(void);

int ext_adv_start(const uint8_t *ext_data, uint8_t ext_len,
                  const uint8_t *per_data, uint8_t per_len);

/* Start the JBL host beacon on BEACON_HANDLE with its own random static address. */
int beacon_adv_start(const uint8_t *adv, uint8_t adv_len,
                     const uint8_t *rsp, uint8_t rsp_len,
                     const uint8_t rand_addr[6]);

/* Update the running beacon's payload / interval (blocking; app task only). */
int beacon_adv_data_update(const uint8_t *adv, uint8_t adv_len);
int beacon_set_interval(uint16_t ms);

/* Re-arm the beacon after a peer disconnects (non-blocking; safe from callbacks). */
void beacon_adv_restart(void);

/* Replace the periodic-advertising data while the broadcast runs (blocking;
 * call from an app task, not from a Bluetooth callback). */
int per_adv_data_update(const uint8_t *per_data, uint8_t per_len);
