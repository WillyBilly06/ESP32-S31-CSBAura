/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Two advertisers:
 *   ADV_HANDLE    (0) - Auracast broadcast: extended + periodic, public address.
 *   BEACON_HANDLE (1) - JBL-style host beacon: legacy connectable ADV_IND,
 *                       own random static address, Service Data 0xFDDF.
 * This mirrors the JBL Xtreme 5, which hosts with its broadcast on its main
 * address and the 0xFDDF beacon on a second address.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"

#include "adv.h"
#include "../jbl_assistant.h"
#include "../party_role.h"

static SemaphoreHandle_t adv_sem;
/* Serialises the blocking helpers below: they share adv_sem, and since the
 * party-role switch they are called from several tasks (jbl_pa, party). */
static SemaphoreHandle_t adv_lock;

/* Controller status latched by gap_event_handler for EXAMPLE_WAIT_API_CHECK. */
static esp_bt_status_t adv_op_status;

#define WAIT_API(_call) EXAMPLE_WAIT_API_CHECK(_call, adv_sem, portMAX_DELAY, adv_op_status)

/* Wrap a helper body (which returns on error from inside WAIT_API) in adv_lock. */
#define ADV_LOCKED(_body_fn, ...) ({                                \
        xSemaphoreTake(adv_lock, portMAX_DELAY);                    \
        int _r = _body_fn(__VA_ARGS__);                             \
        xSemaphoreGive(adv_lock);                                   \
        _r; })

static esp_ble_gap_ext_adv_params_t ext_adv_params = {
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_NONCONN_NONSCANNABLE_UNDIRECTED,
    .interval_min = ESP_BLE_GAP_ADV_ITVL_MS(ADV_INTERVAL_MS),
    .interval_max = ESP_BLE_GAP_ADV_ITVL_MS(ADV_INTERVAL_MS),
    .channel_map = ADV_CHNL_ALL,
    .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .primary_phy = ESP_BLE_GAP_PHY_1M,
    .max_skip = 0,
    .secondary_phy = ESP_BLE_GAP_PHY_2M,
    .sid = ADV_SID,
    .scan_req_notif = false,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .tx_power = ADV_TX_POWER,
};

static esp_ble_gap_ext_adv_params_t beacon_adv_params = {
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_LEGACY_IND,      /* connectable + scannable */
    .interval_min = ESP_BLE_GAP_ADV_ITVL_MS(BEACON_INTERVAL_MS),
    .interval_max = ESP_BLE_GAP_ADV_ITVL_MS(BEACON_INTERVAL_MS),
    .channel_map = ADV_CHNL_ALL,
    .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .primary_phy = ESP_BLE_GAP_PHY_1M,
    .max_skip = 0,
    .secondary_phy = ESP_BLE_GAP_PHY_1M,
    .sid = BEACON_SID,
    .scan_req_notif = true,      /* log who actively scans the beacon */
    .own_addr_type = BLE_ADDR_TYPE_RANDOM,
    .tx_power = ADV_TX_POWER,
};

static esp_ble_gap_periodic_adv_params_t periodic_adv_params = {
    .interval_min = ESP_BLE_GAP_PERIODIC_ADV_ITVL_MS(PER_ADV_INTERVAL_MS),
    .interval_max = ESP_BLE_GAP_PERIODIC_ADV_ITVL_MS(PER_ADV_INTERVAL_MS),
    .properties = 0,
};

static esp_ble_gap_ext_adv_t ext_adv_inst[1] = {
    [0] = { ADV_HANDLE, 0, 0 },
};

static esp_ble_gap_ext_adv_t beacon_adv_inst[1] = {
    [0] = { BEACON_HANDLE, 0, 0 },
};

static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
        adv_op_status = param->ext_adv_set_params.status;
        xSemaphoreGive(adv_sem);
        break;
    case ESP_GAP_BLE_EXT_ADV_SET_RAND_ADDR_COMPLETE_EVT:
        adv_op_status = param->ext_adv_set_rand_addr.status;
        xSemaphoreGive(adv_sem);
        break;
    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
        adv_op_status = param->ext_adv_data_set.status;
        xSemaphoreGive(adv_sem);
        break;
    case ESP_GAP_BLE_EXT_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_op_status = param->scan_rsp_set.status;
        xSemaphoreGive(adv_sem);
        break;
    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
        adv_op_status = param->ext_adv_start.status;
        xSemaphoreGive(adv_sem);
        break;
    case ESP_GAP_BLE_PERIODIC_ADV_SET_PARAMS_COMPLETE_EVT:
        adv_op_status = param->peroid_adv_set_params.status;
        xSemaphoreGive(adv_sem);
        break;
    case ESP_GAP_BLE_PERIODIC_ADV_DATA_SET_COMPLETE_EVT:
        adv_op_status = param->period_adv_data_set.status;
        xSemaphoreGive(adv_sem);
        break;
    case ESP_GAP_BLE_PERIODIC_ADV_START_COMPLETE_EVT:
        adv_op_status = param->period_adv_start.status;
        xSemaphoreGive(adv_sem);
        break;
    case ESP_GAP_BLE_PERIODIC_ADV_STOP_COMPLETE_EVT:
        adv_op_status = param->period_adv_stop.status;
        xSemaphoreGive(adv_sem);
        break;
    case ESP_GAP_BLE_EXT_ADV_STOP_COMPLETE_EVT:
        adv_op_status = param->ext_adv_stop.status;
        xSemaphoreGive(adv_sem);
        break;
    case ESP_GAP_BLE_SCAN_REQ_RECEIVED_EVT: {
        /* Log each scanner address the first time it is seen. */
        static uint8_t seen[32][6];
        static int n_seen;
        const uint8_t *a = param->scan_req_received.scan_addr;
        bool known = false;
        for (int k = 0; k < n_seen; k++) {
            if (memcmp(seen[k], a, 6) == 0) {
                known = true;
                break;
            }
        }
        if (!known) {
            memcpy(seen[n_seen % 32], a, 6);
            if (n_seen < 32) {
                n_seen++;
            }
            ESP_LOGW(TAG, ">>> NEW SCANNER on set %u: %02x:%02x:%02x:%02x:%02x:%02x (type %u)",
                     param->scan_req_received.adv_instance, a[0], a[1], a[2], a[3], a[4], a[5],
                     param->scan_req_received.scan_addr_type);
        }
        break;
    }
    case ESP_GAP_BLE_ADV_TERMINATED_EVT:
        /* A connectable advertiser stops when someone connects to it. */
        if (param->adv_terminate.adv_instance == BEACON_HANDLE &&
            param->adv_terminate.status == 0x00) {
            ESP_LOGW(TAG, ">>> PEER CONNECTED TO JBL BEACON (conn_idx %u) - a speaker is "
                     "talking to our 'host'", param->adv_terminate.conn_idx);
        } else {
            ESP_LOGI(TAG, "adv set %u terminated, status 0x%02x",
                     param->adv_terminate.adv_instance, param->adv_terminate.status);
        }
        break;
    default:
        /* scan params/start/stop complete -> party role (LISTEN scanner);
         * scan reports, scan start, pairing -> auto-join assistant */
        party_role_gap_event(event, param);
        jbl_assistant_gap_event(event, param);
        break;
    }
}

int app_host_init(void)
{
    esp_err_t err;

    adv_lock = xSemaphoreCreateMutex();
    adv_sem = xSemaphoreCreateBinary();
    if (adv_sem == NULL || adv_lock == NULL) {
        ESP_LOGE(TAG, "Failed to create adv semaphore");
        return -1;
    }

    err = esp_ble_gap_register_callback(gap_event_handler);
    if (err) {
        ESP_LOGE(TAG, "Failed to register GAP callback, err %d", err);
        vSemaphoreDelete(adv_sem);
        return err;
    }

    return 0;
}

/* Like WAIT_API, but a failure is only logged and the wait is bounded: used
 * for "stop" commands, which must not abort a teardown or an interval change
 * (e.g. the beacon set may already be stopped by an incoming connection). */
#define WAIT_API_SOFT(_call) do {                                           \
        esp_err_t _e = (_call);                                             \
        if (_e != ESP_OK) {                                                 \
            ESP_LOGW(TAG, #_call " failed, err %d", _e);                    \
        } else if (xSemaphoreTake(adv_sem, pdMS_TO_TICKS(1000)) != pdTRUE) { \
            ESP_LOGW(TAG, #_call " timed out");                             \
        } else if (adv_op_status != ESP_BT_STATUS_SUCCESS) {                \
            ESP_LOGW(TAG, #_call " ctrl status %d", adv_op_status);         \
        }                                                                   \
    } while (0)

static int ext_adv_start_impl(const uint8_t *ext_data, uint8_t ext_len,
                              const uint8_t *per_data, uint8_t per_len)
{
    WAIT_API(esp_ble_gap_ext_adv_set_params(ADV_HANDLE, &ext_adv_params));
    WAIT_API(esp_ble_gap_config_ext_adv_data_raw(ADV_HANDLE, ext_len, ext_data));
    WAIT_API(esp_ble_gap_periodic_adv_set_params(ADV_HANDLE, &periodic_adv_params));
#if CONFIG_BT_BLE_FEAT_PERIODIC_ADV_ENH
    WAIT_API(esp_ble_gap_config_periodic_adv_data_raw(ADV_HANDLE, per_len, per_data, false));
    WAIT_API(esp_ble_gap_periodic_adv_start(ADV_HANDLE, true));
#else
    WAIT_API(esp_ble_gap_config_periodic_adv_data_raw(ADV_HANDLE, per_len, per_data));
    WAIT_API(esp_ble_gap_periodic_adv_start(ADV_HANDLE));
#endif
    WAIT_API(esp_ble_gap_ext_adv_start(1, ext_adv_inst));

    ESP_LOGI(TAG, "Advertising started (handle %u)", ADV_HANDLE);
    return 0;
}

int ext_adv_start(const uint8_t *ext_data, uint8_t ext_len,
                  const uint8_t *per_data, uint8_t per_len)
{
    return ADV_LOCKED(ext_adv_start_impl, ext_data, ext_len, per_data, per_len);
}

static int ext_adv_stop_impl(void)
{
    uint8_t h = ADV_HANDLE;

    WAIT_API_SOFT(esp_ble_gap_periodic_adv_stop(ADV_HANDLE));
    WAIT_API_SOFT(esp_ble_gap_ext_adv_stop(1, &h));
    ESP_LOGI(TAG, "Broadcast advertising stopped (handle %u)", ADV_HANDLE);
    return 0;
}

int ext_adv_stop(void)
{
    return ADV_LOCKED(ext_adv_stop_impl);
}

static int beacon_adv_start_impl(const uint8_t *adv, uint8_t adv_len,
                                 const uint8_t *rsp, uint8_t rsp_len,
                                 const uint8_t rand_addr[6])
{
    esp_bd_addr_t addr;

    memcpy(addr, rand_addr, sizeof(addr));

    WAIT_API(esp_ble_gap_ext_adv_set_params(BEACON_HANDLE, &beacon_adv_params));
    WAIT_API(esp_ble_gap_ext_adv_set_rand_addr(BEACON_HANDLE, addr));
    WAIT_API(esp_ble_gap_config_ext_adv_data_raw(BEACON_HANDLE, adv_len, adv));
    if (rsp != NULL && rsp_len > 0) {
        WAIT_API(esp_ble_gap_config_ext_scan_rsp_data_raw(BEACON_HANDLE, rsp_len, rsp));
    }
    WAIT_API(esp_ble_gap_ext_adv_start(1, beacon_adv_inst));

    ESP_LOGW(TAG, "JBL host beacon started (handle %u) at %02x:%02x:%02x:%02x:%02x:%02x",
             BEACON_HANDLE, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    return 0;
}

int beacon_adv_start(const uint8_t *adv, uint8_t adv_len,
                     const uint8_t *rsp, uint8_t rsp_len,
                     const uint8_t rand_addr[6])
{
    return ADV_LOCKED(beacon_adv_start_impl, adv, adv_len, rsp, rsp_len, rand_addr);
}

static int per_adv_data_update_impl(const uint8_t *per_data, uint8_t per_len)
{
#if CONFIG_BT_BLE_FEAT_PERIODIC_ADV_ENH
    WAIT_API(esp_ble_gap_config_periodic_adv_data_raw(ADV_HANDLE, per_len, per_data, false));
#else
    WAIT_API(esp_ble_gap_config_periodic_adv_data_raw(ADV_HANDLE, per_len, per_data));
#endif
    return 0;
}

int per_adv_data_update(const uint8_t *per_data, uint8_t per_len)
{
    return ADV_LOCKED(per_adv_data_update_impl, per_data, per_len);
}

/* Replace the beacon's advertising data while it runs (blocking; app task only). */
static int beacon_adv_data_update_impl(const uint8_t *adv, uint8_t adv_len)
{
    WAIT_API(esp_ble_gap_config_ext_adv_data_raw(BEACON_HANDLE, adv_len, adv));
    return 0;
}

int beacon_adv_data_update(const uint8_t *adv, uint8_t adv_len)
{
    return ADV_LOCKED(beacon_adv_data_update_impl, adv, adv_len);
}

/* Change the beacon interval: stop, re-param, restart (blocking; app task only).
 * The stop is waited for now: its STOP_COMPLETE event gives adv_sem, and an
 * unwaited give would let the next WAIT_API return early. */
static int beacon_set_interval_impl(uint16_t ms)
{
    uint8_t h = BEACON_HANDLE;
    WAIT_API_SOFT(esp_ble_gap_ext_adv_stop(1, &h));
    beacon_adv_params.interval_min = ESP_BLE_GAP_ADV_ITVL_MS(ms);
    beacon_adv_params.interval_max = ESP_BLE_GAP_ADV_ITVL_MS(ms);
    WAIT_API(esp_ble_gap_ext_adv_set_params(BEACON_HANDLE, &beacon_adv_params));
    WAIT_API(esp_ble_gap_ext_adv_start(1, beacon_adv_inst));
    ESP_LOGI(TAG, "JBL beacon interval now %u ms", ms);
    return 0;
}

int beacon_set_interval(uint16_t ms)
{
    return ADV_LOCKED(beacon_set_interval_impl, ms);
}

void beacon_adv_restart(void)
{
    /* Non-blocking: called from the GATTS callback (same task as GAP events). */
    esp_err_t err = esp_ble_gap_ext_adv_start(1, beacon_adv_inst);
    ESP_LOGI(TAG, "JBL host beacon re-armed (%s)", esp_err_to_name(err));
}