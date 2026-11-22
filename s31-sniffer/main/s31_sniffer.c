/*
 * ESP32-S31 BLE advertising observer
 * ----------------------------------------------------------------------------
 * Watches the air for JBL speakers and reports what they advertise, to answer:
 * is the Xtreme 5's button "Auracast" mode a standard LE Audio broadcast, or
 * something else? It does three things:
 *   1. Extended-advertising scan. For any device carrying Service Data under
 *      16-bit UUID 0xFDDF (the JBL/Harman UUID), it prints the full payload,
 *      and prints again ONLY when that payload changes — so toggling the
 *      speaker's Auracast shows the exact bytes that flip (you reported the
 *      tail going 40 00 00 -> 40 AA 00).
 *   2. Flags a standard Auracast broadcast: Service Data UUID 0x1852
 *      (Broadcast Audio Announcement) + the 3-byte Broadcast ID.
 *   3. If a device reports a periodic-advertising interval, it auto-creates a
 *      periodic sync. A successful sync = the device IS doing LE periodic
 *      advertising (the backbone of an LE Audio broadcast), and the periodic
 *      reports are dumped (that train carries the BASE / BIGInfo for LE Audio).
 *
 * Read the log: 0xFDDF + no periodic sync  => JBL discovery beacon only.
 *               periodic sync establishes   => LE broadcast present (promising).
 *               0x1852 seen                 => standard Auracast announcement.
 * ----------------------------------------------------------------------------
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#define TAG "S31SNIFF"
#define JBL_UUID    0xFDDF
#define AURACAST_UUID 0x1852

static SemaphoreHandle_t s_sem;

static esp_ble_ext_scan_params_t scan_params = {
    .own_addr_type   = BLE_ADDR_TYPE_PUBLIC,
    .filter_policy   = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_duplicate  = BLE_SCAN_DUPLICATE_DISABLE,   /* keep re-reports so we catch changes */
    .cfg_mask        = ESP_BLE_GAP_EXT_SCAN_CFG_UNCODE_MASK | ESP_BLE_GAP_EXT_SCAN_CFG_CODE_MASK,
    .uncoded_cfg     = {BLE_SCAN_TYPE_ACTIVE, 80, 80},
    .coded_cfg       = {BLE_SCAN_TYPE_ACTIVE, 80, 80},
};

static esp_ble_gap_periodic_adv_sync_params_t sync_params = {
    .filter_policy = 0, .sid = 0, .addr_type = BLE_ADDR_TYPE_RANDOM,
    .skip = 0, .sync_timeout = 1200,
};

/* remember last 0xFDDF payload + sync state per address (small table) */
typedef struct { esp_bd_addr_t addr; uint8_t last[64]; uint8_t len; bool synced; bool is_jbl; bool used; } peer_t;
static peer_t peers[8];

static peer_t *peer_find(const uint8_t *a) {
    for (int i = 0; i < 8; i++)
        if (peers[i].used && memcmp(peers[i].addr, a, 6) == 0) return &peers[i];
    return NULL;
}
static peer_t *peer_get(const uint8_t *a) {
    peer_t *p = peer_find(a);
    if (p) return p;
    int free_i = -1;
    for (int i = 0; i < 8; i++) if (!peers[i].used) { free_i = i; break; }
    if (free_i < 0) free_i = 0;
    memset(&peers[free_i], 0, sizeof(peer_t));
    memcpy(peers[free_i].addr, a, 6); peers[free_i].used = true;
    return &peers[free_i];
}

static void hex(const uint8_t *p, int n, char *out) {
    for (int i = 0; i < n; i++) sprintf(out + 3*i, "%02x ", p[i]);
    out[n ? 3*n-1 : 0] = 0;
}

/* walk AD structures; find a Service Data (0x16) whose 16-bit UUID == want.
 * returns pointer to the data AFTER the 2 UUID bytes, sets *dlen. */
static const uint8_t *find_svc_data(const uint8_t *d, int len, uint16_t want, int *dlen) {
    int i = 0;
    while (i + 1 < len) {
        int l = d[i]; if (l == 0) break;
        int t = d[i+1];
        if (i + 1 + l > len) break;
        if (t == 0x16 && l >= 3) {
            uint16_t uuid = d[i+2] | (d[i+3] << 8);
            if (uuid == want) { *dlen = l - 3; return &d[i+4]; }
        }
        i += l + 1;
    }
    return NULL;
}

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SET_EXT_SCAN_PARAMS_COMPLETE_EVT:
    case ESP_GAP_BLE_EXT_SCAN_START_COMPLETE_EVT:
        xSemaphoreGive(s_sem);
        break;

    case ESP_GAP_BLE_EXT_ADV_REPORT_EVT: {
        esp_ble_gap_ext_adv_report_t *r = &param->ext_adv_report.params;
        char mac[18];
        sprintf(mac, "%02x:%02x:%02x:%02x:%02x:%02x",
                r->addr[0],r->addr[1],r->addr[2],r->addr[3],r->addr[4],r->addr[5]);

        int dl = 0;
        const uint8_t *jbl = find_svc_data(r->adv_data, r->adv_data_len, JBL_UUID, &dl);
        int al = 0;
        const uint8_t *au  = find_svc_data(r->adv_data, r->adv_data_len, AURACAST_UUID, &al);

        /* 0x1852 standard Auracast — print on (addr,id) change, and SYNC to its
         * periodic train to confirm a real LE Audio broadcast + read the BASE. */
        if (au && al >= 3) {
            static uint8_t last_id[3]; static esp_bd_addr_t last_addr; static bool au_seen;
            if (!au_seen || memcmp(last_id, au, 3) || memcmp(last_addr, r->addr, 6)) {
                ESP_LOGW(TAG, "[%s] STANDARD AURACAST (0x1852) Broadcast_ID=%02x%02x%02x rssi=%d per_int=%d",
                         mac, au[2], au[1], au[0], r->rssi, r->per_adv_interval);
                memcpy(last_id, au, 3); memcpy(last_addr, r->addr, 6); au_seen = true;
            }
            static uint8_t last_ext[255]; static int last_ext_len = -1;
            if (r->adv_data_len != last_ext_len || memcmp(last_ext, r->adv_data, r->adv_data_len)) {
                char raw[3 * 120 + 1]; hex(r->adv_data, r->adv_data_len > 120 ? 120 : r->adv_data_len, raw);
                ESP_LOGW(TAG, "[%s] MASTER EXT ADV (%d B) sid=%d: %s", mac, r->adv_data_len, r->sid, raw);
                memcpy(last_ext, r->adv_data, r->adv_data_len); last_ext_len = r->adv_data_len;
            }
            peer_t *ap = peer_get(r->addr);
            if (r->per_adv_interval != 0 && !ap->synced) {
                ap->synced = true;
                ESP_LOGW(TAG, "[%s] AURACAST broadcaster PERIODIC (int=%d sid=%d) -> syncing to read BASE...",
                         mac, r->per_adv_interval, r->sid);
                sync_params.sid = r->sid; sync_params.addr_type = r->addr_type;
                memcpy(sync_params.addr, r->addr, 6);
                esp_ble_gap_periodic_adv_create_sync(&sync_params);
            }
        }

        peer_t *p = NULL;
        if (jbl) {
            p = peer_get(r->addr);
            p->is_jbl = true;
            bool changed = (p->len != dl) || memcmp(p->last, jbl, dl) != 0;
            if (changed) {
                char h[200]; hex(jbl, dl > 60 ? 60 : dl, h);
                char raw[256]; hex(r->adv_data, r->adv_data_len > 80 ? 80 : r->adv_data_len, raw);
                ESP_LOGW(TAG, "[%s] JBL 0xFDDF (%d B): %s", mac, dl, h);
                ESP_LOGI(TAG, "      rssi=%d sid=%d per_adv_interval=%d  evt_type=0x%x",
                         r->rssi, r->sid, r->per_adv_interval, r->event_type);
                ESP_LOGI(TAG, "      full adv: %s", raw);
                memcpy(p->last, jbl, dl); p->len = dl;
            }
        } else {
            p = peer_find(r->addr);   /* known device from an earlier 0xFDDF advert? */
        }

        /* Any advert from a known-JBL MAC that carries a periodic interval ->
         * sync to confirm an LE broadcast train (even if THIS advert has no 0xFDDF). */
        if (p && p->is_jbl && r->per_adv_interval != 0 && !p->synced) {
            p->synced = true;
            ESP_LOGW(TAG, "[%s] JBL has PERIODIC ADV (interval=%d sid=%d) -> creating sync...",
                     mac, r->per_adv_interval, r->sid);
            sync_params.sid = r->sid;
            sync_params.addr_type = r->addr_type;
            memcpy(sync_params.addr, r->addr, 6);
            esp_ble_gap_periodic_adv_create_sync(&sync_params);
        }
        break;
    }

    case ESP_GAP_BLE_PERIODIC_ADV_SYNC_ESTAB_EVT:
        ESP_LOGW(TAG, ">>> LE PERIODIC ADV CONFIRMED: sync ok=%d interval=%d phy=%d  => button mode IS LE broadcast",
                 param->periodic_adv_sync_estab.status,
                 param->periodic_adv_sync_estab.period_adv_interval,
                 param->periodic_adv_sync_estab.adv_phy);
        break;

    case ESP_GAP_BLE_PERIODIC_ADV_REPORT_EVT: {
        esp_ble_gap_periodic_adv_report_t *pr = &param->period_adv_report.params;
        /* Print on change, ignoring the JBL block's counter (NN) and changing byte (YY). */
        static uint8_t last[255]; static int last_len = -1; static uint32_t cnt;
        uint8_t cur[255]; int n = pr->data_length; memcpy(cur, pr->data, n);
        for (int i = 0; i + 18 < n; i++) {
            if (cur[i] == 0xff && cur[i + 1] == 0xdf && cur[i + 2] == 0xfd) {   /* mfg AD 0xFDDF */
                cur[i + 9] = 0; cur[i + 16] = 0;                               /* NN, YY */
                break;
            }
        }
        cnt++;
        if (n != last_len || memcmp(cur, last, n) || cnt % 100 == 0) {
            char h[3 * 100 + 1]; hex(pr->data, n > 100 ? 100 : n, h);
            ESP_LOGW(TAG, ">>> PERIODIC DATA (len=%d): %s", n, h);
            memcpy(last, cur, n); last_len = n;
        }
        break;
    }

    case ESP_GAP_BLE_PERIODIC_ADV_SYNC_LOST_EVT:
        ESP_LOGW(TAG, "periodic sync lost (master stopped?) - will re-sync on next session");
        for (int i = 0; i < 8; i++) peers[i].synced = false;
        break;

    default: break;
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    esp_bluedroid_config_t bcfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&bcfg));
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_cb));

    s_sem = xSemaphoreCreateBinary();
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_ERROR_CHECK(esp_ble_gap_set_ext_scan_params(&scan_params));
    xSemaphoreTake(s_sem, portMAX_DELAY);
    ESP_ERROR_CHECK(esp_ble_gap_start_ext_scan(0, 0));
    xSemaphoreTake(s_sem, portMAX_DELAY);

    ESP_LOGW(TAG, "Scanning. Toggle the Xtreme 5 Auracast button and watch for the 0xFDDF payload change + any PERIODIC/0x1852 lines.");
}
