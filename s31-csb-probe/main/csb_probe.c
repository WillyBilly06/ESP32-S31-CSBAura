/*
 * ESP32-S31 CSB/CPB controller-capability probe
 * =============================================================================
 * The question: does the ESP32-S31 Bluetooth CONTROLLER implement the
 * Connectionless (Peripheral/Slave) Broadcast HCI command set that JBL
 * PartyBoost is built on? ESP-IDF exposes no high-level CSB API, so the only
 * possible route is raw VHCI. This firmware sends raw HCI over VHCI and tells
 * you whether the controller recognizes the CSB command set (Core spec 4.1+):
 *   Link Control (OGF 0x01):  0x0441 Set_CSB, 0x0442 Set_CSB_Receive,
 *                             0x0443 Start_Sync_Train, 0x0444 Receive_Sync_Train
 *   Ctrl & Baseband (0x03):   0x0C74/0x0C75 Set/Delete_Reserved_LT_ADDR,
 *                             0x0C76 Set_CSB_Data, 0x0C77/0x0C78 R/W Sync_Train_Params
 * (An earlier note here called 0x0C76 "Set_CSB" — wrong; it's Set_CSB_Data.)
 *
 * Three checks, all printed:
 *   1) HCI_Read_Local_Supported_Commands (0x1002): dumps the 64-byte supported-
 *      commands bitmap (RAW HEX — decode the CSB bits against the Core spec's
 *      "Supported Commands" table rather than trusting a hard-coded index).
 *   2) Read_Synchronization_Train_Parameters (0x0C77): no params, pure read,
 *      zero side effects.
 *   3) Set_Connectionless_Slave_Broadcast (0x0441) with Enable=0 (safe no-op).
 * For 2) and 3) the Command Complete status is the verdict:
 *        status 0x01  "Unknown HCI Command"  -> controller lacks that command
 *        any other     (0x00/0x0C/0x11/0x12) -> controller RECOGNIZES it
 *
 * Build:  idf.py set-target esp32s31 && idf.py build flash monitor
 * Look for the line beginning ">>> CSB VERDICT".
 *
 * NOTE: CSB is BR/EDR (Classic). If the controller can't enable a Classic mode
 * at all, that alone means no PartyBoost path — the log says so.
 * =============================================================================
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

static const char *TAG = "csb_probe";

#define H4_CMD 0x01
#define OP(ogf, ocf)  ((uint16_t)(((ogf) << 10) | (ocf)))
#define OPCODE_RESET          OP(0x03, 0x003)   /* 0x0C03 HCI_Reset              */
#define OPCODE_READ_SUP_CMDS  OP(0x04, 0x002)   /* 0x1002 Read_Local_Sup_Commands*/
#define OPCODE_SET_CSB        OP(0x01, 0x041)   /* 0x0441 Set_Connectionless_SB  */
#define OPCODE_RD_SYNC_TRAIN  OP(0x03, 0x077)   /* 0x0C77 Read_Sync_Train_Params */

static void hexdump(const char *label, const uint8_t *p, int n)
{
    char line[3 * 16 + 1];
    ESP_LOGI(TAG, "%s (%d bytes):", label, n);
    for (int i = 0; i < n; i += 16) {
        int m = (n - i) > 16 ? 16 : (n - i);
        for (int j = 0; j < m; j++) sprintf(line + 3 * j, "%02x ", p[i + j]);
        line[3 * m] = 0;
        ESP_LOGI(TAG, "  [octet %2d] %s", i, line);
    }
}

static void vhci_send_available(void) { }

/* Controller -> host: HCI event packets. [0x04][evt][plen][params...] */
static int vhci_recv(uint8_t *data, uint16_t len)
{
    if (len < 3 || data[0] != 0x04) return 0;
    uint8_t evt = data[1];
    if (evt == 0x0E && len >= 7) {               /* Command Complete */
        uint16_t opcode = data[4] | (data[5] << 8);
        uint8_t  status = data[6];
        if (opcode == OPCODE_READ_SUP_CMDS) {
            hexdump("Supported_Commands bitmap", &data[7], len - 7);
            ESP_LOGI(TAG, "    -> inspect octets 31-33 for the CSB command bits");
        } else if (opcode == OPCODE_RD_SYNC_TRAIN) {
            ESP_LOGW(TAG, ">>> CSB VERDICT: Read_Sync_Train_Params(0x0C77) status=0x%02x -> %s",
                     status,
                     status == 0x01 ? "UNKNOWN COMMAND (no sync-train support)"
                                    : "RECOGNIZED (sync train supported)");
        } else if (opcode == OPCODE_SET_CSB) {
            ESP_LOGW(TAG, ">>> CSB VERDICT: Set_CSB(0x0441) status=0x%02x -> %s",
                     status,
                     status == 0x01
                         ? "UNKNOWN COMMAND: controller has NO CSB/PartyBoost path"
                         : "RECOGNIZED: controller implements the CSB opcodes");
        } else {
            ESP_LOGI(TAG, "CmdComplete op=0x%04x status=0x%02x", opcode, status);
        }
    } else if (evt == 0x0F && len >= 7) {        /* Command Status */
        ESP_LOGI(TAG, "CmdStatus status=0x%02x op=0x%04x",
                 data[3], data[5] | (data[6] << 8));
    }
    return 0;
}

static esp_vhci_host_callback_t vhci_cb = {
    .notify_host_send_available = vhci_send_available,
    .notify_host_recv = vhci_recv,
};

static void send_cmd(uint16_t opcode, const uint8_t *params, uint8_t plen)
{
    uint8_t buf[4 + 255];
    buf[0] = H4_CMD;
    buf[1] = opcode & 0xff;
    buf[2] = opcode >> 8;
    buf[3] = plen;
    if (plen) memcpy(&buf[4], params, plen);
    for (int i = 0; i < 100 && !esp_vhci_host_check_send_available(); i++)
        vTaskDelay(pdMS_TO_TICKS(10));
    esp_vhci_host_send_packet(buf, 4 + plen);
    vTaskDelay(pdMS_TO_TICKS(150));
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&cfg));

    /* CSB needs a Classic-capable controller mode. */
    esp_err_t e = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "enable(BTDM/Classic) failed (0x%x). If only BLE enables, the"
                      " controller has no Classic radio active -> no PartyBoost.", e);
        if (esp_bt_controller_enable(ESP_BT_MODE_BLE) != ESP_OK) {
            ESP_LOGE(TAG, "enable(BLE) also failed; aborting"); return;
        }
        ESP_LOGW(TAG, "Running BLE-only: a CSB 'unknown command' result below is"
                      " expected because Classic isn't active.");
    }

    ESP_ERROR_CHECK(esp_vhci_host_register_callback(&vhci_cb));
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "Probing controller over VHCI ...");
    send_cmd(OPCODE_RESET, NULL, 0);
    send_cmd(OPCODE_READ_SUP_CMDS, NULL, 0);
    send_cmd(OPCODE_RD_SYNC_TRAIN, NULL, 0);

    /* Set_CSB (0x0441), Enable=0: 11-byte block = Enable, LT_ADDR, LPO_Allowed,
     * Packet_Type[2], Interval_Min[2], Interval_Max[2], Supervision_TO[2]. */
    uint8_t set_csb[11] = {0};
    send_cmd(OPCODE_SET_CSB, set_csb, sizeof(set_csb));

    ESP_LOGI(TAG, "Probe complete — see '>>> CSB VERDICT' above.");
}
