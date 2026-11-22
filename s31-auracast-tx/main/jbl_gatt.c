/*
 * JBL-speaker GATT database for the S31, so a peer that connects to our
 * beacon sees what it sees on a real JBL speaker:
 *
 *   GAP     Device Name "JBL Xtreme 4", Appearance 0x0885 (speaker)
 *   "excelpoint.cnl"  65786365-6c70-6f69-6e74-2e636e6c0000
 *        ...0001  read + notify  (speaker -> peer)
 *        ...0002  write          (peer -> speaker, commands)
 *   "excelpoint.com"  65786365-6c70-6f69-6e74-2e636f6d0000
 *        ...0001  read + notify
 *        ...0002  write
 *   Device Information 0x180A: manufacturer / model / firmware
 *
 * The two "excelpoint" services and their characteristic layout are copied
 * from the JBL Xtreme 5 (seen in nRF Connect). Their command protocol is not
 * known yet, so every write is logged (see gatts_cb in main.c).
 */

#include <string.h>

#include "esp_log.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"

#include "adv.h"
#include "jbl_gatt.h"

#define JBL_GAP_NAME        "JBL Xtreme 4"
#define JBL_GAP_APPEARANCE  0x0885

static const uint16_t uuid_prim = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t uuid_char = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t uuid_cccd = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t prop_read = ESP_GATT_CHAR_PROP_BIT_READ;

/* 128-bit UUIDs, little-endian as ESP-IDF stores them.
 * "excelpoint.cnl" = 65 78 63 65 6c 70 6f 69 6e 74 2e 63 6e 6c (big-endian). */
static const uint8_t cnl_svc[16] = { 0x00, 0x00, 0x6c, 0x6e, 0x63, 0x2e, 0x74, 0x6e,
                                     0x69, 0x6f, 0x70, 0x6c, 0x65, 0x63, 0x78, 0x65 };
static const uint8_t cnl_c1[16]  = { 0x01, 0x00, 0x6c, 0x6e, 0x63, 0x2e, 0x74, 0x6e,
                                     0x69, 0x6f, 0x70, 0x6c, 0x65, 0x63, 0x78, 0x65 };
static const uint8_t cnl_c2[16]  = { 0x02, 0x00, 0x6c, 0x6e, 0x63, 0x2e, 0x74, 0x6e,
                                     0x69, 0x6f, 0x70, 0x6c, 0x65, 0x63, 0x78, 0x65 };
static const uint8_t com_svc[16] = { 0x00, 0x00, 0x6d, 0x6f, 0x63, 0x2e, 0x74, 0x6e,
                                     0x69, 0x6f, 0x70, 0x6c, 0x65, 0x63, 0x78, 0x65 };
static const uint8_t com_c1[16]  = { 0x01, 0x00, 0x6d, 0x6f, 0x63, 0x2e, 0x74, 0x6e,
                                     0x69, 0x6f, 0x70, 0x6c, 0x65, 0x63, 0x78, 0x65 };
static const uint8_t com_c2[16]  = { 0x02, 0x00, 0x6d, 0x6f, 0x63, 0x2e, 0x74, 0x6e,
                                     0x69, 0x6f, 0x70, 0x6c, 0x65, 0x63, 0x78, 0x65 };

static uint8_t cnl_c1_val[20];
static uint8_t cnl_c2_val[244];
static uint8_t com_c1_val[20];
static uint8_t com_c2_val[244];
static uint8_t cnl_cccd_val[2];
static uint8_t com_cccd_val[2];

enum { XP_SVC, XP_C1_DECL, XP_C1_VAL, XP_C1_CCCD, XP_C2_DECL, XP_C2_VAL, XP_N };

#define RW (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE)

#define XP_TABLE(svc, c1, c1v, cccdv, c2, c2v) {                                              \
    [XP_SVC]     = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&uuid_prim,               \
                    ESP_GATT_PERM_READ, 16, 16, (uint8_t *)(svc)}},                              \
    [XP_C1_DECL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&uuid_char,               \
                    ESP_GATT_PERM_READ, 1, 1, (uint8_t *)&prop_read_notify}},                     \
    [XP_C1_VAL]  = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)(c1),                    \
                    ESP_GATT_PERM_READ, sizeof(c1v), 1, (c1v)}},                                  \
    [XP_C1_CCCD] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&uuid_cccd,               \
                    RW, 2, 2, (cccdv)}},                                                          \
    [XP_C2_DECL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&uuid_char,               \
                    ESP_GATT_PERM_READ, 1, 1, (uint8_t *)&prop_write}},                           \
    [XP_C2_VAL]  = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)(c2),                    \
                    ESP_GATT_PERM_WRITE, sizeof(c2v), 0, (c2v)}},                                 \
}

static const esp_gatts_attr_db_t cnl_db[XP_N] = XP_TABLE(cnl_svc, cnl_c1, cnl_c1_val, cnl_cccd_val, cnl_c2, cnl_c2_val);
static const esp_gatts_attr_db_t com_db[XP_N] = XP_TABLE(com_svc, com_c1, com_c1_val, com_cccd_val, com_c2, com_c2_val);

/* Device Information Service */
static const uint16_t uuid_dis = ESP_GATT_UUID_DEVICE_INFO_SVC;
static const uint16_t uuid_mfr = ESP_GATT_UUID_MANU_NAME;
static const uint16_t uuid_model = ESP_GATT_UUID_MODEL_NUMBER_STR;
static const uint16_t uuid_fw = ESP_GATT_UUID_FW_VERSION_STR;
static uint8_t dis_mfr[] = "HARMAN";
static uint8_t dis_model[] = "JBL Xtreme 4";
static uint8_t dis_fw[] = "S31-1.0";

enum { DIS_SVC, DIS_MFR_D, DIS_MFR_V, DIS_MODEL_D, DIS_MODEL_V, DIS_FW_D, DIS_FW_V, DIS_N };

#define DIS_CHAR(uuidp, val) \
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&uuid_char, ESP_GATT_PERM_READ, 1, 1, (uint8_t *)&prop_read}}, \
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)(uuidp), ESP_GATT_PERM_READ, sizeof(val) - 1, sizeof(val) - 1, (val)}}

static const esp_gatts_attr_db_t dis_db[DIS_N] = {
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&uuid_prim, ESP_GATT_PERM_READ, 2, 2, (uint8_t *)&uuid_dis}},
    DIS_CHAR(&uuid_mfr, dis_mfr),
    DIS_CHAR(&uuid_model, dis_model),
    DIS_CHAR(&uuid_fw, dis_fw),
};

static uint16_t cnl_handles[XP_N];
static uint16_t com_handles[XP_N];

void jbl_gatt_register(esp_gatt_if_t gatts_if)
{
    esp_ble_gap_set_device_name(JBL_GAP_NAME);
    esp_ble_gap_config_local_icon(JBL_GAP_APPEARANCE);

    esp_ble_gatts_create_attr_tab(cnl_db, gatts_if, XP_N, 0);
    esp_ble_gatts_create_attr_tab(com_db, gatts_if, XP_N, 1);
    esp_ble_gatts_create_attr_tab(dis_db, gatts_if, DIS_N, 2);
}

void jbl_gatt_on_attr_tab(const esp_ble_gatts_cb_param_t *p)
{
    const struct gatts_add_attr_tab_evt_param *t = &p->add_attr_tab;
    const char *name = t->svc_inst_id == 0 ? "excelpoint.cnl" :
                       t->svc_inst_id == 1 ? "excelpoint.com" : "Device Information";

    if (t->status != ESP_GATT_OK) {
        ESP_LOGE(TAG, "GATT %s table failed, status 0x%x", name, t->status);
        return;
    }
    if (t->svc_inst_id == 0 && t->num_handle == XP_N) {
        memcpy(cnl_handles, t->handles, sizeof(cnl_handles));
    } else if (t->svc_inst_id == 1 && t->num_handle == XP_N) {
        memcpy(com_handles, t->handles, sizeof(com_handles));
    }
    esp_ble_gatts_start_service(t->handles[0]);
    if (t->svc_inst_id <= 1) {
        ESP_LOGW(TAG, "GATT %s up: notify 0x%04x, write 0x%04x", name,
                 t->handles[XP_C1_VAL], t->handles[XP_C2_VAL]);
    } else {
        ESP_LOGW(TAG, "GATT %s up (start 0x%04x)", name, t->handles[0]);
    }
}

const char *jbl_gatt_handle_name(uint16_t handle)
{
    if (handle == cnl_handles[XP_C2_VAL]) return "cnl.0002 (write)";
    if (handle == cnl_handles[XP_C1_CCCD]) return "cnl.0001 CCCD";
    if (handle == com_handles[XP_C2_VAL]) return "com.0002 (write)";
    if (handle == com_handles[XP_C1_CCCD]) return "com.0001 CCCD";
    return "other";
}
