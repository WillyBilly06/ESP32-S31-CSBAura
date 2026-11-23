/*
 * s31_ident.c - see s31_ident.h.
 *
 * "s31cfg" partition (4 KB, written by tools/flasher/s31_flasher.py):
 *   0  "S31C"        magic
 *   4  version       1
 *   5  color         JBL color ID
 *   6  flags         reserved, 0
 *   7  name_len      0..32
 *   8  name[32]      UTF-8, not terminated
 *   40 crc16         CRC-16/ARC of bytes 0..39, little-endian
 */
#include "s31_ident.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_partition.h"

static const char *TAG = "IDENT";

#define CFG_MAGIC   "S31C"
#define CFG_VERSION 1
#define CFG_SIZE    42

static uint8_t s_color = S31_COLOR_BLACK;
static char s_name[33];
static char s_serial[12];
static uint32_t s_std_id = 0x5a31c3u;
static bool s_done;

static uint16_t crc16_arc(const uint8_t *d, int n)
{
    uint16_t crc = 0;
    for (int i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++) crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
    }
    return crc;
}

static bool cfg_read(void)
{
    const esp_partition_t *p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "s31cfg");
    if (p == NULL) return false;
    uint8_t b[CFG_SIZE];
    if (esp_partition_read(p, 0, b, sizeof(b)) != ESP_OK) return false;
    if (memcmp(b, CFG_MAGIC, 4) != 0 || b[4] != CFG_VERSION) return false;
    uint16_t crc = (uint16_t)(b[40] | (b[41] << 8));
    if (crc != crc16_arc(b, 40)) {
        ESP_LOGE(TAG, "s31cfg partition CRC mismatch - using defaults");
        return false;
    }
    s_color = b[5];
    uint8_t n = b[7] > 32 ? 32 : b[7];
    memcpy(s_name, b + 8, n);
    s_name[n] = 0;
    return true;
}

void s31_ident_init(void)
{
    if (s_done) return;
    s_done = true;
    uint8_t mac[6] = { 0 };
    esp_read_mac(mac, ESP_MAC_BT);                /* factory-burned, unique per chip */
    snprintf(s_serial, sizeof(s_serial), "S31X5%02X%02X%02X", mac[3], mac[4], mac[5]);
    /* a per-board standard Broadcast_ID: phones list broadcasts by it */
    uint32_t id = ((uint32_t)crc16_arc(mac, 6) << 8) ^ mac[5] ^ ((uint32_t)mac[4] << 16);
    id &= 0xFFFFFFu;
    if (id == 0 || id == 0x112233u) id ^= 0x5a31c3u;
    s_std_id = id;
    bool cfg = cfg_read();
    ESP_LOGW(TAG, "identity: serial %s, standard Broadcast_ID 0x%06lx, color 0x%02x%s%s%s (%s)", s_serial,
             (unsigned long)s_std_id, s_color, s_name[0] ? ", name '" : "", s_name, s_name[0] ? "'" : "",
             cfg ? "from the s31cfg partition" : "no s31cfg partition: defaults");
}

uint8_t s31_ident_color(void) { return s_color; }
const char *s31_ident_name(void) { return s_name; }
const char *s31_ident_serial(void) { return s_serial; }
uint32_t s31_ident_std_broadcast_id(void) { return s_std_id; }
