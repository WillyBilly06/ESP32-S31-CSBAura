/*
 * s31_ident.c - see s31_ident.h.
 *
 * "s31cfg" partition (4 KB, written by tools/flasher/s31_flasher.py):
 *   0  "S31C"        magic
 *   4  version       1 or 2
 *   5  color         JBL color ID
 *   6  flags         reserved, 0
 *   7  name_len      0..32
 *   8  name[32]      UTF-8, not terminated
 *   v1: 40 crc16     CRC-16/ARC of bytes 0..39, little-endian
 *   v2: 40 id[6]     this speaker's Bluetooth address (all 0: the chip's own MAC)
 *       46 crc16     CRC-16/ARC of bytes 0..45, little-endian
 *
 * The identity (Classic + LE address, beacon address, macCrc, serial, Broadcast_ID) comes
 * from the chip's factory MAC, so every board is its own Xtreme 5 and keeps its pairings
 * across flashes. id[6] is no longer used (see s31_ident_init).
 */
#include "s31_ident.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "IDENT";

#define CFG_MAGIC "S31C"
#define CFG_READ  48

static uint8_t s_color = S31_COLOR_BLACK;
static char s_name[33];
static char s_serial[12];
static uint32_t s_std_id = 0x5a31c3u;
static uint8_t s_mac[6];
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

/* fills color / name; id_out = the flashed identity (or zeros); false: no valid partition */
static bool cfg_read(uint8_t id_out[6])
{
    memset(id_out, 0, 6);
    const esp_partition_t *p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "s31cfg");
    if (p == NULL) return false;
    uint8_t b[CFG_READ];
    if (esp_partition_read(p, 0, b, sizeof(b)) != ESP_OK) return false;
    if (memcmp(b, CFG_MAGIC, 4) != 0 || (b[4] != 1 && b[4] != 2)) return false;
    int body = b[4] == 1 ? 40 : 46;
    uint16_t crc = (uint16_t)(b[body] | (b[body + 1] << 8));
    if (crc != crc16_arc(b, body)) {
        ESP_LOGE(TAG, "s31cfg partition CRC mismatch - using defaults");
        return false;
    }
    s_color = b[5];
    uint8_t n = b[7] > 32 ? 32 : b[7];
    memcpy(s_name, b + 8, n);
    s_name[n] = 0;
    if (b[4] == 2) memcpy(id_out, b + 40, 6);
    return true;
}

static bool id_usable(const uint8_t id[6])
{
    static const uint8_t zero[6] = { 0 }, ones[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    return memcmp(id, zero, 6) != 0 && memcmp(id, ones, 6) != 0 && (id[0] & 1) == 0;   /* unicast */
}

/* pairings belong to the Bluetooth address they were made with: if NVS holds the bonds of
 * another address (an older firmware that set its own address), clear them once */
static void nvs_follow_identity(const uint8_t id[6])
{
    uint8_t old[6] = { 0 };
    size_t sz = sizeof(old);
    nvs_handle_t h;
    bool known = false, same = false;
    if (nvs_open("s31id", NVS_READONLY, &h) == ESP_OK) {
        known = nvs_get_blob(h, "mac", old, &sz) == ESP_OK && sz == 6;
        same = known && memcmp(old, id, 6) == 0;
        nvs_close(h);
    }
    if (same) return;
    if (known) {
        ESP_LOGW(TAG, "Bluetooth address changed: clearing the pairings made with the old one");
        nvs_flash_deinit();
        nvs_flash_erase();
        nvs_flash_init();
    }
    if (nvs_open("s31id", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, "mac", id, 6);
        nvs_commit(h);
        nvs_close(h);
    }
}

void s31_ident_init(void)
{
    if (s_done) return;
    s_done = true;
    uint8_t id[6];
    bool cfg = cfg_read(id);
    /* The Bluetooth address stays the chip's factory MAC (unique per chip). Setting another
     * one (esp_iface_mac_addr_set) is accepted by the ESP32-S31 controller, but its BR/EDR
     * authentication then fails on every reconnect: phones pair once and never reconnect,
     * and LE Audio breaks with it. A v2 s31cfg id from that time is ignored. */
    (void)id_usable;
    esp_read_mac(s_mac, ESP_MAC_BT);
    nvs_follow_identity(s_mac);
    bool own_id = false;
    snprintf(s_serial, sizeof(s_serial), "S31X5%02X%02X%02X", s_mac[3], s_mac[4], s_mac[5]);
    /* a per-speaker standard Broadcast_ID: phones list broadcasts by it */
    uint32_t sid = ((uint32_t)crc16_arc(s_mac, 6) << 8) ^ s_mac[5] ^ ((uint32_t)s_mac[4] << 16);
    sid &= 0xFFFFFFu;
    if (sid == 0 || sid == 0x112233u) sid ^= 0x5a31c3u;
    s_std_id = sid;
    ESP_LOGW(TAG, "identity: BT %02X:%02X:%02X:%02X:%02X:%02X (%s), serial %s, standard Broadcast_ID 0x%06lx, "
             "color 0x%02x%s%s%s", s_mac[0], s_mac[1], s_mac[2], s_mac[3], s_mac[4], s_mac[5],
             own_id ? "flashed identity" : cfg ? "chip MAC" : "chip MAC, no s31cfg partition", s_serial,
             (unsigned long)s_std_id, s_color, s_name[0] ? ", name '" : "", s_name, s_name[0] ? "'" : "");
}

uint8_t s31_ident_color(void) { return s_color; }
const char *s31_ident_name(void) { return s_name; }
const char *s31_ident_serial(void) { return s_serial; }
uint32_t s31_ident_std_broadcast_id(void) { return s_std_id; }

void s31_ident_beacon_addr(uint8_t out[6])
{
    /* random static address (top bits 11), different per speaker, stable for it */
    uint16_t c = crc16_arc(s_mac, 6);
    out[0] = (uint8_t)(0xC0 | (s_mac[5] & 0x3F));
    out[1] = s_mac[4] ^ 0x58;
    out[2] = s_mac[3] ^ 0x34;
    out[3] = (uint8_t)(c >> 8);
    out[4] = (uint8_t)c;
    out[5] = s_mac[2] ^ 0x04;
}
