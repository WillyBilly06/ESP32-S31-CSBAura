/*
 * btstack_aes128_esp.c - BTstack's AES-128 block (HAVE_AES128) on the S31's AES peripheral.
 *
 * BTstack's SMP needs a few dozen AES blocks per LE pairing (c1/s1 or the SC f4/f5/f6 CMACs,
 * CTKD h6/h7, address resolution). By default BTstack asks the controller for each one with
 * HCI LE Encrypt; on the S31 that answer was sometimes not a result, so we compute it here.
 * At the first call the peripheral is checked against FIPS-197 C.1; if it ever disagrees,
 * BTstack's own software AES (rijndael) is used instead.
 */
#include <string.h>

#include "aes/esp_aes.h"
#include "btstack_crypto.h"
#include "esp_log.h"
#include "rijndael.h"

static int s_hw = -1;      /* -1 not checked yet, 1 hardware AES, 0 rijndael */

static void aes_sw(const uint8_t *key, const uint8_t *in, uint8_t *out)
{
    uint32_t rk[RKLENGTH(128)];
    int nrounds = rijndaelSetupEncrypt(rk, key, 128);
    rijndaelEncrypt(rk, nrounds, in, out);
}

static bool aes_hw(const uint8_t *key, const uint8_t *in, uint8_t *out)
{
    esp_aes_context ctx;
    esp_aes_init(&ctx);
    bool ok = esp_aes_setkey(&ctx, key, 128) == 0 && esp_aes_crypt_ecb(&ctx, ESP_AES_ENCRYPT, in, out) == 0;
    esp_aes_free(&ctx);
    return ok;
}

static void self_test(void)
{
    static const uint8_t key[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                     0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };
    static const uint8_t pt[16]  = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                                     0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff };
    static const uint8_t ct[16]  = { 0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
                                     0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a };
    uint8_t out[16] = { 0 };
    s_hw = aes_hw(key, pt, out) && memcmp(out, ct, 16) == 0;
    ESP_LOGW("SMP", "AES-128 for pairing: %s", s_hw ? "S31 AES peripheral (FIPS-197 check ok)"
                                                   : "software (AES peripheral failed the FIPS-197 check)");
}

void btstack_aes128_calc(const uint8_t *key, const uint8_t *plaintext, uint8_t *ciphertext)
{
    if (s_hw < 0) self_test();
    if (s_hw && aes_hw(key, plaintext, ciphertext)) return;
    aes_sw(key, plaintext, ciphertext);
}
