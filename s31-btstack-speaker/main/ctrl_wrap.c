/*
 * ctrl_wrap.c - a link-time wrapper around the ESP32-S31 BLE controller's BIGInfo parser
 * (libble_app.a, obfuscated r_sym_ble_rOljucnfkqMusVymFaAm; linked with
 * -Wl,--wrap=..., see CMakeLists.txt).
 *
 * The parser decodes a received BIGInfo into the sync slot, then runs the validator
 * r_sym_ble_RtqZieNafYrxj8Y74NgO (~20 rules); on any failure the BIGInfo is dropped
 * silently - no LE BIGInfo Advertising Report, and LE BIG Create Sync answers 0x0C.
 *
 * Rule -20: when BIS_Spacing >= Sub_Interval the controller assumes SEQUENTIAL packing and
 * wants BIS_Spacing >= NSE * Sub_Interval. A JBL Xtreme 5 stereo primary sends ONE BIS with
 * BIS_Spacing == Sub_Interval (594 us) - legal INTERLEAVED packing (Sub_Interval >=
 * Num_BIS * BIS_Spacing). With one BIS both packings put subevent k at k * Sub_Interval,
 * so the wrapper rewrites BIS_Spacing to NSE * Sub_Interval (the sequential equivalent)
 * and runs the controller's validator again.
 *
 * Decoded slot layout (from the parser): +0 u16 BIG_Offset, +2 u16 ISO_Interval,
 * +4 u8 offset units, +5 u8 Num_BIS, +6 u8 NSE, +7 u8 BN, +8 u32 Sub_Interval,
 * +12 u32 BIS_Spacing, +16 u8 PTO, +17 u8 IRC, +18 u16 Max_PDU, +24 u32 SDU_Interval,
 * +28 u16 Max_SDU, +45 u8 PHY.
 *
 * Runs in the controller's context: no logging here, only copies.
 */
#include "ctrl_wrap.h"

#include <string.h>

int __real_r_sym_ble_rOljucnfkqMusVymFaAm(void *slot, const uint8_t *raw, int len);
int r_sym_ble_RtqZieNafYrxj8Y74NgO(void *slot);     /* the validator: 0 or a negative rule code */

volatile ctrl_biginfo_t g_ctrl_biginfo;
volatile bool g_ctrl_biginfo_bypass;

int __wrap_r_sym_ble_rOljucnfkqMusVymFaAm(void *slot, const uint8_t *raw, int len)
{
    int rc = __real_r_sym_ble_rOljucnfkqMusVymFaAm(slot, raw, len);
    int rule = 0, fixed = 0;
    if (rc != 0 && slot != NULL) {
        uint8_t *s = (uint8_t *)slot;
        rule = r_sym_ble_RtqZieNafYrxj8Y74NgO(slot);
        uint32_t sub, spc;
        memcpy(&sub, s + 8, 4);
        memcpy(&spc, s + 12, 4);
        /* only a fully decoded BIGInfo (33 B, or 57 B encrypted): a parser that failed earlier
         * left the slot holding the previous one */
        if (rule == -20 && (len == 33 || len == 57) && s[5] == 1 && spc >= sub && sub != 0) {   /* one BIS, interleaved */
            uint32_t seq = (uint32_t)s[6] * sub;
            memcpy(s + 12, &seq, 4);
            if (r_sym_ble_RtqZieNafYrxj8Y74NgO(slot) == 0) {
                rc = 0;
                fixed = 1;
            } else {
                memcpy(s + 12, &spc, 4);                 /* still invalid: as it was */
            }
        }
    }
    if (raw != NULL && len > 0) {
        uint8_t n = (uint8_t)(len > (int)sizeof(g_ctrl_biginfo.raw) ? sizeof(g_ctrl_biginfo.raw) : len);
        if (rc != g_ctrl_biginfo.rc || rule != g_ctrl_biginfo.rule || n != g_ctrl_biginfo.len ||
            memcmp((const void *)g_ctrl_biginfo.raw, raw, n) != 0) {
            memcpy((void *)g_ctrl_biginfo.raw, raw, n);
            g_ctrl_biginfo.len = n;
            g_ctrl_biginfo.rc = rc;
            g_ctrl_biginfo.rule = rule;
            g_ctrl_biginfo.changes++;
        }
        g_ctrl_biginfo.calls++;
        if (fixed) g_ctrl_biginfo.fixed++;
    }
    if (rc != 0 && g_ctrl_biginfo_bypass) {
        g_ctrl_biginfo.bypassed++;
        return 0;
    }
    return rc;
}
