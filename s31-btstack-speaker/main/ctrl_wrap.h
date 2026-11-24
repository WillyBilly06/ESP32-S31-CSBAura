/* ctrl_wrap.h - BIGInfo parser wrapper around the BLE controller, see ctrl_wrap.c */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t raw[58];        /* the BIGInfo as received (33 B, or 57 B encrypted) */
    uint8_t len;
    int rc;                 /* what the controller gets: 0 = valid (after any fix) */
    int rule;               /* the validator's verdict before any fix: 0 or a negative rule code */
    uint32_t calls, changes, bypassed, fixed;
} ctrl_biginfo_t;

extern volatile ctrl_biginfo_t g_ctrl_biginfo;
/* true: a BIGInfo still invalid after the fixes is reported as valid (diagnostics) */
extern volatile bool g_ctrl_biginfo_bypass;
