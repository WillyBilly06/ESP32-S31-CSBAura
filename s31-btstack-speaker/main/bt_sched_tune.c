/*
 * bt_sched_tune.c - see bt_sched_tune.h and re/controller_sched.md.
 *
 * Priority model of the ESP32-S31 controller (4-bit priorities, higher wins,
 * BR/EDR and LE meet only in the HW list arbiter):
 *   BR/EDR ACL (peripheral):  min(14, dft[0] + min(14, (txq+rxq)>>2) + min(14, missed Tpoll))
 *                             dft[0] = 2 stock; cap 14 is an immediate in IRAM code.
 *   BIS subevent (k = per-BIS subevent index / BN = payload group):
 *                             k == 0 -> iso[33] (9), 0 < k < IRC -> iso[34] (7), else iso[35] (8)
 *   BIG control subevent:     iso[36] (12)
 *   periodic adv 10, ext/legacy adv 1/5 escalating +1 per skip to 9/10, scan 1/4
 *   (hard-coded in the LE code, not tunable).
 */
#include "bt_sched_tune.h"

#include <string.h>

#include "esp_log.h"
#include "esp_memory_utils.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "bt_sched";

/* ---- controller symbols (libbredr_app.a / libble_app.a) ------------------ */
extern uint8_t r_olc_sched_pti_dft_get(uint8_t idx);   /* tbl[2*idx]   */
extern uint8_t r_olc_sched_pti_inc_get(uint8_t idx);   /* tbl[2*idx+1] */
/* Obfuscated names of the July-2026 controller libs; the Nov-2026 libs renamed
 * them. Weak: on newer libs the LE (BIS priority) half is skipped, the BR/EDR
 * half still works. */
extern uint8_t *r_sym_ble_kBC0rRwtjATsFkriXVDL(void) __attribute__((weak));  /* returns r_sym_ble_u66q9VTswFBa3W6uzpwY */
extern uint8_t *r_sym_ble_u66q9VTswFBa3W6uzpwY __attribute__((weak));        /* 64-byte ISO config block (heap) */
/* Nov-2026 libs (IDF release/v6.1 c43d665a96): no getter, a global pointer to the
 * same block (libble_app.a ble_56.o; NULL until ISO is initialised). re/controller_sched.md s7 */
extern uint8_t *sym_ble_Ta1qTQXQA7wXSdDHIIz2 __attribute__((weak));
extern uint8_t _data_start[], _data_end[];

/* CIS (the phone's LE Audio stream into the S31) priority, cfg[37..39], stock 9/12/12
 * (adaptive between them). Hosting a party, the BIS repetitions (12) and pre-
 * transmissions (11) won the overlaps and ~15 % of the phone's SDUs never arrived
 * (PLC on the speaker and on the broadcast). 14: above the repetitions, below the
 * BIS first copies (15) - the CIS has its own retransmissions. */
#ifndef BT_SCHED_CIS
#define BT_SCHED_CIS 14
#endif
/* LE Audio input feeding the broadcast: CIS 15, BIS first copies 13 */
#define BT_SCHED_CIS_LE_IN   15
#define BT_SCHED_FIRST_LE_IN 15   /* 13 made no measurable difference to the CIS loss */
static bool s_le_input;

/* ---- profiles ------------------------------------------------------------ */
typedef struct {
    uint8_t acl;      /* BR/EDR ACL base priority (pti table dft[0]) */
    uint8_t first;    /* iso[33] BIS first copy of each payload  */
    uint8_t rep;      /* iso[34] BIS repetitions                 */
    uint8_t pto;      /* iso[35] BIS pre-transmission groups     */
    uint8_t ctrl;     /* iso[36] BIG control subevent            */
} prio_profile_t;

/* Not streaming (phone absent, connected-idle, paused) - both modes.
 * ACL stays at the stock 2 so idle-link escalation (missed polls) can never
 * reach the BIS first copies at 15; repetitions sit above every adv (<=10). */
#ifndef BT_SCHED_IDLE_ACL
#define BT_SCHED_IDLE_ACL   2
#endif
#ifndef BT_SCHED_IDLE_FIRST
#define BT_SCHED_IDLE_FIRST 15
#endif
#ifndef BT_SCHED_IDLE_REP
#define BT_SCHED_IDLE_REP   12
#endif
#ifndef BT_SCHED_IDLE_PTO
#define BT_SCHED_IDLE_PTO   11
#endif

static const prio_profile_t s_idle = {
    BT_SCHED_IDLE_ACL, BT_SCHED_IDLE_FIRST, BT_SCHED_IDLE_REP, BT_SCHED_IDLE_PTO, BT_SCHED_IDLE_FIRST,
};

/* Streaming profiles, indexed by bt_sched_mode_t. */
static const prio_profile_t s_stream[3] = {
    /* A2DP_FIRST: ACL 14 (= its cap, so the escalation terms change nothing and
     * the controller's "new pti" log stays quiet) > BIS 13 > 12 > 11 > PA 10 > adv > scan */
    [BT_SCHED_MODE_A2DP_FIRST]     = { 14, 13, 12, 11, 13 },
    /* AURACAST_FIRST: BIS first copy 15 > ACL 8..14 > repetitions 7 */
    [BT_SCHED_MODE_AURACAST_FIRST] = { 8, 15, 7, 7, 15 },
    /* MASTER: same as idle - stock ACL 2, all BIS copies above it */
    /* MASTER while streaming = idle (ACL 2, every BIS copy above it). Tested:
     * ACL 13 above the repetitions lost ~48 % of whole SDUs - the hardware does
     * not cut an ACL exchange already in progress, so it also eats first copies. */
    [BT_SCHED_MODE_MASTER]         = { BT_SCHED_IDLE_ACL, BT_SCHED_IDLE_FIRST, BT_SCHED_IDLE_REP, BT_SCHED_IDLE_PTO, BT_SCHED_IDLE_FIRST },
};

/* ---- table location / verification ---------------------------------------- */
#define PTI_ENTRIES 18
/* Stock {dft,inc} pairs; byte 0 (ACL dft) is not compared (may already be patched). */
static const uint8_t s_pti_sig[PTI_ENTRIES * 2] = {
    2, 1, 9, 2, 10, 1, 4, 1, 12, 2, 8, 1, 2, 1, 4, 1, 4, 1,
    4, 1, 2, 1, 10, 1, 3, 1, 11, 1, 6, 1, 6, 1, 4, 1, 4, 1,
};

static uint8_t *s_pti;                 /* located BR/EDR table, NULL until verified */
static bool s_pti_failed;              /* do not rescan after a failed scan */
static uint8_t s_pti_stock_acl;
static bt_sched_mode_t s_mode = BT_SCHED_MODE;
static bool s_streaming;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static bool pti_matches(const uint8_t *p)
{
    if (p[0] == 0 || p[0] > 15 || memcmp(p + 1, s_pti_sig + 1, sizeof(s_pti_sig) - 1) != 0) {
        return false;
    }
    /* Must be the very array the controller's getters read. */
    for (int i = 0; i < PTI_ENTRIES; i++) {
        if (r_olc_sched_pti_dft_get(i) != p[2 * i] || r_olc_sched_pti_inc_get(i) != p[2 * i + 1]) {
            return false;
        }
    }
    return true;
}

static uint8_t *pti_find(void)
{
    if (s_pti != NULL) {
        return pti_matches(s_pti) ? s_pti : NULL;
    }
    if (s_pti_failed) {
        return NULL;
    }
    uint8_t *hit = NULL;
    int hits = 0;
    for (uint8_t *p = _data_start; p + sizeof(s_pti_sig) <= _data_end; p++) {
        if (p[1] == s_pti_sig[1] && p[2] == s_pti_sig[2] &&
            memcmp(p + 1, s_pti_sig + 1, sizeof(s_pti_sig) - 1) == 0 && p[0] >= 1 && p[0] <= 15) {
            hit = p;
            hits++;
        }
    }
    if (hits != 1 || !pti_matches(hit)) {
        ESP_LOGE(TAG, "BR/EDR pti table: %d candidates, getter dft[0]=%u - ACL priority NOT tuned", hits,
                 r_olc_sched_pti_dft_get(0));
        s_pti_failed = true;
        return NULL;
    }
    s_pti = hit;
    s_pti_stock_acl = hit[0];
    ESP_LOGI(TAG, "BR/EDR pti table at %p (ACL dft %u inc %u)", hit, hit[0], hit[1]);
    return hit;
}

/* Returns the verified ISO config block or NULL. */
static uint8_t *iso_cfg_find(void)
{
    uint8_t *p = NULL;
    if (r_sym_ble_kBC0rRwtjATsFkriXVDL != NULL && &r_sym_ble_u66q9VTswFBa3W6uzpwY != NULL) {
        p = r_sym_ble_kBC0rRwtjATsFkriXVDL();                 /* Jul-2026 libs */
        if (p != r_sym_ble_u66q9VTswFBa3W6uzpwY) {
            p = NULL;
        }
    } else if (&sym_ble_Ta1qTQXQA7wXSdDHIIz2 != NULL) {
        p = sym_ble_Ta1qTQXQA7wXSdDHIIz2;                       /* Nov-2026 libs */
    } else {
        ESP_LOGW(TAG, "ISO priority block not known in this controller lib - BIS priorities left at stock");
        return NULL;
    }
    if (p == NULL || !esp_ptr_byte_accessible(p) ||
        !esp_ptr_byte_accessible(p + 63)) {
        ESP_LOGE(TAG, "ISO cfg block %p not valid - BIS priorities NOT tuned", p);
        return NULL;
    }
    uint32_t w0, w8, w12, w52;
    memcpy(&w0, p + 0, 4);
    memcpy(&w8, p + 8, 4);
    memcpy(&w12, p + 12, 4);
    memcpy(&w52, p + 52, 4);
    /* Fingerprint of the stock defaults (r_sym_ble_19UgKHutGrGyTis8atJz) around the
     * priority bytes, which themselves only need to be valid nibbles. */
    bool ok = w0 == 1250 && w8 == 20000 && w12 == 1000 && w52 == 500000 && p[32] == 0x1F &&
              ((p[37] == 9 && p[38] == 12 && p[39] == 12) ||
               (p[37] == p[38] && p[38] == p[39] && (p[37] == BT_SCHED_CIS || p[37] == BT_SCHED_CIS_LE_IN))) &&
              p[33] <= 15 && p[34] <= 15 && p[35] <= 15 && p[36] <= 15;
    if (!ok) {
        ESP_LOGE(TAG, "ISO cfg block %p fingerprint mismatch (w0=%lu b32=%02x b33..39=%u/%u/%u/%u/%u/%u/%u)"
                 " - BIS priorities NOT tuned", p, (unsigned long)w0, p[32], p[33], p[34], p[35], p[36],
                 p[37], p[38], p[39]);
        return NULL;
    }
    return p;
}

static void apply(const prio_profile_t *pr, const char *why)
{
    uint8_t *pti = pti_find();
    uint8_t *iso = iso_cfg_find();
    uint8_t old_acl = 0, old_iso[4] = {0};
    bool acl_ok = false, iso_ok = false;

    portENTER_CRITICAL(&s_lock);
    if (pti != NULL) {
        old_acl = pti[0];
        pti[0] = pr->acl;
    }
    uint8_t first = s_le_input && pr->first > BT_SCHED_FIRST_LE_IN ? BT_SCHED_FIRST_LE_IN : pr->first;
    uint8_t cis = s_le_input ? BT_SCHED_CIS_LE_IN : BT_SCHED_CIS;
    if (iso != NULL) {
        memcpy(old_iso, iso + 33, 4);
        iso[33] = first;
        iso[34] = pr->rep;
        iso[35] = pr->pto;
        iso[36] = pr->ctrl;
        iso[37] = iso[38] = iso[39] = cis;
    }
    portEXIT_CRITICAL(&s_lock);
    static bool cis_logged;
    if (iso != NULL && !cis_logged) {
        cis_logged = true;
        ESP_LOGW(TAG, "CIS priority %u (stock 9..12): the phone's LE Audio stream above the BIS repetitions",
                 BT_SCHED_CIS);
    }

    if (pti != NULL) {
        acl_ok = r_olc_sched_pti_dft_get(0) == pr->acl;
        if (!acl_ok) {
            ESP_LOGE(TAG, "ACL write did not stick (getter %u) - restoring %u", r_olc_sched_pti_dft_get(0), old_acl);
            pti[0] = old_acl;
        }
    }
    if (iso != NULL) {
        iso_ok = iso[33] == first && iso[34] == pr->rep && iso[35] == pr->pto && iso[36] == pr->ctrl;
    }
    bool changed = (acl_ok && old_acl != pr->acl) || (iso_ok && memcmp(old_iso, iso + 33, 4) != 0);
    if (changed || !acl_ok || !iso_ok) {
        ESP_LOGW(TAG, "%s [%s]: ACL %u->%s%u | BIS first %u->%u rep %u->%u pto %u->%u ctrl %u->%u%s", why,
                 s_mode == BT_SCHED_MODE_A2DP_FIRST ? "A2DP_FIRST" : s_mode == BT_SCHED_MODE_MASTER ? "MASTER" : "AURACAST_FIRST", old_acl,
                 acl_ok ? "" : "FAILED ", pr->acl, old_iso[0], first, old_iso[1], pr->rep, old_iso[2],
                 pr->pto, old_iso[3], pr->ctrl, iso_ok ? "" : " (ISO FAILED)");
    }
}

/* ---- public API ------------------------------------------------------------ */
void bt_sched_tune_init(void)
{
    s_streaming = false;
    apply(&s_idle, "init");
}

void bt_sched_tune_streaming(bool on)
{
    s_streaming = on;
    apply(on ? &s_stream[s_mode] : &s_idle, on ? "A2DP streaming" : "A2DP idle");
}

void bt_sched_tune_set_mode(bt_sched_mode_t mode)
{
    if (mode != BT_SCHED_MODE_A2DP_FIRST && mode != BT_SCHED_MODE_AURACAST_FIRST && mode != BT_SCHED_MODE_MASTER) {
        return;
    }
    s_mode = mode;
    bt_sched_tune_streaming(s_streaming);
}

void bt_sched_tune_le_input(bool on)
{
    if (s_le_input == on) return;
    s_le_input = on;
    ESP_LOGW(TAG, "LE Audio input %s: CIS %u, BIS first copies %s", on ? "on" : "off",
             on ? BT_SCHED_CIS_LE_IN : BT_SCHED_CIS, on ? "13 (below the CIS)" : "back to the profile");
    bt_sched_tune_streaming(s_streaming);
}

/* A/B testing: apply an explicit profile (same verification as the modes). */
void bt_sched_tune_apply_custom(uint8_t acl, uint8_t first, uint8_t rep, uint8_t pto, const char *why)
{
    const prio_profile_t pr = { acl, first, rep, pto, first };
    apply(&pr, why);
}
