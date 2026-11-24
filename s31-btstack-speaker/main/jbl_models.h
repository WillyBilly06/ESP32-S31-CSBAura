/* jbl_models.h - the JBL speakers the S31 can be (table: jbl_models_table.h, generated) */
#pragma once

#include <stdint.h>

typedef enum { JBL_FAM_V2, JBL_FAM_V5, JBL_FAM_TWS, JBL_FAM_ONE } jbl_family_t;

#define JBL_F_AURACAST      (1u << 0)
#define JBL_F_LLS           (1u << 1)   /* long-lasting stereo */
#define JBL_F_PLAYTIME      (1u << 2)
#define JBL_F_PRESET_EQ     (1u << 3)
#define JBL_F_NO_PARTYBOOST (1u << 4)
#define JBL_F_LIGHT         (1u << 5)
#define JBL_F_LE_AUDIO      (1u << 6)
#define JBL_F_REDIRECT      (1u << 7)   /* the JBL Portable app hands it to JBL One */
#define JBL_F_HFP           (1u << 8)
#define JBL_F_FEEDBACK      (1u << 9)   /* feedback tone setting */
#define JBL_F_EQ_BALANCE    (1u << 10)
#define JBL_F_EQ_OUTDOOR    (1u << 11)
#define JBL_F_MFB           (1u << 12)  /* multifunction button setting */
#define JBL_F_JBL_ONE       (1u << 13)  /* JBL One has a model config for it (V5 protocol too) */

typedef struct {
    uint8_t id;
    const char *name;
} jbl_color_t;

typedef struct {
    uint16_t pid;
    const char *name;       /* as the apps show it */
    const char *adv_name;   /* as the speaker advertises it */
    jbl_family_t family;
    const char *platform;
    uint32_t features;
    const jbl_color_t *colors;
    uint8_t ncolors;
    const jbl_color_t *one_colors;   /* JBL One's IDs for the same colors (NULL: same as colors) */
    uint8_t n_one_colors;
} jbl_model_t;

const jbl_model_t *jbl_model_by_pid(uint16_t pid);
const jbl_model_t *jbl_model_at(int i);
int jbl_model_count(void);
/* JBL One's color ID for the color `id` of the model's own (JBL Portable) list, matched by name */
uint8_t jbl_model_one_color(const jbl_model_t *m, uint8_t id);
