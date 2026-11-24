/* jbl_models.c - lookup over the generated model table */
#include "jbl_models.h"

#include <stddef.h>
#include <string.h>

#include "jbl_models_table.h"

const jbl_model_t *jbl_model_by_pid(uint16_t pid)
{
    for (unsigned i = 0; i < JBL_MODEL_COUNT; i++) {
        if (s_models[i].pid == pid) return &s_models[i];
    }
    return NULL;
}

const jbl_model_t *jbl_model_at(int i)
{
    return (i >= 0 && (unsigned)i < JBL_MODEL_COUNT) ? &s_models[i] : NULL;
}

int jbl_model_count(void) { return (int)JBL_MODEL_COUNT; }

uint8_t jbl_model_one_color(const jbl_model_t *m, uint8_t id)
{
    if (m == NULL || m->one_colors == NULL) return id;
    for (int i = 0; i < m->ncolors; i++) {
        if (m->colors[i].id != id) continue;
        for (int k = 0; k < m->n_one_colors; k++) {
            if (strcmp(m->one_colors[k].name, m->colors[i].name) == 0) return m->one_colors[k].id;
        }
    }
    return m->n_one_colors ? m->one_colors[0].id : id;
}
