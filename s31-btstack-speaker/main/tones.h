#pragma once
#include <stdbool.h>

/* Speaker prompt tones (the Xtreme 5's own set), played on the local I2S line
 * over whatever is playing (music ducked while a tone sounds). The PCM lives in
 * the "tones" flash partition, written from local files with tools/tones_pack.py;
 * without it the speaker simply stays silent. JBL One's feedback tone switch
 * (0x0C40) turns them off. Names as on the Xtreme 5: poweron, poweroff, bt_cnt,
 * bt_pair, ent_aura, exi_aura, vol_max, indicate, c_err, batlow, mic_con,
 * usound, 0..9. */

void tones_setup(void);             /* once, after audio_out_setup() */
bool tones_play(const char *name);  /* any thread; false if missing or tones are off */
void tones_stop(void);              /* any thread */
