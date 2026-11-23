#pragma once
#include <stdbool.h>

/* Soft power: the S31 has no power switch, so "off" (JBL One auto power off,
 * remote power off) plays the power-off tone, shuts the radio down and waits;
 * any button press restarts the speaker. */

void power_soft_off(const char *why);  /* BT thread */
bool power_is_off(void);               /* any thread */
void power_wake(void);                 /* any thread: restart if off */
