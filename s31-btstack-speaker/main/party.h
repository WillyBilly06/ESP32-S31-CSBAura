#pragma once
#include <stdbool.h>

/* JBL party roles, as an Xtreme 5:
 *   HOST   (Auracast broadcast of the phone's audio) as soon as A2DP audio plays;
 *   LISTEN (sync to another JBL party master's broadcast, play it on I2S) once
 *          no audio has played for PARTY_IDLE_TO_LISTEN_MS, phone connected or not.
 * Playing always wins: audio starting while listening drops the listen at once.
 * At power-up the S31 hosts first, then falls back to LISTEN when idle. */

void party_setup(void);            /* BT thread, before HCI power on */
void party_on_hci_working(void);   /* BT thread, after auracast_host_start() */
bool party_listening(void);        /* any thread */
void party_log_stats(void);        /* any thread, every 2 s */
