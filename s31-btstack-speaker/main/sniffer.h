#pragma once

/* Auracast / JBL party sniffer (build with S31_SNIFFER=1 in main.c). BT thread. */
void sniffer_setup(void);       /* before HCI power on */
void sniffer_start(void);       /* HCI working */
void sniffer_log_stats(void);   /* every 2 s */
