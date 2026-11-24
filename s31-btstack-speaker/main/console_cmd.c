/*
 * console_cmd.c - development commands on the console serial port (UART0, 921600).
 *
 * One command per line; each runs on the BTstack thread exactly like the real button /
 * JBL One action, so a test from the PC goes through the same code paths:
 *
 *   pair | aura | aura on | aura off | vol+ | vol- | play
 *   leaudio on | leaudio off      the JBL One "LE Audio" switch (V5 Set 0x0BA0)
 *   forget                        forget every phone (Classic keys + LE bonds)
 *   switchpair                    the LE Audio switch's forget + pairing, without changing the setting
 *   v5set <id> <bytes>            a JBL One Set (hex), e.g. "v5set 2c07 00" ungroups a stereo pair
 *   tone <name>                   play a prompt tone (poweron, bt_pair, ...)
 *   status                        one line: links, pairing, LE Audio, Auracast
 *   aestest                       controller AES (HCI LE Encrypt) against the FIPS-197 vector, 8 runs
 *   stacks                        every task's stack address and whether DMA can reach it
 *   csb rx <bd_addr>               receive a speaker's CPB (its sync train, then its broadcast)
 *   csb rx off | csb tx [slots] | csb tx off | csb data <hex>   CPB receive / transmit tests
 *   model [pid hex [color hex]]     list the JBL models / become one (NVS, reboot); "model 0" = flashed one
 *   reboot | help
 *
 * Send with: python tools/s31_console/s31ctl.py send "<command>"
 */
#include "bt_sched_tune.h"
#include "console_cmd.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "btstack.h"
#include "driver/uart.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_memory_utils.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/freertos_debug.h"
#include "sdkconfig.h"

#include "a2dp_in.h"
#include "power.h"
#include "auracast.h"
#include "jbl_v5.h"
#include "csb.h"
#include "lls.h"
#include "jbl_models.h"
#include "s31_ident.h"
#include "le_audio.h"
#include "party.h"
#include "tones.h"

static const char *TAG = "CMD";

#define CMD_LINE_MAX 64
static char s_line[CMD_LINE_MAX];                     /* the command being run on the BT thread */
static volatile bool s_busy;
static btstack_context_callback_registration_t s_cb;

static void status_line(void)
{
    static const char *st[] = { "off", "starting", "on", "stopping" };
    auracast_state_t a = auracast_state();
    ESP_LOGW(TAG, "status: A2DP %s%s | pairing %s%s | LE Audio %s, phone %s | Auracast %s (%s%s) | party %s",
             a2dp_in_connected() ? "connected" : "idle", a2dp_in_streaming() ? " streaming" : "",
             a2dp_in_pairing() ? "ON" : "off", a2dp_in_pairing_is_switch() ? " (LE Audio switch)" : "",
             le_audio_enabled() ? "on" : "off", le_audio_phone_linked() ? "linked" : "none",
             party_auracast_enabled() ? "enabled" : "disabled", (unsigned)a < 4 ? st[a] : "?",
             auracast_big_held() ? ", BIG held" : "", party_listening() ? "listening" : "-");
}

/* --- aestest: HCI LE Encrypt with FIPS-197 C.1 (key 00..0f, plain 0011..ff), LSB first --- */
static const uint8_t AES_KEY[16] = { 0x0f, 0x0e, 0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08,
                                     0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00 };
static const uint8_t AES_PT[16]  = { 0xff, 0xee, 0xdd, 0xcc, 0xbb, 0xaa, 0x99, 0x88,
                                     0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00 };
static const uint8_t AES_CT[16]  = { 0x5a, 0xc5, 0xb4, 0x70, 0x80, 0xb7, 0xcd, 0xd8,
                                     0x30, 0x04, 0x7b, 0x6a, 0xd8, 0xe0, 0xc4, 0x69 };
static int s_aes_left, s_aes_ok, s_aes_bad;
static bool s_aes_wait;
static btstack_timer_source_t s_aes_timer;
static btstack_packet_callback_registration_t s_aes_hci;

static void aes_next(btstack_timer_source_t *t)
{
    (void)t;
    if (s_aes_left <= 0) {
        ESP_LOGW(TAG, "aestest: %d ok, %d bad", s_aes_ok, s_aes_bad);
        return;
    }
    if (!s_aes_wait && hci_can_send_command_packet_now()) {
        s_aes_wait = true;
        hci_send_cmd(&hci_le_encrypt, AES_KEY, AES_PT);
    }
    btstack_run_loop_set_timer(&s_aes_timer, 30);
    btstack_run_loop_add_timer(&s_aes_timer);
}

static void aes_hci(uint8_t type, uint16_t ch, uint8_t *p, uint16_t len)
{
    (void)ch;
    if (type != HCI_EVENT_PACKET || !s_aes_wait || hci_event_packet_get_type(p) != HCI_EVENT_COMMAND_COMPLETE) return;
    if (hci_event_command_complete_get_command_opcode(p) != HCI_OPCODE_HCI_LE_ENCRYPT || len < 22) return;
    s_aes_wait = false;
    uint8_t st = p[5];
    bool ok = st == 0 && memcmp(&p[6], AES_CT, 16) == 0;
    ok ? s_aes_ok++ : s_aes_bad++;
    ESP_LOGW(TAG, "aestest %d: status 0x%02x, %s (%02x %02x %02x %02x ...)", 8 - s_aes_left + 1, st,
             ok ? "correct" : "WRONG", p[6], p[7], p[8], p[9]);
    s_aes_left--;
}

static void aes_test_start(void)
{
    if (!s_aes_hci.callback) {
        s_aes_hci.callback = aes_hci;
        hci_add_event_handler(&s_aes_hci);
    }
    s_aes_left = 8;
    s_aes_ok = s_aes_bad = 0;
    s_aes_wait = false;
    btstack_run_loop_remove_timer(&s_aes_timer);
    btstack_run_loop_set_timer_handler(&s_aes_timer, aes_next);
    aes_next(&s_aes_timer);
}

/* --- stacks: where each task's stack lives (the BT MAC's AES engine DMAs into the caller's stack) --- */
static void stacks_dump(void)
{
    static TaskSnapshot_t snap[40];
    UBaseType_t tcb_sz;
    UBaseType_t n = uxTaskGetSnapshotAll(snap, 40, &tcb_sz);
    for (UBaseType_t i = 0; i < n; i++) {
        void *sp = snap[i].pxTopOfStack;
        ESP_LOGW(TAG, "stack %-16s top %p end %p | %s%s", pcTaskGetName((TaskHandle_t)snap[i].pxTCB), sp,
                 snap[i].pxEndOfStack, esp_ptr_external_ram(sp) ? "PSRAM" : esp_ptr_internal(sp) ? "internal" : "other",
                 esp_ptr_dma_capable(sp) ? ", DMA ok" : ", NOT DMA-capable");
    }
}

static void run_cmd(void *ctx)
{
    (void)ctx;
    char *c = s_line;
    ESP_LOGW(TAG, "> %s", c);
    if (!strcmp(c, "pair")) {
        a2dp_in_button(A2DP_BTN_PAIRING);
    } else if (!strcmp(c, "aura")) {
        party_set_auracast_enabled(!party_auracast_enabled());
    } else if (!strcmp(c, "aura on") || !strcmp(c, "aura off")) {
        party_set_auracast_enabled(c[6] == 'n');
    } else if (!strcmp(c, "vol+")) {
        a2dp_in_button(A2DP_BTN_VOL_UP);
    } else if (!strcmp(c, "vol-")) {
        a2dp_in_button(A2DP_BTN_VOL_DOWN);
    } else if (!strcmp(c, "play")) {
        a2dp_in_button(A2DP_BTN_PLAY_PAUSE);
    } else if (!strcmp(c, "leaudio on") || !strcmp(c, "leaudio off")) {
        uint8_t v = c[9] == 'n' ? 0x80 : 0x00;
        jbl_v5_debug_set(0x0BA0, &v, 1);             /* exactly the JBL One switch */
    } else if (!strncmp(c, "prio ", 5)) {               /* prio <acl> <bis first> <bis rep> <bis pre-tx> */
        unsigned v[4];
        if (sscanf(c + 5, "%u %u %u %u", &v[0], &v[1], &v[2], &v[3]) == 4) {
            bt_sched_tune_apply_custom(v[0], v[1], v[2], v[3], "console");
        } else {
            ESP_LOGW(TAG, "usage: prio <acl> <bis first> <bis rep> <bis pre-tx>");
        }
    } else if (!strcmp(c, "lereach on") || !strcmp(c, "lereach off")) {
        le_audio_set_reachable(c[9] == 'n');
    } else if (!strncmp(c, "avrcpwait ", 10)) {
        a2dp_in_set_avrcp_wait((uint32_t)strtoul(c + 10, NULL, 10));
    } else if (!strcmp(c, "callback")) {
        a2dp_in_callback_now();
    } else if (!strcmp(c, "forget")) {
        int n = a2dp_in_forget_all();
        ESP_LOGW(TAG, "forgot every phone (Classic keys, %d LE bonds)", n);
    } else if (!strncmp(c, "v5set ", 6)) {
        /* v5set <id hex> <value hex bytes>: as if JBL One sent that Set */
        unsigned id = 0;
        uint8_t v[32];
        int n = 0, pos = 0;
        if (sscanf(c + 6, "%x%n", &id, &pos) == 1) {
            const char *q = c + 6 + pos;
            unsigned b;
            int k;
            while (n < (int)sizeof(v) && sscanf(q, "%2x%n", &b, &k) == 1) {
                v[n++] = (uint8_t)b;
                q += k;
                while (*q == ' ') q++;
            }
            jbl_v5_debug_set((uint16_t)id, v, (uint16_t)n);
            ESP_LOGW(TAG, "V5 Set 0x%04x (%d B) applied", id, n);
        } else {
            ESP_LOGW(TAG, "usage: v5set <id hex> <bytes hex>");
        }
    } else if (!strncmp(c, "csb ", 4)) {
        const char *a = c + 4;
        if (!strcmp(a, "rx off")) {
            csb_rx_stop();
        } else if (!strncmp(a, "join ", 5)) {
            bd_addr_t addr;
            if (sscanf_bd_addr(a + 5, addr)) csb_rx_join(addr);
        } else if (!strncmp(a, "rx ", 3)) {
            bd_addr_t addr;
            if (sscanf_bd_addr(a + 3, addr)) csb_rx_start(addr);
            else ESP_LOGW(TAG, "usage: csb rx <master BD_ADDR, aa:bb:cc:dd:ee:ff>");
        } else if (!strcmp(a, "tx off")) {
            csb_tx_stop();
        } else if (!strncmp(a, "tx", 2)) {
            unsigned slots = 0x50, pt = 0x0008, lpo = 0, sto = 0x2000;   /* csb tx [slots] [ptype hex] [lpo] [sto hex] */
            sscanf(a + 2, "%u %x %u %x", &slots, &pt, &lpo, &sto);
            csb_tx_set_params((uint16_t)pt, (uint8_t)lpo, (uint16_t)sto);
            static const uint8_t hello[] = { 'S', '3', '1', ' ', 'C', 'P', 'B' };
            csb_tx_start((uint16_t)slots, hello, sizeof(hello));
        } else if (!strncmp(a, "data ", 5)) {
            uint8_t v[200];
            int n = 0, k;
            unsigned b;
            const char *q = a + 5;
            while (n < (int)sizeof(v) && sscanf(q, "%2x%n", &b, &k) == 1) { v[n++] = (uint8_t)b; q += k; while (*q == ' ') q++; }
            csb_tx_data(v, (uint8_t)n);
        } else {
            ESP_LOGW(TAG, "usage: csb rx <addr> | csb rx off | csb tx [slots] | csb tx off | csb data <hex>");
        }
    } else if (!strcmp(c, "model")) {
        const jbl_model_t *cur = jbl_model_by_pid(s31_ident_model());
        ESP_LOGW(TAG, "this board: 0x%04x %s, color 0x%02x", s31_ident_model(), cur ? cur->name : "(not in the table)",
                 s31_ident_color());
        for (int i = 0; i < jbl_model_count(); i++) {
            const jbl_model_t *m = jbl_model_at(i);
            char cs[160];
            int k = 0;
            for (int j = 0; j < m->ncolors && k < (int)sizeof(cs) - 24; j++)
                k += snprintf(cs + k, sizeof(cs) - (size_t)k, "%s%02x %s", j ? ", " : "", m->colors[j].id, m->colors[j].name);
            ESP_LOGW(TAG, "  %04x %-22s %s | %s", m->pid, m->name,
                     m->family == JBL_FAM_V5 ? "V5" : m->family == JBL_FAM_V2 ? "V2" : m->family == JBL_FAM_TWS ? "TWS" : "One", cs);
        }
    } else if (!strncmp(c, "model ", 6)) {
        unsigned pid = 0, col = 0;
        int n = sscanf(c + 6, "%x %x", &pid, &col);
        if (n >= 1 && (pid == 0 || jbl_model_by_pid((uint16_t)pid))) {
            s31_ident_set_model((uint16_t)pid, (uint8_t)col);
            ESP_LOGW(TAG, "model %s - reboot to apply", pid ? "set" : "override removed");
        } else {
            ESP_LOGW(TAG, "usage: model <pid hex> [color hex] (see 'model'); 0 = the flashed model");
        }
    } else if (!strncmp(c, "beacons", 7)) {
        int sec = atoi(c + 7);
        lls_beacon_dump((uint16_t)(sec > 0 ? sec : 15));
    } else if (!strcmp(c, "switchpair")) {
        a2dp_in_button(A2DP_BTN_PAIRING_SWITCH);
    } else if (!strncmp(c, "tone ", 5)) {
        if (!tones_play(c + 5)) ESP_LOGW(TAG, "no tone '%s'", c + 5);
    } else if (!strcmp(c, "status")) {
        status_line();
    } else if (!strcmp(c, "aestest")) {
        aes_test_start();
    } else if (!strcmp(c, "stacks")) {
        stacks_dump();
    } else if (!strcmp(c, "reboot")) {
        ESP_LOGW(TAG, "rebooting (links closed first, as a power cycle)");
        power_reboot();
    } else if (!strcmp(c, "help") || !c[0]) {
        ESP_LOGW(TAG, "commands: pair | aura [on|off] | vol+ | vol- | play | leaudio on|off | forget | switchpair | v5set <id> <hex> | "
                      "tone <name> | status | aestest | stacks | reboot");
    } else {
        ESP_LOGW(TAG, "unknown command '%s' (help)", c);
    }
    s_busy = false;
}

static void cmd_task(void *arg)
{
    (void)arg;
    char buf[CMD_LINE_MAX];
    int n = 0;
    for (;;) {
        uint8_t ch;
        if (uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, &ch, 1, portMAX_DELAY) != 1) continue;
        if (ch == '\r' || ch == '\n') {
            if (n == 0) continue;
            buf[n] = 0;
            n = 0;
            while (s_busy) vTaskDelay(pdMS_TO_TICKS(10));   /* one command at a time */
            /* trim + lower case */
            char *p = buf;
            while (*p == ' ') p++;
            size_t l = strlen(p);
            while (l && p[l - 1] == ' ') p[--l] = 0;
            for (char *q = p; *q; q++) *q = (char)tolower((unsigned char)*q);
            strncpy(s_line, p, CMD_LINE_MAX - 1);
            s_line[CMD_LINE_MAX - 1] = 0;
            s_busy = true;
            s_cb.callback = run_cmd;
            s_cb.context = NULL;
            btstack_run_loop_execute_on_main_thread(&s_cb);
        } else if (n < CMD_LINE_MAX - 1 && ch >= 0x20 && ch < 0x7f) {
            buf[n++] = (char)ch;
        }
    }
}

void console_cmd_setup(void)
{
    /* RX through the driver; the log keeps writing to the UART as before */
    esp_err_t e = uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "console commands off: uart_driver_install %s", esp_err_to_name(e));
        return;
    }
    /* stack in PSRAM: a line reader, nothing timing-critical, internal RAM is the scarce one */
    xTaskCreatePinnedToCoreWithCaps(cmd_task, "console_cmd", 3072, NULL, 3, NULL, tskNO_AFFINITY,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_LOGW(TAG, "console commands on (help)");
}
