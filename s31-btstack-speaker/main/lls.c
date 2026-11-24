/*
 * lls.c - JBL Xtreme 5 stereo pair ("letws") link, as the Xtreme 5 firmware does it
 * (re/fw_jbl/REPORT_X5_STEREO.md, local):
 *
 *   secondary (role 1) connectable low-duty DIRECTED EXTENDED advert (props 0x0005, 50 ms,
 *                      SID 4, 1M/1M, no data) from its public address to the primary's
 *                      public address (JBL One's 0x2C81); it is the GATT server of
 *                      "excelpoint.cnl" 65786365-6c70-6f69-6e74-2e636e6c0000
 *   primary (role 2)   passive scan; a directed advert whose AdvA is the partner (public)
 *                      -> LE connect (30 ms, latency 0, 8 s); finds cnl, subscribes to
 *                      ...0001 (notify), writes without response to ...0002
 *   handshake          `EE EE 01 06 00x6` version from both; the peripheral also sends
 *                      `EE EE 02 09 <9 B info>` at once, the central answers a 01 with a 02;
 *                      receiving a 02 = linked (JBL One 0x2C80 = 1)
 *   then               `AA <cmd> <len> <payload>` frames: 11/12 device info, 13 group info,
 *                      00 ack, 31 keys, F1/F5 broadcast sync (logged, acked)
 *   audio              the primary runs its Auracast broadcast; on `AA F1 00` it hands the
 *                      broadcast's periodic advertising to the secondary with HCI 0x205B (PAST)
 * Frames the link can't take at once wait in a small queue. BT thread only.
 */
#include "lls.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#include "a2dp_in.h"
#include "auracast.h"
#include "party.h"
#include "power.h"
#include "tones.h"
#include "jbl_util.h"
#include "jbl.h"
#include "jbl_gatt.h"
#include "jbl_v5.h"
#include "s31_ident.h"

static const char *TAG = "LLS";

#define LLS_DEFAULT_S   20

static const uint8_t CNL_SERVICE[16] = { 0x65, 0x78, 0x63, 0x65, 0x6c, 0x70, 0x6f, 0x69,
                                         0x6e, 0x74, 0x2e, 0x63, 0x6e, 0x6c, 0x00, 0x00 };

typedef enum { L_IDLE, L_ADV, L_SCAN, L_CONNECTING, L_DISCOVER, L_CHARS, L_SUBSCRIBE, L_WAIT_CCC, L_HANDSHAKE,
               L_LINKED } lls_state_t;
static const char *const s_names[] = { "idle", "advertising", "scanning", "connecting", "service discovery",
                                       "characteristics", "subscribing", "waiting for subscription", "handshake",
                                       "LINKED" };

static lls_state_t s_state;
static uint8_t s_role;
static bd_addr_t s_partner;             /* its public address */
static lls_group_t s_group;
static lls_cb_t s_cb;
static hci_con_handle_t s_con = HCI_CON_HANDLE_INVALID;
static bool s_central;                  /* we are the LE central (primary) on s_con */
static btstack_timer_source_t s_timer;
static btstack_packet_callback_registration_t s_hci_reg;

static uint8_t s_adv_handle = 0xFF;            /* created once, reused (removal is asynchronous) */
static bool s_adv_on;
static le_advertising_set_t s_set;
static le_extended_advertising_parameters_t s_params = {
    .advertising_event_properties = 0x0005,        /* connectable | directed (low duty), extended */
    .primary_advertising_interval_min = 0x20,      /* 20 ms (the Xtreme 5: 50 ms): found sooner */
    .primary_advertising_interval_max = 0x20,
    .primary_advertising_channel_map = 0x07,
    .own_address_type = BD_ADDR_TYPE_LE_PUBLIC,    /* the address the app gave the partner (0x000A) */
    .peer_address_type = BD_ADDR_TYPE_LE_PUBLIC,
    .advertising_filter_policy = 0,
    .advertising_tx_power = 127,
    .primary_advertising_phy = 1,
    .secondary_advertising_max_skip = 0,
    .secondary_advertising_phy = 1,
    .advertising_sid = 4,
    .scan_request_notification_enable = 0,
};

static gatt_client_service_t s_svc;
static bool s_svc_found;
static gatt_client_characteristic_t s_ntf, s_wr;
static bool s_ntf_found, s_wr_found;
static gatt_client_notification_t s_listener;

static uint32_t s_starts, s_links, s_fails, s_rx_frames, s_tx_frames, s_directed_seen, s_queued, s_past_sent;

#define TXQ_N   8
#define TXQ_MAX 128                  /* AA 93 / 92 EQ frames are 118 B */
static struct { uint8_t d[TXQ_MAX]; uint8_t n; } s_txq[TXQ_N];
static int s_txq_head, s_txq_len;
static btstack_context_callback_registration_t s_wr_reg;
static bool s_wr_wait;

/* PAST: the secondary asked for our broadcast; sent once the broadcast is on */
static bool s_past_pending;
static int s_past_tries;
static btstack_timer_source_t s_past_timer;
static const hci_cmd_t hci_le_pa_set_info_transfer = { 0x205B, "H21" };   /* conn, service data, adv handle */

/* after linking: watch the partner's beacon for a while (what a real linked speaker advertises) */
static btstack_timer_source_t s_watch_timer;
static bool s_watching;
static bool s_watch_all;                     /* console "beacons": every JBL beacon, not only the partner */
static uint16_t s_partner_crc[2];
static btstack_timer_source_t s_hs_timer;    /* secondary: handshake (re)send */
/* the partner's AA 12 (what the app reads through Forward Gets) */
static struct { bool valid; uint8_t batt; char serial[17]; uint8_t ver[4]; uint16_t mv; } s_pinfo;
static btstack_timer_source_t s_f4_timer;    /* battery voltage to the partner, every 5 s */
static btstack_timer_source_t s_id_timer;    /* partner identify (950 ms) */
static btstack_timer_source_t s_ug_timer;    /* ungroup: close after the partner's ack, 2 s at most */
static bool s_ungrouping;
static btstack_timer_source_t s_sync_timer;  /* secondary: broadcast sync watch (F1 / F5) */
static int s_sync_tries;
static bool s_synced;
static const hci_cmd_t hci_le_set_past_params = { 0x205C, "H1221" };   /* conn, mode, skip, timeout, CTE */
static int s_hs_tries;

static void set_state(lls_state_t st)
{
    if (st != s_state) ESP_LOGW(TAG, "%s -> %s", s_names[s_state], s_names[st]);
    s_state = st;
}

static void hexlog(const char *what, const uint8_t *d, uint16_t n)
{
    char h[3 * 48 + 4];
    int o = 0;
    for (uint16_t i = 0; i < n && i < 48; i++) o += snprintf(h + o, sizeof(h) - (size_t)o, "%02x ", d[i]);
    if (n > 48) snprintf(h + o, sizeof(h) - (size_t)o, "...");
    ESP_LOGW(TAG, "%s (%u): %s", what, n, h);
}

static void adv_stop(void)
{
    if (s_adv_handle != 0xFF && s_adv_on) gap_extended_advertising_stop(s_adv_handle);
    s_adv_on = false;
}

static bool s_forever;                       /* lls_start(LLS_FOREVER): a stored pair, never give up */
static btstack_timer_source_t s_retry_timer;
static void retry_cb(btstack_timer_source_t *ts);

static void finish(bool linked, const char *why)
{
    btstack_run_loop_remove_timer(&s_timer);
    if (!linked) {
        btstack_run_loop_remove_timer(&s_f4_timer);
        btstack_run_loop_remove_timer(&s_id_timer);
        s_pinfo.valid = false;
        btstack_run_loop_remove_timer(&s_sync_timer);
        btstack_run_loop_remove_timer(&s_hs_timer);
        party_set_stereo_secondary(false);
        a2dp_in_set_stereo_secondary(false);
    }
    if (!linked) {
        s_txq_len = 0;
        s_wr_wait = false;
        s_past_pending = false;
        btstack_run_loop_remove_timer(&s_past_timer);
        if (s_watching) { s_watching = false; btstack_run_loop_remove_timer(&s_watch_timer); gap_stop_scan(); }
    }
    adv_stop();
    if (s_state == L_SCAN) gap_stop_scan();
    if (!linked) {
        if (s_state == L_CONNECTING) gap_connect_cancel();
        if (s_con != HCI_CON_HANDLE_INVALID) {
            if (s_central) gatt_client_stop_listening_for_characteristic_value_updates(&s_listener);
            gap_disconnect(s_con);
            s_con = HCI_CON_HANDLE_INVALID;
        }
        s_fails++;
        set_state(L_IDLE);
        jbl_assist_set_paused(false);
    } else {
        s_links++;
        set_state(L_LINKED);
    }
    ESP_LOGW(TAG, "stereo link %s: %s", linked ? "UP" : "not made", why);
    if (!linked && s_forever && s_cb != NULL) {  /* a stored pair: a failed attempt just starts the next */
        btstack_run_loop_remove_timer(&s_retry_timer);
        btstack_run_loop_set_timer_handler(&s_retry_timer, retry_cb);
        btstack_run_loop_set_timer(&s_retry_timer, 100);
        btstack_run_loop_add_timer(&s_retry_timer);
        ESP_LOGW(TAG, "stored pair: searching again");
        return;
    }
    if (s_cb) s_cb(linked);
}

static void on_timeout(btstack_timer_source_t *ts)
{
    (void)ts;
    if (s_state != L_LINKED && s_state != L_IDLE) finish(false, "timeout");
}

/* ---- frames to the partner ------------------------------------------------------------ */

static uint8_t tx_now(const uint8_t *d, uint16_t n)
{
    if (s_central) {
        return s_wr_found ? gatt_client_write_value_of_characteristic_without_response(s_con, s_wr.value_handle, n,
                                                                                         (uint8_t *)(uintptr_t)d)
                          : ERROR_CODE_COMMAND_DISALLOWED;
    }
    return jbl_gatt_notify(s_con, jbl_gatt_cnl_notify_handle(), d, n) ? ERROR_CODE_SUCCESS : BTSTACK_ACL_BUFFERS_FULL;
}

static void txq_kick(void);

static void txq_can_send(void *ctx)
{
    (void)ctx;
    s_wr_wait = false;
    txq_kick();
}

/* send queued frames in order; when the link is busy, wait for its can-send callback */
static void txq_kick(void)
{
    while (s_txq_len > 0 && s_con != HCI_CON_HANDLE_INVALID && !s_wr_wait) {
        uint8_t st = tx_now(s_txq[s_txq_head].d, s_txq[s_txq_head].n);
        if (st == ERROR_CODE_SUCCESS) {
            s_txq_head = (s_txq_head + 1) % TXQ_N;
            s_txq_len--;
            continue;
        }
        if (s_central) {
            s_wr_reg.callback = &txq_can_send;
            s_wr_reg.context = NULL;
            if (gatt_client_request_to_write_without_response(&s_wr_reg, s_con) == ERROR_CODE_SUCCESS) {
                s_wr_wait = true;
                return;
            }
        }
        ESP_LOGW(TAG, "queued frame dropped (0x%02x)", st);
        s_txq_head = (s_txq_head + 1) % TXQ_N;
        s_txq_len--;
    }
}

static void send(const uint8_t *d, uint16_t n, const char *what)
{
    if (s_con == HCI_CON_HANDLE_INVALID || n > TXQ_MAX) return;
    uint8_t st = (s_txq_len == 0 && !s_wr_wait) ? tx_now(d, n) : BTSTACK_ACL_BUFFERS_FULL;
    bool queued = false;
    if (st != ERROR_CODE_SUCCESS && s_txq_len < TXQ_N) {
        int k = (s_txq_head + s_txq_len) % TXQ_N;
        memcpy(s_txq[k].d, d, n);
        s_txq[k].n = (uint8_t)n;
        s_txq_len++;
        s_queued++;
        queued = true;
        txq_kick();
    }
    s_tx_frames++;
    char w[64];
    snprintf(w, sizeof(w), "-> %s%s", what, st == ERROR_CODE_SUCCESS ? "" : queued ? " (queued)" : " (NOT SENT)");
    hexlog(w, d, n);
}

static void send_version(void)
{
    static const uint8_t v[] = { 0xEE, 0xEE, 0x01, 0x06, 0, 0, 0, 0, 0, 0 };
    send(v, sizeof(v), "letws version");
}

static void send_info(void)
{
    uint8_t f[13] = { 0xEE, 0xEE, 0x02, 0x09 };
    f[4] = a2dp_in_connected() ? 0x01 : 0x00;  /* b0 bit0: a phone is connected */
    f[11] = s_central ? 1 : 0;                 /* b7: 1 = I am the primary (a real X5 sends 1 / 0) */
    f[12] = a2dp_in_streaming() ? 1 : 0;       /* b8: stream */
    send(f, sizeof(f), "letws info");
}

static void send_ack(uint8_t cmd)
{
    const uint8_t a[] = { 0xAA, 0x00, 0x02, cmd, 0x00 };
    send(a, sizeof(a), "ack");
}

/* AA 13 <len> 00 35 <partner channel> 37 <addr6> C2 <n> <name> 3B <gid4> */
static void send_group(void)
{
    uint8_t f[64];
    int i = 0;
    f[i++] = 0xAA; f[i++] = 0x13; i++;          /* length below */
    f[i++] = 0x00;
    f[i++] = 0x35; f[i++] = (uint8_t)(s_group.channel == 2 ? 1 : 2);   /* the partner's channel: the other one */
    bd_addr_t own;
    gap_local_bd_addr(own);
    f[i++] = 0x37; memcpy(f + i, own, 6); i += 6;
    size_t nl = strnlen(s_group.name, sizeof(s_group.name));
    if (nl > 30) nl = 30;
    f[i++] = 0xC2; f[i++] = (uint8_t)(nl + 1); memcpy(f + i, s_group.name, nl); i += (int)nl;
    f[i++] = 0x00;                              /* the Xtreme 5 counts and sends the NUL */
    f[i++] = 0x3B; memcpy(f + i, s_group.gid, 4); i += 4;
    f[2] = (uint8_t)(i - 3);
    send(f, (uint16_t)i, "group info");
}

/* AA 12 <len> 00 33 <batt> 40 <serial, exactly 16 ASCII> 41 <ver 4 digits> C1 <n> <name> 3B <gid4>
 * (the X5 copies a fixed 16 serial bytes; batt = charging<<7 | percent) */
static void send_dev_info(void)
{
    uint8_t f[96];
    int i = 0;
    f[i++] = 0xAA; f[i++] = 0x12; i++;
    f[i++] = 0x00;
    f[i++] = 0x33; f[i++] = 80;                 /* as our beacon: 80 % */
    f[i++] = 0x40;
    const char *sn = s31_ident_serial();
    memset(f + i, '0', 16);
    size_t sl = strnlen(sn, 16);
    memcpy(f + i + (16 - sl), sn, sl); i += 16;   /* right-aligned, '0'-padded to 16 */
    f[i++] = 0x41; f[i++] = 26; f[i++] = 29; f[i++] = 12; f[i++] = 0;   /* 26.29.12 */
    const char *nm = jbl_device_name();
    size_t nl = strlen(nm);
    if (nl > 32) nl = 32;
    f[i++] = 0xC1; f[i++] = (uint8_t)nl; memcpy(f + i, nm, nl); i += (int)nl;
    f[i++] = 0x3B; memcpy(f + i, s_group.gid, 4); i += 4;
    f[2] = (uint8_t)(i - 3);
    send(f, (uint16_t)i, "device info");
}

static void past_try(btstack_timer_source_t *ts)
{
    (void)ts;
    if (!s_past_pending || s_con == HCI_CON_HANDLE_INVALID) { s_past_pending = false; return; }
    uint8_t adv;
    if (auracast_pa_handle(&adv)) {
        uint8_t st = hci_send_cmd(&hci_le_pa_set_info_transfer, s_con, 0, adv);
        ESP_LOGW(TAG, "PAST: our broadcast (adv set %u) -> the secondary (con 0x%04x): 0x%02x", adv, s_con, st);
        if (st == ERROR_CODE_SUCCESS) { s_past_sent++; s_past_pending = false; return; }
    } else if (s_past_tries == 0) {
        ESP_LOGW(TAG, "PAST: waiting for our broadcast to be on");
    }
    if (++s_past_tries > 40) {                  /* 20 s */
        ESP_LOGW(TAG, "PAST: gave up (broadcast not on)");
        s_past_pending = false;
        return;
    }
    btstack_run_loop_set_timer_handler(&s_past_timer, past_try);
    btstack_run_loop_set_timer(&s_past_timer, 500);
    btstack_run_loop_add_timer(&s_past_timer);
}

static void watch_end(btstack_timer_source_t *ts)
{
    (void)ts;
    if (!s_watching) return;
    s_watching = false;
    s_watch_all = false;
    gap_stop_scan();
    jbl_assist_set_paused(false);
    ESP_LOGW(TAG, "beacon watch over");
}

void lls_beacon_dump(uint16_t seconds)
{
    jbl_assist_set_paused(true);
    gap_set_scan_params(0 /* passive */, 0x0060, 0x0030, 0);
    gap_start_scan();
    s_watching = true;
    s_watch_all = true;
    btstack_run_loop_remove_timer(&s_watch_timer);
    btstack_run_loop_set_timer_handler(&s_watch_timer, watch_end);
    btstack_run_loop_set_timer(&s_watch_timer, (uint32_t)seconds * 1000u);
    btstack_run_loop_add_timer(&s_watch_timer);
    ESP_LOGW(TAG, "logging every JBL beacon for %u s", seconds);
}

static void sync_watch(btstack_timer_source_t *ts)
{
    (void)ts;
    if (s_state != L_LINKED || s_central) return;
    if (party_listen_streaming()) {
        if (!s_synced) {
            s_synced = true;
            static const uint8_t f5[] = { 0xAA, 0xF5, 0x00 };
            send(f5, sizeof(f5), "synced to the primary's broadcast");
        }
    } else {
        s_synced = false;
        if (++s_sync_tries % 10 == 0) {          /* every 5 s without audio: ask again */
            hci_send_cmd(&hci_le_set_past_params, s_con, 2, 0, 400, 0);
            static const uint8_t f1[] = { 0xAA, 0xF1, 0x00 };
            send(f1, sizeof(f1), "broadcast request (again)");
        }
    }
    btstack_run_loop_set_timer_handler(&s_sync_timer, sync_watch);
    btstack_run_loop_set_timer(&s_sync_timer, 500);
    btstack_run_loop_add_timer(&s_sync_timer);
}

static void f4_send(btstack_timer_source_t *ts)
{
    (void)ts;
    if (s_state != L_LINKED) return;
    static const uint8_t f4[] = { 0xAA, 0xF4, 0x01, 0x04, 0x10 };   /* 4100 mV, as the X5 frames it */
    if (s_con != HCI_CON_HANDLE_INVALID) tx_now(f4, sizeof(f4));     /* quiet: every 5 s */
    btstack_run_loop_set_timer_handler(&s_f4_timer, f4_send);
    btstack_run_loop_set_timer(&s_f4_timer, 5000);
    btstack_run_loop_add_timer(&s_f4_timer);
}

static void on_linked(void)
{
    finish(true, "letws handshake done");
    s_ungrouping = false;
    btstack_run_loop_remove_timer(&s_f4_timer);
    btstack_run_loop_set_timer_handler(&s_f4_timer, f4_send);
    btstack_run_loop_set_timer(&s_f4_timer, 5000);
    btstack_run_loop_add_timer(&s_f4_timer);
    if (!s_central) send_dev_info();            /* the primary fills its partner cache from it */
    /* watch the partner's beacon for 15 s: what a linked Xtreme 5 advertises */
    uint8_t r[6];
    for (int i = 0; i < 6; i++) r[i] = s_partner[5 - i];
    s_partner_crc[0] = jbl_crc16_arc(s_partner, 6);
    s_partner_crc[1] = jbl_crc16_arc(r, 6);
    (void)watch_end;                            /* beacon watch: console "beacons" only (no scan at link-up) */
    if (!s_central) {                           /* secondary: take the primary's broadcast (PAST) */
        uint8_t st = hci_send_cmd(&hci_le_set_past_params, s_con, 2 /* sync, reports on */, 0, 400, 0);
        ESP_LOGW(TAG, "SECONDARY: PAST parameters (skip 0, timeout 4 s): 0x%02x", st);
        party_set_stereo_secondary(true);
        a2dp_in_set_stereo_secondary(true);        /* like a JBL secondary: no phone, all airtime to the BIG */
        static const uint8_t f1[] = { 0xAA, 0xF1, 0x00 };
        send(f1, sizeof(f1), "broadcast request");
        s_sync_tries = st == ERROR_CODE_SUCCESS ? 0 : 9;   /* HCI busy: ask again on the next tick */
        s_synced = false;
        btstack_run_loop_remove_timer(&s_sync_timer);
        btstack_run_loop_set_timer_handler(&s_sync_timer, sync_watch);
        btstack_run_loop_set_timer(&s_sync_timer, 500);
        btstack_run_loop_add_timer(&s_sync_timer);
    }
    if (s_central) {                            /* primary: tell the secondary the group, ask its info */
        send_group();
        static const uint8_t req[] = { 0xAA, 0x11, 0x00 };
        send(req, sizeof(req), "device info request");
        lls_send_tone_setting();
        lls_light_changed();                    /* the secondary takes our theme at once */
        /* as the Xtreme 5 primary once its broadcast source is up (fw 0x10FE834): 75 ms, latency 0,
         * 4 s - the link only carries small control frames, the airtime goes to the BIG */
        uint8_t st = gap_update_connection_parameters(s_con, 60, 60, 0, 400);
        ESP_LOGW(TAG, "PRIMARY: stereo link to 75 ms: 0x%02x", st);
    }
}

/* ---- keys and prompts across the pair (re/fw_jbl/REPORT_X5_STEREO_BEHAVIOR.md) ------------- */
static volatile bool s_tone_from_partner;

static void key_from_partner(uint8_t k)
{
    ESP_LOGW(TAG, "partner key %u%s", k, s_central ? "" : " (we are not the primary: ignored)");
    if (!s_central) return;
    switch (k) {
    case 1: a2dp_in_button(A2DP_BTN_PLAY_PAUSE); break;
    case 3: a2dp_in_button(A2DP_BTN_VOL_UP); break;
    case 4: a2dp_in_button(A2DP_BTN_VOL_DOWN); break;
    case 8: a2dp_in_button(A2DP_BTN_PAIRING); break;
    case 10: a2dp_in_media_op(4); break;        /* next */
    case 11: a2dp_in_media_op(3); break;        /* previous */
    case 13: jbl_v5_stereo_ungroup_local(); break;
    default: break;                             /* 2 pause: log only; 5 party / 12 power / 14 macro: not here */
    }
}

/* one slot each: a second key / prompt before the BT thread ran replaces the first (keys are
 * human-paced; a lost repeat is harmless) */
static volatile uint8_t s_key_out, s_f2_out;
static btstack_context_callback_registration_t s_out_reg;

static void out_on_bt(void *ctx)
{
    (void)ctx;
    if (s_state != L_LINKED) { s_key_out = s_f2_out = 0; return; }
    if (s_key_out) {
        uint8_t f[] = { 0xAA, 0x31, 0x01, s_key_out };
        s_key_out = 0;
        send(f, sizeof(f), "key to the primary");
    }
    if (s_f2_out) {
        uint8_t f[] = { 0xAA, 0xF2, 0x01, s_f2_out };
        s_f2_out = 0;
        send(f, sizeof(f), "prompt to the secondary");
    }
}

static void out_post(void)
{
    s_out_reg.callback = out_on_bt;
    btstack_run_loop_execute_on_main_thread(&s_out_reg);
}

/* ---- primary -> secondary: EQ, lights, BT LED, volume ring --------------------------------- */
static uint8_t s_bt_led = 3;                    /* AA 41 value: 2 pairing, 3 connected/idle */

void lls_send_tone_setting(void)
{
    if (s_state != L_LINKED || !s_central) return;
    uint8_t f[] = { 0xAA, 0x60, 0x01, (uint8_t)(jbl_v5_feedback_tone_on() ? 1 : 0) };
    send(f, sizeof(f), "prompt-tone setting");
}

static btstack_timer_source_t s_eq_retry;
static void eq_retry_cb(btstack_timer_source_t *ts);

/* the 118 B EQ frame fits this link? (central: our client MTU; peripheral: the server's) */
static bool eq_fits(void)
{
    uint16_t mtu = s_central ? 0 : att_server_get_mtu(s_con);
    if (s_central && gatt_client_get_mtu(s_con, &mtu) != ERROR_CODE_SUCCESS) mtu = 0;
    if (mtu >= 3 + 118) return true;
    ESP_LOGW(TAG, "EQ frame: ATT MTU %u too small for its 118 B", mtu);
    return false;
}

static volatile bool s_eq_dirty;
static void light_on_bt(void *ctx);
static btstack_context_callback_registration_t s_light_reg;

/* coalesced: a slider drag (a Set every ~100 ms) sends only the latest EQ */
void lls_send_eq(void)
{
    if (s_state != L_LINKED || !s_central) return;
    s_eq_dirty = true;
    s_light_reg.callback = light_on_bt;
    btstack_run_loop_execute_on_main_thread(&s_light_reg);
}

void lls_send_bt_led(uint8_t state)
{
    s_bt_led = state;
    if (s_state != L_LINKED || !s_central) return;
    uint8_t f[] = { 0xAA, 0x41, 0x01, state };
    send(f, sizeof(f), "BT LED state");
}

static volatile bool s_light_dirty;
static volatile int8_t s_vol_ring = -1;

static void light_on_bt(void *ctx)
{
    (void)ctx;
    if (s_state != L_LINKED || !s_central) { s_light_dirty = false; s_eq_dirty = false; s_vol_ring = -1; return; }
    if (s_eq_dirty && s_txq_len == 0) {         /* the link drains first: then the newest EQ */
        s_eq_dirty = false;
        if (eq_fits()) {
            uint8_t f[3 + 115] = { 0xAA, 0x93, 115 };
            jbl_v5_eq_tws_build(f + 3);
            send(f, sizeof(f), "EQ (AA 93)");
        }
    } else if (s_eq_dirty) {
        btstack_run_loop_remove_timer(&s_eq_retry);
        btstack_run_loop_set_timer_handler(&s_eq_retry, eq_retry_cb);
        btstack_run_loop_set_timer(&s_eq_retry, 100);
        btstack_run_loop_add_timer(&s_eq_retry);
    }
    if (s_light_dirty) {
        s_light_dirty = false;
        uint8_t f[18] = { 0xAA, 0x43, 0x0F };
        jbl_v5_light_record(f + 3);
        send(f, sizeof(f), "light state (AA 43)");
    }
    if (s_vol_ring >= 0) {
        uint8_t f[] = { 0xAA, 0x45, 0x01, (uint8_t)s_vol_ring };
        s_vol_ring = -1;
        send(f, sizeof(f), "volume ring (AA 45)");
    }
}

static void eq_retry_cb(btstack_timer_source_t *ts)
{
    (void)ts;
    light_on_bt(NULL);
}

void lls_light_changed(void)
{
    s_light_dirty = true;
    s_light_reg.callback = light_on_bt;
    btstack_run_loop_execute_on_main_thread(&s_light_reg);
}

void lls_volume_ring(uint8_t vol_0_127)
{
    if (s_state != L_LINKED || !s_central) return;
    s_vol_ring = (int8_t)((vol_0_127 * 32 + 63) / 127);
    s_light_reg.callback = light_on_bt;
    btstack_run_loop_execute_on_main_thread(&s_light_reg);
}

bool lls_forward_key(uint8_t k)
{
    if (s_state != L_LINKED || s_central) return false;
    s_key_out = k;
    out_post();
    return true;
}

void lls_on_local_tone(const char *name)
{
    if (s_state != L_LINKED || !s_central || s_tone_from_partner || name == NULL) return;
    uint8_t ev = !strcmp(name, "bt_pair") ? 0x50 : !strcmp(name, "vol_max") ? 0x52 : !strcmp(name, "bt_cnt") ? 0x51 : 0;
    if (!ev) return;
    s_f2_out = ev;
    out_post();
}

static void handle_frame(const uint8_t *d, uint16_t n)
{
    s_rx_frames++;
    hexlog("<- partner", d, n);
    if (n >= 4 && d[0] == 0xEE && d[1] == 0xEE) {
        if (d[2] == 0x01) {
            ESP_LOGW(TAG, "partner letws version");
            if (s_central) send_info();         /* the central answers a 01 with a 02 */
        } else if (d[2] == 0x02) {
            ESP_LOGW(TAG, "partner letws info: phone %u, a_s %u, role %u, stream %u", n > 4 ? d[4] & 1 : 0, n > 4 ? (d[4] >> 1) & 1 : 0,
                     n > 11 ? d[11] : 0, n > 12 ? d[12] : 0);
            if (s_state != L_LINKED) on_linked();
        }
        return;
    }
    if (n >= 3 && d[0] == 0xAA) {
        uint8_t cmd = d[1];
        switch (cmd) {
        case 0x00:                              /* ack */
            if (n >= 4 && d[3] == 0x13 && s_ungrouping) {
                ESP_LOGW(TAG, "partner acked the ungroup - closing the stereo link");
                btstack_run_loop_remove_timer(&s_ug_timer);
                s_ungrouping = false;
                lls_stop();
            }
            break;
        case 0x11: send_dev_info(); break;
        case 0x12: {                            /* 00 33 <batt> [40 <16 serial> 41 <4 ver> C1 <n> <name> 3B <gid4>] */
            uint8_t pgid[4] = { 0 };
            bool have_pgid = false, full = false;
            for (int i = 4; i < n;) {
                uint8_t t = d[i++];
                if (t == 0x33 && i < n) { s_pinfo.batt = d[i++]; s_pinfo.valid = true; }
                else if (t == 0x40 && i + 16 <= n) { memcpy(s_pinfo.serial, d + i, 16); s_pinfo.serial[16] = 0; i += 16; full = true; }
                else if (t == 0x41 && i + 4 <= n) { memcpy(s_pinfo.ver, d + i, 4); i += 4; }
                else if (t == 0xC1 && i < n) { i += 1 + d[i]; }
                else if (t == 0x3B && i + 4 <= n) { memcpy(pgid, d + i, 4); have_pgid = true; i += 4; }
                else break;
            }
            ESP_LOGW(TAG, "partner info: battery %u %%%s, serial %s, fw %u.%u.%u", s_pinfo.batt & 0x7F,
                     (s_pinfo.batt & 0x80) ? " (charging)" : "", s_pinfo.serial, s_pinfo.ver[0], s_pinfo.ver[1], s_pinfo.ver[2]);
            if (s_central && full) {            /* the Xtreme 5 primary: EQ + BT LED state after a full AA 12 */
                lls_send_eq();
                lls_send_bt_led(s_bt_led);
                if (have_pgid && memcmp(pgid, s_group.gid, 4) != 0 &&
                    (s_group.gid[0] | s_group.gid[1] | s_group.gid[2] | s_group.gid[3])) {
                    ESP_LOGW(TAG, "partner group id differs: channel + group id again");
                    lls_send_channel(s_group.channel);
                }
            }
            break;
        }
        case 0x13: {                            /* 00 35 <ch> 37 <addr6> C2 <n> <name> 3B <gid4>, any order */
            uint8_t ch = 0;
            bool gid_zero = false, have_gid = false;
            for (int i = 4; i < n;) {
                uint8_t t = d[i++];
                if (t == 0x35 && i < n) ch = d[i++];
                else if (t == 0x37 && i + 6 <= n) i += 6;
                else if (t == 0xC2 && i < n) i += 1 + d[i];
                else if (t == 0x3B && i + 4 <= n) { have_gid = true; gid_zero = !(d[i] | d[i + 1] | d[i + 2] | d[i + 3]); i += 4; }
                else break;
            }
            send_ack(cmd);
            if (have_gid && gid_zero) {
                ESP_LOGW(TAG, "partner ungrouped the pair");
                jbl_v5_stereo_partner_ungrouped();
            } else if (ch == 1 || ch == 2) {
                ESP_LOGW(TAG, "partner sets our channel: %s", ch == 1 ? "LEFT" : "RIGHT");
                jbl_v5_stereo_partner_channel(ch);
            }
            break;
        }
        case 0x31:                              /* the secondary's key: run it as ours (primary) */
            send_ack(cmd);
            key_from_partner(n > 3 ? d[3] : 0);
            break;
        case 0x46: break;                       /* no ack (as the Xtreme 5) */
        case 0xF4:                              /* battery voltage, mV LE (no ack) */
            if (n >= 5) s_pinfo.mv = (uint16_t)(d[3] | (d[4] << 8));
            break;
        case 0x43:                              /* the primary's light state (theme, on, brightness, colour) */
            if (!s_central && n >= 18) jbl_v5_light_from_partner(d + 3);
            break;
        case 0x44: case 0x45: case 0x47: case 0x48:   /* BT blink / volume ring / battery ring / team: LED only */
            break;
        case 0x60:                              /* the primary's prompt-tone setting: adopt it */
            if (!s_central && n >= 4) jbl_v5_feedback_tone_from_partner(d[3] != 0);
            break;
        case 0x93:                              /* the primary's EQ */
            send_ack(cmd);
            if (!s_central && n >= 6) jbl_v5_eq_tws_apply(d + 3, (uint16_t)(n - 3));
            break;
        case 0x91: {                            /* EQ query: answer AA 92 (no ack) */
            if (!eq_fits()) break;
            uint8_t f[3 + 115] = { 0xAA, 0x92, 115 };
            jbl_v5_eq_tws_build(f + 3);
            send(f, sizeof(f), "EQ (AA 92)");
            break;
        }
        case 0xF1:
            ESP_LOGW(TAG, "partner asks for our broadcast (PAST)");
            send_ack(cmd);
            if (s_central && !s_past_pending) {
                s_past_pending = true;
                s_past_tries = 0;
                past_try(NULL);
            }
            break;
        case 0xF5: ESP_LOGW(TAG, "partner synced to the broadcast"); break;
        case 0xF2:                              /* system event; 4B = identify (the app's channel select) */
            send_ack(cmd);
            if (n >= 4 && d[3] == 0x4B) {
                ESP_LOGW(TAG, "partner: identify - indicate tone");
                tones_play("indicate");
            } else if (n >= 4 && d[3] == 0x50) {
                ESP_LOGW(TAG, "partner: pairing - pairing tone");
                s_tone_from_partner = true;
                tones_play("bt_pair");
                s_tone_from_partner = false;
            } else if (n >= 4 && d[3] == 0x04) {
                ESP_LOGW(TAG, "partner: wake up");
                power_wake();
            } else if (n >= 4 && d[3] == 0x03) {
                ESP_LOGW(TAG, "partner: standby");
            } else if (n >= 4 && d[3] == 0x52) {
                ESP_LOGW(TAG, "partner: max volume - beep");
                s_tone_from_partner = true;
                tones_play("vol_max");
                s_tone_from_partner = false;
            } else {
                ESP_LOGW(TAG, "partner system event 0x%02x", n >= 4 ? d[3] : 0);
            }
            break;
        default: send_ack(cmd); break;
        }
    }
}

/* ---- primary: GATT client on the secondary ------------------------------------------------ */

static void gatt_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;
    switch (hci_event_packet_get_type(packet)) {
    case GATT_EVENT_SERVICE_QUERY_RESULT:
        gatt_event_service_query_result_get_service(packet, &s_svc);
        s_svc_found = true;
        break;
    case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT: {
        gatt_client_characteristic_t c;
        gatt_event_characteristic_query_result_get_characteristic(packet, &c);
        ESP_LOGW(TAG, "partner cnl char ...%02x%02x: value 0x%04x, props 0x%02x", c.uuid128[14], c.uuid128[15],
                 c.value_handle, c.properties);
        if (c.properties & ATT_PROPERTY_NOTIFY) { s_ntf = c; s_ntf_found = true; }
        if (c.properties & (ATT_PROPERTY_WRITE | ATT_PROPERTY_WRITE_WITHOUT_RESPONSE)) { s_wr = c; s_wr_found = true; }
        break;
    }
    case GATT_EVENT_NOTIFICATION:
        handle_frame(gatt_event_notification_get_value(packet), gatt_event_notification_get_value_length(packet));
        break;
    case GATT_EVENT_QUERY_COMPLETE: {
        uint8_t st = gatt_event_query_complete_get_att_status(packet);
        if (st != ATT_ERROR_SUCCESS) {
            ESP_LOGW(TAG, "GATT step '%s' failed: ATT 0x%02x", s_names[s_state], st);
            finish(false, "GATT error");
            break;
        }
        if (s_state == L_DISCOVER) {
            if (!s_svc_found) { finish(false, "partner has no excelpoint.cnl service"); break; }
            set_state(L_CHARS);
            gatt_client_discover_characteristics_for_service(&gatt_handler, s_con, &s_svc);
        } else if (s_state == L_CHARS) {
            if (!s_ntf_found || !s_wr_found) { finish(false, "cnl service without notify/write"); break; }
            set_state(L_SUBSCRIBE);
            gatt_client_listen_for_characteristic_value_updates(&s_listener, &gatt_handler, s_con, &s_ntf);
            gatt_client_write_client_characteristic_configuration(&gatt_handler, s_con, &s_ntf,
                GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
        } else if (s_state == L_SUBSCRIBE) {
            set_state(L_HANDSHAKE);
            send_version();
        }
        break;
    }
    default:
        break;
    }
}

/* ---- HCI: adverts, connections --------------------------------------------------------- */

static void on_directed(const bd_addr_t a, uint8_t atype, uint16_t etype, const bd_addr_t da)
{
    bd_addr_t own;
    gap_local_bd_addr(own);
    s_directed_seen++;
    char as[18];
    snprintf(as, sizeof(as), "%s", bd_addr_to_str(a));
    bool us = bd_addr_cmp(da, own) == 0;
    bool partner = bd_addr_cmp(a, s_partner) == 0;
    ESP_LOGW(TAG, ">>> DIRECTED advert from %s (type %u, evt 0x%04x) to %s%s%s", as, atype, etype,
             bd_addr_to_str(da), us ? " = US" : "", partner ? " - THE PARTNER" : "");
    if (s_state != L_SCAN || !us || !(etype & 1)) return;
    if (!partner) return;                       /* as the Xtreme 5: only the app-given partner */
    gap_stop_scan();
    /* the Xtreme 5's connection: scan 60/30 ms, interval 30 ms, latency 0, timeout 8 s */
    /* full-duty initiator; 15 ms while discovering + handshaking (the Xtreme 5: 30 ms), 75 ms once linked */
    gap_set_connection_parameters(0x0030, 0x0030, 0x000C, 0x000C, 0, 0x0320, 2, 4);
    uint8_t st = gap_connect(a, (bd_addr_type_t)atype);
    ESP_LOGW(TAG, "PRIMARY: connecting to the secondary %s: 0x%02x", as, st);
    if (st != ERROR_CODE_SUCCESS) { finish(false, "connect not started"); return; }
    set_state(L_CONNECTING);
}

static void hci_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;
    switch (hci_event_packet_get_type(packet)) {
    case GAP_EVENT_EXTENDED_ADVERTISING_REPORT:
        if (s_watching) {
            const uint8_t *d = gap_event_extended_advertising_report_get_data(packet);
            uint8_t n = gap_event_extended_advertising_report_get_data_length(packet);
            for (int i = 0; i + 1 < n; i += d[i] + 1) {
                uint8_t l = d[i];
                if (l < 3 || i + l >= n) break;
                if (d[i + 1] != 0x16 || d[i + 2] != 0xDF || d[i + 3] != 0xFD) continue;
                const uint8_t *v = d + i + 4;
                uint8_t vn = (uint8_t)(l - 3);
                bool partner = false;
                for (int k = 0; k + 1 < vn; k++) {
                    uint16_t c = (uint16_t)(v[k] | (v[k + 1] << 8));
                    if (c == s_partner_crc[0] || c == s_partner_crc[1]) partner = true;
                }
                if (s_watch_all) {                  /* console dump: each address, on change */
                    static struct { bd_addr_t a; uint32_t h; } tab[16];
                    static int ntab;
                    bd_addr_t a;
                    gap_event_extended_advertising_report_get_address(packet, a);
                    uint32_t h = 2166136261u;
                    for (int k = 0; k < vn; k++) h = (h ^ v[k]) * 16777619u;
                    int t;
                    for (t = 0; t < ntab; t++) if (bd_addr_cmp(tab[t].a, a) == 0) break;
                    if (t == ntab && ntab < 16) { memcpy(tab[ntab].a, a, 6); tab[ntab].h = 0; ntab++; }
                    if (t < 16 && tab[t].h != h) {
                        tab[t].h = h;
                        char w[64];
                        snprintf(w, sizeof(w), "BEACON %s evt 0x%04x rssi %d", bd_addr_to_str(a),
                                 gap_event_extended_advertising_report_get_advertising_event_type(packet),
                                 gap_event_extended_advertising_report_get_rssi(packet));
                        hexlog(w, v, vn);
                    }
                    continue;
                }
                static uint8_t last[31];
                static uint8_t last_n;
                if (partner && vn <= sizeof(last) && (vn != last_n || memcmp(last, v, vn) != 0)) {
                    memcpy(last, v, vn);
                    last_n = vn;
                    hexlog("partner beacon (linked)", v, vn);
                }
            }
        }
        if (s_state == L_SCAN) {
            uint16_t et = gap_event_extended_advertising_report_get_advertising_event_type(packet);
            if (et & 0x04) {
                bd_addr_t a, da;
                gap_event_extended_advertising_report_get_address(packet, a);
                gap_event_extended_advertising_report_get_direct_address(packet, da);
                on_directed(a, gap_event_extended_advertising_report_get_address_type(packet), et, da);
            }
        }
        break;
    case GAP_EVENT_ADVERTISING_REPORT:
        if (s_state == L_SCAN && gap_event_advertising_report_get_advertising_event_type(packet) == 1) {
            bd_addr_t a, da;
            gap_event_advertising_report_get_address(packet, a);
            gap_local_bd_addr(da);              /* ADV_DIRECT_IND: the controller only reports those for us */
            on_directed(a, gap_event_advertising_report_get_address_type(packet), 0x15, da);
        }
        break;
    case HCI_EVENT_COMMAND_COMPLETE: {
        uint16_t op = hci_event_command_complete_get_command_opcode(packet);
        if (s_state == L_ADV && op >= 0x2035 && op <= 0x2039) {
            uint8_t st = hci_event_command_complete_get_return_parameters(packet)[0];
            if (st) ESP_LOGE(TAG, "advert command 0x%04x -> status 0x%02x", op, st);
        }
        break;
    }
    case HCI_EVENT_LE_META:
        if (hci_event_le_meta_get_subevent_code(packet) == HCI_SUBEVENT_LE_ADVERTISING_SET_TERMINATED &&
            hci_subevent_le_advertising_set_terminated_get_advertising_handle(packet) == s_adv_handle) {
            ESP_LOGW(TAG, "directed advert ended: status 0x%02x, con 0x%04x",
                     hci_subevent_le_advertising_set_terminated_get_status(packet),
                     hci_subevent_le_advertising_set_terminated_get_connection_handle(packet));
            /* the connection complete event may not name our advertising set: this one does */
            if (s_state == L_ADV && hci_subevent_le_advertising_set_terminated_get_status(packet) == ERROR_CODE_SUCCESS) {
                s_con = hci_subevent_le_advertising_set_terminated_get_connection_handle(packet);
                s_central = false;
                s_adv_on = false;               /* a directed set ends with the connection */
                ESP_LOGW(TAG, ">>> the primary connected to our directed advert (con 0x%04x)", s_con);
                set_state(L_WAIT_CCC);
            }
        }
        break;
    case HCI_EVENT_META_GAP:
        if (hci_event_gap_meta_get_subevent_code(packet) == GAP_SUBEVENT_ADVERTISING_SET_INSTALLED &&
            gap_subevent_advertising_set_installed_get_advertisement_handle(packet) == s_adv_handle) {
            uint8_t st = gap_subevent_advertising_set_installed_get_status(packet);
            if (st) ESP_LOGE(TAG, "directed advert rejected by the controller: 0x%02x", st);
            else ESP_LOGW(TAG, "directed advert on air (handle %u)", s_adv_handle);
            break;
        }
        if (hci_event_gap_meta_get_subevent_code(packet) != GAP_SUBEVENT_LE_CONNECTION_COMPLETE) break;
        {
            bd_addr_t a;
            gap_subevent_le_connection_complete_get_peer_address(packet, a);
            uint8_t st = gap_subevent_le_connection_complete_get_status(packet);
            hci_con_handle_t con = gap_subevent_le_connection_complete_get_connection_handle(packet);
            uint8_t role = gap_subevent_le_connection_complete_get_role(packet);
            if (s_state == L_CONNECTING && role == HCI_ROLE_MASTER && bd_addr_cmp(a, s_partner) == 0) {
                if (st != ERROR_CODE_SUCCESS) { finish(false, "connect failed"); break; }
                s_con = con;
                s_central = true;
                ESP_LOGW(TAG, ">>> connected to the secondary %s (con 0x%04x)", bd_addr_to_str(a), con);
                set_state(L_DISCOVER);
                s_svc_found = s_ntf_found = s_wr_found = false;
                gatt_client_discover_primary_services_by_uuid128(&gatt_handler, s_con, CNL_SERVICE);
            } else if (s_state == L_ADV && st == ERROR_CODE_SUCCESS && role == HCI_ROLE_SLAVE &&
                       (bd_addr_cmp(a, s_partner) == 0 ||
                        gap_subevent_le_connection_complete_get_advertising_handle(packet) == s_adv_handle)) {
                s_con = con;
                s_central = false;
                s_adv_on = false;               /* a directed set ends with the connection */
                ESP_LOGW(TAG, ">>> the primary %s connected to our directed advert (con 0x%04x)", bd_addr_to_str(a), con);
                set_state(L_WAIT_CCC);
            }
        }
        break;
    case HCI_EVENT_DISCONNECTION_COMPLETE:
        if (hci_event_disconnection_complete_get_connection_handle(packet) == s_con) {
            uint8_t r = hci_event_disconnection_complete_get_reason(packet);
            s_con = HCI_CON_HANDLE_INVALID;
            if (s_state == L_LINKED) {
                ESP_LOGW(TAG, "stereo link closed (reason 0x%02x)", r);
                btstack_run_loop_remove_timer(&s_sync_timer);
                party_set_stereo_secondary(false);
                a2dp_in_set_stereo_secondary(false);
                s_txq_len = 0;
                s_wr_wait = false;
                s_past_pending = false;
                btstack_run_loop_remove_timer(&s_past_timer);
                if (s_watching) { s_watching = false; btstack_run_loop_remove_timer(&s_watch_timer); gap_stop_scan(); }
                set_state(L_IDLE);
                jbl_assist_set_paused(false);
                if (s_cb) s_cb(false);
            } else if (s_state != L_IDLE) {
                ESP_LOGW(TAG, "partner left during setup (reason 0x%02x)", r);
                finish(false, "disconnected");
            }
        }
        break;
    default:
        break;
    }
}

/* ---- secondary: the primary uses our GATT server -------------------------------------- */

static void hs_send(btstack_timer_source_t *ts)
{
    (void)ts;
    if (s_state != L_HANDSHAKE || s_central || s_con == HCI_CON_HANDLE_INVALID) return;
    send_version();
    send_info();
    if (++s_hs_tries < 15) {
        btstack_run_loop_set_timer_handler(&s_hs_timer, hs_send);
        btstack_run_loop_set_timer(&s_hs_timer, 1000);
        btstack_run_loop_add_timer(&s_hs_timer);
    }
}

void lls_on_cnl_cccd(hci_con_handle_t con, uint16_t value)
{
    ESP_LOGW(TAG, "con 0x%04x wrote cnl.0001 CCCD = 0x%04x%s", con, value, con == s_con ? " (the primary)" : "");
    hci_connection_t *hc = hci_connection_for_handle(con);
    bool from_partner = hc != NULL && s_role == 1 && bd_addr_cmp(hc->address, s_partner) == 0;
    if ((value & 1) && from_partner && (s_state == L_IDLE || s_state == L_ADV) && (s_forever || s_state == L_ADV)) {
        if (s_state == L_IDLE) {
            btstack_run_loop_remove_timer(&s_retry_timer);
            ESP_LOGW(TAG, "SECONDARY: the primary came back through another advert (con 0x%04x) - taking it", con);
        }
        s_con = con;
        s_central = false;
        adv_stop();
        set_state(L_WAIT_CCC);
    } else if (s_state == L_ADV && s_role == 1 && (value & 1) && from_partner) {   /* subscription before the connection event */
        s_con = con;
        s_central = false;
        adv_stop();
        set_state(L_WAIT_CCC);
    }
    if (con != s_con || !(value & 1) || s_state != L_WAIT_CCC) return;
    set_state(L_HANDSHAKE);
    /* the peripheral sends 01 and 02 - after the subscription write has been answered (sent
     * from inside the write they were lost), and again every second until the primary answers */
    s_hs_tries = 0;
    btstack_run_loop_remove_timer(&s_hs_timer);
    btstack_run_loop_set_timer_handler(&s_hs_timer, hs_send);
    btstack_run_loop_set_timer(&s_hs_timer, 10);   /* just after the write response (was 300 ms) */
    btstack_run_loop_add_timer(&s_hs_timer);
}

void lls_on_cnl_write(hci_con_handle_t con, const uint8_t *data, uint16_t len)
{
    if (con == s_con && s_con != HCI_CON_HANDLE_INVALID) handle_frame(data, len);
    else hexlog("cnl.0002 write (not the stereo partner)", data, len);
}

/* ---- API --------------------------------------------------------------------------------- */

void lls_setup(void)
{
    s_hci_reg.callback = &hci_handler;
    hci_add_event_handler(&s_hci_reg);
}

static void retry_cb(btstack_timer_source_t *ts)
{
    (void)ts;
    if (s_state == L_IDLE && s_forever && s_cb != NULL) lls_start(s_role, s_partner, NULL, LLS_FOREVER, s_cb);
}

void lls_start(uint8_t role, const uint8_t partner[6], const lls_group_t *group, uint16_t timeout_s, lls_cb_t cb)
{
    if (s_state != L_IDLE) lls_stop();
    btstack_run_loop_remove_timer(&s_retry_timer);
    s_forever = timeout_s == LLS_FOREVER;
    s_role = role;
    s_cb = cb;
    memcpy(s_partner, partner, 6);
    if (group) s_group = *group;
    s_starts++;
    btstack_run_loop_remove_timer(&s_timer);
    /* a stored pair (LLS_FOREVER) is searched for without end, as the Xtreme 5 does - but each
     * attempt has 30 s: one stuck half-way (connected, never subscribed / handshaked) starts over */
    btstack_run_loop_set_timer_handler(&s_timer, on_timeout);
    btstack_run_loop_set_timer(&s_timer, timeout_s == LLS_FOREVER ? 30000u
                                         : (uint32_t)(timeout_s ? timeout_s : LLS_DEFAULT_S) * 1000u);
    btstack_run_loop_add_timer(&s_timer);
    jbl_assist_set_paused(true);                /* the LE scanner / initiator is ours now */
    if (role == 2) {
        gap_set_scan_params(0 /* passive */, 0x0030, 0x0030, 0);
        gap_start_scan();
        set_state(L_SCAN);
        ESP_LOGW(TAG, "PRIMARY: scanning for the directed advert of %s", bd_addr_to_str(s_partner));
    } else {
        memcpy(s_params.peer_address, s_partner, 6);
        uint8_t st;
        if (s_adv_handle == 0xFF) {
            st = gap_extended_advertising_setup(&s_set, &s_params, &s_adv_handle);
        } else {
            st = gap_extended_advertising_set_params(s_adv_handle, &s_params);   /* new partner address */
        }
        if (st != ERROR_CODE_SUCCESS) {
            ESP_LOGE(TAG, "no advertising set (0x%02x)", st);
            finish(false, "no adv set");
            return;
        }
        gap_extended_advertising_start(s_adv_handle, 0, 0);
        s_adv_on = true;
        set_state(L_ADV);
        hci_connection_t *hc = hci_connection_for_bd_addr_and_type(s_partner, BD_ADDR_TYPE_LE_PUBLIC);
        if (hc != NULL) {                       /* already linked to us: use that connection */
            s_con = hc->con_handle;
            s_central = false;
            adv_stop();
            ESP_LOGW(TAG, "SECONDARY: the primary is already connected (con 0x%04x) - handshake on it", s_con);
            set_state(L_HANDSHAKE);
            s_hs_tries = 0;
            btstack_run_loop_remove_timer(&s_hs_timer);
            btstack_run_loop_set_timer_handler(&s_hs_timer, hs_send);
            btstack_run_loop_set_timer(&s_hs_timer, 300);
            btstack_run_loop_add_timer(&s_hs_timer);
        }
        ESP_LOGW(TAG, "SECONDARY: directed advert (handle %u) from our public address to %s (20 ms)", s_adv_handle,
                 bd_addr_to_str(s_partner));
    }
}

void lls_stop(void)
{
    btstack_run_loop_remove_timer(&s_retry_timer);
    s_forever = false;
    if (s_state == L_IDLE) return;
    s_cb = NULL;
    finish(false, "stopped");
}

bool lls_partner_get(uint16_t feat, uint8_t *v, uint16_t *n)
{
    if (s_state != L_LINKED || !s_pinfo.valid) return false;
    switch (feat) {
    case 0x0008: case 0x0009: *n = (uint16_t)strnlen(s_pinfo.serial, 16); memcpy(v, s_pinfo.serial, *n); return *n > 0;
    case 0x000C: memcpy(v, s_pinfo.ver, 3); *n = 3; return true;
    case 0x000D: v[0] = s_pinfo.batt; *n = 1; return true;
    default: return false;
    }
}

static void id_send(btstack_timer_source_t *ts)
{
    (void)ts;
    static const uint8_t f[] = { 0xAA, 0xF2, 0x01, 0x4B };
    send(f, sizeof(f), "identify the partner");
}

void lls_partner_identify(void)
{
    if (s_state != L_LINKED) return;
    btstack_run_loop_remove_timer(&s_id_timer);
    btstack_run_loop_set_timer_handler(&s_id_timer, id_send);
    btstack_run_loop_set_timer(&s_id_timer, 950);
    btstack_run_loop_add_timer(&s_id_timer);
}

void lls_send_channel(uint8_t own_channel)
{
    if (s_state != L_LINKED || !s_central) return;
    uint8_t f[] = { 0xAA, 0x13, 0x08, 0x00, 0x3B, s_group.gid[0], s_group.gid[1], s_group.gid[2], s_group.gid[3],
                    0x35, (uint8_t)(own_channel == 1 ? 2 : 1) };
    send(f, sizeof(f), "partner channel");
}

static void ug_timeout(btstack_timer_source_t *ts)
{
    (void)ts;
    if (!s_ungrouping) return;
    s_ungrouping = false;
    ESP_LOGW(TAG, "no ungroup ack from the partner in 2 s - closing anyway");
    lls_stop();
}

void lls_ungroup(void)
{
    if (s_state != L_LINKED) { lls_stop(); return; }
    static const uint8_t f[] = { 0xAA, 0x13, 0x12, 0x00, 0x35, 0x00, 0x37, 0, 0, 0, 0, 0, 0, 0xC2, 0x01, 0x00,
                                 0x3B, 0x00, 0x00, 0x00, 0x00 };
    send(f, sizeof(f), "ungroup");
    s_ungrouping = true;
    s_cb = NULL;                                /* the app already knows */
    btstack_run_loop_remove_timer(&s_ug_timer);
    btstack_run_loop_set_timer_handler(&s_ug_timer, ug_timeout);
    btstack_run_loop_set_timer(&s_ug_timer, 2000);
    btstack_run_loop_add_timer(&s_ug_timer);
}

bool lls_linked(void) { return s_state == L_LINKED; }
bool lls_is_partner_addr(const bd_addr_t a) { return s_role != 0 && bd_addr_cmp(a, s_partner) == 0; }
bool lls_is_link(hci_con_handle_t con) { return con != HCI_CON_HANDLE_INVALID && con == s_con; }

void lls_log_stats(void)
{
    ESP_LOGI(TAG, "LLS %s role %u | starts %lu, links %lu, fails %lu | frames rx %lu tx %lu (queued %lu) | "
             "directed adverts %lu | PAST sent %lu", s_names[s_state], s_role, (unsigned long)s_starts,
             (unsigned long)s_links, (unsigned long)s_fails, (unsigned long)s_rx_frames, (unsigned long)s_tx_frames,
             (unsigned long)s_queued, (unsigned long)s_directed_seen, (unsigned long)s_past_sent);
}
