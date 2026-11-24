/*
 * csb.c - BR/EDR Connectionless Peripheral Broadcast (CPB, "CSB") on the S31.
 *
 * The S31 controller has CPB TX + RX and the synchronization train (libbredr_app.a:
 * olc_cpb_rx, olc_sync_train, HCI Set_CPB / Set_CPB_Receive / Receive_Sync_Train ...),
 * switched on with CONFIG_BT_CTRL_BR_EDR_CPB_TX_EN / _RX_EN. BTstack knows none of it:
 * the commands are defined here, the events (page 2 of the event mask) are parsed here.
 *
 * Receive (a JBL speaker broadcasting, e.g. a stereo primary or a PartyBoost master):
 *   Receive_Synchronization_Train(its BD_ADDR) -> Synchronization_Train_Received (its clock
 *   offset, AFH map, LT_ADDR, next instant, interval) -> Set_CPB_Receive -> CPB_Receive
 *   events carry every broadcast packet.
 * Transmit:
 *   Set_Reserved_LT_ADDR -> Write_Synchronization_Train_Parameters -> Set_CPB ->
 *   Set_CPB_Data -> Start_Synchronization_Train.
 */
#include "csb.h"

#include <stdio.h>
#include <string.h>

#include "btstack.h"
#include "esp_log.h"

static const char *TAG = "CSB";

#define OGF_LC 0x01
#define OGF_CB 0x03
#define OPC(ogf, ocf) ((uint16_t)(((ogf) << 10) | (ocf)))

static const hci_cmd_t cmd_set_cpb = { OPC(OGF_LC, 0x41), "1112222" };
static const hci_cmd_t cmd_set_cpb_receive = { OPC(OGF_LC, 0x42), "1B12442112M" };
static const hci_cmd_t cmd_start_sync_train = { OPC(OGF_LC, 0x43), "" };
static const hci_cmd_t cmd_receive_sync_train = { OPC(OGF_LC, 0x44), "B222" };
static const hci_cmd_t cmd_set_resv_lt_addr = { OPC(OGF_CB, 0x74), "1" };
static const hci_cmd_t cmd_del_resv_lt_addr = { OPC(OGF_CB, 0x75), "1" };
static const hci_cmd_t cmd_set_cpb_data = { OPC(OGF_CB, 0x76), "11JV" };
static const hci_cmd_t cmd_write_sync_train_params = { OPC(OGF_CB, 0x78), "2241" };
static const hci_cmd_t cmd_set_event_mask_2 = { OPC(OGF_CB, 0x63), "44" };
static const hci_cmd_t cmd_truncated_page = { OPC(OGF_LC, 0x3F), "B12" };

#define EVT_SYNC_TRAIN_COMPLETE 0x4F
#define EVT_SYNC_TRAIN_RECEIVED 0x50
#define EVT_CPB_RECEIVE         0x51
#define EVT_CPB_TIMEOUT         0x52
#define EVT_TRUNCATED_PAGE      0x53
#define EVT_PERIPH_PAGE_TO      0x54
#define EVT_CPB_CHANNEL_MAP     0x55

/* Set_CPB packet types: this controller accepts only 0x0008 (DM1 + every EDR type allowed,
 * so 2/3-DH1..DH5 by payload size); any basic-rate DHx/DMx bit -> 0x12. Receive: all. */
#define CPB_PKT_TYPES 0xCC18u
#define CPB_TX_PKT_TYPES 0x0008u
/* LT_ADDR 7: ACLs (where we are master) take 1, 2, ... - 1 was the phone's (0x0B) */
#define TX_LT_ADDR    7

/* one command at a time: queued steps, sent when the controller takes commands */
typedef enum { Q_NONE, Q_MASK, Q_TPAGE, Q_RX_TRAIN, Q_RX_SET, Q_RX_OFF, Q_TX_LT, Q_TX_PARAMS, Q_TX_CPB, Q_TX_DATA, Q_TX_TRAIN,
               Q_TX_OFF, Q_TX_LT_DEL } qstep_t;
#define QMAX 12
static qstep_t s_q[QMAX];
static int s_qn;
static btstack_timer_source_t s_qtimer;
static btstack_packet_callback_registration_t s_hci_reg;

static struct {
    bool on;
    bd_addr_t addr;
    uint8_t lt_addr;
    uint16_t interval;
    uint32_t clock_offset, next_instant;
    uint8_t afh[10];
    uint32_t pkts, bad, bytes, timeouts;
    uint8_t tpage_tries;
    bool synced;
} s_rx;
static struct {
    bool on;
    uint16_t interval;
    uint8_t data[256];
    uint8_t len;
    uint16_t ptype, sto;
    uint8_t lpo;
} s_tx;
static csb_rx_cb_t s_rx_cb;
static uint32_t s_tpages;
static void (*s_tpage_cb)(void);

void csb_set_tpage_callback(void (*cb)(void)) { s_tpage_cb = cb; }

static void q_run(btstack_timer_source_t *ts);

static void q_kick(uint32_t ms)
{
    btstack_run_loop_remove_timer(&s_qtimer);
    btstack_run_loop_set_timer_handler(&s_qtimer, q_run);
    btstack_run_loop_set_timer(&s_qtimer, ms);
    btstack_run_loop_add_timer(&s_qtimer);
}

static void q_push(qstep_t s)
{
    if (s_qn < QMAX) s_q[s_qn++] = s;
    q_kick(1);
}

static void q_run(btstack_timer_source_t *ts)
{
    (void)ts;
    if (s_qn == 0) return;
    if (!hci_can_send_command_packet_now()) {
        q_kick(10);
        return;
    }
    qstep_t s = s_q[0];
    memmove(s_q, s_q + 1, (size_t)(--s_qn) * sizeof(s_q[0]));
    switch (s) {
    case Q_MASK:
        /* page 2: BTstack's 0x02000000 + Sync Train Complete/Received, CPB Receive/Timeout,
         * Truncated Page, Peripheral Page Response Timeout, CPB Channel Map Change (b15-21) */
        hci_send_cmd(&cmd_set_event_mask_2, 0x02000000u | 0x003F8000u, 0u);
        break;
    case Q_TPAGE:
        /* Truncated Page: wakes the broadcaster up - a CSB receiver asks it to start its train */
        ESP_LOGW(TAG, "RX: truncated page to %s (asking it to start its synchronization train)", bd_addr_to_str(s_rx.addr));
        hci_send_cmd(&cmd_truncated_page, s_rx.addr, 0x01 /* R1 */, 0x0000);
        break;
    case Q_RX_TRAIN:
        /* scan for its train: timeout ~41 s, window 300 ms every 400 ms */
        ESP_LOGW(TAG, "RX: looking for the synchronization train of %s", bd_addr_to_str(s_rx.addr));
        hci_send_cmd(&cmd_receive_sync_train, s_rx.addr, 0xFFFE, 0x01E0, 0x0280);
        break;
    case Q_RX_SET:
        ESP_LOGW(TAG, "RX: receiving the CPB of %s (LT_ADDR %u, every %u slots)", bd_addr_to_str(s_rx.addr), s_rx.lt_addr,
                 s_rx.interval);
        hci_send_cmd(&cmd_set_cpb_receive, 1, s_rx.addr, s_rx.lt_addr, s_rx.interval, s_rx.clock_offset,
                     s_rx.next_instant, 0x2000, 20, 0, CPB_PKT_TYPES, s_rx.afh);
        break;
    case Q_RX_OFF:
        hci_send_cmd(&cmd_set_cpb_receive, 0, s_rx.addr, s_rx.lt_addr, 0x0002, 0, 0, 0x2000, 20, 0, CPB_PKT_TYPES, s_rx.afh);
        break;
    case Q_TX_LT:
        hci_send_cmd(&cmd_set_resv_lt_addr, TX_LT_ADDR);
        break;
    case Q_TX_PARAMS:
        /* train every 80 ms (128 slots), for ~2 min (0x2EE00 slots = 120 s), service data 0 */
        hci_send_cmd(&cmd_write_sync_train_params, 0x0080, 0x0080, 0x0002EE00u, 0);
        break;
    case Q_TX_CPB:
        ESP_LOGW(TAG, "TX: Set_CPB interval %u..%u slots, packet types 0x%04x, LPO %u, supervision 0x%04x", s_tx.interval,
                 s_tx.interval + 16, s_tx.ptype, s_tx.lpo, s_tx.sto);
        hci_send_cmd(&cmd_set_cpb, 1, TX_LT_ADDR, s_tx.lpo, s_tx.ptype, s_tx.interval, s_tx.interval + 16, s_tx.sto);
        break;
    case Q_TX_DATA:
        hci_send_cmd(&cmd_set_cpb_data, TX_LT_ADDR, 3 /* complete */, s_tx.len, s_tx.data);
        break;
    case Q_TX_TRAIN:
        hci_send_cmd(&cmd_start_sync_train);
        break;
    case Q_TX_OFF:
        hci_send_cmd(&cmd_set_cpb, 0, TX_LT_ADDR, s_tx.lpo, s_tx.ptype, s_tx.interval, s_tx.interval + 16, s_tx.sto);
        break;
    case Q_TX_LT_DEL:
        hci_send_cmd(&cmd_del_resv_lt_addr, TX_LT_ADDR);
        break;
    default:
        break;
    }
    if (s_qn) q_kick(5);
}

static const char *opname(uint16_t op)
{
    switch (op) {
    case 0x0441: return "Set_CPB";
    case 0x0442: return "Set_CPB_Receive";
    case 0x0443: return "Start_Sync_Train";
    case 0x0444: return "Receive_Sync_Train";
    case 0x043F: return "Truncated_Page";
    case 0x0C74: return "Set_Reserved_LT_ADDR";
    case 0x0C75: return "Delete_Reserved_LT_ADDR";
    case 0x0C76: return "Set_CPB_Data";
    case 0x0C78: return "Write_Sync_Train_Params";
    case 0x0C63: return "Set_Event_Mask_Page_2";
    default: return NULL;
    }
}

static void hci_handler(uint8_t packet_type, uint16_t channel, uint8_t *p, uint16_t size)
{
    (void)channel;
    if (packet_type != HCI_EVENT_PACKET) return;
    switch (hci_event_packet_get_type(p)) {
    case HCI_EVENT_COMMAND_COMPLETE: {
        uint16_t op = hci_event_command_complete_get_command_opcode(p);
        const char *n = opname(op);
        if (n && size >= 6) ESP_LOGW(TAG, "%s -> status 0x%02x", n, p[5]);
        break;
    }
    case HCI_EVENT_COMMAND_STATUS: {
        uint16_t op = hci_event_command_status_get_command_opcode(p);
        const char *n = opname(op);
        if (n) ESP_LOGW(TAG, "%s -> status 0x%02x%s", n, p[2], p[2] == 0x01 ? " (UNKNOWN COMMAND: CPB not in the controller)" : "");
        break;
    }
    case EVT_SYNC_TRAIN_RECEIVED: {
        if (size < 2 + 29) break;
        uint8_t st = p[2];
        if (st != 0) {
            ESP_LOGW(TAG, "RX: no synchronization train from %s (status 0x%02x)", bd_addr_to_str(s_rx.addr), st);
            s_rx.on = false;
            break;
        }
        reverse_bd_addr(p + 3, s_rx.addr);
        s_rx.synced = true;
        s_rx.clock_offset = little_endian_read_32(p, 9);
        memcpy(s_rx.afh, p + 13, 10);
        s_rx.lt_addr = p[23];
        s_rx.next_instant = little_endian_read_32(p, 24);
        s_rx.interval = little_endian_read_16(p, 28);
        uint8_t service = p[30];
        ESP_LOGW(TAG, "RX: SYNC TRAIN from %s: LT_ADDR %u, interval %u slots (%u.%03u ms), clock offset 0x%08lx, next 0x%08lx, "
                      "service data 0x%02x, AFH %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
                 bd_addr_to_str(s_rx.addr), s_rx.lt_addr, s_rx.interval, s_rx.interval * 625 / 1000, s_rx.interval * 625 % 1000,
                 (unsigned long)s_rx.clock_offset, (unsigned long)s_rx.next_instant, service, s_rx.afh[0], s_rx.afh[1],
                 s_rx.afh[2], s_rx.afh[3], s_rx.afh[4], s_rx.afh[5], s_rx.afh[6], s_rx.afh[7], s_rx.afh[8], s_rx.afh[9]);
        q_push(Q_RX_SET);
        break;
    }
    case EVT_CPB_RECEIVE: {
        if (size < 2 + 18) break;
        uint8_t lt = p[8];
        uint32_t clk = little_endian_read_32(p, 9);
        uint8_t rx_status = p[17], frag = p[18], dl = p[19];
        const uint8_t *d = p + 20;
        if (size < 20u + dl) break;                   /* a truncated event: no read past it */
        if (rx_status) s_rx.bad++;
        else { s_rx.pkts++; s_rx.bytes += dl; }
        if (s_rx.pkts + s_rx.bad <= 20 || ((s_rx.pkts + s_rx.bad) % 200) == 0) {
            char h[3 * 48 + 1];
            int k = 0;
            for (int i = 0; i < dl && i < 48; i++) k += snprintf(h + k, sizeof(h) - (size_t)k, "%02x ", d[i]);
            h[k] = 0;
            ESP_LOGW(TAG, "RX CPB #%lu: LT %u clk 0x%08lx status %u frag %u len %u: %s", (unsigned long)(s_rx.pkts + s_rx.bad),
                     lt, (unsigned long)clk, rx_status, frag, dl, h);
        }
        if (s_rx_cb && !rx_status) s_rx_cb(clk, frag, d, dl);
        break;
    }
    case EVT_CPB_TIMEOUT:
        s_rx.timeouts++;
        ESP_LOGW(TAG, "RX: CPB timeout (the broadcaster went away) - %lu packets, %lu bad", (unsigned long)s_rx.pkts,
                 (unsigned long)s_rx.bad);
        s_rx.on = false;
        break;
    case EVT_SYNC_TRAIN_COMPLETE:
        ESP_LOGW(TAG, "TX: synchronization train ended (status 0x%02x)%s", p[2], s_tx.on ? " - restarting it" : "");
        if (s_tx.on) q_push(Q_TX_TRAIN);
        break;
    case EVT_CPB_CHANNEL_MAP:
        ESP_LOGW(TAG, "CPB channel map changed");
        break;
    case EVT_TRUNCATED_PAGE:
        ESP_LOGW(TAG, "truncated page done, status 0x%02x%s", p[2], p[2] == 0x04 ? " (no answer: it is not page scanning)" : "");
        if (s_rx.on && !s_rx.synced && ++s_rx.tpage_tries < 4) q_push(Q_TPAGE);    /* again while its train is awaited */
        break;
    case EVT_PERIPH_PAGE_TO:
        /* someone truncated-paged us: a CSB receiver asking for our synchronization train */
        s_tpages++;
        ESP_LOGW(TAG, "TRUNCATED PAGE received (#%lu): a receiver asks for our synchronization train%s", (unsigned long)s_tpages,
                 s_tx.on ? " - restarting it" : "");
        if (s_tx.on) q_push(Q_TX_TRAIN);
        if (s_tpage_cb) s_tpage_cb();
        break;
    default:
        break;
    }
}

void csb_setup(void)
{
    s_hci_reg.callback = &hci_handler;
    hci_add_event_handler(&s_hci_reg);
}

void csb_on_hci_working(void)
{
    q_push(Q_MASK);
}

void csb_rx_start(const bd_addr_t addr)
{
    memset(&s_rx, 0, sizeof(s_rx));
    bd_addr_copy(s_rx.addr, addr);
    s_rx.on = true;
    q_push(Q_RX_TRAIN);
}

void csb_rx_join(const bd_addr_t addr)
{
    memset(&s_rx, 0, sizeof(s_rx));
    bd_addr_copy(s_rx.addr, addr);
    s_rx.on = true;
    q_push(Q_TPAGE);
    q_push(Q_RX_TRAIN);
}

void csb_rx_stop(void)
{
    if (!s_rx.on) return;
    s_rx.on = false;
    q_push(Q_RX_OFF);
}

void csb_rx_set_callback(csb_rx_cb_t cb) { s_rx_cb = cb; }

void csb_tx_set_params(uint16_t ptype, uint8_t lpo, uint16_t supervision)
{
    s_tx.ptype = ptype;
    s_tx.lpo = lpo;
    s_tx.sto = supervision;
}

void csb_tx_start(uint16_t interval_slots, const uint8_t *data, uint8_t len)
{
    if (!s_tx.ptype) { s_tx.ptype = CPB_TX_PKT_TYPES; s_tx.sto = 0x2000; }
    s_tx.on = true;
    s_tx.interval = interval_slots < 2 ? 2 : interval_slots;
    s_tx.len = len;
    memcpy(s_tx.data, data, len);
    ESP_LOGW(TAG, "TX: CPB on LT_ADDR %u every %u slots + synchronization train", TX_LT_ADDR, s_tx.interval);
    q_push(Q_TX_LT);
    q_push(Q_TX_PARAMS);
    q_push(Q_TX_CPB);
    q_push(Q_TX_DATA);
    q_push(Q_TX_TRAIN);
}

void csb_tx_data(const uint8_t *data, uint8_t len)
{
    if (!s_tx.on) return;
    s_tx.len = len;
    memcpy(s_tx.data, data, len);
    q_push(Q_TX_DATA);
}

void csb_tx_stop(void)
{
    if (!s_tx.on) return;
    s_tx.on = false;
    q_push(Q_TX_OFF);
    q_push(Q_TX_LT_DEL);
}

void csb_log_stats(void)
{
    if (s_rx.on || s_rx.pkts)
        ESP_LOGI(TAG, "CPB RX %s: %lu packets (%lu B), %lu bad, %lu timeouts", s_rx.on ? "on" : "off",
                 (unsigned long)s_rx.pkts, (unsigned long)s_rx.bytes, (unsigned long)s_rx.bad, (unsigned long)s_rx.timeouts);
}
