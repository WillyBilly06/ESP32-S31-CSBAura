/*
 * JBL Xtreme 5-style Auracast party role switching: HOST while the phone
 * streams, LISTEN (BAP broadcast sink of another JBL master) otherwise.
 * See party_role.h for the behaviour.
 *
 * State machine (party task; every arrow is logged as "PARTY: A -> B (why)"):
 *
 *   HOST ---(no A2DP audio for 3 s)---> HOST_STOPPING ---> LISTEN
 *   LISTEN ---(A2DP audio STARTED)---> LISTEN_STOPPING ---> HOST_STARTING ---> HOST
 *
 *   inside LISTEN:
 *     SCAN --(JBL master ext adv)--> PA_SYNCING --(PA synced)--> BASE_WAIT
 *       --(BASE + BIGInfo)--> BIG_SYNCING --(all BIS started)--> STREAMING
 *     any failure / timeout / loss --> tear down sink + PA --> SCAN
 *
 * Contexts: the party task makes every stack call (create/terminate sync,
 * sink create/sync/stop/delete, scan start/stop, host start/stop). The iso
 * task (LE Audio GAP app callback, sink + stream callbacks) and the BTC task
 * (Bluedroid GAP events, A2DP) only post events / set event bits. Events from
 * an earlier sink session carry an older generation number and are dropped.
 * Decoded audio: iso task (ISO SDU) -> queue -> lc3_dec task -> I2S.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_bt_device.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/idf_additions.h"

#include "esp_ble_audio_lc3_defs.h"
#include "esp_ble_audio_bap_api.h"
#include "esp_ble_audio_pacs_api.h"
#include "esp_ble_audio_codec_api.h"

#include "driver/i2s_std.h"

#include "lc3.h"

#include "party_role.h"
#include "a2dp_bridge.h"
#include "jbl_assistant.h"
#include "app_task.h"

#define PTAG                    "PARTY"

/* ---- tuning ----------------------------------------------------------------- */
#ifndef PARTY_HOST_SETTLE_MS
#define PARTY_HOST_SETTLE_MS    300
#endif
#define SCAN_ITVL               240     /* 150 ms */
#define SCAN_WIN                48      /* 30 ms (20 % duty). 50/100 ms starved BR/EDR paging: the
                                         * phone's reconnects failed ("page recycle resched failed")
                                         * and its A2DP link dropped while LISTEN scanned. */
#define PA_SYNC_SKIP            0
#define PA_SYNC_TIMEOUT_10MS    200     /* 2 s supervision */
#define T_PA_SYNC_MS            5000    /* create sync -> established */
#define T_BASE_MS               5000    /* PA synced -> BASE + BIGInfo */
#define T_BIG_MS                5000    /* BIG create sync -> all BIS started */
#define T_NO_SDU_MS             3000    /* streaming but nothing received */
#define T_CANCEL_MS             3000    /* create-sync cancel never answered */
#define BLACKLIST_FAIL_MS       5000
#define BLACKLIST_ENC_MS        60000   /* encrypted broadcast: we have no code */
#define LOOKING_LOG_MS          10000
#define ASSIST_QUIET_MS         1500

#define JBL_BROADCAST_ID        0x112233    /* every JBL master hosts with this ID */
#define HARMAN_COMPANY_ID       0x0057
#define JBL_FDDF                0xFDDF
#define UUID_BCAST_AUDIO        0x1852

#define I2S_BCLK_PIN            2
#define I2S_WS_PIN              3
#define I2S_DOUT_PIN            4

#define SNK_STREAMS             CONFIG_BT_BAP_BROADCAST_SNK_STREAM_COUNT
#define MAX_FS                  480         /* 10 ms @ 48 kHz */
#define MAX_SDU                 320         /* 2 x 155 octets (two channels in one BIS) */
#define SDU_Q_LEN               16

/* ---- states ---------------------------------------------------------------- */
typedef enum {
    ROLE_HOST, ROLE_HOST_STOPPING, ROLE_LISTEN, ROLE_LISTEN_STOPPING, ROLE_HOST_STARTING,
} role_t;
static const char *const role_name[] = {
    "HOST", "HOST_STOPPING", "LISTEN", "LISTEN_STOPPING", "HOST_STARTING",
};

typedef enum {
    L_IDLE, L_SCAN, L_PA_SYNCING, L_BASE_WAIT, L_BIG_SYNCING, L_STREAMING,
} lstate_t;
static const char *const lstate_name[] = {
    "idle", "scanning", "PA syncing", "waiting BASE/BIGInfo", "BIG syncing", "streaming",
};

enum {
    EV_WAKE, EV_CAND, EV_PA_SYNC, EV_PA_LOST, EV_BASE, EV_SYNCABLE,
    EV_STREAM_STARTED, EV_STREAM_STOPPED, EV_SINK_STOPPED, EV_SRC_STOPPED,
};

typedef struct {
    uint8_t  addr[6];
    uint8_t  addr_type;
    uint8_t  sid;
    uint32_t broadcast_id;
    int8_t   rssi;
    bool     harman;
    char     name[33];
} cand_t;

typedef struct {
    uint8_t  type;
    uint8_t  status;
    uint16_t handle;
    uint32_t val;
    uint32_t gen;
    cand_t   cand;
} pr_evt_t;

#define EB_SCAN_PARAMS          BIT0
#define EB_SCAN_START           BIT1
#define EB_SCAN_STOP            BIT2
#define EB_PA_LOST              BIT3
#define EB_STREAMS_STOPPED      BIT4

/* ---- shared state ---------------------------------------------------------- */
static const party_role_ops_t *s_ops;
static QueueHandle_t      s_q;
static EventGroupHandle_t s_eg;
static volatile role_t    s_role = ROLE_HOST;
static volatile lstate_t  s_lstate = L_IDLE;
static volatile uint32_t  s_gen;
static TickType_t         s_lstate_t;          /* when s_lstate was entered */
static TickType_t         s_listen_t;          /* when LISTEN was entered */
static TickType_t         s_looking_log_t;
static uint16_t           s_own_crc;
static uint8_t            s_own_addr[6];
static bool               s_host_up;           /* broadcast source running (party task view) */
static TickType_t         s_host_try_t;
static volatile bool      s_src_running;       /* source started cb seen, no stopped cb since */

/* scanner (owned by the party task while listening) */
static volatile bool      s_scan_owner;
static volatile esp_bt_status_t s_scan_status;
static bool               s_scan_on;
static TickType_t         s_scan_try_t;
static volatile bool      s_want_cand;         /* iso cb may post one candidate */
static esp_ble_ext_scan_params_t s_scan_params = {
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
    .cfg_mask = ESP_BLE_GAP_EXT_SCAN_CFG_UNCODE_MASK,
    .uncoded_cfg = { BLE_SCAN_TYPE_PASSIVE, SCAN_ITVL, SCAN_WIN },
};

/* blacklist: one address (a master we just failed on) */
static uint8_t            s_bl_addr[6];
static volatile TickType_t s_bl_until;

/* PA / sink session */
static cand_t             s_cur;
static bool               s_create_pending;
static TickType_t         s_create_t;
static volatile bool      s_pa_synced;
static volatile uint16_t  s_pa_handle;
static esp_ble_audio_bap_broadcast_sink_t *volatile s_sink;
static volatile uint32_t  s_base_bits;
static volatile bool      s_base_posted;
static bool               s_syncable, s_encrypted;
static uint8_t            s_streams_wanted;
static volatile uint8_t   s_streams_started;

/* audio path */
static struct sink_stream {
    esp_ble_audio_bap_stream_t stream;
    lc3_decoder_t dec[2];
    void         *dec_mem[2];
    volatile int  nch;        /* channels in this BIS; 0 = decode path off */
    int           side;       /* 0 L / 1 R for a single-channel BIS */
    int           opf;        /* octets per LC3 frame */
    int           fs;         /* samples per channel per frame */
    uint32_t      sr_hz;
    bool          started;
} s_snk[SNK_STREAMS];
static esp_ble_audio_bap_stream_t *s_snk_p[SNK_STREAMS];

typedef struct {
    uint8_t  idx;
    uint16_t len;             /* 0 = lost SDU -> packet loss concealment */
    uint8_t  data[MAX_SDU];
} sdu_t;
static QueueHandle_t      s_sdu_q;
static SemaphoreHandle_t  s_dec_lock;          /* decoders + I2S channel */
static i2s_chan_handle_t  s_i2s;
static volatile bool      s_rx_on;             /* recv cb queues SDUs */
static volatile bool      s_mono;
static volatile TickType_t s_last_sdu_t;

static struct {
    volatile uint32_t sdus, lost, qdrops, decoded, plc, errors, i2s_late;
} s_st;

/* ---- helpers --------------------------------------------------------------- */

static const char *pi_name(uint16_t pi)
{
    return pi == PARTY_INFO_HOST ? "Primary" :
           pi == PARTY_INFO_LOOKING ? "Secondary, looking for master" :
           pi == PARTY_INFO_SECONDARY ? "Secondary, connected" : "?";
}

static void set_party_info(uint16_t pi)
{
    static uint16_t cur;
    if (pi == cur) {
        return;
    }
    cur = pi;
    ESP_LOGW(PTAG, "PARTY: beacon partyInfo -> 0x%04x (%s)", pi, pi_name(pi));
    if (s_ops && s_ops->set_party_info) {
        s_ops->set_party_info(pi);
    }
}

static void set_role(role_t r, const char *why)
{
    role_t old = s_role;
    if (old == r) {
        return;
    }
    s_role = r;
    ESP_LOGW(PTAG, "PARTY: %s -> %s (%s)", role_name[old], role_name[r], why);
}

static void set_lstate(lstate_t l, const char *why)
{
    lstate_t old = s_lstate;
    s_lstate = l;
    s_lstate_t = xTaskGetTickCount();
    s_want_cand = (l == L_SCAN);
    if (old != l) {
        ESP_LOGW(PTAG, "PARTY: LISTEN %s -> %s (%s)", lstate_name[old], lstate_name[l], why);
    }
}

static void post(uint8_t type, uint8_t status, uint16_t handle, uint32_t val)
{
    pr_evt_t ev = { .type = type, .status = status, .handle = handle, .val = val, .gen = s_gen };
    if (s_q != NULL) {
        xQueueSend(s_q, &ev, 0);
    }
}

static bool elapsed(TickType_t since, uint32_t ms)
{
    return (TickType_t)(xTaskGetTickCount() - since) >= pdMS_TO_TICKS(ms);
}

static uint32_t ms_since(TickType_t t)
{
    return (uint32_t)((xTaskGetTickCount() - t) * portTICK_PERIOD_MS);
}

static bool addr_is_own(const uint8_t *a)
{
    bool fwd = memcmp(a, s_own_addr, 6) == 0, rev = true;
    for (int i = 0; i < 6; i++) {
        if (a[i] != s_own_addr[5 - i]) {
            rev = false;
            break;
        }
    }
    return fwd || rev;
}

static void blacklist(const uint8_t *a, uint32_t ms)
{
    memcpy(s_bl_addr, a, 6);
    s_bl_until = xTaskGetTickCount() + pdMS_TO_TICKS(ms);
}

static bool blacklisted(const uint8_t *a)
{
    return (int32_t)(s_bl_until - xTaskGetTickCount()) > 0 && memcmp(a, s_bl_addr, 6) == 0;
}

/* ---- scanner --------------------------------------------------------------- */

void party_role_gap_event(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    if (!s_scan_owner || s_eg == NULL) {
        return;
    }
    switch (event) {
    case ESP_GAP_BLE_SET_EXT_SCAN_PARAMS_COMPLETE_EVT:
        s_scan_status = param->set_ext_scan_params.status;
        xEventGroupSetBits(s_eg, EB_SCAN_PARAMS);
        break;
    case ESP_GAP_BLE_EXT_SCAN_START_COMPLETE_EVT:
        s_scan_status = param->ext_scan_start.status;
        xEventGroupSetBits(s_eg, EB_SCAN_START);
        break;
    case ESP_GAP_BLE_EXT_SCAN_STOP_COMPLETE_EVT:
        s_scan_status = param->ext_scan_stop.status;
        xEventGroupSetBits(s_eg, EB_SCAN_STOP);
        break;
    default:
        break;
    }
}

static bool scan_wait(EventBits_t bit, const char *what)
{
    EventBits_t b = xEventGroupWaitBits(s_eg, bit, pdTRUE, pdFALSE, pdMS_TO_TICKS(1000));
    if (!(b & bit)) {
        ESP_LOGW(PTAG, "scan %s: no completion event", what);
        return false;
    }
    if (s_scan_status != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGW(PTAG, "scan %s: controller status 0x%x", what, s_scan_status);
        return false;
    }
    return true;
}

static bool scan_start(void)
{
    s_scan_try_t = xTaskGetTickCount();
    if (s_scan_on) {
        return true;
    }
    s_scan_owner = true;
    xEventGroupClearBits(s_eg, EB_SCAN_PARAMS | EB_SCAN_START | EB_SCAN_STOP);
    if (esp_ble_gap_set_ext_scan_params(&s_scan_params) != ESP_OK || !scan_wait(EB_SCAN_PARAMS, "params")) {
        return false;
    }
    if (esp_ble_gap_start_ext_scan(0, 0) != ESP_OK || !scan_wait(EB_SCAN_START, "start")) {
        return false;
    }
    s_scan_on = true;
    ESP_LOGI(PTAG, "scanning for JBL masters (%u/%u ms)", SCAN_WIN * 5 / 8, SCAN_ITVL * 5 / 8);
    return true;
}

static void scan_stop(void)
{
    if (s_scan_on) {
        xEventGroupClearBits(s_eg, EB_SCAN_STOP);
        if (esp_ble_gap_stop_ext_scan() == ESP_OK) {
            scan_wait(EB_SCAN_STOP, "stop");
        }
        s_scan_on = false;
    }
    s_scan_owner = false;
}

/* ---- scan report filter (iso task) ------------------------------------------ */

/* Offset of a V5 beacon field (same table as jbl_assistant.c), or -1. */
static int v5_field_offset(const uint8_t *b, int n, int bit, int size)
{
    static const int8_t sz[16] = { 0, 2, 5, 2, 3, 3, 1, 2, 1, 2, 1, 6, 2, 2, 2, 1 };
    if (n < 4) {
        return -1;
    }
    uint16_t mask = b[2] | (b[3] << 8);
    if (!(mask & (1 << bit))) {
        return -1;
    }
    int off = 4;
    for (int k = 15; k > bit; k--) {
        if (mask & (1 << k)) {
            off += sz[k];
        }
    }
    return off + size <= n ? off : -1;
}

static bool contains_jbl(const uint8_t *s, int n)
{
    for (int i = 0; i + 3 <= n; i++) {
        if (s[i] == 'J' && s[i + 1] == 'B' && s[i + 2] == 'L') {
            return true;
        }
    }
    return false;
}

static void on_beacon(const uint8_t *b, int n)
{
    static uint16_t seen[8];
    static int nseen;
    int co = v5_field_offset(b, n, 12, 2);
    int po = v5_field_offset(b, n, 7, 2);
    if (co < 0 || po < 0) {
        return;
    }
    uint16_t crc = b[co] | (b[co + 1] << 8);
    uint16_t pi = b[po] | (b[po + 1] << 8);
    if (crc == s_own_crc || !(pi & 0x8000) || ((pi >> 9) & 3) != 2) {
        return;                     /* us, or not a Primary with party on */
    }
    for (int i = 0; i < nseen; i++) {
        if (seen[i] == crc) {
            return;
        }
    }
    if (nseen < (int)(sizeof(seen) / sizeof(seen[0]))) {
        seen[nseen++] = crc;
    }
    ESP_LOGW(PTAG, "PARTY: JBL master beacon seen: macCrc 0x%04x partyInfo 0x%04x - looking for its broadcast",
             crc, pi);
}

static void on_ext_scan_recv(const esp_ble_audio_gap_app_event_t *e)
{
    const uint8_t *d = e->ext_scan_recv.data;
    int len = e->ext_scan_recv.data_len;
    cand_t c = {0};
    bool bcast = false;

    for (int i = 0; d != NULL && i + 1 < len;) {
        int l = d[i];
        if (l == 0 || i + 1 + l > len) {
            break;
        }
        uint8_t type = d[i + 1];
        const uint8_t *v = d + i + 2;
        int vl = l - 1;
        if (type == 0x16 && vl >= 2) {                     /* Service Data 16 */
            uint16_t uuid = v[0] | (v[1] << 8);
            if (uuid == UUID_BCAST_AUDIO && vl >= 5) {
                c.broadcast_id = v[2] | (v[3] << 8) | ((uint32_t)v[4] << 16);
                bcast = true;
            } else if (uuid == JBL_FDDF) {
                on_beacon(v + 2, vl - 2);
            }
        } else if (type == 0xFF && vl >= 2) {              /* Manufacturer Specific */
            if ((v[0] | (v[1] << 8)) == HARMAN_COMPANY_ID) {
                c.harman = true;
            }
        } else if (type == 0x30 || ((type == 0x09 || type == 0x08) && c.name[0] == 0)) {
            int n = vl < (int)sizeof(c.name) - 1 ? vl : (int)sizeof(c.name) - 1;
            memcpy(c.name, v, n);
            c.name[n] = 0;
        }
        i += l + 1;
    }

    if (!bcast || e->ext_scan_recv.per_adv_itvl == 0 || !s_want_cand) {
        return;
    }
    if (!c.harman && c.broadcast_id != JBL_BROADCAST_ID && !contains_jbl((const uint8_t *)c.name, strlen(c.name))) {
        return;                                            /* some other Auracast source */
    }
    if (addr_is_own(e->ext_scan_recv.addr.val) || blacklisted(e->ext_scan_recv.addr.val)) {
        return;
    }
    memcpy(c.addr, e->ext_scan_recv.addr.val, 6);
    c.addr_type = e->ext_scan_recv.addr.type;
    c.sid = e->ext_scan_recv.sid;
    c.rssi = e->ext_scan_recv.rssi;

    s_want_cand = false;                                   /* one at a time */
    pr_evt_t ev = { .type = EV_CAND, .gen = s_gen, .cand = c };
    if (xQueueSend(s_q, &ev, 0) != pdTRUE) {
        s_want_cand = true;
    }
}

void party_role_iso_gap_cb(esp_ble_audio_gap_app_event_t *event)
{
    switch (event->type) {
    case ESP_BLE_AUDIO_GAP_EVENT_EXT_SCAN_RECV:
        if (s_role == ROLE_LISTEN) {
            on_ext_scan_recv(event);
        }
        break;
    case ESP_BLE_AUDIO_GAP_EVENT_PA_SYNC: {
        pr_evt_t ev = {
            .type = EV_PA_SYNC, .status = event->pa_sync.status,
            .handle = event->pa_sync.sync_handle, .gen = s_gen,
        };
        memcpy(ev.cand.addr, event->pa_sync.addr.val, 6);
        ev.cand.sid = event->pa_sync.sid;
        if (s_q != NULL) {
            xQueueSend(s_q, &ev, pdMS_TO_TICKS(50));       /* must not be lost (orphan syncs) */
        }
        break;
    }
    case ESP_BLE_AUDIO_GAP_EVENT_PA_SYNC_LOST:
        if (s_pa_synced && event->pa_sync_lost.sync_handle == s_pa_handle && s_eg != NULL) {
            xEventGroupSetBits(s_eg, EB_PA_LOST);
        }
        post(EV_PA_LOST, event->pa_sync_lost.reason, event->pa_sync_lost.sync_handle, 0);
        break;
    default:
        break;
    }
}

/* ---- broadcast sink callbacks (iso task) ------------------------------------ */

static void base_recv_cb(esp_ble_audio_bap_broadcast_sink_t *sink,
                         const esp_ble_audio_bap_base_t *base, size_t base_size)
{
    uint32_t bits = 0;
    if (sink != s_sink || s_base_posted) {
        return;
    }
    if (esp_ble_audio_bap_base_get_bis_indexes(base, &bits) != ESP_OK || bits == 0) {
        return;
    }
    s_base_bits = bits;
    s_base_posted = true;
    post(EV_BASE, 0, 0, bits);
}

static void syncable_cb(esp_ble_audio_bap_broadcast_sink_t *sink, const esp_ble_iso_biginfo_t *biginfo)
{
    if (sink != s_sink) {
        return;
    }
    ESP_LOGI(PTAG, "BIGInfo: %u BIS, NSE %u, ISO interval %u, BN %u, IRC %u, PTO %u, max PDU %u, %s",
             biginfo->num_bis, biginfo->sub_evt_count, biginfo->iso_interval, biginfo->burst_number,
             biginfo->rep_count, biginfo->offset, biginfo->max_pdu,
             biginfo->encryption ? "ENCRYPTED" : "open");
    post(EV_SYNCABLE, 0, 0, biginfo->encryption ? 1 : 0);
}

static void sink_started_cb(esp_ble_audio_bap_broadcast_sink_t *sink)
{
    ESP_LOGI(PTAG, "broadcast sink: BIG synced");
}

static void sink_stopped_cb(esp_ble_audio_bap_broadcast_sink_t *sink, uint8_t reason)
{
    ESP_LOGI(PTAG, "broadcast sink: BIG sync ended, reason 0x%02x", reason);
    if (s_eg != NULL) {
        xEventGroupSetBits(s_eg, EB_STREAMS_STOPPED);
    }
    post(EV_SINK_STOPPED, reason, 0, 0);
}

static esp_ble_audio_bap_broadcast_sink_cb_t s_sink_cbs = {
    .base_recv = base_recv_cb,
    .syncable  = syncable_cb,
    .started   = sink_started_cb,
    .stopped   = sink_stopped_cb,
};

static int snk_index(const esp_ble_audio_bap_stream_t *stream)
{
    for (int i = 0; i < SNK_STREAMS; i++) {
        if (&s_snk[i].stream == stream) {
            return i;
        }
    }
    return -1;
}

static void snk_started_cb(esp_ble_audio_bap_stream_t *stream)
{
    int idx = snk_index(stream);
    if (idx < 0) {
        return;
    }
    struct sink_stream *ss = &s_snk[idx];
    const esp_ble_audio_codec_cfg_t *cc = stream->codec_cfg;
    uint32_t sr = 48000, dt = 10000;
    uint16_t opf = 0;
    esp_ble_audio_location_t alloc = 0;
    int nch = 0;

    if (cc != NULL) {
        esp_ble_audio_codec_cfg_freq_t f;
        esp_ble_audio_codec_cfg_frame_dur_t fd;
        if (esp_ble_audio_codec_cfg_get_freq(cc, &f) == ESP_OK) {
            esp_ble_audio_codec_cfg_freq_to_freq_hz(f, &sr);
        }
        if (esp_ble_audio_codec_cfg_get_frame_dur(cc, &fd) == ESP_OK) {
            esp_ble_audio_codec_cfg_frame_dur_to_frame_dur_us(fd, &dt);
        }
        esp_ble_audio_codec_cfg_get_octets_per_frame(cc, &opf);
        esp_ble_audio_codec_cfg_get_chan_allocation(cc, &alloc, true);
    }
    for (uint32_t m = (uint32_t)alloc; m; m &= m - 1) {
        nch++;
    }
    nch = nch < 1 ? 1 : nch > 2 ? 2 : nch;

    xSemaphoreTake(s_dec_lock, portMAX_DELAY);
    ss->opf = opf;
    ss->sr_hz = sr;
    ss->fs = lc3_frame_samples((int)dt, (int)sr);
    if (ss->fs > MAX_FS || ss->fs <= 0) {
        ss->fs = 0;
    }
    ss->side = (alloc == ESP_BLE_AUDIO_LOCATION_FRONT_RIGHT) ? 1 : 0;
    bool ok = ss->fs > 0 && opf > 0;
    for (int c = 0; ok && c < nch; c++) {
        if (ss->dec[c] == NULL) {
            unsigned sz = lc3_decoder_size((int)dt, (int)sr);
            ss->dec_mem[c] = heap_caps_malloc(sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (ss->dec_mem[c] == NULL) {
                ss->dec_mem[c] = heap_caps_malloc(sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            }
            if (ss->dec_mem[c] != NULL) {
                ss->dec[c] = lc3_setup_decoder((int)dt, (int)sr, 0, ss->dec_mem[c]);
            }
            ok = ss->dec[c] != NULL;
        }
    }
    ss->nch = ok ? nch : 0;
    ss->started = true;
    xSemaphoreGive(s_dec_lock);

    ESP_LOGI(PTAG, "[SNK #%d] stream started: LC3 %lu Hz / %lu us / %u octets, %d ch (%s)%s", idx,
             (unsigned long)sr, (unsigned long)dt, opf, nch,
             nch >= 2 ? "L+R" : ss->side ? "R" : "L", ok ? "" : " - NO DECODER, muted");
    s_streams_started++;
    post(EV_STREAM_STARTED, 0, idx, 0);
}

static void snk_stopped_cb(esp_ble_audio_bap_stream_t *stream, uint8_t reason)
{
    int idx = snk_index(stream);
    if (idx < 0) {
        return;
    }
    s_snk[idx].nch = 0;             /* decode path stops touching it */
    if (s_snk[idx].started) {
        s_snk[idx].started = false;
        if (s_streams_started > 0) {
            s_streams_started--;
        }
    }
    ESP_LOGI(PTAG, "[SNK #%d] stream stopped, reason 0x%02x", idx, reason);
    if (s_streams_started == 0 && s_eg != NULL) {
        xEventGroupSetBits(s_eg, EB_STREAMS_STOPPED);
    }
    post(EV_STREAM_STOPPED, reason, idx, 0);
}

static void snk_recv_cb(esp_ble_audio_bap_stream_t *stream, const esp_ble_iso_recv_info_t *info,
                        const uint8_t *data, uint16_t len)
{
    int idx = snk_index(stream);
    if (idx < 0 || !s_rx_on || s_sdu_q == NULL) {
        return;
    }
    sdu_t it;
    it.idx = idx;
    bool valid = data != NULL && len > 0 && len <= MAX_SDU &&
                 (info == NULL || (info->flags & ESP_BLE_ISO_FLAGS_VALID));
    it.len = valid ? len : 0;
    if (valid) {
        memcpy(it.data, data, len);
    } else {
        s_st.lost++;
    }
    s_st.sdus++;
    s_last_sdu_t = xTaskGetTickCount();
    if (xQueueSend(s_sdu_q, &it, 0) != pdTRUE) {
        s_st.qdrops++;
    }
}

static esp_ble_audio_bap_stream_ops_t s_snk_ops = {
    .started = snk_started_cb,
    .stopped = snk_stopped_cb,
    .recv    = snk_recv_cb,
};

/* Remote Broadcast Assistants (phones) cannot steer us: the party task decides. */
static int pa_sync_req_cb(esp_ble_conn_t *conn,
                          const esp_ble_audio_bap_scan_delegator_recv_state_t *recv_state,
                          bool past_available, uint16_t pa_interval)
{
    return -ENOTSUP;
}

static esp_ble_audio_bap_scan_delegator_cb_t s_deleg_cbs = {
    .pa_sync_req = pa_sync_req_cb,
};

/* ---- audio out --------------------------------------------------------------- */

static void i2s_out(const int16_t *l, const int16_t *r, int fs)
{
    static int32_t out[MAX_FS * 2];
    if (s_i2s == NULL) {
        return;                     /* not streaming yet: decode only (keeps LC3 state warm) */
    }
    for (int i = 0; i < fs; i++) {
        out[2 * i]     = ((int32_t)l[i]) << 16;
        out[2 * i + 1] = ((int32_t)r[i]) << 16;
    }
    size_t wr = 0;
    i2s_channel_write(s_i2s, out, (size_t)fs * 2 * sizeof(int32_t), &wr, pdMS_TO_TICKS(40));
    if (wr < (size_t)fs * 2 * sizeof(int32_t)) {
        s_st.i2s_late++;
    }
}

/* LC3 frame dump of the master's first BIS while listening, for host-side
 * analysis of a real JBL encoder (tools/lc3_dump_analyze.py). One line per
 * SDU: "LC3F <n> <len> <base64>" (len 0 = lost). ~16 KB/s of text at 921600. */
#ifndef PARTY_LC3_DUMP
#define PARTY_LC3_DUMP 0   /* debug only: ~16 KB/s of UART text busy-waits CPU 1 */
#endif
#if PARTY_LC3_DUMP
static void dump_frame(const sdu_t *it)
{
    static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    static uint32_t n;
    char out[4 * (MAX_SDU / 3 + 1) + 1];
    int o = 0;
    for (int i = 0; i < it->len; i += 3) {
        uint32_t v = (uint32_t)it->data[i] << 16;
        if (i + 1 < it->len) v |= (uint32_t)it->data[i + 1] << 8;
        if (i + 2 < it->len) v |= it->data[i + 2];
        out[o++] = b64[(v >> 18) & 63];
        out[o++] = b64[(v >> 12) & 63];
        out[o++] = i + 1 < it->len ? b64[(v >> 6) & 63] : '=';
        out[o++] = i + 2 < it->len ? b64[v & 63] : '=';
    }
    out[o] = 0;
    printf("LC3F %lu %u %s\n", (unsigned long)n++, it->len, out);
}
#endif

static void decode_task(void *arg)
{
    static int16_t pcm[2][MAX_FS];
    static sdu_t it;
    bool have[2] = { false, false };
    int fs_have = 0;

    for (;;) {
        if (xQueueReceive(s_sdu_q, &it, pdMS_TO_TICKS(200)) != pdTRUE) {
            have[0] = have[1] = false;
            continue;
        }
        /* Behind (more than 4 SDUs = 40 ms queued): drop the oldest so the
         * decoder never runs flat out and starves the idle task. */
        while (uxQueueMessagesWaiting(s_sdu_q) > 4 && xQueueReceive(s_sdu_q, &it, 0) == pdTRUE) {
            s_st.qdrops++;
        }
        xSemaphoreTake(s_dec_lock, portMAX_DELAY);
        struct sink_stream *ss = &s_snk[it.idx];
        int nch = ss->nch;
        if (!s_rx_on || nch <= 0 || ss->fs <= 0) {
            have[0] = have[1] = false;
            xSemaphoreGive(s_dec_lock);
            continue;
        }
        int fs = ss->fs;
#if PARTY_LC3_DUMP
        if (it.idx == 0) {
            dump_frame(&it);
        }
#endif
        for (int c = 0; c < nch; c++) {
            int side = nch >= 2 ? c : ss->side;
            if (have[side]) {
                /* the other side's frame never came: play this one on both */
                i2s_out(pcm[side], pcm[side], fs_have);
                have[0] = have[1] = false;
            }
            const uint8_t *in = (it.len >= (c + 1) * ss->opf) ? it.data + c * ss->opf : NULL;
            int r = ss->dec[c] ? lc3_decode(ss->dec[c], in, in ? ss->opf : 0,
                                            LC3_PCM_FORMAT_S16, pcm[side], 1) : -1;
            if (r < 0) {
                s_st.errors++;
                memset(pcm[side], 0, sizeof(int16_t) * fs);
            } else if (r == 1 || in == NULL) {
                s_st.plc++;
            } else {
                s_st.decoded++;
            }
            have[side] = true;
            fs_have = fs;
        }
        if (s_mono && have[0] != have[1]) {
            int s = have[0] ? 0 : 1;
            i2s_out(pcm[s], pcm[s], fs);
            have[0] = have[1] = false;
        } else if (have[0] && have[1]) {
            i2s_out(pcm[0], pcm[1], fs);
            have[0] = have[1] = false;
        }
        xSemaphoreGive(s_dec_lock);
    }
}

static bool rx_resources_init(void)
{
    static bool done;
    if (done) {
        return true;
    }
    /* Created on the first LISTEN only, so a speaker that always hosts pays
     * nothing. Queue storage in PSRAM; decoder stack internal (hot). */
    s_sdu_q = xQueueCreateWithCaps(SDU_Q_LEN, sizeof(sdu_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_sdu_q == NULL) {
        s_sdu_q = xQueueCreate(SDU_Q_LEN, sizeof(sdu_t));
    }
    /* Static internal stack: created late, a heap stack landed in RTC RAM
     * once internal RAM was short, and the decoder starved IDLE1 (task WDT). */
    static StackType_t s_dec_stack[8192 / sizeof(StackType_t)];
    static StaticTask_t s_dec_tcb;
    if (s_sdu_q == NULL ||
        xTaskCreateStaticPinnedToCore(decode_task, "lc3_dec", sizeof(s_dec_stack) / sizeof(StackType_t), NULL,
                                      configMAX_PRIORITIES - 8, s_dec_stack, &s_dec_tcb, 0) == NULL) {   /* core 0: DSP */
        ESP_LOGE(PTAG, "no memory for the LISTEN audio path");
        return false;
    }
    done = true;
    return true;
}

static void audio_out_start(uint32_t sr_hz)
{
    xSemaphoreTake(s_dec_lock, portMAX_DELAY);
    if (s_i2s == NULL) {
        i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
        cc.dma_desc_num = 6;
        cc.dma_frame_num = 240;
        cc.auto_clear_after_cb = true;      /* underrun -> silence, not a repeated buffer */
        i2s_std_config_t std = {
            .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(sr_hz),
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
            .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = I2S_BCLK_PIN,
                .ws   = I2S_WS_PIN,
                .dout = I2S_DOUT_PIN,
                .din  = I2S_GPIO_UNUSED,
            },
        };
        i2s_chan_handle_t h = NULL;
        esp_err_t err = i2s_new_channel(&cc, &h, NULL);
        if (err == ESP_OK) {
            err = i2s_channel_init_std_mode(h, &std);
        }
        if (err == ESP_OK) {
            err = i2s_channel_enable(h);
        }
        if (err != ESP_OK) {
            ESP_LOGE(PTAG, "I2S start failed: %s", esp_err_to_name(err));
            if (h != NULL) {
                i2s_del_channel(h);
            }
        } else {
            s_i2s = h;
            ESP_LOGI(PTAG, "I2S out: %lu Hz 32-bit stereo, BCLK=%d WS=%d DOUT=%d",
                     (unsigned long)sr_hz, I2S_BCLK_PIN, I2S_WS_PIN, I2S_DOUT_PIN);
        }
    }
    xSemaphoreGive(s_dec_lock);
}

/* Stop playing and free the decoders (after the sink streams have stopped). */
static void audio_out_stop(void)
{
    s_rx_on = false;
    xSemaphoreTake(s_dec_lock, portMAX_DELAY);
    if (s_i2s != NULL) {
        i2s_channel_disable(s_i2s);
        i2s_del_channel(s_i2s);
        s_i2s = NULL;
    }
    for (int i = 0; i < SNK_STREAMS; i++) {
        s_snk[i].nch = 0;
        for (int c = 0; c < 2; c++) {
            if (s_snk[i].dec_mem[c] != NULL) {
                free(s_snk[i].dec_mem[c]);
                s_snk[i].dec_mem[c] = NULL;
            }
            s_snk[i].dec[c] = NULL;
        }
    }
    if (s_sdu_q != NULL) {
        xQueueReset(s_sdu_q);
    }
    xSemaphoreGive(s_dec_lock);
}

/* ---- LISTEN: sink session ------------------------------------------------------ */

static void listen_teardown(bool stop_scan)
{
    s_want_cand = false;
    s_rx_on = false;

    if (s_sink != NULL) {
        if (s_lstate >= L_BIG_SYNCING || s_streams_started > 0) {
            xEventGroupClearBits(s_eg, EB_STREAMS_STOPPED);
            esp_err_t err = esp_ble_audio_bap_broadcast_sink_stop(s_sink);   /* BIG terminate (sync) */
            if (err == ESP_OK && s_streams_started > 0 &&
                !(xEventGroupWaitBits(s_eg, EB_STREAMS_STOPPED, pdTRUE, pdFALSE, pdMS_TO_TICKS(1500)) &
                  EB_STREAMS_STOPPED)) {
                ESP_LOGW(PTAG, "BIG sync stop: streams did not report stopped");
            }
        }
        esp_err_t err = ESP_FAIL;
        for (int tries = 0; tries < 5 && err != ESP_OK; tries++) {
            err = esp_ble_audio_bap_broadcast_sink_delete(s_sink);
            if (err != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }
        if (err != ESP_OK) {
            ESP_LOGE(PTAG, "broadcast sink delete failed (%s) - sink leaked", esp_err_to_name(err));
        }
        s_sink = NULL;
    }
    if (s_create_pending && s_lstate == L_PA_SYNCING) {
        esp_ble_gap_periodic_adv_sync_cancel();             /* answered by a PA_SYNC event, status 0x44 */
        s_create_t = xTaskGetTickCount();
    }
    if (s_pa_synced) {
        xEventGroupClearBits(s_eg, EB_PA_LOST);
        if (esp_ble_gap_periodic_adv_sync_terminate(s_pa_handle) == ESP_OK &&
            !(xEventGroupWaitBits(s_eg, EB_PA_LOST, pdTRUE, pdFALSE, pdMS_TO_TICKS(1000)) & EB_PA_LOST)) {
            ESP_LOGW(PTAG, "PA sync terminate: no sync-lost event");
        }
        s_pa_synced = false;
    }
    audio_out_stop();
    if (stop_scan) {
        scan_stop();
    }
    s_streams_started = 0;
    for (int i = 0; i < SNK_STREAMS; i++) {
        s_snk[i].started = false;
    }
    s_base_bits = 0;
    s_base_posted = false;
    s_syncable = s_encrypted = false;
    s_gen++;                                                 /* drop late events of this session */
    set_lstate(L_IDLE, "torn down");
}

static void listen_look(const char *why)
{
    set_party_info(PARTY_INFO_LOOKING);
    s_looking_log_t = xTaskGetTickCount();
    set_lstate(L_SCAN, why);
    if (!scan_start()) {
        ESP_LOGW(PTAG, "scan start failed - retrying");
    }
}

/* Something went wrong with the current master: drop it and look again. */
static void listen_restart(const char *why)
{
    ESP_LOGW(PTAG, "PARTY: LISTEN: leaving '%s' (%s)", s_cur.name, why);
    listen_teardown(true);
    listen_look(why);
}

static void start_pa_sync(const cand_t *c)
{
    esp_ble_gap_periodic_adv_sync_params_t p = {
        .filter_policy = 0,
        .sid = c->sid,
        .addr_type = c->addr_type,
        .skip = PA_SYNC_SKIP,
        .sync_timeout = PA_SYNC_TIMEOUT_10MS,
    };
    memcpy(p.addr, c->addr, 6);
    ESP_LOGW(PTAG, "PARTY: found JBL master '%s' Broadcast_ID 0x%06lx%s at %02x:%02x:%02x:%02x:%02x:%02x "
             "(sid %u, %d dBm) -> PA sync", c->name, (unsigned long)c->broadcast_id,
             c->harman ? " +Harman marker" : "", c->addr[0], c->addr[1], c->addr[2], c->addr[3],
             c->addr[4], c->addr[5], c->sid, c->rssi);
    esp_err_t err = esp_ble_gap_periodic_adv_create_sync(&p);
    if (err != ESP_OK) {
        ESP_LOGW(PTAG, "create sync failed: %s", esp_err_to_name(err));
        blacklist(c->addr, BLACKLIST_FAIL_MS);
        s_want_cand = true;
        return;
    }
    s_cur = *c;
    s_create_pending = true;
    s_create_t = xTaskGetTickCount();
    set_lstate(L_PA_SYNCING, "master found");
}

static void on_pa_synced(uint16_t handle)
{
    s_pa_handle = handle;
    s_pa_synced = true;
    /* BASE and BIGInfo now come over the PA sync: the scanner is not needed. */
    scan_stop();
    s_base_bits = 0;
    s_base_posted = false;
    s_syncable = s_encrypted = false;
    esp_ble_audio_bap_broadcast_sink_t *sink = NULL;
    esp_err_t err = esp_ble_audio_bap_broadcast_sink_create(handle, s_cur.broadcast_id, &sink);
    s_sink = sink;
    if (err != ESP_OK || sink == NULL) {
        ESP_LOGE(PTAG, "broadcast sink create failed: %s", esp_err_to_name(err));
        blacklist(s_cur.addr, BLACKLIST_FAIL_MS);
        listen_restart("sink create failed");
        return;
    }
    set_lstate(L_BASE_WAIT, "PA synced");
}

static void try_big_sync(void)
{
    if (!s_syncable || s_base_bits == 0) {
        return;
    }
    if (s_encrypted) {
        blacklist(s_cur.addr, BLACKLIST_ENC_MS);
        listen_restart("encrypted broadcast, no code");
        return;
    }
    uint32_t bits = 0;
    uint8_t n = 0;
    for (int b = 1; b < 32 && n < SNK_STREAMS; b++) {          /* lowest BIS indexes first */
        if (s_base_bits & (1u << b)) {
            bits |= 1u << b;
            n++;
        }
    }
    if (n == 0) {
        listen_restart("BASE has no usable BIS");
        return;
    }
    s_streams_wanted = n;
    s_streams_started = 0;
    s_mono = false;
    xEventGroupClearBits(s_eg, EB_STREAMS_STOPPED);
    uint8_t code[ESP_BLE_ISO_BROADCAST_CODE_SIZE] = {0};
    s_rx_on = true;                                            /* queue from the first SDU */
    esp_err_t err = esp_ble_audio_bap_broadcast_sink_sync(s_sink, bits, s_snk_p, code);
    if (err != ESP_OK) {
        ESP_LOGE(PTAG, "BIG sync request failed: %s", esp_err_to_name(err));
        listen_restart("BIG sync request failed");
        return;
    }
    ESP_LOGI(PTAG, "BIG sync: BIS mask 0x%08lx (BASE 0x%08lx), %u stream(s)",
             (unsigned long)bits, (unsigned long)s_base_bits, n);
    set_lstate(L_BIG_SYNCING, "BASE + BIGInfo");
}

static void enter_streaming(void)
{
    int total_ch = 0;
    for (int i = 0; i < SNK_STREAMS; i++) {
        if (s_snk[i].started) {
            total_ch += s_snk[i].nch > 0 ? s_snk[i].nch : 1;
        }
    }
    s_mono = total_ch < 2;
    s_last_sdu_t = xTaskGetTickCount();
    audio_out_start(s_snk[0].sr_hz ? s_snk[0].sr_hz : 48000);
    set_party_info(PARTY_INFO_SECONDARY);
    set_lstate(L_STREAMING, "all BIS synced");
    ESP_LOGW(PTAG, "PARTY: LISTEN synced to '%s' (Broadcast_ID 0x%06lx): %u BIS, %s -> I2S, %lu ms after LISTEN",
             s_cur.name, (unsigned long)s_cur.broadcast_id, s_streams_wanted, s_mono ? "mono" : "stereo",
             (unsigned long)ms_since(s_listen_t));
}

static bool addr_eq(const uint8_t *a, const uint8_t *b)
{
    return memcmp(a, b, 6) == 0;
}

/* PA sync established (or failed) - in any role. */
static void handle_pa_sync_evt(const pr_evt_t *ev)
{
    bool expected = s_role == ROLE_LISTEN && s_lstate == L_PA_SYNCING &&
                    addr_eq(ev->cand.addr, s_cur.addr) && ev->cand.sid == s_cur.sid;
    s_create_pending = false;
    if (expected && ev->status == 0) {
        ESP_LOGI(PTAG, "PA synced, handle 0x%04x (%lu ms after create)", ev->handle,
                 (unsigned long)ms_since(s_create_t));
        on_pa_synced(ev->handle);
    } else if (expected) {
        ESP_LOGW(PTAG, "PA sync failed, status 0x%02x", ev->status);
        blacklist(s_cur.addr, BLACKLIST_FAIL_MS);
        set_lstate(L_SCAN, "PA sync failed");
        if (!s_scan_on) {
            scan_start();
        }
    } else if (ev->status == 0) {
        /* A sync we no longer want (cancel raced with establishment). */
        ESP_LOGW(PTAG, "terminating unwanted PA sync 0x%04x", ev->handle);
        esp_ble_gap_periodic_adv_sync_terminate(ev->handle);
    }
}

static void listen_event(const pr_evt_t *ev)
{
    if (ev->type == EV_PA_SYNC) {
        handle_pa_sync_evt(ev);
        return;
    }
    if (ev->gen != s_gen) {
        return;                     /* from a torn-down session */
    }
    switch (ev->type) {
    case EV_CAND:
        /* not re-armed here while a create is pending: listen_poll does it */
        if (s_lstate == L_SCAN && !s_create_pending) {
            start_pa_sync(&ev->cand);
        }
        break;
    case EV_PA_LOST:
        if (s_pa_synced && ev->handle == s_pa_handle) {
            s_pa_synced = false;
            listen_restart("PA sync lost");
        }
        break;
    case EV_BASE:
        if (s_lstate == L_BASE_WAIT) {
            ESP_LOGI(PTAG, "BASE received: BIS indexes 0x%08lx", (unsigned long)ev->val);
            try_big_sync();
        }
        break;
    case EV_SYNCABLE:
        if (s_lstate == L_BASE_WAIT) {
            s_syncable = true;
            s_encrypted = ev->val != 0;
            try_big_sync();
        }
        break;
    case EV_STREAM_STARTED:
        if (s_lstate == L_BIG_SYNCING && s_streams_started >= s_streams_wanted) {
            enter_streaming();
        }
        break;
    case EV_STREAM_STOPPED:
    case EV_SINK_STOPPED:
        if (s_lstate == L_BIG_SYNCING || s_lstate == L_STREAMING) {
            char why[40];
            snprintf(why, sizeof(why), "BIG sync %s, reason 0x%02x",
                     s_lstate == L_STREAMING ? "lost" : "failed", ev->status);
            listen_restart(why);
        }
        break;
    default:
        break;
    }
}

static void listen_poll(void)
{
    switch (s_lstate) {
    case L_SCAN:
        if (!s_scan_on && elapsed(s_scan_try_t, 2000)) {
            scan_start();
        }
        if (s_create_pending && elapsed(s_create_t, T_CANCEL_MS)) {
            s_create_pending = false;               /* cancel never answered */
        }
        if (!s_create_pending && !s_want_cand) {
            s_want_cand = true;                     /* ready for the next candidate */
        }
        if (elapsed(s_looking_log_t, LOOKING_LOG_MS)) {
            s_looking_log_t = xTaskGetTickCount();
            ESP_LOGW(PTAG, "PARTY: LISTEN - no JBL master found yet (%lu s), still looking",
                     (unsigned long)(ms_since(s_lstate_t) / 1000));
        }
        break;
    case L_PA_SYNCING:
        if (elapsed(s_lstate_t, T_PA_SYNC_MS)) {
            ESP_LOGW(PTAG, "PA sync to '%s' timed out - cancelling", s_cur.name);
            esp_ble_gap_periodic_adv_sync_cancel();
            s_create_t = xTaskGetTickCount();       /* still pending until its event */
            blacklist(s_cur.addr, BLACKLIST_FAIL_MS);
            set_lstate(L_SCAN, "PA sync timeout");
        }
        break;
    case L_BASE_WAIT:
        if (elapsed(s_lstate_t, T_BASE_MS)) {
            blacklist(s_cur.addr, BLACKLIST_FAIL_MS);
            listen_restart(s_base_bits ? "no BIGInfo in 5 s" : "no BASE in 5 s");
        }
        break;
    case L_BIG_SYNCING:
        if (elapsed(s_lstate_t, T_BIG_MS)) {
            blacklist(s_cur.addr, BLACKLIST_FAIL_MS);
            listen_restart("BIG sync timeout");
        }
        break;
    case L_STREAMING:
        if (elapsed(s_last_sdu_t, T_NO_SDU_MS)) {
            listen_restart("no audio SDUs for 3 s");
        }
        break;
    default:
        break;
    }
}

/* ---- role transitions --------------------------------------------------------- */

static void go_listen(const char *why)
{
    TickType_t t0 = xTaskGetTickCount();
    set_role(ROLE_HOST_STOPPING, why);
    if (s_ops->host_stop() != 0) {
        ESP_LOGW(PTAG, "host stop reported an error (continuing)");
    }
    s_host_up = false;
    jbl_assistant_set_paused(true);
    for (int i = 0; i < ASSIST_QUIET_MS / 50 && !jbl_assistant_scan_idle(); i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!rx_resources_init()) {
        ESP_LOGE(PTAG, "LISTEN without audio output");
    }
    s_gen++;
    s_listen_t = xTaskGetTickCount();
    set_role(ROLE_LISTEN, "broadcast stopped");
    ESP_LOGW(PTAG, "PARTY: HOST stopped in %lu ms", (unsigned long)ms_since(t0));
    listen_look("LISTEN entered");
}

static void go_host(const char *why)
{
    TickType_t t0 = xTaskGetTickCount();
    set_role(ROLE_LISTEN_STOPPING, why);
    listen_teardown(true);                              /* also bumps s_gen */
    set_role(ROLE_HOST_STARTING, "left the other broadcast");
    /* Twice the controller asserted (ble/56.o, inside HCI LE Create BIG Test)
     * when the BIG was created within a few ms of the phone's AVDTP start and
     * the LE scan stop; the same switch without the phone link never failed.
     * Let the A2DP stream and the scan teardown settle first. */
    vTaskDelay(pdMS_TO_TICKS(PARTY_HOST_SETTLE_MS));
    s_host_try_t = xTaskGetTickCount();
    int err = s_ops->host_start();
    s_host_up = (err == 0);
    set_role(ROLE_HOST, s_host_up ? "broadcast source started" : "broadcast start FAILED, will retry");
    set_party_info(PARTY_INFO_HOST);
    jbl_assistant_set_paused(false);
    ESP_LOGW(PTAG, "PARTY: LISTEN -> HOST took %lu ms", (unsigned long)ms_since(t0));
}

static void party_task(void *arg)
{
    TickType_t last_audio = xTaskGetTickCount();
    pr_evt_t ev;

    for (;;) {
        bool got = xQueueReceive(s_q, &ev, pdMS_TO_TICKS(100)) == pdTRUE;
        bool streaming = a2dp_bridge_streaming();
        if (streaming) {
            last_audio = xTaskGetTickCount();
        }

        switch (s_role) {
        case ROLE_HOST:
            if (got && ev.type == EV_PA_SYNC) {
                handle_pa_sync_evt(&ev);                /* orphan from a cancelled create */
            }
            if (got && ev.type == EV_SRC_STOPPED && ev.gen == s_gen && s_host_up) {
                ESP_LOGW(PTAG, "PARTY: HOST - broadcast source stopped unexpectedly, restarting it");
                s_host_up = false;
                s_host_try_t = 0;
            }
            if (!streaming && elapsed(last_audio, PARTY_HOLDOFF_MS)) {
                go_listen(a2dp_bridge_connected() ? "phone paused, no A2DP audio for 3 s"
                                                  : "no phone audio for 3 s");
                last_audio = xTaskGetTickCount();
            } else if (!s_host_up && elapsed(s_host_try_t, 2000)) {
                s_host_try_t = xTaskGetTickCount();
                s_ops->host_stop();                     /* clean slate */
                s_gen++;                                /* its stopped callback is not "unexpected" */
                s_host_up = s_ops->host_start() == 0;
                ESP_LOGW(PTAG, "PARTY: HOST broadcast restart %s", s_host_up ? "OK" : "FAILED");
            }
            break;
        case ROLE_LISTEN:
            if (streaming) {
                go_host("phone A2DP audio started");
                last_audio = xTaskGetTickCount();
                break;
            }
            if (got) {
                listen_event(&ev);
            }
            if (s_role == ROLE_LISTEN) {
                listen_poll();
            }
            break;
        default:
            break;                                      /* transitions run inline above */
        }
    }
}

/* ---- public ------------------------------------------------------------------- */

esp_err_t party_role_init(void)
{
    static uint8_t codec_data[] = ESP_BLE_AUDIO_CODEC_CAP_LC3_DATA(
        ESP_BLE_AUDIO_CODEC_CAP_FREQ_16KHZ | ESP_BLE_AUDIO_CODEC_CAP_FREQ_24KHZ |
        ESP_BLE_AUDIO_CODEC_CAP_FREQ_48KHZ,
        ESP_BLE_AUDIO_CODEC_CAP_DURATION_7_5 | ESP_BLE_AUDIO_CODEC_CAP_DURATION_10,
        ESP_BLE_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(1, 2),
        26, 155, 1);
    static uint8_t codec_meta[] = ESP_BLE_AUDIO_CODEC_CAP_LC3_META(
        ESP_BLE_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | ESP_BLE_AUDIO_CONTEXT_TYPE_MEDIA);
    static const esp_ble_audio_codec_cap_t codec_cap = ESP_BLE_AUDIO_CODEC_CAP_LC3(codec_data, codec_meta);
    static esp_ble_audio_pacs_cap_t cap = { .codec_cap = &codec_cap };
    const esp_ble_audio_pacs_register_param_t pacs_param = { .snk_pac = true, .snk_loc = true };
    esp_err_t err;

    s_dec_lock = xSemaphoreCreateMutex();
    if (s_dec_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }
    err = esp_ble_audio_pacs_register(&pacs_param);
    if (err) {
        ESP_LOGE(PTAG, "PACS register failed, err %d", err);
        return err;
    }
    for (int i = 0; i < SNK_STREAMS; i++) {
        esp_ble_audio_bap_stream_cb_register(&s_snk[i].stream, &s_snk_ops);
        s_snk_p[i] = &s_snk[i].stream;
    }
    err = esp_ble_audio_pacs_cap_register(ESP_BLE_AUDIO_DIR_SINK, &cap);
    if (err) {
        ESP_LOGE(PTAG, "PACS sink capability register failed, err %d", err);
        return err;
    }
    err = esp_ble_audio_bap_scan_delegator_register(&s_deleg_cbs);
    if (err) {
        ESP_LOGE(PTAG, "scan delegator register failed, err %d", err);
        return err;
    }
    err = esp_ble_audio_bap_broadcast_sink_register_cb(&s_sink_cbs);
    if (err) {
        ESP_LOGE(PTAG, "broadcast sink callbacks register failed, err %d", err);
        return err;
    }
    ESP_LOGI(PTAG, "broadcast sink side ready (%d BIS max)", SNK_STREAMS);
    return ESP_OK;
}

esp_err_t party_role_start(const party_role_ops_t *ops, uint16_t own_mac_crc, bool hosting)
{
    s_ops = ops;
    s_own_crc = own_mac_crc;
    memcpy(s_own_addr, esp_bt_dev_get_address(), 6);
    s_host_up = hosting;
    s_role = ROLE_HOST;
    if (!PARTY_AUTO_SWITCH) {
        ESP_LOGW(PTAG, "PARTY: auto role switch disabled - always HOST");
        return ESP_OK;
    }
    s_eg = xEventGroupCreate();
    s_q = xQueueCreate(16, sizeof(pr_evt_t));
    if (s_eg == NULL || s_q == NULL) {
        return ESP_ERR_NO_MEM;
    }
    /* Core 1, PSRAM stack (state machine + Bluedroid/LE Audio/I2S driver calls,
     * never NVS/flash). Same priority as jbl_pa / jbl_assist: it mostly blocks. */
    app_task_cold("party", party_task, 6144, 5);
    ESP_LOGW(PTAG, "PARTY: role switching on - HOST while the phone streams, LISTEN to other JBL masters "
             "after %d s without phone audio", PARTY_HOLDOFF_MS / 1000);
    return ESP_OK;
}

void party_role_on_audio_state(bool streaming)
{
    if (streaming) {
        post(EV_WAKE, 0, 0, 1);     /* LISTEN -> HOST without waiting for the 100 ms poll */
    }
}

void party_role_on_source_started(void)
{
    s_src_running = true;
}

void party_role_on_source_stopped(void)
{
    bool was = s_src_running;
    s_src_running = false;
    if (was && s_role == ROLE_HOST) {
        post(EV_SRC_STOPPED, 0, 0, 0);
    }
}

bool party_role_is_hosting(void)
{
    role_t r = s_role;
    return r == ROLE_HOST || r == ROLE_HOST_STARTING;
}

void party_role_log_stats(void)
{
    uint32_t sdus = s_st.sdus, lost = s_st.lost, qd = s_st.qdrops, dec = s_st.decoded,
             plc = s_st.plc, errs = s_st.errors, late = s_st.i2s_late;
    s_st.sdus = s_st.lost = s_st.qdrops = s_st.decoded = s_st.plc = s_st.errors = s_st.i2s_late = 0;
    if (s_role != ROLE_LISTEN) {
        ESP_LOGW(PTAG, "PARTY 2s: %s", role_name[s_role]);
        return;
    }
    if (s_lstate != L_STREAMING) {
        ESP_LOGW(PTAG, "PARTY 2s: LISTEN, %s%s%s%s", lstate_name[s_lstate],
                 s_lstate > L_SCAN ? " '" : "", s_lstate > L_SCAN ? s_cur.name : "",
                 s_lstate > L_SCAN ? "'" : "");
        return;
    }
    ESP_LOGW(PTAG, "PARTY 2s: LISTEN to '%s' | SDUs %lu (expect 400), lost %lu, LC3 decoded %lu, PLC %lu, "
             "errors %lu, queue drops %lu, I2S late %lu", s_cur.name, (unsigned long)sdus, (unsigned long)lost,
             (unsigned long)dec, (unsigned long)plc, (unsigned long)errs, (unsigned long)qd, (unsigned long)late);
}
