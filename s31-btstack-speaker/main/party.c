/*
 * JBL party role switching on BTstack (see party.h).
 *
 * Role (BT thread, 100 ms tick):
 *   HOST --(no A2DP audio for 1.5 s)--> HOST_STOPPING --(Auracast OFF)--> LISTEN
 *   LISTEN --(A2DP audio)--> LISTEN_STOPPING --(sync torn down)--> HOST
 *   HOST_STOPPING --(audio again)--> HOST
 *
 * LISTEN (BT thread):
 *   SCAN --(JBL master ext adv)--> PA_SYNC --(synced)--> W4_BASE (BASE + BIGInfo)
 *     --> BIG_SYNC --(created)--> STREAM ; any timeout / loss --> back to SCAN
 *
 * Audio: ISO SDUs (Auracast forwards non-own ISO data here, BT thread) ->
 * queue -> "party_dec" task on core 0: liblc3 decode (PLC for lost SDUs) ->
 * audio_out (shared I2S line, AO_LISTEN), delayed by the master's presentation
 * delay so the S31 plays with the master and its other party speakers.
 *
 * Auracast button off: no hosting, no listening - phone audio plays locally only.
 * Needs idf-patches/0010: received ISO data reaches the host at all.
 */
#include "party.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "btstack.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_memory_utils.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lc3.h"

#include "a2dp_in.h"
#include "audio_out.h"
#include "auracast.h"
#include "jbl.h"

static const char *TAG = "PARTY";

/* ---- tuning ---------------------------------------------------------------- */
#define PARTY_IDLE_TO_LISTEN_MS 1500
#define TICK_MS                 100
#define SCAN_ITVL               240      /* 150 ms */
#define SCAN_WIN                48       /* 30 ms: longer windows starved BR/EDR paging (Bluedroid build) */
#define PA_SYNC_TIMEOUT_10MS    200      /* 2 s supervision */
#define T_PA_SYNC_MS            5000
#define T_BASE_MS               5000
#define T_BIG_MS                5000
#define T_NO_SDU_MS             3000
#define T_TEARDOWN_MS           1500
#define BLACKLIST_MS            5000
#define BIG_SYNC_HANDLE         0        /* our own BIG is terminated before we sync */

#define JBL_BROADCAST_ID        0x112233u
#define HARMAN_COMPANY_ID       0x0057u

#define LISTEN_RX_LAG_MS        5        /* ISO RX arrives ~ at the SDU reference; decode is ~1 ms */

#define MAX_BIS                 2
#define MAX_FS                  480      /* 10 ms @ 48 kHz */
#define MAX_SDU                 320      /* 2 channels x up to 155 octets in one BIS */
#define SDU_Q_LEN               48       /* 120 ms of 2-BIS SDUs: bursts never overflow */

/* ---- state ------------------------------------------------------------------ */
typedef enum { R_HOST, R_HOST_STOPPING, R_LISTEN, R_LISTEN_STOPPING, R_OFF } role_t;
typedef enum { L_OFF, L_SCAN, L_PA_SYNC, L_W4_BASE, L_BIG_SYNC, L_STREAM } lstate_t;
static const char *const role_names[] = { "HOST", "HOST_STOPPING", "LISTEN", "LISTEN_STOPPING", "OFF (Auracast off)" };
#ifndef PARTY_BOOT_AURACAST
#define PARTY_BOOT_AURACAST 0   /* broadcast off at power-up; the Auracast button (GPIO 36) turns it on/off */
#endif
static volatile bool s_enabled = PARTY_BOOT_AURACAST;   /* Auracast button */
static const char *const lstate_names[] = { "off", "scanning", "PA syncing", "waiting for BASE", "BIG syncing", "STREAMING" };

static role_t s_role = R_HOST;
static lstate_t s_ls = L_OFF;
static uint32_t s_idle_ms, s_state_ms, s_role_ms;
static btstack_timer_source_t s_tick;
static btstack_packet_callback_registration_t s_hci_reg;

/* candidate / sync */
static bd_addr_t s_cand_addr;
static bd_addr_type_t s_cand_type;
static uint8_t s_cand_sid;
static char s_cand_name[33];
static hci_con_handle_t s_pa_handle = HCI_CON_HANDLE_INVALID;
static bool s_pa_pending, s_big_active, s_big_pending;
static le_audio_big_sync_t s_big;
static le_audio_big_sync_params_t s_big_params;
static hci_con_handle_t s_bis[MAX_BIS];
static uint8_t s_num_bis;
static bool s_have_base, s_have_biginfo, s_biginfo_enc;
static struct { bd_addr_t a; uint32_t until; } s_black[4];
/* A phone paging us: the LE scan (20 % duty) starves the LMP setup / role
 * switch and the controller drops the link (0x16). Hold the scan meanwhile. */
#define CONN_SETUP_HOLD_MS 4000
static uint32_t s_scan_hold_until;
static bool s_scan_held;
static bool s_boot_hold;                         /* boot: scan waits for the phone */

/* BASE */
typedef struct {
    uint32_t sr_hz;
    uint32_t pd_us;        /* presentation delay */
    uint16_t dt_us;
    uint16_t opf;          /* octets per channel per frame */
    uint8_t nbis;
    uint8_t bis_index[MAX_BIS];
    uint8_t nch[MAX_BIS];  /* channels per BIS (from Audio_Channel_Allocation) */
    uint8_t side[MAX_BIS]; /* 0 left / 1 right when nch == 1 */
} base_cfg_t;
static base_cfg_t s_base;

/* audio path */
typedef struct {
    uint8_t idx;
    uint16_t seq;
    uint16_t len;          /* 0 = lost (PLC) */
    uint8_t data[MAX_SDU];
} sdu_t;
static QueueHandle_t s_sdu_q;
static SemaphoreHandle_t s_dec_lock;
static lc3_decoder_t s_dec[MAX_BIS][2];
static void *s_dec_mem[MAX_BIS][2];
static volatile bool s_rx_on;
static base_cfg_t s_dcfg;              /* decoder config (copy under s_dec_lock) */

static struct {
    uint32_t rx[MAX_BIS], lost[MAX_BIS], gaps[MAX_BIS];
    uint32_t decoded, plc, errors, qdrops, i2s_late;
    uint32_t switches_listen, switches_host, syncs;
} s_st;
static uint16_t s_last_seq[MAX_BIS];
static bool s_have_seq[MAX_BIS];
static uint32_t s_last_sdu_ms;

/* ---- helpers ---------------------------------------------------------------- */
static uint32_t now_ms(void) { return btstack_run_loop_get_time_ms(); }

static void set_role(role_t r, const char *why)
{
    if (r != s_role) {
        ESP_LOGW(TAG, "role %s -> %s (%s)", role_names[s_role], role_names[r], why);
        s_role = r;
        s_role_ms = now_ms();
    }
}

static void set_ls(lstate_t l, const char *why)
{
    if (l != s_ls) {
        ESP_LOGW(TAG, "listen: %s -> %s (%s)", lstate_names[s_ls], lstate_names[l], why);
        s_ls = l;
        s_state_ms = now_ms();
    }
    jbl_set_party_info(l == L_STREAM ? JBL_PARTY_INFO_SECONDARY : JBL_PARTY_INFO_LOOKING);
}

static bool blacklisted(const bd_addr_t a)
{
    uint32_t t = now_ms();
    for (unsigned i = 0; i < sizeof(s_black) / sizeof(s_black[0]); i++) {
        if (bd_addr_cmp(s_black[i].a, a) == 0 && (int32_t)(s_black[i].until - t) > 0) {
            return true;
        }
    }
    return false;
}

static void blacklist(const bd_addr_t a, uint32_t ms)
{
    static unsigned next;
    bd_addr_copy(s_black[next].a, a);
    s_black[next].until = now_ms() + ms;
    next = (next + 1) % (sizeof(s_black) / sizeof(s_black[0]));
}

/* ---- audio out (decode task, core 0) -------------------------------------------- */
static void i2s_out(const int16_t *l, const int16_t *r, int fs)
{
    static int16_t lr[MAX_FS * 2];
    for (int i = 0; i < fs; i++) {
        lr[2 * i] = l[i];
        lr[2 * i + 1] = r[i];
    }
    audio_out_push(AO_LISTEN, lr, fs);
}

/* Decoded frames per output side, paired by SDU sequence number - never by
 * arrival order: with BN 2 an event delivers BIS1 k, BIS1 k+1, BIS2 k, BIS2 k+1. */
#define SIDE_FIFO 6
typedef struct {
    int16_t pcm[SIDE_FIFO][MAX_FS];
    uint16_t seq[SIDE_FIFO];
    uint8_t head, n;
} side_fifo_t;
static side_fifo_t *s_side;   /* [2], PSRAM, allocated at BIG sync (lazy, as the codec fork) */
static volatile uint32_t st_dec_us, st_dec_max_us, st_dec_n, st_mono_fill, st_seq_drop;

static void fifo_reset(void)
{
    if (s_side == NULL) return;
    s_side[0].head = s_side[0].n = 0;
    s_side[1].head = s_side[1].n = 0;
}

static int16_t *fifo_slot_in(side_fifo_t *f, uint16_t seq)
{
    if (f->n == SIDE_FIFO) {                       /* other side gone for a while: oldest out */
        f->head = (uint8_t)((f->head + 1) % SIDE_FIFO);
        f->n--;
        st_seq_drop++;
    }
    uint8_t i = (uint8_t)((f->head + f->n) % SIDE_FIFO);
    f->seq[i] = seq;
    f->n++;
    return f->pcm[i];
}

static void fifo_pop(side_fifo_t *f)
{
    f->head = (uint8_t)((f->head + 1) % SIDE_FIFO);
    f->n--;
}

static void pairs_out(int fs, bool mono_stream)
{
    side_fifo_t *l = &s_side[0], *r = &s_side[1];
    if (mono_stream) {
        side_fifo_t *f = l->n ? l : r;
        while (f->n) {
            i2s_out(f->pcm[f->head], f->pcm[f->head], fs);
            fifo_pop(f);
        }
        return;
    }
    for (;;) {
        if (l->n && r->n) {
            int16_t d = (int16_t)(l->seq[l->head] - r->seq[r->head]);
            if (d == 0) {
                i2s_out(l->pcm[l->head], r->pcm[r->head], fs);
                fifo_pop(l);
                fifo_pop(r);
            } else {                               /* partner of the older frame never came */
                side_fifo_t *o = d < 0 ? l : r;
                i2s_out(o->pcm[o->head], o->pcm[o->head], fs);
                fifo_pop(o);
                st_mono_fill++;
            }
        } else if (l->n > 3 || r->n > 3) {         /* one BIS silent: play the other on both */
            side_fifo_t *o = l->n ? l : r;
            i2s_out(o->pcm[o->head], o->pcm[o->head], fs);
            fifo_pop(o);
            st_mono_fill++;
        } else {
            break;
        }
    }
}

static void decode_task(void *arg)
{
    (void)arg;
    static sdu_t it;
    for (;;) {
        if (xQueueReceive(s_sdu_q, &it, pdMS_TO_TICKS(200)) != pdTRUE) {
            xSemaphoreTake(s_dec_lock, portMAX_DELAY);
            fifo_reset();
            xSemaphoreGive(s_dec_lock);
            continue;
        }
        xSemaphoreTake(s_dec_lock, portMAX_DELAY);
        if (!s_rx_on || s_side == NULL || it.idx >= s_dcfg.nbis) {
            fifo_reset();
            xSemaphoreGive(s_dec_lock);
            continue;
        }
        int fs = (int)(s_dcfg.sr_hz / 1000u * s_dcfg.dt_us / 1000u);
        int nch = s_dcfg.nch[it.idx];
        for (int c = 0; c < nch; c++) {
            int side = nch >= 2 ? c : s_dcfg.side[it.idx];
            int16_t *out = fifo_slot_in(&s_side[side], it.seq);
            const uint8_t *in = it.len >= (c + 1) * s_dcfg.opf ? it.data + c * s_dcfg.opf : NULL;
            int64_t t0 = esp_timer_get_time();
            int r = s_dec[it.idx][c] ? lc3_decode(s_dec[it.idx][c], in, in ? s_dcfg.opf : 0,
                                                  LC3_PCM_FORMAT_S16, out, 1) : -1;
            uint32_t us = (uint32_t)(esp_timer_get_time() - t0);
            st_dec_us += us;
            st_dec_n++;
            if (us > st_dec_max_us) st_dec_max_us = us;
            if (r < 0) {
                s_st.errors++;
                memset(out, 0, sizeof(int16_t) * fs);
            } else if (r == 1 || in == NULL) {
                s_st.plc++;
            } else {
                s_st.decoded++;
            }
        }
        pairs_out(fs, s_dcfg.nbis == 1 && s_dcfg.nch[0] == 1);
        xSemaphoreGive(s_dec_lock);
    }
}

static bool audio_start(void)
{
    xSemaphoreTake(s_dec_lock, portMAX_DELAY);
    s_dcfg = s_base;
    bool ok = true;
    if (s_side == NULL) {
        s_side = heap_caps_calloc(2, sizeof(side_fifo_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        ok = s_side != NULL;
    }
    /* decoders in the shared LC3 workspace (the encoders' memory while hosting) */
    size_t half = 0;
    uint8_t *ws = ok ? auracast_lc3_workspace_take(&half) : NULL;
    int inst = 0;
    ok = ok && ws != NULL;
    for (int b = 0; b < s_dcfg.nbis && ok; b++) {
        for (int c = 0; c < s_dcfg.nch[b]; c++) {
            unsigned sz = lc3_decoder_size(s_dcfg.dt_us, s_dcfg.sr_hz);
            if (inst >= 2 || sz > half) {
                ESP_LOGE(TAG, "LC3 workspace too small: %u B decoder #%d (slot %u B)", sz, inst, (unsigned)half);
                ok = false;
                break;
            }
            s_dec_mem[b][c] = ws + (size_t)inst++ * half;
            s_dec[b][c] = lc3_setup_decoder(s_dcfg.dt_us, s_dcfg.sr_hz, 0, s_dec_mem[b][c]);
            if (s_dec[b][c] == NULL) {
                ok = false;
                break;
            }
        }
    }
    ESP_LOGW(TAG, "LC3 decoders %s in the shared workspace %p", ok ? "set up" : "FAILED", ws);
    if (ok) {
        uint32_t d = s_dcfg.pd_us / 1000u;
        d = d > LISTEN_RX_LAG_MS + 20 ? d - LISTEN_RX_LAG_MS : 20;
        audio_out_set_rate(s_dcfg.sr_hz);
        audio_out_begin(AO_LISTEN, d);
    }
    xQueueReset(s_sdu_q);
    fifo_reset();
    s_rx_on = ok;
    xSemaphoreGive(s_dec_lock);
    ESP_LOGW(TAG, "audio out %s: %lu Hz, %u us frames, %u B/ch, %u BIS (%u+%u ch), presentation delay %lu us",
             ok ? "ON" : "FAILED", (unsigned long)s_dcfg.sr_hz, s_dcfg.dt_us, s_dcfg.opf, s_dcfg.nbis,
             s_dcfg.nch[0], s_dcfg.nbis > 1 ? s_dcfg.nch[1] : 0, (unsigned long)s_dcfg.pd_us);
    return ok;
}

static void audio_stop(void)
{
    s_rx_on = false;
    xSemaphoreTake(s_dec_lock, portMAX_DELAY);
    audio_out_end(AO_LISTEN);
    audio_out_set_rate(48000);
    heap_caps_free(s_side);
    s_side = NULL;
    for (int b = 0; b < MAX_BIS; b++) {
        for (int c = 0; c < 2; c++) {
            s_dec_mem[b][c] = NULL;              /* shared workspace: not ours to free */
            s_dec[b][c] = NULL;
        }
    }
    xQueueReset(s_sdu_q);
    xSemaphoreGive(s_dec_lock);
}

/* ---- ISO RX (BT thread, forwarded by auracast.c) ------------------------------- */
static void iso_rx(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    if (packet_type != HCI_ISO_DATA_PACKET || size < 4 || s_ls != L_STREAM) {
        return;
    }
    uint16_t hdr = little_endian_read_16(packet, 0);
    hci_con_handle_t h = hdr & 0x0fff;
    int b = -1;
    for (int i = 0; i < s_num_bis; i++) {
        if (s_bis[i] == h) b = i;
    }
    if (b < 0) {
        return;
    }
    uint16_t off = 4 + (((hdr >> 14) & 1) ? 4 : 0);
    if (size < off + 4) {
        return;
    }
    static sdu_t it;
    it.idx = (uint8_t)b;
    it.seq = little_endian_read_16(packet, off);
    uint16_t lp = little_endian_read_16(packet, off + 2);
    uint16_t len = lp & 0x0fff;
    uint8_t ps = (uint8_t)(lp >> 14);
    off += 4;
    if (ps != 0 || len == 0 || size < off + len || len > MAX_SDU) {
        it.len = 0;
        s_st.lost[b]++;
    } else {
        it.len = len;
        memcpy(it.data, packet + off, len);
        s_st.rx[b]++;
    }
    if (s_have_seq[b] && (uint16_t)(it.seq - s_last_seq[b]) != 1) {
        s_st.gaps[b]++;
    }
    s_last_seq[b] = it.seq;
    s_have_seq[b] = true;
    s_last_sdu_ms = now_ms();
    if (xQueueSend(s_sdu_q, &it, 0) != pdTRUE) {
        sdu_t junk;
        xQueueReceive(s_sdu_q, &junk, 0);              /* drop the OLDEST, keep the newest */
        xQueueSend(s_sdu_q, &it, 0);
        s_st.qdrops++;
    }
}

/* ---- BASE parsing --------------------------------------------------------------- */
static bool parse_base(const uint8_t *d, uint16_t n, base_cfg_t *cfg)
{
    /* PA AD structures -> Service Data 0x1851 (Basic Audio Announcement) */
    ad_context_t c;
    for (ad_iterator_init(&c, n, d); ad_iterator_has_more(&c); ad_iterator_next(&c)) {
        if (ad_iterator_get_data_type(&c) != BLUETOOTH_DATA_TYPE_SERVICE_DATA_16_BIT_UUID) continue;
        const uint8_t *v = ad_iterator_get_data(&c);
        uint8_t vl = ad_iterator_get_data_len(&c);
        if (vl < 2 || little_endian_read_16(v, 0) != 0x1851) continue;
        const uint8_t *p = v + 2, *e = v + vl;
        if (e - p < 4) return false;
        uint32_t pd = little_endian_read_24(p, 0);    /* presentation delay */
        p += 3;
        uint8_t nsub = *p++;
        memset(cfg, 0, sizeof(*cfg));
        cfg->sr_hz = 48000;
        cfg->dt_us = 10000;
        cfg->pd_us = pd;
        for (uint8_t s = 0; s < nsub && p < e; s++) {
            if (e - p < 7) return false;
            uint8_t nb = *p++;
            p += 5;                                    /* codec id */
            uint8_t cl = *p++;
            const uint8_t *cc = p;
            p += cl;
            uint32_t sub_loc = 0;
            for (const uint8_t *q = cc; q + 1 < cc + cl && q + 1 + q[0] <= cc + cl; q += q[0] + 1) {
                uint8_t t = q[1];
                if (t == 0x01 && q[0] >= 2) {
                    static const uint32_t rates[] = { 0, 8000, 11025, 16000, 22050, 24000, 32000, 44100, 48000 };
                    if (q[2] < 9) cfg->sr_hz = rates[q[2]];
                } else if (t == 0x02 && q[0] >= 2) {
                    cfg->dt_us = q[2] == 0 ? 7500 : 10000;
                } else if (t == 0x03 && q[0] >= 5) {
                    sub_loc = little_endian_read_32(q, 2);
                } else if (t == 0x04 && q[0] >= 3) {
                    cfg->opf = little_endian_read_16(q, 2);
                }
            }
            if (p >= e) return false;
            uint8_t ml = *p++;
            p += ml;
            for (uint8_t b = 0; b < nb && p + 2 <= e; b++) {
                uint8_t idx = *p++;
                uint8_t bl = *p++;
                uint32_t loc = sub_loc;
                for (const uint8_t *q = p; q + 1 < p + bl && q + 1 + q[0] <= p + bl; q += q[0] + 1) {
                    if (q[1] == 0x03 && q[0] >= 5) loc = little_endian_read_32(q, 2);
                }
                p += bl;
                if (cfg->nbis < MAX_BIS) {
                    int i = cfg->nbis++;
                    cfg->bis_index[i] = idx;
                    int bits = __builtin_popcount(loc);
                    cfg->nch[i] = bits >= 2 ? 2 : 1;
                    cfg->side[i] = (loc & 0x2) && !(loc & 0x1) ? 1 : (loc == 0 ? (uint8_t)i : 0);
                }
            }
        }
        return cfg->nbis > 0 && cfg->opf > 0 && cfg->opf * 2 <= MAX_SDU;
    }
    return false;
}

/* ---- LISTEN control (BT thread) ------------------------------------------------ */
static void scan_start(void)
{
    if ((int32_t)(s_scan_hold_until - now_ms()) > 0) {
        s_scan_held = true;                           /* resumed by listen_poll */
        return;
    }
    s_scan_held = false;
    gap_set_scan_params(0 /* passive */, SCAN_ITVL, SCAN_WIN, 0);
    gap_start_scan();
}

static void listen_reset(const char *why)
{
    if (s_big_active || s_big_pending) {
        gap_big_sync_terminate(BIG_SYNC_HANDLE);
        s_big_active = s_big_pending = false;
    }
    if (s_pa_handle != HCI_CON_HANDLE_INVALID) {
        gap_periodic_advertising_terminate_sync(s_pa_handle);
        s_pa_handle = HCI_CON_HANDLE_INVALID;
    }
    if (s_pa_pending) {
        gap_periodic_advertising_create_sync_cancel();
        s_pa_pending = false;
    }
    if (s_rx_on) {
        audio_stop();
    }
    s_have_base = s_have_biginfo = s_biginfo_enc = false;
    s_num_bis = 0;
    memset(s_have_seq, 0, sizeof(s_have_seq));
    if (s_role == R_LISTEN) {
        scan_start();
        set_ls(L_SCAN, why);
    } else {
        gap_stop_scan();
        set_ls(L_OFF, why);
    }
}

static void try_big_sync(void)
{
    if (!s_have_base || !s_have_biginfo || s_big_pending || s_big_active) {
        return;
    }
    if (s_biginfo_enc) {
        ESP_LOGW(TAG, "'%s' is encrypted - skipping it", s_cand_name);
        blacklist(s_cand_addr, 60000);
        listen_reset("encrypted broadcast");
        return;
    }
    memset(&s_big_params, 0, sizeof(s_big_params));
    s_big_params.big_handle = BIG_SYNC_HANDLE;
    s_big_params.sync_handle = s_pa_handle;
    s_big_params.big_sync_timeout_10ms = 200;
    s_big_params.num_bis = s_base.nbis;
    for (int i = 0; i < s_base.nbis; i++) {
        s_big_params.bis_indices[i] = s_base.bis_index[i];
    }
    uint8_t rc = gap_big_sync_create(&s_big, &s_big_params);
    if (rc != ERROR_CODE_SUCCESS) {
        ESP_LOGW(TAG, "BIG sync create refused 0x%02x", rc);
        blacklist(s_cand_addr, BLACKLIST_MS);
        listen_reset("BIG sync refused");
        return;
    }
    s_big_pending = true;
    set_ls(L_BIG_SYNC, "BASE + BIGInfo");
}

static void on_ext_adv(const uint8_t *packet)
{
    if (s_ls != L_SCAN) {
        return;
    }
    uint8_t len = gap_event_extended_advertising_report_get_data_length(packet);
    const uint8_t *data = gap_event_extended_advertising_report_get_data(packet);
    if (gap_event_extended_advertising_report_get_periodic_advertising_interval(packet) == 0) {
        return;
    }
    bool baa = false, harman = false, jbl_name = false;
    uint32_t bid = 0;
    char name[33] = { 0 };
    ad_context_t c;
    for (ad_iterator_init(&c, len, data); ad_iterator_has_more(&c); ad_iterator_next(&c)) {
        uint8_t t = ad_iterator_get_data_type(&c), n = ad_iterator_get_data_len(&c);
        const uint8_t *d = ad_iterator_get_data(&c);
        if (t == BLUETOOTH_DATA_TYPE_SERVICE_DATA_16_BIT_UUID && n >= 5 && little_endian_read_16(d, 0) == 0x1852) {
            baa = true;
            bid = little_endian_read_24(d, 2);
        } else if (t == BLUETOOTH_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA && n >= 2 &&
                   little_endian_read_16(d, 0) == HARMAN_COMPANY_ID) {
            harman = true;
        } else if (t == 0x30 || ((t == BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME || t == BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME) && name[0] == 0)) {
            memcpy(name, d, n < sizeof(name) - 1 ? n : sizeof(name) - 1);
            name[n < sizeof(name) - 1 ? n : sizeof(name) - 1] = 0;
            if (strstr(name, "JBL")) jbl_name = true;
        }
    }
    if (!baa || !(harman || bid == JBL_BROADCAST_ID || jbl_name)) {
        return;                                        /* not a JBL party master */
    }
    bd_addr_t a;
    gap_event_extended_advertising_report_get_address(packet, a);
    if (blacklisted(a)) {
        return;
    }
    bd_addr_copy(s_cand_addr, a);
    s_cand_type = (bd_addr_type_t)gap_event_extended_advertising_report_get_address_type(packet);
    s_cand_sid = gap_event_extended_advertising_report_get_advertising_sid(packet);
    strcpy(s_cand_name, name);
    gap_periodic_advertiser_list_clear();
    gap_periodic_advertiser_list_add(s_cand_type, s_cand_addr, s_cand_sid);
    uint8_t rc = gap_periodic_advertising_create_sync(0x01, s_cand_sid, s_cand_type, s_cand_addr, 0,
                                                      PA_SYNC_TIMEOUT_10MS, 0);
    if (rc != ERROR_CODE_SUCCESS) {
        ESP_LOGW(TAG, "PA create sync refused 0x%02x", rc);
        return;
    }
    s_pa_pending = true;
    ESP_LOGW(TAG, "JBL party master found: '%s' %s (rssi %d) -> syncing", name, bd_addr_to_str(a),
             (int8_t)gap_event_extended_advertising_report_get_rssi(packet));
    set_ls(L_PA_SYNC, "master found");
}

static void hci_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }
    switch (hci_event_packet_get_type(packet)) {
    case HCI_EVENT_CONNECTION_REQUEST:
        s_scan_hold_until = now_ms() + CONN_SETUP_HOLD_MS;
        if (s_ls == L_SCAN && !s_scan_held) {
            gap_stop_scan();
            s_scan_held = true;
            ESP_LOGW(TAG, "phone connecting: LE scan paused %d s", CONN_SETUP_HOLD_MS / 1000);
        }
        break;
    case HCI_EVENT_CONNECTION_COMPLETE:
        s_scan_hold_until = now_ms() + 2000;          /* encryption / AVDTP setup follows */
        break;
    case GAP_EVENT_EXTENDED_ADVERTISING_REPORT:
        on_ext_adv(packet);
        break;
    case HCI_EVENT_LE_META:
        switch (hci_event_le_meta_get_subevent_code(packet)) {
        case HCI_SUBEVENT_LE_PERIODIC_ADVERTISING_SYNC_ESTABLISHMENT:
            if (!s_pa_pending) break;
            s_pa_pending = false;
            if (hci_subevent_le_periodic_advertising_sync_establishment_get_status(packet) != 0) {
                blacklist(s_cand_addr, BLACKLIST_MS);
                listen_reset("PA sync failed");
                break;
            }
            s_pa_handle = hci_subevent_le_periodic_advertising_sync_establishment_get_sync_handle(packet);
            if (s_role != R_LISTEN) {                  /* raced with audio starting */
                listen_reset("no longer listening");
                break;
            }
            gap_stop_scan();
            set_ls(L_W4_BASE, "PA synced");
            break;
        case HCI_SUBEVENT_LE_PERIODIC_ADVERTISING_REPORT:
            if (s_ls == L_W4_BASE && !s_have_base &&
                hci_subevent_le_periodic_advertising_report_get_sync_handle(packet) == s_pa_handle &&
                hci_subevent_le_periodic_advertising_report_get_data_status(packet) == 0 &&
                parse_base(hci_subevent_le_periodic_advertising_report_get_data(packet),
                           hci_subevent_le_periodic_advertising_report_get_data_length(packet), &s_base)) {
                s_have_base = true;
                try_big_sync();
            }
            break;
        case HCI_SUBEVENT_LE_BIGINFO_ADVERTISING_REPORT:
            if (s_ls == L_W4_BASE && !s_have_biginfo &&
                hci_subevent_le_biginfo_advertising_report_get_sync_handle(packet) == s_pa_handle) {
                s_have_biginfo = true;
                s_biginfo_enc = hci_subevent_le_biginfo_advertising_report_get_encryption(packet);
                try_big_sync();
            }
            break;
        case HCI_SUBEVENT_LE_PERIODIC_ADVERTISING_SYNC_LOST:
            if (hci_subevent_le_periodic_advertising_sync_lost_get_sync_handle(packet) == s_pa_handle) {
                s_pa_handle = HCI_CON_HANDLE_INVALID;
                listen_reset("PA sync lost (master stopped?)");
            }
            break;
        case HCI_SUBEVENT_LE_BIG_SYNC_LOST:
            s_big_active = false;
            listen_reset("BIG sync lost");
            break;
        default:
            break;
        }
        break;
    case HCI_EVENT_META_GAP:
        switch (hci_event_gap_meta_get_subevent_code(packet)) {
        case GAP_SUBEVENT_BIG_SYNC_CREATED:
            if (!s_big_pending) break;
            s_big_pending = false;
            if (gap_subevent_big_sync_created_get_status(packet) != 0) {
                ESP_LOGW(TAG, "BIG sync failed 0x%02x", gap_subevent_big_sync_created_get_status(packet));
                blacklist(s_cand_addr, BLACKLIST_MS);
                listen_reset("BIG sync failed");
                break;
            }
            s_big_active = true;
            s_num_bis = gap_subevent_big_sync_created_get_num_bis(packet);
            if (s_num_bis > MAX_BIS) s_num_bis = MAX_BIS;
            for (int i = 0; i < s_num_bis; i++) {
                s_bis[i] = gap_subevent_big_sync_created_get_bis_con_handles(packet, i);
            }
            if (s_role != R_LISTEN) {
                listen_reset("no longer listening");
                break;
            }
            if (!audio_start()) {
                listen_reset("audio path failed");
                break;
            }
            s_st.syncs++;
            s_last_sdu_ms = now_ms();
            ESP_LOGW(TAG, "listening to '%s': %u BIS, ISO interval %u us", s_cand_name, s_num_bis,
                     gap_subevent_big_sync_created_get_iso_interval_1250us(packet) * 1250u);
            set_ls(L_STREAM, "BIG synced");
            break;
        case GAP_SUBEVENT_BIG_SYNC_STOPPED:
            s_big_active = false;
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

/* ---- role tick (BT thread) ------------------------------------------------------ */
static void listen_poll(void)
{
    uint32_t age = now_ms() - s_state_ms;
    switch (s_ls) {
    case L_PA_SYNC:
        if (age > T_PA_SYNC_MS) { blacklist(s_cand_addr, BLACKLIST_MS); listen_reset("PA sync timeout"); }
        break;
    case L_W4_BASE:
        if (age > T_BASE_MS) { blacklist(s_cand_addr, BLACKLIST_MS); listen_reset("no BASE/BIGInfo"); }
        break;
    case L_BIG_SYNC:
        if (age > T_BIG_MS) { blacklist(s_cand_addr, BLACKLIST_MS); listen_reset("BIG sync timeout"); }
        break;
    case L_STREAM:
        if (now_ms() - s_last_sdu_ms > T_NO_SDU_MS) listen_reset("no audio received");
        break;
    case L_OFF:
        if (now_ms() - s_state_ms >= TICK_MS) listen_reset("start listening");
        break;
    case L_SCAN:
        if (s_boot_hold && (a2dp_in_connected() || (int32_t)(s_scan_hold_until - now_ms()) <= 0)) {
            s_boot_hold = false;
            if (a2dp_in_connected()) s_scan_hold_until = now_ms() + 1500;   /* AVDTP/AVRCP setup */
        }
        if (s_scan_held && !s_boot_hold && (int32_t)(s_scan_hold_until - now_ms()) <= 0) {
            scan_start();
            ESP_LOGW(TAG, "LE scan resumed");
        }
        break;
    default:
        break;
    }
}

static void tick(btstack_timer_source_t *ts)
{
    bool playing = a2dp_in_streaming();
    s_idle_ms = playing ? 0 : s_idle_ms + TICK_MS;
    if (!s_enabled && s_role != R_OFF) {              /* Auracast button: off */
        if (s_role == R_LISTEN || s_role == R_LISTEN_STOPPING) {
            set_role(R_OFF, "Auracast turned off");
            listen_reset("Auracast off");
        } else {
            auracast_host_stop();
            set_role(R_OFF, "Auracast turned off");
        }
        jbl_assist_set_paused(true);                  /* no party: don't pull speakers in */
        jbl_set_party_info(JBL_PARTY_INFO_OFF);
    }
    switch (s_role) {
    case R_OFF:
        if (s_enabled && auracast_state() == AURACAST_STATE_OFF) {
            int rc = auracast_host_start();               /* back on: host first */
            s_idle_ms = 0;
            jbl_assist_set_paused(false);
            jbl_set_party_info(JBL_PARTY_INFO_HOST);
            set_role(R_HOST, rc == 0 ? "Auracast turned on" : "Auracast turned on (host start error)");
        }
        break;
    case R_HOST:
        if (!playing && s_idle_ms >= PARTY_IDLE_TO_LISTEN_MS) {
            auracast_host_stop();
            set_role(R_HOST_STOPPING, "no audio for 1.5 s");
        }
        break;
    case R_HOST_STOPPING:
        if (playing) {
            auracast_host_start();
            set_role(R_HOST, "audio again");
        } else if (auracast_state() == AURACAST_STATE_OFF) {
            set_role(R_LISTEN, "broadcast stopped");
            s_st.switches_listen++;
            jbl_assist_set_paused(true);     /* its pause stops the scanner: start ours on the next tick */
            set_ls(L_OFF, "assistant pausing");
        }
        break;
    case R_LISTEN:
        if (playing) {
            set_role(R_LISTEN_STOPPING, "audio playing - host has priority");
            listen_reset("stop listening");
        } else {
            listen_poll();
        }
        break;
    case R_LISTEN_STOPPING:
        if (!s_big_active && !s_big_pending && !s_pa_pending) {
            jbl_assist_set_paused(false);
            jbl_set_party_info(JBL_PARTY_INFO_HOST);
            int rc = auracast_host_start();
            s_st.switches_host++;
            set_role(R_HOST, rc == 0 ? "listen torn down" : "listen torn down (host start error)");
        } else if (now_ms() - s_role_ms > T_TEARDOWN_MS) {
            s_big_active = s_big_pending = s_pa_pending = false;    /* never answered: go anyway */
        }
        break;
    }
    btstack_run_loop_set_timer(ts, TICK_MS);
    btstack_run_loop_add_timer(ts);
}

/* ---- API ------------------------------------------------------------------------ */
/* Boot benchmark (no radio yet): real 48 kHz / 10 ms / 100 B frame */
static void decode_bench(void)
{
    unsigned esz = lc3_encoder_size(10000, 48000), dsz = lc3_decoder_size(10000, 48000);
    void *em = heap_caps_malloc(esz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    void *dm = heap_caps_malloc(dsz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    static int16_t pcm[480];
    static uint8_t frame[100];
    if (em == NULL || dm == NULL) { free(em); free(dm); return; }
    lc3_encoder_t enc = lc3_setup_encoder(10000, 48000, 0, em);
    lc3_decoder_t dec = lc3_setup_decoder(10000, 48000, 0, dm);
    for (int i = 0; i < 480; i++) pcm[i] = (int16_t)(8000.0f * sinf(2.0f * 3.14159265f * 1000.0f * (float)i / 48000.0f));
    lc3_encode(enc, LC3_PCM_FORMAT_S16, pcm, 1, sizeof(frame), frame);
    int64_t t0 = esp_timer_get_time();
    for (int k = 0; k < 50; k++) lc3_decode(dec, frame, sizeof(frame), LC3_PCM_FORMAT_S16, pcm, 1);
    int64_t t1 = esp_timer_get_time();
    for (int k = 0; k < 20; k++) lc3_decode(dec, NULL, 0, LC3_PCM_FORMAT_S16, pcm, 1);
    int64_t t2 = esp_timer_get_time();
    ESP_LOGW(TAG, "LC3 decode bench (boot, core %d, state in PSRAM): %lld us/frame, PLC %lld us/frame",
             xPortGetCoreID(), (t1 - t0) / 50, (t2 - t1) / 20);
    free(em);
    free(dm);
}

void party_setup(void)
{
    decode_bench();
    s_sdu_q = xQueueCreateWithCaps(SDU_Q_LEN, sizeof(sdu_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_dec_lock = xSemaphoreCreateMutex();
    static StackType_t s_stack[8192 / sizeof(StackType_t)];
    static StaticTask_t s_tcb;
    /* core 0 = DSP core; the LC3 encoder is idle while we listen */
    xTaskCreateStaticPinnedToCore(decode_task, "party_dec", sizeof(s_stack) / sizeof(StackType_t), NULL,
                                  configMAX_PRIORITIES - 6, s_stack, &s_tcb, 0);
    s_hci_reg.callback = &hci_handler;
    hci_add_event_handler(&s_hci_reg);
    auracast_set_iso_forward_handler(&iso_rx);
}

void party_on_hci_working(void)
{
    /* boot: Auracast is off; go straight to LISTEN, but hold the scan until the
     * phone is back (power-up call-back pages it) or 6 s have passed */
    s_scan_hold_until = now_ms() + 6000;
    s_boot_hold = true;
    s_role = R_HOST_STOPPING;
    s_role_ms = now_ms();
    s_idle_ms = 0;
    btstack_run_loop_set_timer_handler(&s_tick, tick);
    btstack_run_loop_set_timer(&s_tick, TICK_MS);
    btstack_run_loop_add_timer(&s_tick);
}

bool party_auracast_enabled(void) { return s_enabled; }

void party_set_auracast_enabled(bool on)
{
    if (on != s_enabled) {
        ESP_LOGW(TAG, "Auracast %s (button)", on ? "ON" : "OFF");
        s_enabled = on;
    }
}

bool party_listening(void)
{
    return s_role == R_LISTEN || s_role == R_LISTEN_STOPPING;
}

void party_log_stats(void)
{
    static uint32_t l_rx[MAX_BIS], l_lost[MAX_BIS], l_dec, l_plc, l_qd;
    if (s_role == R_LISTEN && s_ls == L_STREAM) {
        uint32_t dn = st_dec_n ? st_dec_n : 1;
        ESP_LOGW(TAG, "LISTEN decode: avg %lu us max %lu us (%lu calls) | mono fills %lu, seq drops %lu",
                 (unsigned long)(st_dec_us / dn), (unsigned long)st_dec_max_us, (unsigned long)st_dec_n,
                 (unsigned long)st_mono_fill, (unsigned long)st_seq_drop);
        st_dec_us = st_dec_n = st_dec_max_us = 0;
        ESP_LOGW(TAG, "LISTEN '%s' 2s: BIS1 rx %lu lost %lu gaps %lu | BIS2 rx %lu lost %lu gaps %lu | decoded %lu PLC %lu errors %lu | q drops %lu, I2S late %lu",
                 s_cand_name,
                 (unsigned long)(s_st.rx[0] - l_rx[0]), (unsigned long)(s_st.lost[0] - l_lost[0]), (unsigned long)s_st.gaps[0],
                 (unsigned long)(s_st.rx[1] - l_rx[1]), (unsigned long)(s_st.lost[1] - l_lost[1]), (unsigned long)s_st.gaps[1],
                 (unsigned long)(s_st.decoded - l_dec), (unsigned long)(s_st.plc - l_plc), (unsigned long)s_st.errors,
                 (unsigned long)(s_st.qdrops - l_qd), (unsigned long)s_st.i2s_late);
    } else {
        ESP_LOGI(TAG, "role %s, listen %s | switches to listen %lu, to host %lu, syncs %lu",
                 role_names[s_role], lstate_names[s_ls], (unsigned long)s_st.switches_listen,
                 (unsigned long)s_st.switches_host, (unsigned long)s_st.syncs);
    }
    memcpy(l_rx, s_st.rx, sizeof(l_rx));
    memcpy(l_lost, s_st.lost, sizeof(l_lost));
    l_dec = s_st.decoded;
    l_plc = s_st.plc;
    l_qd = s_st.qdrops;
}
