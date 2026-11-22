/*
 * Auracast broadcast source on BTstack (see auracast.h for the on-air format).
 *
 * Audio pipeline, ISO-clocked:
 *
 *   core 0  "lc3_enc"   a2dp_in_read(480) -> v5_eq_process -> 2 x lc3_encode
 *                       -> ready queue (lock-free SPSC, 4 SDU pairs, internal RAM)
 *                       -> kick the BT thread (btstack_run_loop_execute_on_main_thread)
 *   core 1  "btstack"   ready pair -> HCI_EVENT_BIS_CAN_SEND_NOW -> ISO SDU on BIS 1 + 2
 *                       Number Of Completed Packets -> grant the encoder new pairs
 *
 * Credit accounting (BT thread):
 *   total = pairs in flight in the controller + pairs in the ready queue
 *         + pairs granted to the encoder but not produced yet
 * is held at AURA_PIPELINE (pre-roll 2 + 1 = 3): every pair the controller
 * completes grants exactly one new encode, so the encoder runs on the BIG's own
 * clock. Underflow: AURA_DEADLINE_MS after a completion the next BIG event must
 * have its BN = 2 SDUs per BIS in the controller; if the encoder is late a
 * pre-encoded silence pair is sent instead (the BIG never starves) and the late
 * pair is trimmed when it arrives, so the latency stays constant. Sequence
 * numbers advance per BIS for every SDU handed to the controller.
 */
#include "auracast.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "btstack.h"
#include "le-audio/le_audio_base_builder.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lc3.h"

#include "a2dp_in.h"
#include "bt_sched_tune.h"

static const char *TAG = "AURA";

/* JBL One EQ, implemented elsewhere; applied to the PCM right before encoding. */
void v5_eq_process(int16_t *pcm, int frames) __attribute__((weak));

/* liblc3 is built with LC3_ENCODER_STATS=1 and counts into this array
 * (frames, bw < FB, bw changes, TNS on, attack). Weak: someone else may own it. */
__attribute__((weak)) volatile uint32_t lc3_enc_stat[5];

/* ESP32-S31 controller, ISO "non-standard flow control" (CONFIG_BT_LE_ISO_NSFC_EN,
 * on in sdkconfig): no HCI Number Of Completed Packets for ISO; the controller
 * reports each BIS's TX completions through this callback instead (controller
 * task). Layout as esp_ble_iso/host/common/include/common/iso.h. */
typedef struct {
    uint16_t completed_pkts;
    uint16_t not_txd_pkts;
    uint8_t pkt_cnt;
    struct {
        uint16_t seq_num;
        uint32_t tx_time_stamp;
        uint32_t time_offset : 24;
        uint32_t drift : 1;
        uint32_t sdu_ref_time;
        uint32_t sdu_hdl_time;
        uint32_t sdu_rec_time;
    } pkt[2];
} ctl_iso_tx_info_t;
extern void r_ble_ll_isoal_tx_comp_cb_set(void *cb);
extern uint16_t r_ble_ll_iso_free_buf_num_get(uint16_t conn_handle);

/* ---- broadcast identity (keep exactly: the Xtreme 5 joins on these) -------- */
#define AURA_BROADCAST_ID      0x112233u
#define AURA_BROADCAST_NAME    "ESP32-S31's JBL Xtreme 5"
#define AURA_NAME_LEN          (sizeof(AURA_BROADCAST_NAME) - 1)
#define AURA_APPEARANCE        0x0885u
#define AURA_PBA_UUID          0x1856u
#define AURA_PBA_FEATURES      0x04u      /* High Quality, not encrypted */

/* ---- advertising (s31-jbl-speaker/main/bluedroid/adv.c) -------------------- */
#define AURA_ADV_SID           2          /* the Xtreme 5 broadcasts on SID 2 */
#define AURA_ADV_INTERVAL      128        /* 80 ms in 0.625 ms */
#define AURA_PA_INTERVAL       80         /* 100 ms in 1.25 ms */
#define AURA_PA_UPDATE_MS      500
#define AURA_GUARD_MS          3000       /* max wait for an adv / BIG step */

/* ---- codec / BIG ----------------------------------------------------------- */
#define AURA_NUM_BIS           2
#define AURA_SR_HZ             48000
#define AURA_FRAME_US          10000
#define AURA_FRAME_SAMPLES     480
#define AURA_SDU               120        /* octets per LC3 frame = SDU: 120 like the S26 Ultra broadcast (48_4), which the Xtreme 5 plays cleanly */
#define AURA_PD_US             80000
#define AURA_BIG_HANDLE        0
#define AURA_BN                3          /* 3 SDUs per BIG event (30 ms) */
#define AURA_LOC_FL            0x00000001u
#define AURA_LOC_FR            0x00000002u
#define AURA_CONTEXT           0x0005u    /* Unspecified | Media, as the Xtreme 5 */

/* The controller allocates BIG state from internal DMA RAM inside Create BIG
 * Test and asserts if that fails: never send it without headroom. */
#define AURA_BIG_MIN_FREE      (8 * 1024)
#define AURA_BIG_MIN_BLOCK     (2 * 1024)

/* ---- pacing ---------------------------------------------------------------- */
#define AURA_PREROLL           4          /* one BIG event (BN) + 1 ahead */
#define AURA_PIPELINE          (AURA_PREROLL + 1)  /* SDU pairs encoded or in flight */
#define AURA_ISO_QUEUE         (AURA_PIPELINE + 1) /* BTstack per-BIS cap (controller has 12 for all) */
#define AURA_RQ_DEPTH          8                   /* ready queue, power of 2 */
#define AURA_DEADLINE_MS       8                   /* after a completion: next event's SDUs must be in */
#define AURA_WDOG_MS           30                  /* no grant this long while ON: keep PCM in real time */

#define AURA_ENC_STACK         12288
#define AURA_ENC_PRIO          (configMAX_PRIORITIES - 4)

/* ---- JBL blocks (captured from a real Xtreme 5 master) --------------------- */
/* Ext adv: Manufacturer Specific Data, Harman 0x0057, 16 x 00, df fd. */
static const uint8_t jbl_master_mfg_ad[] = {
    0x15, 0xff, 0x57, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xdf, 0xfd,
};
/* PA, after the BASE: Manufacturer Specific Data, company 0xFDDF, TLVs
 * [02 02 1f] [03 f0 11 NN] [12 11 <17-byte status, YY at +17>]. */
static const uint8_t jbl_pa_mfg_ad[] = {
    0x1d, 0xff, 0xdf, 0xfd,
    0x02, 0x02, 0x1f,
    0x03, 0xf0, 0x11, 0x02,
    0x12, 0x11, 0x01, 0x01, 0x01, 0x00, 0x49, 0x00, 0x01, 0x01, 0x3c,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0xb9, 0x01,
};
/* A/B test: 0 = periodic data carries the BASE only (no JBL party block whose
 * NN/YY bytes we can only imitate; the real master's YY is not a timer). */
#ifndef AURA_JBL_PA_BLOCK
#define AURA_JBL_PA_BLOCK      1
#endif
#define JBL_PA_NN_OFFSET       10
#define JBL_PA_YY_OFFSET       17

/* ---- advertising / BIG storage (BT thread) --------------------------------- */
static uint8_t s_ext_adv[7 + 6 + 4 + 2 * (2 + AURA_NAME_LEN) + sizeof(jbl_master_mfg_ad)];
static uint8_t s_pa[64 + sizeof(jbl_pa_mfg_ad)];
static uint16_t s_pa_len, s_pa_jbl_pos;
static uint8_t s_pa_nn = 0x02;

static le_advertising_set_t s_adv_set;
static uint8_t s_adv_handle;
static bool s_adv_created;

static const le_extended_advertising_parameters_t s_ext_params = {
    .advertising_event_properties = 0,             /* extended, non-connectable, non-scannable */
    .primary_advertising_interval_min = AURA_ADV_INTERVAL,
    .primary_advertising_interval_max = AURA_ADV_INTERVAL,
    .primary_advertising_channel_map = 0x07,
    .own_address_type = BD_ADDR_TYPE_LE_PUBLIC,
    .peer_address_type = BD_ADDR_TYPE_LE_PUBLIC,
    .peer_address = { 0 },
    .advertising_filter_policy = 0,
    .advertising_tx_power = 127,                   /* no preference */
    .primary_advertising_phy = 1,                  /* LE 1M */
    .secondary_advertising_max_skip = 0,
    .secondary_advertising_phy = 2,                /* LE 2M */
    .advertising_sid = AURA_ADV_SID,
    .scan_request_notification_enable = 0,
};
static const le_periodic_advertising_parameters_t s_pa_params = {
    .periodic_advertising_interval_min = AURA_PA_INTERVAL,
    .periodic_advertising_interval_max = AURA_PA_INTERVAL,
    .periodic_advertising_properties = 0,
};

static le_audio_big_t s_big;
static le_audio_big_params_t s_big_params = {
    .big_handle = AURA_BIG_HANDLE,
    .num_bis = AURA_NUM_BIS,
    .sdu_interval_us = AURA_FRAME_US,
    .max_sdu = AURA_SDU,
    .max_transport_latency_ms = 65,                /* not used by Create BIG Test */
    .rtn = 2,                                      /* not used by Create BIG Test */
    .phy = 2,                                      /* 2M */
    .packing = 1,                                  /* interleaved */
    .framing = 0,                                  /* unframed */
    .encryption = 0,
};
/* Layout of the S26 Ultra's own broadcast (its BIGInfo, which the Xtreme 5
 * plays cleanly): ISO interval 30 ms, BN 3, IRC 2, PTO 1 -> NSE 9, PDU 120,
 * 2M. ~32 % airtime (the 120 B / 20 ms / IRC 4 layout took enough that the
 * phone's A2DP link starved). The old Xtreme 5 layout was 16/8/100/2/4/0. */
static const le_audio_big_test_params_t s_big_test = {
    .iso_interval_1250us = 24,
    .nse = 9,
    .max_pdu = AURA_SDU,
    .bn = AURA_BN,
    .irc = 2,
    .pto = 1,
};

/* ---- state machine (BT thread) --------------------------------------------- */
typedef enum {
    PH_OFF,
    PH_ADV_ON_WAIT,     /* STARTING: ext + periodic adv being enabled */
    PH_BIG_WAIT,        /* STARTING: Create BIG Test sent */
    PH_ON,
    PH_BIG_TERM_WAIT,   /* STOPPING: Terminate BIG sent */
    PH_ADV_OFF_WAIT,    /* STOPPING: periodic + ext adv being disabled */
} phase_t;

static phase_t s_phase;
static volatile auracast_state_t s_pub_state;
static volatile bool s_want_on;
static bool s_hci_up;
static uint8_t s_fail_status;
static auracast_state_cb_t s_state_cb;
static btstack_packet_handler_t s_iso_fwd;
static btstack_packet_callback_registration_t s_hci_reg;
static btstack_timer_source_t s_pa_timer, s_guard_timer, s_deadline_timer;

/* ---- audio pipeline -------------------------------------------------------- */
typedef struct {
    uint8_t sdu[AURA_NUM_BIS][AURA_SDU];
    uint16_t epoch;
} sdu_pair_t;

/* SPSC ready queue: head written by the encoder, tail by the BT thread. */
static sdu_pair_t s_rq[AURA_RQ_DEPTH];
static uint32_t s_rq_head, s_rq_tail;
/* (epoch << 16) | granted: written by the BT thread only */
static uint32_t s_grant;
/* (epoch << 16) | produced: written by the encoder only */
static uint32_t s_produced;
static volatile bool s_streaming;            /* BIG up: encoder watchdog keeps PCM in real time */

/* BT thread */
static uint16_t s_epoch, s_granted;
static hci_con_handle_t s_bis[AURA_NUM_BIS];
static uint32_t s_sent[AURA_NUM_BIS], s_done[AURA_NUM_BIS];
static uint16_t s_seq[AURA_NUM_BIS];
static uint8_t s_tx[AURA_NUM_BIS][AURA_SDU];
static bool s_tx_valid, s_csn_requested;
static uint8_t s_silence_due;
static uint8_t s_silence[AURA_SDU];          /* LC3 frame of digital silence, encoded at setup */
static btstack_context_callback_registration_t s_kick_reg, s_comp_reg;

/* controller task -> BT thread: completions per BIS not yet processed */
static volatile hci_con_handle_t s_bis_ctl[AURA_NUM_BIS] = { HCI_CON_HANDLE_INVALID, HCI_CON_HANDLE_INVALID };
static uint32_t s_comp_pending[AURA_NUM_BIS];
static volatile bool s_vendor_comp;          /* controller callback seen: ignore HCI NOCP */

/* encoder (core 0), internal RAM */
static lc3_encoder_t s_enc[AURA_NUM_BIS];
static int16_t s_pcm[AURA_FRAME_SAMPLES * 2];
static TaskHandle_t s_enc_task;
static StaticTask_t s_enc_tcb;
static StackType_t s_enc_stack[AURA_ENC_STACK / sizeof(StackType_t)];

/* ---- statistics: monotonic counters, deltas taken by auracast_log_stats ----- */
static volatile uint32_t st_sent[AURA_NUM_BIS], st_done[AURA_NUM_BIS];
static volatile uint32_t st_not_sent, st_late, st_trim, st_rq_full, st_not_txd;
static volatile uint32_t st_pcm_underruns, st_wdog, st_enc_frames, st_enc_us_sum;
static volatile uint32_t st_enc_us_max, st_enc_max_window, st_stats_window;

/* ======================================================================== */
/* advertising data                                                          */
/* ======================================================================== */

static void log_hex(const char *what, const uint8_t *d, unsigned n)
{
    char h[3 * 96 + 1];
    unsigned k;
    for (k = 0; k < n && k < 96; k++) {
        snprintf(h + 3 * k, 4, "%02x ", d[k]);
    }
    h[3 * k] = 0;
    ESP_LOGI(TAG, "%s (%u B): %s", what, n, h);
}

static void ext_adv_build(void)
{
    uint8_t *d = s_ext_adv;
    unsigned i = 0;

    d[i++] = 6;                                       /* Broadcast Audio Announcement */
    d[i++] = BLUETOOTH_DATA_TYPE_SERVICE_DATA_16_BIT_UUID;
    little_endian_store_16(d, i, ORG_BLUETOOTH_SERVICE_BROADCAST_AUDIO_ANNOUNCEMENT_SERVICE);
    i += 2;
    little_endian_store_24(d, i, AURA_BROADCAST_ID);
    i += 3;

    d[i++] = 5;                                       /* Public Broadcast Announcement */
    d[i++] = BLUETOOTH_DATA_TYPE_SERVICE_DATA_16_BIT_UUID;
    little_endian_store_16(d, i, AURA_PBA_UUID);
    i += 2;
    d[i++] = AURA_PBA_FEATURES;
    d[i++] = 0;                                       /* no metadata */

    d[i++] = 3;
    d[i++] = BLUETOOTH_DATA_TYPE_APPEARANCE;
    little_endian_store_16(d, i, AURA_APPEARANCE);
    i += 2;

    d[i++] = AURA_NAME_LEN + 1;
    d[i++] = BLUETOOTH_DATA_TYPE_BROADCAST_NAME;
    memcpy(d + i, AURA_BROADCAST_NAME, AURA_NAME_LEN);
    i += AURA_NAME_LEN;

    d[i++] = AURA_NAME_LEN + 1;                       /* as the real JBL master */
    d[i++] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
    memcpy(d + i, AURA_BROADCAST_NAME, AURA_NAME_LEN);
    i += AURA_NAME_LEN;

    memcpy(d + i, jbl_master_mfg_ad, sizeof(jbl_master_mfg_ad));   /* the JBL master marker */
    i += sizeof(jbl_master_mfg_ad);

    btstack_assert(i == sizeof(s_ext_adv));
}

static void pa_build(void)
{
    static const uint8_t codec_id[5] = { HCI_AUDIO_CODING_FORMAT_LC3, 0, 0, 0, 0 };
    /* subgroup: 48 kHz, 10 ms, FL|FR, 100 octets (frames per SDU 1 = omitted) */
    static const uint8_t cc_subgroup[] = {
        0x02, 0x01, 0x08,
        0x02, 0x02, 0x01,
        0x05, 0x03, 0x03, 0x00, 0x00, 0x00,
        0x03, 0x04, AURA_SDU, 0x00,
    };
    static const uint8_t metadata[] = { 0x03, 0x02, AURA_CONTEXT & 0xff, AURA_CONTEXT >> 8 };
    static const uint8_t cc_bis_fl[] = { 0x05, 0x03, 0x01, 0x00, 0x00, 0x00 };
    static const uint8_t cc_bis_fr[] = { 0x05, 0x03, 0x02, 0x00, 0x00, 0x00 };

    le_audio_base_builder_t b;
    le_audio_base_builder_init(&b, s_pa, sizeof(s_pa) - sizeof(jbl_pa_mfg_ad), AURA_PD_US);
    le_audio_base_builder_add_subgroup(&b, codec_id, sizeof(cc_subgroup), cc_subgroup,
                                       sizeof(metadata), metadata);
    le_audio_base_builder_add_bis(&b, 1, sizeof(cc_bis_fl), cc_bis_fl);
    le_audio_base_builder_add_bis(&b, 2, sizeof(cc_bis_fr), cc_bis_fr);
    s_pa_jbl_pos = le_audio_base_builder_get_ad_data_size(&b);
#if AURA_JBL_PA_BLOCK
    memcpy(s_pa + s_pa_jbl_pos, jbl_pa_mfg_ad, sizeof(jbl_pa_mfg_ad));
    s_pa_len = s_pa_jbl_pos + sizeof(jbl_pa_mfg_ad);
#else
    s_pa_len = s_pa_jbl_pos;                      /* BASE only */
#endif
}

/* BT thread: tick the JBL party block and hand it to the controller. BTstack
 * uploads from s_pa on its next run (single fragment, < 252 B). */
static void pa_refresh(void)
{
#if !AURA_JBL_PA_BLOCK
    return;
#endif
    s_pa[s_pa_jbl_pos + JBL_PA_NN_OFFSET] = s_pa_nn;
    s_pa[s_pa_jbl_pos + JBL_PA_YY_OFFSET] = (uint8_t)(btstack_run_loop_get_time_ms() / 10);
    gap_periodic_advertising_set_data(s_adv_handle, s_pa_len, s_pa);
}

static void pa_timer_handler(btstack_timer_source_t *ts)
{
    if (s_phase == PH_OFF || s_phase == PH_ADV_OFF_WAIT) {
        return;
    }
    s_pa_nn++;
    pa_refresh();
    btstack_run_loop_set_timer(ts, AURA_PA_UPDATE_MS);
    btstack_run_loop_add_timer(ts);
}

/* ======================================================================== */
/* encoder (core 0)                                                          */
/* ======================================================================== */

static void enc_produce(sdu_pair_t *slot)
{
    if (a2dp_in_read(s_pcm, AURA_FRAME_SAMPLES) < AURA_FRAME_SAMPLES && a2dp_in_streaming()) {
        st_pcm_underruns++;
    }
    if (v5_eq_process != NULL) {
        v5_eq_process(s_pcm, AURA_FRAME_SAMPLES);
    }
    int64_t t0 = esp_timer_get_time();
    for (int ch = 0; ch < AURA_NUM_BIS; ch++) {
        if (lc3_encode(s_enc[ch], LC3_PCM_FORMAT_S16, s_pcm + ch, 2, AURA_SDU, slot->sdu[ch]) != 0) {
            memcpy(slot->sdu[ch], s_silence, AURA_SDU);
        }
    }
    uint32_t us = (uint32_t)(esp_timer_get_time() - t0);
    st_enc_us_sum += us;
    st_enc_frames++;
    uint32_t w = st_stats_window;
    if (w != st_enc_max_window) {                 /* stats window rolled over: restart max */
        st_enc_max_window = w;
        st_enc_us_max = 0;
    }
    if (us > st_enc_us_max) {
        st_enc_us_max = us;
    }
}

static void enc_task(void *arg)
{
    (void)arg;
    uint16_t my_epoch = 0, produced = 0;
    for (;;) {
        uint32_t woken = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(AURA_WDOG_MS));
        bool made = false;
        for (;;) {
            uint32_t g = __atomic_load_n(&s_grant, __ATOMIC_ACQUIRE);
            uint16_t e = (uint16_t)(g >> 16);
            if (e != my_epoch) {
                my_epoch = e;
                produced = 0;
            }
            uint16_t avail = (uint16_t)((uint16_t)g - produced);
            if (avail == 0 || avail > 0x8000u) {
                break;
            }
            uint32_t head = s_rq_head;
            if (head - __atomic_load_n(&s_rq_tail, __ATOMIC_ACQUIRE) >= AURA_RQ_DEPTH) {
                st_rq_full++;                     /* BT thread drains it, then grants again */
                break;
            }
            sdu_pair_t *slot = &s_rq[head & (AURA_RQ_DEPTH - 1)];
            enc_produce(slot);
            slot->epoch = my_epoch;
            __atomic_store_n(&s_rq_head, head + 1, __ATOMIC_RELEASE);
            produced++;
            __atomic_store_n(&s_produced, ((uint32_t)my_epoch << 16) | produced, __ATOMIC_RELEASE);
            btstack_run_loop_execute_on_main_thread(&s_kick_reg);
            made = true;
        }
        /* BIG stalled (no completions -> no grants): keep reading PCM in real
         * time so the A2DP ring and its clock recovery do not run away. */
        if (!woken && !made && s_streaming) {
            for (int k = 0; k < AURA_WDOG_MS / 10; k++) {
                a2dp_in_read(s_pcm, AURA_FRAME_SAMPLES);
            }
            st_wdog++;
        }
    }
}

/* ======================================================================== */
/* ISO TX (BT thread)                                                        */
/* ======================================================================== */

static inline uint32_t rq_count(void)
{
    return __atomic_load_n(&s_rq_head, __ATOMIC_ACQUIRE) - s_rq_tail;
}

static inline void rq_release(void)
{
    __atomic_store_n(&s_rq_tail, s_rq_tail + 1, __ATOMIC_RELEASE);
}

/* Drop pairs encoded for a previous BIG session (always at the queue front). */
static void rq_drop_stale(void)
{
    while (rq_count() > 0 && s_rq[s_rq_tail & (AURA_RQ_DEPTH - 1)].epoch != s_epoch) {
        rq_release();
    }
}

static uint32_t in_flight(void)
{
    uint32_t worst = 0;
    for (int i = 0; i < AURA_NUM_BIS; i++) {
        uint32_t n = s_sent[i] - s_done[i];
        if (n > 0x80000000u) {                    /* completions overtook (cannot happen) */
            s_done[i] = s_sent[i];
            n = 0;
        }
        if (n > worst) {
            worst = n;
        }
    }
    return worst;
}

static uint32_t outstanding(void)
{
    uint32_t p = __atomic_load_n(&s_produced, __ATOMIC_ACQUIRE);
    if ((uint16_t)(p >> 16) != s_epoch) {
        return s_granted;                         /* encoder has not produced for this epoch yet */
    }
    return (uint16_t)(s_granted - (uint16_t)p);
}

static void grant_publish(void)
{
    __atomic_store_n(&s_grant, ((uint32_t)s_epoch << 16) | s_granted, __ATOMIC_RELEASE);
    xTaskNotifyGive(s_enc_task);
}

/* Hold in-flight + ready + granted at AURA_PIPELINE. */
static void rebalance(void)
{
    rq_drop_stale();
    uint32_t rq = rq_count();
    uint32_t total = in_flight() + rq + outstanding();
    if (total < AURA_PIPELINE) {
        s_granted = (uint16_t)(s_granted + (AURA_PIPELINE - total));
        grant_publish();
    } else if (total > AURA_PIPELINE && rq > 0) {
        /* silence went out in place of a late pair: drop the oldest audio so
         * the latency does not grow */
        uint32_t n = total - AURA_PIPELINE;
        if (n > rq) {
            n = rq;
        }
        while (n--) {
            rq_release();
            st_trim++;
        }
    }
}

static void tx_kick(void)
{
    if (s_phase != PH_ON) {
        return;
    }
    rebalance();
    if (!s_csn_requested && (s_silence_due > 0 || rq_count() > 0)) {
        /* set first: BTstack may emit HCI_EVENT_BIS_CAN_SEND_NOW from inside the call */
        s_csn_requested = true;
        if (hci_request_bis_can_send_now_events(AURA_BIG_HANDLE) != ERROR_CODE_SUCCESS) {
            s_csn_requested = false;
        }
    }
}

static void kick_cb(void *context)
{
    (void)context;
    tx_kick();
}

static void send_sdu(int bis)
{
    hci_reserve_packet_buffer();
    uint8_t *b = hci_get_outgoing_packet_buffer();
    little_endian_store_16(b, 0, (uint16_t)s_bis[bis] | (2u << 12));   /* complete SDU, no time stamp */
    little_endian_store_16(b, 2, 4 + AURA_SDU);
    little_endian_store_16(b, 4, s_seq[bis]);
    little_endian_store_16(b, 6, AURA_SDU);
    memcpy(b + 8, s_tx[bis], AURA_SDU);
    uint8_t st = hci_send_iso_packet_buffer(8 + AURA_SDU);
    s_seq[bis]++;                                 /* the slot is used either way */
    if (st == ERROR_CODE_SUCCESS) {
        s_sent[bis]++;
        st_sent[bis]++;
    } else {
        st_not_sent++;
    }
}

static void on_bis_can_send_now(uint8_t bis)
{
    if (s_phase != PH_ON || bis >= AURA_NUM_BIS) {
        return;
    }
    if (bis == 0) {
        rq_drop_stale();
        s_tx_valid = false;
        if (rq_count() > 0) {
            const sdu_pair_t *p = &s_rq[s_rq_tail & (AURA_RQ_DEPTH - 1)];
            memcpy(s_tx, p->sdu, sizeof(s_tx));
            rq_release();                         /* slot back to the encoder */
            s_silence_due = 0;                    /* real audio made it in time */
            s_tx_valid = true;
        } else if (s_silence_due > 0) {
            s_silence_due--;
            for (int i = 0; i < AURA_NUM_BIS; i++) {
                memcpy(s_tx[i], s_silence, AURA_SDU);
            }
            st_late++;
            s_tx_valid = true;
        }
    }
    if (s_tx_valid) {
        send_sdu(bis);
    }
    if (bis == AURA_NUM_BIS - 1) {
        s_tx_valid = false;
        s_csn_requested = false;
        tx_kick();
    }
}

static void deadline_handler(btstack_timer_source_t *ts)
{
    (void)ts;
    if (s_phase != PH_ON) {
        return;
    }
    rq_drop_stale();
    uint32_t f = in_flight();
    if (f < AURA_BN && rq_count() == 0) {
        s_silence_due = (uint8_t)(AURA_BN - f);
        tx_kick();
    }
}

/* SDUs left the controller (sent or flushed): arm the underflow check for the
 * next BIG event and grant the encoder the freed pairs. */
static void completions_processed(void)
{
    btstack_run_loop_remove_timer(&s_deadline_timer);
    btstack_run_loop_set_timer(&s_deadline_timer, AURA_DEADLINE_MS);
    btstack_run_loop_add_timer(&s_deadline_timer);
    tx_kick();
}

/* HCI Number Of Completed Packets (standard flow control). BTstack has
 * already released its own per-BIS counts. */
static void on_hci_completed(const uint8_t *packet, uint16_t size)
{
    if (s_phase != PH_ON || s_vendor_comp || size < 3) {
        return;
    }
    unsigned n = packet[2];
    bool ours = false;
    if (size < 3u + 4u * n) {
        return;
    }
    for (unsigned k = 0; k < n; k++) {
        hci_con_handle_t h = little_endian_read_16(packet, 3 + 4 * k) & 0x0fff;
        uint16_t c = little_endian_read_16(packet, 5 + 4 * k);
        for (int i = 0; i < AURA_NUM_BIS; i++) {
            if (h == s_bis[i]) {
                s_done[i] += c;
                st_done[i] += c;
                ours = true;
            }
        }
    }
    if (ours) {
        completions_processed();
    }
}

/* Controller task: vendor TX completion (non-standard flow control). Only
 * counts and wakes the BT thread; no locks held by the controller are taken. */
static void ctl_tx_comp_cb(uint16_t conn_handle, void *info, size_t size)
{
    if (info == NULL || size < 4) {
        return;
    }
    const ctl_iso_tx_info_t *ti = (const ctl_iso_tx_info_t *)info;
    for (int i = 0; i < AURA_NUM_BIS; i++) {
        if (conn_handle == s_bis_ctl[i]) {
            __atomic_fetch_add(&s_comp_pending[i], (uint32_t)ti->completed_pkts + ti->not_txd_pkts,
                               __ATOMIC_RELEASE);
            st_not_txd += ti->not_txd_pkts;
            s_vendor_comp = true;
            btstack_run_loop_execute_on_main_thread(&s_comp_reg);
            return;
        }
    }
}

/* BT thread: feed the controller-reported completions into BTstack's ISO
 * flow control and our pipeline. */
static void comp_drain_cb(void *context)
{
    (void)context;
    bool any = false;
    for (int i = 0; i < AURA_NUM_BIS; i++) {
        uint32_t c = __atomic_exchange_n(&s_comp_pending[i], 0, __ATOMIC_ACQUIRE);
        if (c == 0 || s_phase != PH_ON) {
            continue;
        }
        hci_iso_packets_completed(s_bis[i], (uint16_t)c);
        s_done[i] += c;
        st_done[i] += c;
        any = true;
    }
    if (any) {
        completions_processed();
    }
}

/* New epoch: anything still queued or being encoded belongs to the old one. */
static void pipeline_reset(bool run)
{
    s_epoch++;
    s_granted = run ? AURA_PIPELINE : 0;          /* run: pre-roll, then ISO-clocked */
    __atomic_store_n(&s_rq_tail, __atomic_load_n(&s_rq_head, __ATOMIC_ACQUIRE), __ATOMIC_RELEASE);
    memset(s_sent, 0, sizeof(s_sent));
    memset(s_done, 0, sizeof(s_done));
    memset(s_seq, 0, sizeof(s_seq));
    for (int i = 0; i < AURA_NUM_BIS; i++) {
        __atomic_store_n(&s_comp_pending[i], 0, __ATOMIC_RELEASE);
        if (!run) {
            s_bis_ctl[i] = HCI_CON_HANDLE_INVALID;   /* late controller completions are ignored */
        }
    }
    s_tx_valid = false;
    s_csn_requested = false;
    s_silence_due = 0;
    btstack_run_loop_remove_timer(&s_deadline_timer);
    s_streaming = run;
    grant_publish();
}

/* ======================================================================== */
/* host start / stop state machine (BT thread)                               */
/* ======================================================================== */

static void set_phase(phase_t ph)
{
    s_phase = ph;
    switch (ph) {
    case PH_OFF:
        s_pub_state = AURACAST_STATE_OFF;
        break;
    case PH_ADV_ON_WAIT:
    case PH_BIG_WAIT:
        s_pub_state = AURACAST_STATE_STARTING;
        break;
    case PH_ON:
        s_pub_state = AURACAST_STATE_ON;
        break;
    default:
        s_pub_state = AURACAST_STATE_STOPPING;
        break;
    }
}

static void guard_arm(void)
{
    btstack_run_loop_remove_timer(&s_guard_timer);
    btstack_run_loop_set_timer(&s_guard_timer, AURA_GUARD_MS);
    btstack_run_loop_add_timer(&s_guard_timer);
}

static void guard_cancel(void)
{
    btstack_run_loop_remove_timer(&s_guard_timer);
}

static void notify(auracast_state_t st, uint8_t status)
{
    if (s_state_cb != NULL) {
        s_state_cb(st, status);
    }
}

static int begin_start(void);

static void adv_off_done(void)
{
    guard_cancel();
    set_phase(PH_OFF);
    uint8_t status = s_fail_status;
    s_fail_status = 0;
    if (status != 0) {
        s_want_on = false;                        /* a failed start is not retried here */
        ESP_LOGE(TAG, "broadcast start failed, status 0x%02x", status);
    } else {
        ESP_LOGW(TAG, "broadcast off");
    }
    notify(AURACAST_STATE_OFF, status);
    if (s_want_on && begin_start() != 0) {
        s_want_on = false;
        notify(AURACAST_STATE_OFF, ERROR_CODE_MEMORY_CAPACITY_EXCEEDED);
    }
}

static void begin_adv_off(void)
{
    btstack_run_loop_remove_timer(&s_pa_timer);
    gap_periodic_advertising_stop(s_adv_handle);
    gap_extended_advertising_stop(s_adv_handle);
    set_phase(PH_ADV_OFF_WAIT);
    /* BTstack sends LE Set Periodic Advertising Enable(0) last; its Command
     * Complete ends the stop. Nothing to wait for if PA never got enabled. */
    if ((s_adv_set.state & LE_ADVERTISEMENT_STATE_PERIODIC_ACTIVE) == 0) {
        adv_off_done();
        return;
    }
    guard_arm();
}

static void begin_big_terminate(void)
{
    s_streaming = false;
    pipeline_reset(false);
    set_phase(PH_BIG_TERM_WAIT);
    if (gap_big_terminate(AURA_BIG_HANDLE) != ERROR_CODE_SUCCESS) {
        begin_adv_off();                          /* no BIG known to BTstack */
        return;
    }
    guard_arm();
}

static bool big_heap_ok(void)
{
    const uint32_t caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT | MALLOC_CAP_DMA;
    size_t fr = heap_caps_get_free_size(caps), lb = heap_caps_get_largest_free_block(caps);
    if (fr < AURA_BIG_MIN_FREE || lb < AURA_BIG_MIN_BLOCK) {
        ESP_LOGE(TAG, "BIG create skipped: internal DMA free %u B, largest %u B (need %u/%u)",
                 (unsigned)fr, (unsigned)lb, (unsigned)AURA_BIG_MIN_FREE, (unsigned)AURA_BIG_MIN_BLOCK);
        return false;
    }
    return true;
}

static int begin_start(void)
{
    if (!big_heap_ok()) {
        return -ENOMEM;
    }
    if (!s_adv_created) {
        if (gap_extended_advertising_setup(&s_adv_set, &s_ext_params, &s_adv_handle) != ERROR_CODE_SUCCESS) {
            return -EIO;
        }
        gap_extended_advertising_set_adv_data(s_adv_handle, sizeof(s_ext_adv), s_ext_adv);
        gap_periodic_advertising_set_params(s_adv_handle, &s_pa_params);
        s_adv_created = true;
        ESP_LOGI(TAG, "advertising set handle %u (SID %u)", s_adv_handle, AURA_ADV_SID);
    }
    s_fail_status = 0;
    r_ble_ll_isoal_tx_comp_cb_set((void *)ctl_tx_comp_cb);
    pa_refresh();
    gap_periodic_advertising_start(s_adv_handle, false);
    gap_extended_advertising_start(s_adv_handle, 0, 0);
    set_phase(PH_ADV_ON_WAIT);
    btstack_run_loop_remove_timer(&s_pa_timer);
    btstack_run_loop_set_timer(&s_pa_timer, AURA_PA_UPDATE_MS);
    btstack_run_loop_add_timer(&s_pa_timer);
    guard_arm();
    return 0;
}

static void on_pa_enable_complete(uint8_t status)
{
    if (s_phase == PH_ADV_OFF_WAIT) {
        adv_off_done();
        return;
    }
    if (s_phase != PH_ADV_ON_WAIT) {
        return;
    }
    guard_cancel();
    if (!s_want_on) {
        begin_adv_off();
        return;
    }
    if (status != ERROR_CODE_SUCCESS) {
        ESP_LOGE(TAG, "periodic advertising enable failed, status 0x%02x", status);
        s_fail_status = status;
        begin_adv_off();
        return;
    }
    if (!big_heap_ok()) {
        s_fail_status = ERROR_CODE_MEMORY_CAPACITY_EXCEEDED;
        begin_adv_off();
        return;
    }
    s_big_params.advertising_handle = s_adv_handle;
    uint8_t st = gap_big_create_test(&s_big, &s_big_params, &s_big_test);
    if (st != ERROR_CODE_SUCCESS) {
        ESP_LOGE(TAG, "gap_big_create_test: 0x%02x", st);
        s_fail_status = st;
        begin_adv_off();
        return;
    }
    set_phase(PH_BIG_WAIT);
    guard_arm();
}

static void on_big_created(const uint8_t *packet)
{
    if (gap_subevent_big_created_get_big_handle(packet) != AURA_BIG_HANDLE || s_phase != PH_BIG_WAIT) {
        return;
    }
    guard_cancel();
    uint8_t status = gap_subevent_big_created_get_status(packet);
    if (status != ERROR_CODE_SUCCESS) {
        ESP_LOGE(TAG, "Create BIG Test failed, status 0x%02x", status);
        s_fail_status = status;                   /* BTstack already dropped the BIG */
        begin_adv_off();
        return;
    }
    for (int i = 0; i < AURA_NUM_BIS; i++) {
        s_bis[i] = gap_subevent_big_created_get_bis_con_handles(packet, (uint8_t)i);
        s_bis_ctl[i] = s_bis[i];
    }
    set_phase(PH_ON);
    pipeline_reset(true);
    ESP_LOGW(TAG, "broadcast ON: '%s' Broadcast_ID 0x%06x, BIS 0x%04x/0x%04x, ISO interval %u us",
             AURA_BROADCAST_NAME, (unsigned)AURA_BROADCAST_ID, s_bis[0], s_bis[1],
             gap_subevent_big_created_get_iso_interval_1250us(packet) * 1250u);
    notify(AURACAST_STATE_ON, 0);
    if (!s_want_on) {
        begin_big_terminate();
    }
}

static void on_big_terminated(const uint8_t *packet)
{
    if (gap_subevent_big_terminated_get_big_handle(packet) != AURA_BIG_HANDLE) {
        return;
    }
    if (s_phase == PH_BIG_TERM_WAIT || s_phase == PH_BIG_WAIT || s_phase == PH_ON) {
        guard_cancel();
        s_streaming = false;
        pipeline_reset(false);
        begin_adv_off();
    }
}

static void guard_handler(btstack_timer_source_t *ts)
{
    (void)ts;
    switch (s_phase) {
    case PH_ADV_ON_WAIT:
        ESP_LOGE(TAG, "no periodic advertising enable within %u ms", AURA_GUARD_MS);
        s_fail_status = ERROR_CODE_CONNECTION_TIMEOUT;
        begin_adv_off();
        break;
    case PH_BIG_WAIT:
        ESP_LOGE(TAG, "no BIG Created within %u ms - terminating", AURA_GUARD_MS);
        s_fail_status = ERROR_CODE_CONNECTION_TIMEOUT;
        begin_big_terminate();
        break;
    case PH_BIG_TERM_WAIT:
        ESP_LOGE(TAG, "no BIG Terminated within %u ms", AURA_GUARD_MS);
        begin_adv_off();
        break;
    case PH_ADV_OFF_WAIT:
        ESP_LOGE(TAG, "no periodic advertising disable within %u ms", AURA_GUARD_MS);
        adv_off_done();
        break;
    default:
        break;
    }
}

/* ======================================================================== */
/* BTstack handlers (BT thread)                                              */
/* ======================================================================== */

static void hci_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }
    switch (hci_event_packet_get_type(packet)) {
    case BTSTACK_EVENT_STATE:
        s_hci_up = btstack_event_state_get_state(packet) == HCI_STATE_WORKING;
        if (!s_hci_up && s_phase != PH_OFF) {
            /* controller gone: BTstack state is reset, so is ours */
            guard_cancel();
            btstack_run_loop_remove_timer(&s_pa_timer);
            s_streaming = false;
            pipeline_reset(false);
            set_phase(PH_OFF);
            s_want_on = false;
            notify(AURACAST_STATE_OFF, ERROR_CODE_HARDWARE_FAILURE);
        }
        break;
    case HCI_EVENT_NUMBER_OF_COMPLETED_PACKETS:
        on_hci_completed(packet, size);
        break;
    case HCI_EVENT_COMMAND_COMPLETE:
        if (hci_event_command_complete_get_command_opcode(packet) == HCI_OPCODE_HCI_LE_SET_PERIODIC_ADVERTISING_ENABLE &&
            size >= 6) {
            on_pa_enable_complete(hci_event_command_complete_get_return_parameters(packet)[0]);
        }
        break;
    case HCI_EVENT_META_GAP:
        switch (hci_event_gap_meta_get_subevent_code(packet)) {
        case GAP_SUBEVENT_BIG_CREATED:
            on_big_created(packet);
            break;
        case GAP_SUBEVENT_BIG_TERMINATED:
            on_big_terminated(packet);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

static void iso_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    if (packet_type == HCI_EVENT_PACKET && hci_event_packet_get_type(packet) == HCI_EVENT_BIS_CAN_SEND_NOW &&
        hci_event_bis_can_send_now_get_big_handle(packet) == AURA_BIG_HANDLE) {
        on_bis_can_send_now(hci_event_bis_can_send_now_get_bis_index(packet));
        return;
    }
    if (s_iso_fwd != NULL) {
        s_iso_fwd(packet_type, channel, packet, size);
    }
}

/* ======================================================================== */
/* public API                                                                */
/* ======================================================================== */

void auracast_setup(void)
{
    ext_adv_build();
    pa_build();
    log_hex("EXT ADV", s_ext_adv, sizeof(s_ext_adv));
    log_hex("PA (BASE + JBL)", s_pa, s_pa_len);

    /* hot encoder state: internal RAM explicitly */
    const unsigned esz = lc3_encoder_size(AURA_FRAME_US, AURA_SR_HZ);
    for (int ch = 0; ch < AURA_NUM_BIS; ch++) {
        void *mem = heap_caps_malloc(esz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        btstack_assert(mem != NULL);
        s_enc[ch] = lc3_setup_encoder(AURA_FRAME_US, AURA_SR_HZ, 0, mem);
        btstack_assert(s_enc[ch] != NULL);
    }
    /* digital silence, from a throw-away encoder so the live ones keep their state */
    {
        void *mem = heap_caps_malloc(esz, MALLOC_CAP_8BIT);
        btstack_assert(mem != NULL);
        lc3_encoder_t e = lc3_setup_encoder(AURA_FRAME_US, AURA_SR_HZ, 0, mem);
        memset(s_pcm, 0, sizeof(s_pcm));
        lc3_encode(e, LC3_PCM_FORMAT_S16, s_pcm, 2, AURA_SDU, s_silence);
        heap_caps_free(mem);
    }

    s_kick_reg.callback = &kick_cb;
    s_comp_reg.callback = &comp_drain_cb;
    btstack_run_loop_set_timer_handler(&s_pa_timer, &pa_timer_handler);
    btstack_run_loop_set_timer_handler(&s_guard_timer, &guard_handler);
    btstack_run_loop_set_timer_handler(&s_deadline_timer, &deadline_handler);

    /* core 0 = DSP core, above the SBC decoder (configMAX_PRIORITIES - 6) */
    s_enc_task = xTaskCreateStaticPinnedToCore(enc_task, "lc3_enc", AURA_ENC_STACK, NULL, AURA_ENC_PRIO,
                                               s_enc_stack, &s_enc_tcb, 0);
    btstack_assert(s_enc_task != NULL);

    s_hci_reg.callback = &hci_handler;
    hci_add_event_handler(&s_hci_reg);
    hci_register_iso_packet_handler(&iso_handler);
    hci_set_num_iso_packets_to_queue(AURA_ISO_QUEUE);
    set_phase(PH_OFF);
    ESP_LOGI(TAG, "LC3 %d Hz / %d us / %d B x %d BIS, encoder %u B each, pre-roll %d",
             AURA_SR_HZ, AURA_FRAME_US, AURA_SDU, AURA_NUM_BIS, esz, AURA_PREROLL);
}

void auracast_on_hci_working(void)
{
    s_hci_up = true;
    r_ble_ll_isoal_tx_comp_cb_set((void *)ctl_tx_comp_cb);   /* after the controller is up */
    bt_sched_tune_init();                         /* MASTER profile = idle: BIS above the ACL */
}

int auracast_host_start(void)
{
    if (!s_hci_up) {
        return -ENODEV;
    }
    if (s_want_on) {
        return 0;                                 /* starting / on, or restart already queued */
    }
    s_want_on = true;
    if (s_phase != PH_OFF) {
        return 0;                                 /* stopping: restarts when it settles */
    }
    int err = begin_start();
    if (err != 0) {
        s_want_on = false;
    }
    return err;
}

int auracast_host_stop(void)
{
    s_want_on = false;
    switch (s_phase) {
    case PH_ON:
        begin_big_terminate();
        break;
    case PH_BIG_WAIT:
        guard_cancel();
        begin_big_terminate();                    /* BTstack terminates once the BIG exists */
        break;
    default:
        /* PH_ADV_ON_WAIT: stops when the enable completes; stopping / off: nothing to do */
        break;
    }
    return 0;
}

bool auracast_hosting(void)
{
    auracast_state_t st = s_pub_state;
    return st == AURACAST_STATE_STARTING || st == AURACAST_STATE_ON;
}

auracast_state_t auracast_state(void)
{
    return s_pub_state;
}

void auracast_set_state_callback(auracast_state_cb_t cb)
{
    s_state_cb = cb;
}

void auracast_set_iso_forward_handler(btstack_packet_handler_t handler)
{
    s_iso_fwd = handler;
}

void auracast_log_stats(void)
{
    static uint32_t l_sent[AURA_NUM_BIS], l_done[AURA_NUM_BIS];
    static uint32_t l_not_sent, l_not_txd, l_late, l_trim, l_ur, l_wdog, l_frames, l_us;
    static const char *const names[] = { "off", "starting", "ON", "stopping" };

    uint32_t sent0 = st_sent[0], sent1 = st_sent[1], done0 = st_done[0], done1 = st_done[1];
    uint32_t ns = st_not_sent, ntx = st_not_txd, late = st_late, trim = st_trim, ur = st_pcm_underruns, wd = st_wdog;
    uint32_t fr = st_enc_frames, us = st_enc_us_sum, mx = st_enc_us_max;
    uint32_t df = fr - l_frames;
    uint32_t rq = __atomic_load_n(&s_rq_head, __ATOMIC_ACQUIRE) - __atomic_load_n(&s_rq_tail, __ATOMIC_ACQUIRE);
    bool on = s_pub_state == AURACAST_STATE_ON;
    uint32_t f0 = s_sent[0] - s_done[0], f1 = s_sent[1] - s_done[1];   /* racy snapshot, display only */
    uint32_t infl = f0 > f1 ? f0 : f1;
    unsigned ctl_free = on ? r_ble_ll_iso_free_buf_num_get(s_bis[0]) : 0;
    st_stats_window++;                            /* encoder restarts its max */

    ESP_LOGW(TAG, "AURA 2s: %s | SDUs sent L %lu R %lu, done L %lu R %lu (%s), not sent: host %lu ctrl %lu | "
             "late (silence) %lu, trimmed %lu | ready %lu/%d, in flight %lu, ctrl free %u | "
             "LC3 L+R avg %lu us max %lu us (%lu fr) | PCM underruns %lu, wdog %lu, rq full %lu, stack free %u B",
             names[s_pub_state & 3],
             (unsigned long)(sent0 - l_sent[0]), (unsigned long)(sent1 - l_sent[1]),
             (unsigned long)(done0 - l_done[0]), (unsigned long)(done1 - l_done[1]),
             s_vendor_comp ? "NSFC" : "NOCP",
             (unsigned long)(ns - l_not_sent), (unsigned long)(ntx - l_not_txd),
             (unsigned long)(late - l_late), (unsigned long)(trim - l_trim),
             (unsigned long)rq, AURA_RQ_DEPTH, (unsigned long)(on ? infl : 0), ctl_free,
             (unsigned long)(df ? (us - l_us) / df : 0), (unsigned long)mx, (unsigned long)df,
             (unsigned long)(ur - l_ur), (unsigned long)(wd - l_wdog), (unsigned long)st_rq_full,
             s_enc_task ? (unsigned)uxTaskGetStackHighWaterMark(s_enc_task) : 0u);

    l_sent[0] = sent0; l_sent[1] = sent1; l_done[0] = done0; l_done[1] = done1;
    l_not_sent = ns; l_not_txd = ntx; l_late = late; l_trim = trim; l_ur = ur; l_wdog = wd; l_frames = fr; l_us = us;
}
