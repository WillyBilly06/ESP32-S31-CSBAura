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

#include "esp_bt.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lc3.h"

#include "a2dp_in.h"
#include "audio_out.h"
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

/* Controller libs from Nov 2026 (IDF release/v6.1 c43d665a96+): versioned,
 * one entry per SDU with PDUs sent / late. iso.h BT_ISO_TX_CB_INFO_VERSION 1. */
typedef struct {
    uint8_t version;
    uint8_t framing;
    uint8_t sdu_cnt;
    uint8_t sdu_dropped;
    struct {
        uint16_t seq_num;
        uint32_t tx_time_stamp;
        uint32_t sdu_ref_time;
        uint32_t sdu_hdl_time;
        uint32_t sdu_rec_time;
        union {
            struct {
                uint8_t pdu_cnt;
                uint8_t txd_cnt;
                uint8_t late : 1;
                uint8_t more : 1;
                uint32_t sdu_end_time;
            } unf;
            struct {
                uint32_t time_offset : 24;
            } fra;
        };
    } sdu[4];
} ctl_iso_tx_info_v1_t;
extern void r_ble_ll_isoal_tx_comp_cb_set(void *cb);
extern uint16_t r_ble_ll_iso_free_buf_num_get(uint16_t conn_handle);

/* ---- broadcast identity (keep exactly: the Xtreme 5 joins on these) -------- */
/* 1 = plain Auracast source like a phone (no JBL master marker, no JBL PA
 * block, phone BIG layout, stock encoder): the Xtreme 5 then joins as an
 * ordinary Auracast receiver (its Auracast button), not as a JBL party member. */
#ifndef AURA_GENERIC
#define AURA_GENERIC           0
#endif
#define AURA_BROADCAST_ID      (AURA_GENERIC ? 0x5a31c3u : 0x112233u)
#define AURA_BROADCAST_NAME    (AURA_GENERIC ? "S31 Auracast" : "ESP32-S31's JBL Xtreme 5")
#define AURA_NAME_LEN          (AURA_GENERIC ? sizeof("S31 Auracast") - 1 : sizeof("ESP32-S31's JBL Xtreme 5") - 1)
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
/* BIG profile: 1 = exactly the real Xtreme 5 master (its BIGInfo: 20 ms, BN 2,
 * NSE 8, IRC 4, PTO 0, 100 B), 0 = the S26 Ultra's layout (30 ms, BN 3, 120 B,
 * here with IRC 3 / PTO 0). Pair 1 with liblc3 LC3_ENCODER_JBL_COMPAT/JBL_NF. */
#ifndef AURA_JBL_PROFILE
#define AURA_JBL_PROFILE       (!AURA_GENERIC)
#endif
#define AURA_SDU               100        /* octets per LC3 frame = SDU (80 kbps/channel, as the real JBL master) */
#define AURA_PD_US             80000
#define AURA_BIG_HANDLE        0
#define AURA_BN                3          /* SDUs per BIS per BIG event (30 ms) */
#define AURA_LOC_FL            0x00000001u
#define AURA_LOC_FR            0x00000002u
#define AURA_CONTEXT           0x0005u    /* Unspecified | Media, as the Xtreme 5 */

/* The controller allocates BIG state from internal DMA RAM inside Create BIG
 * Test and asserts if that fails: never send it without headroom. */
#define AURA_BIG_MIN_FREE      (8 * 1024)
#define AURA_BIG_MIN_BLOCK     (2 * 1024)

/* ---- pacing ---------------------------------------------------------------- */
#define AURA_PREROLL           (AURA_BN + 1)                   /* one BIG event (BN) + 1 ahead */
#define AURA_PIPELINE          (AURA_PREROLL + 1)  /* SDU pairs encoded or in flight */
#define AURA_ISO_QUEUE         (AURA_PIPELINE + 1) /* BTstack per-BIS cap (controller has 12 for all) */
#define AURA_RQ_DEPTH          8                   /* ready queue, power of 2 */
#define AURA_DEADLINE_MS       8                   /* after a completion: next event's SDUs must be in */
#define AURA_ISO_INTERVAL_MS   30         /* s_big_test.iso_interval_1250us * 1.25 */
/* No grant this long while ON: the BIG has stalled, keep PCM in real time.
 * Grants come once per BIG event (every ISO interval), so this must be a few
 * intervals: at 30 ms (= one interval) every bit of completion jitter fired it,
 * threw 30 ms of audio away and made the next SDUs late (not transmitted). */
#define AURA_WDOG_MS           (4 * AURA_ISO_INTERVAL_MS)

#define AURA_ENC_STACK         7168    /* measured peak ~5 KB */
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
 * [02 02 03] [03 f0 11 NN] [12 11 <17-byte status, YY at +17, TG at +20>].
 * Byte-exact copy of a real Xtreme 5 party master sniffed on 2026-11-22
 * (s31-btstack-speaker S31_SNIFFER): NN +1 per update, YY = clock (1/64 s units mod 200), TG (00/01) varies
 * with no time pattern, updates every 400-1100 ms. */
static const uint8_t jbl_pa_mfg_ad[] = {
    0x1d, 0xff, 0xdf, 0xfd,
    0x02, 0x02, 0x03,
    0x03, 0xf0, 0x11, 0x02,
    0x12, 0x11, 0x01, 0x01, 0x01, 0x00, 0x43, 0x00, 0x01, 0x00, 0x3c,
                0x00, 0xff, 0xff, 0xff, 0x00, 0x24, 0xb9, 0x00,
};
/* A/B test: 0 = periodic data carries the BASE only (no JBL party block whose
 * NN/YY bytes we can only imitate; the real master's YY is not a timer). */
#ifndef AURA_JBL_PA_BLOCK
#define AURA_JBL_PA_BLOCK      (!AURA_GENERIC)
#endif
#define JBL_PA_VOL_OFFSET      6          /* TLV 02 02 <vol>: master volume 0..100 (JBL AudioVolume scale; a partial sweep read 9..62) */
#define JBL_PA_VOL_MAX         100
#define JBL_PA_NN_OFFSET       10
#define JBL_PA_YY_OFFSET       17
#define JBL_PA_TG_OFFSET       20

/* ---- advertising / BIG storage (BT thread) --------------------------------- */
static uint8_t s_ext_adv[7 + 6 + 4 + 2 * (2 + 32) + sizeof(jbl_master_mfg_ad)];

/* Standard Auracast (JBL One "Broadcast to Others"): a plain LE Audio broadcast
 * any Auracast receiver can join - its own Broadcast ID and name, no JBL master
 * marker, no JBL party block in the periodic data, optional broadcast code.
 * Chosen per start; the JBL party broadcast (Auracast button) is the default. */
#define STD_BROADCAST_ID       0x5a31c3u
static bool s_std;
static char s_std_name[33] = "S31 Auracast";
static bool s_std_enc;
static uint8_t s_std_code[16];
static uint16_t s_ext_adv_len;
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
static le_audio_big_test_params_t s_big_run;      /* what Create BIG Test gets this time */
static const le_audio_big_test_params_t s_big_test = {
    /* 20 ms / BN 2 / NSE 8 / IRC 4 / 100 B = the real Xtreme 5 master. Measured
     * beside the phone's A2DP (controller Tpoll fixed 10 ms): this layout 0 underruns
     * with a 120 ms A2DP buffer; 120 B + IRC 4 starves A2DP (54 % BIG duty); a 10 ms
     * BIG starves it too (84-91 pkts / 2 s) and broke connection setup with 120 B. */
    .iso_interval_1250us = 24,            /* 30 ms: one 14.3 ms BIG block + a 15.7 ms gap the phone's 3.56 ms exchanges always fit in */
    .nse = 12,                            /* BN 3 x IRC 4: 4 copies of every frame */
    .max_pdu = AURA_SDU,
    .bn = AURA_BN,
    .irc = 4,
    .pto = 0,       /* skipped 2 BIG events every 10-20 s (4-11 / 90 s), PTO 0: none in 150 s */
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

/* Local I2S in sync with the receivers: a frame read by the encoder now is
 * rendered at (its BIG event ~ our completion of it) + presentation delay.
 * lead = completion time - encode time, measured per SDU pair (FIFO order). */
#define LEAD_SLOTS 64
static int64_t s_prod_us[LEAD_SLOTS];        /* encoder writes, BT thread reads */
static uint16_t s_lead_done;                 /* BT thread: BIS0 completions seen this epoch */
static float s_lead_ms = -1.0f;              /* EMA */
static volatile uint32_t st_lead_ms, st_local_ms;
#define LOCAL_SYNC_TRIM_MS 0                 /* + = S31 later (tune by ear against the Xtreme 5) */

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
/* Lazy (as the codec fork): encoder state exists only while hosting - in
 * LISTEN that internal RAM is what the controller's BIG sync RX needs. */
static void *s_enc_mem[AURA_NUM_BIS];
/* One LC3 workspace shared by both roles (lazy reuse as in the codec fork's
 * one-active-codec slots): the 2 encoders while hosting, the 2 decoders while
 * listening - the S31 never does both. Allocated once at setup, before HCI
 * power-on: allocated later the same encode ran 20x slower. */
static uint8_t *s_lc3_ws;
static size_t s_lc3_ws_size, s_lc3_half;
static SemaphoreHandle_t s_enc_lock;
static volatile bool s_enc_free_req;
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
    little_endian_store_24(d, i, s_std ? STD_BROADCAST_ID : AURA_BROADCAST_ID);
    i += 3;

    d[i++] = 5;                                       /* Public Broadcast Announcement */
    d[i++] = BLUETOOTH_DATA_TYPE_SERVICE_DATA_16_BIT_UUID;
    little_endian_store_16(d, i, AURA_PBA_UUID);
    i += 2;
    d[i++] = s_std ? (uint8_t)(0x04u | (s_std_enc ? 0x01u : 0x00u)) : AURA_PBA_FEATURES;
    d[i++] = 0;                                       /* no metadata */

    d[i++] = 3;
    d[i++] = BLUETOOTH_DATA_TYPE_APPEARANCE;
    little_endian_store_16(d, i, AURA_APPEARANCE);
    i += 2;

    const char *nm = s_std ? s_std_name : AURA_BROADCAST_NAME;
    size_t nl = s_std ? strlen(s_std_name) : AURA_NAME_LEN;
    if (nl > 32) nl = 32;
    d[i++] = (uint8_t)(nl + 1);
    d[i++] = BLUETOOTH_DATA_TYPE_BROADCAST_NAME;
    memcpy(d + i, nm, nl);
    i += nl;

    d[i++] = (uint8_t)(nl + 1);                       /* as the real JBL master */
    d[i++] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
    memcpy(d + i, nm, nl);
    i += nl;

    if (!AURA_GENERIC && !s_std) {
        memcpy(d + i, jbl_master_mfg_ad, sizeof(jbl_master_mfg_ad));   /* the JBL master marker */
        i += sizeof(jbl_master_mfg_ad);
    }
    btstack_assert(i <= sizeof(s_ext_adv));
    s_ext_adv_len = (uint16_t)i;
}

static void pa_build(void)
{
    static const uint8_t codec_id[5] = { HCI_AUDIO_CODING_FORMAT_LC3, 0, 0, 0, 0 };
    /* subgroup, exactly as the real master: 48 kHz, 10 ms, octets, 1 frame block
     * per SDU - and NO subgroup-level channel allocation (we used to send FL|FR
     * here, i.e. "2 channels per BIS" to a receiver that reads the subgroup) */
    static const uint8_t cc_subgroup[] = {
        0x02, 0x01, 0x08,
        0x02, 0x02, 0x01,
        0x03, 0x04, AURA_SDU, 0x00,
        0x02, 0x05, 0x01,
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
    if (!s_std) {
        memcpy(s_pa + s_pa_jbl_pos, jbl_pa_mfg_ad, sizeof(jbl_pa_mfg_ad));
        s_pa_len = s_pa_jbl_pos + sizeof(jbl_pa_mfg_ad);
    } else {
        s_pa_len = s_pa_jbl_pos;                  /* standard: BASE only */
    }
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
    if (s_std) {
        return;                                   /* no JBL party block to tick */
    }
    uint32_t r = esp_random();
    s_pa[s_pa_jbl_pos + JBL_PA_NN_OFFSET] = s_pa_nn;
    /* master volume: members join at it and follow it. Decoded from a sniffed
 * sweep of a real Xtreme 5 master (read 9..62 over a partial sweep); the copied
     * template carried 3, so every member joined at ~5 %. */
    s_pa[s_pa_jbl_pos + JBL_PA_VOL_OFFSET] =
        (uint8_t)(((unsigned)a2dp_in_volume() * JBL_PA_VOL_MAX + 63u) / 127u);
    /* YY is NOT random: the real master's YY is a clock - +~62 per second,
     * wrapping at 200 (1/64 s units mod 200), decoded from 385 sniffed updates.
     * Random values made party members see the master's time jump every update
     * (their volume dived to ~5 %). */
    s_pa[s_pa_jbl_pos + JBL_PA_YY_OFFSET] = (uint8_t)((uint64_t)esp_timer_get_time() * 64u / 1000000u % 200u);
    (void)r;
    /* TG: 1 = active. The real master's TG is not random (runs of 1 with 0/1
     * flicker between); a random 0 here looked like a master not ready yet and
     * a joining Xtreme 5 started very quiet. Constant 1 under test. */
    s_pa[s_pa_jbl_pos + JBL_PA_TG_OFFSET] = 1;
    gap_periodic_advertising_set_data(s_adv_handle, s_pa_len, s_pa);
}

void auracast_volume_changed(void)
{
    if (s_phase == PH_ADV_ON_WAIT || s_phase == PH_BIG_WAIT || s_phase == PH_ON) {
        s_pa_nn++;                                /* members act on a new update counter */
        pa_refresh();
    }
}

static int s_exp;                                 /* bench experiment phase (S31_TEST_SIGNAL) */
#if S31_TEST_SIGNAL
#define EXP_MS 90000
#define AURA_EXP 0                                /* 1 = cycle the adv on/off experiments */
static btstack_timer_source_t s_exp_timer;
static void exp_handler(btstack_timer_source_t *ts);
#endif

static void pa_timer_handler(btstack_timer_source_t *ts)
{
    if (s_phase == PH_OFF || s_phase == PH_ADV_OFF_WAIT) {
        return;
    }
    s_pa_nn++;
    if (s_exp != 1) {
        pa_refresh();
    }
    btstack_run_loop_set_timer(ts, 400 + 100 * (esp_random() % 7));   /* 400-1000 ms, like the master */
    btstack_run_loop_add_timer(ts);
}

/* ======================================================================== */
/* encoder (core 0)                                                          */
/* ======================================================================== */

/* Bench SDU dump: test signal -> one UP-sweep; otherwise 30 s of phone audio
 * starting 1 s after A2DP streaming starts. 0 = off. */
#ifndef AURA_SDU_DUMP
#define AURA_SDU_DUMP 0
#endif
#if AURA_SDU_DUMP
/* Bench: record the left SDUs of one whole UP-sweep (made at 48 kHz) and
 * print them afterwards from a low-priority task as
 * "LC3F <n> 0 0 <len> <base64>" (the s31-auracast-rx capture format). */
#define DUMP_FRAMES (S31_TEST_SIGNAL ? 900 : 3000)
static uint8_t *s_dump;
static volatile int s_dump_n, s_dump_state;   /* 0 wait, 1 recording, 2 printing, 3 done */
static TaskHandle_t s_dump_task;

static void b64(const uint8_t *in, int n, char *out)
{
    static const char t[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    int o = 0;
    for (int i = 0; i < n; i += 3) {
        uint32_t v = (uint32_t)in[i] << 16 | (i + 1 < n ? (uint32_t)in[i + 1] << 8 : 0) | (i + 2 < n ? in[i + 2] : 0);
        out[o++] = t[(v >> 18) & 63];
        out[o++] = t[(v >> 12) & 63];
        out[o++] = i + 1 < n ? t[(v >> 6) & 63] : '=';
        out[o++] = i + 2 < n ? t[v & 63] : '=';
    }
    out[o] = 0;
}

static void dump_task(void *arg)
{
    (void)arg;
    for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ESP_LOGW(TAG, "DUMP: %d frames follow", s_dump_n);
    vTaskDelay(pdMS_TO_TICKS(50));
    static char line[AURA_SDU * 4 / 3 + 8];
    for (int i = 0; i < s_dump_n; i++) {
        b64(s_dump + i * AURA_SDU, AURA_SDU, line);
        printf("LC3F %d 0 0 %d %s\n", i, AURA_SDU, line);
        if ((i & 3) == 3) {
            vTaskDelay(1);
        }
    }
    ESP_LOGW(TAG, "DUMP: end");
    s_dump_state = 3;                             /* re-armed when the phone stream stops */
    }
}

static void dump_frame(const uint8_t *sdu)
{
#if !S31_TEST_SIGNAL
    static int streamed;
    if (s_dump_state == 3 && !a2dp_in_streaming()) {
        s_dump_n = 0;
        s_dump_state = 0;
    }
    if (s_dump == NULL || s_dump_state >= 2) {
        return;
    }
    streamed = a2dp_in_streaming() ? streamed + 1 : 0;
    if (s_dump_state == 0 && streamed >= 100) {
        s_dump_state = 1;
        ESP_LOGW(TAG, "DUMP: recording %d frames of phone audio", DUMP_FRAMES);
    }
    if (s_dump_state == 1) {
        memcpy(s_dump + s_dump_n * AURA_SDU, sdu, AURA_SDU);
        if (++s_dump_n >= DUMP_FRAMES) {
            s_dump_state = 2;
            xTaskNotifyGive(s_dump_task);
        }
    }
    return;
#endif
    static int prev = -1;
    int ph = a2dp_in_test_phase();
    if (s_dump_state == 0 && ph == 0 && prev == 3 && s_dump != NULL) {
        s_dump_state = 1;                         /* second cycle onward: start of an UP-sweep */
    }
    prev = ph;
    if (s_dump_state == 1) {
        if (ph == 0 && s_dump_n < DUMP_FRAMES) {
            memcpy(s_dump + s_dump_n * AURA_SDU, sdu, AURA_SDU);
            s_dump_n++;
        } else if (ph != 0) {
            s_dump_state = 2;
            xTaskNotifyGive(s_dump_task);
        }
    }
}
#endif

static void enc_produce(sdu_pair_t *slot, uint16_t idx)
{
    if (a2dp_in_read(s_pcm, AURA_FRAME_SAMPLES) < AURA_FRAME_SAMPLES && a2dp_in_streaming()) {
        st_pcm_underruns++;
    }
    if (!S31_TEST_SIGNAL && v5_eq_process != NULL) {
        v5_eq_process(s_pcm, AURA_FRAME_SAMPLES);
    }
    s_prod_us[idx % LEAD_SLOTS] = esp_timer_get_time();
    audio_out_push(AO_HOST, s_pcm, AURA_FRAME_SAMPLES);   /* same audio, played here in sync */
    int64_t t0 = esp_timer_get_time();
    xSemaphoreTake(s_enc_lock, portMAX_DELAY);
    for (int ch = 0; ch < AURA_NUM_BIS; ch++) {
        if (s_enc[ch] == NULL ||
            lc3_encode(s_enc[ch], LC3_PCM_FORMAT_S16, s_pcm + ch, 2, AURA_SDU, slot->sdu[ch]) != 0) {
            memcpy(slot->sdu[ch], s_silence, AURA_SDU);
        }
    }
    xSemaphoreGive(s_enc_lock);
#if AURA_SDU_DUMP
    dump_frame(slot->sdu[0]);
#endif
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
            enc_produce(slot, produced);
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
                audio_out_push(AO_HOST, s_pcm, AURA_FRAME_SAMPLES);
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

/* SDU time grid, as Espressif's own S31 broadcast source (esp-gmf esp_bt_audio
 * bt_audio_le_tx_group.c): anchor (seq, controller timestamp) from LE Read ISO
 * TX Sync, every SDU stamped anchor_ts + (seq - anchor_seq) * SDU interval, so
 * the controller puts each SDU in exactly its own payload slot instead of
 * "the next free one". Re-read every second, re-anchored if > 4 SDUs off. */
#ifndef AURA_SDU_TIMESTAMPS
#define AURA_SDU_TIMESTAMPS    1
#endif
static bool s_ts_valid;
static uint32_t s_ts_anchor;
static uint16_t s_seq_anchor;
static btstack_timer_source_t s_sync_timer;
static uint32_t st_ts_reanchor, st_ts_reads;
static int32_t st_ts_drift;

static void sync_timer_handler(btstack_timer_source_t *ts)
{
    if (s_phase != PH_ON) {
        return;
    }
    if (hci_can_send_command_packet_now()) {
        hci_send_cmd(&hci_le_read_iso_tx_sync, s_bis[0]);
        btstack_run_loop_set_timer(ts, 1000);
    } else {
        btstack_run_loop_set_timer(ts, 50);
    }
    btstack_run_loop_add_timer(ts);
}

static void on_tx_sync(const uint8_t *rp)
{
    if (rp[0] != 0) {
        ESP_LOGW(TAG, "LE Read ISO TX Sync: status 0x%02x (no SDU sent yet?)", rp[0]);
        return;
    }
    uint16_t seq = little_endian_read_16(rp, 3);
    uint32_t ts = little_endian_read_32(rp, 5);
    uint32_t off = little_endian_read_24(rp, 9);
    st_ts_reads++;
    if (s_ts_valid) {
        uint32_t expect = s_ts_anchor + (uint32_t)(int32_t)(int16_t)(uint16_t)(seq - s_seq_anchor) * AURA_FRAME_US;
        int32_t d = (int32_t)(ts - expect);
        st_ts_drift = d;
        if (d > 4 * AURA_FRAME_US || d < -4 * AURA_FRAME_US) {
            ESP_LOGW(TAG, "TX sync: seq %u ts %lu is %ld us off the grid -> re-anchor", seq, (unsigned long)ts, (long)d);
            st_ts_reanchor++;
            s_ts_anchor = ts;
            s_seq_anchor = seq;
        }
        return;
    }
    s_ts_anchor = ts;
    s_seq_anchor = seq;
    s_ts_valid = AURA_SDU_TIMESTAMPS;
    ESP_LOGW(TAG, "TX sync anchor: seq %u at %lu us (time offset %lu us) -> SDUs now %s", seq, (unsigned long)ts,
             (unsigned long)off, s_ts_valid ? "time-stamped" : "unstamped (AURA_SDU_TIMESTAMPS 0)");
}

static void send_sdu(int bis)
{
    hci_reserve_packet_buffer();
    uint8_t *b = hci_get_outgoing_packet_buffer();
    uint16_t n;
    if (s_ts_valid) {
        uint32_t ts = s_ts_anchor + (uint32_t)(int32_t)(int16_t)(uint16_t)(s_seq[bis] - s_seq_anchor) * AURA_FRAME_US;
        little_endian_store_16(b, 0, (uint16_t)s_bis[bis] | (2u << 12) | (1u << 14));   /* complete SDU, time stamp */
        little_endian_store_16(b, 2, 8 + AURA_SDU);
        little_endian_store_32(b, 4, ts);
        little_endian_store_16(b, 8, s_seq[bis]);
        little_endian_store_16(b, 10, AURA_SDU);
        memcpy(b + 12, s_tx[bis], AURA_SDU);
        n = 12 + AURA_SDU;
    } else {
        little_endian_store_16(b, 0, (uint16_t)s_bis[bis] | (2u << 12));   /* complete SDU, no time stamp */
        little_endian_store_16(b, 2, 4 + AURA_SDU);
        little_endian_store_16(b, 4, s_seq[bis]);
        little_endian_store_16(b, 6, AURA_SDU);
        memcpy(b + 8, s_tx[bis], AURA_SDU);
        n = 8 + AURA_SDU;
    }
    uint8_t st = hci_send_iso_packet_buffer(n);
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
static void lead_update(void)
{
    uint16_t done = (uint16_t)s_done[0];
    int64_t now = esp_timer_get_time();
    while (s_lead_done != done) {
        int64_t d = now - s_prod_us[s_lead_done % LEAD_SLOTS];
        s_lead_done++;
        if (d > 0 && d < 400000) {
            float ms = (float)d / 1000.0f;
            s_lead_ms = s_lead_ms < 0.0f ? ms : s_lead_ms + (ms - s_lead_ms) * 0.02f;
        }
    }
    if (s_lead_ms >= 0.0f) {
        uint32_t local = (uint32_t)(s_lead_ms + 0.5f) + AURA_PD_US / 1000u + LOCAL_SYNC_TRIM_MS;
        st_lead_ms = (uint32_t)(s_lead_ms + 0.5f);
        if (local != st_local_ms) {
            st_local_ms = local;
            audio_out_set_delay(AO_HOST, local);
        }
    }
}

static void completions_processed(void)
{
    lead_update();
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

/* completion timing on BIS 0 (controller task writes, stats reads) */
static volatile uint32_t ct_long_total;
static volatile uint32_t st_late_ctl, st_partial;
static volatile bool s_ctl_v1;
static volatile uint32_t sk_gap, sk_comp, sk_ntx, sk_cnt, sk_seq, sk_ts, sk_free, sk_infl;
static volatile bool sk_new;
static volatile uint32_t ct_n, ct_pkts, ct_gap_min = UINT32_MAX, ct_gap_max, ct_long, ct_cb_multi;
static volatile uint64_t ct_gap_sum;
static int64_t ct_last;
static volatile uint32_t al_seq[AURA_NUM_BIS], al_ts[AURA_NUM_BIS], al_ref[AURA_NUM_BIS], al_cnt[AURA_NUM_BIS], al_pk[AURA_NUM_BIS];

/* Controller task: vendor TX completion (non-standard flow control). Only
 * counts and wakes the BT thread; no locks held by the controller are taken. */
static void ctl_tx_comp_cb(uint16_t conn_handle, void *info, size_t size)
{
    if (info == NULL || size < 4) {
        return;
    }
    /* normalise old (Jul 2026) and versioned (Nov 2026) controller layouts */
    uint32_t comp, ntx = 0, late = 0, cnt;
    uint16_t seq0 = 0;
    uint32_t ts0 = 0, ref0 = 0;
    if (size == sizeof(ctl_iso_tx_info_v1_t) && ((const ctl_iso_tx_info_v1_t *)info)->version >= 1) {
        const ctl_iso_tx_info_v1_t *v = (const ctl_iso_tx_info_v1_t *)info;
        cnt = v->sdu_cnt > 4 ? 4 : v->sdu_cnt;
        comp = cnt + v->sdu_dropped;
        for (uint32_t k = 0; k < cnt; k++) {
            if (v->framing == 0) {
                if (v->sdu[k].unf.txd_cnt == 0) ntx++;
                if (v->sdu[k].unf.late) late++;
                if (v->sdu[k].unf.txd_cnt < v->sdu[k].unf.pdu_cnt) st_partial++;
            }
        }
        if (cnt) {
            seq0 = v->sdu[0].seq_num;
            ts0 = v->sdu[0].tx_time_stamp;
            ref0 = v->sdu[0].sdu_ref_time;
        }
        s_ctl_v1 = true;
    } else {
        const ctl_iso_tx_info_t *o = (const ctl_iso_tx_info_t *)info;
        comp = (uint32_t)o->completed_pkts + o->not_txd_pkts;
        ntx = o->not_txd_pkts;
        cnt = o->pkt_cnt;
        if (cnt) {
            seq0 = o->pkt[0].seq_num;
            ts0 = o->pkt[0].tx_time_stamp;
            ref0 = o->pkt[0].sdu_ref_time;
        }
    }
    for (int i = 0; i < AURA_NUM_BIS; i++) {
        if (conn_handle == s_bis_ctl[i]) {
            if (cnt > 0) {
                al_seq[i] = seq0;
                al_ts[i] = ts0;
                al_ref[i] = ref0;
                al_pk[i] = cnt;
                al_cnt[i]++;
            }
            st_late_ctl += late;
            if (i == 0) {
                int64_t now = esp_timer_get_time();
                if (ct_last != 0) {
                    uint32_t g = (uint32_t)(now - ct_last);
                    if (g > 1000) {                /* a new BIG event (not a 2nd callback of the same one) */
                        ct_n++;
                        ct_gap_sum += g;
                        if (g < ct_gap_min) ct_gap_min = g;
                        if (g > ct_gap_max) ct_gap_max = g;
                        if (g > AURA_ISO_INTERVAL_MS * 1500) {
                            ct_long++;
                            ct_long_total++;
                            sk_gap = g; sk_comp = comp; sk_ntx = ntx;
                            sk_cnt = cnt; sk_seq = seq0; sk_ts = ts0;
                            sk_free = r_ble_ll_iso_free_buf_num_get(s_bis_ctl[0]);
                            sk_infl = s_sent[0] - s_done[0];
                            sk_new = true;
                        }
                    } else {
                        ct_cb_multi++;
                    }
                }
                ct_last = now;
                ct_pkts += comp;
            }
            __atomic_fetch_add(&s_comp_pending[i], comp, __ATOMIC_RELEASE);
            st_not_txd += ntx;
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
    s_lead_done = 0;
    for (int i = 0; i < AURA_NUM_BIS; i++) {
        __atomic_store_n(&s_comp_pending[i], 0, __ATOMIC_RELEASE);
        if (!run) {
            s_bis_ctl[i] = HCI_CON_HANDLE_INVALID;   /* late controller completions are ignored */
        }
    }
    s_tx_valid = false;
    s_csn_requested = false;
    s_silence_due = 0;
    s_ts_valid = false;                           /* new BIG / stop: re-anchor the SDU time grid */
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
        audio_out_end(AO_HOST);

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

/* BT thread: (re)create the encoders for a new hosting session */
static bool encoders_alloc(void)
{
    bool ok = true;
    xSemaphoreTake(s_enc_lock, portMAX_DELAY);
    for (int ch = 0; ch < AURA_NUM_BIS; ch++) {
        s_enc[ch] = lc3_setup_encoder(AURA_FRAME_US, AURA_SR_HZ, 0, s_enc_mem[ch]);
        ok = ok && s_enc[ch] != NULL;
    }
    xSemaphoreGive(s_enc_lock);
    return ok;
}

/* BT thread, Auracast OFF: hand the workspace to the decoder side. */
void *auracast_lc3_workspace_take(size_t *half)
{
    if (auracast_state() != AURACAST_STATE_OFF) {
        return NULL;
    }
    xSemaphoreTake(s_enc_lock, portMAX_DELAY);
    for (int ch = 0; ch < AURA_NUM_BIS; ch++) {
        s_enc[ch] = NULL;                         /* encoder no longer valid: its memory is reused */
    }
    xSemaphoreGive(s_enc_lock);
    if (half) *half = s_lc3_half;
    return s_lc3_ws;
}

static int begin_start(void)
{
    if (!encoders_alloc()) {
        ESP_LOGE(TAG, "no RAM for the LC3 encoders");
        return -ENOMEM;
    }
    if (!big_heap_ok()) {
        return -ENOMEM;
    }
    if (!s_adv_created) {
        if (gap_extended_advertising_setup(&s_adv_set, &s_ext_params, &s_adv_handle) != ERROR_CODE_SUCCESS) {
            return -EIO;
        }
        gap_periodic_advertising_set_params(s_adv_handle, &s_pa_params);
        s_adv_created = true;
        ESP_LOGI(TAG, "advertising set handle %u (SID %u)", s_adv_handle, AURA_ADV_SID);
    }
    /* JBL party or standard broadcast: rebuild both every start (they were built
     * once at setup, so a standard broadcast went on air as the JBL party) */
    ext_adv_build();
    pa_build();
    gap_extended_advertising_set_adv_data(s_adv_handle, s_ext_adv_len, s_ext_adv);
    gap_periodic_advertising_set_data(s_adv_handle, s_pa_len, s_pa);
    log_hex("EXT ADV", s_ext_adv, s_ext_adv_len);
    log_hex(s_std ? "PA (BASE)" : "PA (BASE + JBL)", s_pa, s_pa_len);
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
    s_big_params.encryption = (s_std && s_std_enc) ? 1 : 0;
    memcpy(s_big_params.broadcast_code, s_std_code, 16);
    s_big_run = s_big_test;
    if (a2dp_in_ext_active() && s_std) {   /* JBL party: the Xtreme 5 does not join IRC 2 */
        /* the phone's LE Audio CIS (its own timing) has to fit next to the BIG: with
         * 4 copies the BIG block is 14.3 ms of every 30 and the two kept colliding
         * (7-18 % of the phone's SDUs lost, BIG events skipped, local sync off).
         * 2 copies = the S26 Ultra's own broadcast, which the Xtreme 5 plays cleanly. */
        s_big_run.irc = 2;
        s_big_run.nse = AURA_BN * 2;
        ESP_LOGW(TAG, "input is LE Audio (standard broadcast): BIG with IRC 2 (NSE %u) to leave room for the CIS",
                 s_big_run.nse);
    }
    uint8_t st = gap_big_create_test(&s_big, &s_big_params, &s_big_run);
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
    bt_sched_tune_init();                         /* new controller libs: ISO priority block exists only once ISO is up */
    s_ts_valid = false;
    btstack_run_loop_remove_timer(&s_sync_timer);
    btstack_run_loop_set_timer_handler(&s_sync_timer, sync_timer_handler);
    btstack_run_loop_set_timer(&s_sync_timer, 300);        /* after the first SDUs went out */
    btstack_run_loop_add_timer(&s_sync_timer);
    for (int i = 0; i < AURA_NUM_BIS; i++) {          /* per-handle TX power for the BIS, if the controller takes it */
        esp_err_t e = esp_ble_tx_power_set_enhanced(ESP_BLE_ENHANCED_PWR_TYPE_CONN, s_bis[i], ESP_PWR_LVL_P20);
        ESP_LOGW(TAG, "BIS 0x%04x TX power +20 dBm: %s (reads back level %d)", s_bis[i], esp_err_to_name(e),
                 esp_ble_tx_power_get_enhanced(ESP_BLE_ENHANCED_PWR_TYPE_CONN, s_bis[i]));
    }
    ESP_LOGW(TAG, "broadcast ON (%s): '%s' Broadcast_ID 0x%06x, BIS 0x%04x/0x%04x, ISO interval %u us",
             s_std ? "standard Auracast" : "JBL party", s_std ? s_std_name : AURA_BROADCAST_NAME,
             (unsigned)(s_std ? STD_BROADCAST_ID : AURA_BROADCAST_ID), s_bis[0], s_bis[1],
             gap_subevent_big_created_get_iso_interval_1250us(packet) * 1250u);
    s_lead_ms = -1.0f;
    st_local_ms = AURA_PIPELINE * 10u + AURA_PD_US / 1000u + LOCAL_SYNC_TRIM_MS;
    audio_out_begin(AO_HOST, st_local_ms);
    notify(AURACAST_STATE_ON, 0);
#if S31_TEST_SIGNAL
    s_exp = 0;
    btstack_run_loop_remove_timer(&s_exp_timer);
    btstack_run_loop_set_timer_handler(&s_exp_timer, exp_handler);
#if AURA_EXP
    btstack_run_loop_set_timer(&s_exp_timer, EXP_MS);
    btstack_run_loop_add_timer(&s_exp_timer);
#endif
#endif
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
        if (hci_event_command_complete_get_command_opcode(packet) == HCI_OPCODE_HCI_LE_READ_ISO_TX_SYNC && size >= 5 + 12) {
            on_tx_sync(hci_event_command_complete_get_return_parameters(packet));
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
    log_hex("EXT ADV", s_ext_adv, s_ext_adv_len);
    log_hex("PA (BASE + JBL)", s_pa, s_pa_len);

    /* hot encoder state: internal RAM, allocated HERE (before HCI power-on).
     * Allocated later (lazily at host start) the same encode ran 20x slower
     * (lc3_enc 99 % of core 0 next to btdm 97 % of core 1): keep it at setup. */
    const unsigned esz = lc3_encoder_size(AURA_FRAME_US, AURA_SR_HZ);
    const unsigned dsz = lc3_decoder_size(AURA_FRAME_US, AURA_SR_HZ);
    s_enc_lock = xSemaphoreCreateMutex();
    s_lc3_half = ((esz > dsz ? esz : dsz) + 15u) & ~15u;
    s_lc3_ws_size = 2 * s_lc3_half;
    s_lc3_ws = heap_caps_aligned_alloc(16, s_lc3_ws_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    btstack_assert(s_lc3_ws != NULL);
    for (int ch = 0; ch < AURA_NUM_BIS; ch++) {
        s_enc_mem[ch] = s_lc3_ws + ch * s_lc3_half;
    }
    ESP_LOGW(TAG, "LC3 workspace %u B at %p (2 x max(encoder %u, decoder %u)): encoders when hosting, decoders when listening",
             (unsigned)s_lc3_ws_size, s_lc3_ws, esz, dsz);
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
#if AURA_SDU_DUMP
    s_dump = heap_caps_malloc(DUMP_FRAMES * AURA_SDU, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    xTaskCreatePinnedToCore(dump_task, "sdu_dump", 4096, NULL, 1, &s_dump_task, 0);
#endif

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

#if S31_TEST_SIGNAL
/* Bench: 90 s each - 0 normal, 1 PA data frozen, 2 JBL beacon off,
 * 3 Auracast ext adv (set 2) off; BIG skips counted per phase. */
#include "jbl_beacon.h"
static void exp_handler(btstack_timer_source_t *ts)
{
    static uint32_t last;
    static const char *names[] = { "normal", "PA data frozen", "JBL beacon off", "Auracast ext adv off" };
    uint32_t n = ct_long_total;
    ESP_LOGW(TAG, "EXP result: '%s' -> %lu BIG skips in %d s", names[s_exp], (unsigned long)(n - last), EXP_MS / 1000);
    last = n;
    /* undo the old phase */
    if (s_exp == 2) jbl_beacon_pause(false);
    if (s_exp == 3) gap_extended_advertising_start(s_adv_handle, 0, 0);
    s_exp = (s_exp + 1) & 3;
    if (s_exp == 2) jbl_beacon_pause(true);
    if (s_exp == 3) gap_extended_advertising_stop(s_adv_handle);
    ESP_LOGW(TAG, "EXP start: '%s'", names[s_exp]);
    btstack_run_loop_set_timer(ts, EXP_MS);
    btstack_run_loop_add_timer(ts);
}
#endif

void auracast_set_standard(bool standard, const char *name, const uint8_t *code)
{
    s_std = standard;
    if (name && name[0]) {
        size_t k = strlen(name);
        if (k > 32) k = 32;
        memcpy(s_std_name, name, k);
        s_std_name[k] = 0;
    }
    s_std_enc = standard && code != NULL;
    memset(s_std_code, 0, sizeof(s_std_code));
    if (s_std_enc) memcpy(s_std_code, code, 16);
    ESP_LOGW(TAG, "next broadcast: %s%s%s%s", standard ? "standard Auracast '" : "JBL party",
             standard ? s_std_name : "", standard ? "'" : "", s_std_enc ? ", encrypted" : "");
}

bool auracast_is_standard(void) { return s_std; }

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

    {
        uint32_t n = ct_n, mn = ct_gap_min, mx2 = ct_gap_max, lg = ct_long, mc = ct_cb_multi, pk = ct_pkts;
        uint64_t sum = ct_gap_sum;
        ct_n = 0; ct_gap_sum = 0; ct_gap_min = UINT32_MAX; ct_gap_max = 0; ct_long = 0; ct_cb_multi = 0; ct_pkts = 0;
        ESP_LOGW(TAG, "BIG timing (BIS0 completions): %lu events, gap avg %lu.%03lu ms min %lu us max %lu us, >1.5 itvl %lu, same-event cbs %lu, SDUs %lu (%lu.%02lu per event) | SDU ts %s, sync reads %lu, drift %ld us, re-anchors %lu | ctl %s: late %lu, partly unsent %lu",
                 (unsigned long)n, (unsigned long)(n ? sum / n / 1000 : 0), (unsigned long)(n ? (sum / n) % 1000 : 0),
                 (unsigned long)(n ? mn : 0), (unsigned long)mx2, (unsigned long)lg, (unsigned long)mc, (unsigned long)pk,
                 (unsigned long)(n ? pk / n : 0), (unsigned long)(n ? (pk * 100 / n) % 100 : 0),
                 s_ts_valid ? "ON" : "off", (unsigned long)st_ts_reads, (long)st_ts_drift, (unsigned long)st_ts_reanchor,
                 s_ctl_v1 ? "v1" : "old", (unsigned long)st_late_ctl, (unsigned long)st_partial);
    }
    if (sk_new) {
        sk_new = false;
        ESP_LOGW(TAG, "SKIP: gap %lu us, then completed %lu not_txd %lu (pkt_cnt %lu, seq %lu tx %lu) | ctrl free %lu, host in flight %lu",
                 (unsigned long)sk_gap, (unsigned long)sk_comp, (unsigned long)sk_ntx, (unsigned long)sk_cnt,
                 (unsigned long)sk_seq, (unsigned long)sk_ts, (unsigned long)sk_free, (unsigned long)sk_infl);
    }
    {
        uint32_t s0 = al_seq[0], t0 = al_ts[0], r0 = al_ref[0], s1 = al_seq[1], t1 = al_ts[1], r1 = al_ref[1];
        /* L/R alignment: tx time of BIS1 minus BIS0, normalised to the same SDU (10 ms per seq) */
        int32_t off = (int32_t)(t1 - t0) - (int32_t)(int16_t)(uint16_t)(s1 - s0) * AURA_FRAME_US;
        int32_t roff = (int32_t)(r1 - r0) - (int32_t)(int16_t)(uint16_t)(s1 - s0) * AURA_FRAME_US;
        ESP_LOGW(TAG, "L/R align: BIS0 seq %lu tx %lu ref %lu | BIS1 seq %lu tx %lu ref %lu | tx offset %ld us, ref offset %ld us (pkt_cnt %lu/%lu)",
                 (unsigned long)s0, (unsigned long)t0, (unsigned long)r0, (unsigned long)s1, (unsigned long)t1, (unsigned long)r1,
                 (long)off, (long)roff, (unsigned long)al_pk[0], (unsigned long)al_pk[1]);
    }
    l_sent[0] = sent0; l_sent[1] = sent1; l_done[0] = done0; l_done[1] = done1;
    l_not_sent = ns; l_not_txd = ntx; l_late = late; l_trim = trim; l_ur = ur; l_wdog = wd; l_frames = fr; l_us = us;
}
