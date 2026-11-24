/*
 * LE Audio unicast sink - BAP unicast server for one phone.
 *
 *   PACS  one LC3 sink record: 16/24/32/48 kHz, 7.5 + 10 ms, 1-2 channels per
 *         CIS, 26-155 octets, 1 frame per SDU; locations FL|FR; contexts
 *         unspecified | media | game (no source: the S31 has no microphone)
 *   ASCS  one sink ASE carrying both channels on one CIS (the controller is
 *         configured for few CIS; Android supports this "one stereo ASE" setup)
 *   VCS   volume 0..255 <-> the speaker volume 0..127 (a2dp_in), mute
 *   CAS   empty (CAP acceptor, not part of a set)
 *   TMAS  TMAP role: Unicast Media Receiver
 *
 * Audio: the CIS SDUs are queued into a2dp_in's DSP task (ext source) and
 * decoded there with liblc3, so LE Audio takes the same path as phone A2DP:
 * resampler, PCM ring, local speaker, and the JBL party broadcast when it is on.
 *
 * Advertising: its own legacy connectable set on the PUBLIC address (= the
 * Classic address), with the ASCS / CAS / TMAS announcements, so Android ties
 * it to the Classic-paired speaker and offers "LE Audio" for it.
 */
#include "lls.h"
#include "le_audio.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "hci_dump.h"
#include "esp_timer.h"
#include "esp_cpu.h"

#include "lc3.h"

#include "a2dp_in.h"
#include "jbl_gatt.h"
#include "jbl_v5.h"
#include "tones.h"
#include "jbl.h"
#include "party.h"

static const char *TAG = "LE_AUDIO";

/* ---- capabilities ------------------------------------------------------------- */
#define LA_SINK_CONTEXTS   0x000Du                   /* unspecified | media | game */
#define LA_LOCATIONS       0x00000003u               /* front left | front right */
#define LA_TMAP_ROLE       0x0008u                   /* Unicast Media Receiver */
#define LA_APPEARANCE      0x0841u                   /* standalone speaker */
#define LA_MAX_SDU         310u                      /* 2 x 155 octets */
#define LA_ASE_ID          1

static const uint8_t s_sink_pac[] = {
    1,                                               /* one PAC record */
    0x06, 0x00, 0x00, 0x00, 0x00,                    /* LC3 */
    19,                                              /* codec specific capabilities */
    0x03, 0x01, 0xB4, 0x00,                          /*   rates: 16, 24, 32, 48 kHz */
    0x02, 0x02, 0x03,                                /*   frame durations: 7.5 + 10 ms */
    0x02, 0x03, 0x03,                                /*   channel counts: 1, 2 */
    0x05, 0x04, 26, 0x00, 155, 0x00,                 /*   octets per frame 26..155 */
    0x02, 0x05, 0x01,                                /*   1 frame per SDU */
    4,                                               /* metadata */
    0x03, 0x01, LA_SINK_CONTEXTS & 0xFF, LA_SINK_CONTEXTS >> 8,   /* preferred contexts */
};

/* ---- ASE ------------------------------------------------------------------------ */
typedef enum { ASE_IDLE = 0, ASE_CODEC = 1, ASE_QOS = 2, ASE_ENABLING = 3, ASE_STREAMING = 4,
               ASE_DISABLING = 5, ASE_RELEASING = 6 } ase_state_t;
static const char *const s_state_names[] = { "Idle", "Codec Configured", "QoS Configured", "Enabling",
                                             "Streaming", "Disabling", "Releasing" };

static struct {
    ase_state_t state;
    /* codec configuration */
    uint8_t target_latency, target_phy;
    uint8_t cc[32];
    uint8_t cc_len;
    uint32_t sr_hz;
    uint16_t dt_us;
    uint16_t opf;                                    /* octets per frame (per channel) */
    uint8_t nch;
    /* QoS */
    uint8_t cig, cis;
    uint32_t sdu_itvl;
    uint8_t framing, phy, rtn;
    uint16_t max_sdu, latency;
    uint32_t pd;
    /* metadata */
    uint8_t md[32];
    uint8_t md_len;
} s_ase;

static bool s_enabled;
static hci_con_handle_t s_con = HCI_CON_HANDLE_INVALID;    /* LE link that drives the ASE */
static hci_con_handle_t s_cis = HCI_CON_HANDLE_INVALID;
static bool s_audio_on;
static bool s_got_first;                                    /* see le_audio_iso_rx */
static uint32_t s_iso_us = 30000;                           /* CIS ISO interval */
static uint16_t s_ccc[LA_N];                                /* one client at a time */

/* ---- volume (VCS) ------------------------------------------------------------- */
static uint8_t s_vol_counter;
static uint8_t s_vol_last, s_mute_last;
static bool s_vol_force;                             /* notify even if unchanged (after a phone write) */
static btstack_timer_source_t s_vol_timer;

/* ---- LC3 decode (DSP task) ------------------------------------------------------- */
/* LC3 decoder states: in the a2dp_in decoder workspace (internal SRAM) */
static lc3_decoder_t s_dec[2];
static int s_dec_nch, s_dec_n, s_dec_opf;
static uint32_t s_dec_sr;
static int16_t s_lr[2 * 480];

/* ---- stats --------------------------------------------------------------------- */
static volatile uint32_t st_rx, st_lost, st_bad, st_plc, st_cp_ops;

/* ---- advertising --------------------------------------------------------------- */
#define LA_ADV_SID         3
static le_advertising_set_t s_adv_set;
static uint8_t s_adv_handle = 0xFF;
static bool s_adv_on;
static uint8_t s_adv[31], s_adv_len, s_rsp[31], s_rsp_len;
static le_extended_advertising_parameters_t s_adv_params = {
    .advertising_event_properties = 0x13,          /* legacy ADV_IND */
    .primary_advertising_interval_min = 160,       /* 100 ms */
    .primary_advertising_interval_max = 160,
    .primary_advertising_channel_map = 0x07,
    .own_address_type = BD_ADDR_TYPE_LE_PUBLIC,    /* = the Classic address */
    .peer_address_type = BD_ADDR_TYPE_LE_PUBLIC,
    .advertising_filter_policy = 0,
    .advertising_tx_power = 127,
    .primary_advertising_phy = 1,
    .secondary_advertising_phy = 1,
    .advertising_sid = LA_ADV_SID,
    .scan_request_notification_enable = 0,
};
static btstack_packet_callback_registration_t s_hci_reg;

/* ================================================================================ */

static void connected_tone(void);
static void mcp_start_ms(uint32_t ms);
/* Play over LE (media control client) state, see mcp_play() */
static btstack_timer_source_t s_mcp_timer;
static gatt_client_service_t s_mcs_svc;
static gatt_client_characteristic_t s_mcs_cp_chr;
static gatt_client_notification_t s_mcs_notif;
static bool s_mcs_found, s_mcs_cp_found, s_mcp_done, s_mcs_listening;
static uint8_t s_mcp_state;                  /* 0 idle, 1 service, 2 char, 3 CCCD, 4 Play write */
static uint8_t s_mcp_op = 0x01;                      /* MCS opcode to write: 0x01 Play, 0x02 Pause */
static uint32_t s_last_gatt_ms;                               /* last GATT write by the phone */
static hci_con_handle_t s_paired_con = HCI_CON_HANDLE_INVALID;   /* LE link that just paired */

static void notify(le_audio_attr_t a, const uint8_t *d, uint16_t n)
{
    if (s_con == HCI_CON_HANDLE_INVALID || (s_ccc[a] & 1) == 0) return;
    jbl_gatt_notify(s_con, jbl_gatt_le_audio_handle(a), d, n);
}

static uint16_t ase_value(uint8_t *v)
{
    uint16_t i = 0;
    v[i++] = LA_ASE_ID;
    v[i++] = (uint8_t)s_ase.state;
    switch (s_ase.state) {
    case ASE_CODEC:
        v[i++] = 0x00;                               /* unframed ISOAL PDUs supported */
        v[i++] = 0x02;                               /* preferred PHY: 2M */
        /* next to a 4-copy BIG (14.3 ms of every 30) the CIS loses slots: as many
         * retransmissions as BAP's high-reliability configs use */
        v[i++] = 13;                                 /* preferred retransmissions */
        little_endian_store_16(v, i, 100); i += 2;   /* max transport latency 100 ms */
        little_endian_store_24(v, i, 20000); i += 3; /* presentation delay min 20 ms */
        little_endian_store_24(v, i, 400000); i += 3; /*                   max 400 ms */
        little_endian_store_24(v, i, 0); i += 3;     /* no preferred range */
        little_endian_store_24(v, i, 0); i += 3;
        v[i++] = 0x06; v[i++] = 0; v[i++] = 0; v[i++] = 0; v[i++] = 0;
        v[i++] = s_ase.cc_len;
        memcpy(v + i, s_ase.cc, s_ase.cc_len); i += s_ase.cc_len;
        break;
    case ASE_QOS:
        v[i++] = s_ase.cig;
        v[i++] = s_ase.cis;
        little_endian_store_24(v, i, s_ase.sdu_itvl); i += 3;
        v[i++] = s_ase.framing;
        v[i++] = s_ase.phy;
        little_endian_store_16(v, i, s_ase.max_sdu); i += 2;
        v[i++] = s_ase.rtn;
        little_endian_store_16(v, i, s_ase.latency); i += 2;
        little_endian_store_24(v, i, s_ase.pd); i += 3;
        break;
    case ASE_ENABLING:
    case ASE_STREAMING:
    case ASE_DISABLING:
        v[i++] = s_ase.cig;
        v[i++] = s_ase.cis;
        v[i++] = s_ase.md_len;
        memcpy(v + i, s_ase.md, s_ase.md_len); i += s_ase.md_len;
        break;
    default:
        break;
    }
    return i;
}

static void ase_set(ase_state_t st, const char *why)
{
    if (st != s_ase.state) {
        ESP_LOGW(TAG, "ASE %s -> %s (%s)", s_state_names[s_ase.state], s_state_names[st], why);
    }
    s_ase.state = st;
    uint8_t v[64];
    notify(LA_ASE, v, ase_value(v));
}

/* ---- audio ----------------------------------------------------------------------- */
static volatile bool s_dec_setup;                    /* new stream: set the decoders up (DSP task) */
static bool dec_setup(void);

static void la_decode(const uint8_t *d, uint16_t len)       /* DSP task */
{
    if (s_dec_setup || !a2dp_in_codec_ws_take(2)) {          /* new stream, or AAC had the workspace */
        s_dec_setup = false;
        if (!dec_setup()) {
            ESP_LOGE(TAG, "LC3 decoder setup failed (%lu Hz, %u us)", (unsigned long)s_ase.sr_hz, s_ase.dt_us);
            s_dec[0] = s_dec[1] = NULL;
            return;
        }
    }
    int n = s_dec_n;
    bool ok = len >= (uint16_t)(s_dec_nch * s_dec_opf) && len > 0;
    int64_t t0 = esp_timer_get_time();
    for (int c = 0; c < s_dec_nch; c++) {
        if (s_dec[c] == NULL) return;
        lc3_decode(s_dec[c], ok ? d + c * s_dec_opf : NULL, ok ? s_dec_opf : 0, LC3_PCM_FORMAT_S16, s_lr + c, 2);
    }
    {
        static uint32_t us_sum, us_max, cnt;
        uint32_t us = (uint32_t)(esp_timer_get_time() - t0);
        us_sum += us; cnt++;
        if (us > us_max) us_max = us;
        if (cnt == 1000) {
            ESP_LOGI(TAG, "LC3 decode: avg %lu us, max %lu us per SDU (%u ch, core %d)", (unsigned long)(us_sum / cnt),
                     (unsigned long)us_max, s_dec_nch, (int)esp_cpu_get_core_id());
            a2dp_in_probe_log("LC3");
            us_sum = us_max = cnt = 0;
        }
    }
    if (!ok) st_plc++;
    if (s_dec_nch == 1) {
        for (int i = 0; i < n; i++) s_lr[2 * i + 1] = s_lr[2 * i];
    }
    a2dp_in_ext_pcm(s_lr, n, (int)s_dec_sr);
}

/* BT thread: stream parameters only; the decoders are set up on the DSP task (they
 * live in the shared decoder workspace, which AAC uses at other times) */
static bool audio_prepare(void)
{
    unsigned sz = lc3_decoder_size(10000, 48000), ws = 0;    /* the largest we accept */
    a2dp_in_codec_ws(&ws);
    if (2 * ((sz + 15u) & ~15u) > ws) {
        ESP_LOGE(TAG, "LC3 decoders need %u B > workspace %u B", 2 * sz, ws);
        return false;
    }
    s_dec_nch = s_ase.nch ? s_ase.nch : 1;
    s_dec_opf = s_ase.opf;
    s_dec_sr = s_ase.sr_hz;
    s_dec_n = (int)(s_ase.sr_hz / 100u * s_ase.dt_us / 10000u);
    if (s_dec_n > 480) s_dec_n = 480;
    s_dec_setup = true;
    return true;
}

static bool dec_setup(void)                                  /* DSP task */
{
    unsigned ws = 0;
    uint8_t *base = (uint8_t *)a2dp_in_codec_ws(&ws);
    unsigned step = (lc3_decoder_size(10000, 48000) + 15u) & ~15u;
    a2dp_in_codec_ws_take(2);
    for (int c = 0; c < 2; c++) {
        s_dec[c] = c < s_dec_nch ? lc3_setup_decoder(s_ase.dt_us, (int)s_ase.sr_hz, 0, base + c * step) : NULL;
        if (c < s_dec_nch && s_dec[c] == NULL) return false;
    }
    return true;
}

static void audio_start(void)
{
    if (s_audio_on) return;
    if (!audio_prepare()) {
        ESP_LOGE(TAG, "LC3 decoder setup failed (%lu Hz, %u us)", (unsigned long)s_ase.sr_hz, s_ase.dt_us);
        return;
    }
    s_audio_on = true;
    s_got_first = false;
    st_rx = st_lost = st_bad = st_plc = 0;
    /* the phone delivers one ISO interval of SDUs per burst; the broadcast encoder
     * also pulls in ISO-interval bursts, so hosting needs a second interval */
    /* a2dp_in adds the second interval itself while the broadcast is on */
    a2dp_in_ext_start(&la_decode, "LE Audio", s_iso_us / 1000u + 20u);
    ESP_LOGW(TAG, "LE Audio stream: LC3 %lu Hz, %u.%u ms, %u ch, %u B/frame, presentation delay %lu us",
             (unsigned long)s_ase.sr_hz, s_ase.dt_us / 1000, (s_ase.dt_us % 1000) / 100, s_dec_nch, s_ase.opf,
             (unsigned long)s_ase.pd);
}

static void audio_stop(const char *why)
{
    if (!s_audio_on) return;
    s_audio_on = false;
    a2dp_in_ext_stop();
    ESP_LOGW(TAG, "LE Audio stream stopped (%s)", why);
}

/* the first CIS events carry no audio yet (the phone starts sending ~150 ms after the CIS
 * is up): the controller reports them as empty / lost. They are not lost music - concealing
 * them padded the buffer with ~160 ms of PLC and made every stream start "lose" ~16 SDUs. */
bool le_audio_iso_rx(const uint8_t *packet, uint16_t size)
{
    if (size < 4 || s_cis == HCI_CON_HANDLE_INVALID) return false;
    uint16_t hdr = little_endian_read_16(packet, 0);
    if ((hdr & 0x0fff) != s_cis) return false;
    if (!s_audio_on) return true;
    uint8_t pb = (hdr >> 12) & 3;
    uint16_t off = 4 + (((hdr >> 14) & 1) ? 4 : 0);
    if (pb != 2 || size < off + 4) {                 /* fragments: not expected (SDU <= 310 B) */
        st_bad++;
        return true;
    }
    uint16_t lp = little_endian_read_16(packet, off + 2);
    uint16_t len = lp & 0x0fff;
    uint8_t ps = (uint8_t)(lp >> 14);
    off += 4;
    if (ps != 0 || len == 0 || size < off + len) {
        if (!s_got_first) return true;               /* before the phone's first SDU */
        st_lost++;
        a2dp_in_ext_push(NULL, 0);                   /* PLC */
        return true;
    }
    st_rx++;
    s_got_first = true;
    a2dp_in_ext_push(packet + off, len);
    return true;
}

/* ---- ASE control point --------------------------------------------------------------- */
enum { RSP_OK = 0, RSP_UNSUPPORTED_OP = 1, RSP_INVALID_LEN = 2, RSP_INVALID_ASE = 3, RSP_BAD_TRANSITION = 4,
       RSP_BAD_DIRECTION = 5, RSP_UNSUPPORTED_CAPS = 6, RSP_UNSUPPORTED_PARAM = 7, RSP_REJECTED_PARAM = 8,
       RSP_INVALID_PARAM = 9, RSP_UNSUPPORTED_MD = 10, RSP_REJECTED_MD = 11, RSP_INVALID_MD = 12,
       RSP_NO_RESOURCES = 13, RSP_UNSPECIFIED = 14 };

static uint32_t sr_from_index(uint8_t i)
{
    switch (i) {
    case 0x03: return 16000;
    case 0x05: return 24000;
    case 0x06: return 32000;
    case 0x08: return 48000;
    default:   return 0;
    }
}

/* parse + check an LC3 codec configuration; reason byte for a rejection */
static uint8_t check_codec(const uint8_t *cc, uint8_t n, uint8_t *reason)
{
    uint32_t sr = 0;
    uint16_t dt = 10000, opf = 0;
    uint32_t alloc = 0;
    bool have_alloc = false;
    uint8_t blocks = 1;
    for (uint8_t i = 0; i + 1 < n;) {
        uint8_t l = cc[i], t = cc[i + 1];
        if (l == 0 || i + 1 + l > n) { *reason = 0x02; return RSP_INVALID_PARAM; }
        const uint8_t *v = cc + i + 2;
        switch (t) {
        case 0x01: sr = sr_from_index(v[0]); if (sr == 0) { *reason = 0x02; return RSP_UNSUPPORTED_PARAM; } break;
        case 0x02: dt = v[0] == 0 ? 7500 : v[0] == 1 ? 10000 : 0; if (!dt) { *reason = 0x02; return RSP_UNSUPPORTED_PARAM; } break;
        case 0x03: if (l >= 5) { alloc = little_endian_read_32(v, 0); have_alloc = true; } break;
        case 0x04: if (l >= 3) opf = little_endian_read_16(v, 0); break;
        case 0x05: blocks = v[0]; break;
        default: break;
        }
        i += 1 + l;
    }
    uint8_t nch = have_alloc ? (uint8_t)__builtin_popcount(alloc) : 1;
    if (nch == 0) nch = 1;
    if (sr == 0 || opf < 26 || opf > 155 || nch > 2 || blocks != 1) {
        *reason = 0x02;
        return RSP_UNSUPPORTED_PARAM;
    }
    s_ase.sr_hz = sr;
    s_ase.dt_us = dt;
    s_ase.opf = opf;
    s_ase.nch = nch;
    return RSP_OK;
}

static void cp_respond(uint8_t op, uint8_t code, uint8_t reason)
{
    uint8_t r[5] = { op, 1, LA_ASE_ID, code, reason };
    notify(LA_ASE_CP, r, sizeof(r));
}

static void cp_respond_bad(uint8_t op, uint8_t code)
{
    uint8_t r[5] = { op, 0xFF, 0x00, code, 0x00 };
    notify(LA_ASE_CP, r, sizeof(r));
}

static void ase_control(hci_con_handle_t con, const uint8_t *b, uint16_t n)
{
    if (n < 2) { cp_respond_bad(n ? b[0] : 0, RSP_INVALID_LEN); return; }
    s_con = con;
    st_cp_ops++;
    uint8_t op = b[0], num = b[1];
    if (op < 1 || op > 8) { cp_respond_bad(op, RSP_UNSUPPORTED_OP); return; }
    if (num != 1 || n < 3) { cp_respond_bad(op, RSP_INVALID_LEN); return; }
    if (b[2] != LA_ASE_ID) { cp_respond(op, RSP_INVALID_ASE, 0); return; }
    const uint8_t *p = b + 3, *e = b + n;
    uint8_t reason = 0;
    switch (op) {
    case 0x01: {                                     /* Config Codec */
        if (e - p < 8 || e - p < 8 + p[7]) { cp_respond_bad(op, RSP_INVALID_LEN); return; }
        if (s_ase.state != ASE_IDLE && s_ase.state != ASE_CODEC && s_ase.state != ASE_QOS) {
            cp_respond(op, RSP_BAD_TRANSITION, 0); return;
        }
        if (p[2] != 0x06) { cp_respond(op, RSP_UNSUPPORTED_CAPS, 0x01); return; }   /* LC3 only */
        uint8_t code = check_codec(p + 8, p[7], &reason);
        if (code != RSP_OK) { cp_respond(op, code, reason); return; }
        s_ase.target_latency = p[0];
        s_ase.target_phy = p[1];
        s_ase.cc_len = p[7] > sizeof(s_ase.cc) ? sizeof(s_ase.cc) : p[7];
        memcpy(s_ase.cc, p + 8, s_ase.cc_len);
        cp_respond(op, RSP_OK, 0);
        ase_set(ASE_CODEC, "Config Codec");
        ESP_LOGW(TAG, "codec: LC3 %lu Hz, %u us, %u ch, %u B/frame (target latency %u)", (unsigned long)s_ase.sr_hz,
                 s_ase.dt_us, s_ase.nch, s_ase.opf, s_ase.target_latency);
        return;
    }
    case 0x02:                                       /* Config QoS */
        if (e - p < 15) { cp_respond_bad(op, RSP_INVALID_LEN); return; }
        if (s_ase.state != ASE_CODEC && s_ase.state != ASE_QOS) { cp_respond(op, RSP_BAD_TRANSITION, 0); return; }
        s_ase.cig = p[0];
        s_ase.cis = p[1];
        s_ase.sdu_itvl = little_endian_read_24(p, 2);
        s_ase.framing = p[5];
        s_ase.phy = p[6];
        s_ase.max_sdu = little_endian_read_16(p, 7);
        s_ase.rtn = p[9];
        s_ase.latency = little_endian_read_16(p, 10);
        s_ase.pd = little_endian_read_24(p, 12);
        ESP_LOGW(TAG, "phone QoS: SDU every %lu us, max SDU %u B, RTN %u, max transport latency %u ms, "
                 "presentation delay %lu us, PHY 0x%02x", (unsigned long)s_ase.sdu_itvl, s_ase.max_sdu, s_ase.rtn,
                 s_ase.latency, (unsigned long)s_ase.pd, s_ase.phy);
        if (s_ase.max_sdu > LA_MAX_SDU) { cp_respond(op, RSP_UNSUPPORTED_PARAM, 0x04); return; }
        cp_respond(op, RSP_OK, 0);
        ase_set(ASE_QOS, "Config QoS");
        return;
    case 0x03:                                       /* Enable */
        if (e - p < 1 || e - p < 1 + p[0]) { cp_respond_bad(op, RSP_INVALID_LEN); return; }
        if (s_ase.state != ASE_QOS) { cp_respond(op, RSP_BAD_TRANSITION, 0); return; }
        if (!s_enabled) { cp_respond(op, RSP_NO_RESOURCES, 0); return; }
        s_ase.md_len = p[0] > sizeof(s_ase.md) ? sizeof(s_ase.md) : p[0];
        memcpy(s_ase.md, p + 1, s_ase.md_len);
        cp_respond(op, RSP_OK, 0);
        ase_set(ASE_ENABLING, "Enable");
        if (s_cis != HCI_CON_HANDLE_INVALID) {       /* CIS already up: we are the receiver, start */
            ase_set(ASE_STREAMING, "CIS up");
            audio_start();
        }
        return;
    case 0x04:                                       /* Receiver Start Ready: source ASEs only */
    case 0x06:                                       /* Receiver Stop Ready: source ASEs only */
        cp_respond(op, RSP_BAD_DIRECTION, 0);
        return;
    case 0x05:                                       /* Disable: a sink ASE goes back to QoS Configured */
        if (s_ase.state != ASE_ENABLING && s_ase.state != ASE_STREAMING) { cp_respond(op, RSP_BAD_TRANSITION, 0); return; }
        cp_respond(op, RSP_OK, 0);
        audio_stop("Disable");
        ase_set(ASE_QOS, "Disable");
        return;
    case 0x07:                                       /* Update Metadata */
        if (e - p < 1 || e - p < 1 + p[0]) { cp_respond_bad(op, RSP_INVALID_LEN); return; }
        if (s_ase.state != ASE_ENABLING && s_ase.state != ASE_STREAMING) { cp_respond(op, RSP_BAD_TRANSITION, 0); return; }
        s_ase.md_len = p[0] > sizeof(s_ase.md) ? sizeof(s_ase.md) : p[0];
        memcpy(s_ase.md, p + 1, s_ase.md_len);
        cp_respond(op, RSP_OK, 0);
        ase_set(s_ase.state, "Update Metadata");
        return;
    case 0x08:                                       /* Release */
        if (s_ase.state == ASE_IDLE || s_ase.state == ASE_RELEASING) { cp_respond(op, RSP_BAD_TRANSITION, 0); return; }
        cp_respond(op, RSP_OK, 0);
        audio_stop("Release");
        ase_set(ASE_RELEASING, "Release");
        if (s_cis == HCI_CON_HANDLE_INVALID) ase_set(ASE_IDLE, "released");   /* else when the CIS goes */
        return;
    default:
        cp_respond_bad(op, RSP_UNSUPPORTED_OP);
        return;
    }
}

/* ---- volume ------------------------------------------------------------------------ */
static uint8_t vol_setting(void)
{
    uint8_t v = a2dp_in_volume();
    return v >= 127 ? 255 : (uint8_t)(v * 2);
}

static uint16_t vol_state(uint8_t *v)
{
    v[0] = vol_setting();
    v[1] = a2dp_in_muted() ? 1 : 0;
    v[2] = s_vol_counter;
    return 3;
}

static void vol_check(void)
{
    uint8_t v = vol_setting(), m = a2dp_in_muted() ? 1 : 0;
    if (s_vol_force || v != s_vol_last || m != s_mute_last) {   /* changed (buttons, AVRCP, app, the phone) */
        s_vol_force = false;
        s_vol_last = v;
        s_mute_last = m;
        s_vol_counter++;
        uint8_t st[3];
        notify(LA_VOL_STATE, st, vol_state(st));
        ESP_LOGI(TAG, "volume state -> phone: %u%s, counter %u%s", st[0], st[1] ? " muted" : "", st[2],
                 (s_ccc[LA_VOL_STATE] & 1) ? "" : " (not subscribed)");
    }
}

static void vol_tick(btstack_timer_source_t *ts)
{
    vol_check();
    btstack_run_loop_set_timer(ts, 300);
    btstack_run_loop_add_timer(ts);
}

static int vol_control(const uint8_t *b, uint16_t n)
{
    if (n < 2) return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
    ESP_LOGW(TAG, "phone volume op 0x%02x, counter %u (ours %u)%s%u", b[0], b[1], s_vol_counter,
             n >= 3 ? ", value " : "", n >= 3 ? b[2] : 0);
    if (b[1] != s_vol_counter) return 0x80;          /* Invalid Change Counter */
    int v = vol_setting();
    bool mute = a2dp_in_muted();
    const int step = 16;
    switch (b[0]) {
    case 0x00: v -= step; break;
    case 0x01: v += step; break;
    case 0x02: v -= step; mute = false; break;
    case 0x03: v += step; mute = false; break;
    case 0x04: if (n < 3) return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH; v = b[2]; break;
    case 0x05: mute = false; break;
    case 0x06: mute = true; break;
    default: return 0x81;                            /* Opcode Not Supported */
    }
    bool past_end = v < 0 || v > 255;                /* relative step beyond an end stop */
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    uint8_t was = a2dp_in_volume();
    uint8_t now = (uint8_t)(v >= 255 ? 127 : v / 2);
    a2dp_in_set_volume(now, "LE Audio volume");
    if ((now == 127 || now == 0) && (was != now || past_end)) {
        tones_play("vol_max");                       /* end stop, as for the Classic phone slider */
    }
    a2dp_in_set_mute(mute);
    s_vol_force = true;                              /* every accepted write moves the counter */
    vol_check();
    return 0;
}

/* ---- GATT ------------------------------------------------------------------------- */
/* ---- BASS: scan delegator for the phone's "Listen to audio broadcast" -------------------
 * The phone (broadcast assistant) finds an Auracast broadcast and writes Add Source; we
 * join it with the JBL One assistant's machinery (party_assist_join: PA + BIG sync, our own
 * broadcast off) and report progress in the one Broadcast Receive State. An encrypted
 * broadcast: we report "Broadcast_Code required", the phone asks the user and writes Set
 * Broadcast Code, we join again with it. */
#define BASS_SRC_ID 1
#define BASS_SUB_MAX 2
#define BASS_META_MAX 24
#define BASS_RS_MAX (16 + 16 + BASS_SUB_MAX * (5 + BASS_META_MAX))
static struct {
    bool used;
    uint8_t addr_type;
    bd_addr_t addr;                     /* BTstack order */
    uint8_t sid;
    uint32_t bcast_id;
    uint8_t pa_req;                     /* 0 do not sync, 1/2 sync */
    uint8_t nsub;
    uint32_t bis_req[BASS_SUB_MAX];
    uint8_t meta_len[BASS_SUB_MAX];
    uint8_t meta[BASS_SUB_MAX][BASS_META_MAX];
    bool has_code;
    uint8_t code[16];                   /* party order (reversed HCI order) */
    uint8_t pa_state;                   /* 0 not synced, 2 synced, 3 failed */
    uint8_t enc;                        /* 0 none, 1 code required, 2 decrypting, 3 bad code */
    bool joined;
} s_bass;

static uint16_t bass_rs_value(uint8_t *v)
{
    if (!s_bass.used) return 0;
    uint16_t i = 0;
    v[i++] = BASS_SRC_ID;
    v[i++] = s_bass.addr_type;
    reverse_bd_addr(s_bass.addr, v + i); i += 6;
    v[i++] = s_bass.sid;
    little_endian_store_24(v, i, s_bass.bcast_id); i += 3;
    v[i++] = s_bass.pa_state;
    v[i++] = s_bass.enc;
    if (s_bass.enc == 3) { memset(v + i, 0, 16); i += 16; }   /* Bad_Code */
    v[i++] = s_bass.nsub;
    party_assist_t as;
    party_assist_get(&as);
    uint32_t mask = s_bass.joined ? (as.bis_mask ? as.bis_mask : 0x3u) : 0;
    bool given = false;
    for (int k = 0; k < s_bass.nsub; k++) {
        uint32_t st = given ? 0 : (mask & s_bass.bis_req[k]);
        if (!given && !st && k == s_bass.nsub - 1) st = mask;     /* none matched: the last one */
        if (st) given = true;
        little_endian_store_32(v, i, st); i += 4;
        v[i++] = s_bass.meta_len[k];
        memcpy(v + i, s_bass.meta[k], s_bass.meta_len[k]); i += s_bass.meta_len[k];
    }
    return i;
}

static void bass_notify(void)
{
    uint8_t v[BASS_RS_MAX];
    uint16_t n = bass_rs_value(v);
    notify(LA_BASS_RS, v, n);
    ESP_LOGW(TAG, "BASS receive state: %s PA %u, encryption %u%s", s_bass.used ? "source," : "empty,",
             s_bass.pa_state, s_bass.enc, s_bass.joined ? ", playing" : "");
}

static void bass_join(void)
{
    s_bass.joined = false;
    s_bass.pa_state = 0;
    if (s_bass.enc != 1 || s_bass.has_code) s_bass.enc = 0;
    ESP_LOGW(TAG, "BASS: phone asks us to listen to broadcast %s (SID %u, Broadcast_ID 0x%06lx)%s",
             bd_addr_to_str(s_bass.addr), s_bass.sid, (unsigned long)s_bass.bcast_id,
             s_bass.has_code ? " with its Broadcast_Code" : "");
    party_bass_join(s_bass.addr, s_bass.addr_type, s_bass.has_code ? s_bass.code : NULL);
}

/* subgroups of Add / Modify Source: [BIS_Sync 4][Metadata_Length][Metadata] each */
static bool bass_subgroups(const uint8_t *p, const uint8_t *e, uint8_t nsub)
{
    uint8_t keep = nsub > BASS_SUB_MAX ? BASS_SUB_MAX : nsub;   /* more: only the first ones */
    for (int k = 0; k < nsub; k++) {
        if (e - p < 5) return false;
        uint32_t bis = little_endian_read_32(p, 0);
        uint8_t ml = p[4];
        p += 5;
        if (e - p < ml) return false;
        if (k < keep) {
            s_bass.bis_req[k] = bis;
            s_bass.meta_len[k] = ml > BASS_META_MAX ? BASS_META_MAX : ml;
            memcpy(s_bass.meta[k], p, s_bass.meta_len[k]);
        }
        p += ml;
    }
    s_bass.nsub = keep;
    return true;
}

static int bass_control(hci_con_handle_t con, const uint8_t *b, uint16_t n)
{
    if (n < 1) return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
    if (gap_get_connection_type(con) != GAP_CONNECTION_ACL) s_con = con;
    const uint8_t *e = b + n;
    switch (b[0]) {
    case 0x00: case 0x01:                                 /* Remote Scan Stopped / Started */
        ESP_LOGI(TAG, "BASS: phone %s scanning for us", b[0] ? "started" : "stopped");
        return 0;
    case 0x02: {                                          /* Add Source */
        if (n < 16) return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
        bool was = s_bass.used;
        bd_addr_t a;
        reverse_bd_addr(b + 2, a);
        bool same = was && bd_addr_cmp(a, s_bass.addr) == 0;
        s_bass.used = true;
        s_bass.addr_type = b[1];
        bd_addr_copy(s_bass.addr, a);
        s_bass.sid = b[8];
        s_bass.bcast_id = little_endian_read_24(b, 9);
        s_bass.pa_req = b[12];
        if (!same) { s_bass.has_code = false; s_bass.enc = 0; }
        if (!bass_subgroups(b + 16, e, b[15])) return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
        s_bass.joined = false;
        s_bass.pa_state = 0;
        if (s_bass.pa_req) bass_join();
        else if (was) party_assist_leave();
        bass_notify();
        return 0;
    }
    case 0x03: {                                          /* Modify Source */
        if (n < 6) return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
        if (!s_bass.used || b[1] != BASS_SRC_ID) return 0x81;   /* Invalid Source_ID */
        s_bass.pa_req = b[2];
        if (!bass_subgroups(b + 6, e, b[5])) return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
        bool any_bis = false;
        for (int k = 0; k < s_bass.nsub; k++) any_bis |= s_bass.bis_req[k] != 0;
        if (!s_bass.pa_req || !any_bis) {
            ESP_LOGW(TAG, "BASS: phone stops our broadcast listening");
            party_assist_leave();
            s_bass.joined = false;
            s_bass.pa_state = 0;
        } else if (!s_bass.joined && !party_assist_active()) {
            bass_join();
        }
        bass_notify();
        return 0;
    }
    case 0x04:                                            /* Set Broadcast Code */
        if (n < 18) return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
        if (!s_bass.used || b[1] != BASS_SRC_ID) return 0x81;
        for (int i = 0; i < 16; i++) s_bass.code[i] = b[2 + 15 - i];   /* HCI order -> party order */
        s_bass.has_code = true;
        ESP_LOGW(TAG, "BASS: phone sent the Broadcast_Code");
        if (s_bass.pa_req) bass_join();
        bass_notify();
        return 0;
    case 0x05:                                            /* Remove Source */
        if (n < 2) return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
        if (!s_bass.used || b[1] != BASS_SRC_ID) return 0x81;
        ESP_LOGW(TAG, "BASS: phone removed the broadcast source");
        party_assist_leave();
        memset(&s_bass, 0, sizeof(s_bass));
        bass_notify();
        return 0;
    default:
        return 0x80;                                      /* Opcode Not Supported */
    }
}

void le_audio_bass_assist_result(uint8_t action, uint8_t status)
{
    if (!s_bass.used) return;
    if (action == 0) {                                    /* left */
        s_bass.joined = false;
        s_bass.pa_state = 0;
    } else {
        switch (status) {
        case 1: {
            party_assist_t as;
            party_assist_get(&as);
            s_bass.joined = true;
            s_bass.pa_state = 2;
            s_bass.enc = as.encrypted ? 2 : 0;
            break;
        }
        case 3: s_bass.joined = false; s_bass.pa_state = 2; s_bass.enc = 3; break;
        case 4: s_bass.joined = false; s_bass.pa_state = 2; s_bass.enc = 1; s_bass.has_code = false; break;
        default: s_bass.joined = false; s_bass.pa_state = 3; break;
        }
    }
    bass_notify();
}

uint16_t le_audio_att_read(hci_con_handle_t con, le_audio_attr_t a, bool ccc, uint16_t offset,
                           uint8_t *buf, uint16_t size)
{
    (void)con;
    if (ccc) return att_read_callback_handle_little_endian_16(s_ccc[a], offset, buf, size);
    uint8_t v[64];
    uint16_t n = 0;
    switch (a) {
    case LA_PAC:   return att_read_callback_handle_blob(s_sink_pac, sizeof(s_sink_pac), offset, buf, size);
    case LA_LOC:   little_endian_store_32(v, 0, LA_LOCATIONS); n = 4; break;
    case LA_AVAIL: little_endian_store_16(v, 0, s_enabled ? LA_SINK_CONTEXTS : 0); little_endian_store_16(v, 2, 0); n = 4; break;
    case LA_SUPP:  little_endian_store_16(v, 0, LA_SINK_CONTEXTS); little_endian_store_16(v, 2, 0); n = 4; break;
    case LA_ASE:   n = ase_value(v); break;
    case LA_VOL_STATE: n = vol_state(v); break;
    case LA_VOL_FLAGS: v[0] = 0x01; n = 1; break; /* volume setting persisted */
    case LA_TMAP:  little_endian_store_16(v, 0, LA_TMAP_ROLE); n = 2; break;
    case LA_BASS_RS: {
        uint8_t w[BASS_RS_MAX];
        uint16_t k = bass_rs_value(w);
        return att_read_callback_handle_blob(w, k, offset, buf, size);
    }
    default: return 0;
    }
    return att_read_callback_handle_blob(v, n, offset, buf, size);
}

int le_audio_att_write(hci_con_handle_t con, le_audio_attr_t a, bool ccc, const uint8_t *buf, uint16_t len)
{
    s_last_gatt_ms = btstack_run_loop_get_time_ms();      /* handshake activity */
    if (ccc) {
        if (len >= 2) s_ccc[a] = little_endian_read_16(buf, 0);
        ESP_LOGI(TAG, "phone %s notifications for attribute %d", (len >= 2 && (buf[0] & 3)) ? "enables" : "disables", (int)a);
        if (gap_get_connection_type(con) != GAP_CONNECTION_ACL) s_con = con;   /* the LE Audio client */

        return 0;
    }
    switch (a) {
    case LA_ASE_CP: ase_control(con, buf, len); return 0;
    case LA_BASS_CP: {
        int r = bass_control(con, buf, len);
        if (r) ESP_LOGW(TAG, "BASS control point op 0x%02x refused: 0x%02x", len ? buf[0] : 0, r);
        return r;
    }
    case LA_VOL_CP: {
        s_con = con;
        int r = vol_control(buf, len);
        if (r) ESP_LOGW(TAG, "phone volume write refused: ATT error 0x%02x", r);
        return r;
    }
    default: return ATT_ERROR_WRITE_NOT_PERMITTED;
    }
}

/* ---- advertising -------------------------------------------------------------------- */
static bool s_discoverable = true;
/* LE Audio switched OFF: a phone that used LE Audio reconnects only over LE, to our public
 * address. For the switch's pairing window that address stays connectable (no LE Audio data),
 * so the phone gets there, its encryption fails "key missing" (the bonds are gone) and it
 * drops its old entry, as with the Xtreme 5 - instead of showing "Disconnected" forever */
static bool s_reach;
static hci_con_handle_t s_reach_con = HCI_CON_HANDLE_INVALID;
static bool s_adv_full;                                  /* the advert holds the LE Audio data */

static void adv_build(void)
{
    uint8_t *d = s_adv;
    unsigned i = 0;
    s_adv_full = s_enabled;
    if (!s_enabled) {                                    /* reachable only: flags + appearance */
        d[i++] = 2; d[i++] = BLUETOOTH_DATA_TYPE_FLAGS;
        d[i++] = 0x18;                                   /* not discoverable, dual mode */
        d[i++] = 3; d[i++] = BLUETOOTH_DATA_TYPE_APPEARANCE;
        little_endian_store_16(d, i, LA_APPEARANCE); i += 2;
        s_adv_len = (uint8_t)i;
        s_rsp_len = 0;
        return;
    }
    d[i++] = 2; d[i++] = BLUETOOTH_DATA_TYPE_FLAGS;
    d[i++] = s_discoverable ? 0x1A : 0x18;   /* (LE general discoverable) + LE/BR-EDR dual mode;
                                              * bonded phones reconnect without the general flag */
    d[i++] = 9; d[i++] = BLUETOOTH_DATA_TYPE_SERVICE_DATA_16_BIT_UUID;
    little_endian_store_16(d, i, ORG_BLUETOOTH_SERVICE_AUDIO_STREAM_CONTROL_SERVICE); i += 2;
    /* BAP announcement type: general while pairing; TARGETED otherwise - Android only
     * auto-reconnects a bonded LE Audio device on a targeted announcement (it saw our
     * general one after a reboot and stayed "Disconnected" until Connect was tapped) */
    d[i++] = s_discoverable ? 0x00 : 0x01;
    little_endian_store_16(d, i, LA_SINK_CONTEXTS); i += 2;
    little_endian_store_16(d, i, 0); i += 2;
    d[i++] = 0;                                      /* no metadata */
    d[i++] = 4; d[i++] = BLUETOOTH_DATA_TYPE_SERVICE_DATA_16_BIT_UUID;
    little_endian_store_16(d, i, ORG_BLUETOOTH_SERVICE_COMMON_AUDIO_SERVICE); i += 2;
    d[i++] = s_discoverable ? 0x00 : 0x01;           /* CAP: same announcement type */
    d[i++] = 5; d[i++] = BLUETOOTH_DATA_TYPE_SERVICE_DATA_16_BIT_UUID;
    little_endian_store_16(d, i, ORT_BLUETOOTH_SERVICE_TMAS_SERVICE); i += 2;
    little_endian_store_16(d, i, LA_TMAP_ROLE); i += 2;
    d[i++] = 3; d[i++] = BLUETOOTH_DATA_TYPE_APPEARANCE;
    little_endian_store_16(d, i, LA_APPEARANCE); i += 2;
    s_adv_len = (uint8_t)i;

    const char *nm = jbl_device_name();
    size_t nl = strlen(nm);
    if (nl > 29) nl = 29;
    s_rsp[0] = (uint8_t)(nl + 1);
    s_rsp[1] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
    memcpy(s_rsp + 2, nm, nl);
    s_rsp_len = (uint8_t)(nl + 2);
}

void le_audio_set_discoverable(bool on)
{
    if (s_discoverable == on) return;
    s_discoverable = on;
    if (s_adv_on && s_adv_handle != 0xFF) {
        adv_build();
        gap_extended_advertising_set_adv_data(s_adv_handle, s_adv_len, s_adv);
    }
}

/* stereo pair secondary: no phone links, the radio belongs to the primary's broadcast */
static bool s_quiet;
static void adv_update(void);

void le_audio_set_quiet(bool on)
{
    if (on == s_quiet) return;
    s_quiet = on;
    ESP_LOGW(TAG, "LE Audio advertising %s (stereo pair secondary)", on ? "held" : "released");
    if (hci_get_state() == HCI_STATE_WORKING) adv_update();
}

static void adv_update(void)
{
    bool want = !s_quiet && (s_enabled || s_reach) && s_con == HCI_CON_HANDLE_INVALID && s_reach_con == HCI_CON_HANDLE_INVALID;
    if (want && s_adv_on && s_adv_full != s_enabled) {   /* LE Audio <-> reachable only: new data */
        adv_build();
        gap_extended_advertising_set_adv_data(s_adv_handle, s_adv_len, s_adv);
        gap_extended_advertising_set_scan_response_data(s_adv_handle, s_rsp_len, s_rsp);
        ESP_LOGW(TAG, "LE advertising now %s", s_enabled ? "LE Audio" : "reachable only (LE Audio switch)");
    }
    if (want == s_adv_on) return;
    if (want) {
        if (s_adv_handle == 0xFF) {
            uint8_t err = gap_extended_advertising_setup(&s_adv_set, &s_adv_params, &s_adv_handle);
            if (err != ERROR_CODE_SUCCESS) {
                ESP_LOGE(TAG, "no advertising set for LE Audio (0x%02x)", err);
                return;
            }
        }
        adv_build();
        gap_extended_advertising_set_adv_data(s_adv_handle, s_adv_len, s_adv);
        gap_extended_advertising_set_scan_response_data(s_adv_handle, s_rsp_len, s_rsp);
        gap_extended_advertising_start(s_adv_handle, 0, 0);
        ESP_LOGW(TAG, "advertising %s (set %u, public address)",
                 s_enabled ? "LE Audio" : "reachable only, for phones with the old bond", s_adv_handle);
    } else if (s_adv_handle != 0xFF) {
        gap_extended_advertising_stop(s_adv_handle);
        ESP_LOGW(TAG, "LE Audio advertising off");
    }
    s_adv_on = want;
}

/* ---- HCI events ---------------------------------------------------------------------- */
static void link_gone(const char *why)
{
    audio_stop(why);
    s_cis = HCI_CON_HANDLE_INVALID;
    s_con = HCI_CON_HANDLE_INVALID;
    s_paired_con = HCI_CON_HANDLE_INVALID;
    s_mcp_done = false;
    s_mcp_state = 0;
    s_mcs_cp_found = false;
    if (s_mcs_listening) {
        gatt_client_stop_listening_for_characteristic_value_updates(&s_mcs_notif);
        s_mcs_listening = false;
    }
    memset(s_ccc, 0, sizeof(s_ccc));
    s_ase.state = ASE_IDLE;
    s_adv_on = false;                                /* the set stopped when the phone connected */
    adv_update();
}

/* ---- phone name ----------------------------------------------------------------------
 * JBL One lists a speaker as "connected to this phone" when the beacon's srcName1Crc is
 * the CRC of the phone's own Bluetooth name. A2DP gets the name by a remote name request
 * at connect; an LE Audio phone has no BR/EDR link, so ask its identity (public) address
 * the same way. (A GATT read of 0x2A00 went unanswered on a Galaxy and the phone then
 * dropped the link.) */
static btstack_timer_source_t s_name_timer;
static uint8_t s_name_tries;

static bool phone_identity(bd_addr_t out)
{
    int idx = sm_le_device_index(s_con);
    if (idx >= 0) {
        int type;
        bd_addr_t a;
        le_device_db_info(idx, &type, a, NULL);
        if (type == BD_ADDR_TYPE_LE_PUBLIC) {
            bd_addr_copy(out, a);
            return true;
        }
    }
    hci_connection_t *c = hci_connection_for_handle(s_con);
    if (c != NULL && c->address_type == BD_ADDR_TYPE_LE_PUBLIC) {
        bd_addr_copy(out, c->address);
        return true;
    }
    return false;                                     /* random identity: no BR/EDR name */
}

static void name_read(btstack_timer_source_t *ts)
{
    (void)ts;
    if (s_con == HCI_CON_HANDLE_INVALID || a2dp_in_connected()) return;   /* A2DP: its own name */
    bd_addr_t a;
    if (!phone_identity(a)) {
        ESP_LOGI(TAG, "LE Audio phone has no public identity address: no name for the JBL beacon");
        return;
    }
    if (!jbl_request_phone_name(a) && ++s_name_tries < 3) {
        btstack_run_loop_set_timer(&s_name_timer, 2000);   /* another name request pending */
        btstack_run_loop_add_timer(&s_name_timer);
    }
}

/* ---- PAC refresh ---------------------------------------------------------------------
 * Android keeps the sink PAC it read at first discovery. If that read failed, it keeps an
 * EMPTY PAC ("No matching PAC record" -> MEDIA unavailable, never configures the ASE) and
 * does not read it again. A PAC notification overwrites its copy: send one per connect,
 * once the MTU is big enough for the whole record in one notification. */
static btstack_timer_source_t s_pac_timer;
static bool s_toned;                                         /* connect tone played for this link */

/* the phone subscribed to the ASE: its LE Audio profile is up (not just an LE link,
 * which a phone also opens for JBL One or a GATT scan) - the "connected" tone now */
/* 4 s later: still on LE and A2DP did not take over (Samsung after a fresh pair drops the
 * LE link and goes A2DP, which plays its own tone) - only then the LE "connected" tone */
static void tone_check(btstack_timer_source_t *ts)
{
    (void)ts;
    static uint32_t last_ms;
    uint32_t now = btstack_run_loop_get_time_ms();
    uint32_t since_a2dp = a2dp_in_ms_since_connect_tone();
    if (s_con == HCI_CON_HANDLE_INVALID || a2dp_in_connected()) return;
    if (since_a2dp != 0 && since_a2dp < 10000) return;      /* just heard the A2DP one */
    if (last_ms == 0 || now - last_ms > 30000) {
        tones_play("bt_cnt");                                /* as a Classic connect; once, not per retry */
        last_ms = now;
    }
}

/* "connected" once the LE Audio handshake is done and stable: the phone subscribed to
 * the codec list (PAC), the stream endpoint (ASE) and the volume, then 1.5 s with no
 * further setup. Not on a link that just paired (Samsung goes A2DP after a new bond). */
static btstack_timer_source_t s_ready_timer;

void le_audio_note_pairing(hci_con_handle_t con) { s_paired_con = con; }

static void ready_check(btstack_timer_source_t *ts)
{
    (void)ts;
    if (s_con == HCI_CON_HANDLE_INVALID || s_toned) return;
    bool subscribed = (s_ccc[LA_PAC] & 1) && (s_ccc[LA_ASE] & 1) && (s_ccc[LA_VOL_STATE] & 1);
    uint32_t quiet = btstack_run_loop_get_time_ms() - s_last_gatt_ms;
    if (!subscribed || quiet < 1500) {
        btstack_run_loop_set_timer(&s_ready_timer, 300);
        btstack_run_loop_add_timer(&s_ready_timer);
        return;
    }
    if (s_paired_con == s_con) {
        ESP_LOGI(TAG, "LE link of a new pairing: no LE Audio tone (the phone goes A2DP)");
        s_toned = true;
        return;
    }
    connected_tone();
}

static void ready_start(void)
{
    btstack_run_loop_remove_timer(&s_ready_timer);
    btstack_run_loop_set_timer_handler(&s_ready_timer, &ready_check);
    btstack_run_loop_set_timer(&s_ready_timer, 500);
    btstack_run_loop_add_timer(&s_ready_timer);
}

static void connected_tone(void)
{
    if (s_toned) return;
    s_toned = true;
    ESP_LOGW(TAG, "LE Audio phone connected (handshake done)");
    a2dp_in_pairing_done("LE Audio phone connected");
    tone_check(NULL);
    mcp_start_ms(500);                           /* switched mid-song: Play over LE media control */
}

/* ---- Play over LE (Media Control client) ---------------------------------------------
 * A phone moving A2DP -> LE Audio mid-song pauses; there is no AVRCP on LE. The phone's
 * Generic Media Control Service (0x1849) takes Play (0x01) on its Media Control Point
 * (0x2BA4). The phone did not answer a GATT read before when BTstack's client started
 * with its own MTU exchange: that is off for this link (a one-byte write anyway). */

static void mcp_fail(const char *what, uint8_t st)
{
    ESP_LOGW(TAG, "Play over LE: %s (ATT 0x%02x)", what, st);
    s_mcp_state = 0;
}

static void mcp_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;
    switch (hci_event_packet_get_type(packet)) {
    case GATT_EVENT_SERVICE_QUERY_RESULT:
        gatt_event_service_query_result_get_service(packet, &s_mcs_svc);
        s_mcs_found = true;
        break;
    case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT:
        gatt_event_characteristic_query_result_get_characteristic(packet, &s_mcs_cp_chr);
        s_mcs_cp_found = true;
        break;
    case GATT_EVENT_NOTIFICATION: {
        const uint8_t *v = gatt_event_notification_get_value(packet);
        uint16_t n = gatt_event_notification_get_value_length(packet);
        if (n >= 2) ESP_LOGW(TAG, "Play over LE: phone answers opcode 0x%02x result %u (1 = done)", v[0], v[1]);
        break;
    }
    case GATT_EVENT_QUERY_COMPLETE: {
        uint8_t st = gatt_event_query_complete_get_att_status(packet);
        if (s_con == HCI_CON_HANDLE_INVALID) { s_mcp_state = 0; break; }
        switch (s_mcp_state) {
        case 1:
            if (!s_mcs_found || st != ATT_ERROR_SUCCESS) { mcp_fail("no media control service", st); break; }
            ESP_LOGW(TAG, "Play over LE: media control service 0x%04x-0x%04x", s_mcs_svc.start_group_handle,
                     s_mcs_svc.end_group_handle);
            s_mcp_state = 2;
            s_mcs_cp_found = false;
            gatt_client_discover_characteristics_for_service_by_uuid16(&mcp_handler, s_con, &s_mcs_svc,
                                                                       ORG_BLUETOOTH_CHARACTERISTIC_MEDIA_CONTROL_POINT);
            break;
        case 2:
            if (!s_mcs_cp_found || st != ATT_ERROR_SUCCESS) { mcp_fail("no media control point", st); break; }
            ESP_LOGW(TAG, "Play over LE: control point 0x%04x (end 0x%04x): subscribing first",
                     s_mcs_cp_chr.value_handle, s_mcs_cp_chr.end_handle);
            if (!s_mcs_listening) {
                gatt_client_listen_for_characteristic_value_updates(&s_mcs_notif, &mcp_handler, s_con, &s_mcs_cp_chr);
                s_mcs_listening = true;
            }
            /* Android's media control service rejects control point writes from a client
             * that has not enabled the control point's notifications */
            s_mcp_state = 3;
            gatt_client_write_client_characteristic_configuration(&mcp_handler, s_con, &s_mcs_cp_chr,
                GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
            break;
        case 3:
            ESP_LOGW(TAG, "Play over LE: control point notifications on (ATT 0x%02x) - sending %s", st,
                     s_mcp_op == 0x02 ? "Pause" : "Play");
            s_mcp_state = 4;
            gatt_client_write_value_of_characteristic(&mcp_handler, s_con, s_mcs_cp_chr.value_handle, 1, &s_mcp_op);
            break;
        case 4:
            ESP_LOGW(TAG, "Play over LE: written (ATT 0x%02x)", st);
            s_mcp_state = 0;
            break;
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
}

static void mcp_play(btstack_timer_source_t *ts)
{
    (void)ts;
    if (s_con == HCI_CON_HANDLE_INVALID || s_mcp_state != 0 || s_mcp_done) return;
    /* no ASE check: the phone always opens a silent (unspecified) stream first, and
     * Play to a player that already plays changes nothing. Not AVRCP: a phone on LE
     * Audio closes a new AVRCP channel within ms (Play dropped). */
    if (!a2dp_in_was_playing(30000)) return;
    s_mcp_done = true;                                        /* once per connection */
    s_mcp_op = 0x01;
    s_mcs_found = false;
    s_mcp_state = 1;
    gatt_client_mtu_enable_auto_negotiation(0);             /* this link's client context: no MTU exchange */
    uint8_t st = gatt_client_discover_primary_services_by_uuid16(&mcp_handler, s_con,
                                                                 ORG_BLUETOOTH_SERVICE_GENERIC_MEDIA_CONTROL_SERVICE);
    gatt_client_mtu_enable_auto_negotiation(1);             /* JBL speaker links keep theirs */
    ESP_LOGW(TAG, "A2DP was playing before the switch: Play over LE (media control) -> 0x%02x", st);
    if (st != ERROR_CODE_SUCCESS) s_mcp_state = 0;
}

/* multipoint: another phone started playing over A2DP - ask this LE Audio phone to pause
 * (Media Control Point Pause), as a JBL speaker pauses the phone that played before */
void le_audio_media_pause(void)
{
    if (s_con == HCI_CON_HANDLE_INVALID || s_mcp_state != 0) return;
    s_mcp_op = 0x02;
    if (s_mcs_cp_found && s_mcs_listening) {                  /* control point known: write now */
        s_mcp_state = 4;
        uint8_t st = gatt_client_write_value_of_characteristic(&mcp_handler, s_con, s_mcs_cp_chr.value_handle, 1, &s_mcp_op);
        ESP_LOGW(TAG, "multipoint: Pause over LE to the LE Audio phone -> 0x%02x", st);
        if (st != ERROR_CODE_SUCCESS) s_mcp_state = 0;
        return;
    }
    s_mcs_found = false;
    s_mcp_state = 1;
    gatt_client_mtu_enable_auto_negotiation(0);
    uint8_t st = gatt_client_discover_primary_services_by_uuid16(&mcp_handler, s_con,
                                                                 ORG_BLUETOOTH_SERVICE_GENERIC_MEDIA_CONTROL_SERVICE);
    gatt_client_mtu_enable_auto_negotiation(1);
    ESP_LOGW(TAG, "multipoint: Pause over LE to the LE Audio phone (finding its media control) -> 0x%02x", st);
    if (st != ERROR_CODE_SUCCESS) s_mcp_state = 0;
}

/* the LE Audio phone's identity address (its bond), false when not known */
bool le_audio_phone_identity(bd_addr_t out)
{
    if (s_con == HCI_CON_HANDLE_INVALID) return false;
    int idx = sm_le_device_index(s_con);
    if (idx < 0) return false;
    int type = BD_ADDR_TYPE_UNKNOWN;
    sm_key_t irk;
    le_device_db_info(idx, &type, out, irk);
    return type != BD_ADDR_TYPE_UNKNOWN;
}

static void mcp_start_ms(uint32_t ms)
{
    if (s_mcp_done) return;                                  /* sent on this connection */
    btstack_run_loop_remove_timer(&s_mcp_timer);
    btstack_run_loop_set_timer_handler(&s_mcp_timer, &mcp_play);
    btstack_run_loop_set_timer(&s_mcp_timer, ms);
    btstack_run_loop_add_timer(&s_mcp_timer);
}

/* on the phone's CCCDs: a fallback, the handshake-done path re-arms it sooner */
static void mcp_start(void)
{
    s_mcp_state = 0;
    mcp_start_ms(6000);
}
static uint8_t s_pac_tries;

static void pac_refresh(btstack_timer_source_t *ts)
{
    (void)ts;
    if (s_con == HCI_CON_HANDLE_INVALID) return;
    uint16_t mtu = att_server_get_mtu(s_con);
    if ((s_ccc[LA_PAC] & 1) == 0 || mtu < sizeof(s_sink_pac) + 3) {
        if (++s_pac_tries < 10) {
            btstack_run_loop_set_timer(&s_pac_timer, 500);
            btstack_run_loop_add_timer(&s_pac_timer);
        } else {
            ESP_LOGW(TAG, "PAC refresh skipped (notify %s, MTU %u)", (s_ccc[LA_PAC] & 1) ? "on" : "off", mtu);
        }
        return;
    }
    notify(LA_PAC, s_sink_pac, sizeof(s_sink_pac));
    uint8_t v[4];
    little_endian_store_16(v, 0, s_enabled ? LA_SINK_CONTEXTS : 0);
    little_endian_store_16(v, 2, 0);
    notify(LA_AVAIL, v, 4);
    ESP_LOGI(TAG, "sink PAC + available contexts re-sent to the phone (%u B, MTU %u)", (unsigned)sizeof(s_sink_pac), mtu);
}

static void pac_start(void)
{
    s_pac_tries = 0;
    btstack_run_loop_remove_timer(&s_pac_timer);
    btstack_run_loop_set_timer_handler(&s_pac_timer, &pac_refresh);
    btstack_run_loop_set_timer(&s_pac_timer, 2500);          /* after the phone's discovery + MTU exchange */
    btstack_run_loop_add_timer(&s_pac_timer);
}

static void name_start(void)
{
    s_name_tries = 0;
    btstack_run_loop_remove_timer(&s_name_timer);
    btstack_run_loop_set_timer_handler(&s_name_timer, &name_read);
    btstack_run_loop_set_timer(&s_name_timer, 1500);         /* after encryption (bond lookup) */
    btstack_run_loop_add_timer(&s_name_timer);
}

static void hci_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;
    switch (hci_event_packet_get_type(packet)) {
    case HCI_EVENT_LE_META:
        switch (hci_event_le_meta_get_subevent_code(packet)) {
        case HCI_SUBEVENT_LE_ADVERTISING_SET_TERMINATED:
            if (hci_subevent_le_advertising_set_terminated_get_advertising_handle(packet) == s_adv_handle &&
                hci_subevent_le_advertising_set_terminated_get_status(packet) == ERROR_CODE_SUCCESS) {
                if (!s_enabled) {                        /* the reachable-only advert */
                    s_reach_con = hci_subevent_le_advertising_set_terminated_get_connection_handle(packet);
                    s_adv_on = false;
                    ESP_LOGW(TAG, "phone reached us over LE (con 0x%04x) with LE Audio off: its old keys fail, "
                                  "it pairs again", s_reach_con);
                    break;
                }
                {
                    hci_con_handle_t c = hci_subevent_le_advertising_set_terminated_get_connection_handle(packet);
                    hci_connection_t *hc = hci_connection_for_handle(c);
                    if (hc != NULL && lls_is_partner_addr(hc->address)) {
                        ESP_LOGW(TAG, "LE connection 0x%04x is the stereo partner, not a phone", c);
                        s_adv_on = false;
                        adv_update();                    /* keep advertising for phones */
                        break;
                    }
                }
                s_con = hci_subevent_le_advertising_set_terminated_get_connection_handle(packet);
                ESP_LOGW(TAG, "phone connected for LE Audio (con 0x%04x)", s_con);
                name_start();                            /* srcName1Crc for JBL One */
                a2dp_in_refresh_discoverable();          /* a phone is linked: out of pairing */
                pac_start();                             /* heal a phone's stale (empty) PAC copy */
                mcp_start();                             /* switched mid-song: Play over LE */
                jbl_set_streaming(true);                 /* beacon 20 -> 100 ms: air for the CIS setup */
                s_toned = false;                         /* tone once the LE Audio handshake is done */
                s_last_gatt_ms = btstack_run_loop_get_time_ms();
                ready_start();
            }
            break;
        case HCI_SUBEVENT_LE_CIS_ESTABLISHED:
            if (hci_subevent_le_cis_established_get_status(packet) == ERROR_CODE_SUCCESS) {
                ESP_LOGW(TAG, "CIS layout: NSE %u, BN %u, FT %u, max PDU %u B, ISO interval %u us, "
                         "transport latency %lu us, PHY %u",
                         hci_subevent_le_cis_established_get_nse(packet),
                         hci_subevent_le_cis_established_get_bn_c_to_p(packet),
                         hci_subevent_le_cis_established_get_ft_c_to_p(packet),
                         hci_subevent_le_cis_established_get_max_pdu_c_to_p(packet),
                         hci_subevent_le_cis_established_get_iso_interval(packet) * 1250u,
                         (unsigned long)hci_subevent_le_cis_established_get_transport_latency_c_to_p(packet),
                         hci_subevent_le_cis_established_get_phy_c_to_p(packet));
            }
            break;
        case HCI_SUBEVENT_LE_CIS_REQUEST: {
            hci_con_handle_t cis = hci_subevent_le_cis_request_get_cis_connection_handle(packet);
            hci_con_handle_t acl = hci_subevent_le_cis_request_get_acl_connection_handle(packet);
            bool ok = s_enabled && s_ase.state >= ASE_QOS && s_ase.state <= ASE_STREAMING;
            ESP_LOGW(TAG, "CIS request 0x%04x on con 0x%04x (CIG %u CIS %u) -> %s", cis, acl,
                     hci_subevent_le_cis_request_get_cig_id(packet), hci_subevent_le_cis_request_get_cis_id(packet),
                     ok ? "accept" : "reject");
            if (ok) {
                s_con = acl;
                gap_cis_accept(cis);
            } else {
                gap_cis_reject(cis);
            }
            break;
        }
        default:
            break;
        }
        break;
    case HCI_EVENT_META_GAP:
        if (hci_event_gap_meta_get_subevent_code(packet) == GAP_SUBEVENT_CIS_CREATED) {
            uint8_t st = gap_subevent_cis_created_get_status(packet);
            hci_con_handle_t cis = gap_subevent_cis_created_get_cis_con_handle(packet);
            ESP_LOGW(TAG, "CIS 0x%04x %s (status 0x%02x, ISO interval %u us)", cis, st ? "FAILED" : "up", st,
                     gap_subevent_cis_created_get_iso_interval_1250us(packet) * 1250u);
            if (st == ERROR_CODE_SUCCESS) {
                s_cis = cis;
                s_iso_us = gap_subevent_cis_created_get_iso_interval_1250us(packet) * 1250u;
                if (s_ase.state == ASE_ENABLING) {
                    ase_set(ASE_STREAMING, "CIS up");
                    audio_start();
                }
            }
        }
        break;
    case HCI_EVENT_DISCONNECTION_COMPLETE: {
        hci_con_handle_t h = hci_event_disconnection_complete_get_connection_handle(packet);
        if (h == s_cis) {
            s_cis = HCI_CON_HANDLE_INVALID;
            audio_stop("CIS closed");
            if (s_ase.state == ASE_RELEASING) {
                ase_set(ASE_IDLE, "released");
            } else if (s_ase.state == ASE_ENABLING || s_ase.state == ASE_STREAMING || s_ase.state == ASE_DISABLING) {
                ase_set(ASE_QOS, "CIS closed");
            }
        } else if (h == s_con) {
            ESP_LOGW(TAG, "LE Audio phone left (reason 0x%02x)", hci_event_disconnection_complete_get_reason(packet));
            link_gone("phone left");
            a2dp_in_refresh_discoverable();
        } else if (h == s_reach_con) {
            ESP_LOGW(TAG, "phone with the old bond left (reason 0x%02x)", hci_event_disconnection_complete_get_reason(packet));
            s_reach_con = HCI_CON_HANDLE_INVALID;
            adv_update();
        }
        break;
    }
    default:
        break;
    }
}

/* ---- public ----------------------------------------------------------------------- */
void le_audio_setup(void)
{
    s_hci_reg.callback = &hci_handler;
    hci_add_event_handler(&s_hci_reg);
    s_vol_last = 0xFF;
    btstack_run_loop_set_timer_handler(&s_vol_timer, &vol_tick);
    btstack_run_loop_set_timer(&s_vol_timer, 300);
    btstack_run_loop_add_timer(&s_vol_timer);
}

void le_audio_on_hci_working(void)
{
    adv_update();
}

void le_audio_set_enabled(bool on)
{
    if (on == s_enabled) return;
    s_enabled = on;
    ESP_LOGW(TAG, "LE Audio %s", on ? "ON" : "OFF");
    jbl_gatt_set_le_audio(on);                       /* off: no LE Audio services for a phone to find */
    if (!on) {
        audio_stop("LE Audio off");
        if (s_con != HCI_CON_HANDLE_INVALID && gap_get_connection_type(s_con) != GAP_CONNECTION_ACL &&
            s_ase.state != ASE_IDLE) {
            gap_disconnect(s_con);                   /* the phone falls back to A2DP */
        }
    }
    if (hci_get_state() == HCI_STATE_WORKING) adv_update();
    uint8_t v[4];
    little_endian_store_16(v, 0, on ? LA_SINK_CONTEXTS : 0);
    little_endian_store_16(v, 2, 0);
    notify(LA_AVAIL, v, 4);
}

bool le_audio_enabled(void) { return s_enabled; }

void le_audio_set_reachable(bool on)
{
    if (on == s_reach) return;
    s_reach = on;
    if (hci_get_state() == HCI_STATE_WORKING) adv_update();
}

bool le_audio_phone_linked(void) { return s_con != HCI_CON_HANDLE_INVALID; }

bool le_audio_starting(void) { return s_ase.state >= ASE_CODEC && s_ase.state <= ASE_ENABLING; }   /* Config Codec on */

bool le_audio_setting_up(void) { return s_con != HCI_CON_HANDLE_INVALID && (!s_toned || le_audio_starting()); }

bool le_audio_owns_link(hci_con_handle_t con)
{
    return con != HCI_CON_HANDLE_INVALID && con == s_con;
}

void le_audio_log_stats(void)
{
    if (!s_enabled) return;
    ESP_LOGI(TAG, "LE Audio: %s, ASE %s, CIS 0x%04x | SDUs rx %lu lost %lu bad %lu PLC %lu | CP ops %lu",
             s_con != HCI_CON_HANDLE_INVALID ? "phone linked" : (s_adv_on ? "advertising" : "idle"),
             s_state_names[s_ase.state], s_cis, (unsigned long)st_rx, (unsigned long)st_lost, (unsigned long)st_bad,
             (unsigned long)st_plc, (unsigned long)st_cp_ops);
}

bool le_audio_cis_counts(uint32_t *rx, uint32_t *lost)
{
    *rx = st_rx;
    *lost = st_lost;
    return s_ase.state == ASE_STREAMING && s_cis != HCI_CON_HANDLE_INVALID;
}
