# s31-btstack-speaker

Main firmware of this repository: an ESP32-S31 that acts as a JBL Xtreme 5 (A2DP sink, Auracast
JBL party host and listener, JBL One app server, local I2S output). It runs
[BTstack](https://github.com/bluekitchen/btstack) as the Bluetooth host on top of ESP-IDF's
controller (controller-only build, `CONFIG_BT_CONTROLLER_ONLY=y`, HCI over VHCI). It replaces the
Bluedroid-based [`s31-jbl-speaker`](../s31-jbl-speaker) and is on air identical to it.

Build, flash, pins, partitions and prompt tones are described in the
[root README](../README.md#build-and-flash-s31-btstack-speaker).

## Threads and cores

| Core | Tasks |
|---|---|
| 1 | BT controller; BTstack run loop `btstack` (HCI, L2CAP, AVDTP/A2DP, AVRCP, SDP, ATT/GATT, ISO); `buttons`; `stats` (status lines every 2 s) |
| 0 | `sbc_dsp` (SBC decode + resampler), `lc3_enc` (EQ + LC3 encode, paced by the BIG), `party_dec` (LC3 decode in LISTEN), `i2s_out` / `i2s_local` (I2S output) |

Everything in the JBL modules and in `auracast.c` / `party.c` runs on the BTstack thread unless a
function is marked "any thread" in its header.

## Modules (`main/`)

| File | Role |
|---|---|
| `main.c` | Start-up order, core layout, stats task |
| `a2dp_in.c` | A2DP sink + AVRCP: media queue, SBC or AAC-LC decode, 44.1 -> 48 kHz windowed-sinc resampler with PI clock recovery, PSRAM PCM ring with a small constant target fill, delay reporting, buttons' volume / play-pause / pairing, last-phone reconnect |
| `a2dp_dsp.h` | Resampler and ring code (also used by the host tests in `tools/`) |
| `auracast.c` | BAP broadcast source: JBL-master extended + periodic advertising, BASE, BIG via HCI LE Create BIG Test, ISO-clocked LC3 encode pipeline with silence fill on underrun; at run time also a standard broadcast (`auracast_set_standard()`: own Broadcast ID and name, no JBL blocks, optional broadcast code) |
| `party.c` | Party roles HOST / LISTEN: scan for a JBL master, PA sync, BASE + BIGInfo, BIG sync, LC3 decode with PLC, playback delayed by the master's presentation delay; JBL One Auracast Assistant ("Join Nearby Broadcast" with an optional password, "Broadcast to Others" as a standard broadcast), which keeps the JBL party off |
| `audio_out.c` | I2S output (BCK 35, DOUT 37, LRCK 39; APLL, 48 kHz, 32-bit stereo) fed from a delay line by exactly one producer: broadcast PCM, local phone audio, or LISTEN audio |
| `jbl.c` / `jbl.h` | JBL personality glue: setup order, identity (`macCrc` of the S31's own Classic address), LE security |
| `jbl_beacon.c` | JBL `0xFDDF` host beacon on its own advertising set (party info, battery, misc flags incl. the bit the iOS app needs) |
| `jbl_gatt.c`, `jbl.gatt`, `jbl_gatt_db.h` | GATT database (GAP, JBL services, Device Information) served on LE and BR/EDR |
| `jbl_v5.c` | JBL V5 protocol server for JBL One (Android and iOS): Get / Set / Notify, volume, media, Auracast, rename, factory reset, NVS settings, OTA capture into the `jblota` partition |
| `v5_eq.c` | The app's EQ (biquad cascade per channel) with lock-free coefficient hand-over to core 0 |
| `jbl_assist.c` | Fallback assistant: connects to a nearby JBL speaker that is looking for a master and tells it to join |
| `link_tune.c` | BR/EDR packet types (3-DH5), QoS poll interval (7.5 ms without a BIG, 15 ms while broadcasting, back to 7.5 ms while the A2DP cushion is low), scan trimming, LE connection interval |
| `bt_sched_tune.c` | Controller BR/EDR vs ISO scheduling priorities (see [`re/controller_sched.md`](../re/controller_sched.md)) |
| `buttons.c` | GPIO buttons, active low with pull-ups, 30 ms debounce: 36 Auracast, 38 vol+, 40 vol-, 42 pairing, 44 play/pause |
| `tones.c` | Prompt tones played from the memory-mapped `tones` partition: the Auracast on/off tones cut the music until they end, the others (volume limit, ...) duck it |
| `power.c` | Soft power off (no power switch on the board); any button restarts |
| `sniffer.c` | Party/Auracast sniffer, only in the `S31_SNIFFER` build |

`components/helix_aac/` is the AAC-LC decoder (RealNetworks Helix, RPSL; LC only, no SBR) with a small LATM parser.
`components/btstack_s31/` builds the needed parts of BTstack from `../third_party/btstack` and adds
the S31 VHCI transport (`hci_transport_s31_vhci.c`) and an NVS-backed TLV store for bonding keys
(`btstack_tlv_esp32.c`). LC3 comes from `../s31-jbl-speaker/components/liblc3`.

## Compile-time switches

Defined with a default of 0 in the sources; set them as compile definitions for a variant build.

| Switch | File | Effect |
|---|---|---|
| `S31_SNIFFER` | `main.c` | Sniffer only: no A2DP, no broadcast, no JBL GATT; logs a JBL master's adverts, PA, BIGInfo and ISO |
| `AURA_GENERIC` | `auracast.c` | Plain Auracast broadcast instead of the JBL-master one (receivers join as ordinary Auracast sinks) |
| `S31_TEST_SIGNAL` | `a2dp_in.h` | Replace phone audio with a sweep test signal |
| `S31_HCI_DUMP` | `main.c` | Print every HCI packet (bring-up) |

## Configuration

`sdkconfig.defaults` holds the project configuration (dual-mode controller, ISO test parameters,
PSRAM, 16 MB flash with the custom `partitions.csv`, 921600 baud console, core pinning). Some of
its comments still describe the earlier Bluedroid build.
