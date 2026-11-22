# JBL Xtreme 4 OTA Firmware — Auracast / LE Audio Reverse-Engineering Report

**Subject file:** `xtreme4_ota_all0650 (1).bin`
**Size:** 1,590,190 bytes (0x1843AE)
**Working copy:** `...\scratchpad\fw\fw.bin` (original left untouched)
**Scope:** How the speaker broadcasts and receives Auracast (Bluetooth LE Audio broadcast), from firmware evidence plus the published LE Audio/BAP spec.
**Authorized research** — user owns the device and the firmware file.

> **Evidence convention:** Claims tagged **[FW @offset]** are directly observed in the firmware at the stated byte offset. Claims tagged **[SPEC]** are standard Bluetooth LE Audio / Auracast behavior used to explain the firmware, not read from it. Claims tagged **[INFERENCE]** are my reasoning from indirect evidence.

---

## 1. Executive summary

- The image is a **Harman ("HM") custom OTA container**, not a standard Linux/UBI/SquashFS image. It is **two concatenated Harman-packaged blobs** wrapped in `HMHEADER` / `PARTHMDT` / `HMDT` / `FOOTERHM` records. **[FW]**
- The payload is **plaintext ARM Thumb-2 (Cortex-M class) firmware** — an RTOS-based Bluetooth-audio SoC image. Entropy is ~6.5 bits/byte across the bulk of the file, i.e. **ordinary compiled code, not encrypted and not whole-image compressed.** Only a handful of small 4 KB regions are high-entropy (packed/compressed data blobs). **[FW]**
- **Auracast support is present and is implemented with generic LE Audio terminology, not the marketing word "Auracast."** The string "Auracast" does **not** appear anywhere in the image. **[FW]** Instead there is an explicit **broadcast/unicast state machine**: `BSRC_START` / `BSRC_STOP` / `BSRC_AUX_START` (**B**roadcast **S**ou**rc**e = transmit), `BSNK_START` / `BSNK_STOP` (**B**roadcast **S**i**nk** = receive), `UC_START` / `UC_STOP` / `US_START` / `US_STOP` (Unicast), and `DM_UPDATE_UNICAST_ADV`. **[FW @0x103E06–0x103E6A, 0x103DDA]**
- **LC3 codec** (the mandatory LE Audio codec) is present with init/encode/decode paths. **[FW @0x113CD2, 0x113D2E]**
- A dedicated **`App Task Broadcast`** RTOS task, a **`Broadcast Queue`**, and controller-level `broadcast_master_exit` / `broadcast_slave_exit` / `broadcast_isr` / `ABIS_MasterExit` routines handle the broadcast (BIG/BIS) link. **[FW @0x32D6E, 0x32FBA, 0x335A5, 0x33679, 0x34945, 0x3E9F1]**
- So: the firmware can act as **both an Auracast transmitter (broadcast source) and an Auracast receiver (broadcast sink)**, plus LE Audio unicast. The actual bit-level advertising templates and any default broadcast code are built inside compiled code (not stored as plaintext), so they are **not directly readable**; the mechanism is reconstructed from the state-machine strings + spec.

---

## 2. Container format & architecture

### 2.1 Magic / header

Hex dump of the first bytes **[FW @0x0]**:

```
00000000  48 4d 48 45 41 44 45 52 46 00 00 00 ...   HMHEADER F...
00000052  50 41 52 54 48 4d 44 54 ...               PARTHMDT
00000056  48 4d 44 54 00 00 01 20 71 00 04 00 ...   HMDT ....
```

Record tags found across the file **[FW]**:

| Tag | Meaning (interpreted) | Offsets |
|-----|------------------------|---------|
| `HMHEADER` | Harman package header | 0x0, 0x13AC2 |
| `PARTHMDT` | Partition descriptor | 0x52, 0x13ACE, 0x2EF0A, 0x111F8E, 0x15C396, 0x16439E |
| `HMDT` | Harman data section | 0x56, 0x13AD2, 0x2EF0E, 0x111F92, 0x15C39A, 0x1643A2 |
| `FOOTERHM` | Harman package footer | 0x13ADA, 0x1843A6 (8 bytes before EOF) |

**Interpretation [INFERENCE]:** two Harman images are concatenated:
- **Image A — 0x0 … ~0x13AC2 (~80 KB):** a small front image holding the **factory/production AT-command interface, device identity/config, Google Fast Pair data, and an OTA stub**. Contains the `AT+HMDev…` command table, `JBL Xtreme 4` device-name strings, `GFPS`/`AT+GFPSWriteModelId`, build stamp `2024-07-24` and `v0.0.00`.
- **Image B — 0x13AC2 … 0x1843AE (~1.44 MB):** the **main application firmware**. Its container descriptor (`HMHEADER`+`PARTHMDT`+`HMDT`+`FOOTERHM`) sits at 0x13AC2–0x13AE1, immediately followed by ARM Thumb-2 code (`2D E9 FC 47` = `PUSH {r2-r9,lr}`). The inner `PARTHMDT`/`HMDT` markers at 0x2EF0A, 0x111F8E, 0x15C396, 0x16439E delimit sub-sections (code, DSP image, config/NVM, resources). A closing `FOOTERHM` ends the package at 0x1843A6. **[FW]**

### 2.2 CPU / architecture

- **ARM Thumb-2, Cortex-M-class.** From offset ~0xC8 and throughout: `70 47` (`BX LR`), `1F B5`/`10 B5` (`PUSH {…,LR}`), `2D E9 …` (`PUSH.W`), and Thumb-2 `BL`/`B.W` encodings. **[FW]** This is a microcontroller-class Bluetooth-audio SoC, not an application processor running Linux (no ELF, no SquashFS/UBI, no kernel strings).

### 2.3 SoC identity

I **cannot name the exact SoC from an explicit part-number string** (none present as plaintext). Evidence that constrains it **[FW]**:

- **RTOS task architecture:** `App Task Codec`, `App Task DSP`, `App Task BTCntlr`, `App Task BTHost`, `App Task BTApp`, `App Task Pecker`, `App Task Broadcast`, plus `VOS Task SysTimer`, `MccBle Queue`, `Mmi Queue`, `Pecker Queue`, `HLP Queue`, `IPC Service`, `A2DP SRC/SNK Service`. The split `BTHost`/`BTCntlr`, the `VOS` ("vendor OS") prefix, and the distinctive `Pecker` / `MccBle` task names are the signature of a **proprietary dual-core (MCU + audio-DSP) BT-audio SoC RTOS**.
- **DSP audio IP is ZG Micro:** `Zgmicro` strings and log lines `CLO init ZgmicroCLO_EN`, `AEC init ZgmicroAEC_EN`, and a `BLE_ZGMICRO` identifier. **[FW @0x2E70B, 0x113BA2, 0x113BE7, 0x15D118]** DSP tasks move data `Sram2DspDm` / `DspDm2Sram`.
- The vendor names visible at 0x1030xx–0x1037xx (`MediaTek`, `Qualcomm`, `Realtek`, `Marvell`, `Harman International`, `3DSP`, `BandSpeed`, `Socket Mobile`, …) are the **Bluetooth SIG Company-Identifier lookup table**, i.e. a standard constant table — **not** the SoC vendor. Do not read "Qualcomm/MediaTek" here as the chip.

**[INFERENCE]** The SoC is an ARM Cortex-M–class dual-core BT-audio chip whose **audio DSP IP is from ZG Micro** (or is a ZG Micro SoC). I am not asserting a specific part number because the firmware does not state one.

Build/version strings **[FW]**: `2024-07-24 17:23:33` (@0x22E1E), `2024-11-13` (@0xF82C9), `2024-09-09`, `V1.25` (@0xFEA59), `v0.0.00` (@0x2311F) — consistent with a mid/late-2024 JBL Xtreme 4 release.

---

## 3. Entropy / section map

4 KB-block Shannon entropy (bits/byte). Bulk of the image is MID (~6.5) = plain ARM code + strings; isolated HIGH blocks are packed/compressed data; LOW blocks are zero/sparse padding or tables. **[FW]**

| Band | Range | Size | avg entropy | Likely content |
|------|-------|------|-------------|----------------|
| MID | 0x00000–0x26000 | 152 KB | 6.61 | Image A: AT cmds, config, strings, code |
| HIGH | 0x26000–0x2C000 | 24 KB | 7.71 | packed/compressed blob |
| MID | 0x2F000–0xF7000 | 800 KB | 6.71 | main ARM code (Image B) |
| HIGH | 0x100000, 0x111000, 0x11A000, 0x136000 | 4–12 KB each | 7.1–7.9 | packed data / DSP coefficients |
| LOW | 0x112000–0x11A000 | 32 KB | 0.93 | sparse table |
| LOW | 0x11C000–0x135000 | 100 KB | 0.03 | zero padding |
| MID | 0x142000–0x15D000 | 108 KB | 6.46 | code/resources |
| MID/LOW | 0x15D000–0x185000 | ~160 KB | mixed | config/NVM, resources, footer |

No region looks AES/stream-encrypted (that would show sustained ~7.99 across large spans). The scattered small 7.7–7.9 blocks are consistent with per-asset compression (e.g. LC3/DSP coefficient tables, tone/LED resources), not whole-image encryption. **[FW]**

Signature scan: the gzip/xz/UBI/SquashFS/ELF magics were **absent**; `lzma`/`jffs2` "hits" were sparse 3–4-byte coincidences inside code (false positives), **not** real filesystems. **[FW]**

---

## 4. Auracast / LE-Audio-relevant strings (with offsets)

All **[FW]**, from ASCII string extraction (min length 5). Full dumps saved at
`...\scratchpad\scripts\strings_ascii.txt` (5,603 lines) and `strings_utf16.txt` (140 lines).

### 4.1 Broadcast / unicast state machine (the core evidence)

Contiguous enum/string table at **0x103DCA–0x103E6C** (little gaps are NUL padding):

| Offset | String | Meaning |
|--------|--------|---------|
| 0x103DDA | `DM_UPDATE_UNICAST_ADV` | update LE unicast advertising data |
| 0x103E06 | `BSRC_AUX_START,` | **Broadcast Source** start, from **AUX-in** source |
| 0x103E1E | `BSRC_STOP` | Broadcast Source stop |
| 0x103E2A | `UC_STOP` | Unicast stop |
| 0x103E32 | `BSNK_STOP` | **Broadcast Sink** stop |
| 0x103E3E | `US_STOP` | Unicast-Source/Stream stop |
| 0x103E46 | `BSRC_START` | **Broadcast Source** start |
| 0x103E52 | `UC_START` | Unicast start |
| 0x103E5E | `BSNK_START` | **Broadcast Sink** start |
| 0x103E6A | `US_START` | Unicast-Source/Stream start |

Adjacent classic/TWS broadcast-mode controls in the same MMI table **[FW]**:
`A2DPBROADCAST_SWITCH` @0x103CFA, `A2DPBROADCAST_ENTER` @0x103D9A, `A2DPBROADCAST_QUIT` @0x103DB6, `ENTER_PAIRING` @0x103CDA, `SECOND_CONNECTED` @0x103E76.

### 4.2 Broadcast task & controller layer

| Offset | String | Meaning |
|--------|--------|---------|
| 0x32D6E | `App Task Broadcast` | dedicated RTOS broadcast task |
| 0x32FBA | `Broadcast Queue` | its message queue |
| 0x335A5 | `broadcast_master_exit` | controller: leave broadcast as transmitter |
| 0x33679 | `broadcast_slave_exit` | controller: leave broadcast as receiver |
| 0x34945 | `broadcast_isr` | broadcast ISR (isochronous timing) |
| 0x34956 | `master_exit Lock` | sync lock for broadcast teardown |
| 0x3E9F1 | `ABIS_MasterExit` | **A/BIS** (Broadcast Isochronous Stream) master exit |

### 4.3 LC3 codec (LE Audio mandatory codec)

| Offset | String |
|--------|--------|
| 0x113CD2 | `LC3 Init Encode: nchannels: %d, bitrate: %d, mode: %d` |
| 0x113D2E | `LC3 Init Decode: nchannels: %d, bitrate: %d, mode: %d` |

LC3 lives in the DSP firmware (`VSYSTEM_DSP_SetStreamConfig`, `VDSP_PlayerAlgProcess` @0x113A46/0x113B3A), alongside ZG Micro EQ/AEC/CLO/DRC algorithms.

### 4.4 Device identity / advertising name

| Offset | String |
|--------|--------|
| 0x10375, 0x23134, 0x2ED24, 0x15C81D | `JBL Xtreme 4` / `JBL Xtreme4` (BT/BLE local name) |
| 0x2ED94 | `JBL Flip 5` (companion/stereo-pair reference in config) |
| 0xF60A / 0xFAF6 | `AT+HMDevNAME` / `AT+HMDevNAME=` (get/set device name) |

### 4.5 What was searched for and **not** found (honest negatives) **[FW]**

- Literal `auracast` / `Auracast` / `AURACAST`: **absent** (confirmed by byte search).
- No plaintext **broadcast code / passphrase / broadcast name template / Broadcast_ID** constant was recoverable. These are generated/stored in compiled code or runtime NVM, not as readable strings.
- The 16-bit LE-Audio GATT service UUID scan (0x1850 ASCS/BASS/PACS/BAAS/TMAS/PBAS range) returned matches, but with 70+ hits each across a code image those are overwhelmingly **coincidental 2-byte matches, not reliable UUID evidence** — I am not resting any claim on them.

---

## 5. What decoded vs. what stayed opaque

**Decoded / plaintext (high confidence):**
- Container format and section boundaries (Harman HM records). **[FW]**
- CPU/arch (ARM Thumb-2 Cortex-M class). **[FW]**
- RTOS task/service map and the broadcast/unicast **state-machine enum**. **[FW]**
- Presence of LC3, DSP pipeline, ZG Micro audio IP. **[FW]**
- Device name, AT/Fast-Pair production protocol, build dates. **[FW]**

**Opaque / not recoverable without disassembly or a live device:**
- The exact **Extended/Periodic advertising payload bytes** (Broadcast Audio Announcement, Basic Audio Announcement BASE structure, BIGInfo contents). Built in compiled code.
- Any **default broadcast code** (encryption passphrase) or **Broadcast Name** template.
- The precise **LC3 configuration actually broadcast** (sampling rate / frame duration / octets-per-frame). `LC3 Init …: bitrate/mode %d` confirms it is configurable but the chosen preset is a runtime value, not a string.
- The small HIGH-entropy blocks (DSP coefficient tables / packed resources) were not unpacked (no identifiable standard compression header; would need the SoC's own loader logic).

A full control-flow reconstruction would require loading Image B into a Thumb-2 disassembler (Ghidra/radare2) at the correct base address and tracing from the `App Task Broadcast` and `DM_UPDATE_UNICAST_ADV` references — out of scope for a strings/entropy pass, and not installed in this environment. The plaintext state machine already identifies every Auracast role the firmware supports.

---

## 6. How the Xtreme 4 does Auracast — mechanism

Combining the firmware evidence (§4) with the Bluetooth LE Audio / Public Broadcast Profile spec **[SPEC]**:

### 6.1 As an Auracast **transmitter** (broadcast source) — `BSRC_*`

Triggered by the `BSRC_START` / `BSRC_AUX_START` state (and the user/app path through the MMI table). **[FW]** Standard sequence **[SPEC]**:

1. The speaker's audio input (BT/USB/streamed, or the analog **AUX** input — hence the dedicated `BSRC_AUX_START`) is encoded to **LC3** by the DSP (`LC3 Init Encode`). **[FW]**
2. The controller sets up a **BIG (Broadcast Isochronous Group)** containing one or more **BIS** (Broadcast Isochronous Streams). The `App Task Broadcast` + `broadcast_master_exit` + `broadcast_isr` + `ABIS_MasterExit` routines are the master/transmit side of this BIG. **[FW]**
3. It advertises on a periodic-advertising train: an **Extended Advertisement** carrying the **Broadcast Audio Announcement** (plus, for Auracast/Public Broadcast, a **Public Broadcast Announcement**), a **Periodic Advertisement** carrying the **BASE** (codec = LC3, stream layout, optional Broadcast Name), and **BIGInfo** so receivers can sync to the isochronous stream. **[SPEC]**
4. If a broadcast code is set, the BIS audio is AES-CCM encrypted and receivers need the code; otherwise the stream is open. The firmware supports this but the default/stored code is not a plaintext constant. **[FW: not found / SPEC]**

The older **`A2DPBROADCAST_*`** states are the classic BR/EDR "broadcast to another JBL speaker" (PartyBoost-style) path and are **separate** from the LE-Audio `BSRC/BSNK` Auracast path — both exist in this firmware. **[FW]**

### 6.2 As an Auracast **receiver** (broadcast sink) — `BSNK_*`

Triggered by `BSNK_START`; torn down by `BSNK_STOP` / `broadcast_slave_exit`. **[FW]** Standard sequence **[SPEC]**:

1. Scan for Broadcast Audio Announcements, select a source, and **sync to its periodic advertising** (PA sync), then read **BIGInfo**. **[SPEC]**
2. Join the **BIG** as a receiver (`broadcast_slave_exit` is the leave-as-receiver counterpart of `broadcast_master_exit`), extract the chosen **BIS**, and if encrypted apply the broadcast code. **[FW/SPEC]**
3. **LC3-decode** the stream (`LC3 Init Decode`) and render it through the DSP (EQ/DRC/gain). **[FW]**

### 6.3 How a phone / the JBL app is involved

- **Control/transport channel:** the firmware exposes a Harman command protocol over **`AT+HMDev…`** (name, battery, LEDs, pairing: `AT+HMDevSTARTPAIR`, `AT+HMDevNAME`, `AT+HMDevLINKED`, …) and supports **Google Fast Pair** (`AT+GFPSWriteModelId`, `GFPS3`, model-id data). **[FW @0xF3B6–0xFB26, 0xF672, 0xFC1ED]** The app uses this channel plus LE GATT to drive the device.
- **Unicast advertising** for app/phone LE-Audio connections is refreshed via `DM_UPDATE_UNICAST_ADV`, and unicast streams start/stop via `UC_START/UC_STOP` / `US_START/US_STOP`. **[FW]**
- **Assistant model [SPEC]:** in the Auracast/BAP architecture a phone acts as a **Broadcast Assistant** using the **Broadcast Audio Scan Service (BASS)** on the sink to hand it the source address, Broadcast_ID, PA interval and broadcast code (PAST — Periodic Advertising Sync Transfer) so the speaker can join a broadcast without scanning itself. The firmware's `BSNK_*` sink role is the device side of this; the exact BASS GATT table is in compiled code and was not extracted as plaintext. **[FW: role present / SPEC: protocol]**

---

## 7. Bottom line

The Xtreme 4 firmware is a **plaintext, non-encrypted ARM Cortex-M BT-audio image in a Harman "HM" OTA container**, and it **implements Auracast using standard LE Audio primitives** — a broadcast **source** role (`BSRC_*`, incl. an AUX-in broadcast mode), a broadcast **sink** role (`BSNK_*`), LE-Audio **unicast** (`UC_*`/`US_*`, `DM_UPDATE_UNICAST_ADV`), an **LC3** codec, and a dedicated broadcast RTOS task + BIG/BIS controller layer. JBL's "Auracast" branding is **not** a firmware string; the implementation is pure BAP/LE-Audio under the hood. The exact advertising byte templates, LC3 broadcast preset, and any default broadcast code are embedded in compiled code and would require Thumb-2 disassembly (or a live-device BLE sniff) to recover — this report distinguishes those firmware-confirmed facts from standard-spec behavior throughout.

---

### Artifacts produced (scratchpad)
- Working firmware copy: `...\scratchpad\fw\fw.bin`
- Analysis scripts: `...\scratchpad\scripts\analyze.py`, `strings.py`, `deep.py`
- Full strings: `...\scratchpad\scripts\strings_ascii.txt`, `strings_utf16.txt`
