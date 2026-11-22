# JBL Xtreme 4 OTA Firmware — Auracast & PartyBoost Reverse-Engineering Report

**Subject file:** `C:\Users\ngocv\Downloads\xtreme4_ota_all0650 (1).bin`
**Analyzed copy (isolated):** `...\scratchpad\fw55\fw.bin`
**Size:** 1,590,190 bytes (0x1842AE)
**Firmware version (from footer):** `8.0.8.13`, git `295a227`, built `2024-7-30 17:18:33`
**Tools available:** Python 3.12 only. `strings`/`binwalk`/`radare2`/`objdump` were **not installed**; all extraction (strings, entropy, byte-pattern search, carving) was done with custom `python -I` scripts. No disassembler was available, so ARM Thumb-2 code was identified by prologue/opcode patterns but not disassembled.

> **Honesty note up front:** Neither the string `"Auracast"` nor `"PartyBoost"` (nor `"party"`, `"Connect+"`, `"Airoha"`, `"stereo"`) appears **anywhere** in the image (ASCII or UTF-16). What *is* present in plaintext is a **proprietary BT-Classic broadcast engine** (master/slave, CSB, "A2DP Broadcast") that is the multi-speaker/"party-link" mechanism, plus the **LC3 codec** (the LE-Audio/Auracast codec building block). The genuine LE-Audio broadcast control plane (BIS/BIG/BASS/periodic-advertising) is **not in plaintext** — the most likely home for it, a separate `BLE_ZGMICRO` partition, is **opaque (encrypted/compressed)**. Firmware-derived claims below carry exact offsets; everything else is explicitly labelled **[spec/known behavior]**.

---

## 1. File format & architecture

### 1.1 Container format — Harman "HM" OTA

The image is **not** a raw SoC image; it is wrapped in Harman's proprietary container using ASCII frame markers. `HM` = **Harman**.

| Marker | Offsets found | Meaning |
|---|---|---|
| `HMHEADER` | `0x0`, `0x13ac2` | image header start |
| `PARTHMDT` | `0x52`, `0x13ace`, `0x2ef0a`, `0x111f8e`, `0x15c396`, `0x16439e` | **PART**ition **H**ar**M**an **D**a**T**a (partition table / sub-image headers) |
| `FOOTERHM` | `0x13ada`, `0x1843a6` | image footer end |

Header @0x0 (hex): `48 4D 48 45 41 44 45 52` (`HMHEADER`) → partition table at `0x52`. The footer tail (`0x184372`) carries the build metadata in plaintext:
```
0x184372  "TUD" .. "295a227" .. "8.0.8.13" .. "2024-7-30 17:18:33"
```
(`295a227` = source-control hash; `8.0.8.13` = firmware version; the build timestamp.)

### 1.2 "all0650" = a combined **dual-image** OTA

The filename token `all` and the two header/footer pairs confirm a **combined package of two images**:

| Image | Range | Size | Contents (from strings) |
|---|---|---|---|
| **Image 1** | `0x0` – `0x13ada` | ~80 KB | Factory/DUT **`AT+HM…` command interface** (`0x1b7e`–`0xf712`), device-key table, Google Fast Pair AT hooks. Likely the **bootloader / DUT (factory-test) partition**. |
| **Image 2** | `0x13ac2` – `0x1843a6` | ~1.5 MB | **Main application**: BT stack (A2DP/AVRCP/HFP), the broadcast/party engine, DSP/codec, BT-SIG company table, BLE advertising templates, and a nested BLE controller partition. |

Image 2 begins (`0x13ae6`) with an ARM **Thumb-2** function prologue `2D E9 FC 47` (`PUSH {r2-r6,sl,lr}`), i.e. the main image is executable Cortex-M code.

Image 2 itself contains **nested `PARTHMDT` sub-partitions**:
- `0x111f8e` — a BSP partition; nearby plaintext `Kproj_bsp32.X` (`0x111fe5`), `proj_bsp32.P` (`0x11208a`), and `bsp32_xtreme4.bin` (`0x1120da`). This names the build target: **`bsp32_xtreme4`** ("bsp32" = the 32-bit board-support project for the Xtreme 4).
- `0x15c396` … `0x16439e` — the **`BLE_ZGMICRO`** partition (see §1.4), with its own build date `2024-09-09 19:01:37` (`0x16042e`) and a base64 key/signature blob `3nZzlCRa4roI0OA+2LPTcfkL7ubx3yx03/nQqTTAlTg=` (`0x160778`, 32-byte value).

### 1.3 SoC / SDK identification — **ZG Micro** BT-audio SoC

Strong, consistent evidence that the main SoC is a **ZG Micro** (ZGMicro) Bluetooth-audio SoC — ARM Cortex-M (Thumb-2) core with a vector-DSP ("VDSP") audio engine and a C++ application framework:

- DSP log strings naming **Zgmicro** algorithms:
  - `0x113ba2` `…CLO init ZgmicroCLO_EN %d`
  - `0x113be7` `…AEC init ZgmicroAEC_EN %d, samprate %d`
  - `0x113b50` `…running algs: EQ %d, AEC %d, CLO %d, DigitalGain %d, DRC %d`
  - `0x113b3a` `VDSP_PlayerAlgProcess`
- BLE companion/partition tag `BLE_ZGMICRO` (`0x15d118`), `ZGMICRO` (`0x15d138`, `0xfeb92`).
- C++ frontend source paths (debug symbols): `../frontend/BTAudioHandle.cpp` (`0x1138c6`), `../frontend/main.cpp` (`0x1139bc`), `../frontend/Interface.c` (`0x113a9a`), `../frontend/lbr_interface.c` (`0x113d8a`), `../lib_system/ISR.cpp`, `../lib_system/MessageTransfer.cpp`, `../frontend/data_path.c`.

The BT-SIG company-identifier name table at `0x10320e`–`0x103936` (contains `Harman International Industries, Inc.` at `0x103262`, plus ~120 other vendors) is the **standard Bluetooth company-ID lookup table** every stack ships — it does **not** identify the SoC vendor (do not read "Marvell"/"Qualcomm"/"Broadcom" entries as the chip; they are just table rows).

### 1.4 The LE / BLE partition is opaque

The `0x15c396`–`0x16439e` partition (`BLE_ZGMICRO`, ~33 KB) is almost entirely **high-entropy / non-printable** (encrypted or compressed): it holds only a device-name string (`JBL Xtreme 4` @`0x15c81d`), the `BLE_ZGMICRO`/`ZGMICRO` tags, a later build date, and a base64 key blob; the rest is opaque. **If** a true LE-Audio/Auracast broadcast state machine exists on this device, this opaque BLE partition (or ROM in the BLE controller, not shipped in this OTA) is where it would live — it could **not** be read.

---

## 2. Section / entropy map (4 KB blocks)

Transitions between low (<4.5), mid (4.5–7.3, = ARM code + strings), and HIGH (>7.3, = compressed/encrypted):

| Offset | Entropy | Class | Interpretation |
|---|---|---|---|
| `0x000000` | 6.81 | mid | Image-1 header + AT/DUT code+strings |
| `0x026000` | 7.70 | **HIGH** | packed/encrypted block (~24 KB) |
| `0x02c000` | 6.30 | mid | Image-2 app code/strings (broadcast engine, A2DP, company table) |
| `0x0f7000`–`0x0fe000` | 3.1–5.3 | low/mid | tables / sparse data |
| `0x112000` | 0.75 | low | padding (`proj_bsp32` boundary) |
| `0x11a000` | 7.38 | **HIGH** | packed block |
| `0x11c000` | 0.00 | low | zero padding |
| `0x136000` | 7.90 | **HIGH** | packed block (~12 KB) |
| `0x13f000`–`0x15c000` | 1.5–6.1 | low/mid | data/code + patch table |
| `0x15d000`–`0x164000` | ~5.7 mixed | **opaque** | `BLE_ZGMICRO` encrypted/compressed partition |
| `0x175000`–end | 3.8–5.7 | low/mid | tail data, footer |

Most of the image is **mid-entropy plaintext code/strings** (good for analysis); the HIGH islands and the `BLE_ZGMICRO` partition are the opaque regions.

---

## 3. Auracast findings (LE-Audio broadcast)

### 3.1 What was found in firmware

**Branded term `Auracast`: NOT present** (0 hits, ASCII or UTF-16).

**LC3 codec — present (the LE-Audio / Auracast codec):** The DSP implements LC3 encode/decode, referred to internally as **"LBR" (Low Bit Rate)**:
- `0x113cd2` `[DSP>> ] …LC3 Init Encode: nchannels: %d, bitrate: %d, mode: %d`
- `0x113d2e` `[DSP>> ] …LC3 Init Decode: nchannels: %d, bitrate: %d, mode: %d`
- `0x113d1e` `VLBREncode_Init`, `0x113d7a` `VLBRDecode_Init`, `0x113d8a` `../frontend/lbr_interface.c`
- `0x103b6a` `lbrde` (LBR decode), `0x103b7a` `lbren` (LBR encode) — pipeline node names.

**LE link management — present (but not broadcast):**
- `0x103dda` `DM_UPDATE_UNICAST_ADV` — Device-Manager event to update **LE *unicast* advertising** (the connectable advert the phone/JBL app connects to), not broadcast.
- `0x103be2` `BTM_BLE_DISCONNECTED`, `0x103c12` `BTM_BLE_CONNECTED` — LE connection state events.

**NOT found (searched exhaustively, ASCII + UTF-16):** `BIS`, `BIG`, `BIGInfo`, `BASS`, `BAP`, `PBP`, `broadcast assistant/sink/source`, `broadcast code`, `periodic adv`/`PAST`, `scan delegator`, `PA sync`, `isochronous`, `CSIS`/`CSIP`, service UUIDs `1850/1851/1852`. (The one `BIS` byte-match at `0x3e9f3` is the tail of **`ABIS`**, not the LE-Audio BIS — see §4.)

### 3.2 Interpretation — how Auracast would work here

Evidence-based conclusion: **this OTA image does not expose an LE-Audio Auracast broadcast/receive state machine in plaintext.** The LC3 codec is present and the BLE control code is in the opaque `BLE_ZGMICRO` partition, so an LE-Audio implementation *may* exist there but **could not be confirmed from this image**. It is equally consistent with the evidence that the JBL app's "connect multiple speakers" feature on the Xtreme 4 is served by the **BT-Classic broadcast engine documented in §4**, with LC3 used for the audio stream.

The following is **[spec/known behavior]**, not found in this firmware, provided so the mechanism is documented:

- **Broadcasting (source):** An Auracast transmitter sends **Extended Advertising** PDUs carrying a *Broadcast Audio Announcement* (Service UUID `0x1852`) plus *Public Broadcast Announcement* (`0x1856`), with the human-readable **Broadcast Name** in the AD. It then runs **Periodic Advertising** carrying the **BASE** (codec = LC3, sampling rate, stream metadata) and, on the isochronous physical channel, a **BIG** (Broadcast Isochronous Group) containing one or more **BIS** streams. If encryption is on, a 4–16 byte **Broadcast Code** gates decryption.
- **Receiving (sink / "join a broadcast"):** A sink scans extended + periodic advertising, **syncs to the periodic train (PAST/PA sync)**, reads the BASE, then **syncs to the BIG** and renders the selected BIS. A phone acting as **Broadcast Assistant** uses the **BASS** GATT service (`0x184F`) on the sink to hand it the source address, BIS index, and Broadcast Code (the "scan delegator" / "add source" flow).
- **How the JBL app triggers it [spec/known]:** the app would write the broadcast parameters (name, code) over the speaker's GATT control service and command enter/exit; the firmware GATT/BLE side of that is in the opaque `BLE_ZGMICRO` partition here.

---

## 4. PartyBoost / multi-speaker findings (proprietary link)

### 4.1 What was found in firmware — a BT-Classic broadcast engine with master/slave roles

**Branded term `PartyBoost`/`Connect+`: NOT present.** But the mechanism is plainly present as a dedicated RTOS task + ISR + function set, with **master/slave link roles**:

| Offset | String | Meaning |
|---|---|---|
| `0x32d6e` | `App Task Broadcast` | dedicated RTOS task for the party/broadcast engine |
| `0x32fba` | `Broadcast Queue` | its message queue |
| `0x335a5` | `broadcast_master_exit` | **master** role teardown |
| `0x33679` | `broadcast_slave_exit` | **slave** role teardown |
| `0x34945` | `broadcast_isr` | broadcast interrupt handler |
| `0x34956` | `master_exit Lock` | master-role mutex |
| `0x3e9f1` | `ABIS_MasterExit` | `ABIS_` = the broadcast subsystem API prefix; master-role API |

**Transport = BT-Classic Connectionless Slave Broadcast (CSB)** — the BR/EDR technology JBL's Connect+/PartyBoost lineage uses for one-to-many speaker sync:
- `0xbe8a1` `csb_codec` (CSB audio path), `0xbeb46` `Codec Queue`
- `0x58ebe` `Ecsb_TMR` (CSB timer — CSB uses a reserved LT_ADDR + periodic beacon timing)

**A2DP source/sink both present** (a speaker can be the audio source *and* a sink of another speaker):
- `0x34b7e` `A2DP SRC Service`, `0x34b92` `A2DP SNK Service`
- `0xfbd66` `A2DP Sink`, `0xfc09b` `A2DP Source`

**App-level control events** — the party mode is entered/left via these events in the main BT state-machine enum (`0x103b62`–`0x103df2`):
- `0x103d9a` `A2DPBROADCAST_ENTER` — enter party/broadcast mode
- `0x103db6` `A2DPBROADCAST_QUIT` — leave it
- `0x103cfa` `A2DPBROADCAST_SWITCH` — toggle/switch
- supporting: `0x103ba6` `BTM_ACL_ROLE_CHANGED` (master/slave role negotiation), `0x103cda` `ENTER_PAIRING`, `0x103d12` `CONNECT_BACK`, `0x103d22` `DISCONNECT_ALL`, `0x103bce` `DM_STATES_CHANGED`.

### 4.2 Discovery / manufacturer-data format (how speakers & the app recognize each other)

A block of **BLE GAP advertising templates** sits at `0x2ed00`–`0x2ee70` (data region between `PARTHMDT` markers). Cleanly parseable GAP AD structures:

```
0x2ed80:  02 01 18            ; AD: len2, type 0x01 Flags = 0x18
          09 FF 57 14 ..      ; AD: type 0xFF Manufacturer-Specific Data (vendor payload)
0x2ed92:  0B 09 "JBL Flip 5"  ; AD: len0x0B, type 0x09 Complete Local Name
0x2ed22:  0C 5D "JBL Xtreme4" ; internal 0x5D-tagged model record (len 0x0C)
```
Also nearby: `evd_event` (`0x2ed39`, event-driven advertising) and `polling` (`0x2ed4a`, scan polling).

Key observations:
- The advertisement carries a **Manufacturer-Specific Data AD (type `0xFF`)** with a vendor payload plus a **Complete Local Name** — this is the beacon a phone (JBL app) and other JBL speakers scan for to discover and link.
- Crucially, a **`"JBL Flip 5"` name template appears alongside `"JBL Xtreme 4"`** — i.e. the firmware carries templates/match-entries for *other* JBL models, consistent with a **cross-model recognition table** so the engine can discover and group with compatible JBL speakers.
- Two "JBL Xtreme 4" device-name strings also appear in the main app at `0x10375` and `0x23134`.

### 4.3 Pairing / linking control surface

- **Factory/host AT interface** (Image 1) includes link-control hooks: `0xf6c2` `AT+HMDevSTARTPAIR`, `0xf6b2` `AT+HMDevLINKED` (query linked state), `0xf5fa` `AT+HMDevRSSI`, `0xf60a` `AT+HMDevNAME`.
- **Google Fast Pair** is supported (for phone pairing, separate from speaker-to-speaker): `0xf672` `AT+GFPSWriteModelId`, `0xf656` `AT+GPFPSW`/`AT+GPFPSRead`, `0x3596` `AT+GDPSModelID`, and a `GFPS3` tag at `0xfc1ed` inside what looks like an SDP/GATT record (`…GFPS3……`).

### 4.4 Interpretation — how PartyBoost/party-link works here

**Found-in-firmware mechanism:** The Xtreme 4 runs a dedicated **"App Task Broadcast"** engine that, on the `A2DPBROADCAST_ENTER` event, puts the speaker into a **BT-Classic Connectionless Slave Broadcast** session. One speaker takes the **master** role (`broadcast_master_exit`/`ABIS_MasterExit`/`master_exit Lock`), the others take the **slave** role (`broadcast_slave_exit`); role is settled via `BTM_ACL_ROLE_CHANGED`. The master streams the decoded audio (`csb_codec` path) to all slaves over CSB so every speaker plays in sync. Discovery and grouping use the **BLE manufacturer-specific advertisement** (type `0xFF` + JBL local name) described in §4.2, which both the JBL app and peer speakers scan for. `A2DPBROADCAST_QUIT`/`DISCONNECT_ALL` tear the group down.

**[spec/known behavior]** JBL's user-facing flow — press the Auracast/PartyBoost button on two speakers to link them, or use the JBL app's "Auracast"/party screen to group speakers — maps onto the `A2DPBROADCAST_ENTER`/`SWITCH`/`QUIT` events above. (On the Xtreme 4 generation JBL markets this as "Auracast," having dropped the "PartyBoost" name; this firmware's engine is the BT-Classic CSB implementation regardless of the marketing label.)

---

## 5. What decoded vs. opaque

| Decoded (plaintext, analyzed) | Opaque (could not read) |
|---|---|
| Harman HM container, dual-image layout, partition table | HIGH-entropy blocks at `0x026000`, `0x11a000`, `0x136000` (packed/encrypted) |
| ARM Thumb-2 identification (not disassembled — no objdump/r2) | Actual instruction-level logic (no disassembler installed) |
| BT-Classic broadcast engine: task, ISR, master/slave, CSB, A2DP SRC/SNK, event enum | **`BLE_ZGMICRO` partition `0x15c396`–`0x16439e`** — encrypted/compressed; any LE-Audio/GATT/Auracast control code is here, unreadable |
| LC3/LBR codec init, DSP (ZgMicro EQ/AEC/CLO/DRC) | The 32-byte base64 key/signature at `0x160778` (purpose: likely image signing) |
| BLE advertising templates + JBL manufacturer-data format, cross-model name table | Full byte-layout of the 0xFF manufacturer payload (framing readable, field semantics not documented in plaintext) |
| AT/DUT command set, Google Fast Pair hooks, version/build metadata | — |

No disassembler was available; **install `radare2`/`binwalk`/`arm-none-eabi-objdump`** to go further (disassemble the broadcast engine around `0x33000`–`0x35000` and `0x3e9f0`; attempt to decompress the HIGH-entropy blocks and the `BLE_ZGMICRO` partition, which may need the device-specific key).

---

## 6. Practical notes for the user (Xtreme 4)

**Auracast (LE-Audio broadcast):**
- The speaker has the **LC3 codec** on-board (confirmed, `0x113cd2`), so it has the audio building block for Auracast. The LE/GATT control for broadcasting and for *joining* someone else's broadcast is in the encrypted `BLE_ZGMICRO` partition and could not be verified from this image.
- **To broadcast [spec/known]:** start Auracast from the JBL (Portable) app's Auracast screen or the speaker's Auracast control; nearby Auracast receivers (earbuds, other Auracast speakers, Auracast-capable TVs) then "join." If the app sets a **Broadcast Code**, receivers need it to join.
- **To receive/join [spec/known]:** use the app (acting as Broadcast Assistant) to pick a nearby broadcast and hand it to the speaker. Keep the speaker on the latest firmware — the LE/Auracast code lives in the separately-dated (`2024-09-09`) `BLE_ZGMICRO` partition that OTA updates refresh.

**PartyBoost / multi-speaker link (found in firmware):**
- Grouping is a **BT-Classic Connectionless-Slave-Broadcast** engine with one **master** and multiple **slaves**; the master streams synchronized audio to the others. Link/unlink is the `A2DPBROADCAST_ENTER` / `A2DPBROADCAST_QUIT` action — i.e. the Auracast/party button on the speaker or the app's party screen.
- Speakers find each other via a **BLE manufacturer-specific advertisement** (AD type `0xFF`) that also carries the JBL model name; the firmware recognizes multiple JBL models (a `JBL Flip 5` template sits next to the Xtreme 4's), so cross-model grouping with compatible JBL speakers is expected.
- Phone pairing additionally uses **Google Fast Pair** (one-tap pairing on Android).

---

## 7. Evidence index (every firmware-derived offset cited)

Container: `HMHEADER`@`0x0`/`0x13ac2`; `PARTHMDT`@`0x52`,`0x13ace`,`0x2ef0a`,`0x111f8e`,`0x15c396`,`0x16439e`; `FOOTERHM`@`0x13ada`/`0x1843a6`; build meta@`0x184372`(`295a227`,`8.0.8.13`,`2024-7-30`). Code: Thumb-2 prologue@`0x13ae6`. SoC/SDK: `ZgmicroCLO_EN`@`0x113ba2`, `ZgmicroAEC_EN`@`0x113be7`, `VDSP_PlayerAlgProcess`@`0x113b3a`, `BLE_ZGMICRO`@`0x15d118`, `ZGMICRO`@`0x15d138`/`0xfeb92`, `bsp32_xtreme4.bin`@`0x1120da`, `proj_bsp32.X`@`0x111fe5`, cpp paths@`0x1138c6`/`0x1139bc`/`0x113a9a`/`0x113d8a`. Auracast/LC3: `LC3 Init Encode`@`0x113cd2`, `LC3 Init Decode`@`0x113d2e`, `VLBREncode_Init`@`0x113d1e`, `VLBRDecode_Init`@`0x113d7a`, `lbren/lbrde`@`0x103b7a`/`0x103b6a`, `DM_UPDATE_UNICAST_ADV`@`0x103dda`, `BTM_BLE_CONNECTED`@`0x103c12`. PartyBoost/broadcast: `App Task Broadcast`@`0x32d6e`, `Broadcast Queue`@`0x32fba`, `broadcast_master_exit`@`0x335a5`, `broadcast_slave_exit`@`0x33679`, `broadcast_isr`@`0x34945`, `master_exit Lock`@`0x34956`, `ABIS_MasterExit`@`0x3e9f1`, `csb_codec`@`0xbe8a1`, `Ecsb_TMR`@`0x58ebe`, `A2DP SRC/SNK Service`@`0x34b7e`/`0x34b92`, `A2DPBROADCAST_ENTER`@`0x103d9a`, `A2DPBROADCAST_QUIT`@`0x103db6`, `A2DPBROADCAST_SWITCH`@`0x103cfa`, `BTM_ACL_ROLE_CHANGED`@`0x103ba6`. Adv/manufacturer data: templates@`0x2ed00`–`0x2ee70`, `0B 09 "JBL Flip 5"`@`0x2ed92`, `5D "JBL Xtreme4"`@`0x2ed22`, `evd_event`@`0x2ed39`, `polling`@`0x2ed4a`. Control: `AT+HMDevSTARTPAIR`@`0xf6c2`, `AT+HMDevLINKED`@`0xf6b2`, `AT+GFPSWriteModelId`@`0xf672`, `GFPS3`@`0xfc1ed`. BLE partition: `JBL Xtreme 4`@`0x15c81d`, build `2024-09-09`@`0x16042e`, base64 key@`0x160778`.
