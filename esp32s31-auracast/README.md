# ESP32-S31 Auracast demo (behave like the JBL button)

A starting-point ESP-IDF project that turns an **ESP32-S31** into an Auracast
device that can **broadcast** (so a JBL Xtreme 4 can receive it) and
**receive** (join a JBL's Auracast broadcast). One button cycles
`IDLE → BROADCAST → RECEIVE`, mimicking how the JBL Auracast button feels.

> **Reality check — read first**
> - **Auracast on the S31 is the viable path.** The S31 has BLE 5.4 LE Audio
>   with isochronous channels (BIS/CIS) and ESP-IDF ships LE-Audio broadcast
>   source/sink examples. The JBL Xtreme 4's **Auracast** mode is the *open*
>   Bluetooth-SIG standard (Public Broadcast Profile), so S31 ↔ JBL interop is
>   realistic at the spec level.
> - **PartyBoost is NOT reachable here.** JBL's press-button speaker-link is
>   **PartyBoost**, which runs on Bluetooth *Classic* **Connectionless
>   Peripheral Broadcast (CPB,** formerly CSB**)** plus a proprietary handshake.
>   The S31 *has* a Classic radio, but ESP-IDF exposes **no CPB/CSB API**, and
>   A2DP is single-sink — so PartyBoost can't be reproduced from the SDK. This
>   project is Auracast only.
> - **The LE-Audio API is a PREVIEW.** Treat the code here as a scaffold. The
>   authoritative signatures are in the official examples (below). Spots that
>   need reconciling are marked `>>VERIFY<<` in `main/auracast_main.c`.
> - Confirm broadcast source **and** sink are supported for the `esp32s31`
>   target on your IDF version:
>   `docs .../esp32s31/api-guides/esp-ble-audio/ble-audio-feature-support-status.html`

## Why "Auracast", not "emulate the JBL"
The Xtreme 4 runs **two independent systems that don't interoperate**:

| System | Transport | Openness | On the S31? |
|---|---|---|---|
| **Auracast** | BLE / LE Audio (BIS, LC3) | Open SIG standard | ✅ supported (this project) |
| **PartyBoost** | BT Classic CPB/CSB + A2DP | JBL-proprietary | ❌ no SDK API for CPB |

So "make the S31 work with JBL speakers" = **speak standard Auracast to/from
the JBL's Auracast mode** — not reproduce the proprietary PartyBoost link.

## Build & flash
```bash
# ESP-IDF >= 5.5 required
idf.py set-target esp32s31
idf.py menuconfig        # confirm the LE-Audio / ISO symbols (see sdkconfig.defaults)
idf.py build flash monitor
```

## Recommended workflow (most reliable)
Because the API is preview, **start from the official example and layer this
app's button/mode logic on top** rather than hand-writing LE-Audio calls:

```bash
# Public broadcast SOURCE (unencrypted Auracast sender):
idf.py create-project-from-example "espressif/esp_ble_audio:pbp/pbs"   # or copy from $IDF_PATH/examples/...
# Public broadcast SINK (receiver):
idf.py create-project-from-example "espressif/esp_ble_audio:pbp/pbk"
```
Official example trees (exact signatures live here):
- `$IDF_PATH/examples/bluetooth/esp_ble_audio/pbp/pbs` — public source
- `$IDF_PATH/examples/bluetooth/esp_ble_audio/pbp/pbk` — public sink
- `$IDF_PATH/examples/bluetooth/esp_ble_audio/bap/broadcast_source|broadcast_sink|broadcast_assistant`

## Interop notes with the JBL Xtreme 4
- **S31 broadcasts → JBL receives:** Put the JBL into Auracast *receive* via the
  JBL Portable app (the app is the **Broadcast Assistant** that points the
  speaker at a broadcast). Keep the S31 broadcast **public/unencrypted** first
  (`AURACAST_PUBLIC=1`, no broadcast code) to remove the code as a variable.
- **S31 receives → JBL broadcasts:** Start Auracast *send* on the JBL, then let
  the S31 sink scan + sync. You **don't pick the LC3 preset** when receiving —
  you read it from the JBL's **BASE** on sync. If the JBL broadcast is
  encrypted, you need its **broadcast code** (a wrong code syncs then fails
  integrity quickly — that's your signal).
- **Get ground truth cheaply:** a 10-minute **BLE sniff** (nRF Sniffer / a phone
  Auracast scanner) of the JBL while it broadcasts shows the real broadcast
  name, broadcast ID, LC3 preset, and whether it's encrypted — faster than any
  disassembly.

## Files
```
esp32s31-auracast/
├─ CMakeLists.txt           top-level project
├─ sdkconfig.defaults       real Kconfig from pbp/pbs (ISO/BAP/CAP/PBP)
└─ main/
   ├─ CMakeLists.txt        compiles auracast_main.c + the example's adv.c
   ├─ idf_component.yml      idf>=5.5 + example_init/example_utils helper deps
   └─ auracast_main.c       button/mode app + REAL broadcast API; sink outlined
```
To build: start from the example so the helper files come along, then drop in
`auracast_main.c`:
```bash
idf.py create-project-from-example "espressif/esp_ble_audio:pbp/pbs"
# copy auracast_main.c over main/main.c; copy this sdkconfig.defaults; keep the
# example's adv.h, bluedroid/adv.c (or nimble/adv.c), and common_components.
idf.py set-target esp32s31 && idf.py build flash monitor
```

## Status
- App/button/state-machine layer: **complete.**
- **Broadcast (source) path: real ESP-IDF API** (`esp_ble_audio_cap_initiator_
  broadcast_audio_create/start`, preset `..._48_2_1_DEFINE`, `esp_ble_audio_cap_
  stream_send`), verified against esp-idf `@ad20ded6` pbp/pbs. A few `>>VERIFY<<`
  spots depend on example helpers (ADV_HANDLE, adv_info, scheduler).
- Receive (sink) path: **outline**, to fill from `pbp/pbk`.
- **Not build-verified here** (no S31 toolchain in this environment).

## Related
- `../s31-csb-probe/` — flash this first if you also want PartyBoost: it tells
  you whether the S31 controller even implements the CSB/CPB HCI opcodes (the
  one thing gating a PartyBoost attempt).
