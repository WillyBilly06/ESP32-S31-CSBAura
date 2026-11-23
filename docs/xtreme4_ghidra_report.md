# JBL Xtreme 4 — image2.bin Ghidra reverse-engineering report

Target: `fw/image2.bin`, 1,509,612 bytes (0x1708EC). Carved from OTA `xtreme4_ota_all0650 (1).bin` at file offset 0x13AC2.
Ghidra 12.1.2 headless (PyGhidra absent → Java post-scripts), language `ARM:LE:32:Cortex`, raw BinaryLoader, base 0.
Project: `scratchpad/ghidra_proj/x4`. Auto-analysis produced **4436 functions**.

Honesty convention: `[FW @img-off]` = observed directly in image2 at that image offset; `[SPEC]` = Bluetooth-spec / general knowledge; `[INFER]` = my interpretation of FW evidence, not a literal datum. All offsets below are **image2 offsets** unless prefixed `file=`. `img = file − 0x13AC2`.

---

## 0. Header / code entry (verified)

- `[FW @0x00]` Bytes 0x00–0x23: Harman marker block — `48 4D 48 45 41 44 45 52` "HMHEADER", `PARTHMDT`, `FOOTERHM`, each + 4 zero bytes.
- `[FW @0x24]` First real code: `2D E9 FC 47` = `PUSH.W {r2,r3,r4,r5,r6,r7,r8,r9,r10,lr}` (reglist 0x47FC verified) — valid Thumb-2 prologue. Code is contiguous Thumb-2 from 0x24.
- `[FW]` No Cortex-M vector table at 0x00 (it is HMHEADER), so the absolute load address is not stored in the image and had to be recovered.

---

## 1. Base-address recovery — **base = 0x00000000** (flash), with evidence

Method and evidence (scripts: `scratchpad/scripts/litpool*.py`, `base_*.py`, `confirm_base0.py`):

1. `[FW]` **Memory map from PC-relative literal pools.** Decoded 13,927 `LDR Rd,[PC,#imm]` / `LDR.W` literal loads. Histogram of the loaded 32-bit constants:
   - **0x20xxxxxx = 6637** loads  → Cortex-M **SRAM** region (0x2000_0000).
   - **0x40xxxxxx = 860** loads   → Cortex-M **peripheral** region (0x4000_0000).
   - **0x00xxxxxx = 2606** loads   → **flash / code** region based at **0x0000_0000**.
   This is a textbook Cortex-M map and places flash at base 0.
2. `[FW]` **Branch-graph self-consistency.** Of 36,562 decoded `BL` instructions, **30,133 (82.4%)** land inside [0x24, 0x1708EC]. `BL`/`B` are PC-relative (base-independent); the high in-image rate confirms one contiguous code image decoded correctly.
3. `[FW]` **Strings are PC-relative, not absolute pointers.** Exhaustive search found **no** absolute string-pointer table (voting over every 32-bit word × every string; ADR/ADR.W/MOVW-MOVT scans; pointer-table delta search — all negative). Ghidra nonetheless resolved string xrefs correctly **at base 0** (e.g. it auto-named `s_csb_codec_000aaddf`, `s_broadcast_master_exit_0001fae3`, `s_App_Task_Broadcast_0001f2ac`), because those references are PC-relative `ADR`/`LDR` and therefore base-independent.

**Conclusion.** Load image2 at **0x00000000**. Consequences:
- All relative branches (`BL`/`B`) and all PC-relative data/string references (`ADR`, `LDR [pc]`) resolve **exactly** — this is the bulk of the analysis and it is correct.
- Absolute pointers into SRAM (`0x20xx_xxxx`) and peripherals (`0x40xx_xxxx`) resolve to those regions (outside the image, as expected) and show as `DAT_2000…` — correct behaviour, not an error.
- The only thing genuinely unknowable from this blob is whether the HMHEADER physically sits at flash 0x0 or a small fixed offset; it does not affect decompilation because nothing references the image by absolute flash pointer.

---

## 2. PartyBoost / Connectionless-Slave-Broadcast (CSB) link engine

image2 contains **both** the application broadcast engine (PartyBoost logic) *and* the lower BT-Classic host stack that issues the CSB HCI commands.

### 2.1 Application layer — "App Task Broadcast" + "ABIS" message API

- `[FW @0x1F2AC]` `"App Task Broadcast"`, `[FW @0x1F4F8]` `"Broadcast Queue"`, `[FW @0x9BE48]` `"Sync Queue"`.
- `[FW]` `FUN_00020B74` ("App Task Broadcast", entry 0x20B74): creates the broadcast OS task; sets a periodic timer of value `slot * (clock/DAT)` with period field `0x100` — the broadcast heartbeat. Creates the "Broadcast Queue" via `FUN_0001F4D4` (`FUN_000B32B0("Broadcast Queue", depth)`).
- `[FW]` **Role model.** A broadcast context byte at `ctx+1` encodes role: **1 = master, 2 = slave** (seen in `FUN_0001F8C4`/`FUN_0001F998`/`FUN_0001FB00`). Entering a new role first calls the opposite role's exit:
  - `FUN_0001F998` = **broadcast_master_exit** `[FW @str 0x1FAE3]`
  - `FUN_0001FB00` = **broadcast_slave_exit** `[FW @str 0x1FBB7]`
  - `FUN_0001F8C4` = **slave-enter** path (exits prior role, `state=4`, enables sub-systems `FUN_0003d950/37604/379cc/3ebb8`, `func_0x0041c0f4(1,0xffff)`, sets role=2, builds 0xB2 msg, creates Broadcast Queue, copies a **12-byte config block** `func_0x004284c4(dst, ctx+0x44, 0xc)`, starts codec `FUN_000ab5a8(1,0)`).

- `[FW]` **ABIS internal message API** (the app→stack command path). Messages are allocated, filled, and posted via `FUN_0005A8E0 → FUN_00046660(queue, 0, msg, flags)`:
  | Builder | Msg type (half @+8) | Len (half @+10) | Meaning |
  |---|---|---|---|
  | `FUN_0001F2F8` `[FW @0x1F2F8]` | **0xB1** | 0x0C (12) | CSB **master** cfg. Fields: role byte @+12 (0 exit / 4), cfg byte @+13, param_2 (16-bit) @+18 and @+20, param_3 (16-bit) @+22. |
  | `FUN_0001F334` `[FW @0x1F334]` | **0xB2** | 0x24 (36) | CSB **slave** cfg. Fields: role @+12 (1 enter / 0 exit), cfg byte @+19, word @+28, word @+24, half @+20, half @+32. |
  - `[FW]` `FUN_0001FC50`, `FUN_0002708C`, `FUN_000298B0` handle **msg types 0xB3 / 0xB4** = master/slave **role-switch / handover** triggers. Decision is gated on a keyed threshold lookup `FUN_00055AFC(id)` (8-byte-stride table, ≤0x20 entries, returns a byte or 0xFF) compared `> 0x2F` and `!= 0xFF` — i.e. an RSSI/link-quality-like metric `[INFER]`. This is the PartyBoost "who becomes master" negotiation.
- `[INFER]` So "A2DPBROADCAST_ENTER/QUIT/SWITCH" `[FW @0xF02D8/0xF02F4/0xF0238]` are the app-event labels that drive this: ENTER→role-enter builder, SWITCH→0xB3/0xB4 handover, QUIT→role-exit.

### 2.2 CSB broadcast **payload format** (from the ISR parser)

`[FW]` `FUN_00020D0C` (reached from `broadcast_isr` `[FW @str 0x20E83]`, ADR ref at 0x20E5A) parses the connectionless broadcast data block:
- Magic/opcode check `*param_1 == 0x7FE`.
- **Triple-redundant layout**: three copies at relative offsets **+0x02, +0x2A9 (+679), +0x550 (+1358)** — i.e. 3 × ~0x2A7-byte slots. Each slot is validated by a 13-byte CRC/hash `func_0x004252A0(ptr,0xD)` compared to a stored 16-bit check, then integrity-verified by `FUN_00054C00(ptr, len)`. Returns slot index 1/2/3 on first good copy, 4 on total failure.
- `[INFER]` This triple-copy + CRC scheme is how PartyBoost survives the lossy connectionless (no-ACK) broadcast channel — the same payload is transmitted three times and the receiver takes the first that checksums.

### 2.3 Actual BT-Classic **CSB HCI** commands (host stack, in the clear)

Verified by **MOVW 16-bit immediate loads** (reliable — unlike raw byte scans, which collide with `MOVS r0,#imm`). Decompiled issuer functions in `out/d3/`:
- `[FW @0x46D62]` `FUN_00046D62` loads **0x0C76** (OGF3 Set_Connectionless_Peripheral_Broadcast **Data** group `[SPEC]`), allocates a 6-byte cmd, sends via `func_0x0005F5C8`.
- `[FW @0x42606]` `FUN_00042606` registers the **link-control group 0x0437–0x043C** in a loop (`0x0437 Truncated_Page` `[SPEC]`, used by the CSB receiver to sync) via `FUN_00054694(opcode, buf, 7)`.
- `[FW]` Additional CSB / sync-train opcodes loaded via MOVW in the host stack (0x66k–0x6Dk region): **0x0C77 ×4** (@0x660BC/0x66B2A/0x67F96/0x6DC78), **0x0C7A, 0x0C7B, 0x0C7D ×3, 0x0C5F ×2**. These fall in the OGF-3 Controller-&-Baseband CSB / synchronization-train-parameter range `[SPEC]`.
- `[CAVEAT]` Exact OCF→name mapping (0x0C76 vs 0x0C77 = Set_CSB_Data vs Read_Sync_Train_Params, etc.) should be confirmed against the BT 4.1 Core spec; what is **FW-certain** is that these specific opcode constants are loaded and dispatched in image2.
- `[FW]` The precise **CSB interval, LT_ADDR, and sync-train interval** numeric values are **not compile-time constants** — they are carried in the RAM config structs (the 0xB1/0xB2 message fields and the 12-byte block at `ctx+0x44`) and populated at runtime, so they are not recoverable as literals from the image. The *field layout* (section 2.1) is recovered; the *runtime values* are not.

### 2.4 Audio path

- `[FW @0xAAD88]` `FUN_000AAD88` ("csb_codec" `[FW @str 0xAADDF]`) allocates a 1-byte codec-select, posts it via `FUN_000AB24C` — selects the codec feeding the CSB broadcast stream.
- `[FW @0x100224]` `"LC3 Init Encode"` / `[FW @0x100284]` `"LC3 Init Decode"`; internal codec name **"LBR"** `[FW @0x10025D]`. So the broadcast audio is **LC3** (Harman-branded "LBR").
- `[INFER]` Path: A2DP sink → re-encode via LC3/LBR → `csb_codec` → CSB connectionless broadcast payload (triple-redundant, §2.2) to slave speakers.

---

## 3. LE-Audio / Auracast (BIS) — **present in the clear in image2**, not hidden

- `[FW @0xF0344…0xF03AB]` State-machine label table (raw bytes verified):
  `BSRC_AUX_START,` · `NONE` · `BSRC_STOP` · `UC_STOP` · `BSNK_STOP` · `US_STOP` · `BSRC_START` · `UC_START` · `BSNK_START` · `US_START`.
  `BSRC` = Broadcast **Source**, `BSNK` = Broadcast **Sink**, `UC`/`US` = **Unicast**, `AUX` = auxiliary/periodic-advertising train `[INFER]`. `[FW @0xF0318]` `DM_UPDATE_UNICAST_ADV`.
- `[FW]` **Real LE-Audio BIS HCI builders** (MOVW opcode loads + decompiled, `out/d3/`):
  - `[FW @0x63B46]` `FUN_00063B46` → `FUN_00050F3C(0, 0x2068)` = **HCI LE_Create_BIG** `[SPEC]`. The function builds the full BIG/BIS parameter set: BIS count (`r4[0x2C]`), SDU-interval check `in_stack_c % 0x4E2` (**1250** = 1.25 ms ISO-interval granularity `[INFER/SPEC]`), burst/NSE fields (`r4[0x86]=r4[0x87]=0x33`), and a per-BIS event-offset array (`r4[0x318]/[0x338]`, `iVar8+0x1D0/+0x1D4`).
  - `[FW @0x64094]` `FUN_00064094` loads **0x2069** = LE_Create_BIG_Test.
  - `[FW @0x63134]` `FUN_00063134` loads **0x206B** = LE_BIG_Create_Sync (the **sink/receiver** side).
  - `[FW @0x68DAC]` loads **0x206C** = LE_BIG_Terminate_Sync.
  - `[FW]` Also MOVW **0x2041** ×2 (LE_Set_Adv_Set_Random_Addr), **0x2042** (LE ext-scan) in the advertising path.
- `[CAVEAT / corrected method]` Earlier raw-byte counting of `0x20xx` values is **unreliable** — `0x2068`/`0x2040` etc. collide with the Thumb `MOVS r0,#imm` encoding (e.g. the "opcode table" at 0x96732 is actually `MOVS r0,#N; BX LR` return-stubs). The reliable evidence above is MOVW-immediate loads **plus** decompiled call sites into the HCI dispatcher `FUN_00050F3C`.
- `[FW]` Debug strings `BASE`, `BIGInfo`, `Auracast`, `Periodic` are **absent**, but the BIS/periodic-advertising **functionality is unambiguously present** via the HCI command code. So the "opaque partition holds the real Auracast code" hypothesis is **not supported** — the BIS source+sink code is in the main image.

---

## 4. `BLE_ZGMICRO` partition probe — **readable config, NOT encrypted/compressed**

- `[FW @0x149656]` string `"BLE_ZGMICRO"`, `[FW @0x14966A]` `"ZGMICRO"` — partition/descriptor **labels**.
- `[FW]` Probed the region img **0x1488D4–0x1508DC** (= file 0x15C396–0x16439E, ~32 KB; script `probe_zgmicro.py`):
  - Shannon entropy per 4 KB window: **0.59 – 5.25** (mostly structured tables + 0xFF padding). Encrypted/compressed data would be ~7.9+ — this is not.
  - No zlib/gzip/raw-deflate/lzma/xz/lz4/bz2 header found; **no decompressor succeeded** at any tested offset.
  - Contents around the label are a descriptor record (counts, small LE ints, the ASCII names), i.e. a **config/partition table**, in the clear.
- `[FW]` **Where the genuinely high-entropy data actually is:** full-image 8 KB entropy sweep flags only **img 0x12000–0x17000 (≈7.70)** and **img 0x122000–0x125000 (≈7.91)**. `[INFER]` These are most likely **compressed voice/prompt audio or codec tables** (cf. embedded filename `"0kbsp32_xtreme4.bin"` `[FW @0xFE618]`), not protocol code. No base64 key or crypto container was confirmed anywhere; I did not brute-force.

---

## 5. BLE advertising / manufacturer data (partial)

- `[FW @0x1B2D0]` clean GAP AD structure `0B 09 "JBL Flip 5"` = len 0x0B, type **0x09 (Complete Local Name)**.
- `[FW @0x1B25C]` nearby `1F FF E4 41 …` = len 0x1F (31), type **0xFF (Manufacturer-Specific)**, followed by `"JBL Xtreme4"` `[FW @0x1B263]`; device name with space `"JBL Xtreme 4"` at `[FW @0xF672]`.
- `[CAVEAT]` This region (0x1B220–0x1B320) is **interleaved code+data**, so the exact manufacturer-specific byte layout (company-ID endianness, role/model/sync fields) cannot be stated reliably from the raw bytes alone — the company-ID bytes read ambiguously (`E4 41`). To pin the PartyBoost advertising template precisely, the next step is to decompile the **advertising-data builder** that assembles these fields (reachable from the `DM_UPDATE_UNICAST_ADV` / ext-adv path, opcodes 0x2036–0x2039/0x203E–0x2040). Not completed in this pass.

---

## 6. Decoded vs still-opaque

**Decoded (FW-grounded):**
- Base address = 0x00000000 (Cortex-M map, §1).
- PartyBoost/CSB **app engine**: task/queue, master↔slave role model, ABIS message API (0xB1/0xB2 config, 0xB3/0xB4 handover), link-quality-gated role switching.
- CSB **payload format**: triple-redundant ~679-byte slots + 13-byte CRC (§2.2).
- CSB **HCI opcodes** issued in image2: 0x0C76/0x0C77/0x0C7A/0x0C7B/0x0C7D, link-control 0x0437–0x043C (§2.3).
- **Audio**: LC3/"LBR" codec, csb_codec selector (§2.4).
- **LE-Audio BIS/Auracast** source+sink HCI builders: LE_Create_BIG 0x2068, Create_BIG_Test 0x2069, BIG_Create_Sync 0x206B, BIG_Terminate_Sync 0x206C, with BIG/BIS param construction (§3).
- **BLE_ZGMICRO** region = readable config, not encrypted (§4).

**Still opaque / not finished:**
- Runtime numeric CSB params (interval, LT_ADDR, sync-train interval) — held in RAM structs, not constants.
- Exact manufacturer-specific advertising byte layout (needs the adv-builder decompile; §5).
- The two high-entropy blobs at 0x12000 and 0x122000 (likely compressed audio prompts; not decompressed).
- Precise OCF→command-name mapping for the CSB opcodes (cross-check vs BT 4.1 spec).

---

## Artifacts
- Ghidra project: `scratchpad/ghidra_proj/x4` (image2.bin, 4436 functions, analyzed).
- Analysis scripts: `scratchpad/scripts/` (`litpool*.py`, `base_*.py`, `adr_scan.py`, `movwt_scan.py`, `probe_zgmicro.py`, `entropy_and_leaudio.py`, `GhDecompile*.java`).
- Exported decompilation: `./decomp/` next to this report — key functions: `fn_broadcast_master_exit_1f998`, `fn_broadcast_slave_exit_1fb00`, `fn_AppTaskBroadcast_20b74`, `fn_csb_codec_aad88`, `f_1f2f8/1f334` (ABIS 0xB1/0xB2 builders), `f_1f8c4` (slave-enter), `f_1fc50` (0xB3/0xB4 handover), `f_20d0c` (CSB payload parser), `in_46d62` (CSB 0x0C76), `in_42606` (Truncated_Page group), `in_63b46` (LE_Create_BIG 0x2068), `in_63134` (LE_BIG_Create_Sync 0x206B), `in_64094` (0x2069), `in_55afc` (role-quality lookup), `in_5a8e0` (ABIS post).
