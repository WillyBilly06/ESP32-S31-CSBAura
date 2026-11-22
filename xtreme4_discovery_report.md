# JBL Xtreme 4 — Over-the-air discovery: PartyBoost vs Auracast discriminator

How the Xtreme 4 recognizes, from a received BLE advertising PDU, whether a nearby device is a **JBL PartyBoost speaker** vs a **standard Auracast (LE-Audio) broadcaster** — the concrete AD types, UUIDs, company IDs, byte offsets, and the scan/parse/classify code path.

Target: `fw/image2.bin` (main app, 1,509,612 B), loaded at base **0x0**, so image2 offset == memory address. Ghidra 12.1.2, 4436 functions. This report completes the advertising-builder + scanner decompile that §5 of `xtreme4_ghidra_report.md` left open.

Honesty convention: `[FW addr]` = observed directly in image2 (decompiled or raw bytes); `[SPEC]` = Bluetooth-spec / assigned-numbers knowledge; `[INFER]` = my interpretation of FW evidence. I never invent an offset or company ID; where a value lives only in RAM at runtime I say so.

---

## 0. TL;DR — the discriminator

Given a received advertising PDU, the Xtreme 4 classifies the sender by **which AD structure it contains** — this is decided in code, not from any stored template:

| Sender type | Discriminating AD (from the received report) | Field the FW keys on |
|---|---|---|
| **JBL PartyBoost speaker** | **AD type `0xFF` (Manufacturer Specific Data)** whose first two bytes (company ID, little-endian) are **`0xFDDF`** *or* **`0x0057`** (Harman) | then a **nested sub-record with tag `0x02`** carries the JBL **mode/role byte** |
| **Standard Auracast broadcaster** | **AD type `0x16` (Service Data – 16-bit UUID) with UUID `0x1852`** (Broadcast Audio Announcement), payload = **3-byte Broadcast_ID**; the **BASE** lives in periodic adv as **Service Data UUID `0x1851`** | match is on the **3-octet Broadcast_ID** (RX matcher mode 3) |
| **neither** | no `0xFF` AD with company `0xFDDF`/`0x0057`, and no `0x16`/`0x1852` service data | classifier returns `0xFF` ("not ours") |

So the single over-the-air discriminator is: **a type-`0xFF` manufacturer AD with JBL company ID `0xFDDF`/`0x0057` ⇒ PartyBoost; a type-`0x16` Service-Data AD with UUID `0x1852` ⇒ Auracast.** These are mutually exclusive AD structures and are tested by two different code paths.

`[FW addr]` The PartyBoost test is `FUN_00022370` @ **0x22370**. The Auracast advertisement is assembled by `FUN_00022f5c` @ **0x22f5c** / `FUN_0002a78c` @ **0x2a78c** and the matcher is `FUN_000374b8` @ **0x374b8**.

Note on the earlier anchor: the raw bytes near image2 **0x1B25C–0x1B2D0** (`1f ff e4 41 … 0c 5d "JBL Xtreme4" … 0b 09 "JBL Flip 5"`) are **interleaved Thumb-2 code + the two model-name string literals**, *not* a literal advertising template. `[FW addr]` The strings `"JBL Xtreme4"` @0x1B263 and `"JBL Flip 5"` @0x1B2D0 are real, but a reference scan found **no code xref** to either address, and the surrounding `1f ff …` / `0c 5d …` bytes decode as instructions. The real AD bytes are built at runtime by the primitives in §3. This resolves the "company ID reads ambiguously (E4 41)" caveat from the prior report: that `e4 41` is opcode bytes, not a company ID.

---

## 1. Advertisement builders (what the Xtreme 4 TRANSMITS)

### 1.1 AD-assembly primitives — exact byte layout `[FW addr]`

Two leaf functions assemble every AD structure; their byte layout is unambiguous.

**`FUN_0002b78a` @ 0x2b78a — append Service-Data (16-bit UUID) AD** (`param`: dst, uuid16, payload, len):
```
dst[0] = len + 3            ; AD length
dst[1] = 0x16              ; AD type = Service Data - 16-bit UUID
dst[2] = uuid & 0xFF       ; UUID low
dst[3] = uuid >> 8         ; UUID high
dst[4..] = payload[len]    ; memcpy
returns len+4
```

**`FUN_0002b75a` @ 0x2b75a — append Manufacturer-Specific Data AD** (`param`: dst, companyID, payload, len):
```
dst[0] = len + 3            ; AD length (clamped so len+3 <= 0xFB)
dst[1] = 0xFF              ; AD type = Manufacturer Specific Data
dst[2] = companyID & 0xFF  ; company ID low
dst[3] = companyID >> 8    ; company ID high
dst[4..] = payload[len]
returns len+4
```

These two functions are the ground truth for the byte-level layout of both the Auracast and PartyBoost adverts.

### 1.2 Auracast broadcast-source advert `[FW addr]`

`FUN_00022f5c` @ **0x22f5c** and `FUN_0002a78c` @ **0x2a78c** (two mode variants; both reached from dispatcher `FUN_00029ee0` @ 0x29ee0) build the extended + periodic advertising set with this exact sequence:

```c
// extended advertising data:
off  = FUN_0002b78a(buf+0x31, 0x1852, broadcast_id, 3);   // Service Data, Broadcast Audio Announcement
off += FUN_0002b6de(..,  broadcast_name, ..);             // Broadcast Name AD
off += FUN_0002b75a(.., 0x57 /*or 0xFDDF*/, mfg_payload, len); // Manufacturer Specific Data (JBL)
// periodic advertising data:
       FUN_0002b608(.., base_struct);                     // BASE via Service Data UUID 0x1851
```

- `broadcast_id` = `local_38[0] = 0x8105` by default, or a 24-bit value `*(DAT+0x1e) & 0xFFFFFF` when config byte `*(DAT+0x1d)==3`. `[FW addr]` So the 3-octet Broadcast_ID is **runtime-configured**; `0x8105` is the compile-time default seen in the image.
- The Manufacturer-Specific AD uses company **`0x57`** here; in the `*(DAT+0x1d)==6` branch the builder stacks `0x57` + `0xFDDF` + an 18-byte payload. The 6-byte default payload comes from a RAM pointer (`DAT_000231c8`), so the exact mfg bytes are **runtime**, not a literal.

`FUN_0002b608` @ **0x2b608** builds the **BASE** (Basic Audio Announcement) by calling `FUN_0002b78a(.., 0x1851, ..)` — i.e. Service Data UUID **0x1851** in the periodic train, carrying per-BIS codec/metadata (sub-builders `FUN_000364b4`/`FUN_0003651e`). `[SPEC]` This is the standard Auracast BASE.

### 1.3 PartyBoost / CSB advert `[FW addr]`

`FUN_000beea4` @ **0xbeea4** (in the CSB/`csb_codec` subsystem, ~0xbe region) builds its advert as:
```c
i  = FUN_0002b608(buf, base);                       // BASE (Service Data 0x1851)
i += FUN_0002b75a(buf+i, 0xFDDF, payload, len);     // Manufacturer Specific Data, company 0xFDDF
FUN_0003a658(buf, i);                               // hand off to the advertiser
```
The company ID is a verified literal immediate: `[FW addr]` at **0xbeedc** the instruction bytes are `4f f6 df 51` = `MOVW r1,#0xFDDF`, and r1 is passed straight into `FUN_0002b75a` (the BL at 0xbeee0). Ghidra renders it `&DAT_0000fddf`, but it is never dereferenced — it is the value `0xFDDF`. (The bytes physically at 0xfddf are unrelated code.)

**Builder dispatcher** `FUN_00029ee0` @ **0x29ee0** maps app events to these builders: event `0x105`/`0x106` → start advertising (choose `FUN_0002a78c`/`FUN_00022f5c` when the LE-audio coprocessor flag `*(ctx+9)!=0`, else a CSB-style record via `FUN_0002c968(0x21)`); `0x107` → update; `0x108` → start scan (builds a scan-param block `0x22,0x0c,300,300` and a link-control `0x431` record — the CSB/Truncated_Page range); `0x109` → stop.

---

## 2. Scanner / parser / classifier (what it does with RECEIVED ads)

### 2.1 PartyBoost classifier — the concrete match `[FW addr]`

**`FUN_00022370` @ 0x22370** is the PartyBoost recognizer. Faithful transcription:

```c
uint FUN_00022370(byte *ad) {               // ad -> one AD structure: [len][type][..]
  if ((ad[1] == 0xFF) &&                     // AD type == Manufacturer Specific Data
      (*(short*)(ad+2) == 0xFDDF ||          // company ID (LE) == 0xFDDF   (-0x221)
       *(short*)(ad+2) == 0x0057)) {         //            or    == 0x0057  (Harman)
     for (i = 4; i < ad[0]; i += ad[i] + 1) {// walk nested sub-records from offset 4
        if (ad[i] == 0) return 0xFF;         // sub-length 0 -> stop
        if (ad[i+1] == 0x02)                 // nested sub-record tag == 0x02
            return ad[i+2];                  //   -> return the JBL mode/role byte
        if (ad[i+1] == 0xFF && recurse != 0xFF) return recurse; // concatenated 0xFF AD
     }
  }
  return 0xFF;                               // not a JBL PartyBoost advert
}
```

Byte map of a received PartyBoost manufacturer AD, as this parser reads it:
```
+0  len
+1  0xFF                         (AD type: Manufacturer Specific Data)
+2  company ID low  }  = 0xFDDF (DF FD)  OR  0x0057 (57 00), little-endian
+3  company ID high }
+4  nested sub-records: [sub_len][sub_tag][sub_val...]
      sub_tag 0x02 -> sub_val = JBL mode/role byte   <-- the signalling field
```

`[INFER]` The **nested tag-`0x02` byte is the JBL "mode/role/state" field** — the thing that tells a peer "PartyBoost active, I am master/slave/idle". Its exact value set is set on the TX side from RAM config (§1.2), so the literal numbers are **not recoverable from the image**; what *is* FW-certain is the discriminator (company `0xFDDF`/`0x0057` + nested tag `0x02`) and that the returned byte drives the group-role logic. `0xFF` is reserved to mean "no match / not a JBL ad".

### 2.2 Scan-result caller + AD finder `[FW addr]`

**`FUN_0002338c` @ 0x2338c — AD finder**: walks a received AD payload `[len][type]…` and returns the first structure whose **type byte == 0xFF**. This is how the stack extracts the manufacturer AD from a scan report before classifying.

**`FUN_000223bc` @ 0x223bc — scan-result classifier/callback**:
1. `FUN_0002338c(report+2, report[1])` → locate the `0xFF` manufacturer AD.
2. Re-checks company field `== 0xFDDF (-0x221)` or `== 0x0057`.
3. `FUN_00022370(ad)` → extract the mode byte.
4. Stores it in the per-peer record at `+0xD6` (`+0xD7` = "seen JBL peer" flag), maps `0xFF`→`0x28`, and feeds it to `FUN_000222f8` (a 0–100 metric scaler, `[INFER]` RSSI/priority). Mode `0` triggers a teardown branch (clears state bits `0x4000000`/`0x800000`).

**Entry point**: `FUN_0003754e` @ **0x3754e** (scan/adv-report event handler) dispatches BLE report event **`0x12001A`** → `FUN_000223bc`. So the full PartyBoost RX pipeline is:
```
BLE scan report (event 0x12001A)
  -> FUN_0003754e (0x3754e)
  -> FUN_000223bc (0x223bc)       scan-result classifier
     -> FUN_0002338c (0x2338c)    find the 0xFF manufacturer AD
     -> FUN_00022370 (0x22370)    company 0xFDDF/0x0057? extract tag-0x02 mode byte
  -> store peer role at ctx+0xD6, drive master/slave group logic
```
Scanning itself is LE **extended scan**: HCI `LE_Set_Extended_Scan_Parameters` (**0x2041**) and `LE_Set_Extended_Scan_Enable` (**0x2042**) `[SPEC]`, issued via `FUN_00039980` @0x39980, `FUN_00038baa` @0x38baa, `FUN_000c8194` @0xc8194 (`param_2==6` → scan enable). (Correction to the prior report: 0x2041/0x2042 are the ext-scan commands, not Set_Adv_Random_Addr.)

### 2.3 Auracast recognition / receive matcher `[FW addr]`

**`FUN_000374b8` @ 0x374b8** is the periodic/extended-advert **match engine** used to recognize and lock onto a selected broadcaster. It is driven by a target descriptor (`DAT_00023384`) whose byte `+0x1D` selects the **match mode**, with the target value at `+0x1E`:

| match mode (`+0x1D`) | what it compares in the received report |
|---|---|
| `0` | finds the `0xFF` manufacturer AD (`FUN_0002338c`) and uses it |
| `1` | 2-byte field at mfg_AD `+2` (company / id) vs stored `+0x1E` |
| `2` | `memcmp` of mfg payload (`+4`) vs stored (address/name match) |
| **`3`** | **`*(uint*)(report+9) & 0xFFFFFF` (the 3-octet Broadcast_ID) vs `*(DAT+0x1E) & 0xFFFFFF`** |
| `4` | a tagged sub-record via `FUN_00034926` |
| `5` | single byte `report[2]` vs stored |
| `6` | requires `len >= 0x15`; compares 2-byte `+2` and 2-byte `+0x14` |

`[FW addr]` **Mode 3 is the standard Auracast match**: it keys on the **3-byte Broadcast_ID** that lives in the Service Data UUID 0x1852 payload. `[INFER]` The target Broadcast_ID is supplied by the app/assistant (written into `DAT_00023384+0x1E`), consistent with the JBL app acting as broadcast assistant.

**BASE parse on RX**: `FUN_0002b80a` @ **0x2b80a** calls `FUN_0003492c(report, len, 0x1851)` to locate the **Service Data UUID 0x1851 (BASE)** in a received periodic advert and unpacks its BIG/BIS codec records (3-octet presentation delay at `+4`, num_subgroups at `+7`, 5-byte codec IDs). This is the sink reading the BASE before syncing.

**Sink sync**: `FUN_00063134` @ 0x63134 issues HCI `LE_BIG_Create_Sync` (**0x206B**) `[SPEC]` to join the BIG. (Create side: `LE_Create_BIG` 0x2068 @0x63b46; terminate 0x206C @0x68dac.)

**Full LE-Audio UUID inventory present in image2** (all via `MOVW` immediates — `[FW addr]`):
| UUID | Service / announcement | Where |
|---|---|---|
| `0x1851` | Basic Audio Announcement (BASE) | built @0x2b608, parsed @0x2b80a |
| `0x1852` | **Broadcast Audio Announcement** | built @0x22f5c, @0x2a78c |
| `0x1853` | Common Audio Service (CAS) | built @0x2b6fc |
| `0x184E` | Audio Stream Control Service (ASCS, unicast) | @0x2b5c0, @0x2c404, @0x9d83c |
| `0x1850` | Published Audio Capabilities (PACS) | @0x2c404 |
| `0x1844` | Volume Control Service | @0x2c404 |

`[FW addr]` **Not found as immediates (0 hits across 202,868 instructions):** `0x1856` (Public Broadcast Announcement / PBP), `0x184F` (BASS / scan-delegator GATT), and `0x2044` (`LE_Periodic_Advertising_Create_Sync`). `[INFER]` So in this image2 the Xtreme 4 announces itself as a plain Broadcast Audio Announcement (0x1852) broadcaster — no PBP marker — and its scan-delegator/BASS surface and PA-create-sync are either in the separate `BLE_ZGMICRO` coprocessor partition or are driven indirectly; they are **not** in the main image. The receive-side recognition that *is* in image2 keys on the Broadcast_ID (mode 3 above), not on a scan for 0x1852 service data.

---

## 3. The discriminator, stated plainly

Given a received advertising PDU, the Xtreme 4 decides:

1. **Is there an AD structure of type `0xFF` (Manufacturer Specific Data)?** `FUN_0002338c` finds it. If its first two bytes (company ID, LE) are **`0xFDDF`** or **`0x0057`**, it is a **JBL PartyBoost speaker** (`FUN_00022370` → nested tag-`0x02` byte gives its role/mode). Any other company ID ⇒ not JBL.

2. **Is there an AD structure of type `0x16` (Service Data – 16-bit UUID) with UUID `0x1852`?** That is a **standard Auracast broadcaster**; its 3-octet payload is the **Broadcast_ID**, which the receive matcher (`FUN_000374b8` mode 3) compares against the app-selected target. The broadcaster's BASE is Service Data UUID `0x1851` in its periodic train.

3. **Neither present** ⇒ not a relevant device; the PartyBoost classifier returns `0xFF`.

The two are distinguished by **AD type and identifier**: `0xFF`+company `0xFDDF`/`0x0057` (JBL proprietary) vs `0x16`+UUID `0x1852` (SIG-standard Auracast). They do not overlap, and each has its own parse routine.

### What is FW-certain vs runtime
- **FW-certain (byte-level):** AD types `0xFF`/`0x16`; company IDs `0xFDDF` and `0x0057`; UUIDs `0x1851`/`0x1852`/`0x1853`/`0x184E`/`0x1850`/`0x1844`; the nested PartyBoost tag `0x02`; the Broadcast_ID being a 3-octet field matched in mode 3; all the function addresses above; ext-scan HCI `0x2041/0x2042`; BIG sync `0x206B/0x2068/0x206C`.
- **Runtime-only (not recoverable as literals):** the exact PartyBoost mode-byte values behind tag `0x02`; the actual 6-/18-byte JBL manufacturer payloads (from RAM pointers `DAT_000231c8` etc.); the live Broadcast_ID (default literal `0x8105`, else configured); CSB interval/LT_ADDR.
- **Not in image2 (likely in the `BLE_ZGMICRO` coprocessor):** PBP `0x1856`, BASS `0x184F`, `LE_Periodic_Adv_Create_Sync 0x2044`.

### 0xFDDF caveat `[SPEC/INFER]`
`0x0057` is the Bluetooth-SIG company identifier for **Harman International Industries** `[SPEC]`. `0xFDDF` is written and checked in the company-ID field position of the `0xFF` AD `[FW addr]`; I did not find it in a SIG company-ID table in this image, so I report it only as the literal value the firmware emits and accepts — I do not assert what entity it is officially assigned to.

---

## 4. Function index (all image2 addresses)

| Addr | Role |
|---|---|
| 0x22370 | **PartyBoost classifier** — AD type 0xFF + company 0xFDDF/0x0057 + nested tag 0x02 → mode byte |
| 0x223bc | PartyBoost scan-result callback (calls 0x2338c + 0x22370, stores peer role at ctx+0xD6) |
| 0x2338c | AD finder — returns first AD structure with type byte 0xFF |
| 0x3754e | BLE report event handler; event 0x12001A → 0x223bc |
| 0x374b8 | Auracast / periodic-adv **match engine** (mode 3 = 3-byte Broadcast_ID) |
| 0x22f5c, 0x2a78c | **Auracast broadcast-source ad builders** (Service Data 0x1852 + name + mfg + BASE) |
| 0x2b608 | BASE builder (Service Data UUID 0x1851) |
| 0x2b80a | BASE **parser** on RX (FUN_0003492c(.,.,0x1851)) |
| 0x2b78a | primitive: append Service-Data AD `[len][0x16][uuidLE][payload]` |
| 0x2b75a | primitive: append Manufacturer AD `[len][0xFF][companyLE][payload]` |
| 0xbeea4 | **PartyBoost/CSB ad builder** — BASE + mfg AD company **0xFDDF** (MOVW @0xbeedc) |
| 0x29ee0 | adv/scan dispatcher (events 0x105/0x106 adv, 0x108 scan, 0x109 stop) |
| 0x39980, 0x38baa, 0xc8194 | LE ext-scan HCI (0x2041 params / 0x2042 enable) |
| 0x63134 / 0x63b46 / 0x68dac | LE BIG_Create_Sync 0x206B / Create_BIG 0x2068 / Terminate_Sync 0x206C |
| 0x2b6fc / 0x2b5c0 / 0x2c404 / 0x9d83c | LE-Audio UUID builders 0x1853 / 0x184E / 0x1850+0x184E+0x1844 / 0x184E |

Exported decompilations for all of the above are in `scratchpad/out/d4/` and `scratchpad/out/d5/` (per-function `fn_<addr>.c`); constant/xref summaries in `out/d4/_summary.txt` and `out/d5/_summary.txt`. New Ghidra scripts: `scratchpad/scripts/GhDecompile4.java`, `GhDecompile5.java`.
