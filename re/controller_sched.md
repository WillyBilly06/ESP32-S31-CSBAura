# ESP32-S31 controller: BR/EDR vs LE scheduling arbitration (RE notes)

Source blobs: `components/bt/controller/lib_esp32s31/esp32s31-bt-lib/{libbtdm_common.a,libbredr_app.a,libble_app.a}`
(IDF tree `esp-idf-v6.1-codecs`). Method: `ar x` + `objdump -dr` + rodata dumps (scratch:
`...\scratchpad\re_ctrl\`, a local temp dir).
Object names below: `common/N.o`, `ble/N.o` = numbered archive members, `bredr/*.c.o` = named members.
Tags: **[C]** = confirmed in disassembly/data, **[I]** = inference.

## 0. Second pass (corrections + priority model used by `main/bt_sched_tune.c`)

Hardware observations it has to explain (2 BIS, ISO 20 ms, BN 2, NSE 8, IRC 4, 100 B, 2M; phone A2DP sink, S31 = BR/EDR slave):
ACL base 2 -> NOT SENT 0, A2DP ~80 %; ACL base 8 + sequential -> A2DP perfect, NOT SENT 26 %; ACL base 8 + interleaved,
phone connected but *paused* -> NOT SENT 25-34 % (4-8 % with no phone).

### 0.1 BIS subevent priority (`ble/56.o r_sym_ble_FtKLgw2CvtskaTYKEjHw`) **[C]**
* Called only from the BIG event builder `r_sym_ble_tXihWMtPcP61uTJBYYpS`, which inserts **one fixed-time sched item per
  subevent per BIS** (loop counter checked against NumBIS*NSE at +0x356). Item start = `bis_idx*big[+20] + se*big[+48]`
  (+0x112..0x126), i.e. BIS_Spacing / Sub_Interval, so packing only changes *start times*, never the priority.
* "subevent field bits[5:1]" = `bis->word0[5:1]` = **the subevent index `se` (0..NSE-1) within that BIS's event** — not a
  global index over all BIS. It is mirrored in `item+24[23:19]` (asserted equal at tXihWM+0x162), advanced by
  `r_sym_ble_2OrMP2KRFw00zNEkZd0R`; `r_sym_ble_WyHxzomnmoXUwTAR0dY9` ends the event when the last BIS reaches se = NSE-1.
  `bis->word0[6]` is a separate flag (FtKL +0x8a).
* `k = se / BN` = payload *group* index (spec numbering: group g carries payloads 0..BN-1 in se = g*BN + p).
  `prio = k==0 ? cfg[33] : (k < IRC ? cfg[34] : cfg[35])` -> with BN 2, NSE 8, IRC 4: se 0,1 -> 9 (first copy of both
  payloads), se 2..7 -> 7. Same for SEQUENTIAL and INTERLEAVED. So yes, **the first copy of every payload gets cfg[33]**.
* `bis == NULL` path (+0x2b8, taken when `big+332 & 0x10`) = BIG control subevent (channel map / terminate): `cfg[36]` (12).
* `cfg[37..39]` (9/12/12) are only read by `ble/56.o r_sym_ble_1Lwqi..` (adaptive 4-bit prio at obj+284 stepping between
  them) -> CIS **[I]**. `cfg[32]` (0x1F) is written by `ble/60.o` to HW reg `0x20101B88` (5-bit, meaning unknown; untouched).
* Priority nibble: `r_sym_ble_J21QYaRyf1RwwT30YZpY` masks `& 15` and writes low+high nibble -> legal range 0..15; 15 is
  above anything the BR/EDR side can produce (its cap is 14).
* Getter: `uint8_t *r_sym_ble_kBC0rRwtjATsFkriXVDL(void)` returns exactly `r_sym_ble_u66q9VTswFBa3W6uzpwY` (same
  `.LANCHOR1` = `.bss.r_sym_ble_u66q..`, checked with readelf); both global. Defaults re-applied only via
  `jjkD..` <- `ble/26.o fFDl..` (ISO init / HCI_Reset); nothing on BIG create.

### 0.2 How `not_txd_pkts` arises **[C] partly / [I]**
`ble/29.o r_sym_ble_UqGSyDuyu6DSGTF1giOq` builds the TX-complete info: per released SDU buffer `completed += n`, and
`not_txd += n` if `(buf->flags16 & 0xA00) != 0xA00`. Bit 9 is set by `ble/56.o r_sym_ble_wimO0rEwECrJOsxVhRaR`
(tx of the PDU in a subevent) **[C]**. **[I]** => an SDU counts as NOT SENT only when *none* of its IRC copies went on air
(the first copy *and* all repetitions of that payload lost arbitration). Since every subevent is an independent fixed-time
item, losing one subevent does not cancel the others; a lost subevent is never re-placed later (no ASAP mode for BIS).

### 0.3 BR/EDR ACL priority escalation (`r_olc_acl_sched_peripheral_asap.constprop.0`, `.iram1.54`) **[C]**
```
cap = 14;  if (acl->f54 & 2) cap = (dft[14] <= 4 || dft[0] >= dft[14]-4) ? dft[0] : dft[14]-4;   // dft[14] = 6
prio  = dft[0] + min(cap, (txq + rxq) >> 2)            // r_olc_acl_datapath_get_traffic: bytes +79/+80 = queued pkts
prio += min(cap, (now - last_rx_access_code + 1250us) / Tpoll)   // only with the "has deadline" arg (a3=1, always from recycle_sch)
prio  = min(prio, cap)
```
`cap 14` is an immediate (`li s9,14`) in IRAM code, not data; in-list losses re-insert with `prio = min(14, prio + inc[0])`
(`r_olc_sched_insert` +0x13c). The peripheral item is ASAP: start = "now", deadline = last_sync + Tpoll
(`r_olc_acl_sched_recycle_sch` +0x4b0..0x52c), duration `((txpend?1:0)+tx_slot+rx_slot)*625-189 us`, and it is re-issued from
`recycle_sch` after every exchange -> **[I]** an active-mode slave is listening (back-to-back 1-2 slot windows) essentially
all the time, idle or streaming. inc table byte for ACL (`tbl[1]`) = 1.

**Corrected explanation of the 25-34 % loss with base 8 [I, consistent with all three data sets]:**
with base 8 the ACL listen windows (8) beat every repetition (7), so each SDU survives only through its first copy (9).
Any escalation unit (one Tpoll without a received access code, or >= 4 queued packets) lifts the ACL to 9 (tie) or >= 10
and it then also beats the first copies; those SDUs are lost completely. With base 2 the ACL needs >= 8 units to reach 10,
which practically never happens -> NOT SENT 0. The old "k==0 is protected because 9 > 8" model ignored the escalation terms.

### 0.4 Other LE priorities (not tunable: immediates in code) **[C]**
adv `ble/50.o`: 1 (current set) / 5, +1 per skipped event up to 9 (`A9ld..`) or 10 (`GZEt..`); periodic adv 10; scan 1/4;
sync 6/12; LE ACL connection 8 escalating to 14/15 (`ble/51.o`) — none used by this product while streaming **[I]**.

### 0.5 Configuration implemented in `s31-jbl-speaker/main/bt_sched_tune.c`
| profile | ACL dft[0] | BIS first `[33]` | rep `[34]` | pto `[35]` | ctrl `[36]` |
|---|---|---|---|---|---|
| stock | 2 | 9 | 7 | 8 | 12 |
| idle (no A2DP audio, both modes) | 2 | 15 | 12 | 11 | 15 |
| streaming, `BT_SCHED_MODE_A2DP_FIRST` (default) | 14 | 13 | 12 | 11 | 13 |
| streaming, `BT_SCHED_MODE_AURACAST_FIRST` | 8 | 15 | 7 | 7 | 15 |

* ACL 14 = the cap, so `prio` is constant 14 (escalation adds nothing, and `P:%d new pti` is not logged since prio == dft).
  A real nibble 15 for the ACL would need patching the `li 14` immediates in IRAM code -> not done. 14 already beats
  every LE item this product schedules (max 13 BIS, 10 PA/adv).
* A2DP_FIRST expectation **[I]**: A2DP throughput >= the base-8 run (perfect at bitpool 53). The BIG only gets air the slave
  is not listening/exchanging; every BIS subevent overlapping an ACL window is lost, the first copy included, so NOT SENT
  will be **at least the 26-34 % seen with base 8 and probably clearly more** (the 9-vs-8 protection is gone). Whether HW
  aborts an idle listen window early enough for a later subevent to use the rest is not visible in the blob; later
  copies of the same payload *are* independent items and do use gaps after an ACL exchange (that is the only recovery).
  Levers that reduce the loss in this mode: shorter ACL reservations (allow only 2/3-DH3 -> 4 instead of 6 slots per
  exchange), fewer/shorter BIS subevents does not help (each still overlaps), keeping the boost strictly to "audio started".
* AURACAST_FIRST expectation **[I]**: first copies (15) can never lose to the ACL (<= 14) or any adv (<= 10) -> NOT SENT
  ~0 if escalation was the cause (test this first: if loss stays > 0, the HW does not preempt an in-progress ACL window and
  the ACL base must go below the repetitions again). A2DP as in the base-8 run (ACL still beats repetitions 7).
  Cost: repetitions mostly lose to the always-listening slave, so speakers effectively see ~1 copy per payload while the
  phone link is active (on-air reliability, not counted by NOT SENT).
* Idle profile: ACL back to stock so an idle/paused link (missed polls) cannot climb over the BIG; repetitions 12 sit above
  PA (10) and escalated adv (<= 10).

Location/verification at run time: BR/EDR table = unique match of bytes 1..35 of the stock 36-byte table in
`[_data_start,_data_end)` with byte 0 in 1..15, then all 18 `{dft,inc}` pairs must equal `r_olc_sched_pti_dft_get/inc_get(i)`;
write is read back through the getter (restored on mismatch). ISO block = `r_sym_ble_kBC0rRwtjATsFkriXVDL()` == the global,
byte-accessible, fingerprint `w0=1250, w8=20000, w12=1000, w52=500000, b32=0x1F, b37..39=9/12/12`. Every call re-verifies
and re-applies (repairs an HCI_Reset re-default).

Integration: `bt_sched_tune_init()` after the last HCI_Reset (Bluedroid enabled + ISO host up), replacing
`bt_raise_acl_priority()`; `bt_sched_tune_streaming(true)` on `ESP_A2D_AUDIO_STATE_STARTED`, `(false)` on
SUSPEND/STOPPED and on A2DP/ACL disconnect. Mode: `-DBT_SCHED_MODE=BT_SCHED_MODE_AURACAST_FIRST` or
`bt_sched_tune_set_mode()` at run time.

## 1. Architecture of the scheduler

* **[C]** One common "multiple linked-list" scheduler: `common/19.o` = `btdm_common/components/schedule/src/multiple_link/btdm_sched.c`.
  HW has 16 linked-list heads at `0x2010B000 + 16*idx` (cleared in `r_sym_bt_XPuqTHliEO5V9xpR7aJR`, loop `0x2010b000..0x2010b100`);
  `r_sym_bt_iFvwGI2tL5M1WM3fIkHq` reads the currently running list index from `0x2010107C[23:20]`.
* **[C]** BR/EDR owns lists **13, 14, 15** (its own SW env `g_btdm_common_sched_env`, 52 B, 3 x 12-B lists;
  `r_btdm_common_sched_entry_idx_to_list` asserts idx<=15 and indexes `(idx-13)*12`; `r_btdm_common_sched_bredr_onSchedHwListDone`
  handles done-bits `0x2000/0x4000/0x8000` for lists 13/14/15). `r_olc_sched_get_entry_idx`: ACL -> 14/15 (by link id), others -> 13.
* **[C]** BLE inserts into a *different* SW list: `ble/110.o` (`sched_txn.c`) insert wrappers (`r_sym_ble_jVs2DPPaJL7CDx8SeFuo`,
  `..._NveAh7..`, `..._k8nU4..`, `..._iHRqSCIg..`) use `(*r_sym_ble_PLEXNZlitWcpOaSCj7Ht)+16` (or a parent item's sub-list).
* **[C]** The SW overlap check `r_sym_bt_FovcDCPDYkKMaCv7y4Wb(list, item, start, conflict_cb, cb_arg)` only walks **one** list.
  So **SW never compares a BR/EDR item against an LE item**. BR/EDR code has no reference to any BLE/ISO symbol (checked all
  undefined symbols of libbredr_app.a).
* **[I]** Therefore BR/EDR-vs-LE conflicts are resolved by the **HW list arbiter**, using the priority stored in the shared
  (EM, `0x2F00xxxx`) sched item. This is the only place both stacks meet; there is no SW "reservation window" between them.

### Sched item layout (shared by BR/EDR and BLE) **[C]**
| off | meaning (evidence) |
|---|---|
| +0  | HW next-pointer: bits[19:0]*4 OR'ed with base `*(0x20101074)` (walk loops in Fovc, `n5Nbi..`) |
| +24 | **bits[3:0] = priority** (compared `andi 15` in `r_sym_bt_jzw6WTDKUlY3F90cFtr2`, `r_olc_sched_conflict_handler`, Fovc); bits[7:4] second nibble (see below) |
| +48 | bit31 set = "child of parent item"; else checked against now+lead (returns -2 "too late") |
| +68/+72 | start / end time (ticks) |
| +77 | item type (BLE: 2 = adv?, 8..10 ISO; `cC3j..`, `SlRy..`) |
| +79 | flags (bit3 = "pending/soft"; bits1-2 set when removed by a winner) |
| +96 (BR/EDR only) | sched attr: [1:0] mode (0 fixed, 1/2 ASAP), [8:5] resched attempts, [12:9] prio increment, [16:13] list idx; +100 asap limit, +104 duration |

### SW conflict rule inside one list **[C]**
`r_sym_bt_jzw6WTDKUlY3F90cFtr2(new, existing)`:
```
if (new->f79 & 8 || ex->f79 & 8) return 2;          // defer to caller policy
return ((ex->prio & 15) < (new->prio & 15)) ? 0 : -1; // strictly higher wins, tie -> existing keeps slot
```
BR/EDR (`r_olc_sched_insert` -> `r_btdm_common_sched_check_overlap(.., r_olc_sched_conflict_handler, ..)`): on loss it
re-tries ASAP after the blocking item, decrements attempts (attr[8:5]) and does `prio = min(14, prio + inc)`
(insert @0x118-0x14c). Overlapped losers are unlinked into a "removed" list (`r_sym_bt_E8c5Eimm0z6kYe9v4wHr`, sets f79|=6).

## 2. Where the priorities come from

### BR/EDR **[C]**
Internal table `.dram1.38` in `olc_sched.c.o` (36 B, local, no symbol), read by `r_olc_sched_pti_dft_get(i)` = `tbl[2i]`,
`r_olc_sched_pti_inc_get(i)` = `tbl[2i+1]`:
```
02 01 | 09 02 | 0a 01 | 04 01 | 0c 02 | 08 01 | 02 01 | 04 01 | 04 01 | 04 01 | 02 01 | 0a 01 | 03 01 | 0b 01 | 06 01 | 06 01 | 04 01 | 04 01
idx: 0 ACL  1 RSW   2 sniff 3 ?    4 ACL-hs 5 SCO  6 APB  7 ?    8 CPBrx 9 CPBtx 10 inq 11 iscan 12 page 13 pscan 14 syncscan 15 strain 16/17 DTM
```
(index->activity mapped from every call site, e.g. `olc_pscan.c` uses 13, `olc_acl_sched.c` 0/2/4/14.)

ACL peripheral priority, `r_olc_acl_sched_peripheral_asap.constprop.0` (called per anchor from `r_olc_acl_sched_recycle_sch`, a3=1):
```
cap  = 14   (or dft[0]=2 if acl->f54 & 2)
prio = dft[0](=2) + min(cap, (tx_traffic + rx_traffic) >> 2)          @0x2a8-0x2b4
     + min(cap, (now - last_sync + 1250us) / poll_interval_ticks)     @0x2be-0x2f6  (no SCO case)
prio = min(prio, cap);  item->+24 low nibble = prio;  log "P:%d new pti:%d" if != 2
attr_set(mode ASAP, attempts 4, inc = inc[0]=1, ..., duration = ((tx_pending?1:0)+tx_slot+rx_slot)*625-189 us)   @0x276-0x284, 0xa6-0xca
```
Poll interval = `acl_sched->+24` (default 40 slots, `r_olc_acl_sched_set_dft`). So a streaming ACL normally sits at
**2..4** and only climbs ~+1 per missed poll interval (25 ms). The ACL item writes `sb prio,24` (construct @0x242) so the
**high nibble of +24 is always 0** for BR/EDR.

The 5-bit pairs at item+36 come from `g_bcp_param_p` (`bt_coex_pti.c`, RAM copy of `sc_bt_coex_pti_dft_param`, 12 pointers;
entry 4 = ACL table `04 09 0d 0d 09 0d 04 09 04 09 00`). **[I]** these are the external (Wi-Fi) coex PTIs, not the BT-internal priority.

### LE **[C]**
* Setter `r_sym_ble_J21QYaRyf1RwwT30YZpY(item, prio, f)`: `+24 = (+24 & ~0xFF) | prio | prio<<4` (both nibbles).
* Adv/conn/sync items copy `obj->+96` the same way: adv 1/5/10 (`ble/50.o XXPQ..`, `GZEt..`), conn base 8 escalating to 14/15
  (`ble/51.o pAfV..`), sync 6/12 (`ble/68.o`), scan low nibble 1 or 4 with high nibble cleared (`ble/65.o ZZKI..` @0xfc-0x112).
* **BIS** (`ble/56.o r_sym_ble_FtKLgw2CvtskaTYKEjHw` @0xe8-0x116): `k = subevent_field(bits[5:1]) / BN(big+28)`;
  `prio = k==0 ? cfg[33] : (k < IRC(big+31) ? cfg[34] : cfg[35])` where
  `cfg = r_sym_ble_u66q9VTswFBa3W6uzpwY` (global pointer, `ble/35.o .bss`, 64-B block, alloc'd in `..PzGFr48..`).
  Defaults (`r_sym_ble_19UgKHutGrGyTis8atJz`): **cfg[33]=9, cfg[34]=7, cfg[35]=8** (word @+32 = 0x0807091F; @+36 = 0x0C0C090C,
  [I] CIS prios 12/9/12/12). Defaults are re-applied by `r_sym_ble_jjkDTPiFAFneMj3PWNSj` from the ISO init/reset path
  (`ble/26.o r_sym_ble_fFDl7Sbo4WkvatgaBjUv`, called from `ble_ll.c`), i.e. on HCI_Reset.
  **[I]** k==0 = first copy of each payload group (prio 9), 0<k<IRC = repetitions (7), k>=IRC = PTO pre-transmissions (8).

  (Second pass: the k mapping is now **[C]**, see 0.1; the ACL formula above omitted nothing but its effect with a raised
  base was misjudged, see 0.3.)

### Consequence **[I]** (first pass, stock priorities only — see section 0 for raised ACL base)
BIS subevents (7..9) strictly outrank a BR/EDR ACL anchor (2..4 typical), extended adv (1/5) and scan (1/4). Periodic adv (10)
outranks everything. When a BIS subevent overlaps the ACL reservation the ACL anchor is dropped (phone's 2/3-DH5 is not
received/acked, it retransmits) -> throughput loss proportional to the BIG airtime + one ACL reservation. The +24 high
nibble is 0 for BR/EDR and = prio for BIS; if the HW uses it as "in-progress" priority, an ongoing 5-slot BR/EDR exchange is
preemptible by any LE event. Not provable without HW docs.

## 3. BIS Sub_Interval / BIS_Spacing (Create BIG Test) **[C]**
`HCI_LE_Create_BIG_Test` handler `ble/27.o r_sym_ble_CRfNWjTYdx6iLY5SVfz8` copies NSE->big+29, BN->+28, IRC->+31, PTO->+30,
Packing->+33, then `..m99lDKtpTlSRmYs1DLun` (airtime) and `..3dLKOznqgkSdjoFinoe1` (spacing):
```
air  = ble/34.o hiZRIZ..(phy,len,enc): 1M (len+10)*8, 2M (len+11)*4 (+4 B MIC if enc), Coded fixed formula
sub  = air + 150 (T_MSS)                       // big+52 = air
sequential: Sub_Interval = sub, BIS_Spacing = NSE*sub
interleaved: BIS_Spacing = sub, Sub_Interval = NumBIS*sub
BIG_Sync_Delay = (NumBIS-1)*BIS_Spacing + (NSE-1)*Sub_Interval + air
```
For 100-B PDU, 2M: air 444 us, sub 594 us. NSE 8 x 2 BIS = **9.50 ms** busy per 20 ms; NSE 4 = **4.75 ms**.
The event is already back-to-back (minimum T_MSS); there is no host/VSC knob for Sub_Interval/BIS_Spacing in the Test path
(only the BIGInfo-receive path `..KuAvj1..` takes them from the peer). Only NSE, Num_BIS, Max_PDU, PHY change the block length.

## 4. BR/EDR packet length vs LE ISO **[C]**
No code in libbredr_app.a looks at LE/ISO state. The ACL reservation is `((tx_pending?1:0) + tx_slot + rx_slot)*625-189 us`; `rx_slot =
max(acl->f68, r_olc_util_acl_slot_num(allowed_ptype, edr))` when rx traffic is present (`r_olc_acl_sched_get_rx_slot`), i.e.
5 slots when 2/3-DH5 is allowed. Nothing limits max slots when ISO is active.

## 5. Things that are NOT levers **[C]**
* `r_btdm_common_sched_bredr_alter_sched_item_set` = single `ret`; `r_btdm_common_sched_bredr_onSchedCheckRemaining` returns 0.
* `r_olc_acl_sched_poll_interval_set(obj,u16)` = `sh a1,24(a0)`, `r_olc_acl_sched_mode_set(obj,u8)` = `sb a1,33(a0)`;
  they need the internal ACL-sched object pointer (not exposed).
* BR/EDR VSCs present (tables `s_vs_*_hci_tbl`): FD81 write_dm1_enable, FD83 clk_upd, FD87 set_afh, FD88 set_evt_mask,
  FD89 afh_rpt_mode, FD8A mask_rmt_afh_ch_class, FD8C-FD8F DTM, FD90 pca, FD91 discard_ping, FD9A-FD9C lmp_dbg, FCE0 bqb
  sam switch. None touches scheduling/priority.
* `0xFC82` (COEX_STATUS, common VSC in `common/11.o y6oMuwuf..` -> `common/4.o 31TScaFL..`) only calls
  `wr_btdm_coex_schm_status_bit_set/clear` = Wi-Fi coex scheme (no-op without `CONFIG_SW_COEXIST_ENABLE`).
* The ISO-param setters in `ble/35.o` (`..V0JF9..` 28 B block, `..1x9pQ..` 64 B block, ...) have **no caller** in any blob
  (not wired to HCI in this build) but are global and callable.
* No field in `esp_bt.h` controller config reaches these priorities.
* Common timing: reserved 40 us, lead 46 us (`r_sym_bt_HGbPEa9oUCsYybIAWjE0`), not priority related.

## 6. Levers to try (ordered) — first pass; superseded by section 0.5 / `bt_sched_tune.c`
(L2 with base 8 alone loses whole SDUs: ACL escalation 8 -> 9..14 overtakes the first copy at 9.)

**L1. Lower BIS retransmission priority (cheap, runtime, takes effect next BIG event).** Do it after the host stack
is enabled (after HCI_Reset), before or after Create BIG Test:
```c
extern uint8_t *r_sym_ble_kBC0rRwtjATsFkriXVDL(void);      // returns r_sym_ble_u66q9VTswFBa3W6uzpwY
uint8_t *iso = r_sym_ble_kBC0rRwtjATsFkriXVDL();
if (iso) {            // defaults 9/7/8
    iso[33] = 9;      // first copy of each payload (k==0): keep protected
    iso[34] = 1;      // repetitions (0<k<IRC): below ACL base prio 2 -> ACL wins overlaps
    iso[35] = 1;      // PTO groups
}
```
Variants: all three = 3 (ACL wins only once escalated >3); iso[33]=1 too (ACL always wins, LE gets leftovers).
Tie-break is "existing wins" in SW; HW tie behaviour unknown, so keep values strictly below/above 2.

**L2. Raise BR/EDR ACL base priority** (no symbol; find the 36-B table in RAM and verify):
```c
extern uint8_t r_olc_sched_pti_dft_get(uint8_t idx);
static const uint8_t sig[36] = {2,1,9,2,10,1,4,1,12,2,8,1,2,1,4,1,4,1,4,1,2,1,10,1,3,1,11,1,6,1,6,1,4,1,4,1};
// scan internal DRAM (or take the address of ".dram1.38 ... libbredr_app.a(olc_sched.c.o)" from build/*.map),
// require exactly one match, then:
tbl[0] = 8;  // ACL normal: base 2 -> 8 (> BIS repetitions 7, < BIS first copy 9); cap stays 14
assert(r_olc_sched_pti_dft_get(0) == 8);
```
(byte offsets: tbl[2*i]=dft, tbl[2*i+1]=inc; tbl[8] = ACL first-anchor 12, tbl[4] = sniff 10, already high.) Combine with L1 or use alone.

**L3. Shrink the ACL reservation**: allow only 2-DH3/3-DH3 (HCI_Change_Connection_Packet_Type on the A2DP link, Bluedroid
`BTM_SetPacketTypes`) so tx+rx reservation drops from 6 to 4 slots (3.56 -> 2.31 ms) and fits the ISO gap. [I] whether the
phone honours it (LMP_max_slot) must be tested; bitpool 53 still fits 3-DH3 at ~40 pkt/s.

**L4. Shorten the BIG block** (already the only spacing knob): NSE 4/IRC 2 = 4.75 ms vs 9.5 ms; NSE 2/IRC 1 = 2.38 ms.
Sub_Interval/BIS_Spacing are fixed at airtime+150 us; Packing choice does not change the total length.

**L5. (Wi-Fi coex only)** `g_bcp_param_p[4]` (`extern const uint8_t **g_bcp_param_p;`, RAM array of 12 pointers) can be
re-pointed to a custom 11-byte ACL PTI table; [I] affects Wi-Fi/BT coex arbitration, not BR/EDR vs LE.

Verify any change with the controller's own logs: `P:%d new pti:%d`, `P:%d sched failed, status:%d`, `acl skip`
(olc_acl_sched.c, BR/EDR log level DEBUG/WARN) and with A2DP throughput / ISO missed-SDU counters.
