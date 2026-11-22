# ESP32-S31 controller: BR/EDR vs LE scheduling arbitration (RE notes)

Source blobs: `components/bt/controller/lib_esp32s31/esp32s31-bt-lib/{libbtdm_common.a,libbredr_app.a,libble_app.a}`
(IDF tree `esp-idf-v6.1-codecs`). Method: `ar x` + `objdump -dr` + rodata dumps (scratch:
`C:\Users\ngocv\AppData\Local\Temp\claude\9db0574533c6a073\72a8b680-3494-45db-8bf5-04bd924bcbdb\scratchpad\re_ctrl\`).
Object names below: `common/N.o`, `ble/N.o` = numbered archive members, `bredr/*.c.o` = named members.
Tags: **[C]** = confirmed in disassembly/data, **[I]** = inference.

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

### Consequence **[I]**
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

## 6. Levers to try (ordered)

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
