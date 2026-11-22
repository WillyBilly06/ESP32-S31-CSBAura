# ESP-IDF patches

`s31-jbl-speaker` builds against a local ESP-IDF v6.1 tree with two small Bluedroid patches.
Both are guarded by compile definitions that only `s31-jbl-speaker/CMakeLists.txt` sets, so other
projects built from the same tree are unaffected.

| Patch | File | Enabled by | Why |
|---|---|---|---|
| `0001-gatts-start-service-on-br-edr.patch` | `btc/profile/std/gatt/btc_gatts.c` | `CLASSIC_BT_GATT_INCLUDED=1` | JBL One talks to an Xtreme 5 over GATT on **BR/EDR**; services must be started on `BTA_GATT_TRANSPORT_LE_BR_EDR`. |
| `0002-a2dp-sink-prefer-master.patch` | `btc/profile/std/a2dp/btc_av.c` | `BTC_AV_SINK_PREFER_MASTER=1` | Adds `BTA_AV_FEAT_MASTER` so the S31 asks to be BR/EDR master on the A2DP link (coexistence with the LE Audio BIG). |

Apply from the IDF root:

```bash
git apply /path/to/idf-patches/0001-gatts-start-service-on-br-edr.patch
```

`0002` is shown as a hand-written hunk (the surrounding file differs between IDF forks); paste the
`#if ... #endif` block into the "A2DP Enable without AVRC" branch of `btc_av_execute_service`.
