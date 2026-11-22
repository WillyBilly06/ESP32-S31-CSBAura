# ESP-IDF patches

`s31-jbl-speaker` builds against a local ESP-IDF v6.1 tree with small Bluedroid patches.
All are guarded by compile definitions that only `s31-jbl-speaker/CMakeLists.txt` sets, so other
projects built from the same tree are unaffected.

| Patch | File | Enabled by | Why |
|---|---|---|---|
| `0001-gatts-start-service-on-br-edr.patch` | `btc/profile/std/gatt/btc_gatts.c` | `CLASSIC_BT_GATT_INCLUDED=1` | JBL One talks to an Xtreme 5 over GATT on **BR/EDR**; services must be started on `BTA_GATT_TRANSPORT_LE_BR_EDR`. |
| `0002-a2dp-sink-prefer-master.patch` | `btc/profile/std/a2dp/btc_av.c` | `BTC_AV_SINK_PREFER_MASTER=1` (**not set**) | Adds `BTA_AV_FEAT_MASTER`. Disabled: phones refuse the role switch, LMP requests collide and Bluedroid drops the link to retry. |
| `0003-a2dp-sink-bitpool-stats.patch` | `btc/profile/std/a2dp/btc_a2dp_sink.c` | `BTC_A2DP_SINK_BITPOOL_STATS=1` | Reads the SBC bitpool and media sequence gaps from every packet the phone sends, for the 2 s stats log. |

Apply from the IDF root:

```bash
git apply /path/to/idf-patches/0001-gatts-start-service-on-br-edr.patch
```

`0002` is shown as a hand-written hunk (the surrounding file differs between IDF forks); paste the
`#if ... #endif` block into the "A2DP Enable without AVRC" branch of `btc_av_execute_service`.
