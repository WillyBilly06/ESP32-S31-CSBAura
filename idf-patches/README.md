# ESP-IDF patches

Against ESP-IDF `release/v6.1` (ESP32-S31 support), applied from the IDF root with `git apply`.

| Patch | Needed | Why |
|---|---|---|
| `0010-controller-only-iso-rx.patch` | **yes** | In a controller-only build (BTstack is the host) IDF dropped, and leaked, every received ISO packet: party LISTEN and LE Audio got no audio. |
| `0009-btdm-lpclk-force-main-xtal.patch` | older trees | Keeps the controller's low-power timer on the main crystal (the 136 kHz RC is not accurate enough for ISO/BIG timing). The project sets `BTDM_LPCLK_FORCE_MAIN_XTAL=1` for it; it does not apply cleanly to the current `release/v6.1`, where it may no longer be needed. |

## Already applied in the repository

`third_party/esp-idf` is ESP-IDF at `IDF_COMMIT` with `0010` applied (see its
`S31_README.md`), so nothing has to be patched to build. These files are for building with
an ESP-IDF somewhere else.
