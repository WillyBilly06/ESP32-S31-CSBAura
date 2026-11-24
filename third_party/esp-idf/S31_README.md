# ESP-IDF (patched) for the S31

This folder is **ESP-IDF by Espressif Systems** (https://github.com/espressif/esp-idf),
exactly the tree the S31 firmware is built with:

- Commit: `idf-patches/IDF_COMMIT` (`c43d665a96`, `release/v6.1`, ESP32-S31 support), with its
  submodules.
- Already applied: `idf-patches/0010-controller-only-iso-rx.patch` and
  `idf-patches/0011-btdm-task-stack-dma.patch`.
- Left out: `docs/`, `examples/`, CI files and the git history. `version.txt` and
  `S31_IDF.json` were added; they name the version, commit and patch.

Nothing has to be cloned or patched. Only ESP-IDF's tools (compiler, CMake, Ninja, Python
environment, about 1 GB) are installed outside the repository, into `%USERPROFILE%\.espressif`.
The S31 Flasher does that by itself on the first Build + Flash. By hand:

```bash
cd third_party/esp-idf
./install.sh esp32s31 && . ./export.sh      # Windows: .\install.ps1 esp32s31; . .\export.ps1
```

ESP-IDF is licensed under Apache-2.0 ([`LICENSE`](LICENSE)). Some components carry
their own licenses, listed in their folders.
