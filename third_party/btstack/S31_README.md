# BTstack (patched) for the S31

This folder is **BTstack by BlueKitchen GmbH** (https://github.com/bluekitchen/btstack), only
the parts the S31 firmware builds from (`src`, `platform/embedded`, `platform/freertos`, and
`3rd-party/bluedroid`, `md5`, `micro-ecc`, `yxml`).

- Commit: `btstack-patches/BTSTACK_COMMIT` (`9edb5d82ea`).
- Already applied: `btstack-patches/0001` ... `0004` (see those files for what each one changes).

BTstack keeps its own license, in [`LICENSE`](LICENSE). It allows redistribution and use for
**personal, non-commercial purposes only**. For anything else, get a commercial license from
BlueKitchen (contact@bluekitchen-gmbh.com). The copyright notices stay as they are.

To make this folder again from scratch:

```bash
git clone https://github.com/bluekitchen/btstack third_party/btstack
cd third_party/btstack
git checkout $(cat ../../btstack-patches/BTSTACK_COMMIT)
git apply ../../btstack-patches/*.patch
```
