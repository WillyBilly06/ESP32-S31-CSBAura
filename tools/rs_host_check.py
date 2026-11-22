"""Build and run tools/rs_host_test.c (the firmware's resampler + ring code from
s31-jbl-speaker/main/a2dp_dsp.h, fed with random packet sizes), then measure
the spur level of its output.

    python tools/rs_host_check.py [--cc clang]

Spur = strongest spectral line more than 150 Hz away from every input tone,
relative to the strongest tone (Hann window, 65536 points, 48 kHz).
"""
import argparse, os, subprocess, sys, tempfile
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MAIN = os.path.join(HERE, "..", "s31-jbl-speaker", "main")
FS = 48000


def spur(x, tones, fmax):
    y = x[4000:4000 + 65536].astype(np.float64)
    Y = np.abs(np.fft.rfft(y * np.hanning(len(y))))
    f = np.fft.rfftfreq(len(y), 1 / FS)
    db = 20 * np.log10(Y / Y.max() + 1e-15)
    mask = f <= fmax
    for t in tones:
        mask &= np.abs(f - t) > 150
    i = np.argmax(np.where(mask, db, -400))
    return db[i], f[i]


def load(path):
    a = np.fromfile(path, dtype="<i2").reshape(-1, 2)
    return a[:, 0], a[:, 1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cc", default="clang")
    args = ap.parse_args()
    out = tempfile.mkdtemp(prefix="rs_host_")
    exe = os.path.join(out, "rs_host_test" + (".exe" if os.name == "nt" else ""))
    subprocess.check_call([args.cc, "-O2", "-D_USE_MATH_DEFINES", "-I", MAIN,
                           os.path.join(HERE, "rs_host_test.c"), "-o", exe] +
                          ([] if os.name == "nt" else ["-lm"]))
    rc = subprocess.call([exe, out])

    fails = rc != 0
    print()
    for name, label in (("spur_new.raw", "new (a2dp_dsp.h, random packets)"),
                        ("spur_old.raw", "old (per-SBC-frame float code)")):
        l, r = load(os.path.join(out, name))
        dl, fl = spur(l, (1000, 10000), 24000)
        dr, fr = spur(r, (5000, 15000), 24000)
        ok = max(dl, dr) <= -80
        if name == "spur_new.raw":
            fails |= not ok
        print("[%s] %-34s L 1k+10k: worst spur %6.1f dB at %5.0f Hz | R 5k+15k: %6.1f dB at %5.0f Hz"
              % ("PASS" if ok else "FAIL", label, dl, fl, dr, fr))

    print("\nsingle-tone sweep (0.8 FS, new code, random packets): worst spur")
    print("   tone   in 0-20 kHz           in 0-24 kHz")
    worst_inband = -400
    for f in range(1000, 20000, 1000):
        l, _ = load(os.path.join(out, "sweep_%05d.raw" % f))
        d20, f20 = spur(l, (f,), 20000)
        d24, f24 = spur(l, (f,), 24000)
        if f <= 15000:
            worst_inband = max(worst_inband, d20)
        print("  %5d  %6.1f dB @ %5.0f Hz   %6.1f dB @ %5.0f Hz" % (f, d20, f20, d24, f24))
    ok = worst_inband <= -80
    fails |= not ok
    print("[%s] tones 1-15 kHz: worst audible-band (<= 20 kHz) spur %.1f dB"
          % ("PASS" if ok else "FAIL", worst_inband))
    print("\n" + ("SOME CHECKS FAILED" if fails else "all checks passed"))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
