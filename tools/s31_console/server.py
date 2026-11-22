#!/usr/bin/env python3
"""
S31 Console - local flash + serial-monitor server for the ESP32-S31 board.

  python server.py            # real hardware (COM5 @ 921600)
  python server.py --mock     # simulator: fake serial + fake build/flash

Serves the web UI on http://127.0.0.1:8765, a JSON API under /api/* and a
Server-Sent-Events stream on /events.  Owns the serial port: a flash or reset
request while logging transparently pauses the reader, closes the port, runs
esptool and then reopens the port and resumes logging in the same session.

stdlib + pyserial only (pyserial is not needed in --mock mode).
"""
from __future__ import annotations

import argparse
import json
import os
import queue
import random
import re
import subprocess
import sys
import threading
import time
import traceback
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

VERSION = "1.0.0"
HERE = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(HERE, "static")
DEFAULT_PROJECT = r"E:\ESP32-S31-CSBAura\s31-jbl-speaker"
DEFAULT_IDF_PATH = r"D:\ESP32-S31\esp-idf-v6.1-codecs"
STATES = ("IDLE", "BUILDING", "FLASHING", "REBOOTING", "LOGGING", "ERROR")

ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-~]")
CTRL_RE = re.compile(r"[\x00-\x08\x0b-\x1f\x7f]")
LEVEL_RE = re.compile(r"^([EWIDV]) \((\d+)\) ([^:]{1,48}):")
CRASH_RE = re.compile(
    r"Guru Meditation Error|assert failed|task_wdt: Task watchdog got triggered|"
    r"abort\(\) was called|Stack smashing protect|CORRUPT HEAP|\*\*\* PANIC")
REBOOT_RE = re.compile(r"^Rebooting\.\.\.|rst:0x([0-9a-fA-F]+) \(([^)]*)\)")


def fmt_dur(sec: float) -> str:
    return f"{sec:.1f} s"


# =============================================================================
# SSE hub
# =============================================================================
class _ClientQueue(queue.Queue):
    overflow = False


class Hub:
    """Fan-out of server-sent events to all connected browsers/clients."""

    def __init__(self) -> None:
        self._clients: set[_ClientQueue] = set()
        self._lock = threading.Lock()

    def subscribe(self) -> _ClientQueue:
        q = _ClientQueue(maxsize=4000)
        with self._lock:
            self._clients.add(q)
        return q

    def unsubscribe(self, q: _ClientQueue) -> None:
        with self._lock:
            self._clients.discard(q)

    @property
    def count(self) -> int:
        with self._lock:
            return len(self._clients)

    @staticmethod
    def encode(event: str, data) -> bytes:
        return f"event: {event}\ndata: {json.dumps(data, separators=(',', ':'))}\n\n".encode("utf-8")

    def publish(self, event: str, data) -> None:
        msg = self.encode(event, data)
        with self._lock:
            clients = list(self._clients)
        for q in clients:
            try:
                q.put_nowait(msg)
            except queue.Full:
                q.overflow = True  # slow client: handler drops it, browser reconnects + resyncs


# =============================================================================
# Stats parser
# =============================================================================
def _i(m, g=1):
    return int(m.group(g)) if m else None


class StatsParser:
    """Tolerant parser for the firmware's periodic stats / event lines."""

    SERIES = ("not_sent", "underruns", "buf_ms", "enc_avg", "enc_max", "sdu_avg",
              "sdu_max", "bitpool", "drops", "lost_pkts", "on_air")
    HIST = 1500  # ~50 min at one sample / 2 s
    GROUP_RE = re.compile(r"^[EWIDV] \(\d+\) ([^:]+): (.+?) 2s: (.*)$")
    PAIR_RE = re.compile(r"^([A-Za-z][A-Za-z_\-/+ ]*?)\s+(-?\d+(?:\.\d+)?)(?:\s*/\s*(\d+))?")

    def __init__(self) -> None:
        self.latest: dict = {}
        self.updated: dict = {}
        self.groups: dict = {}
        self.history = {k: deque(maxlen=self.HIST) for k in self.SERIES}
        self.events = deque(maxlen=200)  # notable BT events (connect, role, ...)

    def reset(self) -> None:
        self.__init__()

    # -- generic "<key> <number>" pairs for any "<GROUP> 2s:" line ----------
    def _generic(self, text: str) -> None:
        g = self.GROUP_RE.match(text)
        if not g:
            return
        group = g.group(2).strip()
        pairs = {}
        for seg in re.split(r"[,|:]", g.group(3)):
            seg = seg.strip()
            m = self.PAIR_RE.match(seg)
            if m:
                key = re.sub(r"[^a-z0-9]+", "_", m.group(1).lower()).strip("_")
                val = float(m.group(2)) if "." in m.group(2) else int(m.group(2))
                pairs[key] = val
                if m.group(3):
                    pairs[key + "_of"] = int(m.group(3))
        if pairs:
            self.groups.setdefault(group, {}).update(pairs)

    def feed(self, text: str, now: float):
        """Returns (changed_fields: dict, alerts: list[(kind, text)])."""
        ch: dict = {}
        alerts: list = []
        s = re.search

        def put(k, v):
            if v is not None:
                ch[k] = v

        if " 2s:" in text:
            self._generic(text)
            if "TX 2s:" in text:
                put("frames", _i(s(r"(\d+) frames", text)))
                put("expect", _i(s(r"expect (\d+)", text)))
                put("send_err", _i(s(r"send_err (\d+)", text)))
                m = s(r"\|\s*([A-Z][A-Z0-9 _]+?)\s*,", text)
                put("source", m.group(1) if m else None)
                put("a2dp_pkts", _i(s(r"a2dp pkts (\d+)", text)))
                put("buf_ms", _i(s(r"buf (\d+) ms", text)))
                put("underruns", _i(s(r"underruns (\d+)", text)))
                put("drops", _i(s(r"(?<!backlog )\bdrops (\d+)", text)))
                put("latency_skips", _i(s(r"latency skips (\d+)", text)))
                put("pcm_glitches", _i(s(r"PCM glitches (\d+)", text)))
                put("clipped", _i(s(r"clipped (\d+)", text)))
            elif "ISO timing 2s:" in text:
                m = s(r"SDU wait[^|]*?avg (\d+)\s*us[^|]*?max (\d+)", text)
                if m:
                    put("sdu_avg", int(m.group(1)))
                    put("sdu_max", int(m.group(2)))
                m = s(r"tx-ref avg (\d+)\s*us[^|]*?max (\d+)", text)
                if m:
                    put("txref_avg", int(m.group(1)))
                    put("txref_max", int(m.group(2)))
            elif "ISO 2s:" in text:
                put("on_air", _i(s(r"on-air (\d+)", text)))
                put("not_sent", _i(s(r"NOT SENT (\d+)", text)))
                put("drift", _i(s(r"drift (-?\d+)", text)))
                put("ticks_late", _i(s(r"ticks late (\d+)", text)))
                put("worst_gap_us", _i(s(r"worst gap (\d+)", text)))
                m = s(r"in flight (\d+)/(\d+)", text)
                if m:
                    put("in_flight", int(m.group(1)))
                    put("in_flight_max", int(m.group(2)))
                put("backlog_drops", _i(s(r"backlog drops (\d+)", text)))
                put("backoff", _i(s(r"backoff (\d+)", text)))
                put("ticks_skipped", _i(s(r"ticks skipped (\d+)", text)))
            elif "SBC 2s:" in text:
                put("bitpool", _i(s(r"bitpool now (\d+)", text)))
                put("bitpool_min", _i(s(r"\(min (\d+)", text)))
                put("bitpool_max", _i(s(r"max (\d+)\)", text)))
                put("media_pkts", _i(s(r"media pkts (\d+)", text)))
                put("lost_pkts", _i(s(r"lost pkts (\d+)", text)))
            elif "LC3 2s:" in text:
                put("enc_avg", _i(s(r"avg (\d+)\s*us", text)))
                put("enc_max", _i(s(r"max (\d+)\s*us", text)))
                put("enc_core_pct", _i(s(r"~(\d+)%", text)))
        else:
            m = s(r"role on this link: (\w+)", text)
            if m:
                put("bredr_role", m.group(1).upper())
                self.events.append((now, "role " + m.group(1).upper()))
            m = s(r"A2DP Connected \[([0-9a-fA-F:]+)\]", text)
            if m:
                put("a2dp_state", "connected")
                put("a2dp_peer", m.group(1))
                put("a2dp_since", now)
                self.events.append((now, "A2DP connected"))
            elif "A2DP Disconnected" in text:
                put("a2dp_state", "disconnected")
                put("a2dp_since", now)
                m = s(r"Disconnected \[[^\]]*\]\s*\(([^)]*)\)", text)
                put("a2dp_disc_reason", m.group(1) if m else "")
                self.events.append((now, "A2DP disconnected"))
            elif "A2DP audio STARTED" in text or "A2DP audio started" in text:
                put("a2dp_state", "streaming")
                put("a2dp_since", now)
            elif s(r"A2DP audio (suspended|stopped|paused)", text):
                put("a2dp_state", "paused")
                put("a2dp_since", now)
            m = s(r"HCI reason (0x[0-9a-fA-F]+)(?:\s*\(([^)]*)\))?", text)
            if m and "disconnect" in text.lower():
                put("last_hci_reason", m.group(1) + (f" ({m.group(2)})" if m.group(2) else ""))
            m = s(r"bt_sched: (.*)$", text)
            if m:
                put("bt_sched", m.group(1).strip())
            m = s(r"free internal (\d+) ?KB", text)
            if m:
                put("free_internal_kb", int(m.group(1)))
                m2 = s(r"PSRAM (\d+) ?KB", text)
                put("free_psram_kb", int(m2.group(1)) if m2 else None)
            m = s(r"PARTY:\s*([A-Za-z_]+)\s*->\s*([A-Za-z_]+)", text) or \
                s(r"[Pp]arty role[:=]?\s+([A-Za-z_]+)\s*()", text)
            if m:
                role = m.group(2) or m.group(1)
                put("party_role", role.upper())
                self.events.append((now, "party " + role.upper()))

        if CRASH_RE.search(text):
            alerts.append(("crash", text))
        else:
            m = REBOOT_RE.search(text)
            if m:
                if m.group(2) is not None:
                    put("last_reset_reason", m.group(2))
                    if "POWERON" in m.group(2).upper():
                        alerts.append(("boot", text))
                    else:
                        alerts.append(("reboot", text))
                else:
                    alerts.append(("reboot", text))

        for k, v in ch.items():
            self.latest[k] = v
            self.updated[k] = now
            if k in self.SERIES:
                self.history[k].append((round(now, 2), v))
        return ch, alerts

    def snapshot(self, history: bool = True) -> dict:
        out = {"latest": dict(self.latest), "updated": dict(self.updated),
               "groups": json.loads(json.dumps(self.groups)),
               "events": [[round(t, 2), e] for t, e in self.events]}
        if history:
            out["history"] = {k: [list(p) for p in v] for k, v in self.history.items()}
        return out


# =============================================================================
# Flash progress tracking
# =============================================================================
class FlashProgress:
    NEW_RE = re.compile(r"Writing at 0x([0-9a-fA-F]+)\s*\[[^\]]*\]\s*([\d.]+)\s*%")
    OLD_RE = re.compile(r"Writing at 0x([0-9a-fA-F]+)\.*\s*\(\s*([\d.]+)\s*%\s*\)")
    WROTE_RE = re.compile(r"Wrote (\d+) bytes.*? at 0x([0-9a-fA-F]+)")
    NINJA_RE = re.compile(r"^\[(\d+)/(\d+)\]")

    def __init__(self, layout):
        # layout: list of (offset, size, name) or [] when unknown
        self.layout = sorted(layout)
        self.frac = {off: 0.0 for off, _, _ in self.layout}
        self.total = sum(sz for _, sz, _ in self.layout) or 0
        self.cur = None
        self.pct = 0.0
        self.writing = False
        self.last_logged = {}

    def region_for(self, addr):
        best = None
        for off, size, name in self.layout:
            if off <= addr:
                best = (off, size, name)
        return best

    def feed(self, line):
        """Returns (pct|None, phase|None, show_line: bool)."""
        m = self.NEW_RE.search(line) or self.OLD_RE.search(line)
        if m:
            addr = int(m.group(1), 16)
            rp = min(100.0, float(m.group(2)))
            self.writing = True
            reg = self.region_for(addr)
            if reg and self.total:
                off, size, name = reg
                if self.cur is not None and self.cur != off:
                    self.frac[self.cur] = 1.0
                self.cur = off
                self.frac[off] = max(self.frac[off], rp / 100.0)
                pct = 100.0 * sum(self.frac[o] * sz for o, sz, _ in self.layout) / self.total
                label = os.path.basename(name).replace(".bin", "")
                phase = f"Writing {label} @0x{off:x} — {rp:.0f}%"
            else:
                pct = rp
                phase = f"Writing @0x{addr:x} — {rp:.0f}%"
            self.pct = max(self.pct, pct)
            # only show every ~10 % step of each region in the log pane
            key = reg[0] if reg else addr >> 16
            step = int(rp // 10)
            show = self.last_logged.get(key) != step
            self.last_logged[key] = step
            return self.pct, phase, show
        m = self.WROTE_RE.search(line)
        if m:
            off = int(m.group(2), 16)
            if off in self.frac:
                self.frac[off] = 1.0
                if self.total:
                    self.pct = max(self.pct, 100.0 * sum(self.frac[o] * sz for o, sz, _ in self.layout) / self.total)
            return self.pct, None, True
        low = line.lower()
        if "connecting" in low and not self.writing:
            return None, "Connecting to chip…", True
        if ("chip is" in low or "chip type" in low or "connected to" in low) and not self.writing:
            return None, line.strip()[:70], True
        if "changing baud" in low:
            return None, "Changing baud rate…", True
        if "erasing" in low or ("flash will be erased" in low):
            return None, "Erasing flash…", True
        if "hash of data verified" in low:
            return None, "Verified ✓", True
        if "hard resetting" in low or "leaving..." in low:
            return 100.0, "Resetting board…", True
        if self.NINJA_RE.match(line.strip()) and not self.writing:
            return None, "Checking build (ninja)…", True
        return None, None, True


# =============================================================================
# Mock device / mock serial port
# =============================================================================
MOCK_LAYOUT = [(0x2000, 24112, "bootloader/bootloader.bin"),
               (0x10000, 1714976, "s31-jbl-speaker.bin"),
               (0x8000, 3072, "partition_table/partition-table.bin")]


class MockDevice:
    """Generates realistic firmware output; keeps running while the port is closed."""

    PEER = "74:f4:41:e0:a5:aa"

    def __init__(self) -> None:
        self.lock = threading.RLock()  # re-entrant: scheduled reboot runs inside pull()
        self.rng = random.Random()
        self.boot_mono = time.monotonic() - self.rng.uniform(300, 900)
        self.sched: list = []
        self.next_tick = time.monotonic() + 0.4
        self.a2dp = "streaming"
        self.pkts = self.rng.randint(1000, 5000)
        self.party = "HOST"
        self.last_heap = 0.0

    def ms(self) -> int:
        return int((time.monotonic() - self.boot_mono) * 1000)

    def ln(self, lvl, tag, msg) -> bytes:
        s = f"{lvl} ({self.ms()}) {tag}: {msg}"
        col = {"E": "31", "W": "33", "I": "32"}.get(lvl)
        return (f"\x1b[0;{col}m{s}\x1b[0m\r\n" if col else s + "\r\n").encode()

    def at(self, delay, fn) -> None:
        self.sched.append((time.monotonic() + delay, fn))

    def reboot(self, why: str) -> None:
        with self.lock:
            self.sched.clear()
            self.boot_mono = time.monotonic()
            self.a2dp = "disconnected"
            self.pkts = 0
            garbage = bytes(self.rng.randrange(0x80, 0x100) for _ in range(48)) + b"\x00\r\n" + \
                bytes(self.rng.randrange(0x80, 0x100) for _ in range(30)) + b"\r\n"
            self.at(0.05, lambda: garbage)
            self.at(0.35, lambda: self.ln("I", "boot", "ESP-IDF v6.1-dev 2nd stage bootloader"))
            self.at(0.40, lambda: self.ln("I", "cpu_start", "Multicore app"))
            self.at(0.55, lambda: self.ln("I", "app_init", "Project name:     s31-jbl-speaker"))
            self.at(0.90, lambda: self.ln("W", "bt_sched", "init [MASTER]: ACL 2->2 | BIS first 9->15 rep 7->12 pto 8->11 ctrl 12->15"))
            self.at(1.10, lambda: self.ln("W", "JBL_BREDR", "audio buffer 75 KB in PSRAM; free internal 152 KB, PSRAM 14562 KB"))
            self.at(1.20, lambda: f"PARTY: BOOT -> {self.party}\r\n".encode())
            self.at(3.0, self._connect)
            self.next_tick = time.monotonic() + 2.0

    def _connect(self) -> bytes:
        self.a2dp = "connected"
        role_slave = self.rng.random() < 0.12
        out = self.ln("W", "JBL_BREDR", f"A2DP Connected [{self.PEER}]")
        out += self.ln("W", "JBL_BREDR", "BR/EDR role on this link: SLAVE (phone schedules the S31 - bad)" if role_slave
                       else "BR/EDR role on this link: MASTER (S31 schedules the phone - good)")
        self.at(self.rng.uniform(1.5, 3.0), self._start)
        return out

    def _start(self) -> bytes:
        self.a2dp = "streaming"
        return self.ln("W", "JBL_BREDR", "A2DP audio STARTED -> re-broadcasting on Auracast")

    def _tick(self) -> bytes:
        r = self.rng
        out = b""
        streaming = self.a2dp == "streaming"
        if self.a2dp in ("streaming", "connected", "paused"):
            self.pkts += r.randint(95, 105) if streaming else 0
            underruns = (r.randint(1, 3) if r.random() < 0.06 else 0) if streaming else 0
            buf = r.randint(105, 150) if streaming else 0
            if underruns:
                buf = r.randint(20, 60)
            out += self.ln("W", "BAP_BSRC",
                           f"TX 2s: 200 frames (expect 200), send_err 0 | {'PHONE MUSIC' if streaming else 'SILENCE'}, "
                           f"a2dp pkts {self.pkts}, buf {buf} ms, underruns {underruns}, drops 0, latency skips 0, "
                           f"PCM glitches 0, clipped 0")
            x = r.random()
            not_sent = 0 if x < 0.72 else (r.randint(1, 4) if x < 0.9 else r.randint(40, 60))
            out += self.ln("W", "BAP_BSRC",
                           f"ISO 2s: on-air 200, NOT SENT {not_sent}, drift 0 | encode ticks late 0, "
                           f"worst gap {r.randint(10000, 10450)} us | in flight {r.randint(3, 4)}/8, totals: "
                           f"backlog drops 0, backoff 0, ticks skipped 0")
            bp = 53 if r.random() < 0.9 else r.choice([45, 49, 51])
            out += self.ln("W", "BAP_BSRC",
                           f"SBC 2s: bitpool now {bp} (min {bp}, max 53), media pkts {r.randint(94, 100) if streaming else 0}, "
                           f"lost pkts {1 if r.random() < 0.03 else 0}")
            avg = r.randint(1700, 1900)
            mx = avg + r.randint(250, 800)
            out += self.ln("W", "BAP_BSRC", f"LC3 2s: encode L+R avg {avg} us, max {mx} us per 10 ms frame (~{avg // 100}% of one core)")
            sa = r.randint(900, 1600)
            out += self.ln("W", "BAP_BSRC",
                           f"ISO timing 2s: SDU wait in controller avg {sa} us max {sa + r.randint(2000, 5000)} us | "
                           f"tx-ref avg {r.randint(80, 140)} us max {r.randint(180, 320)} us (n 199) raw tx 1 rec 2 ref 3 hdl 4")
        # ---- random events ----
        now = time.monotonic()
        if now - self.last_heap > 30:
            self.last_heap = now
            out += self.ln("W", "JBL_BREDR", f"audio buffer 75 KB in PSRAM; free internal {r.randint(138, 156)} KB, PSRAM 14562 KB")
        if r.random() < 0.15:
            out += self.ln("I", "JBL_AVRC", f"absolute volume {r.randint(40, 110)}")
        if streaming and r.random() < 0.025:
            self.a2dp = "paused"
            out += self.ln("W", "JBL_BREDR", "A2DP audio suspended -> broadcasting silence")
        elif self.a2dp == "paused" and r.random() < 0.3:
            out += self._start()
        elif self.a2dp in ("streaming", "paused") and r.random() < 0.02:
            self.a2dp = "disconnected"
            out += self.ln("W", "JBL_BREDR", f"A2DP Disconnected [{self.PEER}] (ABNORMAL - link lost)")
            out += self.ln("W", "JBL_BREDR", "ACL disconnected, HCI reason 0x08 (supervision timeout)")
            self.at(r.uniform(5, 9), self._connect)
        if r.random() < 0.03:
            out += self.ln("E", "BAP_BSRC", f"bt_le_iso_chan_send failed ({r.choice([-12, -105, -11])})")
        if r.random() < 0.015:
            new = r.choice(["HOST", "LISTEN", "JOIN"])
            out += f"PARTY: {self.party} -> {new}\r\n".encode()
            self.party = new
        if r.random() < 0.004:
            out += self.ln("E", "task_wdt", "Task watchdog got triggered. The following tasks/users did not reset the watchdog in time:")
            out += self.ln("E", "task_wdt", " - IDLE1 (CPU 1)")
        elif r.random() < 0.006:
            out += b"Guru Meditation Error: Core  1 panic'ed (LoadProhibited). Exception was unhandled.\r\n"
            out += b"\r\nCore  1 register dump:\r\nPC      : 0x4200a1b4  PS      : 0x00060a30  A0      : 0x8200c2d1\r\n"
            out += b"\r\nBacktrace: 0x4200a1b1:0x3fcb2e80 0x4200c2ce:0x3fcb2ea0 0x40381f8e:0x3fcb2ec0\r\n\r\n"
            out += b"Rebooting...\r\n"
            self.at(0.2, lambda: (self.reboot("panic"), b"")[1])
        return out

    def pull(self) -> bytes:
        with self.lock:
            now = time.monotonic()
            out = bytearray()
            due = [e for e in self.sched if e[0] <= now]
            if due:
                self.sched = [e for e in self.sched if e[0] > now]
                for _, fn in sorted(due, key=lambda e: e[0]):
                    out += fn() or b""
            if now >= self.next_tick:
                if now - self.next_tick > 4:
                    self.next_tick = now
                out += self._tick()
                self.next_tick += 2.0
            return bytes(out)


class MockSerial:
    """Duck-types the bits of serial.Serial the reader uses."""

    def __init__(self, dev: MockDevice, port: str) -> None:
        self.dev = dev
        self.port = port
        self.is_open = False
        self._rts = False
        self.dtr = False
        self.buf = bytearray()

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False

    def cancel_read(self):
        pass

    @property
    def rts(self):
        return self._rts

    @rts.setter
    def rts(self, v):
        if self._rts and not v and self.is_open:
            self.dev.reboot("rts")
        self._rts = bool(v)

    @property
    def in_waiting(self):
        if not self.is_open:
            raise OSError("port closed")
        self.buf += self.dev.pull()
        return len(self.buf)

    def read(self, n=1):
        if not self.is_open:
            raise OSError("port closed")
        if not self.buf:
            self.buf += self.dev.pull()
        if not self.buf:
            time.sleep(0.05)
            return b""
        data = bytes(self.buf[:n])
        del self.buf[:n]
        return data


def mock_tool_main(kind: str, fail: str) -> int:
    """Fake `idf.py build` / `idf.py flash`, run as a subprocess by the server in --mock mode."""
    rng = random.Random()

    def out(s="", d=0.0, end="\n"):
        sys.stdout.write(s + end)
        sys.stdout.flush()
        if d:
            time.sleep(d)

    proj = DEFAULT_PROJECT
    if kind == "build":
        out("Executing action: all (aliases: build)", 0.15)
        out(f"Running ninja in directory {proj}\\build", 0.05)
        out('Executing "ninja all"...', 0.2)
        files = ["bap_bsrc.c", "jbl_bredr.c", "bt_sched.c", "lc3_enc.c", "party.c", "audio_buf.c", "main.c"]
        n = 42
        for i in range(1, n + 1):
            out(f"[{i}/{n}] Building C object esp-idf/main/CMakeFiles/__idf_main.dir/{rng.choice(files)}.obj",
                rng.uniform(0.03, 0.10))
            if fail == "build" and i == 27:
                out(f"{proj}/main/bap_bsrc.c:412:9: error: 'iso_tx_budget' undeclared (first use in this function)")
                out(f"{proj}/main/bap_bsrc.c:415:5: warning: unused variable 'gap' [-Wunused-variable]")
                out("ninja: build stopped: subcommand failed.")
                out(f"ninja failed with exit code 1, output of the command is in the {proj}\\build\\log\\idf_py_stderr_output_1234")
                return 2
        out(f"Generated {proj}/build/s31-jbl-speaker.bin")
        out("s31-jbl-speaker.bin binary size 0x1a2b20 bytes. Smallest app partition is 0x200000 bytes. 0x5d4e0 bytes (18%) free.")
        out("")
        out("Project build complete. To flash, run:")
        out(" idf.py flash")
        return 0

    # ---- flash ----
    out("Executing action: flash", 0.1)
    out(f"Running ninja in directory {proj}\\build", 0.05)
    out('Executing "ninja flash"...', 0.2)
    for i in range(1, 5):
        out(f"[{i}/5] Checking component dependencies / no work to do", 0.08)
    out("esptool.py --chip esp32s31 -p COM5 -b 921600 --before=default_reset --after=hard_reset write_flash "
        "--flash_mode dio --flash_freq 80m --flash_size 16MB 0x2000 bootloader/bootloader.bin "
        "0x10000 s31-jbl-speaker.bin 0x8000 partition_table/partition-table.bin", 0.1)
    out("esptool v5.1.0")
    out("Serial port COM5:")
    if fail == "connect":
        out("A fatal error occurred: Could not open port 'COM5': PermissionError(13, 'Access is denied.', None, 5)", 0.2)
        return 2
    out("Connecting....", 0.9)
    out("Connected to ESP32-S31 on COM5:")
    out("Chip type:          ESP32-S31 (revision v0.1)")
    out("Features:           Wi-Fi 6, BT 5 (LE), Dual Core + LP Core, 320MHz")
    out("Changing baud rate to 921600...", 0.1)
    out("Changed.")
    out("Configuring flash size...")
    out("Flash will be erased from 0x00002000 to 0x00007fff...")
    out("Flash will be erased from 0x00010000 to 0x001bffff...")
    out("Flash will be erased from 0x00008000 to 0x00008fff...", 0.3)
    for idx, (off, size, name) in enumerate(MOCK_LAYOUT):
        comp = int(size * 0.62)
        out(f"Compressed {size} bytes to {comp}...")
        steps = 8 if size < 50000 else 60
        dur = 0.6 if size < 50000 else 6.5
        for k in range(steps + 1):
            pct = 100.0 * k / steps
            addr = off + int(size * k / steps)
            if fail == "flash" and idx == 1 and pct > 40:
                out("")
                out("A fatal error occurred: Packet content transfer stopped (received 8 bytes)")
                out("Failed to write to target RAM (result was 01070000: Operation timed out)")
                return 2
            if idx == 0:   # tty-style (carriage-return overwrite) new esptool progress
                bar = "=" * int(pct / 5) + ">" + " " * (20 - int(pct / 5))
                out(f"Writing at 0x{addr:08x} [{bar}] {pct:5.1f}% {int(comp * k / steps)}/{comp} bytes...",
                    dur / steps, end="\r")
            elif idx == 1:  # line-per-update new style
                bar = "=" * int(pct / 5) + ">" + " " * (20 - int(pct / 5))
                out(f"Writing at 0x{addr:08x} [{bar}] {pct:5.1f}% {int(comp * k / steps)}/{comp} bytes...", dur / steps)
            else:           # old esptool style
                out(f"Writing at 0x{addr:08x}... ({int(pct)} %)", dur / steps)
        out("")
        out(f"Wrote {size} bytes ({comp} compressed) at 0x{off:08x} in {dur:.1f} seconds "
            f"({int(comp * 8 / dur / 1000)} kbit/s).")
        out("Hash of data verified.", 0.1)
    out("")
    out("Leaving...")
    out("Hard resetting via RTS pin...", 0.1)
    out("Done")
    return 0


# =============================================================================
# Console core
# =============================================================================
class Busy(Exception):
    pass


class Console:
    def __init__(self, args) -> None:
        self.args = args
        self.hub = Hub()
        self.lock = threading.RLock()
        self.server_id = f"{int(time.time())}-{random.randint(1000, 9999)}"
        self.server_started = time.time()

        # state machine
        self.state = "IDLE"
        self.op = None             # "build" | "flash" | "build+flash" | "reset"
        self.op_seq = 0
        self.op_started = None
        self.progress = None       # 0..100 or None
        self.phase = ""
        self.last_result = None
        self.error_sticky = False
        self.op_lock = threading.Lock()
        self.proc = None
        self.cancelled = False
        self.mock_fail = None

        # serial / logging
        self.logging = False          # user wants logging
        self.serial_blocked = None    # reason string while port must stay closed (flash)
        self.port_open = False
        self.port_error = None
        self.ser = None
        self.reader = None
        self.reader_stop = None
        self.first_line_evt = threading.Event()
        self.auto_log = not args.no_auto_log
        self.expect_reboot_until = 0.0

        # session
        self.session_path = None
        self.session_fh = None
        self.session_t0 = None
        self.session_started = None
        self.session_lines = 0

        # log ring
        self.lines = deque(maxlen=5000)
        self.seq = 0
        self.pending = []
        self.line_times = deque()

        # stats / crashes
        self.parser = StatsParser()
        self.crash_count = 0
        self.last_crash = None
        self.last_crash_mono = -1e9

        self.mock_dev = MockDevice() if args.mock else None
        threading.Thread(target=self._flusher, name="flusher", daemon=True).start()

    # ------------------------------------------------------------------ status
    def status(self) -> dict:
        with self.lock:
            now = time.monotonic()
            while self.line_times and now - self.line_times[0] > 3.0:
                self.line_times.popleft()
            return {
                "server_id": self.server_id, "version": VERSION, "mock": bool(self.args.mock),
                "port": self.args.port, "baud": self.args.baud, "flash_baud": self.args.flash_baud,
                "project": self.args.project,
                "state": self.state, "op": self.op, "op_seq": self.op_seq,
                "op_started": self.op_started, "progress": self.progress, "phase": self.phase,
                "cancellable": self.proc is not None,
                "last_result": self.last_result,
                "logging": self.logging, "port_open": self.port_open, "port_error": self.port_error,
                "serial_paused": self.serial_blocked,
                "session_file": os.path.basename(self.session_path) if self.session_path else None,
                "session_path": self.session_path, "session_started": self.session_started,
                "session_lines": self.session_lines,
                "lines_per_s": round(len(self.line_times) / 3.0, 1),
                "auto_log": self.auto_log,
                "crash_count": self.crash_count, "last_crash": self.last_crash,
                "clients": self.hub.count, "now": time.time(),
            }

    def publish_status(self) -> None:
        self.hub.publish("status", self.status())

    def _set_state(self, st, progress="keep", phase=None) -> None:
        with self.lock:
            self.state = st
            if progress != "keep":
                self.progress = progress
            if phase is not None:
                self.phase = phase
        self.publish_status()

    def _idle_state(self) -> str:
        if self.error_sticky:
            return "ERROR"
        return "LOGGING" if self.logging else "IDLE"

    def alert(self, level, title, text="", sound=None, notify=False) -> None:
        self.hub.publish("alert", {"level": level, "title": title, "text": text, "sound": sound,
                                   "notify": notify, "t": time.time()})

    # ------------------------------------------------------------------ log lines
    def emit(self, kind: str, text: str, level: str = "") -> None:
        """kind: d=device  t=tool(build/flash)  m=marker  s=console/system."""
        with self.lock:
            self.seq += 1
            rel = (time.monotonic() - self.session_t0) if self.session_t0 is not None else None
            e = {"seq": self.seq, "t": round(time.time(), 3),
                 "rel": round(rel, 2) if rel is not None else None,
                 "k": kind, "l": level, "x": text}
            self.lines.append(e)
            self.pending.append(e)
            if self.session_fh:
                prefix = {"d": "", "m": "", "t": "[idf] ", "s": "[console] "}[kind]
                try:
                    self.session_fh.write(f"[{rel:6.2f}] {prefix}{text}\n")
                except Exception:
                    pass

    def marker(self, text: str) -> None:
        self.emit("m", f"── {text} ──")

    def _flusher(self) -> None:
        last_rate = 0.0
        while True:
            time.sleep(0.1)
            with self.lock:
                batch, self.pending = self.pending, []
            if batch:
                self.hub.publish("log", batch)
            now = time.monotonic()
            if now - last_rate >= 1.0:
                last_rate = now
                st = self.status()
                self.hub.publish("rate", {"lps": st["lines_per_s"], "session_lines": st["session_lines"],
                                          "now": st["now"]})

    # ------------------------------------------------------------------ session
    def _session_open(self) -> None:
        with self.lock:
            if self.session_fh:
                return
            d = self.args.logs_dir
            os.makedirs(d, exist_ok=True)
            base = datetime.now().strftime("session-%Y%m%d-%H%M%S")
            path = os.path.join(d, base + ".log")
            n = 2
            while os.path.exists(path):
                path = os.path.join(d, f"{base}-{n}.log")
                n += 1
            self.session_fh = open(path, "w", encoding="utf-8", buffering=1)
            self.session_path = path
            self.session_t0 = time.monotonic()
            self.session_started = time.time()
            self.session_lines = 0

    def _session_close(self) -> None:
        with self.lock:
            if self.session_fh:
                try:
                    self.session_fh.close()
                except Exception:
                    pass
            self.session_fh = None
            self.session_t0 = None
            # keep session_path so the UI can still show the last file name

    # ------------------------------------------------------------------ serial
    def _open_serial(self):
        if self.args.mock:
            s = MockSerial(self.mock_dev, self.args.port)
            s.dtr = False
            s.rts = False
            s.open()
            return s
        import serial  # pyserial
        s = serial.Serial()
        s.port = self.args.port
        s.baudrate = self.args.baud
        s.timeout = 0.1
        s.dtr = False      # set BEFORE open() so opening does not reset the board
        s.rts = False
        s.open()
        return s

    @staticmethod
    def _close_ser(ser) -> None:
        try:
            ser.close()
        except Exception:
            pass

    def _start_reader(self) -> None:
        with self.lock:
            if self.reader and self.reader.is_alive():
                return
            self.first_line_evt.clear()
            stop = threading.Event()
            self.reader_stop = stop
            self.reader = threading.Thread(target=self._reader_main, args=(stop,), name="serial-reader", daemon=True)
            self.reader.start()

    def _stop_reader(self) -> None:
        with self.lock:
            t, stop = self.reader, self.reader_stop
            ser = self.ser
        if stop:
            stop.set()
        if ser is not None:
            try:
                ser.cancel_read()
            except Exception:
                pass
        if t:
            t.join(timeout=3.0)
        with self.lock:
            self.reader = None
            self.reader_stop = None

    def _reader_main(self, stop: threading.Event) -> None:
        port = self.args.port
        ser = None
        buf = bytearray()
        last_rx = time.monotonic()
        err_reported = None
        try:
            while not stop.is_set():
                if ser is None:
                    if self.serial_blocked:
                        stop.wait(0.2)
                        continue
                    try:
                        ser = self._open_serial()
                    except Exception as e:  # busy / missing port
                        msg = str(e)
                        with self.lock:
                            self.port_error = msg
                        if err_reported != msg:
                            hint = " (busy? close other serial monitors)" if re.search(
                                r"denied|busy|PermissionError", msg, re.I) else ""
                            self.emit("s", f"cannot open {port}: {msg}{hint} — retrying every 1 s", "E")
                            self.alert("error", f"Cannot open {port}", msg + hint, sound="fail")
                            err_reported = msg
                            self.publish_status()
                        stop.wait(1.0)
                        continue
                    with self.lock:
                        self.ser = ser
                        self.port_open = True
                        self.port_error = None
                    if err_reported:
                        self.emit("s", f"{port} opened", "I")
                        err_reported = None
                    self.publish_status()
                try:
                    n = ser.in_waiting
                    data = ser.read(min(n, 65536) if n else 1)
                except Exception as e:
                    if stop.is_set():
                        break
                    self.marker(f"serial port lost ({e}) — reconnecting")
                    self._close_ser(ser)
                    ser = None
                    with self.lock:
                        self.ser = None
                        self.port_open = False
                    self.publish_status()
                    stop.wait(0.5)
                    continue
                now = time.monotonic()
                if data:
                    buf += data
                    last_rx = now
                    while True:
                        i = buf.find(b"\n")
                        if i < 0:
                            break
                        line = bytes(buf[:i])
                        del buf[:i + 1]
                        self._on_device_line(line)
                    if len(buf) > 4096:
                        self._on_device_line(bytes(buf))
                        buf.clear()
                elif buf and now - last_rx > 0.3:
                    self._on_device_line(bytes(buf))
                    buf.clear()
        except Exception:
            self.emit("s", "serial reader crashed: " + traceback.format_exc().splitlines()[-1], "E")
        finally:
            if buf:
                self._on_device_line(bytes(buf))
            if ser is not None:
                self._close_ser(ser)
            with self.lock:
                self.ser = None
                self.port_open = False
            self.publish_status()

    def _on_device_line(self, raw: bytes) -> None:
        text = raw.decode("utf-8", "replace")
        text = ANSI_RE.sub("", text)
        text = CTRL_RE.sub("", text).rstrip()
        if not text.strip():
            return
        m = LEVEL_RE.match(text)
        lvl = m.group(1) if m else ""
        now = time.time()
        with self.lock:
            ch, alerts = self.parser.feed(text, now)
            self.line_times.append(time.monotonic())
            self.session_lines += 1
        if not lvl and CRASH_RE.search(text):
            lvl = "E"
        self.emit("d", text, lvl)
        self.first_line_evt.set()
        if ch:
            self.hub.publish("stats", {"t": now, "v": ch})
        for kind, txt in alerts:
            self._on_crashlike(kind, txt)

    def _on_crashlike(self, kind: str, text: str) -> None:
        mono = time.monotonic()
        if kind == "boot":
            if mono >= self.expect_reboot_until:
                self.alert("info", "Board booted", text[:160])
            return
        if kind == "reboot" and mono < self.expect_reboot_until:
            return  # we caused it (flash / reset)
        with self.lock:
            new = mono - self.last_crash_mono > 8.0  # Guru + Backtrace + Rebooting... = one crash
            self.last_crash_mono = mono
            if new:
                self.crash_count += 1
                self.last_crash = {"t": time.time(), "text": text[:300]}
            count = self.crash_count
        if new:
            title = "Crash detected" if kind == "crash" else "Unexpected reboot"
            self.hub.publish("crash", {"count": count, "text": text[:300], "t": time.time()})
            self.alert("crash", f"{title} (#{count})", text[:200], sound="crash", notify=True)
            self.publish_status()

    # ------------------------------------------------------------------ logging API
    def log_start(self, announce=True) -> str:
        with self.lock:
            already = self.logging and self.reader is not None and self.reader.is_alive()
            self.logging = True
            if self.op is None:
                self.error_sticky = False
            blocked = self.serial_blocked
        if already:
            with self.lock:
                if self.op is None:
                    self.state = "LOGGING"
            self.publish_status()
            return "already logging"
        self._session_open()
        if blocked:
            self.marker(f"logging requested — will start after {blocked}")
            self.publish_status()
            return f"logging will start after {blocked}"
        self.marker(f"logging started → {os.path.basename(self.session_path)} ({self.args.port} @ {self.args.baud})")
        self._start_reader()
        with self.lock:
            if self.op is None:
                self.state = "LOGGING"
        self.publish_status()
        if announce:
            self.alert("info", "Logging started", os.path.basename(self.session_path), sound="log", notify=True)
        return "logging started"

    def log_stop(self) -> str:
        with self.lock:
            was = self.logging
            self.logging = False
            if self.op is None:
                self.error_sticky = False
        self._stop_reader()
        if was:
            with self.lock:
                dur = time.monotonic() - self.session_t0 if self.session_t0 else 0
                n = self.session_lines
            self.marker(f"logging stopped ({n} lines, {fmt_dur(dur)})")
            path = self.session_path
            self._session_close()
            self.alert("info", "Logging stopped", f"{os.path.basename(path or '')} — {n} lines")
        with self.lock:
            if self.op is None:
                self.state = self._idle_state()
        self.publish_status()
        return "logging stopped" if was else "was not logging"

    def _resume_after_serial_op(self, what: str, was_logging: bool) -> bool:
        """Reopen the port if logging is wanted. Returns True if the reader runs."""
        with self.lock:
            want = self.logging
        if not want:
            return False
        self._session_open()
        if was_logging:
            self.marker("logging resumed")
        else:
            self.marker(f"logging started → {os.path.basename(self.session_path)} ({self.args.port} @ {self.args.baud})")
            self.alert("info", "Logging started", os.path.basename(self.session_path), sound="log", notify=True)
        self._start_reader()
        return True

    # ------------------------------------------------------------------ ops
    def start_op(self, name: str, initial_state: str, fn, *fargs) -> dict:
        if not self.op_lock.acquire(blocking=False):
            with self.lock:
                age = time.time() - (self.op_started or time.time())
                raise Busy(f"cannot start {name}: '{self.op}' is in progress ({self.state}, "
                           f"started {age:.0f} s ago). Wait for it or POST /api/cancel.")
        with self.lock:
            self.op = name
            self.op_seq += 1
            self.op_started = time.time()
            self.progress = 0.0 if initial_state in ("BUILDING", "FLASHING") else None
            self.phase = "Starting…"
            self.state = initial_state
            self.error_sticky = False
            self.cancelled = False
            seq = self.op_seq
        self.publish_status()

        def run():
            try:
                fn(*fargs)
            except Exception as e:
                self.emit("s", f"{name} crashed: {e!r}", "E")
                traceback.print_exc()
                self._finish(name, False, f"internal error: {e}", 0.0)
            finally:
                with self.lock:
                    self.op = None
                    self.proc = None
                    self.progress = None
                    self.phase = ""
                    self.serial_blocked = None
                    self.state = self._idle_state()
                self.op_lock.release()
                self.publish_status()

        threading.Thread(target=run, name=f"op-{name}", daemon=True).start()
        return {"op": name, "op_seq": seq}

    def _finish(self, op: str, ok: bool, detail: str, dur: float, notify=True, extra="") -> None:
        label = {"build": "Build", "flash": "Flash", "reset": "Reset"}.get(op, op.capitalize())
        text = f"{label} finished OK" if ok else f"{label} FAILED"
        with self.lock:
            self.last_result = {"op": op, "ok": ok, "text": text, "detail": detail,
                                "at": time.time(), "duration": round(dur, 1), "op_seq": self.op_seq}
            if not ok:
                self.error_sticky = True
        body = f"{fmt_dur(dur)}" + (f" — {detail}" if detail else "") + (f" · {extra}" if extra else "")
        self.alert("ok" if ok else "error", text, body, sound=("ok" if ok else "fail"), notify=notify)
        self.publish_status()

    def cancel(self) -> str:
        with self.lock:
            p = self.proc
            if p is None:
                return "nothing to cancel"
            self.cancelled = True
        try:
            if os.name == "nt":
                subprocess.run(["taskkill", "/F", "/T", "/PID", str(p.pid)], capture_output=True, timeout=10)
            else:
                p.kill()
        except Exception as e:
            return f"cancel failed: {e}"
        return "cancel requested"

    # -- subprocess runner ---------------------------------------------------
    def _ps_script(self, idf_args: str) -> str:
        a = self.args
        return ("$ErrorActionPreference='Continue'; "
                "try { [Console]::OutputEncoding = [System.Text.Encoding]::UTF8 } catch {}; "
                f'$env:IDF_PATH="{a.idf_path}"; $env:IDF_TOOLS_PATH="$env:USERPROFILE\\.espressif"; '
                '$env:IDF_TARGET="esp32s31"; $env:PYTHONUNBUFFERED="1"; $env:PYTHONIOENCODING="utf-8"; '
                '. "$env:IDF_PATH\\export.ps1" | Out-Null; '
                f'Set-Location "{a.project}"; '
                f"idf.py {idf_args}; "
                "if ($null -eq $LASTEXITCODE) { exit 97 } else { exit $LASTEXITCODE }")

    def _run_tool(self, kind: str, idf_args: str, on_line) -> int:
        if self.args.mock:
            cmd = [sys.executable, os.path.abspath(__file__), "--mock-tool", kind]
            if self.mock_fail:
                cmd += ["--mock-fail", str(self.mock_fail)]
            shown = f"idf.py {idf_args}   [mock]"
        else:
            cmd = ["powershell.exe", "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass",
                   "-Command", self._ps_script(idf_args)]
            shown = f"idf.py {idf_args}"
        self.emit("t", f"> {shown}", "cmd")
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        env["PYTHONIOENCODING"] = "utf-8"
        cwd = self.args.project if os.path.isdir(self.args.project) else HERE
        flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                             cwd=cwd, env=env, creationflags=flags)
        with self.lock:
            self.proc = p
        self.publish_status()
        buf = b""
        splitter = re.compile(rb"\r\n|\r|\n")
        try:
            while True:
                chunk = p.stdout.read1(8192)
                if not chunk:
                    break
                buf += chunk
                parts = splitter.split(buf)
                buf = parts.pop()
                for part in parts:
                    self._tool_line(part, on_line)
            if buf:
                self._tool_line(buf, on_line)
        finally:
            rc = p.wait()
            with self.lock:
                self.proc = None
        return rc

    def _tool_line(self, raw: bytes, on_line) -> None:
        text = ANSI_RE.sub("", raw.decode("utf-8", "replace"))
        text = CTRL_RE.sub("", text).rstrip()
        if not text.strip():
            return
        show = on_line(text)
        if show is False:
            return
        low = text.lower()
        if re.search(r"\berror\b|fatal|failed|could not|denied", low):
            lvl = "E"
        elif re.search(r"\bwarning\b", low):
            lvl = "W"
        else:
            lvl = ""
        self.emit("t", text, lvl)

    # -- build ------------------------------------------------------------------
    def _do_build(self, part_of_flash: bool) -> bool:
        t0 = time.monotonic()
        self._set_state("BUILDING", progress=0.0, phase="Starting build…")
        self.marker("build started")
        seen_ok = [False]
        err = [None]

        def on_line(t):
            m = FlashProgress.NINJA_RE.match(t.strip())
            if m:
                n, tot = int(m.group(1)), int(m.group(2))
                with self.lock:
                    self.progress = round(100.0 * n / max(1, tot), 1)
                    self.phase = f"Compiling {n}/{tot}"
                self.publish_status()
            if "Project build complete" in t:
                seen_ok[0] = True
            if err[0] is None and re.search(r"\berror:|ninja: build stopped|CMake Error|FAILED:", t):
                err[0] = t.strip()[:200]
            return True

        rc = self._run_tool("build", "build", on_line)
        dt = time.monotonic() - t0
        if self.cancelled:
            ok, detail = False, "cancelled by user"
        elif rc == 0:
            ok, detail = True, ""
            if not seen_ok[0]:
                self.emit("s", "idf.py exited 0 but 'Project build complete' was not seen", "W")
        else:
            ok, detail = False, (err[0] or f"idf.py exit code {rc}")
        self.marker(f"build finished (ok, {fmt_dur(dt)})" if ok else f"build FAILED ({detail}, {fmt_dur(dt)})")
        if ok:
            with self.lock:
                self.progress = 100.0
                self.phase = "Build complete"
        if part_of_flash and ok:
            self.alert("ok", "Build finished OK", f"{fmt_dur(dt)} — flashing next", sound=None)
        else:
            self._finish("build", ok, detail, dt, notify=True,
                         extra=("flash skipped" if part_of_flash and not ok else ""))
        return ok

    def op_build(self) -> None:
        self._do_build(part_of_flash=False)

    # -- flash ------------------------------------------------------------------
    def _flash_layout(self):
        if self.args.mock:
            return MOCK_LAYOUT
        try:
            bdir = os.path.join(self.args.project, "build")
            with open(os.path.join(bdir, "flasher_args.json"), encoding="utf-8") as f:
                fa = json.load(f)
            out = []
            for off, rel in fa.get("flash_files", {}).items():
                p = os.path.join(bdir, rel)
                if os.path.exists(p):
                    out.append((int(off, 16), os.path.getsize(p), rel))
            return out
        except Exception:
            return []

    def op_flash(self, build: bool) -> None:
        if build and not self._do_build(part_of_flash=True):
            return  # build failure already reported (last_result = build FAILED); flash skipped
        with self.lock:
            was_logging = self.logging and self.reader is not None
            self.serial_blocked = "flash"
        if was_logging:
            self.marker("pausing logging for flash — closing " + self.args.port)
        self._stop_reader()   # port closed (no-op if not logging)
        t0 = time.monotonic()
        self._set_state("FLASHING", progress=0.0, phase="Starting esptool…")
        self.marker(f"flash started ({self.args.port} @ {self.args.flash_baud})")
        fp = FlashProgress(self._flash_layout())
        flags = {"ok_seen": False, "err": None}

        def on_line(t):
            pct, phase, show = fp.feed(t)
            if pct is not None or phase:
                with self.lock:
                    if pct is not None:
                        self.progress = round(pct, 1)
                    if phase:
                        self.phase = phase
                self.publish_status()
            low = t.lower()
            if "hash of data verified" in low or "hard resetting" in low or "leaving..." in low:
                flags["ok_seen"] = True
            if flags["err"] is None:
                if re.search(r"could not open port|access is denied|permissionerror|port is busy|resource busy", low):
                    flags["err"] = f"{self.args.port} is busy/unavailable — close other serial monitors"
                elif "failed to connect" in low:
                    flags["err"] = "Failed to connect to the chip (boot mode / cable?)"
                elif "a fatal error occurred" in low:
                    flags["err"] = t.split(":", 1)[-1].strip()[:160]
                elif re.search(r"\berror:|ninja: build stopped", t):
                    flags["err"] = t.strip()[:160]
            return show

        rc = self._run_tool("flash", f"-p {self.args.port} -b {self.args.flash_baud} flash", on_line)
        dt = time.monotonic() - t0
        if self.cancelled:
            ok, detail = False, "cancelled by user"
        elif rc == 0 and flags["err"] is None:
            ok, detail = True, ""
            if not flags["ok_seen"]:
                self.emit("s", "esptool exited 0 but no 'Hash of data verified' / 'Hard resetting' seen", "W")
        else:
            ok, detail = False, flags["err"] or f"idf.py exit code {rc}"

        if ok:
            self.marker(f"flash finished (ok, {fmt_dur(dt)})")
            with self.lock:
                self.progress = 100.0
                self.expect_reboot_until = time.monotonic() + 10.0
                if self.auto_log:
                    self.logging = True
            if self.args.mock:
                self.mock_dev.reboot("flash")
            self._set_state("REBOOTING", progress=None, phase="Board rebooting…")
            with self.lock:
                self.serial_blocked = None
            running = self._resume_after_serial_op("flash", was_logging)
            extra = ("logging resumed" if was_logging else "logging started") if running else "auto-log off"
            self._finish("flash", True, "", dt, extra=extra)
            if running:
                self.first_line_evt.wait(timeout=6.0)
        else:
            self.marker(f"flash FAILED ({detail}, {fmt_dur(dt)})")
            with self.lock:
                self.serial_blocked = None
            self._finish("flash", False, detail, dt)
            self._resume_after_serial_op("flash", was_logging)


    # -- reset ------------------------------------------------------------------
    def op_reset(self) -> None:
        t0 = time.monotonic()
        with self.lock:
            self.expect_reboot_until = time.monotonic() + 8.0
            ser = self.ser if (self.reader is not None and self.port_open) else None
        self.marker("board reset (RTS pulse)")
        with self.lock:
            self.phase = "Pulsing RTS…"
        self.publish_status()
        self.first_line_evt.clear()
        if ser is not None:
            ser.dtr = False
            ser.rts = True
            time.sleep(0.1)
            ser.rts = False
        else:
            # not logging (or port currently lost): open the port just for the pulse
            with self.lock:
                was_logging = self.logging
                self.serial_blocked = "reset"
            self._stop_reader()
            try:
                s = self._open_serial()
                s.dtr = False
                s.rts = True
                time.sleep(0.1)
                s.rts = False
                self._close_ser(s)
            except Exception as e:
                with self.lock:
                    self.serial_blocked = None
                hint = " — port busy?" if re.search(r"denied|busy", str(e), re.I) else ""
                self.marker(f"reset FAILED ({e})")
                self._finish("reset", False, f"{e}{hint}", time.monotonic() - t0)
                self._resume_after_serial_op("reset", was_logging)
                return
            with self.lock:
                self.serial_blocked = None
            self._resume_after_serial_op("reset", was_logging)
        with self.lock:
            self.phase = "Board rebooting…"
            logging = self.logging
        self.publish_status()
        if logging:
            self.first_line_evt.wait(timeout=4.0)
        else:
            time.sleep(0.5)
        self.marker("board reset done")
        self._finish("reset", True, "", time.monotonic() - t0, notify=False)

    # ------------------------------------------------------------------ requests
    def request_build(self, body) -> dict:
        self.mock_fail = body.get("mock_fail") if self.args.mock else None
        return self.start_op("build", "BUILDING", self.op_build)

    def request_flash(self, body) -> dict:
        build = bool(body.get("build", False))
        self.mock_fail = body.get("mock_fail") if self.args.mock else None
        return self.start_op("build+flash" if build else "flash", "BUILDING" if build else "FLASHING",
                             self.op_flash, build)

    def request_reset(self, body) -> dict:
        return self.start_op("reset", "REBOOTING", self.op_reset)

    def ack(self) -> str:
        """Clear a sticky ERROR state (the failure stays in last_result)."""
        with self.lock:
            self.error_sticky = False
            if self.op is None:
                self.state = self._idle_state()
        self.publish_status()
        return "error acknowledged"

    def set_config(self, body) -> dict:
        with self.lock:
            if "auto_log" in body:
                self.auto_log = bool(body["auto_log"])
        self.publish_status()
        return {"auto_log": self.auto_log}

    def tail(self, n: int, since: int | None, kind: str | None) -> list:
        with self.lock:
            items = list(self.lines)
        if since is not None:
            items = [e for e in items if e["seq"] > since]
        if kind:
            items = [e for e in items if e["k"] in kind]
        return items[-n:] if n > 0 else items

    def stats(self) -> dict:
        with self.lock:
            snap = self.parser.snapshot()
            snap["crash_count"] = self.crash_count
            snap["last_crash"] = self.last_crash
            snap["now"] = time.time()
            return snap

    def shutdown(self) -> None:
        with self.lock:
            p = self.proc
        if p is not None:
            self.cancel()
        self._stop_reader()
        if self.logging:
            self.marker("console server stopped")
        self._session_close()


# =============================================================================
# HTTP
# =============================================================================
CTYPES = {".html": "text/html; charset=utf-8", ".js": "application/javascript; charset=utf-8",
          ".css": "text/css; charset=utf-8", ".svg": "image/svg+xml", ".png": "image/png",
          ".ico": "image/x-icon", ".json": "application/json"}


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = os.name != "nt"   # on Windows SO_REUSEADDR lets a 2nd instance steal the port
    console: Console


class Handler(BaseHTTPRequestHandler):
    server_version = f"S31Console/{VERSION}"

    def log_message(self, fmt, *a):  # quiet
        if self.server.console.args.verbose:
            super().log_message(fmt, *a)

    @property
    def c(self) -> Console:
        return self.server.console

    def _json(self, code: int, obj) -> None:
        data = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _body(self) -> dict:
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0:
            return {}
        raw = self.rfile.read(n)
        try:
            v = json.loads(raw.decode("utf-8") or "{}")
            return v if isinstance(v, dict) else {}
        except Exception:
            return {}

    # ---------------------------------------------------------------- GET
    def do_GET(self):
        u = urlparse(self.path)
        p = u.path
        qs = parse_qs(u.query)
        try:
            if p == "/events":
                return self._sse()
            if p == "/api/status":
                return self._json(200, self.c.status())
            if p == "/api/stats":
                return self._json(200, self.c.stats())
            if p == "/api/log/tail":
                n = int((qs.get("n") or ["200"])[0])
                since = qs.get("since")
                kind = (qs.get("kind") or [None])[0]
                lines = self.c.tail(n, int(since[0]) if since else None, kind)
                fmt = (qs.get("format") or ["json"])[0]
                if fmt == "text":
                    body = "\n".join(
                        (f"[{e['rel']:6.2f}] " if e["rel"] is not None else "") + e["x"] for e in lines) + "\n"
                    data = body.encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/plain; charset=utf-8")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                    return
                return self._json(200, {"lines": lines, "last_seq": self.c.seq,
                                        "session_file": self.c.session_path})
            return self._static(p)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def _static(self, p: str):
        if p in ("/", ""):
            p = "/index.html"
        rel = p.lstrip("/").replace("/", os.sep)
        full = os.path.normpath(os.path.join(STATIC_DIR, rel))
        if not full.startswith(STATIC_DIR + os.sep) or not os.path.isfile(full):
            return self._json(404, {"ok": False, "error": "not found", "path": p})
        with open(full, "rb") as f:
            data = f.read()
        self.send_response(200)
        self.send_header("Content-Type", CTYPES.get(os.path.splitext(full)[1].lower(), "application/octet-stream"))
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(data)

    def _sse(self):
        c = self.c
        q = c.hub.subscribe()
        try:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            with c.lock:
                lines = list(c.lines)[-2000:]
            hello = {"status": c.status(), "lines": lines, "stats": c.stats()}
            self.wfile.write(b"retry: 1500\n\n" + Hub.encode("hello", hello))
            self.wfile.flush()
            while True:
                if q.overflow:
                    break
                try:
                    msg = q.get(timeout=15)
                except queue.Empty:
                    self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
                    continue
                chunks = [msg]
                while len(chunks) < 200:
                    try:
                        chunks.append(q.get_nowait())
                    except queue.Empty:
                        break
                self.wfile.write(b"".join(chunks))
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass
        finally:
            c.hub.unsubscribe(q)

    # ---------------------------------------------------------------- POST
    def do_POST(self):
        p = urlparse(self.path).path
        body = self._body()
        c = self.c
        try:
            if p == "/api/build":
                r = c.request_build(body)
                return self._json(200, {"ok": True, "message": "build started", **r, "status": c.status()})
            if p == "/api/flash":
                r = c.request_flash(body)
                msg = "build + flash started" if body.get("build") else "flash started"
                return self._json(200, {"ok": True, "message": msg, **r, "status": c.status()})
            if p == "/api/reset":
                r = c.request_reset(body)
                return self._json(200, {"ok": True, "message": "reset started", **r, "status": c.status()})
            if p == "/api/log/start":
                return self._json(200, {"ok": True, "message": c.log_start(), "status": c.status()})
            if p == "/api/log/stop":
                return self._json(200, {"ok": True, "message": c.log_stop(), "status": c.status()})
            if p == "/api/mark":
                text = str(body.get("text") or "").strip() or "mark"
                c.marker(text[:300])
                return self._json(200, {"ok": True, "message": "marker added", "seq": c.seq})
            if p == "/api/ack":
                return self._json(200, {"ok": True, "message": c.ack(), "status": c.status()})
            if p == "/api/cancel":
                return self._json(200, {"ok": True, "message": c.cancel()})
            if p == "/api/config":
                return self._json(200, {"ok": True, **c.set_config(body)})
            if p == "/api/shutdown":
                self._json(200, {"ok": True, "message": "shutting down"})
                threading.Thread(target=self.server.shutdown, daemon=True).start()
                return
            return self._json(404, {"ok": False, "error": "unknown endpoint", "path": p})
        except Busy as e:
            return self._json(409, {"ok": False, "error": "busy", "message": str(e), "status": c.status()})
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass
        except Exception as e:
            traceback.print_exc()
            return self._json(500, {"ok": False, "error": "internal", "message": repr(e)})


# =============================================================================
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="S31 Console server (flash + serial monitor + web UI)")
    ap.add_argument("--mock", action="store_true", help="simulate serial port + build/flash (no hardware)")
    ap.add_argument("--port", default="COM5", help="serial port (default COM5)")
    ap.add_argument("--baud", type=int, default=921600, help="monitor baud (default 921600)")
    ap.add_argument("--flash-baud", type=int, default=921600, help="esptool baud (default 921600)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--http-port", type=int, default=8765)
    ap.add_argument("--project", default=DEFAULT_PROJECT)
    ap.add_argument("--idf-path", default=DEFAULT_IDF_PATH)
    ap.add_argument("--logs-dir", default=None,
                    help="session log folder (default <project>\\logs; --mock: tools\\s31_console\\mock_logs)")
    ap.add_argument("--no-auto-log", action="store_true", help="do not start logging automatically after a flash")
    ap.add_argument("--log-on-start", action="store_true", help="open the port and start logging at server start")
    ap.add_argument("--verbose", action="store_true", help="print HTTP access log")
    ap.add_argument("--mock-tool", help=argparse.SUPPRESS)
    ap.add_argument("--mock-fail", default="", help=argparse.SUPPRESS)
    args = ap.parse_args(argv)

    if args.mock_tool:
        return mock_tool_main(args.mock_tool, args.mock_fail)

    if args.logs_dir is None:
        args.logs_dir = os.path.join(HERE, "mock_logs") if args.mock else os.path.join(args.project, "logs")
    if not args.mock:
        try:
            import serial  # noqa: F401
        except ImportError:
            print("pyserial is not installed in this Python. Use the IDF python env, e.g.\n"
                  r"  C:\Users\<you>\.espressif\python_env\idf6.1_py3.12_env\Scripts\python.exe server.py", file=sys.stderr)
            return 2

    console = Console(args)
    try:
        srv = Server((args.host, args.http_port), Handler)
    except OSError as e:
        print(f"cannot listen on {args.host}:{args.http_port}: {e} (already running?)", file=sys.stderr)
        return 3
    srv.console = console
    mode = "MOCK (simulated board)" if args.mock else f"{args.port} @ {args.baud}"
    print(f"S31 Console {VERSION}  —  http://{args.host}:{args.http_port}/   [{mode}]")
    print(f"  session logs: {args.logs_dir}")
    console.emit("s", f"S31 Console {VERSION} started — {mode}", "I")
    if args.log_on_start:
        console.log_start(announce=False)
    try:
        srv.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        print("\nstopping…")
    finally:
        console.shutdown()
        srv.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
