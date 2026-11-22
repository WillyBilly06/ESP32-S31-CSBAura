#!/usr/bin/env python3
"""
s31ctl - tiny CLI for the S31 Console API (stdlib only).

  s31ctl.py status [--json]
  s31ctl.py build [--wait]
  s31ctl.py flash [--build] [--wait]
  s31ctl.py log start | log stop
  s31ctl.py reset [--wait]
  s31ctl.py mark TEXT...
  s31ctl.py wait-state STATE[,STATE...] [--timeout S]
  s31ctl.py tail [-n N] [--since SEQ]
  s31ctl.py stats
  s31ctl.py cancel | ack | auto-log on|off | shutdown

Exit codes: 0 ok, 1 timeout / operation failed, 2 rejected (409 busy) or HTTP error, 3 server not reachable.
"""
import argparse
import json
import sys
import time
import urllib.error
import urllib.request

URL = "http://127.0.0.1:8765"


def call(method, path, body=None, timeout=10):
    data = json.dumps(body or {}).encode() if method == "POST" else None
    req = urllib.request.Request(URL + path, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode("utf-8") or "{}")
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read().decode("utf-8") or "{}")
        except Exception:
            return e.code, {"message": str(e)}
    except (urllib.error.URLError, ConnectionError, TimeoutError) as e:
        print(f"s31ctl: console server not reachable at {URL} ({e})", file=sys.stderr)
        sys.exit(3)


def post(path, body=None):
    code, j = call("POST", path, body)
    if code != 200:
        print(f"s31ctl: HTTP {code}: {j.get('message') or j}", file=sys.stderr)
        sys.exit(2)
    if j.get("message"):
        print(j["message"])
    return j


def summary(st):
    s = f"{st['state']}"
    if st.get("op"):
        s += f"  op={st['op']}"
        if st.get("progress") is not None:
            s += f" {st['progress']:.0f}%"
        if st.get("phase"):
            s += f"  ({st['phase']})"
    s += "  logging=" + ("on" if st["logging"] else "off")
    if st["logging"]:
        if st.get("serial_paused"):
            s += f" (paused for {st['serial_paused']})"
        elif st.get("port_error"):
            s += f" (waiting: {st['port_error']})"
        s += f" -> {st.get('session_file')} {st.get('lines_per_s')} l/s"
    lr = st.get("last_result")
    if lr:
        s += f"  last: {lr['text']} ({lr['duration']} s){' - ' + lr['detail'] if lr.get('detail') else ''}"
    if st.get("crash_count"):
        s += f"  crashes={st['crash_count']}"
    return s


def wait_op(op_seq, timeout):
    """Wait until op `op_seq` is finished; exit code reflects its result."""
    t_end = time.time() + timeout
    last = None
    while time.time() < t_end:
        _, st = call("GET", "/api/status")
        line = summary(st)
        if line != last:
            print(line, flush=True)
            last = line
        lr = st.get("last_result") or {}
        if not st.get("op") and lr.get("op_seq", 0) >= op_seq:
            return 0 if lr.get("ok") else 1
        if not st.get("op") and st.get("op_seq", 0) >= op_seq and not lr:
            return 1
        time.sleep(0.5)
    print("s31ctl: timeout waiting for operation", file=sys.stderr)
    return 1


def main():
    global URL
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass
    ap = argparse.ArgumentParser(prog="s31ctl", description="Drive the S31 Console server")
    ap.add_argument("--url", default=URL)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("status"); p.add_argument("--json", action="store_true")
    p = sub.add_parser("build"); p.add_argument("--wait", action="store_true"); p.add_argument("--timeout", type=float, default=900)
    p = sub.add_parser("flash"); p.add_argument("--build", action="store_true"); p.add_argument("--wait", action="store_true")
    p.add_argument("--timeout", type=float, default=900)
    p = sub.add_parser("log"); p.add_argument("action", choices=["start", "stop"])
    p = sub.add_parser("reset"); p.add_argument("--wait", action="store_true"); p.add_argument("--timeout", type=float, default=30)
    p = sub.add_parser("mark"); p.add_argument("text", nargs="+")
    p = sub.add_parser("wait-state"); p.add_argument("state"); p.add_argument("--timeout", type=float, default=60)
    p = sub.add_parser("tail"); p.add_argument("-n", type=int, default=50); p.add_argument("--since", type=int)
    sub.add_parser("stats")
    sub.add_parser("cancel")
    sub.add_parser("ack")
    p = sub.add_parser("auto-log"); p.add_argument("value", choices=["on", "off"])
    sub.add_parser("shutdown")
    a = ap.parse_args()
    URL = a.url.rstrip("/")

    if a.cmd == "status":
        _, st = call("GET", "/api/status")
        print(json.dumps(st, indent=2) if a.json else summary(st))
        return 0
    if a.cmd == "build":
        j = post("/api/build")
        return wait_op(j["op_seq"], a.timeout) if a.wait else 0
    if a.cmd == "flash":
        j = post("/api/flash", {"build": a.build})
        return wait_op(j["op_seq"], a.timeout) if a.wait else 0
    if a.cmd == "log":
        post(f"/api/log/{a.action}")
        return 0
    if a.cmd == "reset":
        j = post("/api/reset")
        return wait_op(j["op_seq"], a.timeout) if a.wait else 0
    if a.cmd == "mark":
        post("/api/mark", {"text": " ".join(a.text)})
        return 0
    if a.cmd == "wait-state":
        want = {s.strip().upper() for s in a.state.split(",")}
        t_end = time.time() + a.timeout
        while True:
            _, st = call("GET", "/api/status")
            if st["state"] in want:
                print(summary(st))
                return 0
            if time.time() >= t_end:
                print(f"s31ctl: timeout after {a.timeout:.0f} s; state is {st['state']}", file=sys.stderr)
                return 1
            time.sleep(0.25)
    if a.cmd == "tail":
        q = f"?n={a.n}" + (f"&since={a.since}" if a.since is not None else "")
        _, j = call("GET", "/api/log/tail" + q)
        for e in j["lines"]:
            rel = f"[{e['rel']:6.2f}] " if e.get("rel") is not None else ""
            print(rel + e["x"])
        return 0
    if a.cmd == "stats":
        _, j = call("GET", "/api/stats")
        j.pop("history", None)
        print(json.dumps(j, indent=2))
        return 0
    if a.cmd == "cancel":
        post("/api/cancel"); return 0
    if a.cmd == "ack":
        post("/api/ack"); return 0
    if a.cmd == "auto-log":
        j = post("/api/config", {"auto_log": a.value == "on"})
        print(f"auto_log = {j.get('auto_log')}")
        return 0
    if a.cmd == "shutdown":
        post("/api/shutdown"); return 0
    return 2


if __name__ == "__main__":
    sys.exit(main())
