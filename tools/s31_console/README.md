# S31 Console

Local web app to build, flash and monitor the ESP32-S31 board (COM5) — Python backend (stdlib + pyserial) + single-page UI.

## Start

```powershell
# real board (COM5 @ 921600); opens http://127.0.0.1:8765/
powershell -ExecutionPolicy Bypass -File E:\ESP32-S31-CSBAura\tools\s31_console\start_console.ps1

# simulator (fake serial + fake build/flash, never touches COM5)
powershell -ExecutionPolicy Bypass -File E:\ESP32-S31-CSBAura\tools\s31_console\start_console.ps1 -Mock
```
Options: `-Port COM7 -Baud 115200 -HttpPort 8765 -LogOnStart -NoAutoLog -NoBrowser`.
Direct: `python server.py [--mock] [--port COM5] [--baud 921600] [--flash-baud 921600] [--log-on-start] [--no-auto-log] [--logs-dir DIR]`
(use the IDF python: `%USERPROFILE%\.espressif\python_env\idf6.1_py3.12_env\Scripts\python.exe`).

The server does **not** open the port at start-up unless `--log-on-start` is given.

## Behaviour
- States: `IDLE, BUILDING, FLASHING (progress %), REBOOTING, LOGGING, ERROR`. `ERROR` is sticky after a failed op until the next op, log start/stop, or `ack` (click the red pill).
- Flash/reset while logging: the reader is stopped and COM5 closed, esptool runs, then the port is reopened and logging resumes **in the same session file**, with markers `── flash started ──`, `── flash finished (ok, 12.3 s) ──`, `── logging resumed ──`.
- After a successful flash, logging starts automatically (toggle "Auto-log after flash" / `auto-log on|off`).
- Port opened with DTR/RTS low set *before* `open()` → no reset. Reset = RTS pulse 100 ms.
- Sessions: `s31-jbl-speaker\logs\session-YYYYmmdd-HHMMSS.log`, lines `[  12.34] <line>`; build/flash output is written as `[idf] …`, markers as `── … ──`. (Mock mode writes to `tools\s31_console\mock_logs\`.)
- Crash detection (Guru Meditation, assert failed, task_wdt, abort, `Rebooting...`, non-POWERON `rst:`) → red toast + sound + counter; reboots that we caused (flash/reset) are ignored.

## API (http://127.0.0.1:8765)
| Method | Path | Notes |
|---|---|---|
| GET | `/api/status` | state, op, progress, phase, logging, session, last_result, crash_count… |
| POST | `/api/build` | 409 if another op is running |
| POST | `/api/flash` | body `{"build": true}` = build then flash |
| POST | `/api/reset` | RTS pulse |
| POST | `/api/log/start`, `/api/log/stop` | start during a flash is deferred until the flash ends |
| POST | `/api/mark` | `{"text": "..."}` inserts a marker line |
| POST | `/api/cancel`, `/api/ack`, `/api/config` (`{"auto_log": bool}`), `/api/shutdown` | |
| GET | `/api/log/tail?n=200[&since=SEQ][&kind=d][&format=text]` | `kind`: d=device t=tool m=marker s=console |
| GET | `/api/stats` | latest parsed values, per-field update times, 2 s history, generic `groups` |
| GET | `/events` | SSE: `hello` (snapshot), `status`, `log` (batched), `stats`, `alert`, `crash`, `rate` |

## CLI (`s31ctl.py`)
```powershell
$py = "$env:USERPROFILE\.espressif\python_env\idf6.1_py3.12_env\Scripts\python.exe"
& $py s31ctl.py status
& $py s31ctl.py flash --build --wait      # exit 0 = flash OK, 1 = failed
& $py s31ctl.py wait-state LOGGING --timeout 60   # accepts lists: LOGGING,ERROR
& $py s31ctl.py log start | log stop | reset [--wait] | mark "A/B run 3" | tail -n 50 | stats | cancel | ack | auto-log off
```
Exit codes: 0 ok, 1 timeout/failed, 2 HTTP error/409 busy, 3 server not reachable.

Mock-only test hooks: `POST /api/flash {"build":true,"mock_fail":"build"|"flash"|"connect"}`.
