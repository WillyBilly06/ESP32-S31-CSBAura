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

## Live metrics (left panel)
Parsed by `StatsParser` in `server.py` from the **s31-btstack-speaker** status lines (one each every 2 s) and event lines:

| Card group | Source lines (`TAG: …`) |
|---|---|
| Phone · A2DP: A2DP state, Volume / AVRCP, Latency (PCM ms, catch-ups, clock corr, delay report), A2DP underruns, SBC bitpool, BR/EDR link | `A2DP: A2DP 2s: …` (the `\| catch-ups … \| volume N/127[ (no AVRCP)]` tail is optional), `A2DP: A2DP audio STARTED/suspended/stopped`, `A2DP: volume N/127 (…) <- …`, `A2DP: AVRCP connected/disconnected`, `A2DP: delay report …`, `A2DP: A2DP connected …, role …`, `LINK: …` |
| Auracast: state, not sent / late, LC3 encode, BIG timing, L/R align | `AURA: AURA 2s: …`, `AURA: BIG timing (BIS0 completions): …`, `AURA: L/R align: …`, `AURA: broadcast ON/off` |
| Party · JBL: Party role, Party listen RX, JBL speakers, JBL beacon partyInfo (decoded), JBL app (GATT / V5 / EQ) | `PARTY: role …, listen … \| …`, `PARTY: LISTEN '<name>' 2s: …`, `PARTY: role X -> Y (why)`, `PARTY: listen: a -> b (why)`, `JBL_ASSIST`, `JBL_BEACON`, `JBL_GATT`, `JBL_V5`, `V5_EQ` |
| System: free internal RAM, VHCI / ISO tx errors, crashes | `S31: HEAP: …`, crash/reboot detection |

Firmware totals (underruns, seq gaps, catch-ups, VHCI drops, …) also get a `<key>_d` field = increase since the previous line; tiles colour on that. A tile fades when its line has not been seen for 10 s.

Offline self-test (never opens the port): runs the parser over the tail of a session log and lists which card got values.
```powershell
& $py server.py --selftest --project E:\ESP32-S31-CSBAura\s31-btstack-speaker   # newest logs\session-*.log, last 3000 lines
& $py server.py --selftest mock_logs\sample-btstack-status.log                   # anonymized synthetic sample (incl. listen mode)
```
Exit code 0 = every card has values (cards only printed by newer firmware / listen mode are reported as `n/a`).

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
