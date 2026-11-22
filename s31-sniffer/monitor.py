"""Non-blocking serial logger for the S31 sniffer.

Stays alive forever. Watches for a pause-flag file: when it appears, the logger
CLOSES the serial port (so `idf.py flash` can grab it), and when the flag is
removed it reconnects automatically. So flashing never has to kill the logger:

    "" | Out-File pause.flag      # logger releases COM5
    idf.py -p COM5 flash          # flash freely
    Remove-Item pause.flag        # logger reconnects and resumes logging

Usage: python monitor.py [PORT] [BAUD] [LOGFILE] [PAUSEFLAG]
"""
import serial, sys, time, os
port  = sys.argv[1] if len(sys.argv) > 1 else "COM5"
baud  = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
out   = sys.argv[3] if len(sys.argv) > 3 else "capture.log"
pause = sys.argv[4] if len(sys.argv) > 4 else "pause.flag"

f = open(out, "a", encoding="utf-8", errors="replace")
def w(m):
    f.write(m); f.flush()
    try:
        sys.stdout.write(m); sys.stdout.flush()
    except Exception:
        pass

w("\n==== logger start %s %s@%d (pause file: %s) ====\n" % (time.strftime("%H:%M:%S"), port, baud, pause))
s = None
while True:
    if os.path.exists(pause):
        if s is not None:
            try: s.close()
            except Exception: pass
            s = None
            w("[%s] -- paused: COM released for flashing --\n" % time.strftime("%H:%M:%S"))
        time.sleep(0.3)
        continue
    if s is None:
        try:
            s = serial.Serial(port, baud, timeout=1)
            w("[%s] -- connected %s@%d --\n" % (time.strftime("%H:%M:%S"), port, baud))
        except Exception:
            time.sleep(0.5)
            continue
    try:
        line = s.readline()
    except Exception:
        try: s.close()
        except Exception: pass
        s = None
        time.sleep(0.5)
        continue
    if line:
        w("[%s] %s\n" % (time.strftime("%H:%M:%S"), line.decode("utf-8", errors="replace").rstrip()))
