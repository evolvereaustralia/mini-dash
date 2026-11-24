#!/usr/bin/env python3
# Record the device's serial output during a drive, with the 10 Hz motion log switched on,
# so the drive can be analysed afterwards (docs/calibration.md, road checks). The device is
# powered and logged over the one USB cable from the Mac on the passenger seat.
#
#   cd firmware && ~/.platformio/penv/bin/python tools/drive_log.py      # until Ctrl-C
#   ~/.platformio/penv/bin/python tools/drive_log.py /dev/cu.usbmodem1101 3600
#
# Writes docs/drive-logs/drive-<date>-<time>.log. Resets the board at the start (so the log
# begins with the boot banner), sends 'g' after 5 s and 'O' (OBD status) once a minute. PlatformIO's own Python has pyserial;
# the system python3 does not.
import glob, os, sys, time
from datetime import datetime
try:
    import serial
except ImportError:
    sys.exit("pyserial missing: python3 -m pip install pyserial")

ports = sys.argv[1:2] or sorted(glob.glob("/dev/cu.usbmodem*"))
if not ports:
    sys.exit("no /dev/cu.usbmodem* port: is the device plugged in?")
port = ports[0]
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 6 * 3600
out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "docs", "drive-logs")
os.makedirs(out_dir, exist_ok=True)
path = os.path.join(out_dir, datetime.now().strftime("drive-%Y-%m-%d-%H%M.log"))
s = serial.Serial(port, 115200, timeout=0.2)
s.dtr = False; s.rts = True; time.sleep(0.1); s.rts = False; s.dtr = True
t0 = time.time()
sent = False
last_status = time.time()
print(f"logging {port} to {path}; Ctrl-C to stop")
with open(path, "wb") as f:
    try:
        while time.time() - t0 < secs:
            if not sent and time.time() - t0 >= 5:
                s.write(b"g"); sent = True
            if time.time() - last_status >= 60:  # OBD rates and per-identifier counts once a minute
                s.write(b"O"); last_status = time.time()
            data = s.read(4096)
            if data:
                f.write(data); f.flush()
    except KeyboardInterrupt:
        pass
s.close()
size = os.path.getsize(path)
print(f"stopped after {int(time.time() - t0)} s, {size // 1024} KB in {path}")
