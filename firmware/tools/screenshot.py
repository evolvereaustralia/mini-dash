#!/usr/bin/env python3
"""Take a screenshot of the Mini Dash over USB serial.

Sends 'p', reads the base64 RGB565 dump the firmware prints between "SNAP w h n" and
"ENDSNAP", and writes a PNG. Needs pyserial and Pillow.

usage: screenshot.py PORT OUT.png [--key K ...]   (keys are sent, 0.4 s apart, before 'p')
"""
import base64, sys, time
import serial
from PIL import Image

port, out = sys.argv[1], sys.argv[2]
keys = []
if '--key' in sys.argv:
    keys = sys.argv[sys.argv.index('--key') + 1:]
B64 = set('ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=')

def grab(s):
    """One attempt: returns (w, h, raw) or None. Other tasks may print log lines in the middle
    of the dump, so lines that look like logs are dropped and stray characters stripped."""
    s.reset_input_buffer()
    s.write(b'p')
    w = h = n = 0
    t0 = time.time()
    while time.time() - t0 < 10:
        line = s.readline().decode('ascii', 'replace').strip()
        if line.startswith('SNAP '):
            _, w, h, n = line.split(); w, h, n = int(w), int(h), int(n); break
    if not w:
        return None
    chunks = []
    clean = True
    while time.time() - t0 < 90:
        line = s.readline().decode('ascii', 'replace').strip()
        if line == 'ENDSNAP':
            break
        if not line or '][' in line or line.startswith('['):
            continue  # a log line from another task
        if any(ch not in B64 for ch in line) or (chunks and len(chunks[-1]) != 76):
            clean = False  # a dump line was split by an interleaved print: bytes would shift
        chunks.append(line)
    if not clean:
        return None
    try:
        raw = base64.b64decode(''.join(chunks))
    except Exception:
        return None
    return (w, h, raw) if len(raw) >= w * h * 2 else None

s = serial.Serial(port, 115200, timeout=2)
for k in keys:
    s.write(k.encode()); time.sleep(0.4)
t0 = time.time()
result = None
for attempt in range(3):
    result = grab(s)
    if result:
        break
    time.sleep(0.5)
if not result:
    sys.exit('no complete screenshot after 3 attempts')
w, h, raw = result
img = Image.frombuffer('RGB', (w, h), raw[:w*h*2], 'raw', 'BGR;16', 0, 1)
img.save(out)
print(f'{out}: {w}x{h} in {time.time()-t0:.1f} s')
