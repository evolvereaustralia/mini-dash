#!/bin/sh
# Native tests (no PlatformIO, no board): the attitude filter, speed fusion and accelerometer
# calibration against a synthetic drive, the OBD protocol layer against recorded replies, and
# the trip engine's record bookkeeping.
set -e
cd "$(dirname "$0")"
clang++ -std=c++17 -O1 -Wall -o /tmp/minidash_test_fusion test_fusion.cpp ../../src/app/accel_cal.cpp ../../src/app/attitude.cpp ../../src/app/speed_fusion.cpp
/tmp/minidash_test_fusion
clang++ -std=c++17 -O1 -Wall -o /tmp/minidash_test_speed test_speed.cpp ../../src/app/speed_fusion.cpp
/tmp/minidash_test_speed
clang++ -std=c++17 -O1 -Wall -I../../include -o /tmp/minidash_test_obd test_obd.cpp ../../src/app/obd_proto.cpp ../../src/app/obd_decode.cpp
/tmp/minidash_test_obd
clang++ -std=c++17 -O1 -Wall -I../../include -I../../src/app -o /tmp/minidash_test_trip test_trip.cpp ../../src/app/trip_engine.cpp
/tmp/minidash_test_trip
clang++ -std=c++17 -O1 -Wall -o /tmp/minidash_test_attitude test_attitude.cpp ../../src/app/attitude.cpp ../../src/app/accel_cal.cpp
/tmp/minidash_test_attitude
# The capture line format against its parser (tools/capture_parse.py; standard library only).
clang++ -std=c++17 -O1 -Wall -o /tmp/minidash_test_capture test_capture.cpp
rm -rf /tmp/minidash_capture_test && mkdir -p /tmp/minidash_capture_test
/tmp/minidash_test_capture /tmp/minidash_capture_test/drive.log
python3 ../../tools/capture_parse.py /tmp/minidash_capture_test/drive.log /tmp/minidash_capture_test/out > /tmp/minidash_capture_test/summary.txt || [ $? -eq 3 ]
python3 - <<'PY'
import csv, sys
s = open("/tmp/minidash_capture_test/summary.txt").read()
print(s)
rows = lambda t: list(csv.DictReader(open(f"/tmp/minidash_capture_test/out/{t}.csv")))
checks = [
    ("4000 IMU samples", len(rows("I")) == 4000),
    ("199 GPS reads (one missed)", len(rows("G")) == 199),
    ("time runs on through the 32-bit wrap", abs(float(rows("I")[-1]["t_s"]) - float(rows("I")[0]["t_s"]) - 19.995) < 1e-4),
    ("IMU rate 200 Hz", "200.0 Hz mean" in s),
    ("negative values survive", rows("I")[0]["ay_ug"] == "-25000"),
    ("the missed and the repeated epoch each show as a 200 ms step", "200: 2" in s),
    ("the repeated epoch is counted", "1 repeated epochs" in s),
    ("the late GPS arrival shows in the jitter", "max 30.0 ms" in s),
    ("the dropped count is reported and flagged", "ring full): 7" in s and "INCOMPLETE" in s),
    ("the cut-off line and the stray tilde are skipped, not parsed", "2 malformed line(s)" in s),
    ("the candidate run is listed", "candidate 0-60: 3.912" in s),
    ("the OBD identifier is summarised", "ECU 07 DD68: 40 asked" in s and "unanswered 1" in s),
    ("wheels against GPS", "wheels minus GPS above 20 km/h: median +1.00" in s),
    ("stack margin", "loop task: least free 5120" in s),
]
bad = [n for n, ok in checks if not ok]
print(f"capture format and parser: {len(checks)} checks, {len(bad)} failed")
for n in bad: print("FAIL:", n)
print("RESULT:", "FAIL" if bad else "PASS")
sys.exit(1 if bad else 0)
PY
