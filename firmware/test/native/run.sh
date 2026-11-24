#!/bin/sh
# Native tests (no PlatformIO, no board): the attitude filter, speed fusion and accelerometer
# calibration against a synthetic drive, and the OBD protocol layer against recorded replies.
set -e
cd "$(dirname "$0")"
clang++ -std=c++17 -O1 -Wall -o /tmp/minidash_test_fusion test_fusion.cpp ../../src/app/accel_cal.cpp ../../src/app/attitude.cpp ../../src/app/speed_fusion.cpp
/tmp/minidash_test_fusion
clang++ -std=c++17 -O1 -Wall -I../../include -o /tmp/minidash_test_obd test_obd.cpp ../../src/app/obd_proto.cpp ../../src/app/obd_decode.cpp
/tmp/minidash_test_obd
