#!/bin/sh
# Native test of the attitude filter and speed fusion (no PlatformIO, no board).
set -e
cd "$(dirname "$0")"
clang++ -std=c++17 -O1 -Wall -o /tmp/minidash_test_fusion test_fusion.cpp ../../src/app/attitude.cpp ../../src/app/speed_fusion.cpp
/tmp/minidash_test_fusion
