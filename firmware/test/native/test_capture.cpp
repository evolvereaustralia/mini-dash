// Writes a small capture log with the firmware's own line formatter, as the device would send
// it: mixed with ordinary serial output, crossing a wrap of the 32-bit microsecond clock, with
// one line cut short. run.sh then puts tools/capture_parse.py over it and checks what it finds,
// so the formatter and the parser cannot drift apart unnoticed.
#include <stdio.h>
#include <stdlib.h>

#include "../../src/app/capture_format.h"

using Capture::Rec;

static FILE *g_f;
static void put(const Rec &r) {
  char line[Capture::CAP_LINE_MAX];
  size_t n = Capture::formatLine(r, line, sizeof(line));
  if (!n) {
    printf("FAIL: a record did not fit a line\n");
    exit(1);
  }
  fwrite(line, 1, n, g_f);
}

int main(int argc, char **argv) {
  if (argc < 2) return 2;
  g_f = fopen(argv[1], "wb");
  if (!g_f) return 2;
  // the longest possible line must fit
  Rec worst = {4294967295u, 'I', 255, 65535, {-2147483647 - 1, -2147483647 - 1, -2147483647 - 1, -2147483647 - 1, -2147483647 - 1, -2147483647 - 1}};
  char line[Capture::CAP_LINE_MAX];
  if (!Capture::formatLine(worst, line, sizeof(line))) {
    printf("FAIL: the longest line does not fit CAP_LINE_MAX\n");
    return 1;
  }
  fprintf(g_f, "mode: CAPTURE (runs logged, nothing recorded or stored)\n");
  fprintf(g_f, "~K,1000,ms=1,fw=0.4,mode=capture,fwd=+Y\n");
  // 20 s starting 5 s before the microsecond clock wraps: GPS at 10 Hz, IMU at 200 Hz
  const uint32_t start = 4294967295u - 5000000u;
  uint32_t itow = 500000000;
  for (int ms = 0; ms < 20000; ms += 5) {
    uint32_t t = start + (uint32_t)ms * 1000u;  // wraps on its own
    put({t, 'I', 0, 0, {12000, -25000, 1003000, 100, -200, 300}});
    if (ms % 50 == 0) put({t, 'A', (uint8_t)(1 | 16 | 32 | 64 | 128), 12, {-50000, 89000, 2000, 13889, 89000, 1004000}});
    if (ms % 100 == 0) {
      bool repeat = ms == 3000;        // one epoch read twice
      // a repeat is the previous epoch read again, so the next read steps 200 ms; so does the one after a missed read
      if (ms != 7000 && !repeat) itow += (ms == 7100 || ms == 3100) ? 200 : 100;
      if (ms != 7000) put({t + 40000u + (ms == 12000 ? 30000u : 0u), 'G', 11, (uint16_t)(3 | 0x10 | 0x20 | (repeat ? 0x40 : 0)), {(int32_t)itow, 13889, 900, 4800, 120, 9000}});
      put({t, 'S', (uint8_t)(1 | 2 | 4 | 8 | 16 | 32), 11, {51000, 50000, 51000, 42000, 5070, ms * 14 / 10}});
    }
    if (ms % 500 == 0) put({t, 'O', 0x07, 0xDD68, {ms == 9000 ? 400 : 80, ms == 9000 ? 1 : 0, 1, 4}});
    if (ms == 2000) fprintf(g_f, "[     2] page 0 demo 0 spd 50 ~ this ordinary line happens to contain a tilde\n");
    if (ms == 6000) put({t, 'C', 0, 0, {3912, 0, 123456}});
    if (ms == 10000) put({t, 'T', 0, 0, {5120, 40000, 38000, 6000000}});
    if (ms == 15000) put({t, 'D', 0, 0, {7}});
  }
  fprintf(g_f, "~I,123456,0,0,1,2");  // the stream cut off mid-line when logging stopped
  fclose(g_f);
  printf("capture log written\n");
  return 0;
}
