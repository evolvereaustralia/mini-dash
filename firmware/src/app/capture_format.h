// The capture record and its one-line text form. Plain C++ with no Arduino, so the line format
// is tested natively against tools/capture_parse.py (test/native/test_capture.cpp).
//
// A line is "~<type>,<tUs>,<a>,<b>,<v0>,<v1>,<v2>,<v3>,<v4>,<v5>\n": '~' marks a capture line
// among the ordinary serial output, type is one letter, tUs is micros() when the thing happened
// (32-bit: it wraps every 71.6 minutes and the parser unwraps it), and the rest are integers
// whose meaning depends on the type (capture.h has the table).
#pragma once

#include <stdint.h>
#include <stdio.h>

namespace Capture {

struct Rec {
  uint32_t tUs;
  uint8_t type, a;
  uint16_t b;
  int32_t v[6];
};
static_assert(sizeof(Rec) == 32, "capture record is 32 bytes");

constexpr size_t CAP_LINE_MAX = 112;  // '~', type, ten numbers of at most 11 characters, nine commas, newline, NUL

// Writes the line (with its newline) and returns its length; 0 if it does not fit.
inline size_t formatLine(const Rec &r, char *out, size_t n) {
  int len = snprintf(out, n, "~%c,%lu,%u,%u,%ld,%ld,%ld,%ld,%ld,%ld\n", (char)r.type, (unsigned long)r.tUs, (unsigned)r.a, (unsigned)r.b,
                     (long)r.v[0], (long)r.v[1], (long)r.v[2], (long)r.v[3], (long)r.v[4], (long)r.v[5]);
  return (len > 0 && (size_t)len < n) ? (size_t)len : 0;
}

}  // namespace Capture
