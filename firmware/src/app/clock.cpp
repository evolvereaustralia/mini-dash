#include "clock.h"

#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>

#include "board_config.h"
#include "i2c_bus.h"

namespace {
uint32_t g_baseUtc = 0;    // UTC at g_baseMs
uint32_t g_baseMs = 0;
bool g_valid = false;
bool g_setFromGps = false;

uint8_t bcd2bin(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }
uint8_t bin2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

// PCF85063A: seconds at 0x04 (bit 7 = oscillator stop / time invalid), minutes 0x05, hours
// 0x06, days 0x07, weekdays 0x08, months 0x09, years 0x0A (00..99 = 2000..2099).
bool rtcRead(uint32_t *utc) {
  I2cLock guard(50, "rtc");
  if (!guard.ok()) return false;
  Wire.beginTransmission(I2C_ADDR_PCF85063A);
  Wire.write(0x04);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(I2C_ADDR_PCF85063A, (uint8_t)7) != 7) return false;
  uint8_t b[7];
  for (int i = 0; i < 7; i++) b[i] = Wire.read();
  if (b[0] & 0x80) return false;  // oscillator stopped: time not valid
  uint8_t sec = bcd2bin(b[0] & 0x7F), min = bcd2bin(b[1] & 0x7F), hour = bcd2bin(b[2] & 0x3F);
  uint8_t day = bcd2bin(b[3] & 0x3F), mon = bcd2bin(b[5] & 0x1F);
  int year = 2000 + bcd2bin(b[6]);
  if (mon < 1 || mon > 12 || day < 1 || day > 31 || hour > 23 || min > 59 || sec > 59) return false;
  *utc = Clock::daysFromCivil(year, mon, day) * 86400UL + hour * 3600UL + min * 60UL + sec;
  return true;
}

bool rtcWrite(uint32_t utc) {
  int y;
  unsigned m, d;
  Clock::civilFromDays(utc / 86400UL, &y, &m, &d);
  uint32_t s = utc % 86400UL;
  I2cLock guard(50, "rtc");
  if (!guard.ok()) return false;
  Wire.beginTransmission(I2C_ADDR_PCF85063A);
  Wire.write(0x04);
  Wire.write(bin2bcd(s % 60));  // also clears the oscillator-stop flag
  Wire.write(bin2bcd((s / 60) % 60));
  Wire.write(bin2bcd(s / 3600));
  Wire.write(bin2bcd(d));
  Wire.write(Clock::weekday(utc / 86400UL));
  Wire.write(bin2bcd(m));
  Wire.write(bin2bcd(y - 2000));
  return Wire.endTransmission() == 0;
}
}  // namespace

namespace Clock {

uint32_t daysFromCivil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return (uint32_t)(era * 146097 + (int)doe - 719468);
}

void civilFromDays(uint32_t z, int *y, unsigned *m, unsigned *d) {
  int64_t zz = (int64_t)z + 719468;
  const int64_t era = (zz >= 0 ? zz : zz - 146096) / 146097;
  const unsigned doe = (unsigned)(zz - era * 146097);
  const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const int yy = (int)yoe + (int)era * 400;
  const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const unsigned mp = (5 * doy + 2) / 153;
  *d = doy - (153 * mp + 2) / 5 + 1;
  *m = mp + (mp < 10 ? 3 : -9);
  *y = yy + (*m <= 2);
}

int weekday(uint32_t days) { return (int)((days + 4) % 7); }  // 1970-01-01 was a Thursday

int localOffsetSeconds(uint32_t utc) {
  // Melbourne: AEST UTC+10; AEDT UTC+11 from the first Sunday in October 02:00 AEST to the
  // first Sunday in April 03:00 AEDT (02:00 AEST). Evaluate in AEST.
  int y;
  unsigned m, d;
  uint32_t aest = utc + 36000;
  civilFromDays(aest / 86400UL, &y, &m, &d);
  auto firstSunday = [](int year, unsigned month) {
    uint32_t first = daysFromCivil(year, month, 1);
    int wd = weekday(first);
    return first + (wd == 0 ? 0 : 7 - wd);
  };
  uint32_t dstStart = firstSunday(y, 10) * 86400UL + 2 * 3600UL;  // in AEST seconds
  uint32_t dstEnd = firstSunday(y, 4) * 86400UL + 2 * 3600UL;
  bool dst = (aest >= dstStart) || (aest < dstEnd);
  return dst ? 39600 : 36000;
}

uint32_t localDayNumber(uint32_t utc) { return (utc + localOffsetSeconds(utc)) / 86400UL; }

void localHM(uint32_t utc, uint8_t *hour, uint8_t *minute) {
  uint32_t l = (utc + localOffsetSeconds(utc)) % 86400UL;
  *hour = l / 3600;
  *minute = (l / 60) % 60;
}

void localDate(uint32_t utc, uint16_t *year, uint8_t *month, uint8_t *day) {
  int y;
  unsigned m, d;
  civilFromDays((utc + localOffsetSeconds(utc)) / 86400UL, &y, &m, &d);
  *year = y;
  *month = m;
  *day = d;
}

void formatHM(uint32_t utc, char *out, size_t n) {
  uint8_t h, mi;
  localHM(utc, &h, &mi);
  snprintf(out, n, "%02u:%02u", h, mi);
}

void formatDayMonth(uint32_t utc, char *out, size_t n) {
  static const char *MON[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  uint16_t y;
  uint8_t m, d;
  localDate(utc, &y, &m, &d);
  snprintf(out, n, "%u %s", d, MON[(m - 1) % 12]);
}

bool begin() {
  uint32_t utc;
  if (rtcRead(&utc) && utc > 1700000000UL) {  // after Nov 2023: a set clock, not the 2000-01-01 default
    g_baseUtc = utc;
    g_baseMs = millis();
    g_valid = true;
    return true;
  }
  return false;
}

uint32_t nowUtc() {
  if (!g_valid) return 0;
  return g_baseUtc + (millis() - g_baseMs) / 1000UL;
}

bool valid() { return g_valid; }
bool setFromGpsDone() { return g_setFromGps; }

void setFromGps(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second) {
  if (year < 2024 || month < 1 || month > 12 || day < 1 || day > 31) return;
  uint32_t utc = daysFromCivil(year, month, day) * 86400UL + hour * 3600UL + minute * 60UL + second;
  g_baseUtc = utc;
  g_baseMs = millis();
  g_valid = true;
  if (!g_setFromGps) {
    g_setFromGps = rtcWrite(utc);
    if (g_setFromGps) Serial.printf("clock: RTC set from GPS, %lu\n", (unsigned long)utc);
  }
}

}  // namespace Clock
