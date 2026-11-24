// Wall clock: the board's PCF85063A RTC (I2C 0x51, CR927 cell) read at boot and set from GPS
// once a fix carries a valid date and time (CLAUDE.md section 4, header clock). Timestamps
// are stored in UTC; display and day grouping use Australia/Melbourne with the daylight
// saving rule (first Sunday of October 02:00 standard to first Sunday of April 03:00 daylight).
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace Clock {

bool begin();                 // reads the RTC; returns false if it does not answer or has no valid time
uint32_t nowUtc();            // seconds since 1970, 0 if no source yet
bool valid();                 // a GPS or RTC time has been established this boot
void setFromGps(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
bool setFromGpsDone();        // the RTC has been written from GPS this boot

// Melbourne local time.
int localOffsetSeconds(uint32_t utc);   // 36000 or 39600
uint32_t localDayNumber(uint32_t utc);  // days since 1970 in local time, for "today" grouping
void localHM(uint32_t utc, uint8_t *hour, uint8_t *minute);
void localDate(uint32_t utc, uint16_t *year, uint8_t *month, uint8_t *day);
void formatHM(uint32_t utc, char *out, size_t n);        // "14:12"
void formatDayMonth(uint32_t utc, char *out, size_t n);  // "24 Oct"

// Civil date helpers (proleptic Gregorian), public for tests.
uint32_t daysFromCivil(int y, unsigned m, unsigned d);
void civilFromDays(uint32_t z, int *y, unsigned *m, unsigned *d);
int weekday(uint32_t days);  // 0 = Sunday

}  // namespace Clock
