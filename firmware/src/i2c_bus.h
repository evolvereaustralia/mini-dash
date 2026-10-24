// The one I2C bus mutex (CLAUDE.md section 2): every I2C access, including LVGL's touch
// read, the IO expander and the RTC, goes through I2cBus with a bounded hold time. No driver
// may touch Wire outside a held I2cLock.
#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace I2cBus {

// Initialise Wire on the board pins and create the mutex. Call once from setup().
void begin();

// Take the bus. Returns false if it could not be had within timeoutMs.
bool lock(uint32_t timeoutMs);
void unlock();

// Longest a single holder may keep the bus before a warning is logged. The warning is a
// diagnostic for bring-up; it does not break the hold.
constexpr uint32_t MAX_HOLD_MS = 25;

// Number of times a holder exceeded MAX_HOLD_MS, and the longest hold seen, for the log.
uint32_t overHoldCount();
uint32_t longestHoldMs();
// Zero the hold statistics, so the steady-state figures are not dominated by the one-off
// sensor initialisation (GPS begin alone holds the bus for about 250 ms).
void resetStats();

// Scan 0x08..0x77 while holding the bus. Fills found[] with 7-bit addresses, returns count.
size_t scan(uint8_t *found, size_t maxFound);

}  // namespace I2cBus

// RAII guard. Construct, test ok, use Wire, let it fall out of scope.
class I2cLock {
 public:
  explicit I2cLock(uint32_t timeoutMs = 50, const char *who = "?");
  ~I2cLock();
  bool ok() const { return held_; }
  I2cLock(const I2cLock &) = delete;
  I2cLock &operator=(const I2cLock &) = delete;

 private:
  bool held_;
  uint32_t takenAt_;
  const char *who_;
};
// Who held the bus longest, for the diagnostics line; and a mute for the hold warnings while
// the serial port carries a screenshot dump (other tasks' prints interleave into it).
namespace I2cBus {
const char *longestHolder();
void setQuiet(bool quiet);
}
