#include "i2c_bus.h"

#include "board_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {
SemaphoreHandle_t g_mutex = nullptr;
uint32_t g_overHold = 0;     // written only by lock holders, read for the log
uint32_t g_longestHold = 0;
}  // namespace

namespace I2cBus {

void begin() {
  if (g_mutex == nullptr) {
    g_mutex = xSemaphoreCreateMutex();
  }
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);
  Wire.setTimeOut(50);  // ms, per transaction
}

bool lock(uint32_t timeoutMs) {
  if (g_mutex == nullptr) return false;
  return xSemaphoreTake(g_mutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

void unlock() {
  if (g_mutex != nullptr) xSemaphoreGive(g_mutex);
}

uint32_t overHoldCount() { return g_overHold; }
uint32_t longestHoldMs() { return g_longestHold; }

void resetStats() {
  g_overHold = 0;
  g_longestHold = 0;
}

size_t scan(uint8_t *found, size_t maxFound) {
  size_t n = 0;
  I2cLock guard(200);
  if (!guard.ok()) return 0;
  for (uint8_t addr = 0x08; addr <= 0x77 && n < maxFound; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      found[n++] = addr;
    }
  }
  return n;
}

}  // namespace I2cBus

I2cLock::I2cLock(uint32_t timeoutMs) : held_(I2cBus::lock(timeoutMs)), takenAt_(millis()) {}

I2cLock::~I2cLock() {
  if (!held_) return;
  uint32_t held = millis() - takenAt_;
  if (held > g_longestHold) g_longestHold = held;
  if (held > I2cBus::MAX_HOLD_MS) {
    g_overHold++;
    log_w("I2C held %lu ms (limit %lu)", (unsigned long)held, (unsigned long)I2cBus::MAX_HOLD_MS);
  }
  I2cBus::unlock();
}
