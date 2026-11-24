#include "i2c_bus.h"

#include "board_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {
SemaphoreHandle_t g_mutex = nullptr;
TaskHandle_t g_holder = nullptr;
uint32_t g_nestedAttempts = 0;
uint32_t g_overHold = 0;     // written only by lock holders, read for the log
uint32_t g_longestHold = 0;
const char *g_longestWho = "-";
volatile bool g_quiet = false;
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
  if (g_holder == xTaskGetCurrentTaskHandle()) {
    // The mutex is deliberately not recursive: a nested take from the same task would wait
    // the whole timeout and fail (seen with the RTC write inside the GPS poll, 24 Oct 2026).
    g_nestedAttempts++;
    log_e("I2C lock taken again by its own holder; fix the caller");
    return false;
  }
  if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(timeoutMs)) != pdTRUE) return false;
  g_holder = xTaskGetCurrentTaskHandle();
  return true;
}

void unlock() {
  if (g_mutex == nullptr) return;
  g_holder = nullptr;
  xSemaphoreGive(g_mutex);
}

uint32_t nestedAttempts() { return g_nestedAttempts; }

uint32_t overHoldCount() { return g_overHold; }
uint32_t longestHoldMs() { return g_longestHold; }

void resetStats() {
  g_overHold = 0;
  g_longestHold = 0;
  g_longestWho = "-";
}

const char *longestHolder() { return g_longestWho; }
void setQuiet(bool quiet) { g_quiet = quiet; }

size_t scan(uint8_t *found, size_t maxFound) {
  size_t n = 0;
  I2cLock guard(200, "scan");
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

I2cLock::I2cLock(uint32_t timeoutMs, const char *who) : held_(I2cBus::lock(timeoutMs)), takenAt_(millis()), who_(who) {}

I2cLock::~I2cLock() {
  if (!held_) return;
  uint32_t held = millis() - takenAt_;
  if (held > g_longestHold) {
    g_longestHold = held;
    g_longestWho = who_;
  }
  if (held > I2cBus::MAX_HOLD_MS) {
    g_overHold++;
    if (!g_quiet) log_w("I2C held %lu ms by %s (limit %lu)", (unsigned long)held, who_, (unsigned long)I2cBus::MAX_HOLD_MS);
  }
  I2cBus::unlock();
}
