#include "vehicle_state.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {
SemaphoreHandle_t g_mutex = nullptr;
VehicleState g_state;
}  // namespace

namespace Vs {

void begin() {
  if (g_mutex == nullptr) g_mutex = xSemaphoreCreateRecursiveMutex();
}

bool lock(uint32_t timeoutMs) {
  if (g_mutex == nullptr) return false;
  return xSemaphoreTakeRecursive(g_mutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

void unlock() {
  if (g_mutex) xSemaphoreGiveRecursive(g_mutex);
}

VehicleState &raw() { return g_state; }

void snapshot(VehicleState &out) {
  if (lock(50)) {
    out = g_state;
    unlock();
  }
}

}  // namespace Vs
