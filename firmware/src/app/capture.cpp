#include "capture.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_heap_caps.h"

namespace {
constexpr uint32_t RING = 16384;  // records: 512 KB of PSRAM, over a minute at the full rate
Capture::Rec *g_ring = nullptr;
volatile uint32_t g_head = 0, g_tail = 0;  // head: next write (producers), tail: next read (loop task)
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool g_streaming = false, g_mode = true;
volatile uint32_t g_dropped = 0;
uint32_t g_written = 0, g_droppedReported = 0;
uint32_t g_lastStackMs[4] = {};
}  // namespace

namespace Capture {

bool begin() {
  g_ring = static_cast<Rec *>(heap_caps_malloc(RING * sizeof(Rec), MALLOC_CAP_SPIRAM));
  if (!g_ring) Serial.println("capture: no memory for the ring, capture is unavailable this boot");
  return g_ring != nullptr;
}

void setMode(bool on) { g_mode = on; }
bool mode() { return g_mode; }

void setStreaming(bool on) {
  if (!g_ring) on = false;
  if (on && !g_streaming) {  // a fresh stream starts empty
    portENTER_CRITICAL(&g_mux);
    g_tail = g_head;
    portEXIT_CRITICAL(&g_mux);
  }
  g_streaming = on;
}
bool streaming() { return g_streaming; }

void pushAt(uint32_t tUs, uint8_t type, uint8_t a, uint16_t b, int32_t v0, int32_t v1, int32_t v2, int32_t v3, int32_t v4, int32_t v5) {
  if (!g_streaming) return;
  Rec r = {tUs, type, a, b, {v0, v1, v2, v3, v4, v5}};
  portENTER_CRITICAL(&g_mux);
  uint32_t next = (g_head + 1) % RING;
  if (next == g_tail) {
    g_dropped = g_dropped + 1;  // full: the newest record is the one lost, and the loss is counted
  } else {
    g_ring[g_head] = r;
    g_head = next;
  }
  portEXIT_CRITICAL(&g_mux);
}

void push(uint8_t type, uint8_t a, uint16_t b, int32_t v0, int32_t v1, int32_t v2, int32_t v3, int32_t v4, int32_t v5) {
  pushAt(micros(), type, a, b, v0, v1, v2, v3, v4, v5);
}

void stackMark(uint8_t task) {
  if (!g_streaming || task >= 4) return;
  uint32_t now = millis();
  if (g_lastStackMs[task] && now - g_lastStackMs[task] < 10000) return;
  g_lastStackMs[task] = now;
  push('T', task, 0, (int32_t)uxTaskGetStackHighWaterMark(nullptr), (int32_t)ESP.getFreeHeap(), (int32_t)ESP.getMinFreeHeap(),
       (int32_t)ESP.getFreePsram());
}

void drain() {
  if (!g_ring) return;
  static char block[2048];
  for (int pass = 0; pass < 8; pass++) {  // at most 16 KB a call, so the loop task keeps turning
    int room = Serial.availableForWrite();
    if (room > (int)sizeof(block)) room = sizeof(block);
    size_t used = 0;
    if (g_dropped != g_droppedReported && room >= (int)CAP_LINE_MAX) {
      g_droppedReported = g_dropped;
      Rec d = {micros(), 'D', 0, 0, {(int32_t)g_droppedReported, 0, 0, 0, 0, 0}};
      used += formatLine(d, block + used, sizeof(block) - used);
    }
    while (used + CAP_LINE_MAX <= (size_t)room) {
      Rec r;
      bool have = false;
      portENTER_CRITICAL(&g_mux);
      if (g_tail != g_head) {
        r = g_ring[g_tail];
        g_tail = (g_tail + 1) % RING;
        have = true;
      }
      portEXIT_CRITICAL(&g_mux);
      if (!have) break;
      used += formatLine(r, block + used, sizeof(block) - used);
      g_written++;
    }
    if (!used) return;
    Serial.write(reinterpret_cast<const uint8_t *>(block), used);
    if (g_tail == g_head) return;
  }
}

uint32_t dropped() { return g_dropped; }
uint32_t written() { return g_written; }

}  // namespace Capture
