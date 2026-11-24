// LVGL 8.4 on the 4.3C: the two PSRAM framebuffers of the RGB panel in direct mode (Waveshare's
// tear-avoidance mode 3), flush waits for the bounce-buffer frame-finish event, GT911 through
// the I2C mutex as the pointer, a 2 ms esp_timer tick, and the LVGL task pinned to core 1
// (CLAUDE.md section 5: core 1 runs LVGL and nothing else heavy).
#pragma once

#include "esp_lcd_panel_ops.h"
#include "lvgl.h"

namespace LvglPort {

// Initialise LVGL, the display driver, the touch input and the LVGL task. The panel must
// already be running (DisplayRgb::begin()). touchOk says whether TouchGt911::begin() worked.
bool begin(esp_lcd_panel_handle_t panel, bool touchOk);

// LVGL is not thread safe: take this before touching any lv_ object from another task.
bool lock(int timeoutMs = -1);
void unlock();

// Frames flushed since boot, for the diagnostics page.
uint32_t frameCount();
void touchStats(uint32_t *maxMs, uint32_t *avgMs, uint32_t *fails, uint32_t *calls);
void setTouchBypass(bool on);  // diagnostics: skip the GT911 read
void logNextFrameAreas();      // diagnostics: print the next frame's invalidated areas
// Flush and render statistics for the diagnostics line; resets the handler maximum.
void stats(uint32_t *flushes, uint32_t *waitTimeouts, uint32_t *waitTotalMs, uint32_t *handlerMaxMs, uint32_t *handlerAvgMs);

}  // namespace LvglPort

// RAII guard for LvglPort::lock.
class LvLock {
 public:
  explicit LvLock(int timeoutMs = -1) : held_(LvglPort::lock(timeoutMs)) {}
  ~LvLock() { if (held_) LvglPort::unlock(); }
  bool ok() const { return held_; }
  LvLock(const LvLock &) = delete;
  LvLock &operator=(const LvLock &) = delete;

 private:
  bool held_;
};
