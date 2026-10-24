#include "lvgl_port.h"

#include <Arduino.h>
#include <string.h>

#include "esp_heap_caps.h"

#include "board_config.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "touch_gt911.h"

namespace {
constexpr uint32_t TICK_MS = 2;
constexpr uint32_t TASK_STACK = 8 * 1024;
constexpr UBaseType_t TASK_PRIORITY = 2;
constexpr BaseType_t TASK_CORE = 1;

SemaphoreHandle_t g_mutex = nullptr;
TaskHandle_t g_task = nullptr;
uint32_t g_frames = 0;
uint32_t g_waitTimeouts = 0, g_waitTotalMs = 0, g_handlerMaxMs = 0, g_flushes = 0, g_handlerTotalMs = 0, g_handlerCalls = 0;
uint32_t g_touchMaxMs = 0, g_touchTotalMs = 0, g_touchCalls = 0, g_touchFails = 0;
volatile bool g_touchBypass = false;
volatile bool g_logAreas = false;
lv_disp_draw_buf_t g_drawBuf;
lv_disp_drv_t g_dispDrv;
lv_indev_drv_t g_indevDrv;

// Rendering strategy. LVGL renders dirty areas in partial mode into a draw buffer in internal
// RAM (anti-aliased blending straight into PSRAM cost about 150 ms a frame). Each flushed
// chunk is copied row by row into the back framebuffer; on the last chunk the panel switches
// to that buffer at the next frame, we wait for the switch, then copy the same rows into the
// new back buffer so the two framebuffers stay identical. No tearing, no full-screen redraw.
constexpr int DRAW_ROWS = 40;
lv_color_t *g_drawBufMem = nullptr;
void *g_fb[2] = {nullptr, nullptr};
int g_back = 1;
struct Area { int16_t x1, y1, x2, y2; };
constexpr int MAX_DIRTY = 48;
Area g_dirty[MAX_DIRTY];
int g_nDirty = 0;
bool g_dirtyOverflow = false;

// Called from the RGB panel ISR when a frame buffer has been fully sent, so the panel is
// now scanning the other one.
bool IRAM_ATTR onFrameDone(esp_lcd_panel_handle_t, const esp_lcd_rgb_panel_event_data_t *, void *) {
  BaseType_t woken = pdFALSE;
  if (g_task) vTaskNotifyGiveFromISR(g_task, &woken);
  return woken == pdTRUE;
}

inline void copyRows(void *dst, const void *src, int x1, int y1, int x2, int y2, int srcStride) {
  // src is a chunk whose row 0 is y1 and column 0 is x1 (stride srcStride px); dst is a frame buffer.
  const int w = x2 - x1 + 1;
  for (int y = y1; y <= y2; y++) {
    uint16_t *d = static_cast<uint16_t *>(dst) + y * LCD_H_RES + x1;
    const uint16_t *sp = static_cast<const uint16_t *>(src) + (y - y1) * srcStride;
    memcpy(d, sp, w * sizeof(uint16_t));
  }
}
inline void copyRowsFb(void *dst, const void *src, int x1, int y1, int x2, int y2) {
  const int w = x2 - x1 + 1;
  for (int y = y1; y <= y2; y++) {
    memcpy(static_cast<uint16_t *>(dst) + y * LCD_H_RES + x1, static_cast<const uint16_t *>(src) + y * LCD_H_RES + x1,
           w * sizeof(uint16_t));
  }
}

void flushCb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *colorMap) {
  auto panel = static_cast<esp_lcd_panel_handle_t>(drv->user_data);
  if (g_logAreas) {
    lv_disp_t *d = lv_disp_get_default();
    if (g_nDirty == 0) {
      Serial.printf("frame inv areas (%d):", d->inv_p);
      for (int i = 0; i < d->inv_p; i++) {
        if (d->inv_area_joined[i]) continue;
        Serial.printf(" [%d,%d-%d,%d]", d->inv_areas[i].x1, d->inv_areas[i].y1, d->inv_areas[i].x2, d->inv_areas[i].y2);
      }
      Serial.println();
    }
    if (lv_disp_flush_is_last(drv)) g_logAreas = false;
  }
  copyRows(g_fb[g_back], colorMap, area->x1, area->y1, area->x2, area->y2, area->x2 - area->x1 + 1);
  if (g_nDirty < MAX_DIRTY) {
    g_dirty[g_nDirty++] = {(int16_t)area->x1, (int16_t)area->y1, (int16_t)area->x2, (int16_t)area->y2};
  } else {
    g_dirtyOverflow = true;
  }
  if (lv_disp_flush_is_last(drv)) {
    esp_lcd_panel_draw_bitmap(panel, 0, 0, LCD_H_RES, LCD_V_RES, g_fb[g_back]);  // switch, no copy
    ulTaskNotifyValueClear(nullptr, ULONG_MAX);
    uint32_t t0 = millis();
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) == 0) g_waitTimeouts++;
    g_waitTotalMs += millis() - t0;
    int front = g_back;
    g_back ^= 1;
    if (g_dirtyOverflow) {
      memcpy(g_fb[g_back], g_fb[front], LCD_H_RES * LCD_V_RES * sizeof(uint16_t));
    } else {
      for (int i = 0; i < g_nDirty; i++) copyRowsFb(g_fb[g_back], g_fb[front], g_dirty[i].x1, g_dirty[i].y1, g_dirty[i].x2, g_dirty[i].y2);
    }
    g_nDirty = 0;
    g_dirtyOverflow = false;
    g_frames++;
  }
  g_flushes++;
  lv_disp_flush_ready(drv);
}

void touchReadCb(lv_indev_drv_t *, lv_indev_data_t *data) {
  static lv_point_t last = {0, 0};
  static bool lastPressed = false;
  TouchGt911::Point pts[1];
  if (g_touchBypass) {
    data->point = last;
    data->state = LV_INDEV_STATE_RELEASED;
    return;
  }
  uint32_t t0 = millis();
  int n = TouchGt911::read(pts, 1);
  uint32_t dt = millis() - t0;
  g_touchTotalMs += dt;
  g_touchCalls++;
  if (dt > g_touchMaxMs) g_touchMaxMs = dt;
  if (n < 0) g_touchFails++;
  if (n < 0) {  // bus busy or error: report the previous state unchanged
    data->point = last;
    data->state = lastPressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    return;
  }
  if (n > 0) {
    last.x = pts[0].x;
    last.y = pts[0].y;
    lastPressed = true;
  } else {
    lastPressed = false;
  }
  data->point = last;
  data->state = lastPressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

void tickCb(void *) { lv_tick_inc(TICK_MS); }

void lvglTask(void *) {
  uint32_t delayMs = 10;
  for (;;) {
    if (LvglPort::lock(-1)) {
      uint32_t t0 = millis();
      delayMs = lv_timer_handler();
      uint32_t dt = millis() - t0;
      if (dt > g_handlerMaxMs) g_handlerMaxMs = dt;
      g_handlerTotalMs += dt;
      g_handlerCalls++;
      LvglPort::unlock();
    }
    if (delayMs > 500) delayMs = 500;
    if (delayMs < 5) delayMs = 5;
    vTaskDelay(pdMS_TO_TICKS(delayMs));
  }
}
}  // namespace

namespace LvglPort {

bool begin(esp_lcd_panel_handle_t panel, bool touchOk) {
  if (panel == nullptr) return false;
  lv_init();

  const esp_timer_create_args_t tickArgs = {.callback = &tickCb, .arg = nullptr,
                                            .dispatch_method = ESP_TIMER_TASK, .name = "lv_tick",
                                            .skip_unhandled_events = true};
  esp_timer_handle_t tick = nullptr;
  if (esp_timer_create(&tickArgs, &tick) != ESP_OK) return false;
  esp_timer_start_periodic(tick, TICK_MS * 1000);

  if (esp_lcd_rgb_panel_get_frame_buffer(panel, 2, &g_fb[0], &g_fb[1]) != ESP_OK) return false;
  memset(g_fb[0], 0, LCD_H_RES * LCD_V_RES * sizeof(uint16_t));
  memset(g_fb[1], 0, LCD_H_RES * LCD_V_RES * sizeof(uint16_t));
  g_back = 1;  // the panel shows buffer 0 after init
  g_drawBufMem = static_cast<lv_color_t *>(heap_caps_malloc(LCD_H_RES * DRAW_ROWS * sizeof(lv_color_t),
                                                             MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  if (g_drawBufMem == nullptr) return false;
  lv_disp_draw_buf_init(&g_drawBuf, g_drawBufMem, nullptr, LCD_H_RES * DRAW_ROWS);
  lv_disp_drv_init(&g_dispDrv);
  g_dispDrv.hor_res = LCD_H_RES;
  g_dispDrv.ver_res = LCD_V_RES;
  g_dispDrv.flush_cb = flushCb;
  g_dispDrv.draw_buf = &g_drawBuf;
  g_dispDrv.user_data = panel;
  if (lv_disp_drv_register(&g_dispDrv) == nullptr) return false;

  esp_lcd_rgb_panel_event_callbacks_t cbs = {};
  cbs.on_frame_buf_complete = onFrameDone;  // fires when a frame buffer has been fully sent
  esp_lcd_rgb_panel_register_event_callbacks(panel, &cbs, nullptr);

  if (touchOk) {
    lv_indev_drv_init(&g_indevDrv);
    g_indevDrv.type = LV_INDEV_TYPE_POINTER;
    g_indevDrv.read_cb = touchReadCb;
    lv_indev_drv_register(&g_indevDrv);
  }

  g_mutex = xSemaphoreCreateRecursiveMutex();
  if (g_mutex == nullptr) return false;
  if (xTaskCreatePinnedToCore(lvglTask, "lvgl", TASK_STACK, nullptr, TASK_PRIORITY, &g_task, TASK_CORE) != pdPASS) {
    return false;
  }
  return true;
}

bool lock(int timeoutMs) {
  if (g_mutex == nullptr) return false;
  TickType_t t = timeoutMs < 0 ? portMAX_DELAY : pdMS_TO_TICKS(timeoutMs);
  return xSemaphoreTakeRecursive(g_mutex, t) == pdTRUE;
}

void unlock() {
  if (g_mutex) xSemaphoreGiveRecursive(g_mutex);
}

uint32_t frameCount() { return g_frames; }

void touchStats(uint32_t *maxMs, uint32_t *avgMs, uint32_t *fails, uint32_t *calls) {
  *maxMs = g_touchMaxMs;
  *avgMs = g_touchCalls ? g_touchTotalMs / g_touchCalls : 0;
  *fails = g_touchFails;
  *calls = g_touchCalls;
  g_touchMaxMs = g_touchTotalMs = g_touchCalls = g_touchFails = 0;
}

void setTouchBypass(bool on) { g_touchBypass = on; }
void logNextFrameAreas() { g_logAreas = true; }

void stats(uint32_t *flushes, uint32_t *waitTimeouts, uint32_t *waitTotalMs, uint32_t *handlerMaxMs, uint32_t *handlerAvgMs) {
  *flushes = g_flushes;
  *waitTimeouts = g_waitTimeouts;
  *waitTotalMs = g_waitTotalMs;
  *handlerMaxMs = g_handlerMaxMs;
  *handlerAvgMs = g_handlerCalls ? g_handlerTotalMs / g_handlerCalls : 0;
  g_handlerMaxMs = 0;
  g_handlerTotalMs = g_handlerCalls = 0;
}

}  // namespace LvglPort
