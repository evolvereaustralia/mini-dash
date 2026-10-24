// Mini Dash firmware (phase 3b onward). Boot order (CLAUDE.md section 4 and 5): serial, I2C,
// expander, panel, touch, LVGL and the splash first, so the screen is on within a second;
// then the sensor task on core 0 and the demo replay. LVGL runs on core 1.
//
// Serial keys: d = demo on/off, 1..9 and 0 = backlight 10..100 %, p = screenshot (base64
// RGB565, decode with tools/screenshot.py), n = next tab, D R Y E B = a page, x = settings, i = I2C scan, r = reboot.

#include <Arduino.h>

#include "esp_heap_caps.h"

#include "app/demo_source.h"
#include "app/lvgl_port.h"
#include "app/sensors.h"
#include "app/touch_gt911.h"
#include "app/trip_engine.h"
#include "board_config.h"
#include "display_rgb.h"
#include "i2c_bus.h"
#include "io_expander.h"
#include "ui/theme.h"
#include "ui/ui.h"
#include "vehicle_state.h"

namespace {
constexpr const char *VERSION = "0.3b";
TripEngine g_engine;
esp_lcd_panel_handle_t g_panel = nullptr;
bool g_touchOk = false;

void uiSetDemo(bool on) { Demo::setEnabled(on); }
void uiSetNight(int mode) { Sensors::setNightMode(mode); }
void uiCalibrate() { Sensors::calibrateMounting(); }
void uiResetTrip() {
  VsLock lk;
  if (lk.ok()) {
    g_engine.reset(millis());
    g_engine.publish(Vs::raw(), nullptr);
  }
}
void uiResetPeaks() {
  VsLock lk;
  if (lk.ok()) {
    g_engine.resetSessionPeaks();
    g_engine.publish(Vs::raw(), nullptr);
  }
}
void uiClearRecords() {
  VsLock lk;
  if (lk.ok()) {
    g_engine.setLifetime(RecordSet());
    g_engine.reset(millis());
    g_engine.publish(Vs::raw(), nullptr);
  }
}

// Screenshot: snapshot the active screen into a PSRAM buffer and dump it as base64 RGB565 so
// the Mac side (tools/screenshot.py) can save a PNG. LVGL's own pool is far too small for it.
void screenshot() {
  static const char *B64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  static uint8_t *buf = nullptr;
  static lv_img_dsc_t dsc;
  uint32_t need;
  {
    LvLock lk;
    need = lv_snapshot_buf_size_needed(lv_scr_act(), LV_IMG_CF_TRUE_COLOR);
    if (buf == nullptr) buf = static_cast<uint8_t *>(heap_caps_malloc(need, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (buf == nullptr || lv_snapshot_take_to_buf(lv_scr_act(), LV_IMG_CF_TRUE_COLOR, &dsc, buf, need) != LV_RES_OK) {
      Serial.println("snapshot failed");
      return;
    }
  }
  const uint8_t *src = dsc.data;
  size_t n = dsc.data_size;
  I2cBus::setQuiet(true);  // keep other tasks' warnings out of the dump
  Serial.printf("SNAP %d %d %u\n", dsc.header.w, dsc.header.h, (unsigned)n);
  char line[77];
  int col = 0;
  for (size_t i = 0; i < n; i += 3) {
    uint32_t v = (uint32_t)src[i] << 16 | (i + 1 < n ? src[i + 1] : 0) << 8 | (i + 2 < n ? src[i + 2] : 0);
    line[col++] = B64[(v >> 18) & 63];
    line[col++] = B64[(v >> 12) & 63];
    line[col++] = i + 1 < n ? B64[(v >> 6) & 63] : '=';
    line[col++] = i + 2 < n ? B64[v & 63] : '=';
    if (col >= 76) {
      line[col] = 0;
      Serial.println(line);
      col = 0;
    }
  }
  if (col) {
    line[col] = 0;
    Serial.println(line);
  }
  Serial.println("ENDSNAP");
  I2cBus::setQuiet(false);
}

void handleSerial() {
  while (Serial.available()) {
    int ch = Serial.read();
    if (ch >= '0' && ch <= '9') {
      uint8_t pct = ch == '0' ? 100 : (ch - '0') * 10;
      Sensors::setNightMode(0);
      Sensors::setBrightness(pct);
      Serial.printf("backlight %u%%\n", pct);
    } else if (ch == 'd') {
      Demo::setEnabled(!Demo::enabled());
      Serial.printf("demo %s\n", Demo::enabled() ? "on" : "off");
    } else if (ch == 'p') {
      screenshot();
    } else if (ch == 'i') {
      uint8_t found[32];
      size_t n = I2cBus::scan(found, sizeof(found));
      Serial.printf("I2C: %u devices:", (unsigned)n);
      for (size_t i = 0; i < n; i++) Serial.printf(" 0x%02X", found[i]);
      Serial.println();
    } else if (ch == 'n' || ch == 'x') {
      LvLock lk;
      if (ch == 'x') Ui::setPage(Ui::page() == Ui::PAGE_SETTINGS ? Ui::PAGE_DRIVE : Ui::PAGE_SETTINGS);
      else Ui::setPage((Ui::Page)((Ui::page() >= Ui::PAGE_COUNT ? 0 : Ui::page() + 1) % Ui::PAGE_COUNT));
    } else if (ch == 'D' || ch == 'R' || ch == 'Y' || ch == 'E' || ch == 'B') {
      LvLock lk;
      Ui::setPage(ch == 'D' ? Ui::PAGE_DRIVE : ch == 'R' ? Ui::PAGE_TRIP : ch == 'Y' ? Ui::PAGE_DYNAMICS
                  : ch == 'E' ? Ui::PAGE_ENERGY : Ui::PAGE_BATTERY);
    } else if (ch == 'a') {
      LvglPort::logNextFrameAreas();
    } else if (ch == 'T') {
      static bool bypass = false;
      bypass = !bypass;
      LvglPort::setTouchBypass(bypass);
      Serial.printf("touch bypass %s\n", bypass ? "on" : "off");
    } else if (ch == 'r') {
      ESP.restart();
    }
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  Vs::begin();
  I2cBus::begin();
  // The board controller takes a moment after power-on; bring-up only ever reached it after
  // the scan. Retry briefly rather than wait a fixed time.
  bool expander = false;
  for (int i = 0; i < 10 && !expander; i++) {
    expander = IoExpander::begin();
    if (!expander) delay(30);
  }
  g_panel = DisplayRgb::begin();
  if (g_panel) DisplayRgb::fill(0x0000);
  if (expander) {
    IoExpander::setBacklightEnable(true);
    IoExpander::setBacklightPercent(100);
  }
  g_touchOk = TouchGt911::begin();
  bool lv = LvglPort::begin(g_panel, g_touchOk);
  if (lv) {
    LvLock lk;
    Theme::init();
    Ui::Callbacks cb;
    cb.setDemo = uiSetDemo;
    cb.setNightMode = uiSetNight;
    cb.calibrateMounting = uiCalibrate;
    cb.resetTrip = uiResetTrip;
    cb.resetPeaks = uiResetPeaks;
    cb.clearRecords = uiClearRecords;
    Ui::begin(cb, VERSION);
  }
  Serial.printf("\nmini-dash %s, built %s %s\n", VERSION, __DATE__, __TIME__);
  Serial.printf("expander %s, panel %s, touch %s (%s), lvgl %s, splash at %lu ms\n", expander ? "ok" : "NO",
                g_panel ? "ok" : "NO", g_touchOk ? "ok" : "NO", TouchGt911::productId(), lv ? "ok" : "NO",
                (unsigned long)(millis() - t0));
  {
    VsLock lk;
    if (lk.ok()) {
      Vs::raw().diag.touchOk = g_touchOk;
      strncpy(Vs::raw().diag.touchId, TouchGt911::productId(), sizeof(Vs::raw().diag.touchId) - 1);
    }
  }

  // Sensors and the trip engine come up behind the splash.
  g_engine.reset(millis());
  Sensors::start(&g_engine);
  {
    LvLock lk;
    Ui::setBootStatus(Sensors::gpsOk() ? "GPS acquiring" : "GPS absent", "OBD off (phase 3c)",
                      Sensors::imuOk() ? "IMU ready" : "IMU absent");
  }
  Demo::start(&g_engine);
  Demo::setEnabled(true);  // bench default while phase 3b is under way
  Serial.println("keys: d demo, 1..9/0 backlight, p screenshot, n next tab, D/R/Y/E/B page, x settings, i scan, r reboot");
}

void loop() {
  handleSerial();
  static uint32_t last = 0;
  if (millis() - last >= 5000) {
    last = millis();
    VehicleState vs;
    Vs::snapshot(vs);
    uint32_t fl, to, wt, hm, ha, tk, pl, tmx, tav, tf, tc;
    LvglPort::stats(&fl, &to, &wt, &hm, &ha);
    LvglPort::touchStats(&tmx, &tav, &tf, &tc);
    Ui::timing(&tk, &pl);
    Serial.printf("[%6lu] page %d demo %d spd %.0f G %+.2f/%+.2f kW %.1f gps %d/%d imu %.0f Hz | lvgl %lu frames %lu flushes, wait %lu ms, %lu timeouts, handler max %lu avg %lu ms, tick max %lu ms, plot max %lu ms, touch max %lu avg %lu ms %lu fails/%lu | heap %lu psram %lu i2c %lu ms by %s\n",
                  (unsigned long)(millis() / 1000), (int)Ui::page(), vs.demo, vs.disp.speedKph, vs.motion.latG,
                  vs.motion.lonG, vs.car.powerKw, vs.gps.fixType, vs.gps.sats, vs.imu.rateHz,
                  (unsigned long)LvglPort::frameCount(), (unsigned long)fl, (unsigned long)wt, (unsigned long)to,
                  (unsigned long)hm, (unsigned long)ha, (unsigned long)tk, (unsigned long)pl, (unsigned long)tmx, (unsigned long)tav, (unsigned long)tf, (unsigned long)tc, (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getFreePsram(),
                  (unsigned long)I2cBus::longestHoldMs(), I2cBus::longestHolder());
  }
  delay(10);
}
