// Mini Dash firmware (phase 3b onward). Boot order (CLAUDE.md section 4 and 5): serial, I2C,
// expander, panel, touch, LVGL and the splash first, so the screen is on within a second;
// then the sensor task on core 0 and the demo replay. LVGL runs on core 1.
//
// Serial keys: d = demo on/off, 1..9 and 0 = backlight 10..100 %, p = screenshot (base64
// RGB565, decode with tools/screenshot.py), n = next tab, D R Y E B = a page, x = settings, i = I2C scan, r = reboot.
// Storage: W = self-test of the safe write path, KK = format the trip partition (the only way it is
// ever formatted once it holds data), X = remove old test trips. The rest are in firmware/README.md.

#include <Arduino.h>
#include <Wire.h>
#include <string.h>

#include "esp_heap_caps.h"

#include "app/clock.h"
#include "app/demo_source.h"
#include "app/lvgl_port.h"
#include "app/obd_client.h"
#include "app/persist.h"
#include "app/sensors.h"
#include "app/storage.h"
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
volatile bool g_motionLog = false;

// Nothing in this file writes trips, records, traces or settings: app/persist.* owns those
// writes on this task, and the UI callbacks below only leave it requests.
void uiSetNight(int mode) { Sensors::setNightMode(mode); }
void uiCalibrate() { Sensors::calibrateMounting(); }  // stored by Persist once the sensor task has taken it
void uiCalibrateAccel() { Sensors::startAccelCal(); }
void uiSettingsChanged(const Ui::UiSettings &u) { Persist::requestSettings(u.mph, u.nightMode, u.demo, u.bestField); }
void uiResetPeaks() {
  VsLock lk;
  if (lk.ok()) {
    g_engine.resetSessionPeaks();
    g_engine.publish(Vs::raw());
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

// Flash-write stall test (key Z). A high-priority task on the LVGL core touches the PSRAM
// framebuffer every millisecond and records the longest gap between touches while five event
// files are written. With the prebuilt framework the cache is off for every write, so the gap
// is the write duration (tens of ms); with code executing from PSRAM it should stay near 1 ms.
// This is the bench proxy for "does a flash write stall the panel refill" since the glass
// itself cannot be read back.
// Corrected 24 Nov 2026 (review finding 10): the probe used to index 32-bit words 16 KB apart
// and so read up to 264 KB past the end of the 768,000-byte buffer. The figures taken with it
// (1.5 ms against 20 ms, docs/test-log.md) are preliminary until repeated with this version.
volatile uint32_t g_stallMaxUs = 0;
volatile bool g_stallRun = false;
void stallProbe(void *) {
  void *fb0 = nullptr, *fb1 = nullptr;
  DisplayRgb::getFrameBuffers(&fb0, &fb1);
  volatile uint16_t *p = static_cast<volatile uint16_t *>(fb0);
  const size_t pixels = (size_t)LCD_H_RES * LCD_V_RES, step = pixels / 64;
  uint32_t last = micros();
  uint32_t sink = 0;
  while (g_stallRun && p) {
    for (size_t i = 0; i < 64; i++) sink += p[i * step];  // 64 reads spread over the frame buffer, all inside it
    uint32_t now = micros();
    if (now - last > g_stallMaxUs) g_stallMaxUs = now - last;
    last = now;
    vTaskDelay(1);
  }
  (void)sink;
  g_stallRun = false;
  vTaskDelete(nullptr);
}
void stallTest() {
  void *fb0 = nullptr, *fb1 = nullptr;
  DisplayRgb::getFrameBuffers(&fb0, &fb1);
  if (!fb0) {
    Serial.println("stall test: no frame buffer, not run");
    return;
  }
  g_stallMaxUs = 0;
  g_stallRun = true;
  if (xTaskCreatePinnedToCore(stallProbe, "stall", 3072, nullptr, 5, nullptr, 1) != pdPASS) {
    g_stallRun = false;
    Serial.println("stall test: the probe task could not start, not run");
    return;
  }
  delay(300);
  uint32_t idle = g_stallMaxUs;  // baseline without writes
  g_stallMaxUs = 0;
  uint32_t writeMs = Persist::stallTestWrites();
  delay(100);
  uint32_t during = g_stallMaxUs;
  g_stallRun = false;
  delay(20);
  Serial.printf("stall test: idle max gap %lu us; during 5 event writes (%lu ms total) max gap %lu us\n",
                (unsigned long)idle, (unsigned long)writeMs, (unsigned long)during);
}

void handleSerial() {
  while (Serial.available()) {
    int ch = Serial.read();
    if (ch >= '0' && ch <= '9') {
      uint8_t pct = ch == '0' ? 100 : (ch - '0') * 10;
      Sensors::setNightMode(0);
      Sensors::setBrightness(pct);
      Serial.printf("backlight %u%%\n", pct);
    } else if (ch == 'd') {  // toggles the replay and stores it, as the Settings switch does
      Persist::requestDemo(!Demo::enabled());  // the running trip is closed properly first
      Serial.printf("demo %s requested\n", Demo::enabled() ? "off" : "on");
    } else if (ch == 'p') {
      screenshot();
    } else if (ch == 'i') {
      uint8_t found[32];
      size_t n = I2cBus::scan(found, sizeof(found));
      Serial.printf("I2C: %u devices:", (unsigned)n);
      for (size_t i = 0; i < n; i++) Serial.printf(" 0x%02X", found[i]);
      {  // raw WHO_AM_I of the IMU (0x0F, expect 0x6B) for the start-up failure of 25 Oct 2026
        I2cLock lk(50, "whoami");
        if (lk.ok()) {
          Wire.beginTransmission(I2C_ADDR_ISM330DHCX);
          Wire.write(0x0F);
          uint8_t e = Wire.endTransmission(false);
          uint8_t got = Wire.requestFrom(I2C_ADDR_ISM330DHCX, (uint8_t)1);
          int v = got ? Wire.read() : -1;
          Serial.printf(" | IMU WHO_AM_I: write %u, read %u bytes, value 0x%02X", e, got, v & 0xFF);
        }
      }
      Serial.println();
    } else if (ch == 'n' || ch == 'x') {
      LvLock lk;
      if (ch == 'x') Ui::setPage(Ui::page() == Ui::PAGE_SETTINGS ? Ui::PAGE_DRIVE : Ui::PAGE_SETTINGS);
      else Ui::setPage((Ui::Page)((Ui::page() >= Ui::PAGE_COUNT ? 0 : Ui::page() + 1) % Ui::PAGE_COUNT));
    } else if (ch == 'D' || ch == 'R' || ch == 'Y' || ch == 'E' || ch == 'B') {
      LvLock lk;
      Ui::setPage(ch == 'D' ? Ui::PAGE_DRIVE : ch == 'R' ? Ui::PAGE_TRIP : ch == 'Y' ? Ui::PAGE_DYNAMICS
                  : ch == 'E' ? Ui::PAGE_ENERGY : Ui::PAGE_BATTERY);
    } else if (ch == 'W') {  // storage self-test: one block through the safe write path, read back, removed
      bool ok = Storage::selfTest();
      Serial.printf("storage test: %s | %d trips, %lu KB used, %lu faults since boot\n", ok ? "write, sync, read back and rename ok" : "FAILED",
                    Storage::tripCount(), (unsigned long)(Storage::fsUsedBytes() / 1024), (unsigned long)Storage::faultCount());
    } else if (ch == 'X') {  // remove the test trips older firmware's self-test left in the history
      Serial.printf("storage: %d test trips removed\n", Storage::removeTestTrips());
      Persist::requestStats();
    } else if (ch == 't') {
      static bool tlog = false;
      tlog = !tlog;
      TouchGt911::setLogging(tlog);
      Serial.printf("touch logging %s\n", tlog ? "on" : "off");
    } else if (ch == 'c') {  // accelerometer offset calibration, as the Settings button
      Sensors::startAccelCal();
    } else if (ch == 'g') {  // 10 Hz readout of the motion path, for hand tests of the attitude filter
      g_motionLog = !g_motionLog;
      Serial.printf("motion log %s\n", g_motionLog ? "on: t,lat,lon,yaw,tilt,corr,bias,kph,trust,still,|a|,|w|,rest|a|" : "off");
    } else if (ch == 'Q') {  // dump the stored trace of the trip's 0-60 record as CSV
      static Storage::TraceSample tr[Storage::TRACE_MAX];
      size_t n = 0;
      uint32_t tripId = 0;
      {
        VsLock lk;
        if (lk.ok()) tripId = g_engine.tripId();
      }
      if (Storage::loadEvent(tripId, 0, tr, Storage::TRACE_MAX, &n)) {
        Serial.printf("trace trip %lu rec 0, %u samples: t10,kph10,latMg,lonMg,kw10\n", (unsigned long)tripId, (unsigned)n);
        for (size_t i = 0; i < n; i++) Serial.printf("T,%u,%d,%d,%d,%d\n", tr[i].t10, tr[i].kph10, tr[i].latMg, tr[i].lonMg, tr[i].kw10);
      } else {
        Serial.println("trace: none stored for rec 0");
      }
    } else if (ch == 'I') {
      Persist::setImmediate(!Persist::immediate());
      Serial.printf("immediate writes %s\n", Persist::immediate() ? "on" : "off");
    } else if (ch == 'Z') {
      stallTest();
    } else if (ch == 'V') {
      Persist::setSuppressed(!Persist::suppressed());
      Serial.printf("event saves %s\n", Persist::suppressed() ? "suppressed" : "enabled");
    } else if (ch == 'e') {
      LvLock lk;
      Ui::openEvent(0);
    } else if (ch == 'a') {
      LvglPort::logNextFrameAreas();
    } else if (ch == 'T') {
      static bool bypass = false;
      bypass = !bypass;
      LvglPort::setTouchBypass(bypass);
      Serial.printf("touch bypass %s\n", bypass ? "on" : "off");
    } else if (ch == 'o') {  // OBD on or off, stored, as the Settings switch
      Obd::setEnabled(!Obd::enabled());
      Serial.printf("OBD %s (stored)\n", Obd::enabled() ? "on" : "off");
    } else if (ch == 'O') {
      Obd::printStatus();
    } else if (ch == 'L') {  // every OBD request and reply on serial
      Obd::setTrafficLog(!Obd::trafficLog());
      Serial.printf("OBD traffic log %s\n", Obd::trafficLog() ? "on" : "off");
    } else if (ch == 'F') {
      Obd::forgetDongle();
    } else if (ch == 'P') {  // pair with a dongle now, as the Settings button (only when none is stored)
      Obd::requestPairing();
      Serial.println(Obd::paired() ? "OBD: a dongle is already paired; F forgets it" : "OBD: pairing for 30 s");
    } else if (ch == 'A') {  // the car's forward axis in the IMU frame: cycles +X -X +Y -Y +Z -Z and stores it
      uint8_t axis = (Sensors::forwardAxis() + 1) % 6;
      Sensors::setForwardAxis(axis);
      Serial.printf("imu: forward axis now %s (%s)\n", IMU_AXIS_NAMES[axis], Storage::saveForwardAxis(axis) ? "stored" : "NOT stored");
    } else if (ch == 'K') {  // KK within 3 s: format the trip partition. The only way it is ever formatted once it holds data.
      static uint32_t firstK = 0;
      if (firstK && millis() - firstK < 3000) {
        firstK = 0;
        Storage::formatNow();
        Persist::requestStats();
      } else {
        firstK = millis();
        Serial.println("storage: K again within 3 s formats the trip partition and erases every trip");
      }
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
    cb.setDemo = [](bool on) { Persist::requestDemo(on); };
    cb.setNightMode = uiSetNight;
    cb.calibrateMounting = uiCalibrate;
    cb.calibrateAccel = uiCalibrateAccel;
    cb.setObd = [](bool on) { Obd::setEnabled(on); };
    cb.forgetDongle = []() { Obd::forgetDongle(); };
    cb.pairDongle = []() { Obd::requestPairing(); };
    cb.resetTrip = []() { Persist::requestResetTrip(); };
    cb.resetPeaks = uiResetPeaks;
    cb.clearRecords = []() { Persist::requestClearRecords(); };
    cb.settingsChanged = uiSettingsChanged;
    cb.loadTrace = Persist::loadTrace;
    Ui::begin(cb, VERSION);
  }
  Serial.printf("\nmini-dash %s, built %s %s, reset reason %d (1 power-on, 3 software, 4 panic, 5 int wdt, 6 task wdt, 7 other wdt, 8 deep sleep, 9 brownout, 10 sdio, 11 usb)\n",
                VERSION, __DATE__, __TIME__, (int)esp_reset_reason());
  Serial.printf("expander %s, panel %s, touch %s (%s, config %u x %u), lvgl %s, splash at %lu ms\n", expander ? "ok" : "NO",
                g_panel ? "ok" : "NO", g_touchOk ? "ok" : "NO", TouchGt911::productId(), TouchGt911::configXMax(),
                TouchGt911::configYMax(), lv ? "ok" : "NO", (unsigned long)(millis() - t0));
  {
    VsLock lk;
    if (lk.ok()) {
      Vs::raw().diag.touchOk = g_touchOk;
      strncpy(Vs::raw().diag.touchId, TouchGt911::productId(), sizeof(Vs::raw().diag.touchId) - 1);
    }
  }

  Serial.printf("framework: %s\n", Persist::immediate() ? "code and constants execute from PSRAM, flash writes immediate"
                                                        : "prebuilt core, flash writes deferred to standstills and traces held back");

  // Storage and clock, then the previous trip if a power cut left its checkpoint.
  Storage::begin();
  bool rtc = Clock::begin();
  Serial.printf("clock: RTC %s\n", rtc ? "valid" : "not set, waiting for GPS");
  const Storage::Settings &st = Persist::loadSettings();
  Ui::UiSettings us;
  us.mph = st.mph;
  us.nightMode = st.nightMode;
  us.demo = st.demo;
  us.obd = Storage::loadObdEnabled();
  us.bestField = st.bestField;
  {
    LvLock lk;
    Ui::applySettings(us);
  }
  if (st.accCalibrated) Sensors::setAccelCal(st.accOffset, st.accRadius);
  if (st.calibrated) Sensors::setMountingRef(st.gRef);
  Sensors::setForwardAxis(Storage::loadForwardAxis(IMU_FORWARD_AXIS_DEFAULT));
  // Checkpoints left by power cuts are archived here, before any task that could write exists.
  Persist::begin(&g_engine);

  // Sensors and the trip engine come up behind the splash.
  Sensors::start(&g_engine);
  Sensors::setNightMode(st.nightMode);
  {
    LvLock lk;
    Ui::setBootStatus(!Sensors::gpsOk() ? "GPS absent" : (Sensors::gpsConfigured() ? "GPS acquiring" : "GPS set-up failed"), Storage::loadObdEnabled() ? "OBD scanning" : "OBD off",
                      Sensors::imuOk() ? "IMU ready" : "IMU absent");
  }
  Demo::start(&g_engine);
  Obd::start();  // its own task; Bluetooth comes up there, never in the way of the screen
#ifndef MINIDASH_CAPTURE
  Demo::setEnabled(st.demo);  // stored; off by default, Settings turns the replay on for the bench
#endif
  Persist::startFirstTrip();
  Serial.println("keys: d demo, 1..9/0 backlight, p screenshot, n next tab, D/R/Y/E/B page, x settings, i scan, r reboot");
}

void loop() {
  handleSerial();
  uint32_t nowMs = millis();
  Persist::service(nowMs);  // every flash write for trips, records, traces and settings happens in here
  static uint32_t lastMotion = 0;
  if (g_motionLog && millis() - lastMotion >= 100) {
    lastMotion = millis();
    VehicleState vs;
    Vs::snapshot(vs);
    Serial.printf("M,%lu,%+.3f,%+.3f,%+.1f,%.1f,%.3f,%.2f,%.1f,%d,%d,%.3f,%.1f,%.3f\n", (unsigned long)millis(), vs.motion.latG,
                  vs.motion.lonG, vs.motion.yawDps, vs.diag.attTiltDeg, vs.diag.attCorrDeg, vs.diag.gyroBiasDps,
                  vs.disp.speedKph, vs.diag.attTrust, vs.diag.attStationary, vs.diag.accNorm, vs.diag.gyrNorm, vs.diag.restNorm);
  }
  static uint32_t last = 0;
  if (millis() - last >= 5000) {
    last = millis();
    VehicleState vs;
    Vs::snapshot(vs);
    uint32_t fl, to, wt, hm, ha, tk, pl, tmx, tav, tf, tc;
    LvglPort::stats(&fl, &to, &wt, &hm, &ha);
    LvglPort::touchStats(&tmx, &tav, &tf, &tc);
    Ui::timing(&tk, &pl);
    uint32_t gm, gb, gn, ga;
    Sensors::gpsStats(&gm, &gb, &gn, &ga);
    Serial.printf("[%6lu] page %d demo %d spd %.0f G %+.2f/%+.2f kW %.1f gps %d/%d imu %.0f Hz lux %.0f %s BL %u%% | lvgl %lu frames %lu flushes, wait %lu ms, %lu timeouts, handler max %lu avg %lu ms, tick max %lu ms, plot max %lu ms, touch max %lu avg %lu ms %lu fails/%lu | gps poll max %lu ms (backlog %lu B) avg %lu ms nmea %lu B refused %lu | obd %s %.1f Hz soc %.0f %% | att tilt %.1f corr %.3f bias %.2f dps lonbias %+.3f %s%s | flash writes %lu faults %lu pending %d%s | loop stack free %u | heap %lu psram %lu i2c %lu ms by %s\n",
                  (unsigned long)(millis() / 1000), (int)Ui::page(), vs.demo, vs.disp.speedKph, vs.motion.latG,
                  vs.motion.lonG, vs.car.powerKw, vs.gps.fixType, vs.gps.sats, vs.imu.rateHz, vs.disp.lux,
                  vs.disp.night ? "night" : "day", vs.disp.backlightPct,
                  (unsigned long)LvglPort::frameCount(), (unsigned long)fl, (unsigned long)wt, (unsigned long)to,
                  (unsigned long)hm, (unsigned long)ha, (unsigned long)tk, (unsigned long)pl, (unsigned long)tmx, (unsigned long)tav, (unsigned long)tf, (unsigned long)tc, (unsigned long)gm, (unsigned long)gb, (unsigned long)ga, (unsigned long)gn, (unsigned long)Sensors::gpsRefusedReads(), Obd::stateName(vs.diag.obdState), vs.diag.obdHz, vs.car.socPct, vs.diag.attTiltDeg, vs.diag.attCorrDeg, vs.diag.gyroBiasDps, vs.diag.lonBiasG, vs.diag.fwdAxisBad ? "FORWARD AXIS NEAR VERTICAL" : (vs.diag.attTrust ? "trust" : (vs.diag.attTurnHold ? "hold (turn)" : "hold")), vs.diag.attStationary ? " still" : "", (unsigned long)Storage::writeCount(), (unsigned long)Storage::faultCount(), Persist::pendingRecovery(), Storage::needsOwner() ? " STORAGE NEEDS KK" : "", (unsigned)uxTaskGetStackHighWaterMark(nullptr), (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getFreePsram(),
                  (unsigned long)I2cBus::longestHoldMs(), I2cBus::longestHolder());
  }
  delay(10);
}
