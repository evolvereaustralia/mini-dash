// Mini Dash firmware (phase 3b onward). Boot order (CLAUDE.md section 4 and 5): serial, I2C,
// expander, panel, touch, LVGL and the splash first, so the screen is on within a second;
// then the sensor task on core 0 and the demo replay. LVGL runs on core 1.
//
// Serial keys: d = demo on/off, 1..9 and 0 = backlight 10..100 %, p = screenshot (base64
// RGB565, decode with tools/screenshot.py), n = next tab, D R Y E B = a page, x = settings, i = I2C scan, r = reboot.

#include <Arduino.h>
#include <string.h>
#include <LittleFS.h>

#include "esp_heap_caps.h"

#include "app/clock.h"
#include "app/demo_source.h"
#include "app/lvgl_port.h"
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
Storage::Settings g_settings;
uint32_t g_lastCheckpointMs = 0;
uint32_t g_lastCheckpointWriteMs = 0;
uint32_t g_tripStartUtc = 0;

// Storage statistics into the state (after boot and whenever a trip is saved).
void refreshStoreStats() {
  Storage::DayStats today, all;
  RecordSet todayBest;
  uint32_t now = Clock::nowUtc();
  Storage::aggregateDay(now, todayBest, today, g_engine.tripId());
  Storage::aggregateAll(all);
  VsLock lk;
  if (!lk.ok()) return;
  StoreStats &st = Vs::raw().store;
  st.fsOk = Storage::fsOk();
  st.storedTrips = Storage::tripCount();
  st.todayTrips = today.trips;
  st.todayKm = today.km;
  st.allTrips = all.trips;
  st.allKm = all.km;
  st.usedKb = Storage::fsUsedBytes() / 1024;
  st.totalKb = Storage::fsTotalBytes() / 1024;
}

void flushDeferredWrites(bool force);

// Close the running trip into a trip file (unless it is a demo or went nowhere) and start a
// new one. Used by "Reset this trip" and at boot for a checkpoint left by a power cut.
void finishTrip(bool fromCheckpoint, const Storage::TripSummary *checkpoint) {
  Storage::TripSummary t;
  if (fromCheckpoint) {
    t = *checkpoint;
  } else {
    g_engine.summary(t, millis(), Demo::enabled());
  }
  t.closed = true;
  flushDeferredWrites(true);  // the trip's last events go with it
  if (!t.demo && t.trip.distanceKm > 0.05f) {
    if (Storage::saveTrip(t)) {
      g_engine.setLastTrip(t.records);
      Serial.printf("storage: trip %lu saved, %.1f km\n", (unsigned long)t.id, t.trip.distanceKm);
    }
  } else {
    Storage::deleteEvents(t.id);  // a demo or empty trip leaves nothing behind
  }
  Storage::clearActive();
}

void startNewTrip() {
  uint32_t id = Storage::nextTripId();
  g_tripStartUtc = Clock::nowUtc();
  RecordSet todayBest;
  Storage::DayStats ds;
  Storage::aggregateDay(g_tripStartUtc, todayBest, ds, id);
  {
    VsLock lk;
    if (lk.ok()) {
      g_engine.startTrip(id, millis(), g_tripStartUtc);
      g_engine.setToday(todayBest);
      g_engine.setIsolated(Demo::enabled());
      g_engine.publish(Vs::raw());
    }
  }
  g_lastCheckpointMs = millis();
  g_lastCheckpointWriteMs = millis();  // the overdue hatch counts from the trip start
  refreshStoreStats();
}

void saveSettings(const Ui::UiSettings &u) {
  g_settings.mph = u.mph;
  g_settings.nightMode = u.nightMode;
  g_settings.demo = u.demo;
  g_settings.bestField = u.bestField;
  g_settings.brightness = 100;
  g_settings.calibrated = Sensors::mountingRef(g_settings.gRef);
  Storage::saveSettings(g_settings);
}

// Flash writes are deferred until the car has been still for 2 s (or the trip ends, or a
// checkpoint is 10 min overdue). While the chip writes flash it cannot read PSRAM, and the
// panel's bounce buffers then repeat stale ten-line strips: banded digits and ghost text
// (Logan's photos, 24 Oct 2026). When nothing on screen moves, the stale strips are identical
// to the live ones and the write is invisible.
// Event traces and checkpoints are held back entirely until flash writes stop disturbing the
// panel (framework rebuild or SD card, see docs/test-log.md); key V toggles them for tests.
// Settings and lifetime records still write, rarely and at standstill.
volatile bool g_suppressWrites = true;
volatile bool g_immediateWrites = false; // diagnostics key I: write at once, ignoring the still rule
struct PendingEvent {
  bool pending = false;
  uint32_t tripId = 0;
  size_t n = 0;
  Storage::TraceSample samples[Storage::TRACE_MAX];
};
PendingEvent g_pendingEvents[REC_COUNT];
bool g_checkpointDue = false;
uint32_t g_stillSinceMs = 0;
constexpr uint32_t STILL_MS = 2000;
constexpr uint32_t CHECKPOINT_FORCE_MS = 10 * 60 * 1000;

void eventSink(uint32_t tripId, int rec, const Storage::TraceSample *samples, size_t n) {
  if (g_suppressWrites || rec < 0 || rec >= REC_COUNT) return;
  PendingEvent &p = g_pendingEvents[rec];  // the latest window for a record replaces an unsaved one
  p.pending = true;
  p.tripId = tripId;
  p.n = n > Storage::TRACE_MAX ? Storage::TRACE_MAX : n;
  memcpy(p.samples, samples, p.n * sizeof(Storage::TraceSample));
}

// Write ONE queued item per call while the car is still (the caller runs every 100 ms), so a
// burst of writes cannot spill into the first second of motion; force writes everything.
void flushDeferredWrites(bool force) {
  uint32_t nowMs = millis();
  for (int i = 0; i < REC_COUNT; i++) {
    if (!g_pendingEvents[i].pending) continue;
    Storage::saveEvent(g_pendingEvents[i].tripId, i, g_pendingEvents[i].samples, g_pendingEvents[i].n);
    g_pendingEvents[i].pending = false;
    if (!force) return;
  }
  if (g_checkpointDue && !Demo::enabled() && !g_suppressWrites) {
    Storage::TripSummary t;
    {
      VsLock lk;
      if (lk.ok()) g_engine.summary(t, nowMs, false);
    }
    if (t.trip.distanceKm > 0.05f) Storage::saveActive(t);
    g_checkpointDue = false;
    g_lastCheckpointWriteMs = nowMs;
    if (!force) return;
  }
  if (g_engine.lifetimeDirty()) Storage::saveLifetime(g_engine.lifetime());
}
bool uiLoadTrace(uint32_t tripId, int rec, Storage::TraceSample *out, size_t max, size_t *n) {
  return Storage::loadEvent(tripId, rec, out, max, n);
}

void uiSetDemo(bool on) { Demo::setEnabled(on); }
void uiSetNight(int mode) { Sensors::setNightMode(mode); }
void uiCalibrate() {
  Sensors::calibrateMounting();
  // Persist on the next settings save (the reference is taken by the sensor task shortly).
}
void uiResetTrip() {
  finishTrip(false, nullptr);
  startNewTrip();
}
void uiResetPeaks() {
  VsLock lk;
  if (lk.ok()) {
    g_engine.resetSessionPeaks();
    g_engine.publish(Vs::raw());
  }
}
void uiClearRecords() {
  Storage::clearLifetime();
  VsLock lk;
  if (lk.ok()) {
    g_engine.setLifetime(RecordSet());
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
    } else if (ch == 'W') {
      // Storage self-test: write the current session as a demo-flagged trip file and an event
      // trace, read both back, report. Demo-flagged files never count in Today or All time.
      Storage::TripSummary t;
      {
        VsLock lk;
        if (lk.ok()) g_engine.summary(t, millis(), true);
      }
      t.closed = true;
      t.id = 900000 + (millis() / 1000) % 1000;  // test ids, out of the real sequence
      bool w = Storage::saveTrip(t);
      Storage::TraceSample tr[20];
      for (int i = 0; i < 20; i++) tr[i] = {(uint16_t)i, (int16_t)(i * 50), (int16_t)(i * 10), (int16_t)(100 - i * 5), (int16_t)(i * 30)};
      bool we = Storage::saveEvent(t.id, REC_0_60, tr, 20);
      Storage::TripSummary back;
      bool r = Storage::loadTrip(0, back);
      Storage::TraceSample tb[Storage::TRACE_MAX];
      size_t n = 0;
      bool re = Storage::loadEvent(t.id, REC_0_60, tb, Storage::TRACE_MAX, &n);
      Serial.printf("storage test: trip write %s, event write %s, trip read %s (id %lu, %.2f km, 0-60 %s %.1f s), event read %s (%u samples, last kph10 %d) | %d trips, %lu KB used\n",
                    w ? "ok" : "FAIL", we ? "ok" : "FAIL", r ? "ok" : "FAIL", (unsigned long)back.id, back.trip.distanceKm,
                    back.records.r[0].valid ? "valid" : "none", back.records.r[0].value, re ? "ok" : "FAIL", (unsigned)n,
                    n ? tb[n - 1].kph10 : 0, Storage::tripCount(), (unsigned long)(Storage::fsUsedBytes() / 1024));
      refreshStoreStats();
    } else if (ch == 'X') {
      // Remove the test trips written by W.
      int before = Storage::tripCount();
      Storage::TripSummary t;
      for (int i = 0; Storage::loadTrip(i, t); i++) {
        if (t.id >= 900000) { Storage::deleteEvents(t.id); char path[40]; snprintf(path, sizeof(path), "/trips/%lu.trp", (unsigned long)t.id); LittleFS.remove(path); i--; }
      }
      Serial.printf("storage: test trips removed (%d -> %d)\n", before, Storage::tripCount());
      refreshStoreStats();
    } else if (ch == 't') {
      static bool tlog = false;
      tlog = !tlog;
      TouchGt911::setLogging(tlog);
      Serial.printf("touch logging %s\n", tlog ? "on" : "off");
    } else if (ch == 'I') {
      g_immediateWrites = !g_immediateWrites;
      Serial.printf("immediate writes %s\n", g_immediateWrites ? "on" : "off");
    } else if (ch == 'V') {
      g_suppressWrites = !g_suppressWrites;
      Serial.printf("event saves %s\n", g_suppressWrites ? "suppressed" : "enabled");
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
    cb.settingsChanged = saveSettings;
    cb.loadTrace = uiLoadTrace;
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

  // Storage and clock, then the previous trip if a power cut left its checkpoint.
  Storage::begin();
  bool rtc = Clock::begin();
  Serial.printf("clock: RTC %s\n", rtc ? "valid" : "not set, waiting for GPS");
  Storage::loadSettings(g_settings);
  Ui::UiSettings us;
  us.mph = g_settings.mph;
  us.nightMode = g_settings.nightMode;
  us.demo = g_settings.demo;
  us.bestField = g_settings.bestField;
  {
    LvLock lk;
    Ui::applySettings(us);
  }
  if (g_settings.calibrated) Sensors::setMountingRef(g_settings.gRef);
  Storage::TripSummary leftover;
  if (Storage::loadActive(leftover)) {
    Serial.printf("storage: trip %lu ended by power loss, %.1f km, closing it\n", (unsigned long)leftover.id, leftover.trip.distanceKm);
    finishTrip(true, &leftover);
  }
  RecordSet lifetime;
  Storage::loadLifetime(lifetime);
  g_engine.setLifetime(lifetime);
  Storage::TripSummary last;
  if (Storage::loadTrip(0, last)) g_engine.setLastTrip(last.records);
  g_engine.setEventSink(eventSink);

  // Sensors and the trip engine come up behind the splash.
  Sensors::start(&g_engine);
  Sensors::setNightMode(g_settings.nightMode);
  {
    LvLock lk;
    Ui::setBootStatus(Sensors::gpsOk() ? "GPS acquiring" : "GPS absent", "OBD off (phase 3c)",
                      Sensors::imuOk() ? "IMU ready" : "IMU absent");
  }
  Demo::start(&g_engine);
  Demo::setEnabled(g_settings.demo);  // stored; the bench default is on while phase 3b is under way
  startNewTrip();
  Serial.println("keys: d demo, 1..9/0 backlight, p screenshot, n next tab, D/R/Y/E/B page, x settings, i scan, r reboot");
}

void loop() {
  handleSerial();
  uint32_t nowMs = millis();
  // A checkpoint falls due every 30 s; it is written at the next still moment (below) so a
  // power cut loses at most 30 s plus the time to the next stop, or 10 min on a long run.
  if (nowMs - g_lastCheckpointMs >= Storage::CHECKPOINT_MS) {
    g_lastCheckpointMs = nowMs;
    g_checkpointDue = true;
    if (g_tripStartUtc == 0 && Clock::nowUtc()) {  // the clock arrived after the trip started
      g_tripStartUtc = Clock::nowUtc() - (nowMs / 1000);
      VsLock lk;
      if (lk.ok()) g_engine.startTrip(g_engine.tripId(), nowMs, g_tripStartUtc), g_engine.publish(Vs::raw());
    }
  }
  // Still detection on the displayed speed, and the deferred flush.
  {
    static uint32_t lastStillCheck = 0;
    if (nowMs - lastStillCheck >= 100) {
      lastStillCheck = nowMs;
      VehicleState vs;
      Vs::snapshot(vs);
      bool still = vs.disp.speedKph < 2.0f;
      if (!still) g_stillSinceMs = 0;
      else if (g_stillSinceMs == 0) g_stillSinceMs = nowMs;
      bool stillLongEnough = still && g_stillSinceMs && nowMs - g_stillSinceMs >= STILL_MS;
      // The overdue hatch applies to a real checkpoint only: after 10 min with no stop, one
      // write while moving is the price of not losing the trip to a power cut.
      bool overdue = g_checkpointDue && !Demo::enabled() && nowMs - g_lastCheckpointWriteMs >= CHECKPOINT_FORCE_MS;
      if (stillLongEnough || overdue || g_immediateWrites) flushDeferredWrites(overdue);
    }
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
    Serial.printf("[%6lu] page %d demo %d spd %.0f G %+.2f/%+.2f kW %.1f gps %d/%d imu %.0f Hz lux %.0f %s BL %u%% | lvgl %lu frames %lu flushes, wait %lu ms, %lu timeouts, handler max %lu avg %lu ms, tick max %lu ms, plot max %lu ms, touch max %lu avg %lu ms %lu fails/%lu | gps poll max %lu ms (backlog %lu B) avg %lu ms nmea %lu B | flash writes %lu | heap %lu psram %lu i2c %lu ms by %s\n",
                  (unsigned long)(millis() / 1000), (int)Ui::page(), vs.demo, vs.disp.speedKph, vs.motion.latG,
                  vs.motion.lonG, vs.car.powerKw, vs.gps.fixType, vs.gps.sats, vs.imu.rateHz, vs.disp.lux,
                  vs.disp.night ? "night" : "day", vs.disp.backlightPct,
                  (unsigned long)LvglPort::frameCount(), (unsigned long)fl, (unsigned long)wt, (unsigned long)to,
                  (unsigned long)hm, (unsigned long)ha, (unsigned long)tk, (unsigned long)pl, (unsigned long)tmx, (unsigned long)tav, (unsigned long)tf, (unsigned long)tc, (unsigned long)gm, (unsigned long)gb, (unsigned long)ga, (unsigned long)gn, (unsigned long)Storage::writeCount(), (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getFreePsram(),
                  (unsigned long)I2cBus::longestHoldMs(), I2cBus::longestHolder());
  }
  delay(10);
}
