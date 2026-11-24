#include "persist.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>

#include "clock.h"
#include "demo_source.h"
#include "sensors.h"
#include "trip_engine.h"
#include "vehicle_state.h"

namespace {
// Flash writes and the panel. While the chip writes flash it cannot fetch code or constants
// from flash, and on the prebuilt Arduino core the PSRAM cache goes down with it, so the
// panel's bounce-buffer refill interrupt stalls and the glass repeats stale ten-line strips:
// banded digits and ghost text (Logan's photos and videos, 24 Oct 2026). On that core the
// writes are deferred until the car has been still for 2 s (or the trip ends, or a checkpoint
// is 10 min overdue), and event traces and checkpoints are held back entirely by default.
// Env minidash-xip rebuilds the core with code and read-only data executing from PSRAM and
// the refill interrupt IRAM-safe (CONFIG_SPIRAM_XIP_FROM_PSRAM, CONFIG_LCD_RGB_ISR_IRAM_SAFE):
// measured on the bench 24 Oct 2026 with key Z, a flash write then stalls PSRAM access for no
// longer than the idle jitter (1.5 to 1.8 ms, against 19 to 20 ms on the prebuilt core), so
// that build writes at once, as the brief asks (traces saved on acceptance, checkpoint every
// 30 s). Keys V and I toggle the two behaviours at run time for tests.
#ifdef MINIDASH_XIP
constexpr bool WRITES_STALL_PANEL = false;
#else
constexpr bool WRITES_STALL_PANEL = true;
#endif
constexpr uint32_t STILL_MS = 2000;
constexpr uint32_t CHECKPOINT_FORCE_MS = 10 * 60 * 1000;
constexpr uint32_t WRITE_SPACING_MS = 100;       // one write per pass, at most ten a second
constexpr uint32_t LIFETIME_MIN_MS = 30000;      // the lifetime set is written at most this often
constexpr uint32_t RETRY_MS = 5000;              // after a failed write
constexpr float ELIGIBLE_KM = 0.05f;             // a trip of 50 m or more is kept (owner's decision D2)

TripEngine *g_engine = nullptr;
Storage::Settings g_settings;
bool g_ready = false, g_capture = false;
volatile bool g_suppress = WRITES_STALL_PANEL;    // hold back traces and checkpoints (prebuilt core default)
volatile bool g_immediate = !WRITES_STALL_PANEL;  // write without waiting for a standstill
volatile bool g_tripWrites = true;

// --- requests ---------------------------------------------------------------------------
portMUX_TYPE g_reqMux = portMUX_INITIALIZER_UNLOCKED;
struct SettingsReq {
  bool mph, demo;
  int8_t nightMode;
  uint8_t bestField;
};
SettingsReq g_settingsReq;
volatile bool g_settingsPending = false, g_resetPending = false, g_clearPending = false, g_demoPending = false;
volatile bool g_demoTarget = false;

// --- trace slots (review finding 06) -----------------------------------------------------
struct TraceSlot {
  bool pending = false;
  uint32_t generation = 0;
  uint32_t tripId = 0;
  size_t n = 0;
  Storage::TraceSample samples[Storage::TRACE_MAX];
};
SemaphoreHandle_t g_slotMutex = nullptr;
TraceSlot g_slots[REC_COUNT];
TraceSlot g_work;  // the writer's own copy; only the loop task touches it
uint8_t g_traceRetries[REC_COUNT] = {};

// --- trip state, loop task only ----------------------------------------------------------
uint32_t g_tripStartUtc = 0, g_tripStartMs = 0, g_tripDay = 0;
uint32_t g_lastCheckpointMs = 0, g_lastCheckpointWriteMs = 0, g_lastLifetimeWriteMs = 0;
uint32_t g_lastWriteMs = 0, g_retryAfterMs = 0, g_stillSinceMs = 0, g_lastStillCheckMs = 0;
bool g_checkpointDue = false, g_statsDue = false, g_still = false;
bool g_tripIsolated = false;  // the running trip is a replay (or otherwise kept out of the history)
int g_pendingRecovery = 0;

void refreshStoreStats() {
  Storage::DayStats today, all;
  RecordSet todayBest;
  uint32_t now = Clock::nowUtc();
  uint32_t tripId = 0;
  {
    VsLock lk;
    if (lk.ok()) tripId = g_engine->tripId();
  }
  Storage::aggregateDay(now, todayBest, today, tripId);
  Storage::aggregateAll(all);
  // The file system is read before the state lock is taken, never under it (finding 17).
  uint16_t trips = (uint16_t)Storage::tripCount();
  uint32_t usedKb = Storage::fsUsedBytes() / 1024, totalKb = Storage::fsTotalBytes() / 1024;
  VsLock lk;
  if (!lk.ok()) {
    g_statsDue = true;
    return;
  }
  StoreStats &st = Vs::raw().store;
  st.fsOk = Storage::fsOk();
  st.storedTrips = trips;
  st.todayTrips = today.trips;
  st.todayKm = today.km;
  st.allTrips = all.trips;
  st.allKm = all.km;
  st.usedKb = usedKb;
  st.totalKb = totalKb;
  st.faults = (uint16_t)Storage::faultCount();
  st.pendingRecovery = (uint8_t)g_pendingRecovery;
  st.needsOwner = Storage::needsOwner();
}

// The engine's task, holding the state lock, leaves a finished trace here. The slot is marked
// pending only after it is complete, and the writer takes its own copy before writing.
void eventSink(uint32_t tripId, int rec, const Storage::TraceSample *samples, size_t n) {
  if (g_suppress || !g_tripWrites || rec < 0 || rec >= REC_COUNT || !g_slotMutex) return;
  if (xSemaphoreTake(g_slotMutex, pdMS_TO_TICKS(5)) != pdTRUE) return;  // the writer is copying: this trace is dropped
  TraceSlot &s = g_slots[rec];  // the latest window for a record replaces an unsaved one
  s.pending = false;
  s.tripId = tripId;
  s.n = n > Storage::TRACE_MAX ? Storage::TRACE_MAX : n;
  memcpy(s.samples, samples, s.n * sizeof(Storage::TraceSample));
  s.generation++;
  s.pending = true;
  g_traceRetries[rec] = 0;
  xSemaphoreGive(g_slotMutex);
}

// One pending trace written. Returns true if a write was attempted.
bool writeOneTrace() {
  for (int i = 0; i < REC_COUNT; i++) {
    if (!g_slots[i].pending) continue;  // a peek; the copy below is under the mutex
    if (xSemaphoreTake(g_slotMutex, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    bool have = g_slots[i].pending;
    if (have) {
      g_work = g_slots[i];
      g_slots[i].pending = false;
    }
    xSemaphoreGive(g_slotMutex);
    if (!have) continue;
    if (Storage::saveEvent(g_work.tripId, i, g_work.samples, g_work.n)) return true;
    // Put it back for another try, unless a newer trace has taken the slot meanwhile.
    if (++g_traceRetries[i] <= 3 && xSemaphoreTake(g_slotMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      if (!g_slots[i].pending && g_slots[i].generation == g_work.generation) g_slots[i].pending = true;
      xSemaphoreGive(g_slotMutex);
    }
    return true;
  }
  return false;
}

bool tracesPending() {
  for (int i = 0; i < REC_COUNT; i++)
    if (g_slots[i].pending) return true;
  return false;
}

void dropTraces(uint32_t tripId) {
  if (xSemaphoreTake(g_slotMutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
  for (int i = 0; i < REC_COUNT; i++)
    if (g_slots[i].tripId == tripId) g_slots[i].pending = false;
  xSemaphoreGive(g_slotMutex);
}

bool writeLifetimeIfDirty() {
  RecordSet life;
  bool dirty = false;
  {
    VsLock lk;
    if (!lk.ok()) return false;
    dirty = g_engine->takeLifetimeIfDirty(life);
  }
  if (!dirty) return false;
  if (!Storage::saveLifetime(life)) {
    VsLock lk;
    if (lk.ok()) g_engine->markLifetimeDirty();
  }
  g_lastLifetimeWriteMs = millis();
  return true;
}

// A trip that ended: archived if it qualifies, discarded if not. The checkpoint goes only once
// the trip file is safely written; if that fails it stays (and is refreshed with the final
// figures) for the next boot to archive (review finding 04).
void archive(const Storage::TripSummary &tIn, bool recovered) {
  Storage::TripSummary t = tIn;
  t.closed = true;
  if (t.demo || !(t.trip.distanceKm >= ELIGIBLE_KM) || !g_tripWrites) {
    Storage::deleteEvents(t.id);  // a demo or a trip under 50 m leaves nothing behind
    Storage::clearActive(t.id);
    return;
  }
  if (Storage::saveTrip(t)) {
    Storage::clearActive(t.id);
    Serial.printf("storage: trip %lu saved, %.1f km%s\n", (unsigned long)t.id, t.trip.distanceKm, recovered ? " (ended by power loss)" : "");
    if (!recovered) {
      VsLock lk;
      if (lk.ok()) g_engine->setLastTrip(t.records);
    }
  } else {
    if (!recovered) Storage::saveActive(t);  // the final figures, for the next boot
    g_pendingRecovery++;
    Serial.printf("storage: trip %lu could NOT be archived; its checkpoint is kept and will be retried at the next boot\n", (unsigned long)t.id);
  }
}

void startNewTrip(bool isolated) {
  uint32_t id = Storage::reserveTripId();
  uint32_t utc = Clock::nowUtc();
  RecordSet todayBest;
  Storage::DayStats ds;
  Storage::aggregateDay(utc, todayBest, ds, id);
  uint32_t now = millis();
  {
    VsLock lk(200);
    if (lk.ok()) {
      g_engine->startTrip(id, now, utc);
      g_engine->setToday(todayBest);
      g_engine->setIsolated(isolated);
      g_engine->publish(Vs::raw());
    }
  }
  g_tripStartUtc = utc;
  g_tripStartMs = now;
  g_tripDay = utc ? Clock::localDayNumber(utc) : 0;
  g_lastCheckpointMs = g_lastCheckpointWriteMs = now;  // the overdue hatch counts from the trip start
  g_tripIsolated = isolated;
  g_checkpointDue = false;
  g_statsDue = true;
}

// Close the running trip and start the next. The new trip's number is reserved first; then,
// in one hold of the state lock, the old trip is summarised and the engine moves to the new
// one, so no update can land between the two; then the old trip is written with the lock
// released. Returns false if the lock could not be had (the request stays pending).
bool rotateTrip(bool nextIsolated) {
  uint32_t newId = Storage::reserveTripId();
  uint32_t utc = Clock::nowUtc();
  uint32_t day = utc ? Clock::localDayNumber(utc) : 0;
  RecordSet todayBest;
  bool newDay = day != g_tripDay;
  if (newDay) {  // the next trip starts on another day (or the clock has only now arrived)
    Storage::DayStats ds;
    Storage::aggregateDay(utc, todayBest, ds, newId);
  }
  Storage::TripSummary old;
  RecordSet life;
  bool lifeDirty = false;
  uint32_t now = millis();
  {
    VsLock lk(200);
    if (!lk.ok()) return false;
    g_engine->summary(old, now, g_tripIsolated);  // not Demo::enabled(): the source is already off when a replay is left
    lifeDirty = g_engine->takeLifetimeIfDirty(life);
    g_engine->startTrip(newId, now, utc);
    if (newDay) g_engine->setToday(todayBest);
    g_engine->setIsolated(nextIsolated);
    g_engine->publish(Vs::raw());
  }
  g_tripStartUtc = utc;
  g_tripStartMs = now;
  g_tripDay = day;
  g_tripIsolated = nextIsolated;
  g_lastCheckpointMs = g_lastCheckpointWriteMs = now;
  g_checkpointDue = false;
  // The old trip, with the lock released: its lifetime figures, its last traces, its file.
  if (lifeDirty && g_tripWrites && !Storage::saveLifetime(life)) {
    VsLock lk;
    if (lk.ok()) g_engine->markLifetimeDirty();
  }
  if (old.demo || !(old.trip.distanceKm >= ELIGIBLE_KM)) dropTraces(old.id);
  else
    for (int k = 0; k < REC_COUNT && tracesPending(); k++) writeOneTrace();
  archive(old, false);
  g_statsDue = true;
  return true;
}

bool saveSettingsNow(const char *why) {
  bool ok = Storage::saveSettings(g_settings);
  Serial.printf(ok ? "settings: stored (%s)\n" : "settings: could NOT be stored (%s)\n", why);
  return ok;
}

void handleRequests(uint32_t nowMs) {
  if (g_settingsPending) {
    SettingsReq r;
    portENTER_CRITICAL(&g_reqMux);
    r = g_settingsReq;
    g_settingsPending = false;
    portEXIT_CRITICAL(&g_reqMux);
    g_settings.mph = r.mph;
    g_settings.nightMode = r.nightMode;
    g_settings.demo = r.demo;
    g_settings.bestField = r.bestField;
    g_settings.brightness = 100;
    saveSettingsNow("settings page");
  }
  // Calibrations finish inside the sensor task; a complete result is collected here, stored,
  // and only then acknowledged to the UI (review finding 28).
  float ref[3];
  if (Sensors::mountRefTake(ref)) {
    for (int i = 0; i < 3; i++) g_settings.gRef[i] = ref[i];
    g_settings.calibrated = true;
    bool stored = saveSettingsNow("mounting reference");
    VsLock lk;
    if (lk.ok()) (stored ? Vs::raw().diag.mountRefSaves : Vs::raw().diag.mountRefFails)++;
  }
  bool ok;
  float off[3], radius, res;
  int dirs;
  if (Sensors::accelCalTake(&ok, off, &radius, &res, &dirs) && ok) {
    for (int i = 0; i < 3; i++) g_settings.accOffset[i] = off[i];
    g_settings.accRadius = radius;
    g_settings.accCalibrated = true;
    saveSettingsNow("accelerometer offsets");
  }
  if (g_clearPending) {
    g_clearPending = false;
    Storage::clearLifetime();
    VsLock lk;
    if (lk.ok()) {
      RecordSet none, discard;
      g_engine->setLifetime(none);
      g_engine->takeLifetimeIfDirty(discard);  // nothing older may be written after the clear
      g_engine->publish(Vs::raw());
    }
  }
  if (g_demoPending) {
    bool on = g_demoTarget;
    if (g_capture) {
      g_demoPending = false;  // the replay has no place in a capture drive
      Serial.println("replay: not available in capture mode (serial MM leaves it)");
    } else if (on == Demo::enabled()) {
      g_demoPending = false;
    } else {
      // Leaving the replay: stop the source first and let its last update land, so no synthetic
      // sample reaches the real trip. Entering it: rotate to an isolated trip first.
      if (!on) {
        Demo::setEnabled(false);
        delay(60);
      }
      if (rotateTrip(on)) {
        if (on) Demo::setEnabled(true);
        g_demoPending = false;
        g_settings.demo = on;
        saveSettingsNow(on ? "replay on" : "replay off");
      } else if (!on) {
        Demo::setEnabled(true);  // could not rotate: stay as we were and try again
      }
    }
  }
  if (g_resetPending && rotateTrip(Demo::enabled())) g_resetPending = false;
  (void)nowMs;
}

void recoverCheckpoints() {
  Storage::TripSummary t;
  if (Storage::loadLegacyActive(t)) {
    Serial.printf("storage: trip %lu ended by power loss, %.1f km\n", (unsigned long)t.id, t.trip.distanceKm);
    int before = g_pendingRecovery;
    archive(t, true);
    if (g_pendingRecovery == before) Storage::clearLegacyActive();
  }
  uint32_t ids[Storage::MAX_ACTIVE];
  int n = Storage::listActive(ids, Storage::MAX_ACTIVE);
  RecordSet life;
  Storage::loadLifetime(life);
  bool lifeChanged = false;
  for (int i = 0; i < n; i++) {
    if (!Storage::loadActive(ids[i], t)) {
      Serial.printf("storage: checkpoint %lu is unreadable; set aside as %lu.bad\n", (unsigned long)ids[i], (unsigned long)ids[i]);
      Storage::quarantineActive(ids[i]);
      continue;
    }
    Serial.printf("storage: trip %lu ended by power loss, %.1f km\n", (unsigned long)t.id, t.trip.distanceKm);
    // Its records belong in the lifetime set whether or not that was written before the cut;
    // merging the better values is harmless if it was.
    if (!t.demo && t.trip.distanceKm >= ELIGIBLE_KM && TripEngine::merge(life, t.records, false, true)) lifeChanged = true;
    archive(t, true);
  }
  if (lifeChanged && g_tripWrites) Storage::saveLifetime(life);
}
}  // namespace

namespace Persist {

const Storage::Settings &loadSettings() {
  Storage::loadSettings(g_settings);
  return g_settings;
}

const Storage::Settings &settings() { return g_settings; }

void begin(TripEngine *engine, bool captureMode) {
  g_engine = engine;
  g_slotMutex = xSemaphoreCreateMutex();
  if (!g_slotMutex) Serial.println("persist: no memory for the trace mutex; traces will not be stored");
  g_capture = captureMode;
  if (g_capture) {
    g_tripWrites = false;
    Serial.println("persist: CAPTURE MODE, no trip, record or trace is written; what is stored is left untouched");
  } else {
    Storage::quarantineUnreadableTrips();
    recoverCheckpoints();
    Storage::pruneOrphanEvents();  // traces of demo and discarded trips; trips waiting in /active keep theirs
  }
  RecordSet lifetime;
  Storage::loadLifetime(lifetime);
  g_engine->setLifetime(lifetime);
  Storage::TripSummary last;
  if (Storage::loadTrip(0, last)) g_engine->setLastTrip(last.records);
  g_engine->setEventSink(eventSink);
}

void startFirstTrip() {
  startNewTrip(Demo::enabled());
  g_ready = true;
}

void service(uint32_t nowMs) {
  if (!g_ready) return;
  handleRequests(nowMs);

  // The clock arrived after the trip began: date the trip from how long it has been running,
  // without restarting it, and rebuild Today for that day (review finding 02).
  if (g_tripStartUtc == 0 && Clock::nowUtc()) {
    uint32_t utc = Clock::nowUtc() - (nowMs - g_tripStartMs) / 1000;
    uint32_t tripId = 0;
    {
      VsLock lk;
      if (lk.ok()) tripId = g_engine->tripId();
    }
    RecordSet todayBest;
    Storage::DayStats ds;
    Storage::aggregateDay(utc, todayBest, ds, tripId);
    VsLock lk;
    if (lk.ok()) {
      g_engine->setStartUtc(utc);
      g_engine->rebaseToday(todayBest);
      g_engine->publish(Vs::raw());
      g_tripStartUtc = utc;
      g_tripDay = Clock::localDayNumber(utc);
      g_statsDue = true;
    }
  }

  // A checkpoint falls due every 30 s, and at once when the trip reaches 50 m.
  bool eligibleNow = false, eligible = false;
  {
    VsLock lk;
    if (lk.ok()) {
      eligibleNow = g_engine->takeEligibleEvent();
      eligible = g_engine->eligible();
    }
  }
  if (eligibleNow || nowMs - g_lastCheckpointMs >= Storage::CHECKPOINT_MS) {
    g_lastCheckpointMs = nowMs;
    g_checkpointDue = true;
  }

  // When may flash be written? At once on the rebuilt core. On the prebuilt core only while
  // the car has been still for 2 s, or when a real checkpoint is 10 min overdue.
  bool overdue = false;
  if (!g_immediate && nowMs - g_lastStillCheckMs >= 100) {
    g_lastStillCheckMs = nowMs;
    VehicleState vs;
    if (Vs::snapshot(vs)) {
      bool still = vs.disp.speedKph < 2.0f;
      if (!still) g_stillSinceMs = 0;
      else if (g_stillSinceMs == 0) g_stillSinceMs = nowMs;
      g_still = still && g_stillSinceMs && nowMs - g_stillSinceMs >= STILL_MS;
    }
  }
  if (!g_immediate) overdue = g_checkpointDue && !g_tripIsolated && nowMs - g_lastCheckpointWriteMs >= CHECKPOINT_FORCE_MS;
  bool mayWrite = g_tripWrites && (g_immediate || g_still || overdue) && nowMs - g_lastWriteMs >= WRITE_SPACING_MS &&
                  (int32_t)(nowMs - g_retryAfterMs) >= 0;
  if (mayWrite) {
    // One write per pass. The checkpoint comes first: it is what a power cut costs most.
    if (g_checkpointDue && eligible && !g_tripIsolated && !g_suppress) {
      Storage::TripSummary t;
      bool got = false;
      {
        VsLock lk;
        if (lk.ok()) {
          g_engine->summary(t, nowMs, false);
          got = true;
        }
      }
      if (got) {
        g_lastWriteMs = nowMs;
        if (Storage::saveActive(t)) {
          g_checkpointDue = false;  // cleared only by a checkpoint that was actually written
          g_lastCheckpointWriteMs = nowMs;
        } else {
          g_retryAfterMs = nowMs + RETRY_MS;
        }
      }
    } else if (nowMs - g_lastLifetimeWriteMs >= LIFETIME_MIN_MS && writeLifetimeIfDirty()) {
      g_lastWriteMs = nowMs;  // at most every 30 s: a building peak used to write ten times a second
    } else if (writeOneTrace()) {
      g_lastWriteMs = nowMs;
    }
  }
  if (g_statsDue) {
    g_statsDue = false;
    refreshStoreStats();
  }
}

void requestSettings(bool mph, int8_t nightMode, bool demo, uint8_t bestField) {
  if (!g_ready) return;  // the stored settings are not loaded yet: nothing may overwrite them
  portENTER_CRITICAL(&g_reqMux);
  g_settingsReq = {mph, demo, nightMode, bestField};
  g_settingsPending = true;
  portEXIT_CRITICAL(&g_reqMux);
}
void requestDemo(bool on) {
  g_demoTarget = on;
  g_demoPending = true;
}
void requestResetTrip() { g_resetPending = true; }
void requestStats() { g_statsDue = true; }
void requestClearRecords() { g_clearPending = true; }

bool loadTrace(uint32_t tripId, int rec, Storage::TraceSample *out, size_t max, size_t *n) {
  return Storage::loadEvent(tripId, rec, out, max, n);
}

void setSuppressed(bool on) { g_suppress = on; }
bool suppressed() { return g_suppress; }
void setImmediate(bool on) { g_immediate = on; }
bool immediate() { return g_immediate; }
void setTripWrites(bool on) { g_tripWrites = on; }
bool tripWrites() { return g_tripWrites; }
int pendingRecovery() { return g_pendingRecovery; }

uint32_t stallTestWrites() {
  static Storage::TraceSample tr[Storage::TRACE_MAX];
  for (size_t i = 0; i < Storage::TRACE_MAX; i++) tr[i] = {(uint16_t)i, (int16_t)(i * 10), 0, 0, (int16_t)i};
  uint32_t t0 = millis();
  for (int k = 0; k < 5; k++) Storage::saveEvent(999000 + k, 0, tr, Storage::TRACE_MAX);
  uint32_t ms = millis() - t0;
  for (int k = 0; k < 5; k++) Storage::deleteEvents(999000 + k);
  return ms;
}

}  // namespace Persist
