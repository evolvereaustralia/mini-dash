// Trip integration, timers and records (CLAUDE.md sections 4 and 5). Source-agnostic: it is
// fed the displayed speed, the vehicle-frame G and the battery power at whatever rate they
// arrive, with real time deltas, and it never invents a value. The demo replay and the real
// sensors drive the same engine, so the record rules are tested on the bench.
//
// Timing path: raw speed samples with crossing-time interpolation. Arm when speed is under
// 1 km/h and |longitudinal g| under 0.05 for 1 s with the speed source valid; start on the
// crossing of 1 km/h; 0-60 and 0-100 are the interpolated crossing times; a run is dropped if
// the source goes invalid or a sample gap exceeds 300 ms. 100-0 arms above 100 km/h and takes
// time and distance from the 100 crossing to the first sample under 1 km/h with braking g.
// 60-100 is a rolling run from any crossing of 60 upward to 100. Power peaks are one-second
// means; the instantaneous maximum is kept in aux.
//
// Records live in three sets: the session (this trip), today (seeded from today's stored
// trips) and lifetime (seeded from NVS). Session comparisons are against the last stored trip.
// A 12 s trace ring at 10 Hz is captured; when a record is accepted the window around it is
// handed to the event sink 2 s later, so the trace shows what followed the event too.
// In isolated mode (demo replay) only the session set is touched.
#pragma once

#include <stdint.h>

#include "storage.h"
#include "vehicle_state.h"

class TripEngine {
 public:
  typedef void (*EventSink)(uint32_t tripId, int rec, const Storage::TraceSample *samples, size_t n);

  void startTrip(uint32_t tripId, uint32_t nowMs, uint32_t startUtc);
  void reset(uint32_t nowMs) { startTrip(tripId_, nowMs, startUtc_); }

  // One update with whatever is fresh. speedKph is the displayed (fused or wheel) speed and
  // integrates the trip; timingKph is the raw GPS sample the timers run on (CLAUDE.md section
  // 5, path 2), and timingValid false invalidates a run in progress.
  void update(uint32_t nowMs, float speedKph, bool speedValid, float timingKph, bool timingValid,
              float speedAccMps, uint8_t sats, float latG, float lonG, bool motionValid, float powerKw,
              bool carLive, float socPct, float packTemp, float altM);

  // Writes trip, motion peaks and the three record sets into the state (caller holds the lock).
  void publish(VehicleState &vs) const;

  void resetSessionPeaks();

  void setLifetime(const RecordSet &all) { all_ = all; }
  void setToday(const RecordSet &today) { today_ = today; }
  void setLastTrip(const RecordSet &last) { lastTrip_ = last; }
  void setIsolated(bool isolated) { isolated_ = isolated; }
  void setEventSink(EventSink sink) { sink_ = sink; }

  const RecordSet &lifetime() const { return all_; }
  bool lifetimeDirty() { bool d = lifetimeDirty_; lifetimeDirty_ = false; return d; }
  uint32_t tripId() const { return tripId_; }

  // Fill a summary of the session for the checkpoint or the trip file.
  void summary(Storage::TripSummary &out, uint32_t nowMs, bool demo) const;

 private:
  enum class RunState { Idle, Armed, Running, Done };
  struct Timer {
    RunState state = RunState::Idle;
    uint32_t stillSinceMs = 0;
    uint32_t startMs = 0;
    float lastKph = 0;
    uint32_t lastMs = 0;
    bool crossed60 = false, crossed100 = false;
  } accel_;
  struct Brake {
    bool armed = false;
    uint32_t startMs = 0;
    float distanceM = 0;
    float lastKph = 0;
    uint32_t lastMs = 0;
  } brake_;
  struct Rolling {
    bool running = false;
    uint32_t startMs = 0;
  } roll_;

  // one-second power window
  float powerAcc_ = 0;
  uint32_t powerN_ = 0;
  uint32_t powerWindowStartMs_ = 0;
  float instMaxDraw_ = 0, instMaxRegen_ = 0;

  // trace ring and pending events
  Storage::TraceSample ring_[Storage::TRACE_MAX] = {};
  int ringHead_ = 0, ringCount_ = 0;
  uint32_t lastTraceMs_ = 0;
  uint32_t pendingMs_[REC_COUNT] = {};
  EventSink sink_ = nullptr;

  TripData trip_;
  MotionData peaks_;
  RecordSet session_, today_, all_, lastTrip_;
  bool isolated_ = false;
  bool lifetimeDirty_ = false;
  uint32_t tripId_ = 0, startUtc_ = 0, startMs_ = 0;
  uint32_t lastMs_ = 0;

  void accept(RecordId id, float value, float aux, uint32_t nowMs, float soc, float temp,
              bool lowerIsBetter);
  static float crossingMs(float v0, uint32_t t0, float v1, uint32_t t1, float target);
  void checkSpeedTimers(uint32_t nowMs, float kph, bool valid, float lonG, float speedAccMps,
                        uint8_t sats, float soc, float temp);
  void checkPower(uint32_t nowMs, float powerKw, bool carLive, bool moving, float soc, float temp);
  void traceSample(uint32_t nowMs, float kph, float latG, float lonG, float kw);
  void flushEvents(uint32_t nowMs);
};
