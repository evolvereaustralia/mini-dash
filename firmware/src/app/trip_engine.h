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
#pragma once

#include <stdint.h>

#include "vehicle_state.h"

class TripEngine {
 public:
  void reset(uint32_t nowMs);

  // One update with whatever is fresh. speedValid false means the timers invalidate.
  void update(uint32_t nowMs, float speedKph, bool speedValid, float speedAccMps, uint8_t sats,
              float latG, float lonG, bool motionValid, float powerKw, bool carLive, float socPct,
              float packTemp, float altM);

  // Writes trip, motion peaks and the three record sets into the state (caller holds the lock).
  void publish(VehicleState &vs, const char *clockHHMM) const;

  void resetSessionPeaks();
  void setLifetime(const RecordSet &all) { all_ = all; }  // loaded from NVS at boot (later)

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

  TripData trip_;
  MotionData peaks_;
  RecordSet session_, today_, all_;
  uint32_t lastMs_ = 0;
  float lastSoc_ = -1;

  void accept(RecordId id, float value, float aux, uint32_t nowMs, float soc, float temp,
              bool lowerIsBetter);
  static float crossingMs(float v0, uint32_t t0, float v1, uint32_t t1, float target);
  void checkSpeedTimers(uint32_t nowMs, float kph, bool valid, float lonG, float speedAccMps,
                        uint8_t sats, float soc, float temp);
  void checkPower(uint32_t nowMs, float powerKw, bool carLive, bool moving, float soc, float temp);
  mutable char clock_[12] = "";
};
