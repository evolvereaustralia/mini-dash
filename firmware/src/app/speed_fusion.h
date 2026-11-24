// Displayed speed (CLAUDE.md section 5, path 1): GPS Doppler speed fused with the integrated
// longitudinal acceleration in a complementary filter. The IMU carries the estimate between
// and through the 10 Hz GPS samples at the IMU rate; each GPS sample corrects it against the
// estimate as it was at the instant the receiver measured, so an acceleration is not
// corrected away by a sample that predates it. The timing path never sees this value:
// timers run on raw GPS samples. Plain C++, tested natively.
//
// Rewritten 24 Nov 2026 after the independent review (finding 01 and its notes):
// - the first fix, a fix after a gap of more than a second, and any fix with no IMU samples
//   since the last one SEED the estimate (it then equals the raw Doppler speed), instead of
//   being applied as a correction against an empty history;
// - history is kept as (time, speed) pairs and read by time, not by slot count, and a
//   correction moves the estimate from the measurement instant forward only;
// - the measurement instant comes from the receiver's own epoch (iTOW) mapped onto local
//   time through the smallest arrival delay seen, so a late or batched delivery lands on
//   the right part of the history; a repeated epoch is ignored;
// - standstill is counted in elapsed epoch time, so repeats cannot manufacture it.
// GPS_LATENCY_S remains an assumption to be measured on the road.
#pragma once

#include <stdint.h>

class SpeedFusion {
 public:
  SpeedFusion();
  void reset();
  // Longitudinal acceleration in g (vehicle frame, forward positive), stamped with local
  // microseconds (wraps are handled).
  void imuStep(float lonG, uint32_t nowUs);
  // A valid GPS ground speed with the receiver's epoch (iTOW, ms of week) and the local
  // arrival time.
  void gpsSample(float speedMps, uint32_t epochMs, uint32_t nowUs);
  float speedMps() const { return v_; }
  float speedKph() const { return v_ * 3.6f; }
  // True while the IMU is carrying the estimate between fixes; false when the value is the
  // last raw Doppler speed (no usable IMU history).
  bool inertial() const { return inertial_; }
  // Residual longitudinal offset learnt at standstill, g (diagnostics).
  float lonBiasG() const { return lonBias_; }
  bool stationary() const { return stationaryS_ >= 1.0f; }
  uint32_t seeds() const { return seeds_; }            // diagnostics: times the estimate was reseeded
  uint32_t duplicates() const { return duplicates_; }  // repeated epochs ignored

  static constexpr float GPS_LATENCY_S = 0.10f;  // receiver epoch to earliest arrival; refine on the road
  static constexpr float GPS_GAIN = 0.35f;        // per 10 Hz sample: about 0.25 s time constant
  static constexpr float STILL_MPS = 0.3f;
  static constexpr float LAUNCH_G = 0.05f;        // forward g that ends a standstill before the GPS notices
  static constexpr uint32_t GAP_MS = 1000;        // a fix this long after the last one reseeds

 private:
  static constexpr int HIST = 48;
  static constexpr float HIST_MIN_DT = 0.02f;  // about a second of history
  struct Sample {
    double t;
    float v;
  };
  void seed(float speedMps);
  void push();
  bool at(double t, float *v) const;

  float v_;
  float lonBias_, lonMean_;
  Sample hist_[HIST];
  int histHead_, histCount_;
  double t_;              // local time, seconds, advanced by the IMU
  uint32_t lastImuUs_;
  bool haveImu_;
  uint32_t imuStepsSinceGps_;
  double epochS_;         // receiver time, seconds, unwrapped from iTOW
  uint32_t lastEpochMs_;
  double offset_;         // local arrival time minus receiver time, smallest seen
  bool haveGps_, inertial_;
  float stationaryS_;
  uint32_t seeds_, duplicates_;
};
