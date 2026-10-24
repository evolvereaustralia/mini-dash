// Displayed speed (CLAUDE.md section 5, path 1): GPS Doppler speed fused with the integrated
// longitudinal acceleration in a complementary filter. The IMU carries the estimate between
// and through the 10 Hz GPS samples at the IMU rate; each GPS sample corrects it against the
// estimate as it was when the receiver measured (GPS_LATENCY_S ago), so an acceleration does
// not get corrected away by a sample that predates it. Valid only while GPS samples keep
// arriving (the caller applies the 500 ms staleness rule); the estimate keeps integrating
// through a gap so the picture is continuous when the fix returns. The timing path never
// sees this value: timers run on raw GPS samples. Plain C++, tested natively.
#pragma once

#include <stdint.h>

class SpeedFusion {
 public:
  SpeedFusion();
  void reset();
  // Longitudinal acceleration in g (vehicle frame, forward positive) for dt seconds.
  void imuStep(float lonG, float dt);
  // A fresh, valid GPS ground speed.
  void gpsSample(float speedMps);
  float speedMps() const { return v_; }
  float speedKph() const { return v_ * 3.6f; }
  // Residual longitudinal offset learnt at standstill, g (diagnostics).
  float lonBiasG() const { return lonBias_; }
  bool stationary() const { return stationaryS_ >= 1.0f; }

  static constexpr float GPS_LATENCY_S = 0.10f;  // receiver epoch to our sample; refine on the road
  static constexpr float GPS_GAIN = 0.35f;        // per 10 Hz sample: about 0.25 s time constant
  static constexpr float STILL_MPS = 0.3f;
  static constexpr float LAUNCH_G = 0.05f;  // forward g that ends a standstill before the GPS notices

 private:
  static constexpr int HIST = 32;
  static constexpr float HIST_DT = 0.02f;  // 640 ms of history
  float v_;
  float lonBias_;
  float hist_[HIST];
  int histHead_;
  float histAccumS_;
  float stationaryS_;
  float lonMean_;
  bool haveGps_;
};
