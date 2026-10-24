#include "speed_fusion.h"

namespace {
constexpr float G0 = 9.80665f;
constexpr float BIAS_TAU_S = 5.0f;
}  // namespace

SpeedFusion::SpeedFusion() { reset(); }

void SpeedFusion::reset() {
  v_ = 0;
  lonBias_ = 0;
  for (int i = 0; i < HIST; i++) hist_[i] = 0;
  histHead_ = 0;
  histAccumS_ = 0;
  stationaryS_ = 0;
  lonMean_ = 0;
  haveGps_ = false;
}

void SpeedFusion::imuStep(float lonG, float dt) {
  if (dt <= 0 || dt > 0.5f) return;
  if (stationary() && lonG - lonBias_ > LAUNCH_G) stationaryS_ = 0;  // the IMU sees the launch first
  if (stationary()) {
    // At a standstill the true forward acceleration is zero: whatever remains is the mount's
    // residual, learnt slowly and removed from the integration (not from the displayed G).
    lonMean_ += (lonG - lonMean_) * dt / BIAS_TAU_S;
    lonBias_ = lonMean_;
    v_ = 0;
  } else {
    v_ += (lonG - lonBias_) * G0 * dt;
    if (v_ < 0) v_ = 0;
  }
  histAccumS_ += dt;
  if (histAccumS_ >= HIST_DT) {
    histAccumS_ -= HIST_DT;
    hist_[histHead_] = v_;
    histHead_ = (histHead_ + 1) % HIST;
  }
}

void SpeedFusion::gpsSample(float speedMps) {
  if (speedMps < 0) speedMps = 0;
  if (!haveGps_) {
    v_ = speedMps;
    haveGps_ = true;
  }
  // Standstill detection drives the bias learning and pins the speed to zero.
  if (speedMps < STILL_MPS) stationaryS_ += 0.1f;
  else stationaryS_ = 0;
  // Compare against the estimate as it was when the receiver measured.
  int back = (int)(GPS_LATENCY_S / HIST_DT + 0.5f);
  if (back >= HIST) back = HIST - 1;
  float then = hist_[(histHead_ - 1 - back + 2 * HIST) % HIST];
  float innovation = speedMps - then;
  v_ += GPS_GAIN * innovation;
  if (v_ < 0) v_ = 0;
  // The correction applies to the whole history too, so the next sample sees a consistent past.
  for (int i = 0; i < HIST; i++) {
    hist_[i] += GPS_GAIN * innovation;
    if (hist_[i] < 0) hist_[i] = 0;
  }
}
