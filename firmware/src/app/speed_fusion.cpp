#include "speed_fusion.h"

namespace {
constexpr float G0 = 9.80665f;
constexpr float BIAS_TAU_S = 5.0f;
constexpr double WEEK_MS = 604800000.0;
constexpr double OFFSET_RELAX = 0.002;  // how fast the arrival-delay floor may rise, per fix
}  // namespace

SpeedFusion::SpeedFusion() { reset(); }

void SpeedFusion::reset() {
  v_ = 0;
  lonBias_ = lonMean_ = 0;
  histHead_ = histCount_ = 0;
  t_ = 0;
  lastImuUs_ = 0;
  haveImu_ = false;
  imuStepsSinceGps_ = 0;
  epochS_ = 0;
  lastEpochMs_ = 0;
  offset_ = 0;
  haveGps_ = inertial_ = false;
  stationaryS_ = 0;
  seeds_ = duplicates_ = 0;
}

void SpeedFusion::push() {
  hist_[histHead_] = {t_, v_};
  histHead_ = (histHead_ + 1) % HIST;
  if (histCount_ < HIST) histCount_++;
}

// The estimate as it was at local time t, by linear interpolation; false when t is older
// than the history (or there is none).
bool SpeedFusion::at(double t, float *v) const {
  if (histCount_ == 0) return false;
  int newest = (histHead_ - 1 + HIST) % HIST;
  if (t >= hist_[newest].t) {
    double span = t_ - hist_[newest].t;
    double f = span > 1e-6 ? (t - hist_[newest].t) / span : 1.0;
    if (f > 1.0) f = 1.0;
    *v = hist_[newest].v + (float)f * (v_ - hist_[newest].v);
    return true;
  }
  for (int k = 0; k < histCount_ - 1; k++) {
    int hi = (histHead_ - 1 - k + HIST) % HIST;
    int lo = (hi - 1 + HIST) % HIST;
    if (t >= hist_[lo].t) {
      double span = hist_[hi].t - hist_[lo].t;
      double f = span > 1e-6 ? (t - hist_[lo].t) / span : 0.0;
      *v = hist_[lo].v + (float)f * (hist_[hi].v - hist_[lo].v);
      return true;
    }
  }
  return false;
}

void SpeedFusion::seed(float speedMps) {
  v_ = speedMps;
  histHead_ = histCount_ = 0;
  push();
  seeds_++;
}

void SpeedFusion::imuStep(float lonG, uint32_t nowUs) {
  if (!haveImu_) {
    haveImu_ = true;
    lastImuUs_ = nowUs;
    return;
  }
  float dt = (float)(uint32_t)(nowUs - lastImuUs_) * 1e-6f;
  lastImuUs_ = nowUs;
  if (dt <= 0) return;
  t_ += dt;  // local time always advances, however long the gap
  imuStepsSinceGps_++;
  if (dt > 0.5f) {  // nothing is known about that interval: the next fix reseeds
    imuStepsSinceGps_ = 0;
    return;
  }
  if (stationary() && lonG - lonBias_ > LAUNCH_G) stationaryS_ = 0;  // the IMU sees the launch first
  if (stationary()) {
    // At a standstill the true forward acceleration is zero: whatever remains is the mount's
    // residual, learnt slowly and removed from the integration (not from the displayed G).
    lonMean_ += (lonG - lonMean_) * dt / BIAS_TAU_S;
    lonBias_ = lonMean_;
    v_ = 0;
  } else if (haveGps_) {
    v_ += (lonG - lonBias_) * G0 * dt;
    if (v_ < 0) v_ = 0;
  }
  int newest = (histHead_ - 1 + HIST) % HIST;
  if (histCount_ == 0 || t_ - hist_[newest].t >= HIST_MIN_DT) push();
}

void SpeedFusion::gpsSample(float speedMps, uint32_t epochMs, uint32_t nowUs) {
  if (!(speedMps >= 0)) speedMps = 0;  // negative or NaN
  double deltaS = 0;
  bool gap = false;
  if (haveGps_) {
    double d = (double)epochMs - (double)lastEpochMs_;
    if (d < -WEEK_MS / 2) d += WEEK_MS;  // the week rolled over
    if (d == 0) {
      duplicates_++;
      return;  // the same epoch again: nothing new to learn
    }
    if (d < 0) gap = true;  // time went backwards: the receiver restarted
    else {
      deltaS = d / 1000.0;
      gap = d > GAP_MS;
    }
    epochS_ += gap ? 0 : deltaS;
  }
  lastEpochMs_ = epochMs;

  if (speedMps < STILL_MPS) stationaryS_ += gap ? 0 : (float)deltaS;
  else stationaryS_ = 0;

  // Local arrival time: the IMU's clock carried forward to now.
  double arrival = t_ + (haveImu_ ? (double)(uint32_t)(nowUs - lastImuUs_) * 1e-6 : 0.0);
  bool noInertia = !haveImu_ || imuStepsSinceGps_ == 0;
  imuStepsSinceGps_ = 0;
  if (!haveGps_ || gap || noInertia) {
    // Nothing trustworthy to correct: take the Doppler speed as it is.
    haveGps_ = true;
    if (gap || epochS_ == 0) epochS_ = arrival - GPS_LATENCY_S;  // restart the mapping too
    offset_ = arrival - epochS_;
    seed(speedMps);
    inertial_ = false;
    return;
  }
  double delay = arrival - epochS_;
  if (delay < offset_) offset_ = delay;
  else offset_ += (delay - offset_) * OFFSET_RELAX;
  // offset_ holds the smallest arrival delay, which is the receiver's own latency: the
  // measurement was taken that long before the earliest possible arrival.
  double measured = epochS_ + offset_ - GPS_LATENCY_S;
  float then;
  if (!at(measured, &then)) {  // older than the history: a very late delivery
    seed(speedMps);
    inertial_ = false;
    return;
  }
  float correction = GPS_GAIN * (speedMps - then);
  v_ += correction;
  if (v_ < 0) v_ = 0;
  for (int k = 0; k < histCount_; k++) {  // from the measurement instant forward only
    int i = (histHead_ - 1 - k + HIST) % HIST;
    if (hist_[i].t < measured) break;
    hist_[i].v += correction;
    if (hist_[i].v < 0) hist_[i].v = 0;
  }
  inertial_ = true;
}
