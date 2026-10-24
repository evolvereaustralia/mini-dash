#include "attitude.h"

#include <math.h>

namespace {
constexpr float DEG = 0.017453292f;
constexpr float BIAS_TAU_FAST_S = 2.0f;   // gyro bias learning at standstill for the first...
constexpr float BIAS_FAST_FOR_S = 10.0f;  // ...seconds of standstill since boot, then
constexpr float BIAS_TAU_S = 20.0f;       // slow tracking of the bias as the part warms
constexpr float BIAS_KI = 0.05f;          // bias learnt from the gravity error in steady motion, 1/s^2
constexpr float MAX_BIAS_DPS = 5.0f;
constexpr float REST_NORM_TAU_S = 5.0f;
constexpr float PLAUSIBLE_FRAC = 0.03f;

void normalise3(float v[3]) {
  float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (n > 1e-9f) {
    v[0] /= n;
    v[1] /= n;
    v[2] /= n;
  }
}
void normalise4(float q[4]) {
  float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n > 1e-9f)
    for (int i = 0; i < 4; i++) q[i] /= n;
}
// q = q * r (Hamilton product, both (w, x, y, z))
void multiply(float q[4], const float r[4]) {
  float w = q[0] * r[0] - q[1] * r[1] - q[2] * r[2] - q[3] * r[3];
  float x = q[0] * r[1] + q[1] * r[0] + q[2] * r[3] - q[3] * r[2];
  float y = q[0] * r[2] - q[1] * r[3] + q[2] * r[0] + q[3] * r[1];
  float z = q[0] * r[3] + q[1] * r[2] - q[2] * r[1] + q[3] * r[0];
  q[0] = w;
  q[1] = x;
  q[2] = y;
  q[3] = z;
}
// Rotate q by the body-frame rotation vector rv (radians): q = q * exp(rv / 2).
void rotateBody(float q[4], const float rv[3]) {
  float angle = sqrtf(rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2]);
  if (angle < 1e-9f) return;
  float s = sinf(angle * 0.5f) / angle;
  float dq[4] = {cosf(angle * 0.5f), rv[0] * s, rv[1] * s, rv[2] * s};
  multiply(q, dq);
  normalise4(q);
}
}  // namespace

Attitude::Attitude() : q_{1, 0, 0, 0}, bias_{0, 0, 0}, ready_(false), lastCorrDeg_(0), stationaryS_(0), restNorm_(1.0f) {}

void Attitude::reset(const float a[3]) {
  // The shortest rotation taking the measured up (sensor frame) onto world Z.
  float u[3] = {a[0], a[1], a[2]};
  float n = sqrtf(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
  if (n > 0.5f && n < 1.5f) restNorm_ = n;
  normalise3(u);
  float d = u[2];  // dot with (0, 0, 1)
  if (d < -0.9999f) {
    q_[0] = 0; q_[1] = 1; q_[2] = 0; q_[3] = 0;  // upside down: 180 degrees about X
  } else {
    // axis = u x z = (u1, -u0, 0); q = (1 + d, axis) normalised
    q_[0] = 1.0f + d;
    q_[1] = u[1];
    q_[2] = -u[0];
    q_[3] = 0;
    normalise4(q_);
  }
  ready_ = true;
  lastCorrDeg_ = 0;
}

void Attitude::up(float out[3]) const {
  // World Z expressed in the sensor frame: the third row of the sensor-to-world matrix.
  const float w = q_[0], x = q_[1], y = q_[2], z = q_[3];
  out[0] = 2.0f * (x * z - w * y);
  out[1] = 2.0f * (y * z + w * x);
  out[2] = 1.0f - 2.0f * (x * x + y * y);
}

void Attitude::update(const float a[3], const float w[3], float dt, bool trustAccel, float kp, bool stationary) {
  if (!ready_) {
    reset(a);
    return;
  }
  if (dt <= 0 || dt > 0.5f) return;
  float rate[3] = {w[0] - bias_[0], w[1] - bias_[1], w[2] - bias_[2]};  // deg/s
  if (stationary) {
    // The true rate is zero: the bias is the low-passed gyro reading. Fast while the estimate
    // is young (the car usually stands for a while after power-on), then slow tracking.
    float k = dt / (stationaryS_ < BIAS_FAST_FOR_S ? BIAS_TAU_FAST_S : BIAS_TAU_S);
    stationaryS_ += dt;
    for (int i = 0; i < 3; i++) bias_[i] += k * (w[i] - bias_[i]);
    float n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (n > 0.5f && n < 1.5f) restNorm_ += (n - restNorm_) * dt / REST_NORM_TAU_S;
  }
  float rv[3] = {rate[0] * DEG * dt, rate[1] * DEG * dt, rate[2] * DEG * dt};
  lastCorrDeg_ = 0;
  if (trustAccel && kp > 0) {
    // Error = measured up x estimated up (sensor frame): the small rotation that would carry
    // the estimate onto the measurement. Applied as an extra body rotation, gain kp per second.
    float m[3] = {a[0], a[1], a[2]};
    normalise3(m);
    float e[3];
    up(e);
    float c[3] = {m[1] * e[2] - m[2] * e[1], m[2] * e[0] - m[0] * e[2], m[0] * e[1] - m[1] * e[0]};
    float g = kp * dt;
    if (g > 1.0f) g = 1.0f;
    for (int i = 0; i < 3; i++) rv[i] += g * c[i];
    lastCorrDeg_ = g * sqrtf(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]) / DEG;
    // In steady motion the persistent part of the error is gyro bias about the horizontal
    // axes (the integral term of a Mahony filter); the bias about up is unobservable here.
    if (!stationary)
      for (int i = 0; i < 3; i++) bias_[i] -= BIAS_KI * c[i] * dt / DEG;
  }
  for (int i = 0; i < 3; i++) {
    if (bias_[i] > MAX_BIAS_DPS) bias_[i] = MAX_BIAS_DPS;
    if (bias_[i] < -MAX_BIAS_DPS) bias_[i] = -MAX_BIAS_DPS;
  }
  rotateBody(q_, rv);
}

bool Attitude::accelPlausible(const float a[3]) const {
  float n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  return fabsf(n - restNorm_) < PLAUSIBLE_FRAC * restNorm_;
}

void Attitude::gyroBias(float out[3]) const {
  for (int i = 0; i < 3; i++) out[i] = bias_[i];
}

float Attitude::yawRateDps(const float w[3]) const {
  float u[3];
  up(u);
  return (w[0] - bias_[0]) * u[0] + (w[1] - bias_[1]) * u[1] + (w[2] - bias_[2]) * u[2];
}

float Attitude::tiltDeg() const {
  float u[3];
  up(u);
  float c = u[2];
  if (c > 1) c = 1;
  if (c < -1) c = -1;
  return acosf(c) / DEG;
}

void vehicleFrameG(const float a[3], const float up[3], const float forward[3], float *lonG, float *latG) {
  float lin[3] = {a[0] - up[0], a[1] - up[1], a[2] - up[2]};
  float d = forward[0] * up[0] + forward[1] * up[1] + forward[2] * up[2];
  float f[3] = {forward[0] - d * up[0], forward[1] - d * up[1], forward[2] - d * up[2]};
  normalise3(f);
  // right = forward x up (x forward, y left, z up is right-handed; right is -y)
  float r[3] = {f[1] * up[2] - f[2] * up[1], f[2] * up[0] - f[0] * up[2], f[0] * up[1] - f[1] * up[0]};
  *lonG = lin[0] * f[0] + lin[1] * f[1] + lin[2] * f[2];
  *latG = lin[0] * r[0] + lin[1] * r[1] + lin[2] * r[2];
}
