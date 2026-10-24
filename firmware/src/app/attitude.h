// Attitude of the IMU (CLAUDE.md section 5): a quaternion propagated from the gyro at the IMU
// rate and pulled slowly toward the accelerometer's gravity direction only when the caller
// says the accelerometer can be trusted (near-stationary or steady motion: specific force
// close to 1 g, low gyro rates, GPS speed steady). Static mounting calibration alone is not
// enough: one degree of pitch error projects 0.017 g into the longitudinal axis, and the car
// pitches under acceleration and braking. The gyro bias is learnt while the car stands still
// (the true rate is then zero). Plain C++, no Arduino, so the maths is tested natively
// (test/native/test_fusion.cpp).
//
// Frames: "sensor" is the IMU's own axes; "world" has Z up. up() returns the unit vector that
// a resting accelerometer reports (the reaction to gravity, +1 g) in the sensor frame. The
// vehicle frame is derived by the caller from a nominal forward axis (board_config.h) and up.
//
// Accelerometer zero-g offset (tens of mg on this part) is deliberately not estimated: at rest
// the filter absorbs it into the up direction, and because the offset is fixed in the sensor
// frame while the gyro tracks every orientation change, the residual error while moving is
// second order (offset times the pitch change, well under 1 mg).
#pragma once

#include <stdint.h>

class Attitude {
 public:
  Attitude();
  // Start from the specific force measured at rest (sensor frame, any length).
  void reset(const float a[3]);
  bool ready() const { return ready_; }

  // One IMU sample. a in g, w in deg/s (both sensor frame), dt in seconds.
  // trustAccel: pull toward the measured gravity with gain kp (1/s; 0.1 is a 10 s time
  // constant, a few units converge in a second). stationary: also learn the gyro bias.
  void update(const float a[3], const float w[3], float dt, bool trustAccel, float kp, bool stationary);

  void up(float out[3]) const;
  void gyroBias(float out[3]) const;
  // Bias-corrected rate about the up axis, deg/s, positive anticlockwise seen from above.
  float yawRateDps(const float w[3]) const;
  // Angle between the sensor's +Z and world up, degrees (diagnostics).
  float tiltDeg() const;
  // Correction applied in the last update, degrees (diagnostics: should settle near zero).
  float lastCorrectionDeg() const { return lastCorrDeg_; }

 private:
  float q_[4];  // sensor to world, (w, x, y, z)
  float bias_[3];
  bool ready_;
  float lastCorrDeg_;
  float stationaryS_;  // standstill seen so far, for the bias learning schedule
};

// Vehicle-frame G from one specific-force sample (g, sensor frame), the current up and the
// nominal forward axis of the mount (sensor frame, board_config.h): longitudinal positive
// forward, lateral positive right. Gravity is removed along up; forward is projected onto the
// horizontal plane so the mount's pitch and the car's pitching never leak into the result.
void vehicleFrameG(const float a[3], const float up[3], const float forward[3], float *lonG, float *latG);
