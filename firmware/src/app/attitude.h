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
  // Start from an up direction (sensor frame, any length): a measured specific force at rest, or
  // the stored mounting reference. Only the direction is taken. The magnitude the sensor reports
  // at rest is a separate fact, seeded from a measurement with seedRestNorm: taking it from a
  // unit-length stored reference used to set it to exactly 1 g, and on a part that reads more
  // than 3 percent off that the gate below then never opened (review finding 21).
  void reset(const float a[3]);
  void seedRestNorm(float magnitudeG);
  bool ready() const { return ready_; }

  // One IMU sample. a in g, w in deg/s (both sensor frame), dt in seconds.
  // trustAccel: pull toward the measured gravity with gain kp (1/s; 0.1 is a 10 s time
  // constant, a few units converge in a second). atRest: the rest magnitude is tracked.
  // learnBias: the true rate is known to be zero, so the gyro reading is the bias. The caller
  // sets it only on GPS evidence of a standstill: a quiet IMU alone cannot tell a standstill
  // from a long gentle curve, and learning a real yaw rate as bias corrupts every later
  // reading (review finding 21).
  void update(const float a[3], const float w[3], float dt, bool trustAccel, float kp, bool atRest, bool learnBias);

  void up(float out[3]) const;
  void gyroBias(float out[3]) const;
  // Whether a sample's magnitude is close enough to the magnitude the sensor reports at rest
  // (learnt while stationary; zero-g offsets and scale error make it 2 to 5 percent off 1 g on
  // this part, and in some orientations) to be free of linear acceleration: within 3 percent.
  bool accelPlausible(const float a[3]) const;
  float restNorm() const { return restNorm_; }
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
  float restNorm_;     // |a| at rest, learnt
};

// The gentle sustained turn (review finding 20). In a long sweeping bend the specific force is
// within 3 percent of 1 g, the gyro is far under its limit and the GPS speed is steady, so the
// three tests above all pass while the accelerometer is in fact reading gravity plus a steady
// centripetal acceleration; the filter then leans its "up" into the turn and the lateral g
// fades toward zero. The centripetal acceleration is known without the accelerometer: speed
// times yaw rate. This gate holds the accelerometer off while that exceeds HOLD_G, and lets it
// back only after it has stayed under RELEASE_G for RELEASE_S. The thresholds are the
// reviewer's starting values and are to be tuned from the first logged drive.
class TurnGate {
 public:
  static constexpr float HOLD_G = 0.05f, RELEASE_G = 0.03f, RELEASE_S = 0.5f;
  // speedMps from GPS, yawDps bias-corrected about up. Returns true while the accelerometer
  // must not be used as the gravity reference.
  bool update(float speedMps, float yawDps, float dt);
  bool holding() const { return hold_; }
  float centripetalG() const { return lastG_; }
  void reset() { hold_ = false; belowS_ = 0; lastG_ = 0; }

 private:
  bool hold_ = false;
  float belowS_ = 0, lastG_ = 0;
};

// Vehicle-frame G from one specific-force sample (g, sensor frame), the current up and the
// nominal forward axis of the mount (sensor frame, board_config.h): longitudinal positive
// forward, lateral positive right. Gravity is removed along up; forward is projected onto the
// horizontal plane so the mount's pitch and the car's pitching never leak into the result.
// Returns false, with both outputs zero, when the nominal forward axis stands within 30
// degrees of vertical (horizontal projection under FORWARD_MIN_PROJ): the axis setting is then
// wrong for this mounting and whatever was computed would be noise with a heading (finding 22).
constexpr float FORWARD_MIN_PROJ = 0.5f;
bool vehicleFrameG(const float a[3], const float up[3], const float forward[3], float *lonG, float *latG);
