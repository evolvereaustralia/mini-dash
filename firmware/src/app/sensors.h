// Sensor task on core 0 (CLAUDE.md section 5): GPS PVT at 10 Hz, IMU at 208 Hz, lux at 1 Hz,
// all through the one I2C mutex, written into VehicleState. Also drives night mode from the
// lux reading with hysteresis (under 15 lux for 5 s enters, over 40 lux for 5 s exits) and
// the backlight through the IO expander.
//
// Attitude: the gyro-propagated quaternion of section 5 (attitude.h), seeded by the stored
// mounting reference, corrected toward gravity only at rest or in steady motion; vehicle-frame
// G from it and a 5 Hz low-pass for the display path. Displayed speed is GPS fused with the
// integrated longitudinal acceleration (speed_fusion.h); the timers get raw GPS.
#pragma once

#include <stdint.h>

class TripEngine;

namespace Sensors {

// Probe and configure the sensors (prints what it found), then start the task.
void start(TripEngine *engine);

bool gpsOk();
bool imuOk();
bool luxOk();

// Take the current gravity vector as the mounting reference (car level and still).
void calibrateMounting();
// The stored mounting reference (unit gravity vector in the sensor frame); false if never taken.
bool mountingRef(float out[3]);
// True once after the sensor task has taken a new reference, so the caller can store it.
bool mountRefTake(float out[3]);
void setMountingRef(const float ref[3]);

// Accelerometer offset calibration (docs/calibration.md): start collecting while the user
// turns the device; the sensor task finishes it on its own. accelCalTake returns true once
// when a result is ready (ok says whether it was applied) so the caller can persist it.
void startAccelCal();
bool accelCalTake(bool *ok, float offset[3], float *radius, float *rmsResidual, int *directions);
void setAccelCal(const float offset[3], float radius);

// Night mode override: -1 auto (lux), 0 day, 1 night. Brightness 0..100 when not auto.
void setNightMode(int mode);
void setBrightness(uint8_t percent);

// GPS poll diagnostics since the last call: longest poll, the receiver backlog in bytes seen
// before it, NMEA bytes received in total (should stay 0), and the average poll time.
void gpsStats(uint32_t *pollMaxMs, uint32_t *backlogAtMax, uint32_t *nmeaBytes, uint32_t *pollAvgMs);
// Polls the receiver refused by stretching the clock past the chip's timeout (since boot).
uint32_t gpsRefusedReads();

}  // namespace Sensors
