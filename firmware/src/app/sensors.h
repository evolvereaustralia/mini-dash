// Sensor task on core 0 (CLAUDE.md section 5): GPS PVT at 10 Hz, IMU at 208 Hz, lux at 1 Hz,
// all through the one I2C mutex, written into VehicleState. Also drives night mode from the
// lux reading with hysteresis (under 15 lux for 5 s enters, over 40 lux for 5 s exits) and
// the backlight through the IO expander.
//
// Attitude in this phase: a static gravity reference taken while stationary (the mounting
// calibration) and a 5 Hz low-pass for the display path. The gyro-propagated quaternion of
// section 5 replaces the static reference in the next step; the interfaces do not change.
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
void setMountingRef(const float ref[3]);

// Night mode override: -1 auto (lux), 0 day, 1 night. Brightness 0..100 when not auto.
void setNightMode(int mode);
void setBrightness(uint8_t percent);

// GPS poll diagnostics since the last call: longest poll, the receiver backlog in bytes seen
// before it, NMEA bytes received in total (should stay 0), and the average poll time.
void gpsStats(uint32_t *pollMaxMs, uint32_t *backlogAtMax, uint32_t *nmeaBytes, uint32_t *pollAvgMs);

}  // namespace Sensors
