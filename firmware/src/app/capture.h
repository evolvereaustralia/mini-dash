// Raw capture for the first logged drives (review: "Raw records. None." was the answer to what
// evidence existed). Every GPS epoch, every IMU sample, the gate decisions, what the trip engine
// was fed, every OBD exchange and every timer run are put in a ring in PSRAM by the task that
// saw them, and the loop task writes them to the USB serial port as text lines, as fast as the
// port takes them. Nothing here blocks a producer: a full ring drops the record and counts it,
// and the count is in the stream, so a log says for itself whether it is complete.
//
// Two separate switches:
//  - streaming (serial key C on, U off): records are produced and written only while it is on.
//  - capture MODE (stored, serial MM toggles it and restarts): timer runs and peaks are logged
//    as candidates and never become records, and no trip, record or trace is written. This is
//    the default until the timers have been checked against a logged drive.
//
// Record types (a, b, v0..v5):
//  G GPS epoch as read    a sats, b fixType | fixOk<<4 | timeValid<<5 | repeatedEpoch<<6
//                         v0 iTOW ms, v1 ground speed mm/s, v2 horizontal accuracy mm, v3 altitude cm,
//                         v4 speed accuracy mm/s, v5 heading 0.01 deg. tUs is the arrival time.
//                         No latitude or longitude, on purpose: a log can be shared without
//                         saying where the car lives.
//  I IMU sample, raw      v0..v2 accelerometer in micro-g, v3..v5 gyro in 0.1 mdps (sensor
//                         frame, before the offset calibration). tUs is the read time.
//  A attitude, every 10th a flags: 1 trust, 2 atRest, 4 turnHold, 8 biasLearn, 16 plausible,
//    IMU sample             32 gpsSteady, 64 frameOk, 128 gpsFresh; b gyro norm 0.1 deg/s
//                         v0 lon micro-g, v1 lat micro-g (unfiltered, vehicle frame), v2 yaw mdps,
//                         v3 fused speed mm/s, v4 centripetal micro-g, v5 |a| micro-g
//  F display path, 20 Hz  v0 lat micro-g, v1 lon micro-g (5 Hz filtered), v2 tilt 0.01 deg,
//                         v3 rest |a| micro-g, v4 gyro bias mdps, v5 fusion lon bias micro-g
//  S engine feed, 20 Hz   a flags: 1 gpsFresh, 2 wheelsFresh, 4 carFresh, 8 motionFresh,
//                           16 socFresh, 32 speed source is wheels; b sats
//                         v0 displayed speed m/h (km/h x 1000), v1 raw GPS speed m/h,
//                         v2 wheel average m/h or -1, v3 battery power W, v4 SOC 0.01 % or -1,
//                         v5 trip distance cm
//  C candidate run        a record id (0 0-60, 1 0-100, 2 60-0, 3 lateral, 4 draw, 5 regen)
//                         v0 value x 1000, v1 aux x 1000, v2 engine time ms
//  O OBD exchange         a ECU, b identifier; v0 duration ms, v1 outcome (0 good, 1 unanswered,
//                         2 silent, 3 refused, 4 bad), v2 frames, v3 link state
//  B OBD link state       a new state (obd_client.h)
//  T task stack           a task (0 loop, 1 sensors, 2 OBD, 3 display); v0 least free stack bytes,
//                         v1 free heap, v2 least free heap, v3 free PSRAM
//  D dropped              v0 records dropped since boot (ring full)
// The loop task also prints a "~K,..." line with the settings a replay needs, at the start of
// a stream and every minute.
#pragma once

#include <stdint.h>

#include "capture_format.h"

namespace Capture {

bool begin();  // the ring, in PSRAM; false means no capture this boot (the dash is unaffected)

void setMode(bool captureMode);  // set once at boot from the stored flag
bool mode();

void setStreaming(bool on);
bool streaming();

// Any task; never blocks. Nothing is recorded unless streaming.
void pushAt(uint32_t tUs, uint8_t type, uint8_t a, uint16_t b, int32_t v0 = 0, int32_t v1 = 0, int32_t v2 = 0, int32_t v3 = 0,
            int32_t v4 = 0, int32_t v5 = 0);
void push(uint8_t type, uint8_t a, uint16_t b, int32_t v0 = 0, int32_t v1 = 0, int32_t v2 = 0, int32_t v3 = 0, int32_t v4 = 0,
          int32_t v5 = 0);
// A task's own stack margin, at most once every 10 s per task id (0..3).
void stackMark(uint8_t task);

void drain();  // loop task only: whole lines, only as many as the port has room for
uint32_t dropped();
uint32_t written();

}  // namespace Capture
