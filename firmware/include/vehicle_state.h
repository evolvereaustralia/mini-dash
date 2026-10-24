// The one shared state between the sensor task, the OBD task (phase 3c), the demo replay and
// the UI (CLAUDE.md section 5). Every group carries a monotonic millis() stamp so the UI can
// grey out stale values: GPS after 500 ms, OBD fast group after 1 s, slow group after 5 s.
// Readers take a snapshot under the lock and work on the copy; writers update in place.
#pragma once

#include <stdint.h>

constexpr uint32_t STALE_GPS_MS = 500;
constexpr uint32_t STALE_OBD_FAST_MS = 1000;
constexpr uint32_t STALE_OBD_SLOW_MS = 5000;
constexpr uint32_t STALE_IMU_MS = 200;

struct GpsData {
  uint32_t stampMs = 0;  // 0 = never
  bool fixOk = false;    // gnssFixOk and 3D
  uint8_t fixType = 0;
  uint8_t sats = 0;
  float speedMps = 0, speedAccMps = 0, hAccM = 0;
  float headingDeg = 0, altM = 0;
  double lat = 0, lon = 0;
  uint32_t iTOW = 0;
  bool timeValid = false;
  uint16_t year = 0;
  uint8_t month = 0, day = 0, hour = 0, minute = 0, second = 0;
};

struct ImuData {
  uint32_t stampMs = 0;
  float ax = 0, ay = 0, az = 0;  // g, sensor frame
  float gx = 0, gy = 0, gz = 0;  // deg/s, sensor frame
  uint32_t samples = 0;
  float rateHz = 0;
};

// Vehicle-frame motion after gravity removal and the display low-pass (CLAUDE.md section 5,
// path 3): lateral positive right, longitudinal positive forward (acceleration).
struct MotionData {
  uint32_t stampMs = 0;
  float latG = 0, lonG = 0, combinedG = 0, yawDps = 0;
  float peakLeft = 0, peakRight = 0, peakAccel = 0, peakBrake = 0;  // session peaks, g
};

struct CarData {
  uint32_t fastStampMs = 0, slowStampMs = 0;
  bool linked = false;   // BLE connected to the dongle
  bool asleep = false;   // connected but requests time out
  float socPct = 0, socMinPct = 0, socMaxPct = 100;  // displayed SOC and usable window
  float packV = 0, packA = 0, powerKw = 0;           // positive = discharging
  float tempMin = 0, tempAvg = 0, tempMax = 0;
  float cellAvgV = 0, cellMinV = 0, cellMaxV = 0;
  float motorTemp1 = 0, motorTemp2 = 0;
  float torqueNm = 0, torqueReqNm = 0;
  float pedalPct = 0;
  float wheelKph[4] = {0, 0, 0, 0};
  float wheelAvgKph = 0;
  float v12 = 0;
  float rangeKm = 0;
  float sohPct = 0;
};

enum class SpeedSource : uint8_t { None, Gps, Wheels };

struct TripData {
  uint32_t startMs = 0;
  float distanceKm = 0, movingS = 0, maxKph = 0, avgMovingKph = 0;
  float drawKwh = 0, recoveredKwh = 0, netKwh = 0, whPerKm = 0;
  float socStartPct = -1, socUsedPct = 0;
  float altM = 0;
  float best60to100S = 0;
  bool energyValid = false;  // false until car data has been live
};

// Records (CLAUDE.md section 4, Trip): 0 = best 0-60 s, 1 = best 0-100 s, 2 = best 100-0 s
// (aux = distance m), 3 = max lateral g (aux = signed: negative left), 4 = peak draw kW
// (aux = instantaneous max), 5 = peak regen kW (aux = instantaneous max).
constexpr int REC_COUNT = 6;
enum RecordId { REC_0_60 = 0, REC_0_100, REC_100_0, REC_LAT, REC_DRAW, REC_REGEN };

struct Record {
  bool valid = false;
  float value = 0;
  float aux = 0;
  float socPct = 0, packTemp = 0;
  uint16_t runs = 0;     // valid runs counted for this metric in this scope
  char when[12] = "";    // "14:12" or "24 Oct"
  bool beatLifetime = false;
  float previous = 0;    // baseline for the comparison line
  bool hasPrevious = false;
  uint32_t tripId = 0;   // the trip the event belongs to (its trace file is keyed on it)
  uint32_t utc = 0;      // when, UTC seconds; 0 if no clock
};

struct RecordSet {
  Record r[REC_COUNT];
};

struct DisplayState {
  float speedKph = 0;
  SpeedSource speedSource = SpeedSource::None;
  float lux = -1;
  bool night = false;
  uint8_t backlightPct = 100;
  uint8_t bestField = 0;  // Drive footer trip-best choice, 0..5
};

struct Diagnostics {
  uint32_t i2cLongestHoldMs = 0, i2cOverHold = 0;
  uint32_t lvglFrames = 0;
  float gpsHz = 0, imuHz = 0, obdHz = 0;
  bool touchOk = false;
  char touchId[5] = "";
  // Attitude filter: sensor tilt from level, the correction applied per sample (settles near
  // zero), the learnt gyro bias magnitude and the longitudinal residual learnt at standstill.
  float attTiltDeg = 0, attCorrDeg = 0, gyroBiasDps = 0, lonBiasG = 0;
  bool attTrust = false, attStationary = false;
  float accNorm = 1, gyrNorm = 0, restNorm = 1;  // gate inputs: |a| g, bias-corrected |w| deg/s, learnt |a| at rest
  // Accelerometer offset calibration: 0 idle, 1 collecting, 2 done, 3 failed; directions stored.
  uint8_t accCalState = 0, accCalDirs = 0;
  bool accCalibrated = false;
};

// What the storage layer knows, for the Trip scopes and the Settings page.
struct StoreStats {
  bool fsOk = false;
  uint16_t storedTrips = 0;
  uint16_t todayTrips = 0;   // completed trips today, excluding the current one
  float todayKm = 0;
  uint16_t allTrips = 0;
  float allKm = 0;
  uint32_t usedKb = 0, totalKb = 0;
};

struct VehicleState {
  GpsData gps;
  ImuData imu;
  MotionData motion;
  CarData car;
  TripData trip;
  RecordSet recTrip, recToday, recAll;
  DisplayState disp;
  Diagnostics diag;
  StoreStats store;
  uint32_t utcNow = 0;        // from the RTC or GPS; 0 until a clock exists
  uint32_t tripId = 0;
  uint32_t tripStartUtc = 0;
  bool demo = false;  // demo replay is the source of speed, motion and car data
  bool sensorsReady = false;
};

namespace Vs {
void begin();
bool lock(uint32_t timeoutMs = 50);
void unlock();
VehicleState &raw();            // only while holding the lock
void snapshot(VehicleState &out);  // takes and releases the lock itself
}  // namespace Vs

class VsLock {
 public:
  explicit VsLock(uint32_t timeoutMs = 50) : held_(Vs::lock(timeoutMs)) {}
  ~VsLock() { if (held_) Vs::unlock(); }
  bool ok() const { return held_; }
  VsLock(const VsLock &) = delete;
  VsLock &operator=(const VsLock &) = delete;

 private:
  bool held_;
};
