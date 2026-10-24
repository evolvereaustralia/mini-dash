#include "sensors.h"

#include <Adafruit_VEML7700.h>
#include <Arduino.h>
#include <SparkFun_ISM330DHCX.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Wire.h>
#include <math.h>

#include "attitude.h"
#include "board_config.h"
#include "clock.h"
#include "demo_source.h"
#include "i2c_bus.h"
#include "io_expander.h"
#include "speed_fusion.h"
#include "trip_engine.h"
#include "vehicle_state.h"

namespace {
SFE_UBLOX_GNSS g_gnss;

// Diagnostics for the GPS bus holds: count any NMEA bytes the receiver still sends over I2C
// (it should send none), time each poll and remember the backlog seen before the longest one.
class CountingPrint : public Print {
 public:
  size_t write(uint8_t) override { count++; return 1; }
  volatile uint32_t count = 0;
} g_nmeaSink;
uint32_t g_gpsPollMaxMs = 0, g_gpsPollMaxBacklog = 0, g_gpsPolls = 0, g_gpsPollTotalMs = 0, g_gpsRefused = 0;

uint16_t gpsBytesAvailable() {  // u-blox register 0xFD/0xFE, high bit is an error flag
  Wire.beginTransmission(I2C_ADDR_NEO_M9N);
  Wire.write(0xFD);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(I2C_ADDR_NEO_M9N, (uint8_t)2) != 2) return 0;
  uint16_t v = (uint16_t)Wire.read() << 8;
  v |= Wire.read();
  return v & 0x7FFF;
}
SparkFun_ISM330DHCX g_imu;
Adafruit_VEML7700 g_lux;
bool g_gpsOk = false, g_imuOk = false, g_luxOk = false;
TripEngine *g_engine = nullptr;

// Mounting reference: unit gravity vector in the sensor frame at rest, stored in NVS. It only
// seeds the attitude filter so the G display is right from the first sample; the filter then
// tracks the real orientation (CLAUDE.md section 5).
float g_gRef[3] = {0, 0, 1};
bool g_calibrated = false;
volatile bool g_calibrateRequest = false;

Attitude g_att;
SpeedFusion g_fusion;
uint32_t g_lastImuUs = 0, g_attStartMs = 0;
// Last GPS ground speed and a 1 s history, for the "steady motion" and standstill decisions.
float g_gpsSpeed = 0, g_gpsSpeedHist[10] = {};
int g_gpsSpeedIdx = 0;
uint32_t g_gpsSpeedMs = 0;
bool g_attTrust = false, g_attStationary = false;
float g_accNorm = 1, g_gyrNorm = 0;

// Display path low-pass state (about 5 Hz at 208 Hz).
float g_latLp = 0, g_lonLp = 0;
constexpr float LP_ALPHA = 0.14f;

int g_nightMode = -1;
uint8_t g_manualBrightness = 100;
bool g_night = false;
uint32_t g_luxSinceMs = 0;
bool g_luxBelow = false;

bool gpsBegin() {
  I2cLock guard(800, "gps.begin");
  if (!guard.ok() || !g_gnss.begin(Wire, I2C_ADDR_NEO_M9N)) return false;
  g_gnss.setI2CTransactionSize(128);  // the ESP32 Wire buffer; four times fewer transactions per packet
  g_gnss.setNMEAOutputPort(g_nmeaSink);
  bool ubxOnly = false;
  for (int i = 0; i < 3 && !ubxOnly; i++) ubxOnly = g_gnss.setI2COutput(COM_TYPE_UBX);
  bool rate = g_gnss.setNavigationFrequency(GPS_NAV_RATE_HZ);
  bool autoPvt = g_gnss.setAutoPVT(true);
  Serial.printf("GPS: I2C UBX-only %s, nav rate %s, autoPVT %s\n", ubxOnly ? "ok" : "NOT ACKED", rate ? "ok" : "NOT ACKED",
                autoPvt ? "ok" : "NOT ACKED");
  return true;
}

bool imuBegin() {
  I2cLock guard(200, "imu.begin");
  if (!guard.ok() || !g_imu.begin(Wire, I2C_ADDR_ISM330DHCX)) return false;
  g_imu.deviceReset();
  uint32_t t0 = millis();
  while (!g_imu.getDeviceReset()) {
    if (millis() - t0 > 100) return false;
    delay(1);
  }
  g_imu.setDeviceConfig();
  g_imu.setBlockDataUpdate();
  g_imu.setAccelDataRate(ISM_XL_ODR_208Hz);
  g_imu.setAccelFullScale(ISM_4g);
  g_imu.setGyroDataRate(ISM_GY_ODR_208Hz);
  g_imu.setGyroFullScale(ISM_500dps);
  g_imu.setAccelFilterLP2();
  g_imu.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);
  g_imu.setGyroFilterLP1();
  g_imu.setGyroLP1Bandwidth(ISM_MEDIUM);
  return true;
}

bool luxBegin() {
  I2cLock guard(300, "lux.begin");
  if (!guard.ok() || !g_lux.begin(&Wire)) return false;
  g_lux.setGain(VEML7700_GAIN_1_8);
  g_lux.setIntegrationTime(VEML7700_IT_100MS);
  return true;
}

// Night level: 25 percent read as "dims, still readable" with a finger over the sensor in a
// lit room, but in a 2 lux room Logan could not see an image at all (24 Oct 2026). The PWM
// to brightness curve of this backlight is steep at the dim end; 45 is the next guess to be
// judged by eye on the first night drive (measured brightness is an open test).
constexpr uint8_t NIGHT_BACKLIGHT_PCT = 45;
void applyBacklight() {
  uint8_t pct;
  if (g_nightMode == 0) pct = g_manualBrightness;
  else if (g_nightMode == 1) pct = g_manualBrightness < 60 ? g_manualBrightness : NIGHT_BACKLIGHT_PCT;
  else pct = g_night ? NIGHT_BACKLIGHT_PCT : 100;
  IoExpander::setBacklightPercent(pct);
  VsLock lk;
  if (lk.ok()) {
    Vs::raw().disp.backlightPct = pct;
    Vs::raw().disp.night = (g_nightMode == 1) || (g_nightMode == -1 && g_night);
  }
}

void pollGps(uint32_t now) {
  GpsData g;
  {
    I2cLock guard(20, "gps");
    if (!guard.ok()) return;
    uint32_t t0 = millis();
    uint16_t backlog = gpsBytesAvailable();
    uint32_t t1 = millis();
    bool fresh = g_gnss.getPVT();
    uint32_t dt = millis() - t0;
    g_gpsPolls++;
    g_gpsPollTotalMs += dt;
    if (dt > g_gpsPollMaxMs) {
      g_gpsPollMaxMs = dt;
      g_gpsPollMaxBacklog = backlog;
    }
    // The occasional 26 ms poll (33 in 31 minutes on the bench, 25 Oct 2026): the receiver
    // holds SCL low longer than the ESP32-S3's maximum clock-stretch timeout while it is busy
    // with its navigation solution, the transaction aborts (ESP_ERR_INVALID_STATE in the log)
    // and the PVT arrives at the next poll. Counted here, not fixed: a shorter hardware
    // timeout would fail more polls, and there is no free GPIO for the receiver's TX-ready line.
    if (dt >= 20 && !fresh) g_gpsRefused++;
    (void)t1;
    if (!fresh) return;
    g.stampMs = now;
    g.fixType = g_gnss.getFixType();
    g.fixOk = g_gnss.getGnssFixOk() && g.fixType >= 3;
    g.sats = g_gnss.getSIV();
    g.speedMps = g_gnss.getGroundSpeed() / 1000.0f;
    g.speedAccMps = g_gnss.getSpeedAccEst() / 1000.0f;
    g.hAccM = g_gnss.getHorizontalAccEst() / 1000.0f;
    g.headingDeg = g_gnss.getHeading() / 100000.0f;
    g.altM = g_gnss.getAltitudeMSL() / 1000.0f;
    g.lat = g_gnss.getLatitude() / 1e7;
    g.lon = g_gnss.getLongitude() / 1e7;
    g.iTOW = g_gnss.getTimeOfWeek();
    g.timeValid = g_gnss.getTimeValid() && g_gnss.getDateValid();
    g.year = g_gnss.getYear();
    g.month = g_gnss.getMonth();
    g.day = g_gnss.getDay();
    g.hour = g_gnss.getHour();
    g.minute = g_gnss.getMinute();
    g.second = g_gnss.getSecond();
  }  // the bus is released here: the RTC write below takes its own lock
  if (g.timeValid && g.fixOk) Clock::setFromGps(g.year, g.month, g.day, g.hour, g.minute, g.second);
  if (g.fixOk) {
    g_fusion.gpsSample(g.speedMps);
    g_gpsSpeed = g.speedMps;
    g_gpsSpeedHist[g_gpsSpeedIdx] = g.speedMps;
    g_gpsSpeedIdx = (g_gpsSpeedIdx + 1) % 10;
    g_gpsSpeedMs = now;
  }
  VsLock lk;
  if (lk.ok()) Vs::raw().gps = g;
}

void pollImu(uint32_t now) {
  sfe_ism_data_t a, w;
  {
    I2cLock guard(10, "imu");
    if (!guard.ok() || !g_imu.checkStatus()) return;
    if (!g_imu.getAccel(&a) || !g_imu.getGyro(&w)) return;
  }
  uint32_t us = micros();
  float dt = g_lastImuUs ? (float)(us - g_lastImuUs) * 1e-6f : 1.0f / IMU_ODR_HZ;
  g_lastImuUs = us;
  float acc[3] = {a.xData / 1000.0f, a.yData / 1000.0f, a.zData / 1000.0f};
  float gyr[3] = {w.xData / 1000.0f, w.yData / 1000.0f, w.zData / 1000.0f};
  if (g_calibrateRequest) {
    float n = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (n > 0.5f) {
      for (int i = 0; i < 3; i++) g_gRef[i] = acc[i] / n;
      g_calibrated = true;
      g_att.reset(acc);
      g_fusion.reset();
    }
    g_calibrateRequest = false;
  }
  if (!g_att.ready()) g_att.reset(g_calibrated ? g_gRef : acc);

  // When may the accelerometer serve as the gravity reference? Specific force within 3 percent
  // of 1 g (no linear acceleration, no cornering), gyro rates under 10 deg/s, and the GPS speed
  // steady over the last second when there is one. Standstill (GPS under 0.3 m/s, or no GPS
  // and the IMU quiet) trusts gravity fully and learns the gyro bias; the first 3 s after the
  // filter starts converge fast, the car being at rest when it is switched on.
  float an = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
  float bias[3];
  g_att.gyroBias(bias);
  float wn = sqrtf((gyr[0] - bias[0]) * (gyr[0] - bias[0]) + (gyr[1] - bias[1]) * (gyr[1] - bias[1]) +
                   (gyr[2] - bias[2]) * (gyr[2] - bias[2]));
  bool gpsFresh = g_gpsSpeedMs && now - g_gpsSpeedMs < STALE_GPS_MS;
  bool steady = gpsFresh ? fabsf(g_gpsSpeed - g_gpsSpeedHist[g_gpsSpeedIdx]) < 0.4f : true;
  bool plausible = g_att.accelPlausible(acc);
  bool trust = plausible && wn < 10.0f && steady;
  bool stationary = (gpsFresh ? g_gpsSpeed < 0.3f : plausible) && wn < (gpsFresh ? 3.0f : 1.0f);
  g_accNorm = an;
  g_gyrNorm = wn;
  if (g_attStartMs == 0) g_attStartMs = now;
  float kp = now - g_attStartMs < 3000 ? 2.0f : (stationary ? 1.0f : 0.1f);
  g_att.update(acc, gyr, dt, trust, kp, stationary);
  g_attTrust = trust;
  g_attStationary = stationary;

  float up[3];
  g_att.up(up);
  float lon, lat;
  vehicleFrameG(acc, up, IMU_FORWARD_SENSOR, &lon, &lat);
  g_latLp += LP_ALPHA * (lat - g_latLp);
  g_lonLp += LP_ALPHA * (lon - g_lonLp);
  g_fusion.imuStep(lon, dt);
  float yaw = g_att.yawRateDps(gyr);

  VsLock lk(5);
  if (!lk.ok()) return;
  VehicleState &vs = Vs::raw();
  vs.imu.stampMs = now;
  vs.imu.ax = acc[0];
  vs.imu.ay = acc[1];
  vs.imu.az = acc[2];
  vs.imu.gx = gyr[0];
  vs.imu.gy = gyr[1];
  vs.imu.gz = gyr[2];
  vs.imu.samples++;
  if (!vs.demo) {
    vs.motion.stampMs = now;
    vs.motion.latG = g_latLp;
    vs.motion.lonG = g_lonLp;
    vs.motion.combinedG = sqrtf(g_latLp * g_latLp + g_lonLp * g_lonLp);
    vs.motion.yawDps = yaw;
  }
}

uint32_t g_luxFirstMs = 0;

void pollLux(uint32_t now) {
  float lux;
  {
    I2cLock guard(20, "lux");
    if (!guard.ok()) return;
    lux = g_lux.readLux(VEML_LUX_NORMAL_NOWAIT);
  }
  // The sensor reads 0 until its first integration completes, and the first readings after
  // begin() are not to be trusted either; a false "night" at boot dimmed the panel to 25 %
  // (24 Oct 2026). Ignore the first two seconds and any exact zero.
  if (g_luxFirstMs == 0) g_luxFirstMs = now;
  if (now - g_luxFirstMs < 2000 || lux <= 0.0f) return;
  {
    VsLock lk;
    if (lk.ok()) Vs::raw().disp.lux = lux;
  }
  bool below = lux < 15.0f, above = lux > 40.0f;
  bool want = g_night;
  if (!g_night && below) want = true;
  if (g_night && above) want = false;
  if (want != g_night) {
    if (!g_luxBelow) {
      g_luxBelow = true;
      g_luxSinceMs = now;
    } else if (now - g_luxSinceMs >= 5000) {
      g_night = want;
      g_luxBelow = false;
      Serial.printf("night mode %s (lux %.0f)\n", g_night ? "on" : "off", lux);
      if (g_nightMode == -1) applyBacklight();
    }
  } else {
    g_luxBelow = false;
  }
}

void sensorTask(void *) {
  uint32_t lastLux = 0, lastEngine = 0, lastRate = 0, lastSamples = 0, lastPvt = 0;
  for (;;) {
    uint32_t now = millis();
    if (g_imuOk) pollImu(now);
    if (g_gpsOk) pollGps(now);
    if (g_luxOk && now - lastLux >= LUX_PERIOD_MS) {
      lastLux = now;
      pollLux(now);
    }
    // Feed the trip engine from real data at 20 Hz when the demo is not the source.
    if (!Demo::enabled() && now - lastEngine >= 50 && g_engine) {
      lastEngine = now;
      VsLock lk;
      if (lk.ok()) {
        VehicleState &vs = Vs::raw();
        vs.demo = false;
        bool gpsFresh = vs.gps.stampMs && now - vs.gps.stampMs < STALE_GPS_MS && vs.gps.fixOk;
        bool carFresh = vs.car.linked && !vs.car.asleep && now - vs.car.fastStampMs < STALE_OBD_FAST_MS;
        // Displayed speed: wheels when live, else GPS fused with the IMU; the timers always
        // get the raw GPS sample (section 5, paths 1 and 2).
        float kph = 0;
        float rawKph = vs.gps.speedMps * 3.6f;
        if (carFresh) {
          kph = vs.car.wheelAvgKph;
          vs.disp.speedSource = SpeedSource::Wheels;
        } else if (gpsFresh) {
          kph = g_fusion.speedKph();
          vs.disp.speedSource = SpeedSource::Gps;
        } else {
          vs.disp.speedSource = SpeedSource::None;
        }
        vs.disp.speedKph = kph;
        bool motionFresh = vs.motion.stampMs && now - vs.motion.stampMs < STALE_IMU_MS;
        g_engine->update(now, kph, gpsFresh || carFresh, rawKph, gpsFresh, vs.gps.speedAccMps, vs.gps.sats,
                         vs.motion.latG, vs.motion.lonG, motionFresh, vs.car.powerKw, carFresh, vs.car.socPct,
                         vs.car.tempAvg, vs.gps.altM);
        g_engine->publish(vs);
      }
    }
    if (now - lastRate >= 1000) {
      VsLock lk;
      if (lk.ok()) {
        VehicleState &vs = Vs::raw();
        vs.imu.rateHz = (float)(vs.imu.samples - lastSamples) * 1000.0f / (float)(now - lastRate);
        lastSamples = vs.imu.samples;
        vs.diag.imuHz = vs.imu.rateHz;
        vs.diag.i2cLongestHoldMs = I2cBus::longestHoldMs();
        vs.diag.i2cOverHold = I2cBus::overHoldCount();
        float b[3];
        g_att.gyroBias(b);
        vs.diag.attTiltDeg = g_att.tiltDeg();
        vs.diag.attCorrDeg = g_att.lastCorrectionDeg();
        vs.diag.gyroBiasDps = sqrtf(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
        vs.diag.lonBiasG = g_fusion.lonBiasG();
        vs.diag.attTrust = g_attTrust;
        vs.diag.attStationary = g_attStationary;
        vs.diag.accNorm = g_accNorm;
        vs.diag.gyrNorm = g_gyrNorm;
        vs.diag.restNorm = g_att.restNorm();
        vs.utcNow = Clock::nowUtc();
        (void)lastPvt;
      }
      lastRate = now;
    }
    vTaskDelay(1);
  }
}
}  // namespace

namespace Sensors {

void start(TripEngine *engine) {
  g_engine = engine;
  g_gpsOk = gpsBegin();
  g_imuOk = imuBegin();
  g_luxOk = luxBegin();
  Serial.printf("sensors: GPS %s, IMU %s, lux %s\n", g_gpsOk ? "ok" : "absent", g_imuOk ? "ok" : "absent",
                g_luxOk ? "ok" : "absent");
  I2cBus::resetStats();
  {
    VsLock lk;
    if (lk.ok()) Vs::raw().sensorsReady = true;
  }
  xTaskCreatePinnedToCore(sensorTask, "sensors", 8 * 1024, nullptr, 3, nullptr, 0);
}

bool gpsOk() { return g_gpsOk; }
bool imuOk() { return g_imuOk; }
bool luxOk() { return g_luxOk; }
void calibrateMounting() { g_calibrateRequest = true; }
bool mountingRef(float out[3]) {
  for (int i = 0; i < 3; i++) out[i] = g_gRef[i];
  return g_calibrated;
}
void setMountingRef(const float ref[3]) {
  for (int i = 0; i < 3; i++) g_gRef[i] = ref[i];
  g_calibrated = true;
  g_att.reset(g_gRef);
}

void setNightMode(int mode) {
  g_nightMode = mode;
  applyBacklight();
}

uint32_t gpsRefusedReads() { return g_gpsRefused; }

void gpsStats(uint32_t *pollMaxMs, uint32_t *backlogAtMax, uint32_t *nmeaBytes, uint32_t *pollAvgMs) {
  *pollMaxMs = g_gpsPollMaxMs;
  *backlogAtMax = g_gpsPollMaxBacklog;
  *nmeaBytes = g_nmeaSink.count;
  *pollAvgMs = g_gpsPolls ? g_gpsPollTotalMs / g_gpsPolls : 0;
  g_gpsPollMaxMs = g_gpsPollMaxBacklog = 0;
  g_gpsPolls = g_gpsPollTotalMs = 0;
}

void setBrightness(uint8_t percent) {
  g_manualBrightness = percent > 100 ? 100 : percent;
  applyBacklight();
}

}  // namespace Sensors
