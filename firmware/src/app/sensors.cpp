#include "sensors.h"

#include <Adafruit_VEML7700.h>
#include <Arduino.h>
#include <SparkFun_ISM330DHCX.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Wire.h>
#include <math.h>

#include "accel_cal.h"
#include "attitude.h"
#include "board_config.h"
#include "clock.h"
#include "capture.h"
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
bool g_gpsConfigOk = false;  // UBX only, 10 Hz and automatic PVT all acknowledged by the receiver
TripEngine *g_engine = nullptr;

// Mounting reference: unit gravity vector in the sensor frame at rest, stored in NVS. It only
// seeds the attitude filter so the G display is right from the first sample; the filter then
// tracks the real orientation (CLAUDE.md section 5).
float g_gRef[3] = {0, 0, 1};
bool g_calibrated = false;
// Results the loop task collects and stores (Persist): taken under this lock so a result is
// never read half written.
portMUX_TYPE g_resultMux = portMUX_INITIALIZER_UNLOCKED;
bool g_mountRefReady = false;
volatile bool g_calibrateRequest = false;

Attitude g_att;
SpeedFusion g_fusion;
AccelCal g_accelCal;
float g_accOff[3] = {0, 0, 0}, g_accRadius = 1.0f;
bool g_accCalibrated = false;
uint32_t g_accelCalStartMs = 0;
volatile bool g_accelCalRequest = false;
bool g_accelCalResultReady = false, g_accelCalResultOk = false;
AccelCal::Result g_accelCalResult;
uint8_t g_accCalState = 0;
uint32_t g_lastImuUs = 0, g_attStartMs = 0;
// Last GPS ground speed and a 1 s history, for the "steady motion" and standstill decisions.
float g_gpsSpeed = 0, g_gpsSpeedHist[10] = {};
int g_gpsSpeedIdx = 0;
uint32_t g_gpsSpeedMs = 0;
uint32_t g_lastEpoch = 0;
bool g_haveEpoch = false;
bool g_attTrust = false, g_attStationary = false, g_attTurnHold = false, g_attBiasLearn = false, g_fwdAxisBad = false;
TurnGate g_turn;
volatile uint8_t g_fwdAxis = IMU_FORWARD_AXIS_DEFAULT;
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
  // Each setting is tried up to three times. A receiver that answers but refuses a setting is
  // present and wrongly configured: at its default 1 Hz every timer run would be thrown out as
  // a gap, with nothing on screen to say why. That is reported as its own state, not as "ok"
  // (review finding 35).
  bool ubxOnly = false, rate = false, autoPvt = false;
  for (int i = 0; i < 3 && !ubxOnly; i++) ubxOnly = g_gnss.setI2COutput(COM_TYPE_UBX);
  for (int i = 0; i < 3 && !rate; i++) rate = g_gnss.setNavigationFrequency(GPS_NAV_RATE_HZ);
  for (int i = 0; i < 3 && !autoPvt; i++) autoPvt = g_gnss.setAutoPVT(true);
  g_gpsConfigOk = ubxOnly && rate && autoPvt;
  Serial.printf("GPS: I2C UBX-only %s, nav rate %s, autoPVT %s%s\n", ubxOnly ? "ok" : "NOT ACKED", rate ? "ok" : "NOT ACKED",
                autoPvt ? "ok" : "NOT ACKED", g_gpsConfigOk ? "" : " | GPS IS NOT CONFIGURED AS REQUIRED: timers and speed fusion cannot be relied on this boot");
  return true;
}

bool imuBegin() {
  I2cLock guard(400, "imu.begin");
  if (!guard.ok()) return false;
  // Three attempts: on 25 Oct 2026 the part answered the bus scan but failed begin() on two
  // boots in a row after its cable had been disturbed; the step that fails is logged.
  bool up = false;
  for (int attempt = 0; attempt < 3 && !up; attempt++) {
    if (attempt) delay(30);
    if (!g_imu.begin(Wire, I2C_ADDR_ISM330DHCX)) {
      Serial.printf("IMU: begin (WHO_AM_I) failed, attempt %d\n", attempt + 1);
      continue;
    }
    g_imu.deviceReset();
    uint32_t t0 = millis();
    bool reset = false;
    while (millis() - t0 < 100) {
      if (g_imu.getDeviceReset()) { reset = true; break; }
      delay(1);
    }
    if (!reset) {
      Serial.printf("IMU: software reset did not complete, attempt %d\n", attempt + 1);
      continue;
    }
    up = true;
  }
  if (!up) return false;
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
  // The level is published only if the expander took it: a refused write used to leave the
  // screen at one brightness and the state claiming another.
  bool wrote = IoExpander::setBacklightPercent(pct);  // takes the bus lock itself (the lock is not recursive)
  static uint32_t refused = 0;
  if (!wrote && ++refused <= 5) Serial.printf("backlight: write of %u%% refused (%lu so far)\n", pct, (unsigned long)refused);
  VsLock lk;
  if (lk.ok()) {
    if (wrote) Vs::raw().disp.backlightPct = pct;
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
  // Capture: every PVT as it arrived, repeats and fixless ones included, with its arrival time.
  // Position is deliberately not captured: the logs may be shared and the repository is public.
  Capture::push('G', g.sats, (uint16_t)((g.fixType & 0x0F) | (g.fixOk ? 0x10 : 0) | (g.timeValid ? 0x20 : 0) | ((g_haveEpoch && g.iTOW == g_lastEpoch) ? 0x40 : 0)),
                (int32_t)g.iTOW, (int32_t)lroundf(g.speedMps * 1000.0f), (int32_t)lroundf(g.hAccM * 1000.0f), (int32_t)lroundf(g.altM * 100.0f),
                (int32_t)lroundf(g.speedAccMps * 1000.0f), (int32_t)lroundf(g.headingDeg * 100.0f));
  if (g.timeValid && g.fixOk) Clock::setFromGps(g.year, g.month, g.day, g.hour, g.minute, g.second);
  // One distinct receiver epoch at a time: a repeated iTOW teaches nothing and must not count
  // as another tenth of a second of evidence (review finding 01 and its notes).
  if (g.fixOk && (!g_haveEpoch || g.iTOW != g_lastEpoch)) {
    g_haveEpoch = true;
    g_lastEpoch = g.iTOW;
    g_fusion.gpsSample(g.speedMps, g.iTOW, micros());
    g_gpsSpeed = g.speedMps;
    g_gpsSpeedHist[g_gpsSpeedIdx] = g.speedMps;
    g_gpsSpeedIdx = (g_gpsSpeedIdx + 1) % 10;
    g_gpsSpeedMs = now;
  }
  VsLock lk;
  if (lk.ok() && !Vs::raw().demo) Vs::raw().gps = g;  // during a replay the GPS state is the replay's alone
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
  float raw[3] = {a.xData / 1000.0f, a.yData / 1000.0f, a.zData / 1000.0f};
  Capture::pushAt(us, 'I', 0, 0, (int32_t)lroundf(a.xData * 1000.0f), (int32_t)lroundf(a.yData * 1000.0f), (int32_t)lroundf(a.zData * 1000.0f),
                  (int32_t)lroundf(w.xData * 10.0f), (int32_t)lroundf(w.yData * 10.0f), (int32_t)lroundf(w.zData * 10.0f));
  float acc[3] = {(raw[0] - g_accOff[0]) / g_accRadius, (raw[1] - g_accOff[1]) / g_accRadius, (raw[2] - g_accOff[2]) / g_accRadius};
  float gyr[3] = {w.xData / 1000.0f, w.yData / 1000.0f, w.zData / 1000.0f};
  if (g_calibrateRequest) {
    float n = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (n > 0.5f) {
      portENTER_CRITICAL(&g_resultMux);
      for (int i = 0; i < 3; i++) g_gRef[i] = acc[i] / n;
      g_calibrated = true;
      g_mountRefReady = true;  // stored by the loop task; the Settings label follows the stored count
      portEXIT_CRITICAL(&g_resultMux);
      g_att.reset(acc);
      g_att.seedRestNorm(n);
      g_turn.reset();
      g_fusion.reset();
    }
    g_calibrateRequest = false;
  }
  if (!g_att.ready()) {
    // Direction from the stored mounting reference if there is one; the magnitude at rest
    // always from this sensor, now (the stored reference is a unit vector).
    g_att.reset(g_calibrated ? g_gRef : acc);
    g_att.seedRestNorm(sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]));
  }

  // Offset calibration: raw samples, collected while the gyro is quiet, until ten directions
  // or 60 s; the fit applies at once and the result waits for main to store it.
  if (g_accelCalRequest) {
    g_accelCalRequest = false;
    g_accelCal.start();
    g_accelCalStartMs = now;
    g_accCalState = 1;
    Serial.println("accel cal: turn the device slowly through flat, upside down and its four edges");
  }
  if (g_accelCal.active()) {
    float bq[3];
    g_att.gyroBias(bq);
    float wq = sqrtf((gyr[0] - bq[0]) * (gyr[0] - bq[0]) + (gyr[1] - bq[1]) * (gyr[1] - bq[1]) + (gyr[2] - bq[2]) * (gyr[2] - bq[2]));
    if (g_accelCal.feed(raw, wq < 45.0f)) Serial.printf("accel cal: direction %d stored\n", g_accelCal.directions());
    static uint32_t lastReport = 0;
    if (now - lastReport >= 2000) {
      lastReport = now;
      Serial.printf("accel cal: %d directions, window spread %.0f mg, gyro %.0f deg/s, %lu s\n", g_accelCal.directions(),
                    g_accelCal.lastSpreadG() * 1000, wq, (unsigned long)((now - g_accelCalStartMs) / 1000));
    }
    bool enough = g_accelCal.directions() >= AccelCal::ENOUGH_DIRS;
    bool timeUp = now - g_accelCalStartMs > 90000;
    if (enough || timeUp) {
      g_accelCalResultOk = g_accelCal.finish(g_accelCalResult);
      if (g_accelCalResultOk) {
        for (int i = 0; i < 3; i++) g_accOff[i] = g_accelCalResult.offset[i];
        g_accRadius = g_accelCalResult.radius;
        g_accCalibrated = true;
        for (int i = 0; i < 3; i++) acc[i] = (raw[i] - g_accOff[i]) / g_accRadius;
        g_att.reset(acc);
        g_att.seedRestNorm(sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]));
        g_turn.reset();
        g_fusion.reset();
        g_accCalState = 2;
        Serial.printf("accel cal: done, %d directions, offset %+.1f %+.1f %+.1f mg, radius %.4f g, residual %.1f mg rms, %.1f mg worst\n",
                      g_accelCalResult.directions, g_accOff[0] * 1000, g_accOff[1] * 1000, g_accOff[2] * 1000, g_accRadius,
                      g_accelCalResult.rmsResidual * 1000, g_accelCalResult.maxResidual * 1000);
      } else {
        g_accCalState = 3;
        Serial.printf("accel cal: FAILED and not applied: %d directions (need %d with each axis seen both ways)%s, residual %.1f mg rms "
                      "(limit %.0f), %.1f mg worst (limit %.0f)\n",
                      g_accelCalResult.directions, AccelCal::MIN_DIRS, g_accelCalResult.conditioned ? "" : ", fit ill-conditioned",
                      g_accelCalResult.rmsResidual * 1000, AccelCal::MAX_RMS_G * 1000, g_accelCalResult.maxResidual * 1000,
                      AccelCal::MAX_SINGLE_G * 1000);
      }
      portENTER_CRITICAL(&g_resultMux);
      g_accelCalResultReady = true;
      portEXIT_CRITICAL(&g_resultMux);
    }
  }

  // When may the accelerometer serve as the gravity reference? Specific force within 3 percent
  // of its magnitude at rest (no linear acceleration), gyro rates under 10 deg/s, the GPS speed
  // steady over the last second when there is one, and no turn in progress: speed times yaw
  // rate is the centripetal acceleration whatever the accelerometer says, and above 0.05 g it
  // holds the accelerometer off (TurnGate, review finding 20).
  // "At rest" (gravity trusted fully) is GPS under 0.3 m/s, or with no GPS a quiet IMU. The gyro
  // bias is learnt only on the GPS evidence: a quiet IMU alone cannot tell a standstill from a
  // long gentle curve (review finding 21). The first 3 s after the filter starts converge fast,
  // the car being at rest when it is switched on.
  float an = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
  float bias[3];
  g_att.gyroBias(bias);
  float wn = sqrtf((gyr[0] - bias[0]) * (gyr[0] - bias[0]) + (gyr[1] - bias[1]) * (gyr[1] - bias[1]) +
                   (gyr[2] - bias[2]) * (gyr[2] - bias[2]));
  bool gpsFresh = g_gpsSpeedMs && now - g_gpsSpeedMs < STALE_GPS_MS;  // set only by epochs with a valid 3D fix
  bool steady = gpsFresh ? fabsf(g_gpsSpeed - g_gpsSpeedHist[g_gpsSpeedIdx]) < 0.4f : true;
  bool plausible = g_att.accelPlausible(acc);
  bool turning = gpsFresh ? g_turn.update(g_gpsSpeed, g_att.yawRateDps(gyr), dt) : false;
  if (!gpsFresh) g_turn.reset();
  bool trust = plausible && wn < 10.0f && steady && !turning;
  bool gpsStill = gpsFresh && g_gpsSpeed < 0.3f && wn < 3.0f;
  bool atRest = gpsFresh ? gpsStill : (plausible && wn < 1.0f);
  g_accNorm = an;
  g_gyrNorm = wn;
  if (g_attStartMs == 0) g_attStartMs = now;
  float kp = now - g_attStartMs < 3000 ? 2.0f : (atRest ? 1.0f : 0.1f);
  g_att.update(acc, gyr, dt, trust, kp, atRest, gpsStill);
  g_attTrust = trust;
  g_attStationary = atRest;
  g_attTurnHold = turning;
  g_attBiasLearn = gpsStill;

  float up[3];
  g_att.up(up);
  float lon, lat;
  // A forward axis that stands near vertical for this mounting gives no G figures at all: the
  // values go stale and the screen shows dashes, and the speed falls back to GPS alone.
  bool frameOk = vehicleFrameG(acc, up, IMU_AXES[g_fwdAxis < 6 ? g_fwdAxis : IMU_FORWARD_AXIS_DEFAULT], &lon, &lat);
  if (frameOk != !g_fwdAxisBad) {
    g_fwdAxisBad = !frameOk;
    Serial.printf("imu: forward axis %s %s\n", IMU_AXIS_NAMES[g_fwdAxis < 6 ? g_fwdAxis : 0],
                  frameOk ? "is usable" : "is within 30 degrees of vertical for this mounting: G figures off until it is changed (serial key A)");
  }
  if (frameOk) {
    g_latLp += LP_ALPHA * (lat - g_latLp);
    g_lonLp += LP_ALPHA * (lon - g_lonLp);
    g_fusion.imuStep(lon, us);
  }
  float yaw = g_att.yawRateDps(gyr);
  static uint8_t capN = 0;
  if (++capN >= 10) {
    capN = 0;
    float wnClip = wn > 6000.0f ? 6000.0f : wn;
    Capture::pushAt(us, 'A', (uint8_t)((trust ? 1 : 0) | (atRest ? 2 : 0) | (turning ? 4 : 0) | (gpsStill ? 8 : 0) | (plausible ? 16 : 0) |
                                       (steady ? 32 : 0) | (frameOk ? 64 : 0) | (gpsFresh ? 128 : 0)),
                    (uint16_t)lroundf(wnClip * 10.0f), (int32_t)lroundf(lon * 1e6f), (int32_t)lroundf(lat * 1e6f), (int32_t)lroundf(yaw * 1000.0f),
                    (int32_t)lroundf(g_fusion.speedMps() * 1000.0f), (int32_t)lroundf(g_turn.centripetalG() * 1e6f), (int32_t)lroundf(an * 1e6f));
  }

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
  if (!vs.demo && frameOk) {
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
        bool carUp = vs.car.linked && !vs.car.asleep;
        bool carFresh = carUp && vs.car.fastStampMs && now - vs.car.fastStampMs < STALE_OBD_FAST_MS;
        bool socFresh = carUp && vs.car.socStampMs && now - vs.car.socStampMs < STALE_OBD_SLOW_MS;
        bool tempFresh = carUp && vs.car.tempStampMs && now - vs.car.tempStampMs < STALE_OBD_SLOW_MS;
        // Displayed speed: wheels when live, else GPS fused with the IMU; the timers always
        // get the raw GPS sample (section 5, paths 1 and 2).
        float kph = 0;
        float rawKph = vs.gps.speedMps * 3.6f;
        bool wheelsFresh = carUp && vs.car.wheelStampMs && now - vs.car.wheelStampMs < STALE_OBD_FAST_MS;
        if (wheelsFresh) {
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
        g_engine->update(now, kph, gpsFresh || wheelsFresh, rawKph, gpsFresh, vs.gps.speedAccMps, vs.gps.sats,
                         vs.motion.latG, vs.motion.lonG, motionFresh, vs.car.powerKw, carFresh, socFresh ? vs.car.socPct : -1.0f,
                         tempFresh ? vs.car.tempAvg : -273.0f, vs.gps.altM);
        g_engine->publish(vs);
        if (Capture::streaming()) {
          float gb[3];
          g_att.gyroBias(gb);
          Capture::push('S', (uint8_t)((gpsFresh ? 1 : 0) | (wheelsFresh ? 2 : 0) | (carFresh ? 4 : 0) | (motionFresh ? 8 : 0) | (socFresh ? 16 : 0) |
                                       (vs.disp.speedSource == SpeedSource::Wheels ? 32 : 0)),
                        vs.gps.sats, (int32_t)lroundf(kph * 1000.0f), (int32_t)lroundf(rawKph * 1000.0f),
                        wheelsFresh ? (int32_t)lroundf(vs.car.wheelAvgKph * 1000.0f) : -1, carFresh ? (int32_t)lroundf(vs.car.powerKw * 1000.0f) : 0,
                        socFresh ? (int32_t)lroundf(vs.car.socPct * 100.0f) : -1, (int32_t)lroundf(vs.trip.distanceKm * 100000.0f));
          Capture::push('F', 0, 0, (int32_t)lroundf(g_latLp * 1e6f), (int32_t)lroundf(g_lonLp * 1e6f), (int32_t)lroundf(g_att.tiltDeg() * 100.0f),
                        (int32_t)lroundf(g_att.restNorm() * 1e6f), (int32_t)lroundf(sqrtf(gb[0] * gb[0] + gb[1] * gb[1] + gb[2] * gb[2]) * 1000.0f),
                        (int32_t)lroundf(g_fusion.lonBiasG() * 1e6f));
        }
      }
    }
    Capture::stackMark(1);
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
        vs.diag.attTurnHold = g_attTurnHold;
        vs.diag.attBiasLearn = g_attBiasLearn;
        vs.diag.fwdAxisBad = g_fwdAxisBad;
        vs.diag.fwdAxis = g_fwdAxis;
        vs.diag.centripetalG = g_turn.centripetalG();
        vs.diag.accCalState = g_accCalState;
        vs.diag.accCalDirs = (uint8_t)g_accelCal.directions();
        vs.diag.accCalibrated = g_accCalibrated;
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
  Serial.printf("sensors: GPS %s, IMU %s, lux %s\n", !g_gpsOk ? "absent" : (g_gpsConfigOk ? "ok" : "present but NOT configured"), g_imuOk ? "ok" : "absent",
                g_luxOk ? "ok" : "absent");
  I2cBus::resetStats();
  {
    VsLock lk;
    if (lk.ok()) Vs::raw().sensorsReady = true;
  }
  xTaskCreatePinnedToCore(sensorTask, "sensors", 8 * 1024, nullptr, 3, nullptr, 0);
}

bool gpsOk() { return g_gpsOk; }
bool gpsConfigured() { return g_gpsOk && g_gpsConfigOk; }
bool imuOk() { return g_imuOk; }
bool luxOk() { return g_luxOk; }
void calibrateMounting() { g_calibrateRequest = true; }
bool mountingRef(float out[3]) {
  portENTER_CRITICAL(&g_resultMux);
  for (int i = 0; i < 3; i++) out[i] = g_gRef[i];
  bool c = g_calibrated;
  portEXIT_CRITICAL(&g_resultMux);
  return c;
}
bool mountRefTake(float out[3]) {
  portENTER_CRITICAL(&g_resultMux);
  bool ready = g_mountRefReady;
  g_mountRefReady = false;
  if (ready)
    for (int i = 0; i < 3; i++) out[i] = g_gRef[i];
  portEXIT_CRITICAL(&g_resultMux);
  return ready;
}
void setMountingRef(const float ref[3]) {
  for (int i = 0; i < 3; i++) g_gRef[i] = ref[i];
  g_calibrated = true;
  g_att.reset(g_gRef);
}

void setForwardAxis(uint8_t axis) {
  if (axis < 6) g_fwdAxis = axis;
}
uint8_t forwardAxis() { return g_fwdAxis; }

void startAccelCal() { g_accelCalRequest = true; }
bool accelCalTake(bool *ok, float offset[3], float *radius, float *rmsResidual, int *directions) {
  portENTER_CRITICAL(&g_resultMux);
  bool ready = g_accelCalResultReady;
  g_accelCalResultReady = false;
  portEXIT_CRITICAL(&g_resultMux);
  if (!ready) return false;  // the result itself is not rewritten until the next calibration is started
  *ok = g_accelCalResultOk;
  for (int i = 0; i < 3; i++) offset[i] = g_accelCalResult.offset[i];
  *radius = g_accelCalResult.radius;
  *rmsResidual = g_accelCalResult.rmsResidual;
  *directions = g_accelCalResult.directions;
  return true;
}
void setAccelCal(const float offset[3], float radius) {
  for (int i = 0; i < 3; i++) g_accOff[i] = offset[i];
  g_accRadius = radius > 0.5f && radius < 1.5f ? radius : 1.0f;
  g_accCalibrated = true;
  g_accCalState = 2;
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
