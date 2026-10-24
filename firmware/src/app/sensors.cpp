#include "sensors.h"

#include <Adafruit_VEML7700.h>
#include <Arduino.h>
#include <SparkFun_ISM330DHCX.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Wire.h>
#include <math.h>

#include "board_config.h"
#include "demo_source.h"
#include "i2c_bus.h"
#include "io_expander.h"
#include "trip_engine.h"
#include "vehicle_state.h"

namespace {
SFE_UBLOX_GNSS g_gnss;
SparkFun_ISM330DHCX g_imu;
Adafruit_VEML7700 g_lux;
bool g_gpsOk = false, g_imuOk = false, g_luxOk = false;
TripEngine *g_engine = nullptr;

// Mounting reference: unit gravity vector in the sensor frame, and the forward axis guess.
float g_gRef[3] = {0, 0, 1};
bool g_calibrated = false;
volatile bool g_calibrateRequest = false;

// Display path low-pass state (about 5 Hz at 208 Hz).
float g_latLp = 0, g_lonLp = 0;
constexpr float LP_ALPHA = 0.14f;

int g_nightMode = -1;
uint8_t g_manualBrightness = 100;
bool g_night = false;
uint32_t g_luxSinceMs = 0;
bool g_luxBelow = false;

bool gpsBegin() {
  I2cLock guard(500, "gps.begin");
  if (!guard.ok() || !g_gnss.begin(Wire, I2C_ADDR_NEO_M9N)) return false;
  g_gnss.setI2COutput(COM_TYPE_UBX);
  g_gnss.setNavigationFrequency(GPS_NAV_RATE_HZ);
  g_gnss.setAutoPVT(true);
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

void applyBacklight() {
  uint8_t pct;
  if (g_nightMode == 0) pct = g_manualBrightness;
  else if (g_nightMode == 1) pct = g_manualBrightness < 40 ? g_manualBrightness : 25;
  else pct = g_night ? 25 : 100;
  IoExpander::setBacklightPercent(pct);
  VsLock lk;
  if (lk.ok()) {
    Vs::raw().disp.backlightPct = pct;
    Vs::raw().disp.night = (g_nightMode == 1) || (g_nightMode == -1 && g_night);
  }
}

void pollGps(uint32_t now) {
  I2cLock guard(20, "gps");
  if (!guard.ok() || !g_gnss.getPVT()) return;
  GpsData g;
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
  float ax = a.xData / 1000.0f, ay = a.yData / 1000.0f, az = a.zData / 1000.0f;
  if (g_calibrateRequest) {
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n > 0.5f) {
      g_gRef[0] = ax / n;
      g_gRef[1] = ay / n;
      g_gRef[2] = az / n;
      g_calibrated = true;
    }
    g_calibrateRequest = false;
  }
  // Remove gravity along the reference, then take the IMU's own X as lateral and Y as
  // longitudinal (board axes: X along the connector edge, Y along the header edge, CLAUDE.md
  // section 6). The shell mounts the board with Y pointing forward; sign is set at calibration.
  float gdot = ax * g_gRef[0] + ay * g_gRef[1] + az * g_gRef[2];
  float lx = ax - gdot * g_gRef[0];
  float ly = ay - gdot * g_gRef[1];
  g_latLp += LP_ALPHA * (lx - g_latLp);
  g_lonLp += LP_ALPHA * (ly - g_lonLp);

  VsLock lk(5);
  if (!lk.ok()) return;
  VehicleState &vs = Vs::raw();
  vs.imu.stampMs = now;
  vs.imu.ax = ax;
  vs.imu.ay = ay;
  vs.imu.az = az;
  vs.imu.gx = w.xData / 1000.0f;
  vs.imu.gy = w.yData / 1000.0f;
  vs.imu.gz = w.zData / 1000.0f;
  vs.imu.samples++;
  if (!vs.demo) {
    vs.motion.stampMs = now;
    vs.motion.latG = g_latLp;
    vs.motion.lonG = g_lonLp;
    vs.motion.combinedG = sqrtf(g_latLp * g_latLp + g_lonLp * g_lonLp);
    vs.motion.yawDps = vs.imu.gz;
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
        float kph = 0;
        if (carFresh) {
          kph = vs.car.wheelAvgKph;
          vs.disp.speedSource = SpeedSource::Wheels;
        } else if (gpsFresh) {
          kph = vs.gps.speedMps * 3.6f;
          vs.disp.speedSource = SpeedSource::Gps;
        } else {
          vs.disp.speedSource = SpeedSource::None;
        }
        vs.disp.speedKph = kph;
        bool motionFresh = vs.motion.stampMs && now - vs.motion.stampMs < STALE_IMU_MS;
        char clock[8] = "--:--";
        if (vs.gps.timeValid) snprintf(clock, sizeof(clock), "%02u:%02u", vs.gps.hour, vs.gps.minute);
        g_engine->update(now, kph, gpsFresh || carFresh, vs.gps.speedAccMps, vs.gps.sats, vs.motion.latG,
                         vs.motion.lonG, motionFresh, vs.car.powerKw, carFresh, vs.car.socPct,
                         vs.car.tempAvg, vs.gps.altM);
        g_engine->publish(vs, clock);
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

void setNightMode(int mode) {
  g_nightMode = mode;
  applyBacklight();
}

void setBrightness(uint8_t percent) {
  g_manualBrightness = percent > 100 ? 100 : percent;
  applyBacklight();
}

}  // namespace Sensors
