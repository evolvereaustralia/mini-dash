// Compiled only in env:bringup (the hybrid IDF build ignores build_src_filter).
#ifdef MINIDASH_BRINGUP
// Mini Dash phase 3a: board bring-up (CLAUDE.md section 10).
//
// Proves, in order: USB CDC serial, PSRAM and flash settings, the I2C map against
// board_config.h, the IO expander (backlight PWM ramp), the RGB panel (colour bars),
// the NEO-M9N at 10 Hz PVT, the ISM330DHCX at 208 Hz, the VEML7700 at 1 Hz. One summary
// line per second. Everything on core 1 in loop(); the task split comes in phase 3b.
//
// Serial keys: s = rescan I2C, 0..9 = backlight 10..100 percent, c = colour bars,
// w = white, k = black, r = reboot.

#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_VEML7700.h>
#include <SparkFun_ISM330DHCX.h>
#include <SparkFun_u-blox_GNSS_v3.h>

#include "board_config.h"
#include "display_rgb.h"
#include "i2c_bus.h"
#include "io_expander.h"

namespace {

constexpr const char *FW_NAME = "mini-dash bringup";

SFE_UBLOX_GNSS g_gnss;
SparkFun_ISM330DHCX g_imu;
Adafruit_VEML7700 g_lux;

bool g_expanderOk = false;
bool g_displayOk = false;
bool g_gpsOk = false;
bool g_imuOk = false;
bool g_luxOk = false;
uint8_t g_backlightPercent = 100;

// Latest readings for the once-a-second print.
struct {
  uint8_t fixType = 0;
  bool fixOk = false;
  uint8_t siv = 0;
  float speedMps = 0;
  float speedAccMps = 0;
  float hAccM = 0;
  int32_t lat = 0, lon = 0;
  float altM = 0;
  uint32_t pvtCount = 0;
  uint32_t lastIToW = 0;
  uint16_t year = 0;
  uint8_t month = 0, day = 0, hour = 0, minute = 0, second = 0;
  bool timeValid = false;
} g_gps;

struct {
  sfe_ism_data_t accel{};  // mg
  sfe_ism_data_t gyro{};   // mdps
  uint32_t samples = 0;
  int16_t tempRaw = 0;
} g_imuData;

float g_luxValue = -1;

// ------------------------------------------------------------------------------------
void printBanner() {
  Serial.println();
  Serial.printf("%s, built %s %s\n", FW_NAME, __DATE__, __TIME__);
  Serial.printf("chip: %s rev %d, %d cores, %lu MHz\n", ESP.getChipModel(), ESP.getChipRevision(),
                ESP.getChipCores(), (unsigned long)ESP.getCpuFreqMHz());
  Serial.printf("flash: %lu MB, mode %d; PSRAM: %lu bytes (%s)\n",
                (unsigned long)(ESP.getFlashChipSize() / (1024 * 1024)), (int)ESP.getFlashChipMode(),
                (unsigned long)ESP.getPsramSize(), psramFound() ? "found" : "NOT FOUND");
  Serial.printf("reset reason: %d; sketch %lu bytes, free %lu\n", (int)esp_reset_reason(),
                (unsigned long)ESP.getSketchSize(), (unsigned long)ESP.getFreeSketchSpace());
  Serial.printf("arduino core %d.%d.%d, idf %s\n", ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR,
                ESP_ARDUINO_VERSION_PATCH, esp_get_idf_version());
}

// ------------------------------------------------------------------------------------
const char *expectedName(uint8_t addr) {
  for (const auto &e : I2C_EXPECTED) {
    if (e.addr == addr) return e.name;
  }
  return nullptr;
}

void scanAndReport() {
  uint8_t found[32];
  size_t n = I2cBus::scan(found, sizeof(found));
  Serial.printf("I2C scan: %u device(s)\n", (unsigned)n);
  bool allGood = true;
  for (size_t i = 0; i < n; i++) {
    const char *name = expectedName(found[i]);
    if (name) {
      Serial.printf("  0x%02X  %s\n", found[i], name);
    } else {
      Serial.printf("  0x%02X  UNEXPECTED: not in board_config.h, stop and ask\n", found[i]);
      allGood = false;
    }
  }
  for (const auto &e : I2C_EXPECTED) {
    bool present = false;
    for (size_t i = 0; i < n; i++) present |= (found[i] == e.addr);
    if (!present) {
      Serial.printf("  0x%02X  MISSING: %s (%s)\n", e.addr, e.name,
                    e.onBoard ? "on the display board" : "Mini Dash sensor, check the terminal wiring");
      allGood = false;
    }
  }
  Serial.println(allGood ? "I2C map matches board_config.h" : "I2C map does NOT match board_config.h");
}

// ------------------------------------------------------------------------------------
void setBacklight(uint8_t percent) {
  g_backlightPercent = percent;
  if (!IoExpander::setBacklightPercent(percent)) {
    Serial.println("backlight: expander write failed");
  }
}

void backlightRamp() {
  Serial.print("backlight ramp 100 -> 0 -> 100:");
  for (int p = 100; p >= 0; p -= 10) {
    setBacklight(p);
    Serial.printf(" %d", p);
    delay(150);
  }
  for (int p = 0; p <= 100; p += 10) {
    setBacklight(p);
    Serial.printf(" %d", p);
    delay(150);
  }
  Serial.println();
}

// ------------------------------------------------------------------------------------
bool gpsBegin() {
  I2cLock guard(500);
  if (!guard.ok()) return false;
  // begin() polls the module a few times; it is allowed to exceed the normal hold limit once.
  if (!g_gnss.begin(Wire, I2C_ADDR_NEO_M9N)) return false;
  g_gnss.setI2COutput(COM_TYPE_UBX);  // UBX only on I2C, no NMEA
  g_gnss.setNavigationFrequency(GPS_NAV_RATE_HZ);
  g_gnss.setAutoPVT(true);
  Serial.printf("GPS: protocol version %d.%02d, nav rate %d Hz\n", g_gnss.getProtocolVersionHigh(),
                g_gnss.getProtocolVersionLow(), g_gnss.getNavigationFrequency());
  return true;
}

void gpsPoll() {
  I2cLock guard(20);
  if (!guard.ok()) return;
  if (!g_gnss.getPVT()) return;  // non-blocking with autoPVT
  g_gps.pvtCount++;
  g_gps.fixType = g_gnss.getFixType();
  g_gps.fixOk = g_gnss.getGnssFixOk();
  g_gps.siv = g_gnss.getSIV();
  g_gps.speedMps = g_gnss.getGroundSpeed() / 1000.0f;
  g_gps.speedAccMps = g_gnss.getSpeedAccEst() / 1000.0f;
  g_gps.hAccM = g_gnss.getHorizontalAccEst() / 1000.0f;
  g_gps.lat = g_gnss.getLatitude();
  g_gps.lon = g_gnss.getLongitude();
  g_gps.altM = g_gnss.getAltitudeMSL() / 1000.0f;
  g_gps.lastIToW = g_gnss.getTimeOfWeek();
  g_gps.timeValid = g_gnss.getTimeValid() && g_gnss.getDateValid();
  g_gps.year = g_gnss.getYear();
  g_gps.month = g_gnss.getMonth();
  g_gps.day = g_gnss.getDay();
  g_gps.hour = g_gnss.getHour();
  g_gps.minute = g_gnss.getMinute();
  g_gps.second = g_gnss.getSecond();
}

// ------------------------------------------------------------------------------------
bool imuBegin() {
  I2cLock guard(200);
  if (!guard.ok()) return false;
  if (!g_imu.begin(Wire, I2C_ADDR_ISM330DHCX)) return false;
  g_imu.deviceReset();
  uint32_t t0 = millis();
  while (!g_imu.getDeviceReset()) {
    if (millis() - t0 > 100) {
      Serial.println("IMU: reset did not complete");
      return false;
    }
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
  Serial.printf("IMU: WHO_AM_I 0x%02X (expect 0x6B), 208 Hz, 4 g, 500 dps\n", g_imu.getUniqueId());
  return true;
}

void imuPoll() {
  I2cLock guard(10);
  if (!guard.ok()) return;
  if (!g_imu.checkStatus()) return;
  if (g_imu.getAccel(&g_imuData.accel) && g_imu.getGyro(&g_imuData.gyro)) {
    g_imuData.samples++;
  }
}

// ------------------------------------------------------------------------------------
bool luxBegin() {
  I2cLock guard(100);
  if (!guard.ok()) return false;
  if (!g_lux.begin(&Wire)) return false;
  // Gain 1/8 and 100 ms gives the widest range (to about 120 klux) for a sun-on-dash check.
  g_lux.setGain(VEML7700_GAIN_1_8);
  g_lux.setIntegrationTime(VEML7700_IT_100MS);
  return true;
}

void luxPoll() {
  I2cLock guard(20);
  if (!guard.ok()) return;
  g_luxValue = g_lux.readLux(VEML_LUX_NORMAL_NOWAIT);
}

// ------------------------------------------------------------------------------------
void printStatus(uint32_t uptimeS, uint32_t imuSamplesPerSec) {
  Serial.printf("[%5lu s] ", (unsigned long)uptimeS);
  if (g_gpsOk) {
    if (g_gps.fixOk) {
      Serial.printf("GPS fix %d sats %2d spd %5.2f m/s (+-%4.2f) hAcc %5.1f m alt %6.1f m pvt %lu",
                    g_gps.fixType, g_gps.siv, g_gps.speedMps, g_gps.speedAccMps, g_gps.hAccM, g_gps.altM,
                    (unsigned long)g_gps.pvtCount);
    } else {
      Serial.printf("GPS no fix (type %d) sats %2d pvt %lu", g_gps.fixType, g_gps.siv,
                    (unsigned long)g_gps.pvtCount);
    }
    if (g_gps.timeValid) {
      Serial.printf(" %04u-%02u-%02u %02u:%02u:%02uZ", g_gps.year, g_gps.month, g_gps.day, g_gps.hour,
                    g_gps.minute, g_gps.second);
    }
  } else {
    Serial.print("GPS off");
  }
  Serial.print(" | ");
  if (g_imuOk) {
    Serial.printf("IMU a %+6.3f %+6.3f %+6.3f g  w %+7.2f %+7.2f %+7.2f dps  %lu/s",
                  g_imuData.accel.xData / 1000.0f, g_imuData.accel.yData / 1000.0f,
                  g_imuData.accel.zData / 1000.0f, g_imuData.gyro.xData / 1000.0f,
                  g_imuData.gyro.yData / 1000.0f, g_imuData.gyro.zData / 1000.0f,
                  (unsigned long)imuSamplesPerSec);
  } else {
    Serial.print("IMU off");
  }
  Serial.print(" | ");
  if (g_luxOk) {
    Serial.printf("lux %7.1f", g_luxValue);
  } else {
    Serial.print("lux off");
  }
  Serial.printf(" | BL %3u%% | i2c hold max %lu ms over %lu | heap %lu psram %lu\n", g_backlightPercent,
                (unsigned long)I2cBus::longestHoldMs(), (unsigned long)I2cBus::overHoldCount(),
                (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getFreePsram());
}

void handleSerial() {
  while (Serial.available()) {
    int c = Serial.read();
    if (c >= '0' && c <= '9') {
      uint8_t pct = (c == '0') ? 100 : (c - '0') * 10;
      setBacklight(pct);
      Serial.printf("backlight %u%%\n", pct);
    } else if (c == 's') {
      scanAndReport();
    } else if (c == 'c') {
      DisplayRgb::colourBars();
    } else if (c == 'w') {
      DisplayRgb::fill(DisplayRgb::RGB565_WHITE);
    } else if (c == 'k') {
      DisplayRgb::fill(DisplayRgb::RGB565_BLACK);
    } else if (c == 'r') {
      Serial.println("rebooting");
      delay(50);
      ESP.restart();
    }
  }
}

}  // namespace

// ======================================================================================
void setup() {
  Serial.begin(115200);
  // USB CDC: give a host up to 3 s to open the port, then carry on regardless. The device
  // must boot in the car with nothing listening.
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  printBanner();

  I2cBus::begin();
  scanAndReport();

  g_expanderOk = IoExpander::begin();
  Serial.printf("IO expander: %s\n", g_expanderOk ? "ok" : "NO ACK");

  g_displayOk = (DisplayRgb::begin() != nullptr);
  Serial.printf("RGB panel: %s\n", g_displayOk ? "ok (framebuffers in PSRAM)" : "FAILED");
  if (g_displayOk) DisplayRgb::colourBars();

  if (g_expanderOk) {
    IoExpander::setBacklightEnable(true);
    setBacklight(100);
    backlightRamp();
  }

  g_gpsOk = gpsBegin();
  Serial.printf("GPS NEO-M9N: %s\n", g_gpsOk ? "ok" : "not found");
  g_imuOk = imuBegin();
  Serial.printf("IMU ISM330DHCX: %s\n", g_imuOk ? "ok" : "not found");
  g_luxOk = luxBegin();
  Serial.printf("Light VEML7700: %s\n", g_luxOk ? "ok" : "not found");
  Serial.println("keys: s scan, 1..9/0 backlight 10..100%, c bars, w white, k black, r reboot");
  I2cBus::resetStats();  // init holds are expected to be long; measure steady state from here
}

void loop() {
  static uint32_t lastPrint = 0;
  static uint32_t lastLux = 0;
  static uint32_t lastImuSamples = 0;

  if (g_imuOk) imuPoll();
  if (g_gpsOk) gpsPoll();

  uint32_t now = millis();
  if (g_luxOk && now - lastLux >= LUX_PERIOD_MS) {
    lastLux = now;
    luxPoll();
  }
  if (now - lastPrint >= 1000) {
    lastPrint = now;
    uint32_t perSec = g_imuData.samples - lastImuSamples;
    lastImuSamples = g_imuData.samples;
    printStatus(now / 1000, perSec);
  }
  handleSerial();
  delay(1);
}

#endif  // MINIDASH_BRINGUP
