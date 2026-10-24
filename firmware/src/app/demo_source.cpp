#include "demo_source.h"

#include <Arduino.h>
#include <math.h>

#include "trip_engine.h"
#include "vehicle_state.h"

namespace {
TripEngine *g_engine = nullptr;
volatile bool g_enabled = false;
bool g_wasEnabled = false;

struct Seg {
  float seconds;
  float targetKph;  // speed at the end of the segment, linear ramp from the previous
  float latG;       // lateral g held through the segment (sine-shaped)
};

// Cooper SE: 0-100 in about 7.3 s, 135 kW motor, regen up to about 50 kW at the battery.
const Seg SCRIPT[] = {
    {4.0f, 0, 0},      {2.6f, 60, 0},    {4.7f, 100, 0},   {6.0f, 100, 0},   {5.0f, 80, 0.45f},
    {5.0f, 80, -0.5f}, {4.0f, 100, 0},   {3.2f, 0, 0},     {3.0f, 0, 0},     {6.0f, 50, 0},
    {6.0f, 55, 0.35f}, {6.0f, 60, -0.3f}, {8.0f, 60, 0},   {5.0f, 30, 0},    {6.0f, 70, 0.6f},
    {5.0f, 0, 0},      {4.0f, 0, 0},
};
constexpr int SEG_COUNT = sizeof(SCRIPT) / sizeof(SCRIPT[0]);

float g_t = 0;
float g_soc = 71.0f;
float g_packTemp = 26.5f;
float g_heading = 248.0f;
float g_prevKph = 0;

void scriptAt(float t, float *kph, float *latG) {
  float total = 0;
  for (int i = 0; i < SEG_COUNT; i++) total += SCRIPT[i].seconds;
  t = fmodf(t, total);
  float start = 0, prevKph = 0;
  for (int i = 0; i < SEG_COUNT; i++) {
    const Seg &s = SCRIPT[i];
    if (t < start + s.seconds) {
      float f = (t - start) / s.seconds;
      *kph = prevKph + (s.targetKph - prevKph) * f;
      *latG = s.latG * sinf(f * (float)M_PI);
      return;
    }
    start += s.seconds;
    prevKph = s.targetKph;
  }
  *kph = 0;
  *latG = 0;
}

void demoTask(void *) {
  const uint32_t periodMs = 20;
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(periodMs));
    uint32_t now = millis();
    bool on = g_enabled;
    if (on != g_wasEnabled) {
      g_wasEnabled = on;
      VsLock lk;
      if (lk.ok()) {
        VehicleState &vs = Vs::raw();
        if (g_engine) {
          g_engine->setIsolated(on);  // demo records never reach today or lifetime
          g_engine->reset(now);
          g_engine->publish(vs);
        }
        if (!on) {
          // Leaving the demo: the car data was synthetic, so drop it (OBD is phase 3c).
          vs.car = CarData();
          vs.motion = MotionData();
          vs.disp.speedSource = SpeedSource::None;
          vs.demo = false;
        }
      }
      g_t = 0;
      g_prevKph = 0;
      g_soc = 71.0f;
    }
    if (!on) continue;

    g_t += periodMs / 1000.0f;
    float kph, latG;
    scriptAt(g_t, &kph, &latG);
    float lonG = (kph - g_prevKph) / 3.6f / (periodMs / 1000.0f) / 9.81f;
    // add a little texture so the plot breathes
    lonG += 0.01f * sinf(g_t * 7.0f);
    latG += 0.012f * sinf(g_t * 5.3f);
    g_prevKph = kph;

    // Battery power: tractive power plus drag and auxiliaries, regen capped like the car.
    const float mass = 1440.0f;
    float v = kph / 3.6f;
    float tractive = mass * lonG * 9.81f * v / 1000.0f;  // kW
    float drag = (0.35f * v * v * v / 1000.0f) + 0.012f * mass * 9.81f * v / 1000.0f;
    float power = tractive + drag + 0.9f;
    if (power > 135.0f) power = 135.0f;
    if (power < -52.0f) power = -52.0f;
    if (kph < 0.5f) power = 0.9f;
    g_soc -= power * (periodMs / 1000.0f) / 3600.0f / 28.9f * 100.0f;  // 28.9 kWh usable
    if (g_soc < 20.0f) g_soc = 71.0f;
    g_packTemp += (fabsf(power) * 0.0004f - (g_packTemp - 25.0f) * 0.002f) * (periodMs / 1000.0f);
    g_heading += latG * 12.0f * (periodMs / 1000.0f) * (kph > 5 ? 1.0f : 0.0f);
    if (g_heading >= 360) g_heading -= 360;
    if (g_heading < 0) g_heading += 360;

    char clock[8];
    {
      VsLock lk;
      if (!lk.ok()) continue;
      VehicleState &vs = Vs::raw();
      vs.demo = true;
      vs.gps.stampMs = now;
      vs.gps.fixOk = true;
      vs.gps.fixType = 3;
      vs.gps.sats = 11;
      vs.gps.speedMps = v;
      vs.gps.speedAccMps = 0.12f;
      vs.gps.hAccM = 1.8f;
      vs.gps.headingDeg = g_heading;
      vs.gps.altM = 48.0f;
      vs.motion.stampMs = now;
      vs.motion.latG = latG;
      vs.motion.lonG = lonG;
      vs.motion.combinedG = sqrtf(latG * latG + lonG * lonG);
      vs.motion.yawDps = latG * 9.81f / (v > 1 ? v : 1) * 57.3f * (v > 1 ? 1 : 0);
      vs.car.linked = true;
      vs.car.asleep = false;
      vs.car.fastStampMs = now;
      vs.car.slowStampMs = now;
      vs.car.socPct = g_soc;
      vs.car.socMinPct = 6;
      vs.car.socMaxPct = 94;
      vs.car.packV = 352.0f + (g_soc - 50.0f) * 0.5f - power * 0.08f;
      vs.car.packA = vs.car.packV > 1 ? power * 1000.0f / vs.car.packV : 0;
      vs.car.powerKw = power;
      vs.car.tempMin = g_packTemp - 0.8f;
      vs.car.tempAvg = g_packTemp;
      vs.car.tempMax = g_packTemp + 0.9f;
      vs.car.cellAvgV = vs.car.packV / 96.0f;
      vs.car.cellMinV = vs.car.cellAvgV - 0.006f;
      vs.car.cellMaxV = vs.car.cellAvgV + 0.006f;
      vs.car.motorTemp1 = 38.0f + g_packTemp * 0.4f + fabsf(power) * 0.05f;
      vs.car.motorTemp2 = vs.car.motorTemp1 - 2.0f;
      vs.car.torqueReqNm = lonG * 270.0f * 2.2f;
      vs.car.torqueNm = vs.car.torqueReqNm * 0.97f;
      vs.car.pedalPct = power > 0 ? fminf(100.0f, power / 135.0f * 100.0f) : 0;
      for (int i = 0; i < 4; i++) vs.car.wheelKph[i] = kph * (1.0f + 0.004f * (i - 1.5f) * latG);
      vs.car.wheelAvgKph = kph;
      vs.car.v12 = 12.7f + (power > 0 ? 0.1f : 0.0f);
      vs.car.rangeKm = g_soc * 2.1f;
      vs.car.sohPct = 94.0f;
      vs.disp.speedKph = kph;
      vs.disp.speedSource = SpeedSource::Wheels;
      (void)clock;
      if (g_engine) {
        g_engine->update(now, kph, true, 0.12f, 11, latG, lonG, true, power, true, g_soc, g_packTemp, 48.0f);
        g_engine->publish(vs);
      }
    }
  }
}
}  // namespace

namespace Demo {

void start(TripEngine *engine) {
  g_engine = engine;
  xTaskCreatePinnedToCore(demoTask, "demo", 6 * 1024, nullptr, 1, nullptr, 0);
}

void setEnabled(bool on) { g_enabled = on; }
bool enabled() { return g_enabled; }

}  // namespace Demo
