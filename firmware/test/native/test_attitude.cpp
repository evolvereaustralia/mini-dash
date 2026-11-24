// Native test of the attitude gates and the calibration acceptance (review findings 20 to 23).
// Build and run with test/native/run.sh.
#include <math.h>
#include <stdio.h>

#include "../../src/app/accel_cal.h"
#include "../../src/app/attitude.h"

static int g_checks = 0, g_failed = 0;
#define CHECK(cond, what)                           \
  do {                                              \
    g_checks++;                                     \
    if (!(cond)) {                                  \
      g_failed++;                                   \
      printf("FAIL line %d: %s\n", __LINE__, what); \
    }                                               \
  } while (0)

static const float DT = 1.0f / 208.0f;
static const float FORWARD[3] = {0, 1, 0};  // sensor Y forward, Z up, so right is +X

// A long sweeping left-hand bend at a steady 25 m/s and 2 deg/s: 0.089 g of centripetal
// acceleration, |a| only 0.4 percent above 1 g, the gyro far under its limit, GPS speed steady.
// Returns the lateral g the filter reports after the given time.
static float sweepingBend(bool useGate, float seconds, float *upLeanDeg) {
  Attitude att;
  TurnGate gate;
  const float rest[3] = {0, 0, 1};
  att.reset(rest);
  att.seedRestNorm(1.0f);
  const float v = 25.0f, yaw = 2.0f;
  const float ac = v * yaw * 0.017453292f / 9.80665f;
  const float a[3] = {-ac, 0, 1.0f};  // turning left: the specific force points left
  const float w[3] = {0, 0, yaw};
  float lon = 0, lat = 0;
  for (int i = 0; i < (int)(seconds / DT); i++) {
    bool turning = useGate ? gate.update(v, att.yawRateDps(w), DT) : false;
    bool trust = att.accelPlausible(a) && !turning;  // gyro and speed-steady tests pass throughout
    att.update(a, w, DT, trust, 0.1f, false, false);
    float up[3];
    att.up(up);
    vehicleFrameG(a, up, FORWARD, &lon, &lat);
  }
  float up[3];
  att.up(up);
  *upLeanDeg = acosf(up[2] > 1 ? 1 : up[2]) / 0.017453292f;
  return lat;
}

int main() {
  // --- finding 20: the gentle sustained turn
  float leanOff, leanOn;
  float latOff = sweepingBend(false, 60, &leanOff);
  float latOn = sweepingBend(true, 60, &leanOn);
  printf("sweeping bend, 0.089 g for 60 s: without the gate lat %+.3f g (up leans %.1f deg); with it lat %+.3f g (up leans %.2f deg)\n",
         latOff, leanOff, latOn, leanOn);
  CHECK(fabsf(latOff) < 0.02f, "without the gate the lateral g fades away (the defect, kept as a reference)");
  CHECK(fabsf(latOn + 0.089f) < 0.005f, "with the gate the lateral g is held through the bend");
  CHECK(leanOn < 0.2f, "with the gate up does not lean into the turn");

  // --- the gate itself: thresholds and the wait before release
  TurnGate g;
  const float v = 20.0f;
  auto yawFor = [&](float gTarget) { return gTarget * 9.80665f / v / 0.017453292f; };
  CHECK(!g.update(v, yawFor(0.04f), DT), "0.04 g: not held");
  CHECK(g.update(v, yawFor(0.06f), DT), "0.06 g: held");
  CHECK(g.update(v, yawFor(0.04f), DT), "back to 0.04 g: still held (between the thresholds)");
  bool held = true;
  for (int i = 0; i < (int)(0.4f / DT); i++) held = g.update(v, yawFor(0.02f), DT);
  CHECK(held, "0.02 g for 0.4 s: still held");
  for (int i = 0; i < (int)(0.2f / DT); i++) held = g.update(v, yawFor(0.02f), DT);
  CHECK(!held, "0.02 g for 0.6 s: released");
  CHECK(g.update(v, -yawFor(0.07f), DT), "a right-hand turn holds as well");
  for (int i = 0; i < 80; i++) g.update(v, yawFor(0.02f), DT);
  g.update(v, yawFor(0.04f), DT);  // a touch above the release level restarts the wait
  for (int i = 0; i < 80; i++) held = g.update(v, yawFor(0.02f), DT);
  CHECK(held, "the release wait restarts if the turn resumes");
  TurnGate g2;
  CHECK(!g2.update(0.2f, 40.0f, DT), "at walking pace a fast yaw is not centripetal");

  // --- finding 21: bias is learnt only when told the car is standing
  {
    Attitude att;
    const float rest[3] = {0, 0, 1};
    att.reset(rest);
    att.seedRestNorm(1.0f);
    const float a[3] = {0, 0, 1};
    const float w[3] = {0, 0, 0.8f};  // a long gentle curve with no GPS: the IMU is quiet
    for (int i = 0; i < (int)(30 / DT); i++) att.update(a, w, DT, true, 1.0f, true, false);
    float b[3];
    att.gyroBias(b);
    CHECK(fabsf(b[2]) < 0.05f, "a quiet IMU alone does not turn a real yaw rate into bias");
    CHECK(fabsf(att.yawRateDps(w) - 0.8f) < 0.05f, "so the yaw rate still reads true");
    for (int i = 0; i < (int)(30 / DT); i++) att.update(a, w, DT, true, 1.0f, true, true);
    att.gyroBias(b);
    CHECK(fabsf(b[2] - 0.8f) < 0.05f, "with standstill evidence the same reading is learnt as bias");
  }
  // --- finding 21: the rest magnitude is the sensor's, not the stored reference's
  {
    Attitude att;
    const float storedRef[3] = {0, 0.2079f, 0.9781f};  // unit vector, 12 degrees of tilt
    const float measured[3] = {0, 0.2079f * 1.045f, 0.9781f * 1.045f};  // this part reads 4.5 percent high
    att.reset(storedRef);
    CHECK(!att.accelPlausible(measured), "against an assumed 1 g a part reading 4.5 percent high is never plausible");
    att.seedRestNorm(1.045f);
    CHECK(att.accelPlausible(measured), "seeded from the sensor it is");
    CHECK(fabsf(att.tiltDeg() - 12.0f) < 0.1f, "and the direction still comes from the stored reference");
    att.seedRestNorm(3.0f);
    CHECK(fabsf(att.restNorm() - 1.045f) < 1e-4f, "an absurd magnitude is ignored");
  }

  // --- finding 22: a forward axis near vertical gives no figures
  {
    float lon = 9, lat = 9;
    const float up[3] = {0, 0, 1};
    const float a[3] = {0.1f, 0.3f, 1.0f};
    CHECK(vehicleFrameG(a, up, FORWARD, &lon, &lat) && fabsf(lon - 0.3f) < 1e-4f && fabsf(lat - 0.1f) < 1e-4f, "forward +Y, level: lon and lat as expected");
    const float fz[3] = {0, 0, -1};
    CHECK(!vehicleFrameG(a, up, fz, &lon, &lat) && lon == 0 && lat == 0, "forward along the vertical: refused, both zero");
    const float up20[3] = {0, 0.342f, 0.940f};  // forward axis 20 degrees from vertical
    CHECK(!vehicleFrameG(a, up20, fz, &lon, &lat), "20 degrees from vertical: refused");
    const float up45[3] = {0, 0.7071f, 0.7071f};
    CHECK(vehicleFrameG(a, up45, fz, &lon, &lat), "45 degrees from vertical: accepted");
    // The finished enclosure: board upright facing the driver, tilted back 10 degrees, forward -Z.
    const float upEnc[3] = {0, 0.9848f, 0.1736f};
    const float aEnc[3] = {0, 0.9848f, 0.1736f - 0.25f};  // 0.25 g along -Z, which is nearly forward
    CHECK(vehicleFrameG(aEnc, upEnc, fz, &lon, &lat) && lon > 0.24f && lon < 0.25f, "enclosure mounting with forward -Z reads acceleration as positive");
    CHECK(!vehicleFrameG(aEnc, upEnc, FORWARD, &lon, &lat), "the bench setting (+Y) is refused in the enclosure, where +Y is nearly vertical");
  }

  // --- finding 23: a calibration is accepted only if the samples lie on a sphere
  {
    const float dirs[10][3] = {{0, 0, 1}, {0, 0, -1}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0},
                               {0.7f, 0.7f, 0}, {-0.7f, 0, 0.7f}, {0, -0.7f, -0.7f}, {0.6f, -0.6f, 0.5f}};
    const float off[3] = {-0.0027f, -0.0255f, 0.0174f};  // the offsets measured on this unit
    auto run = [&](float disturbG, int nDirs, AccelCal::Result &r) {
      AccelCal cal;
      cal.start();
      for (int d = 0; d < nDirs; d++) {
        float n = sqrtf(dirs[d][0] * dirs[d][0] + dirs[d][1] * dirs[d][1] + dirs[d][2] * dirs[d][2]);
        for (int k = 0; k < 120; k++) {
          float a[3];
          for (int i = 0; i < 3; i++) a[i] = dirs[d][i] / n * 1.0027f + off[i];
          if (d == 3) a[0] -= disturbG;  // one direction taken while the board was being pushed
          cal.feed(a, true);
        }
      }
      return cal.finish(r);
    };
    AccelCal::Result r;
    CHECK(run(0, 10, r), "ten clean directions: accepted");
    CHECK(fabsf(r.offset[0] - off[0]) < 0.0005f && fabsf(r.offset[1] - off[1]) < 0.0005f && fabsf(r.offset[2] - off[2]) < 0.0005f, "offsets recovered to half a mg");
    CHECK(fabsf(r.radius - 1.0027f) < 0.0005f && r.rmsResidual < 0.001f && r.maxResidual < 0.001f, "radius and residuals as expected");
    CHECK(!run(0.08f, 10, r), "one direction 80 mg out: refused");
    printf("disturbed set: residual %.1f mg rms, %.1f mg worst\n", r.rmsResidual * 1000, r.maxResidual * 1000);
    CHECK(r.maxResidual > AccelCal::MAX_SINGLE_G || r.rmsResidual > AccelCal::MAX_RMS_G, "and the reason is the residual");
    CHECK(!run(0, 5, r) && r.directions == 5, "five directions: refused");
  }

  printf("attitude gates and calibration: %d checks, %d failed\nRESULT: %s\n", g_checks, g_failed, g_failed ? "FAIL" : "PASS");
  return g_failed ? 1 : 0;
}
