// Native tests of the speed fusion against the cases the independent review raised (finding
// 01 and its notes, 24 Nov 2026): a first fix while moving, no IMU at all, a repeated epoch,
// two epochs delivered together, a GPS outage, standstill counted from repeats. Run with
// test/native/run.sh.
#include <math.h>
#include <stdio.h>

#include "../../src/app/speed_fusion.h"

static int g_fail = 0, g_run = 0;
#define CHECK(cond, ...)                  \
  do {                                    \
    g_run++;                              \
    if (!(cond)) {                        \
      g_fail++;                           \
      printf("FAIL line %d: ", __LINE__); \
      printf(__VA_ARGS__);                \
      printf("\n");                       \
    }                                     \
  } while (0)

static const double IMU_DT = 1.0 / 208.0;
static uint32_t us(double t) { return (uint32_t)llround(t * 1e6); }
static uint32_t epoch(double tMeasured) { return (uint32_t)(700000 + llround(tMeasured * 1000.0)); }

int main() {
  // 1. First fix arrives while cruising at 20 m/s, IMU running and reporting no acceleration.
  {
    SpeedFusion f;
    float lo = 1e9f, hi = -1e9f;
    double nextGps = 2.0;
    for (int k = 0; k < 208 * 8; k++) {
      double t = k * IMU_DT;
      f.imuStep(0.0f, us(t));
      if (t >= nextGps) {
        f.gpsSample(20.0f, epoch(t - 0.1), us(t));
        nextGps += 0.1;
      }
      if (t >= 2.0 + IMU_DT) {
        if (f.speedMps() < lo) lo = f.speedMps();
        if (f.speedMps() > hi) hi = f.speedMps();
      }
    }
    CHECK(lo > 19.95f && hi < 20.05f, "first fix in motion: speed ranged %.2f to %.2f, want 20.00 throughout", lo, hi);
    CHECK(f.inertial(), "IMU running: the estimate should be inertial after the first fixes");
  }
  // 2. The same with no IMU samples at all: the output is the raw Doppler speed, never more.
  {
    SpeedFusion f;
    float hi = 0;
    for (int n = 0; n < 60; n++) {
      double t = 2.0 + n * 0.1;
      f.gpsSample(20.0f + (n >= 30 ? 5.0f : 0.0f), epoch(t - 0.1), us(t));
      if (f.speedMps() > hi) hi = f.speedMps();
      CHECK(fabsf(f.speedMps() - (20.0f + (n >= 30 ? 5.0f : 0.0f))) < 0.001f, "no IMU: fix %d gave %.2f", n, f.speedMps());
    }
    CHECK(hi < 25.01f && !f.inertial(), "no IMU: peak %.2f, inertial %d", hi, f.inertial());
  }
  // 3. A repeated epoch must change nothing.
  {
    SpeedFusion f;
    double t = 0;
    for (int k = 0; k < 208 * 3; k++) {
      t = k * IMU_DT;
      f.imuStep(0.0f, us(t));
      if (k % 21 == 0) f.gpsSample(10.0f, epoch(t - 0.1), us(t));
    }
    // a fix 2 m/s above the estimate, delivered three times
    f.imuStep(0.0f, us(t + IMU_DT));
    f.gpsSample(12.0f, epoch(t + 0.05), us(t + IMU_DT));
    float once = f.speedMps();
    f.gpsSample(12.0f, epoch(t + 0.05), us(t + 2 * IMU_DT));
    f.gpsSample(12.0f, epoch(t + 0.05), us(t + 3 * IMU_DT));
    CHECK(fabsf(f.speedMps() - once) < 1e-6f && f.duplicates() == 2, "repeated epoch moved the estimate from %.3f to %.3f (%u ignored)", once, f.speedMps(),
          (unsigned)f.duplicates());
    CHECK(fabsf(once - (10.0f + 0.35f * 2.0f)) < 0.02f, "single correction gave %.3f, want 10.70", once);
  }
  // 4. Under acceleration, two epochs delivered together after a bus delay must give the same
  //    estimate as the same two delivered on time.
  {
    float result[2];
    for (int burst = 0; burst < 2; burst++) {
      SpeedFusion f;
      double held = -1;
      for (int k = 0; k < 208 * 6; k++) {
        double t = k * IMU_DT;
        float a = t >= 2.0 ? 0.3f : 0.0f;  // 0.3 g from t = 2 s
        f.imuStep(a * 1.03f, us(t));       // the IMU over-reads by 3 percent, so corrections matter
        auto truth = [](double tt) { return tt >= 2.0 ? (float)(0.3 * 9.80665 * (tt - 2.0)) : 0.0f; };
        double due = floor(t * 10.0) / 10.0;  // a fix is due every 100 ms
        static double last[2] = {-1, -1};
        if (due != last[burst] && t >= 0.5) {
          bool isHeld = burst == 1 && fabs(due - 4.0) < 1e-6;  // hold the 4.0 s fix until the next one
          if (isHeld) held = due;
          else {
            if (held >= 0) {
              f.gpsSample(truth(held - 0.1) + 5.0f, epoch(held - 0.1), us(t));
              held = -1;
            }
            f.gpsSample(truth(due - 0.1) + 5.0f, epoch(due - 0.1), us(t));
          }
          last[burst] = due;
        }
      }
      result[burst] = f.speedMps();
    }
    CHECK(fabsf(result[0] - result[1]) < 0.05f, "batched delivery: %.3f on time, %.3f batched", result[0], result[1]);
  }
  // 5. A GPS outage: the next fix reseeds instead of nudging a drifted estimate.
  {
    SpeedFusion f;
    for (int k = 0; k < 208 * 3; k++) {
      double t = k * IMU_DT;
      f.imuStep(0.0f, us(t));
      if (k % 21 == 0) f.gpsSample(15.0f, epoch(t - 0.1), us(t));
    }
    for (int k = 208 * 3; k < 208 * 13; k++) f.imuStep(0.02f, us(k * IMU_DT));  // 10 s with no fix and a 0.02 g residual
    float drifted = f.speedMps();
    double t = 13.0;
    f.imuStep(0.0f, us(t));
    f.gpsSample(15.0f, epoch(t - 0.1), us(t));
    CHECK(drifted > 16.5f && fabsf(f.speedMps() - 15.0f) < 0.01f, "outage: drifted to %.2f, after the fix %.2f, want 15.00", drifted, f.speedMps());
  }
  // 6. Standstill needs a second of real epochs under the threshold; repeats do not count.
  {
    SpeedFusion f;
    f.imuStep(0.0f, us(0.0));
    f.imuStep(0.0f, us(IMU_DT));
    f.gpsSample(0.0f, epoch(0.0), us(IMU_DT));
    for (int n = 0; n < 40; n++) f.gpsSample(0.0f, epoch(0.0), us(IMU_DT * (2 + n)));  // forty repeats
    CHECK(!f.stationary(), "forty repeats of one epoch declared a standstill");
    for (int n = 1; n <= 11; n++) {
      f.imuStep(0.0f, us(n * 0.1));
      f.gpsSample(0.1f, epoch(n * 0.1), us(n * 0.1));
    }
    CHECK(f.stationary(), "1.1 s of fixes under 0.3 m/s did not declare a standstill");
  }
  // 7. The week rollover is one ordinary 100 ms step.
  {
    SpeedFusion f;
    uint32_t e = 604800000u - 250u;
    for (int k = 0; k < 208 * 2; k++) {
      double t = k * IMU_DT;
      f.imuStep(0.0f, us(t));
      if (k % 21 == 20) {
        f.gpsSample(8.0f, e, us(t));
        e = (e + 100u) % 604800000u;
      }
    }
    CHECK(fabsf(f.speedMps() - 8.0f) < 0.05f && f.seeds() <= 2, "week rollover: %.2f with %u reseeds", f.speedMps(), (unsigned)f.seeds());
  }
  printf("speed fusion: %d checks, %d failed\n", g_run, g_fail);
  printf(g_fail ? "RESULT: FAIL\n" : "RESULT: PASS\n");
  return g_fail ? 1 : 0;
}
