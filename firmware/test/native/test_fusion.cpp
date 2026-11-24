// Native test of the attitude filter and the speed fusion with a synthetic drive. Run with
// test/native/run.sh (clang++ on the Mac, no PlatformIO). Fails with a non-zero exit when a
// bound is exceeded, and prints the figures either way.
//
// Scenario: the IMU is mounted on a dash sloping 12 degrees nose-up with a 3 degree roll, its
// +Y axis forward. Gyro bias 0.6 deg/s on each axis, accelerometer noise 4 mg, gyro noise
// 0.15 deg/s, zero-g offset 25 mg on X. Drive: rest, 0.35 g acceleration with the nose
// lifting 1.5 degrees, cruise, a 0.4 g left corner, 0.5 g braking with the nose dipping
// 2 degrees, rest. GPS at 10 Hz, speed delayed 100 ms with 0.1 m/s noise.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "../../src/app/accel_cal.h"
#include "../../src/app/attitude.h"
#include "../../src/app/speed_fusion.h"

static const float DEG = 0.017453292f;
static float frand() { return (float)rand() / (float)RAND_MAX * 2.0f - 1.0f; }

// Rotation matrices: v' = R v. Rx, Ry, Rz by angle (rad).
static void rotX(float a, float R[3][3]) { float c = cosf(a), s = sinf(a); float M[3][3] = {{1, 0, 0}, {0, c, -s}, {0, s, c}}; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) R[i][j] = M[i][j]; }
static void rotY(float a, float R[3][3]) { float c = cosf(a), s = sinf(a); float M[3][3] = {{c, 0, s}, {0, 1, 0}, {-s, 0, c}}; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) R[i][j] = M[i][j]; }
static void rotZ(float a, float R[3][3]) { float c = cosf(a), s = sinf(a); float M[3][3] = {{c, -s, 0}, {s, c, 0}, {0, 0, 1}}; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) R[i][j] = M[i][j]; }
static void mul(const float A[3][3], const float B[3][3], float C[3][3]) { float T[3][3]; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) { T[i][j] = 0; for (int k = 0; k < 3; k++) T[i][j] += A[i][k] * B[k][j]; } for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) C[i][j] = T[i][j]; }
static void apply(const float R[3][3], const float v[3], float out[3]) { for (int i = 0; i < 3; i++) out[i] = R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2]; }
static void transpose(const float R[3][3], float T[3][3]) { for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) T[i][j] = R[j][i]; }

int main() {
  srand(7);
  // Vehicle frame: x forward, y left, z up. Sensor frame: +Y forward, +X right, +Z up, then
  // tilted by the mount: pitch 12 degrees nose-up (rotation about the vehicle y axis) and
  // roll 3 degrees. sensorFromVehicle = Rmount^T * Raxes, where Raxes maps vehicle (x fwd,
  // y left, z up) into the nominal sensor axes (x right, y fwd, z up): sx = -vy, sy = vx, sz = vz.
  float Raxes[3][3] = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};
  float Rp[3][3], Rr[3][3], Rmount[3][3], RmountT[3][3], S_from_V[3][3];
  rotY(-12.0f * DEG, Rp);  // nose up: the vehicle x axis tips upward
  rotX(3.0f * DEG, Rr);
  mul(Rr, Rp, Rmount);      // vehicle-frame mount rotation
  transpose(Rmount, RmountT);
  mul(Raxes, RmountT, S_from_V);  // sensor <- vehicle (static mount)
  const float forward[3] = {0, 1, 0};
#ifdef NO_BIAS
  const float bias[3] = {0, 0, 0};
#else
  const float bias[3] = {0.6f, -0.6f, 0.6f};
#endif
  const float accelOffset[3] = {0.020f, -0.030f, 0.040f};  // zero-g offsets, datasheet typical
  const float accelScale[3] = {1.00f, 0.99f, 1.015f};        // sensitivity tolerance

  // --- Offset calibration first: the user turns the device through ten orientations, two
  // seconds each, with the sensor's offsets, scale and noise; the fit must recover the offsets.
  AccelCal cal;
  cal.start();
  {
    const float dirs[10][3] = {{0, 0, 1}, {0, 0, -1}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0},
                               {0.7f, 0.7f, 0}, {-0.7f, 0, 0.7f}, {0, -0.7f, -0.7f}, {0.6f, -0.6f, 0.5f}};
    for (int d = 0; d < 10; d++) {
      float n = sqrtf(dirs[d][0] * dirs[d][0] + dirs[d][1] * dirs[d][1] + dirs[d][2] * dirs[d][2]);
      for (int k = 0; k < 416; k++) {  // 2 s, the first 0.3 s "moving"
        float a[3];
        for (int i = 0; i < 3; i++) a[i] = dirs[d][i] / n * accelScale[i] + accelOffset[i] + 0.004f * frand();
        cal.feed(a, k > 60);
      }
    }
  }
  AccelCal::Result cr;
  bool calOk = cal.finish(cr);
  printf("calibration: %s, %d directions, offset %+.4f %+.4f %+.4f (true %+.3f %+.3f %+.3f), radius %.4f, residual %.4f g\n",
         calOk ? "ok" : "FAILED", cr.directions, cr.offset[0], cr.offset[1], cr.offset[2], accelOffset[0], accelOffset[1], accelOffset[2],
         cr.radius, cr.rmsResidual);
  int calFail = !calOk;
  for (int i = 0; i < 3; i++) if (fabsf(cr.offset[i] - accelOffset[i]) > 0.006f) calFail = 1;

  Attitude att;
  TurnGate turn;
  SpeedFusion fus;
  const float dt = 1.0f / 208.0f;
  float t = 0, vTrue = 0, pitchCar = 0, yawCar = 0;
  float gpsNext = 0;
  const int LAG = 21;  // ~100 ms of true speeds at 208 Hz
  float lagBuf[LAG] = {0};
  int lagIdx = 0;
  float bootKpUntil = 3.0f;
  float tVMax = 0;
  double errLon2 = 0, errLat2 = 0, errLonMax = 0, errLatMax = 0, errV2 = 0, errVMax = 0;
  int nMan = 0, nV = 0;
  double restLon = 0, restLat = 0;
  int nRest = 0, nTrustRest = 0;
  float lpLon = 0, lpLat = 0;
  float gpsHist[10] = {0};
  int gpsHistIdx = 0;

  for (int step = 0; t < 70.0f; step++, t += dt) {
    // Truth
    float aLon = 0, aLat = 0, pitchRate = 0, yawRate = 0, pitchTarget = 0;
    if (t >= 5 && t < 10) { aLon = 0.35f; pitchTarget = 1.5f; }
    else if (t >= 20 && t < 25) { aLat = 0.40f; }
    else if (t >= 25 && t < 30) { aLon = -0.50f; pitchTarget = -2.0f; }
    if (t >= 25 && vTrue <= 0.01f) aLon = 0;
    vTrue += aLon * 9.80665f * dt;
    if (vTrue < 0) { vTrue = 0; }
    float dp = (pitchTarget - pitchCar) * dt / 0.3f;  // the body settles with a 0.3 s time constant
    pitchRate = dp / dt;
    pitchCar += dp;
    yawRate = (vTrue > 1 && aLat != 0) ? aLat * 9.80665f / vTrue / DEG : 0;  // deg/s, left positive
    yawCar += yawRate * dt;

    // Specific force in the vehicle frame (x fwd, y left, z up), then tilt by the car's pitch.
    float fV[3] = {aLon, aLat, 1.0f};
    float Rcar[3][3];
    rotY(-pitchCar * DEG, Rcar);  // nose up by pitchCar
    float RcarT[3][3];
    transpose(Rcar, RcarT);
    float fVtilted[3];
    apply(RcarT, fV, fVtilted);  // world-fixed specific force seen in the pitched body
    float fS[3];
    apply(S_from_V, fVtilted, fS);
    // Angular rate in the vehicle frame: pitch rate about -y (nose up = negative pitch about y
    // in a right-handed x-fwd y-left z-up frame), yaw about z.
    float wV[3] = {0, -pitchRate, yawRate};
    float wS[3];
    apply(S_from_V, wV, wS);
    float a[3], w[3];
    for (int i = 0; i < 3; i++) {
      a[i] = (fS[i] * accelScale[i] + accelOffset[i] + 0.004f * frand() - cr.offset[i]) / cr.radius;  // as the firmware corrects it
      w[i] = wS[i] + bias[i] + 0.15f * frand();
    }

    // GPS: 10 Hz, delayed truth plus noise
    lagBuf[lagIdx] = vTrue;
    lagIdx = (lagIdx + 1) % LAG;
    bool gpsFresh = false;
    static float gpsSpeed = 0;
    if (t >= gpsNext) {
      gpsNext += 0.1f;
      gpsSpeed = lagBuf[lagIdx] + 0.1f * frand();
      if (gpsSpeed < 0) gpsSpeed = 0;
      gpsFresh = true;
      gpsHist[gpsHistIdx] = gpsSpeed;
      gpsHistIdx = (gpsHistIdx + 1) % 10;
    }

    // --- What the firmware does ---
    float an = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    float bb[3];
    att.gyroBias(bb);
    float wn = sqrtf((w[0] - bb[0]) * (w[0] - bb[0]) + (w[1] - bb[1]) * (w[1] - bb[1]) + (w[2] - bb[2]) * (w[2] - bb[2]));
    float oldest = gpsHist[gpsHistIdx];
    bool speedSteady = fabsf(gpsSpeed - oldest) < 0.4f;
    bool plausible = att.accelPlausible(a);
    bool turning = turn.update(gpsSpeed, att.yawRateDps(w), dt);
    bool trust = plausible && wn < 10.0f && speedSteady && !turning;
    bool stationary = gpsSpeed < 0.3f && wn < 3.0f;  // GPS evidence, so the bias is learnt too
    (void)an;
    float kp = t < bootKpUntil ? 2.0f : (stationary ? 1.0f : 0.1f);  // standstill: gravity is exact
    att.update(a, w, dt, trust, kp, stationary, stationary);
    float up[3];
    att.up(up);
    float lon, lat;
    vehicleFrameG(a, up, forward, &lon, &lat);
    lpLon += 0.14f * (lon - lpLon);
    lpLat += 0.14f * (lat - lpLat);
    uint32_t nowUs = (uint32_t)((double)step * 1e6 / 208.0);
    fus.imuStep(lon, nowUs);
    // the receiver measured 100 ms before this sample arrives; its epoch says so
    if (gpsFresh) fus.gpsSample(gpsSpeed, (uint32_t)(500000 + (long)lround((t - 0.1) * 1000.0)), nowUs);

    // --- Scoring (after the boot convergence) ---
    if (t > 4.0f) {
      bool manoeuvre = (t >= 5.3f && t < 9.9f) || (t >= 20.3f && t < 24.9f) || (t >= 25.3f && t < 29 && vTrue > 0.5f);
      float eLon = lpLon - aLon, eLat = lpLat - (-aLat);  // lateral positive right; truth y is left
      if (manoeuvre) {
        errLon2 += eLon * eLon; errLat2 += eLat * eLat; nMan++;
        if (fabsf(eLon) > errLonMax) errLonMax = fabsf(eLon);
        if (fabsf(eLat) > errLatMax) errLatMax = fabsf(eLat);
      }
      if (t > 40) { restLon += eLon; restLat += eLat; nRest++; if (trust) nTrustRest++; }
      float eV = fus.speedMps() - vTrue;
      errV2 += eV * eV; nV++;
      if (fabsf(eV) > errVMax && t > 4.5f) { errVMax = fabsf(eV); tVMax = t; }
    }
    if (step % 208 == 0 && ((t > 4 && t < 11) || (t > 19 && t < 31))) {
      float bb2[3];
      att.gyroBias(bb2);
      float zW[3] = {0, 0, 1}, zB[3], upTrue[3];
      apply(RcarT, zW, zB);
      apply(S_from_V, zB, upTrue);
      float dotu = up[0] * upTrue[0] + up[1] * upTrue[1] + up[2] * upTrue[2];
      if (dotu > 1) dotu = 1;
      printf("t %5.1f v %5.2f fused %5.2f | lon %+.3f (true %+.3f) lat %+.3f (true %+.3f) | tilt %.2f up err %.2f deg trust %d stat %d | bias %+.2f %+.2f %+.2f\n", t, vTrue,
             fus.speedMps(), lpLon, aLon, lpLat, -aLat, att.tiltDeg(), acosf(dotu) / DEG, trust, stationary, bb2[0], bb2[1], bb2[2]);
    }
  }
  double rmsLon = sqrt(errLon2 / nMan), rmsLat = sqrt(errLat2 / nMan), rmsV = sqrt(errV2 / nV);
  printf("manoeuvres: lon rms %.4f g max %.4f g | lat rms %.4f g max %.4f g\n", rmsLon, errLonMax, rmsLat, errLatMax);
  printf("rest after the drive: lon mean %+.4f g lat mean %+.4f g (true 0, mount tilt 12/3 degrees)\n", restLon / nRest, restLat / nRest);
  printf("speed: rms %.3f m/s max %.3f m/s at t %.2f\n", rmsV, errVMax, tVMax);
  printf("trust at rest after the drive: %.0f percent of samples (rest |a| learnt %.3f g)\n", 100.0 * nTrustRest / nRest, att.restNorm());
  int fail = 0;
  if (nTrustRest < 0.9 * nRest) { printf("FAIL: gravity not trusted at rest\n"); fail = 1; }
  if (calFail) { printf("FAIL: offset calibration off by more than 6 mg\n"); fail = 1; }
  if (rmsLon > 0.02 || rmsLat > 0.02) { printf("FAIL: G error during manoeuvres over 0.02 g\n"); fail = 1; }
  if (fabs(restLon / nRest) > 0.01 || fabs(restLat / nRest) > 0.01) { printf("FAIL: G at rest after the drive over 0.01 g\n"); fail = 1; }
  if (rmsV > 0.3 || errVMax > 1.0) { printf("FAIL: speed error over 0.3 m/s rms or 1.0 m/s peak\n"); fail = 1; }
  printf(fail ? "RESULT: FAIL\n" : "RESULT: PASS\n");
  return fail;
}
