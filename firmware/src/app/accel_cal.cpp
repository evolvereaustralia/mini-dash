#include "accel_cal.h"

#include <math.h>

namespace {
constexpr float DEG = 0.017453292f;

// Solve the 4x4 system M x = b in place by Gaussian elimination with partial pivoting, in
// double precision. Fails when the smallest pivot is under 1e-6 of the largest: the directions
// collected do not determine all four unknowns, and the answer would be rounding noise.
bool solve4(double M[4][4], double b[4], double x[4]) {
  double pivMax = 0, pivMin = 1e300;
  for (int c = 0; c < 4; c++) {
    int p = c;
    for (int r = c + 1; r < 4; r++)
      if (fabs(M[r][c]) > fabs(M[p][c])) p = r;
    if (p != c) {
      for (int k = 0; k < 4; k++) { double t = M[c][k]; M[c][k] = M[p][k]; M[p][k] = t; }
      double t = b[c]; b[c] = b[p]; b[p] = t;
    }
    double piv = fabs(M[c][c]);
    if (piv > pivMax) pivMax = piv;
    if (piv < pivMin) pivMin = piv;
    if (piv < 1e-12 || pivMin < 1e-6 * pivMax) return false;
    for (int r = c + 1; r < 4; r++) {
      double f = M[r][c] / M[c][c];
      for (int k = c; k < 4; k++) M[r][k] -= f * M[c][k];
      b[r] -= f * b[c];
    }
  }
  for (int r = 3; r >= 0; r--) {
    double v = b[r];
    for (int k = r + 1; k < 4; k++) v -= M[r][k] * x[k];
    x[r] = v / M[r][r];
  }
  return true;
}
}  // namespace

void AccelCal::start() {
  active_ = true;
  dirs_ = 0;
  winHead_ = winN_ = 0;
  lastSpread_ = 0;
}

bool AccelCal::feed(const float a[3], bool quiet) {
  if (!active_) return false;
  if (!quiet) {
    winN_ = 0;
    return false;
  }
  for (int i = 0; i < 3; i++) win_[winHead_][i] = a[i];
  winHead_ = (winHead_ + 1) % WINDOW;
  if (winN_ < WINDOW) winN_++;
  if (winN_ < WINDOW) return false;
  float m[3] = {0, 0, 0};
  for (int k = 0; k < WINDOW; k++)
    for (int i = 0; i < 3; i++) m[i] += win_[k][i];
  for (int i = 0; i < 3; i++) m[i] /= WINDOW;
  float spread = 0;
  for (int k = 0; k < WINDOW; k++) {
    float d2 = 0;
    for (int i = 0; i < 3; i++) d2 += (win_[k][i] - m[i]) * (win_[k][i] - m[i]);
    if (d2 > spread) spread = d2;
  }
  lastSpread_ = sqrtf(spread);
  if (lastSpread_ > STILL_SPREAD_G) return false;
  float n = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
  if (n < 0.5f || n > 1.5f || dirs_ >= MAX_DIRS) return false;
  for (int i = 0; i < dirs_; i++) {
    const float *s = samples_[i];
    float sn = sqrtf(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);
    float c = (m[0] * s[0] + m[1] * s[1] + m[2] * s[2]) / (n * sn);
    if (c > cosf(NEW_DIR_DEG * DEG)) return false;  // seen this direction already
  }
  for (int i = 0; i < 3; i++) samples_[dirs_][i] = m[i];
  dirs_++;
  winN_ = 0;  // a fresh window for the next position
  return true;
}

bool AccelCal::coverage() const {
  for (int k = 0; k < 3; k++) {
    float lo = 10, hi = -10;
    for (int i = 0; i < dirs_; i++) {
      if (samples_[i][k] < lo) lo = samples_[i][k];
      if (samples_[i][k] > hi) hi = samples_[i][k];
    }
    if (hi - lo < 1.2f) return false;
  }
  return true;
}

bool AccelCal::finish(Result &out) {
  active_ = false;
  out = Result();
  out.directions = dirs_;
  if (dirs_ < MIN_DIRS || !coverage()) return false;
  // |a|^2 = 2 a.c + k with k = r^2 - |c|^2: least squares in (cx, cy, cz, k). The samples are
  // taken about their mean first, so the centre being solved for is small and the normal
  // matrix is as well conditioned as the directions allow.
  double mean[3] = {0, 0, 0};
  for (int i = 0; i < dirs_; i++)
    for (int k = 0; k < 3; k++) mean[k] += samples_[i][k];
  for (int k = 0; k < 3; k++) mean[k] /= dirs_;
  double M[4][4] = {}, b[4] = {};
  for (int i = 0; i < dirs_; i++) {
    double s[3] = {samples_[i][0] - mean[0], samples_[i][1] - mean[1], samples_[i][2] - mean[2]};
    double row[4] = {2 * s[0], 2 * s[1], 2 * s[2], 1};
    double y = s[0] * s[0] + s[1] * s[1] + s[2] * s[2];
    for (int r = 0; r < 4; r++) {
      for (int c = 0; c < 4; c++) M[r][c] += row[r] * row[c];
      b[r] += row[r] * y;
    }
  }
  double x[4];
  if (!solve4(M, b, x)) {
    out.conditioned = false;
    return false;
  }
  double r2 = x[3] + x[0] * x[0] + x[1] * x[1] + x[2] * x[2];
  if (r2 < 0.25 || r2 > 2.25) return false;
  double centre[3] = {x[0] + mean[0], x[1] + mean[1], x[2] + mean[2]};
  for (int i = 0; i < 3; i++) out.offset[i] = (float)centre[i];
  out.radius = (float)sqrt(r2);
  double res = 0, worst = 0;
  for (int i = 0; i < dirs_; i++) {
    const float *s = samples_[i];
    double d = sqrt((s[0] - centre[0]) * (s[0] - centre[0]) + (s[1] - centre[1]) * (s[1] - centre[1]) +
                    (s[2] - centre[2]) * (s[2] - centre[2])) - sqrt(r2);
    res += d * d;
    if (fabs(d) > worst) worst = fabs(d);
  }
  out.rmsResidual = (float)sqrt(res / dirs_);
  out.maxResidual = (float)worst;
  if (out.rmsResidual > MAX_RMS_G || out.maxResidual > MAX_SINGLE_G) return false;
  if (fabsf(out.offset[0]) > 0.3f || fabsf(out.offset[1]) > 0.3f || fabsf(out.offset[2]) > 0.3f) return false;
  return true;
}
