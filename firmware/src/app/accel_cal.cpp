#include "accel_cal.h"

#include <math.h>

namespace {
constexpr float DEG = 0.017453292f;

// Solve the 4x4 system M x = b in place by Gaussian elimination with partial pivoting.
bool solve4(float M[4][4], float b[4], float x[4]) {
  for (int c = 0; c < 4; c++) {
    int p = c;
    for (int r = c + 1; r < 4; r++)
      if (fabsf(M[r][c]) > fabsf(M[p][c])) p = r;
    if (fabsf(M[p][c]) < 1e-9f) return false;
    if (p != c) {
      for (int k = 0; k < 4; k++) { float t = M[c][k]; M[c][k] = M[p][k]; M[p][k] = t; }
      float t = b[c]; b[c] = b[p]; b[p] = t;
    }
    for (int r = c + 1; r < 4; r++) {
      float f = M[r][c] / M[c][c];
      for (int k = c; k < 4; k++) M[r][k] -= f * M[c][k];
      b[r] -= f * b[c];
    }
  }
  for (int r = 3; r >= 0; r--) {
    float s = b[r];
    for (int k = r + 1; k < 4; k++) s -= M[r][k] * x[k];
    x[r] = s / M[r][r];
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
  // |a|^2 = 2 a.c + k with k = r^2 - |c|^2: least squares in (cx, cy, cz, k).
  float M[4][4] = {}, b[4] = {};
  for (int i = 0; i < dirs_; i++) {
    const float *s = samples_[i];
    float row[4] = {2 * s[0], 2 * s[1], 2 * s[2], 1};
    float y = s[0] * s[0] + s[1] * s[1] + s[2] * s[2];
    for (int r = 0; r < 4; r++) {
      for (int c = 0; c < 4; c++) M[r][c] += row[r] * row[c];
      b[r] += row[r] * y;
    }
  }
  float x[4];
  if (!solve4(M, b, x)) return false;
  float r2 = x[3] + x[0] * x[0] + x[1] * x[1] + x[2] * x[2];
  if (r2 < 0.25f || r2 > 2.25f) return false;
  for (int i = 0; i < 3; i++) out.offset[i] = x[i];
  out.radius = sqrtf(r2);
  float res = 0;
  for (int i = 0; i < dirs_; i++) {
    const float *s = samples_[i];
    float d = sqrtf((s[0] - x[0]) * (s[0] - x[0]) + (s[1] - x[1]) * (s[1] - x[1]) + (s[2] - x[2]) * (s[2] - x[2])) - out.radius;
    res += d * d;
  }
  out.rmsResidual = sqrtf(res / dirs_);
  if (fabsf(out.offset[0]) > 0.3f || fabsf(out.offset[1]) > 0.3f || fabsf(out.offset[2]) > 0.3f) return false;
  return true;
}
