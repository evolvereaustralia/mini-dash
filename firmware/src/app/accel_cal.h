// Accelerometer offset calibration (docs/calibration.md). The ISM330DHCX reads 1.02 to 1.05 g
// at rest with tens of mg of zero-g offset per axis. The attitude filter absorbs the offset
// into its up direction at rest, which is harmless until the car yaws: the absorbed error
// swings with the heading and a long corner reads up to 0.05 g of lateral error until the
// next cruise or stop corrects it (native test, 25 Oct 2026). So the offsets are measured
// once per sensor: the user turns the device slowly through at least six orientations,
// quiet half-second averages are collected whenever the direction is new, and a sphere is
// fitted to them (|a - c|^2 = r^2, linear in c and r^2 - |c|^2). The offset c and radius r
// are stored and every sample is corrected as (a - c) / r before anything else sees it.
// Plain C++, tested natively.
#pragma once

#include <stdint.h>

class AccelCal {
 public:
  static constexpr int MAX_DIRS = 16;
  static constexpr int MIN_DIRS = 6;
  static constexpr int ENOUGH_DIRS = 10;  // stop early once this many directions are stored
  static constexpr float NEW_DIR_DEG = 25.0f;

  struct Result {
    float offset[3] = {0, 0, 0};
    float radius = 1;
    float rmsResidual = 0;  // g, distance of the samples from the fitted sphere
    int directions = 0;
  };

  void start();
  bool active() const { return active_; }
  int directions() const { return dirs_; }
  // One raw sample in g; quiet is the caller's coarse gyro veto (fast rotation). Stillness is
  // judged here from the samples themselves: the last WINDOW samples must all lie within
  // STILL_SPREAD_G of their mean, a sliding window so one twitch costs 0.3 s, not the position.
  // Returns true when a new direction was just stored.
  bool feed(const float a[3], bool quiet);
  float lastSpreadG() const { return lastSpread_; }  // diagnostics: current window spread
  // The samples cover each axis both ways (max minus min over 1.2 g on x, y and z).
  bool coverage() const;
  // Fit; false with fewer than MIN_DIRS directions or poor coverage. Ends the collection.
  bool finish(Result &out);
  void cancel() { active_ = false; }

 private:
  bool active_ = false;
  int dirs_ = 0;
  float samples_[MAX_DIRS][3];
  static constexpr int WINDOW = 60;  // 0.3 s at 208 Hz
  static constexpr float STILL_SPREAD_G = 0.03f;
  float win_[WINDOW][3];
  int winHead_ = 0, winN_ = 0;
  float lastSpread_ = 0;
};
