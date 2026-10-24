// The G plot shared by Drive and Dynamics (CLAUDE.md section 4): 0.5 g and 1 g rings labelled
// lower-left, acceleration up, braking down, right positive; current sample as a dot with a
// vector, a fading trail, four session peak markers, and the trip history as a binned density
// cloud.
//
// Rendering: the static layer and the cloud live on an lv_canvas in PSRAM, redrawn at most
// once a second; the trail, vector, dot and peak markers are LVGL objects moved every tick,
// so a tick invalidates a few small areas rather than the whole plot (which cost 150 ms a
// frame on the PSRAM framebuffers).
#pragma once

#include "lvgl.h"

class GPlot {
 public:
  // size is the square side in px; the 1 g ring radius is 0.37 * size.
  bool create(lv_obj_t *parent, int x, int y, int size);
  void setSample(float latG, float lonG, bool valid);
  void setPeaks(float left, float right, float accel, float brake);
  void resetHistory();
  // Move the dynamic objects; redraws the cloud if it is due.
  void update(uint32_t nowMs);

 private:
  static constexpr int TRAIL = 40;
  static constexpr int SEGS = 4;
  static constexpr int BINS = 24;  // density grid over +-1.2 g
  lv_obj_t *canvas_ = nullptr;
  lv_color_t *buf_ = nullptr;
  lv_color_t *bg_ = nullptr;
  lv_obj_t *trail_[SEGS] = {};
  lv_point_t trailPts_[SEGS][TRAIL / SEGS + 2] = {};
  lv_obj_t *vector_ = nullptr;
  lv_point_t vectorPts_[2] = {};
  lv_obj_t *halo_ = nullptr, *dot_ = nullptr;
  lv_obj_t *peak_[4] = {};
  int size_ = 0;
  float lat_ = 0, lon_ = 0;
  bool valid_ = false;
  float trailLat_[TRAIL] = {}, trailLon_[TRAIL] = {};
  int trailHead_ = 0, trailCount_ = 0;
  float peakL_ = 0, peakR_ = 0, peakA_ = 0, peakB_ = 0;
  uint16_t bins_[BINS][BINS] = {};
  uint16_t binMax_ = 1;
  uint32_t cloudDrawnMs_ = 0;
  bool cloudDirty_ = true;
  void point(float latG, float lonG, lv_coord_t *x, lv_coord_t *y) const;
  void drawBackground();
  void drawCloud();
  void blendSquare(int x0, int y0, int w, lv_color_t color, uint8_t opa);
};
