#include "gplot.h"

#include <math.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "theme.h"

namespace {
lv_obj_t *circle(lv_obj_t *parent, int d, uint32_t color, lv_opa_t opa, int border) {
  lv_obj_t *o = Theme::box(parent, 0, 0, d, d);
  lv_obj_set_style_radius(o, LV_RADIUS_CIRCLE, 0);
  if (border) {
    lv_obj_set_style_border_width(o, border, 0);
    lv_obj_set_style_border_color(o, Theme::c(color), 0);
  } else {
    lv_obj_set_style_bg_color(o, Theme::c(color), 0);
    lv_obj_set_style_bg_opa(o, opa, 0);
  }
  return o;
}
}  // namespace

bool GPlot::create(lv_obj_t *parent, int x, int y, int size) {
  size_ = size;
  size_t bytes = LV_CANVAS_BUF_SIZE_TRUE_COLOR(size, size);
  buf_ = static_cast<lv_color_t *>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  bg_ = static_cast<lv_color_t *>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (buf_ == nullptr || bg_ == nullptr) return false;
  lv_obj_t *holder = Theme::box(parent, x, y, size, size);
  canvas_ = lv_canvas_create(holder);
  lv_canvas_set_buffer(canvas_, buf_, size, size, LV_IMG_CF_TRUE_COLOR);
  lv_obj_set_pos(canvas_, 0, 0);
  drawBackground();
  drawCloud();

  for (int i = 0; i < 4; i++) {
    peak_[i] = circle(holder, 9, Theme::MUTED, LV_OPA_COVER, 2);
    lv_obj_add_flag(peak_[i], LV_OBJ_FLAG_HIDDEN);
  }
  for (int s = 0; s < SEGS; s++) {
    trail_[s] = lv_line_create(holder);
    lv_obj_set_style_line_color(trail_[s], Theme::c(Theme::LIME), 0);
    lv_obj_set_style_line_width(trail_[s], 3, 0);
    lv_obj_set_style_line_rounded(trail_[s], true, 0);
    lv_obj_set_style_line_opa(trail_[s], (lv_opa_t)(40 + 215 * (s + 1) / SEGS), 0);
    lv_obj_set_pos(trail_[s], 0, 0);
  }
  vector_ = lv_line_create(holder);
  lv_obj_set_style_line_color(vector_, Theme::c(Theme::LIME), 0);
  lv_obj_set_style_line_width(vector_, 1, 0);
  lv_obj_set_style_line_opa(vector_, LV_OPA_40, 0);
  halo_ = circle(holder, 30, Theme::LIME, LV_OPA_20, 0);
  dot_ = circle(holder, 12, Theme::LIME, LV_OPA_COVER, 0);
  lv_obj_add_flag(halo_, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(dot_, LV_OBJ_FLAG_HIDDEN);
  return true;
}

void GPlot::point(float latG, float lonG, lv_coord_t *x, lv_coord_t *y) const {
  float r = 0.37f * size_;
  *x = (lv_coord_t)(size_ / 2 + latG * r);
  *y = (lv_coord_t)(size_ / 2 - lonG * r);
}

void GPlot::setSample(float latG, float lonG, bool valid) {
  lat_ = latG;
  lon_ = lonG;
  valid_ = valid;
  if (!valid) return;
  trailLat_[trailHead_] = latG;
  trailLon_[trailHead_] = lonG;
  trailHead_ = (trailHead_ + 1) % TRAIL;
  if (trailCount_ < TRAIL) trailCount_++;
  if (fabsf(latG) > 0.05f || fabsf(lonG) > 0.05f) {
    int bx = (int)((latG + 1.2f) / 2.4f * BINS), by = (int)((1.2f - lonG) / 2.4f * BINS);
    if (bx >= 0 && bx < BINS && by >= 0 && by < BINS) {
      if (bins_[by][bx] < 65000) bins_[by][bx]++;
      if (bins_[by][bx] > binMax_) binMax_ = bins_[by][bx];
      cloudDirty_ = true;
    }
  }
}

void GPlot::setPeaks(float left, float right, float accel, float brake) {
  peakL_ = left;
  peakR_ = right;
  peakA_ = accel;
  peakB_ = brake;
}

void GPlot::resetHistory() {
  memset(bins_, 0, sizeof(bins_));
  binMax_ = 1;
  trailCount_ = 0;
  trailHead_ = 0;
  cloudDirty_ = true;
}

void GPlot::drawBackground() {
  const int c = size_ / 2;
  const float r1 = 0.37f * size_;
  lv_canvas_fill_bg(canvas_, Theme::c(Theme::BG), LV_OPA_COVER);
  lv_draw_arc_dsc_t arc;
  lv_draw_arc_dsc_init(&arc);
  arc.color = Theme::c(Theme::LINE);
  arc.width = 1;
  lv_canvas_draw_arc(canvas_, c, c, (lv_coord_t)r1, 0, 360, &arc);
  lv_canvas_draw_arc(canvas_, c, c, (lv_coord_t)(r1 / 2), 0, 360, &arc);
  lv_draw_line_dsc_t line;
  lv_draw_line_dsc_init(&line);
  line.color = Theme::c(Theme::LINE);
  line.width = 1;
  lv_point_t h[2] = {{(lv_coord_t)(c - r1), (lv_coord_t)c}, {(lv_coord_t)(c + r1), (lv_coord_t)c}};
  lv_point_t v[2] = {{(lv_coord_t)c, (lv_coord_t)(c - r1)}, {(lv_coord_t)c, (lv_coord_t)(c + r1)}};
  lv_canvas_draw_line(canvas_, h, 2, &line);
  lv_canvas_draw_line(canvas_, v, 2, &line);
  lv_draw_label_dsc_t txt;
  lv_draw_label_dsc_init(&txt);
  txt.color = Theme::c(Theme::MUTED);
  txt.font = &inter_rg_14;
  txt.letter_space = 1;
  txt.align = LV_TEXT_ALIGN_CENTER;
  lv_canvas_draw_text(canvas_, c - 40, 2, 80, &txt, "ACCEL");
  lv_canvas_draw_text(canvas_, c - 40, size_ - 18, 80, &txt, "BRAKE");
  lv_canvas_draw_text(canvas_, 2, c - 8, 24, &txt, "L");
  lv_canvas_draw_text(canvas_, size_ - 26, c - 8, 24, &txt, "R");
  txt.align = LV_TEXT_ALIGN_LEFT;
  lv_canvas_draw_text(canvas_, (lv_coord_t)(c - r1 * 0.5f - 26), (lv_coord_t)(c + r1 * 0.5f - 4), 40, &txt, "0.5g");
  lv_canvas_draw_text(canvas_, (lv_coord_t)(c - r1 * 0.86f - 14), (lv_coord_t)(c + r1 * 0.78f), 30, &txt, "1g");
  memcpy(bg_, buf_, LV_CANVAS_BUF_SIZE_TRUE_COLOR(size_, size_));
}

void GPlot::blendSquare(int x0, int y0, int w, lv_color_t color, uint8_t opa) {
  for (int y = y0; y < y0 + w; y++) {
    if (y < 0 || y >= size_) continue;
    lv_color_t *row = buf_ + y * size_;
    for (int x = x0; x < x0 + w; x++) {
      if (x < 0 || x >= size_) continue;
      row[x] = lv_color_mix(color, row[x], opa);
    }
  }
}

void GPlot::drawCloud() {
  const int c = size_ / 2;
  const float r1 = 0.37f * size_;
  memcpy(buf_, bg_, LV_CANVAS_BUF_SIZE_TRUE_COLOR(size_, size_));
  const float cellPx = 2.4f * r1 / BINS;
  const int sq = (int)(cellPx * 0.5f) > 1 ? (int)(cellPx * 0.5f) : 1;
  const lv_color_t cloud = Theme::c(Theme::CLOUD);
  for (int by = 0; by < BINS; by++) {
    for (int bx = 0; bx < BINS; bx++) {
      uint16_t n = bins_[by][bx];
      if (n == 0) continue;
      float f = (float)n / (float)binMax_;
      uint8_t opa = (uint8_t)(50 + 150 * sqrtf(f));
      int px = (int)(c - 1.2f * r1 + bx * cellPx + cellPx * 0.25f);
      int py = (int)(c - 1.2f * r1 + by * cellPx + cellPx * 0.25f);
      blendSquare(px, py, sq, cloud, opa);
    }
  }
  lv_obj_invalidate(canvas_);
  cloudDirty_ = false;
}

void GPlot::update(uint32_t nowMs) {
  if (canvas_ == nullptr) return;
  if (cloudDirty_ && nowMs - cloudDrawnMs_ >= 1000) {
    cloudDrawnMs_ = nowMs;
    drawCloud();
  }
  lv_coord_t px, py;
  const float pk[4][2] = {{-peakL_, 0}, {peakR_, 0}, {0, peakA_}, {0, -peakB_}};
  for (int i = 0; i < 4; i++) {
    bool have = pk[i][0] != 0 || pk[i][1] != 0;
    if (have) {
      point(pk[i][0], pk[i][1], &px, &py);
      lv_obj_set_pos(peak_[i], px - 4, py - 4);
      lv_obj_clear_flag(peak_[i], LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(peak_[i], LV_OBJ_FLAG_HIDDEN);
    }
  }

  int per = (trailCount_ + SEGS - 1) / SEGS;
  for (int s = 0; s < SEGS; s++) {
    int from = s * per, to = (s + 1) * per;
    int n = 0;
    if (trailCount_ > 1 && from < trailCount_ - 1) {
      if (to > trailCount_ - 1) to = trailCount_ - 1;
      for (int i = from; i <= to && n < TRAIL / SEGS + 2; i++) {
        int idx = (trailHead_ - trailCount_ + i + TRAIL) % TRAIL;
        point(trailLat_[idx], trailLon_[idx], &trailPts_[s][n].x, &trailPts_[s][n].y);
        n++;
      }
    }
    lv_line_set_points(trail_[s], trailPts_[s], n >= 2 ? n : 0);
  }

  if (valid_) {
    point(lat_, lon_, &px, &py);
    vectorPts_[0].x = size_ / 2;
    vectorPts_[0].y = size_ / 2;
    vectorPts_[1].x = px;
    vectorPts_[1].y = py;
    lv_line_set_points(vector_, vectorPts_, 2);
    lv_obj_set_pos(halo_, px - 15, py - 15);
    lv_obj_set_pos(dot_, px - 6, py - 6);
    lv_obj_clear_flag(halo_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(dot_, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_line_set_points(vector_, vectorPts_, 0);
    lv_obj_add_flag(halo_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(dot_, LV_OBJ_FLAG_HIDDEN);
  }
}
