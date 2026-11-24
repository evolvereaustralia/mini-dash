#include "ui.h"

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../app/clock.h"
#include "gplot.h"
#include "theme.h"
#include "vehicle_state.h"

using namespace Theme;

namespace {

Ui::Callbacks g_cb;
const char *g_version = "";
uint32_t g_bootMs = 0;
bool g_shellShown = false;
lv_timer_t *g_tick = nullptr;
VehicleState g_vs;  // the snapshot the tick works on
uint32_t g_now = 0;
uint32_t g_tickMaxMs = 0, g_plotMaxMs = 0;

// Settings state mirrored in the UI.
int g_nightMode = -1;
bool g_demo = true;
bool g_obd = true;
bool g_units_mph = false;
void notifySettings() {
  if (!g_cb.settingsChanged) return;
  Ui::UiSettings st;
  st.mph = g_units_mph;
  st.nightMode = (int8_t)g_nightMode;
  st.demo = g_demo;
  VehicleState tmp;
  Vs::snapshot(tmp);
  st.bestField = tmp.disp.bestField;
  g_cb.settingsChanged(st);
}
Ui::Page g_page = Ui::PAGE_DRIVE;
Ui::Page g_pageBeforeSettings = Ui::PAGE_DRIVE;

// ---- helpers ------------------------------------------------------------------------------
bool setText(lv_obj_t *l, const char *s) {
  if (l == nullptr) return false;
  if (strcmp(lv_label_get_text(l), s) == 0) return false;
  lv_label_set_text(l, s);
  return true;
}
bool setTextF(lv_obj_t *l, const char *fmt, ...) {
  char buf[256];  // 96 cut the event page's context line short (review finding 36)
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  return setText(l, buf);
}
void setColor(lv_obj_t *l, uint32_t color) {
  if (l == nullptr) return;
  if (lv_color_to32(lv_obj_get_style_text_color(l, 0)) != lv_color_to32(c(color))) lv_obj_set_style_text_color(l, c(color), 0);
}
void show(lv_obj_t *o, bool on) {
  if (o == nullptr) return;
  bool hidden = lv_obj_has_flag(o, LV_OBJ_FLAG_HIDDEN);
  if (on && hidden) lv_obj_clear_flag(o, LV_OBJ_FLAG_HIDDEN);
  else if (!on && !hidden) lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
}
void setBorder(lv_obj_t *o, uint32_t color) {
  if (o == nullptr) return;
  if (lv_color_to32(lv_obj_get_style_border_color(o, 0)) != lv_color_to32(c(color))) lv_obj_set_style_border_color(o, c(color), 0);
}
bool fresh(uint32_t stamp, uint32_t maxAge) { return stamp != 0 && g_now - stamp < maxAge; }
// Each car value is judged on its own stamp (review finding 14).
bool carUp() { return g_vs.car.linked && !g_vs.car.asleep; }
bool carItem(uint32_t stamp, uint32_t maxAge) { return carUp() && fresh(stamp, maxAge); }
bool carFast() { return carItem(g_vs.car.fastStampMs, STALE_OBD_FAST_MS); }  // the power figure
bool gpsFresh() { return fresh(g_vs.gps.stampMs, STALE_GPS_MS) && g_vs.gps.fixOk; }
bool motionFresh() { return fresh(g_vs.motion.stampMs, STALE_IMU_MS); }
float kphToDisplay(float kph) { return g_units_mph ? kph * 0.621371f : kph; }
const char *speedUnit() { return g_units_mph ? "mph" : "km/h"; }

// A value label that greys out and shows a dash when invalid.
void setValue(lv_obj_t *l, bool valid, const char *fmt, ...) {
  if (!valid) {
    setText(l, "—");
    setColor(l, MUTED);
    return;
  }
  char buf[96];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  setText(l, buf);
  setColor(l, FG);
}

lv_obj_t *button(lv_obj_t *parent, int x, int y, int w, int h, const char *text, const lv_font_t *font,
                 uint32_t fg, uint32_t bg, lv_event_cb_t cb, void *user) {
  lv_obj_t *b = panel(parent, x, y, w, h, bg, 8);
  lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_t *l = label(b, font, fg, 0, 0, text);
  lv_obj_center(l);
  if (cb) lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, user);
  return b;
}
lv_obj_t *buttonLabel(lv_obj_t *b) { return lv_obj_get_child(b, 0); }

// Segmented control: n options, one highlighted. Returns the container; children are options.
struct Segmented {
  lv_obj_t *box = nullptr;
  lv_obj_t *opt[3] = {};
  int count = 0;
  void select(int i) {
    for (int k = 0; k < count; k++) {
      bool on = k == i;
      lv_obj_set_style_bg_color(opt[k], c(on ? FG : PANEL), 0);
      lv_obj_set_style_text_color(buttonLabel(opt[k]), c(on ? BG : MUTED), 0);
    }
  }
};
Segmented segmented(lv_obj_t *parent, int x, int y, int optW, int h, const char *const *names, int n,
                    lv_event_cb_t cb, const lv_font_t *font = &inter_rg_16) {
  Segmented s;
  s.count = n;
  s.box = panel(parent, x, y, optW * n + 6, h, PANEL, 9);
  for (int i = 0; i < n; i++) {
    s.opt[i] = button(s.box, 3 + i * optW, 3, optW - 2, h - 6, names[i], font, MUTED, PANEL, cb, (void *)(intptr_t)i);
    lv_obj_set_style_radius(s.opt[i], 7, 0);
  }
  return s;
}

// ---- splash -------------------------------------------------------------------------------
struct {
  lv_obj_t *scr = nullptr, *gps = nullptr, *obd = nullptr, *imu = nullptr, *clock = nullptr;
} S;

void buildSplash() {
  S.scr = lv_scr_act();
  lv_obj_remove_style_all(S.scr);
  lv_obj_set_style_bg_color(S.scr, c(BG), 0);
  lv_obj_set_style_bg_opa(S.scr, LV_OPA_COVER, 0);
  // Faint G-plot rings top right.
  lv_obj_t *ring1 = box(S.scr, 800 - 400, -170, 540, 540);
  lv_obj_set_style_radius(ring1, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(ring1, 1, 0);
  lv_obj_set_style_border_color(ring1, c(LINE), 0);
  lv_obj_set_style_border_opa(ring1, LV_OPA_60, 0);
  lv_obj_t *ring2 = box(S.scr, 800 - 400 + 135, -170 + 135, 270, 270);
  lv_obj_set_style_radius(ring2, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(ring2, 1, 0);
  lv_obj_set_style_border_color(ring2, c(LINE), 0);
  lv_obj_set_style_border_opa(ring2, LV_OPA_60, 0);
  lv_obj_t *dot = panel(S.scr, 800 - 400 + 270 + 95, -170 + 270 - 40, 12, 12, LIME, LV_RADIUS_CIRCLE);
  (void)dot;

  lv_obj_t *img = lv_img_create(S.scr);
  lv_img_set_src(img, &img_mini_wings);
  lv_obj_set_pos(img, (800 - 420) / 2, 120);
  lv_obj_t *model = label(S.scr, &inter_sb_26, MUTED, 0, 310, "COOPER SE");
  lv_obj_set_style_text_letter_space(model, 9, 0);
  lv_obj_set_width(model, 800);
  lv_obj_set_style_text_align(model, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_t *rl = panel(S.scr, (800 - 220) / 2, 362, 220, 4, LIME, 2);
  (void)rl;

  S.gps = label(S.scr, &inter_rg_16, MUTED, 20, 448, "GPS starting");
  S.obd = label(S.scr, &inter_rg_16, MUTED, 300, 448, "OBD off");
  S.imu = label(S.scr, &inter_rg_16, MUTED, 520, 448, "IMU starting");
  S.clock = label(S.scr, &inter_rg_16, MUTED, 0, 448, "");
  lv_obj_set_width(S.clock, 780);
  lv_obj_set_style_text_align(S.clock, LV_TEXT_ALIGN_RIGHT, 0);
}

// ---- shell: header, content, tab bar --------------------------------------------------------
struct {
  lv_obj_t *scr = nullptr, *header = nullptr, *content = nullptr, *tabbar = nullptr;
  lv_obj_t *gps = nullptr, *obd = nullptr, *obdL = nullptr, *demo = nullptr, *night = nullptr, *clock = nullptr;
  lv_obj_t *back = nullptr;
  lv_obj_t *tab[Ui::PAGE_COUNT] = {};
  lv_obj_t *tabInd[Ui::PAGE_COUNT] = {};
  lv_obj_t *page[Ui::PAGE_COUNT + 2] = {};  // + settings
} H;
const char *TAB_NAMES[Ui::PAGE_COUNT] = {"DRIVE", "TRIP", "DYNAMICS", "ENERGY", "BATTERY"};

// ---- Drive -----------------------------------------------------------------------------------
struct {
  lv_obj_t *srcPill = nullptr, *srcL = nullptr, *speed = nullptr, *sub = nullptr;
  lv_obj_t *power = nullptr, *powerUnit = nullptr, *pFill = nullptr, *pWord = nullptr;
  lv_obj_t *soc = nullptr, *socPct = nullptr, *socWin = nullptr, *socFill = nullptr;
  lv_obj_t *range = nullptr, *packLine = nullptr;
  lv_obj_t *combined = nullptr, *trip = nullptr, *bestBtn = nullptr, *bestName = nullptr, *bestVal = nullptr;
  GPlot plot;
} D;
constexpr int PBAR_W = 200;
constexpr float PBAR_ZERO = 0.285f;  // 60 kW regen left of zero, 150 kW draw right
const char *BEST_NAMES[REC_COUNT] = {"Best 0–60", "Best 0–100", "Best 60–0", "Lateral best", "Peak draw", "Peak regen"};
const char *BEST_FMT[REC_COUNT] = {"%.1f s", "%.1f s", "%.1f s", "%.2f g", "%.1f kW", "%.1f kW"};

void bestNextCb(lv_event_t *) {
  {
    VsLock lk;
    if (!lk.ok()) return;
    Vs::raw().disp.bestField = (Vs::raw().disp.bestField + 1) % REC_COUNT;
  }
  notifySettings();
}

void buildDrive(lv_obj_t *p) {
  // Baseline grid (content coordinates), each side column spread over the plot's height:
  // group 1 caption 24, big figure 110 (speed) / 92 (SOC), small line 136; group 2 caption
  // 192, figure 226; group 3 on 268 and 292 (scale and "Drawing" left, caption and pack
  // figure right); footer text 346. Left column at x 20, right ending at x 780, plot on 400.
  // left column
  caption(p, 20, yForBaseline(&inter_rg_16, 24), "SPEED");
  D.srcPill = pill(p, 92, 6, "GPS", MUTED);
  D.srcL = buttonLabel(D.srcPill);
  D.speed = blabel(p, &inter_md_88, FG, 16, 110, "—");
  D.sub = blabel(p, &inter_rg_16, MUTED, 20, 136, "km/h");
  // State of charge moves slowly: the smaller slot (Logan, 24 Oct 2026).
  caption(p, 20, yForBaseline(&inter_rg_16, 192), "STATE OF CHARGE");
  D.soc = blabel(p, &inter_md_30, FG, 20, 226, "—");
  lv_obj_t *strack = panel(p, 20, 240, 200, 10, LINE, 3);
  D.socWin = panel(strack, 0, 0, 200, 10, PANEL2, 3);
  D.socFill = panel(strack, 0, 0, 0, 10, LIME, 3);
  blabel(p, &inter_rg_16, MUTED, 20, 268, "Displayed SOC");

  // centre
  D.plot.create(p, 255, 2, 290);

  // right column, ending at x = 780
  // Battery power changes fast: the big slot, with the zero-centred bar beneath.
  brlabel(p, &inter_rg_16, MUTED, 780, 24, 220, "BATTERY POWER");
  lv_obj_set_style_text_letter_space(lv_obj_get_child(p, lv_obj_get_child_cnt(p) - 1), 1, 0);
  D.power = brlabel(p, &inter_md_60, FG, 728, 92, 220, "—");
  D.powerUnit = brlabel(p, &inter_md_28, MUTED, 780, 92, 42, "kW");
  lv_obj_t *track = panel(p, 580, 104, PBAR_W, 10, LINE, 3);
  D.pFill = panel(track, (int)(PBAR_W * PBAR_ZERO), 0, 0, 10, LIME, 0);
  lv_obj_t *zero = panel(track, (int)(PBAR_W * PBAR_ZERO) - 1, -2, 2, 14, FG, 0);
  (void)zero;
  D.pWord = blabel(p, &inter_rg_16, MUTED, 580, 136, "No car data");
  brlabel(p, &inter_rg_14, MUTED, 780, 136, 120, "−60 · +150 kW");
  brlabel(p, &inter_rg_16, MUTED, 780, 192, 200, "RANGE · CAR");
  lv_obj_set_style_text_letter_space(lv_obj_get_child(p, lv_obj_get_child_cnt(p) - 1), 1, 0);
  D.range = brlabel(p, &inter_md_30, FG, 780, 226, 200, "—");
  brlabel(p, &inter_rg_16, MUTED, 780, 268, 200, "PACK · HV");
  lv_obj_set_style_text_letter_space(lv_obj_get_child(p, lv_obj_get_child_cnt(p) - 1), 1, 0);
  D.packLine = brlabel(p, &inter_md_26, FG, 780, 292, 300, "—");

  // footer: three cells on one baseline, left-aligned, centred, right-aligned
  rule(p, 20, 304, 760);
  blabel(p, &inter_rg_16, MUTED, 20, 346, "Combined G");
  D.combined = blabel(p, &inter_md_26, FG, 122, 346, "—");
  lv_obj_t *mid = box(p, 280, 310, 240, 50);
  lv_obj_set_flex_flow(mid, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(mid, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_END);
  lv_obj_set_style_pad_column(mid, 8, 0);
  lv_obj_t *tl = label(mid, &inter_rg_16, MUTED, 0, 0, "Trip");
  lv_obj_set_style_pad_bottom(tl, 2 + 8, 0);  // baseline match with the 26 px value (bl 6 vs 4)
  D.trip = label(mid, &inter_md_26, FG, 0, 0, "—");
  lv_obj_set_style_pad_bottom(D.trip, 8, 0);
  D.bestBtn = box(p, 540, 304, 240, 56);
  lv_obj_add_flag(D.bestBtn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(D.bestBtn, bestNextCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_set_flex_flow(D.bestBtn, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(D.bestBtn, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_END);
  lv_obj_set_style_pad_column(D.bestBtn, 8, 0);
  D.bestName = label(D.bestBtn, &inter_rg_16, MUTED, 0, 0, "Lateral best");
  lv_obj_set_style_pad_bottom(D.bestName, 2 + 8, 0);
  D.bestVal = label(D.bestBtn, &inter_md_26, LIME, 0, 0, "—");
  lv_obj_set_style_pad_bottom(D.bestVal, 8, 0);
  lv_obj_t *cyc = label(D.bestBtn, &inter_rg_18, MUTED, 0, 0, "↻");
  lv_obj_set_style_pad_bottom(cyc, 2 + 8, 0);
}

void updateDrive() {
  bool live = carFast();
  // The figure is shown only while its own source is fresh at this moment: the state can be
  // older than it looks if the publisher stalls (review finding 40).
  bool wheelsSel = g_vs.disp.speedSource == SpeedSource::Wheels;
  bool wheels = wheelsSel && carItem(g_vs.car.wheelStampMs, STALE_OBD_FAST_MS);
  bool haveSpeed = wheels || (g_vs.disp.speedSource == SpeedSource::Gps && gpsFresh());
  setText(D.srcL, wheelsSel ? "WHEELS" : "GPS");
  uint32_t srcColor = !haveSpeed ? AMBER : (wheels ? LIME : MUTED);
  setColor(D.srcL, srcColor);
  setBorder(D.srcPill, srcColor);
  if (haveSpeed) {
    setTextF(D.speed, "%d", (int)lroundf(kphToDisplay(g_vs.disp.speedKph)));
    setColor(D.speed, FG);
  } else {
    setText(D.speed, "—");
    setColor(D.speed, MUTED);
  }
  char hd[8];
  const char *dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  snprintf(hd, sizeof(hd), "%s", dirs[((int)((g_vs.gps.headingDeg + 22.5f) / 45.0f)) & 7]);
  if (wheels && gpsFresh()) setTextF(D.sub, "%s · GPS %d · %03d° %s", speedUnit(), (int)lroundf(kphToDisplay(g_vs.gps.speedMps * 3.6f)), (int)g_vs.gps.headingDeg, hd);
  else if (wheels) setTextF(D.sub, "%s · GPS —", speedUnit());
  else if (gpsFresh()) setTextF(D.sub, "%s · %03d° %s", speedUnit(), (int)g_vs.gps.headingDeg, hd);
  else setTextF(D.sub, "%s · no fix", speedUnit());

  float p = g_vs.car.powerKw;
  if (live) {
    setTextF(D.power, "%+.1f", p);
    setColor(D.power, FG);
    setText(D.pWord, p < 0 ? "Recovering" : "Drawing");
    int zeroX = (int)(PBAR_W * PBAR_ZERO);
    static int lastX = -1, lastW = -1;
    static bool lastDraw = true;
    int x, w;
    bool draw = p >= 0;
    if (draw) {
      w = (int)(fminf(p / 150.0f, 1.0f) * (PBAR_W - zeroX));
      x = zeroX;
    } else {
      w = (int)(fminf(-p / 60.0f, 1.0f) * zeroX);
      x = zeroX - w;
    }
    if (x != lastX || w != lastW) {
      lv_obj_set_pos(D.pFill, x, 0);
      lv_obj_set_width(D.pFill, w);
      lastX = x;
      lastW = w;
    }
    if (draw != lastDraw) {
      lv_obj_set_style_bg_color(D.pFill, c(draw ? LIME : CYAN), 0);
      lastDraw = draw;
    }
  } else {
    setText(D.power, "—");
    setColor(D.power, MUTED);
    setText(D.pWord, g_vs.car.asleep ? "Car asleep" : "No car data");
    lv_obj_set_width(D.pFill, 0);
  }

  if (carItem(g_vs.car.socStampMs, STALE_OBD_SLOW_MS)) {
    setTextF(D.soc, "%d %%", (int)lroundf(g_vs.car.socPct));
    setColor(D.soc, FG);
    int x0 = (int)(g_vs.car.socMinPct * 2), x1 = (int)(g_vs.car.socMaxPct * 2);
    lv_obj_set_pos(D.socWin, x0, 0);
    lv_obj_set_width(D.socWin, x1 - x0);
    lv_obj_set_pos(D.socFill, x0, 0);
    int fill = (int)(g_vs.car.socRawPct * 2) - x0;
    lv_obj_set_width(D.socFill, fill < 0 ? 0 : fill);
  } else {
    setText(D.soc, "—");
    setColor(D.soc, MUTED);
    lv_obj_set_width(D.socFill, 0);
  }
  if (carItem(g_vs.car.rangeStampMs, STALE_OBD_RANGE_MS)) {
    setTextF(D.range, "%d km", (int)lroundf(g_vs.car.rangeKm));
    setColor(D.range, FG);
  } else {
    setText(D.range, "— km");
    setColor(D.range, MUTED);
  }
  if (carItem(g_vs.car.tempStampMs, STALE_OBD_SLOW_MS) && carItem(g_vs.car.voltsStampMs, STALE_OBD_VOLTS_MS)) {
    setTextF(D.packLine, "%.0f °C · %.0f V", g_vs.car.tempAvg, g_vs.car.packV);
    setColor(D.packLine, FG);
  } else {
    setText(D.packLine, "—");
    setColor(D.packLine, MUTED);
  }

  bool mv = motionFresh();
  D.plot.setSample(g_vs.motion.latG, g_vs.motion.lonG, mv);
  D.plot.setPeaks(g_vs.motion.peakLeft, g_vs.motion.peakRight, g_vs.motion.peakAccel, g_vs.motion.peakBrake);
  uint32_t tp = millis();
  D.plot.update(g_now);
  uint32_t dp = millis() - tp;
  if (dp > g_plotMaxMs) g_plotMaxMs = dp;
  setValue(D.combined, mv, "%.2f g", g_vs.motion.combinedG);
  setValue(D.trip, g_vs.trip.energyValid && g_vs.trip.distanceKm > 0.5f, "%.0f Wh/km", g_vs.trip.whPerKm);

  int bf = g_vs.disp.bestField % REC_COUNT;
  const Record &r = g_vs.recTrip.r[bf];
  setText(D.bestName, BEST_NAMES[bf]);
  if (r.valid) {
    setTextF(D.bestVal, BEST_FMT[bf], r.value);
    setColor(D.bestVal, LIME);
  } else {
    setText(D.bestVal, "—");
    setColor(D.bestVal, MUTED);
  }
}

// ---- Trip --------------------------------------------------------------------------------
enum TripView { TV_RECORDS, TV_JOURNEY, TV_EVENT };
struct {
  lv_obj_t *title = nullptr, *ctx = nullptr;
  Segmented scope;
  int scopeIdx = 0;
  TripView view = TV_RECORDS;
  int eventIdx = 0;
  lv_obj_t *grid = nullptr, *tile[REC_COUNT] = {}, *tVal[REC_COUNT] = {}, *tUnit[REC_COUNT] = {},
           *tSub[REC_COUNT] = {}, *tCmp[REC_COUNT] = {};
  lv_obj_t *strip = nullptr, *sDist = nullptr, *sEff = nullptr, *sRec = nullptr, *sSoc = nullptr, *journeyBtn = nullptr;
  lv_obj_t *journey = nullptr, *jVal[12] = {}, *jBack = nullptr;
  lv_obj_t *event = nullptr, *eTitle = nullptr, *eCtx = nullptr, *eMeta = nullptr, *eNote = nullptr, *eBack = nullptr;
  lv_obj_t *eChart[3] = {}, *eLabel[3] = {}, *eT0 = nullptr, *eT1 = nullptr;
  lv_chart_series_t *eSer[3] = {};
  uint32_t eLoadedTrip = 0;
  int eLoadedRec = -1;
  bool eHasTrace = false;
} T;
const char *REC_NAMES[REC_COUNT] = {"BEST 0–60 KM/H", "BEST 0–100 KM/H", "BEST 60–0 BRAKING",
                                    "MAX LATERAL G", "PEAK BATTERY DRAW", "PEAK BATTERY REGEN"};
const char *REC_UNITS[REC_COUNT] = {"s", "s", "s", "g", "kW", "kW"};
const char *REC_FMT[REC_COUNT] = {"%.1f", "%.1f", "%.1f", "%.2f", "%.1f", "%.1f"};
const char *SCOPE_NAMES[3] = {"This trip", "Today", "All time"};
const char *JOURNEY_NAMES[12] = {"DISTANCE", "MOVING TIME", "AVG MOVING SPEED", "MAX SPEED",
                                 "NET ENERGY", "RECOVERED", "EFFICIENCY", "SOC USED",
                                 "BEST 60–100", "MAX BRAKING G", "PEAK ACCEL G", "ALTITUDE"};

void tripShowView(TripView v) {
  T.view = v;
  show(T.grid, v == TV_RECORDS);
  show(T.strip, v == TV_RECORDS);
  show(T.journey, v == TV_JOURNEY);
  show(T.event, v == TV_EVENT);
  show(T.scope.box, v != TV_EVENT);
  show(T.title, v != TV_EVENT);
  show(T.ctx, v != TV_EVENT);
}
void scopeCb(lv_event_t *e) {
  T.scopeIdx = (int)(intptr_t)lv_event_get_user_data(e);
  T.scope.select(T.scopeIdx);
}
void tileCb(lv_event_t *e) {
  T.eventIdx = (int)(intptr_t)lv_event_get_user_data(e);
  tripShowView(TV_EVENT);
}
void journeyCb(lv_event_t *) { tripShowView(TV_JOURNEY); }
void recordsCb(lv_event_t *) { tripShowView(TV_RECORDS); }

void buildTrip(lv_obj_t *p) {
  // Baselines: title 24, context 44; tiles at y 66 and 187 (111 high): caption 22, value 60,
  // sub 82, comparison 100 inside the tile; strip values 334, strip captions 354.
  T.title = caption(p, 20, yForBaseline(&inter_rg_16, 24), "TRIP RECORDS");
  T.ctx = blabel(p, &inter_rg_16, MUTED, 20, 44, "");
  T.scope = segmented(p, 522, 8, 84, 48, SCOPE_NAMES, 3, scopeCb);
  T.scope.select(0);

  T.grid = box(p, 20, 66, 760, 232);
  for (int i = 0; i < REC_COUNT; i++) {
    int col = i % 3, row = i / 3;
    lv_obj_t *t = panel(T.grid, col * 257, row * 121, 246, 111, PANEL, 10);
    lv_obj_add_flag(t, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(t, tileCb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    T.tile[i] = t;
    caption(t, 14, yForBaseline(&inter_rg_14, 22), REC_NAMES[i], &inter_rg_14);
    T.tVal[i] = blabel(t, &inter_md_40, MUTED, 14, 60, "—");
    T.tUnit[i] = blabel(t, &inter_rg_18, MUTED, 120, 60, "");
    T.tSub[i] = blabel(t, &inter_rg_14, MUTED, 14, 82, "No valid run yet");
    T.tCmp[i] = blabel(t, &inter_rg_14, MUTED, 14, 100, "");
  }
  T.strip = box(p, 20, 304, 760, 56);
  auto kv = [&](int x, const char *cap, lv_obj_t **val) {
    *val = blabel(T.strip, &inter_md_26, FG, x, 30, "—");
    blabel(T.strip, &inter_rg_14, MUTED, x, 50, cap);
  };
  kv(0, "distance", &T.sDist);
  kv(150, "efficiency", &T.sEff);
  kv(330, "recovered", &T.sRec);
  kv(490, "SOC used", &T.sSoc);
  T.journeyBtn = button(T.strip, 606, 4, 154, 48, "Journey totals →", &inter_rg_16, FG, PANEL, journeyCb, nullptr);

  T.journey = box(p, 20, 66, 760, 294);
  for (int i = 0; i < 12; i++) {
    int col = i % 4, row = i / 4;
    caption(T.journey, col * 193, yForBaseline(&inter_rg_14, row * 68 + 14), JOURNEY_NAMES[i], &inter_rg_14);
    T.jVal[i] = blabel(T.journey, &inter_md_30, FG, col * 193, row * 68 + 46, "—");
  }
  blabel(T.journey, &inter_rg_14, MUTED, 0, 226, "Moving time excludes stops · 60–100 is a rolling run, counted on its own");
  T.jBack = button(T.journey, 646, 240, 114, 48, "← Records", &inter_rg_16, FG, PANEL, recordsCb, nullptr);
  show(T.journey, false);

  T.event = box(p, 20, 0, 760, 366);  // replaces the title block while a record is open
  T.eCtx = caption(T.event, 0, yForBaseline(&inter_rg_16, 24), "THIS TRIP");
  T.eTitle = blabel(T.event, &inter_md_30, FG, 0, 58, "");
  // Three traces on a common 12 s axis, each with its own scale: speed 0..120 km/h, G -1..1,
  // battery -60..160 kW. Rows at y 66, 128, 190, each 52 px high.
  static const char *TR_NAME[3] = {"Speed · km/h", "G · lat / lon", "Battery · kW"};
  static const int TR_MIN[3] = {0, -1000, -600}, TR_MAX[3] = {1200, 1000, 1600};
  static const uint32_t TR_COLOR[3] = {FG, CYAN, LIME};
  for (int i = 0; i < 3; i++) {
    int y = 74 + i * 62;
    T.eLabel[i] = blabel(T.event, &inter_rg_14, MUTED, 0, y + 20, TR_NAME[i]);
    T.eChart[i] = lv_chart_create(T.event);
    lv_obj_remove_style_all(T.eChart[i]);
    lv_obj_set_pos(T.eChart[i], 160, y);
    lv_obj_set_size(T.eChart[i], 580, 52);
    lv_chart_set_type(T.eChart[i], LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(T.eChart[i], Storage::TRACE_MAX);
    lv_chart_set_range(T.eChart[i], LV_CHART_AXIS_PRIMARY_Y, TR_MIN[i], TR_MAX[i]);
    lv_chart_set_div_line_count(T.eChart[i], 0, 0);
    lv_obj_set_style_size(T.eChart[i], 0, LV_PART_INDICATOR);
    lv_obj_set_style_line_width(T.eChart[i], 2, LV_PART_ITEMS);
    T.eSer[i] = lv_chart_add_series(T.eChart[i], c(TR_COLOR[i]), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_all_value(T.eChart[i], T.eSer[i], LV_CHART_POINT_NONE);
    rule(T.event, 160, y + 51, 580);
  }
  T.eT0 = blabel(T.event, &inter_rg_14, MUTED, 160, 276, "0 s");
  T.eT1 = brlabel(T.event, &inter_rg_14, MUTED, 740, 276, 60, "12 s");
  T.eMeta = label(T.event, &inter_rg_16, MUTED, 0, 290, "");
  lv_label_set_long_mode(T.eMeta, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(T.eMeta, 740);
  T.eNote = blabel(T.event, &inter_rg_14, MUTED, 160, 112, "No trace stored for this record.");
  show(T.eNote, false);
  T.eBack = button(T.event, 646, 8, 114, 48, "← Records", &inter_rg_16, FG, PANEL, recordsCb, nullptr);
  show(T.event, false);
}

const RecordSet &scopeSet() { return T.scopeIdx == 0 ? g_vs.recTrip : T.scopeIdx == 1 ? g_vs.recToday : g_vs.recAll; }

void updateTrip() {
  const RecordSet &rs = scopeSet();
  if (T.scopeIdx == 0) {
    uint32_t mins = (g_now - g_vs.trip.startMs) / 60000;
    setTextF(T.ctx, "%s · %lu min · %.1f km", g_vs.demo ? "Demo replay" : "This trip", (unsigned long)mins, g_vs.trip.distanceKm);
  } else if (T.scopeIdx == 1) {
    setTextF(T.ctx, "Today · %u earlier trip%s · %.1f km before this one", g_vs.store.todayTrips,
             g_vs.store.todayTrips == 1 ? "" : "s", g_vs.store.todayKm);
  } else {
    setTextF(T.ctx, "Lifetime · %u trip%s stored · %.0f km", g_vs.store.allTrips, g_vs.store.allTrips == 1 ? "" : "s",
             g_vs.store.allKm);
  }
  for (int i = 0; i < REC_COUNT; i++) {
    const Record &r = rs.r[i];
    if (r.valid) {
      bool moved = setTextF(T.tVal[i], REC_FMT[i], r.value);
      setColor(T.tVal[i], FG);
      moved |= setText(T.tUnit[i], REC_UNITS[i]);
      if (moved) {
        lv_point_t sz;
        lv_txt_get_size(&sz, lv_label_get_text(T.tVal[i]), &inter_md_40, -1, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
        lv_obj_set_x(T.tUnit[i], 14 + sz.x + 6);
      }
      char sub[48];
      switch (i) {
        case REC_0_60:
        case REC_0_100: snprintf(sub, sizeof(sub), "%u valid run%s · %s", r.runs, r.runs == 1 ? "" : "s", r.when); break;
        case REC_60_0: snprintf(sub, sizeof(sub), "%.0f m · %s", r.aux, r.when); break;
        case REC_LAT: snprintf(sub, sizeof(sub), "%s · %s", r.aux < 0 ? "left" : "right", r.when); break;
        default: snprintf(sub, sizeof(sub), "1 s mean · max %.1f · %s", r.aux, r.when); break;
      }
      setText(T.tSub[i], sub);
      float d = r.hasPrevious ? r.value - r.previous : 0;
      float minStep = i == REC_LAT ? 0.01f : 0.1f;
      if (r.hasPrevious && fabsf(d) >= minStep) {
        setTextF(T.tCmp[i], "%+.*f %s vs previous", i == REC_LAT ? 2 : 1, d, REC_UNITS[i]);
      } else if (r.hasPrevious) {
        setText(T.tCmp[i], "Best of this trip");
      } else {
        setText(T.tCmp[i], T.scopeIdx == 0 ? "First of this trip" : "First recorded");
      }
      setColor(T.tCmp[i], r.beatLifetime ? LIME : MUTED);
    } else {
      setText(T.tVal[i], "—");
      setColor(T.tVal[i], MUTED);
      setText(T.tUnit[i], "");
      setText(T.tSub[i], "No valid run yet");
      setText(T.tCmp[i], "");
    }
  }
  setTextF(T.sDist, "%.1f km", g_vs.trip.distanceKm);
  setValue(T.sEff, g_vs.trip.energyValid && g_vs.trip.distanceKm > 0.5f, "%.0f Wh/km", g_vs.trip.whPerKm);
  setValue(T.sRec, g_vs.trip.energyValid, "%.2f kWh", g_vs.trip.recoveredKwh);
  setValue(T.sSoc, g_vs.trip.energyValid && g_vs.trip.socStartPct >= 0, "%.0f %%", g_vs.trip.socUsedPct);

  if (T.view == TV_JOURNEY) {
    const TripData &t = g_vs.trip;
    setTextF(T.jVal[0], "%.1f km", t.distanceKm);
    setTextF(T.jVal[1], "%02d:%02d h:mm", (int)(t.movingS / 3600), (int)(t.movingS / 60) % 60);
    setTextF(T.jVal[2], "%.1f %s", kphToDisplay(t.avgMovingKph), speedUnit());
    setTextF(T.jVal[3], "%.0f %s", kphToDisplay(t.maxKph), speedUnit());
    setValue(T.jVal[4], t.energyValid, "%.2f kWh", t.netKwh);
    setValue(T.jVal[5], t.energyValid, "%.2f kWh", t.recoveredKwh);
    setValue(T.jVal[6], t.energyValid && t.distanceKm > 0.5f, "%.0f Wh/km", t.whPerKm);
    setValue(T.jVal[7], t.energyValid && t.socStartPct >= 0, "%.0f %%", t.socUsedPct);
    setValue(T.jVal[8], t.best60to100S > 0, "%.1f s", t.best60to100S);
    setValue(T.jVal[9], g_vs.motion.peakBrake > 0, "%.2f g", g_vs.motion.peakBrake);
    setValue(T.jVal[10], g_vs.motion.peakAccel > 0, "%.2f g", g_vs.motion.peakAccel);
    setValue(T.jVal[11], gpsFresh(), "%.0f m", g_vs.gps.altM);
  }
  if (T.view == TV_EVENT) {
    const Record &r = rs.r[T.eventIdx];
    if (r.valid && (r.tripId != T.eLoadedTrip || T.eventIdx != T.eLoadedRec)) {
      T.eLoadedTrip = r.tripId;
      T.eLoadedRec = T.eventIdx;
      static Storage::TraceSample tr[Storage::TRACE_MAX];
      size_t n = 0;
      T.eHasTrace = g_cb.loadTrace && g_cb.loadTrace(r.tripId, T.eventIdx, tr, Storage::TRACE_MAX, &n) && n > 0;
      for (int k = 0; k < 3; k++) {
        // The trace spans the chart width whatever its length (a record accepted 8 s after boot
        // has an 8 s window), so the right-hand time label sits where the line ends.
        lv_chart_set_point_count(T.eChart[k], T.eHasTrace ? n : Storage::TRACE_MAX);
        lv_chart_set_all_value(T.eChart[k], T.eSer[k], LV_CHART_POINT_NONE);
      }
      if (T.eHasTrace) {
        for (size_t i = 0; i < n; i++) {
          lv_chart_set_value_by_id(T.eChart[0], T.eSer[0], i, tr[i].kph10);
          // G trace: lateral for the lateral record, longitudinal otherwise
          lv_chart_set_value_by_id(T.eChart[1], T.eSer[1], i, T.eventIdx == REC_LAT ? tr[i].latMg : tr[i].lonMg);
          lv_chart_set_value_by_id(T.eChart[2], T.eSer[2], i, tr[i].kw10);
        }
        setTextF(T.eT1, "%.1f s", (tr[n - 1].t10) / 10.0f);
      }
      for (int k = 0; k < 3; k++) show(T.eChart[k], T.eHasTrace);
      show(T.eNote, !T.eHasTrace);
    }
    setTextF(T.eCtx, "%s · %s", SCOPE_NAMES[T.scopeIdx], r.valid ? r.when : "");
    char val[32];
    if (r.valid) snprintf(val, sizeof(val), REC_FMT[T.eventIdx], r.value);
    else snprintf(val, sizeof(val), "—");
    const char *names[REC_COUNT] = {"Best 0–60 km/h", "Best 0–100 km/h", "Best 60–0 braking",
                                    "Max lateral G", "Peak battery draw", "Peak battery regen"};
    setTextF(T.eTitle, "%s   %s %s", names[T.eventIdx], val, r.valid ? REC_UNITS[T.eventIdx] : "");
    if (!r.valid) {
      setText(T.eMeta, "No valid run yet.");
      for (int k = 0; k < 3; k++) show(T.eChart[k], false);
      show(T.eNote, false);
    } else {
      char ctx[40];
      if (r.socPct >= 0 && r.packTemp > -99) snprintf(ctx, sizeof(ctx), "SOC %.0f %% · Pack %.0f °C", r.socPct, r.packTemp);
      else if (r.socPct >= 0) snprintf(ctx, sizeof(ctx), "SOC %.0f %% · Pack —", r.socPct);
      else if (r.packTemp > -99) snprintf(ctx, sizeof(ctx), "SOC — · Pack %.0f °C", r.packTemp);
      else snprintf(ctx, sizeof(ctx), "No car data at the event");
      switch (T.eventIdx) {
        case REC_0_60:
        case REC_0_100:
          setTextF(T.eMeta, "%s · Stationary start, no rollout · %u valid run%s · GPS gate: 3D fix, 7+ sats, speed accuracy under 0.5 m/s · crossing interpolated between samples",
                   ctx, r.runs, r.runs == 1 ? "" : "s");
          break;
        case REC_60_0:
          setTextF(T.eMeta, "%s · Timed from 60 km/h · Distance %.0f m · Max braking %.2f g this session",
                   ctx, r.aux, g_vs.motion.peakBrake);
          break;
        case REC_LAT:
          setTextF(T.eMeta, "%s · Filtered IMU peak (5 Hz path) · %s turn · Session L %.2f · R %.2f g",
                   ctx, r.aux < 0 ? "Left" : "Right", g_vs.motion.peakLeft, g_vs.motion.peakRight);
          break;
        default:
          setTextF(T.eMeta, "%s · One-second mean %.1f kW · Instantaneous max %.1f kW · Battery-side power, not motor output",
                   ctx, r.value, r.aux);
          break;
      }
    }
  }
}

// ---- Dynamics --------------------------------------------------------------------------------
struct {
  GPlot plot;
  lv_obj_t *lat = nullptr, *lon = nullptr, *comb = nullptr, *yaw = nullptr;
  lv_obj_t *pk[4] = {}, *reset = nullptr;
  lv_obj_t *torque = nullptr, *torqueReq = nullptr, *pedal = nullptr, *motor = nullptr;
} Y;
void resetPeaksCb(lv_event_t *) {
  if (g_cb.resetPeaks) g_cb.resetPeaks();
  Y.plot.resetHistory();
  D.plot.resetHistory();
}
void buildDynamics(lv_obj_t *p) {
  Y.plot.create(p, 20, 10, 330);
  const int X = 366, W = 414;
  // Rows of 48 px: label and value share the baseline 30 px into the row; rule at 46.
  auto row = [&](int y, const char *name, const lv_font_t *font, lv_obj_t **val) {
    blabel(p, &inter_rg_16, MUTED, X, y + 30, name);
    *val = brlabel(p, font, FG, 780, y + 30, 230, "—");
    rule(p, X, y + 46, W);
  };
  row(0, "Lateral", &inter_md_28, &Y.lat);
  row(48, "Longitudinal", &inter_md_28, &Y.lon);
  row(96, "Combined", &inter_md_28, &Y.comb);
  row(144, "Yaw rate", &inter_md_28, &Y.yaw);
  // Peaks: four values on baseline 222 with captions on 240, reset button centred on them.
  blabel(p, &inter_rg_16, MUTED, X, 222, "Peaks");
  static const char *PK_CAP[4] = {"L", "R", "ACC", "BRK"};
  for (int i = 0; i < 4; i++) {
    Y.pk[i] = brlabel(p, &inter_md_26, FG, 486 + i * 68, 222, 64, "—");
    lv_obj_t *cp = brlabel(p, &inter_rg_14, MUTED, 486 + i * 68, 240, 64, PK_CAP[i]);
    lv_obj_set_style_text_letter_space(cp, 1, 0);
  }
  Y.reset = button(p, 710, 200, 70, 34, "RESET", &inter_rg_14, LIME, PANEL, resetPeaksCb, nullptr);
  lv_obj_set_style_border_width(Y.reset, 1, 0);
  lv_obj_set_style_border_color(Y.reset, c(LINE), 0);
  rule(p, X, 250, W);
  // Two drivetrain rows of 42 px: baseline 28 into the row, rule at 40.
  auto sub = [&](int y, const char *name, lv_obj_t **val) {
    blabel(p, &inter_rg_16, MUTED, X, y + 28, name);
    *val = brlabel(p, &inter_md_26, FG, 780, y + 28, 230, "—");
    rule(p, X, y + 40, W);
  };
  sub(256, "Torque actual / requested", &Y.torque);
  sub(298, "Pedal · motor temps", &Y.pedal);
}

void updateDynamics() {
  bool mv = motionFresh();
  Y.plot.setSample(g_vs.motion.latG, g_vs.motion.lonG, mv);
  Y.plot.setPeaks(g_vs.motion.peakLeft, g_vs.motion.peakRight, g_vs.motion.peakAccel, g_vs.motion.peakBrake);
  Y.plot.update(g_now);
  setValue(Y.lat, mv, "%+.2f g", g_vs.motion.latG);
  setValue(Y.lon, mv, "%+.2f g", g_vs.motion.lonG);
  setValue(Y.comb, mv, "%.2f g", g_vs.motion.combinedG);
  setValue(Y.yaw, mv, "%+.1f °/s", g_vs.motion.yawDps);
  const float pk[4] = {g_vs.motion.peakLeft, g_vs.motion.peakRight, g_vs.motion.peakAccel, g_vs.motion.peakBrake};
  for (int i = 0; i < 4; i++) setValue(Y.pk[i], pk[i] > 0, "%.2f", pk[i]);
  if (carItem(g_vs.car.driveStampMs, STALE_OBD_DRIVE_MS)) {
    setTextF(Y.torque, "%.0f / %.0f Nm", g_vs.car.torqueNm, g_vs.car.torqueReqNm);
    setColor(Y.torque, FG);
  } else {
    setText(Y.torque, "—");
    setColor(Y.torque, MUTED);
  }
  bool pedal = carItem(g_vs.car.pedalStampMs, STALE_OBD_DRIVE_MS), mtemp = carItem(g_vs.car.motorTempStampMs, STALE_OBD_SLOW_MS);
  if (pedal && mtemp) setTextF(Y.pedal, "%.0f %%  ·  %.0f / %.0f °C", g_vs.car.pedalPct, g_vs.car.motorTemp1, g_vs.car.motorTemp2);
  else if (pedal) setTextF(Y.pedal, "%.0f %%  ·  —", g_vs.car.pedalPct);
  else if (mtemp) setTextF(Y.pedal, "—  ·  %.0f / %.0f °C", g_vs.car.motorTemp1, g_vs.car.motorTemp2);
  else setText(Y.pedal, "—");
  setColor(Y.pedal, pedal || mtemp ? FG : MUTED);
}

// ---- Energy and Battery (charts) ------------------------------------------------------------
lv_obj_t *makeChart(lv_obj_t *p, int x, int y, int w, int h, int ymin, int ymax, lv_chart_series_t **ser, uint32_t color) {
  lv_obj_t *ch = lv_chart_create(p);
  lv_obj_remove_style_all(ch);
  lv_obj_set_pos(ch, x, y);
  lv_obj_set_size(ch, w, h);
  lv_chart_set_type(ch, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(ch, 120);
  lv_chart_set_range(ch, LV_CHART_AXIS_PRIMARY_Y, ymin, ymax);
  lv_chart_set_update_mode(ch, LV_CHART_UPDATE_MODE_SHIFT);
  lv_chart_set_div_line_count(ch, 0, 0);
  lv_obj_set_style_line_width(ch, 3, LV_PART_ITEMS);
  lv_obj_set_style_size(ch, 0, LV_PART_INDICATOR);
  *ser = lv_chart_add_series(ch, c(color), LV_CHART_AXIS_PRIMARY_Y);
  lv_chart_set_all_value(ch, *ser, LV_CHART_POINT_NONE);
  return ch;
}
struct {
  lv_obj_t *avg = nullptr, *net = nullptr, *rec = nullptr, *soc = nullptr, *chart = nullptr, *note = nullptr, *nodata = nullptr;
  lv_chart_series_t *ser = nullptr;
  uint32_t lastPoint = 0;
} E;
struct {
  lv_obj_t *v = nullptr, *a = nullptr, *temp = nullptr, *spread = nullptr, *chart = nullptr;
  lv_obj_t *cells = nullptr, *v12 = nullptr, *soh = nullptr, *nodata = nullptr;
  lv_chart_series_t *ser = nullptr;
  uint32_t lastPoint = 0;
} B;

lv_obj_t *statCol(lv_obj_t *p, int x, const char *cap, lv_obj_t **val) {
  caption(p, x, yForBaseline(&inter_rg_16, 24), cap);
  *val = blabel(p, &inter_md_30, FG, x, 58, "—");
  return *val;
}
// Chart frame: axis labels centred on the exact value lines, which we draw ourselves.
void chartFrame(lv_obj_t *p, const char *unit, const char *const *names, const int *lineY, int n) {
  brlabel(p, &inter_rg_14, MUTED, 60, 84, 40, unit);
  for (int i = 0; i < n; i++) {
    brlabel(p, &inter_rg_14, MUTED, 60, lineY[i] + 5, 40, names[i]);
    rule(p, 70, lineY[i], 710);
  }
  blabel(p, &inter_rg_14, MUTED, 70, 292, "−10 min");
  brlabel(p, &inter_rg_14, MUTED, 780, 292, 60, "now");
}
lv_obj_t *noData(lv_obj_t *p) {
  lv_obj_t *o = panel(p, 300, 157, 200, 56, PANEL, 10);
  lv_obj_set_style_border_width(o, 1, 0);
  lv_obj_set_style_border_color(o, c(LINE), 0);
  lv_obj_t *l = label(o, &inter_rg_18, MUTED, 0, 0, "NO CAR DATA");
  lv_obj_set_style_text_letter_space(l, 2, 0);
  lv_obj_center(l);
  return o;
}
void buildEnergy(lv_obj_t *p) {
  statCol(p, 20, "TRIP AVERAGE", &E.avg);
  statCol(p, 215, "NET ENERGY", &E.net);
  statCol(p, 410, "RECOVERED", &E.rec);
  statCol(p, 605, "SOC USED", &E.soc);
  // range -50..100 over y 100..270: +100 at 100, 0 at 213, -50 at 270
  static const char *names[3] = {"+100", "0", "−50"};
  static const int lines[3] = {100, 213, 270};
  chartFrame(p, "kW", names, lines, 3);
  E.chart = makeChart(p, 70, 100, 710, 170, -50, 100, &E.ser, LIME);
  rule(p, 20, 306, 760);
  E.note = blabel(p, &inter_rg_16, MUTED, 20, 340, "Battery power, last 10 min · above the line draws, below recovers · Wh/km shown only above 15 km/h");
  E.nodata = noData(p);
}
void updateEnergy() {
  bool live = carFast();
  setValue(E.avg, g_vs.trip.energyValid && g_vs.trip.distanceKm > 0.5f, "%.0f Wh/km", g_vs.trip.whPerKm);
  setValue(E.net, g_vs.trip.energyValid, "%.2f kWh", g_vs.trip.netKwh);
  setValue(E.rec, g_vs.trip.energyValid, "%.2f kWh", g_vs.trip.recoveredKwh);
  setValue(E.soc, g_vs.trip.energyValid && g_vs.trip.socStartPct >= 0, "%.0f %%", g_vs.trip.socUsedPct);
  if (g_now - E.lastPoint >= 5000) {  // 120 points over 10 min
    E.lastPoint = g_now;
    lv_chart_set_next_value(E.chart, E.ser, live ? (lv_coord_t)lroundf(g_vs.car.powerKw) : LV_CHART_POINT_NONE);
  }
  show(E.nodata, !live);
}
void buildBattery(lv_obj_t *p) {
  statCol(p, 20, "PACK VOLTAGE", &B.v);
  statCol(p, 200, "CURRENT", &B.a);
  statCol(p, 360, "PACK TEMP MIN · AVG · MAX", &B.temp);
  statCol(p, 630, "CELL SPREAD", &B.spread);
  // range 10..40 over y 100..270: 40 at 100, 25 at 185, 10 at 270
  static const char *names[3] = {"40", "25", "10"};
  static const int lines[3] = {100, 185, 270};
  chartFrame(p, "°C", names, lines, 3);
  B.chart = makeChart(p, 70, 100, 710, 170, 10, 40, &B.ser, LIME);
  rule(p, 20, 306, 760);
  // footer: three cells on baseline 346, like Drive
  blabel(p, &inter_rg_16, MUTED, 20, 346, "Cells avg / min / max");
  B.cells = blabel(p, &inter_md_26, FG, 190, 346, "—");
  blabel(p, &inter_rg_16, MUTED, 466, 346, "12 V");
  B.v12 = blabel(p, &inter_md_26, FG, 506, 346, "—");
  blabel(p, &inter_rg_16, MUTED, 616, 346, "BMS health");
  B.soh = brlabel(p, &inter_md_26, FG, 780, 346, 72, "—");
  B.nodata = noData(p);
}
void updateBattery() {
  bool volts = carItem(g_vs.car.voltsStampMs, STALE_OBD_VOLTS_MS), amps = carItem(g_vs.car.ampsStampMs, STALE_OBD_FAST_MS);
  bool temps = carItem(g_vs.car.tempStampMs, STALE_OBD_SLOW_MS), cells = carItem(g_vs.car.cellStampMs, STALE_OBD_SLOW_MS);
  setValue(B.v, volts, "%.1f V", g_vs.car.packV);
  setValue(B.a, amps, "%+.0f A", g_vs.car.packA);
  setValue(B.temp, temps, "%.0f·%.0f·%.0f °C", g_vs.car.tempMin, g_vs.car.tempAvg, g_vs.car.tempMax);
  setValue(B.spread, cells, "%.0f mV", (g_vs.car.cellMaxV - g_vs.car.cellMinV) * 1000.0f);
  setValue(B.cells, cells, "%.2f · %.2f · %.2f V", g_vs.car.cellAvgV, g_vs.car.cellMinV, g_vs.car.cellMaxV);
  setValue(B.v12, carItem(g_vs.car.v12StampMs, STALE_OBD_SLOW_MS), "%.1f V", g_vs.car.v12);
  setValue(B.soh, carUp() && g_vs.car.sohStampMs != 0, "%.0f %%", g_vs.car.sohPct);  // read once per connection
  if (g_now - B.lastPoint >= 5000) {
    B.lastPoint = g_now;
    lv_chart_set_next_value(B.chart, B.ser, temps ? (lv_coord_t)lroundf(g_vs.car.tempAvg) : LV_CHART_POINT_NONE);
  }
  show(B.nodata, !(volts || amps || temps || cells));
}

// ---- Settings ----------------------------------------------------------------------------
struct {
  Segmented night, demo, units, obd;
  lv_obj_t *forget = nullptr, *forgetL = nullptr;
  lv_obj_t *calib = nullptr, *calibL = nullptr, *accelCal = nullptr, *accelCalL = nullptr, *diag = nullptr, *about = nullptr, *bright = nullptr, *dongle = nullptr;
} G;
void nightCb(lv_event_t *e) {
  int i = (int)(intptr_t)lv_event_get_user_data(e);
  g_nightMode = i == 0 ? -1 : (i == 1 ? 1 : 0);
  G.night.select(i);
  if (g_cb.setNightMode) g_cb.setNightMode(g_nightMode);
  notifySettings();
}
void demoCb(lv_event_t *e) {
  int i = (int)(intptr_t)lv_event_get_user_data(e);
  g_demo = i == 1;
  G.demo.select(i);
  if (g_cb.setDemo) g_cb.setDemo(g_demo);
  notifySettings();
}
void obdCb(lv_event_t *e) {
  int i = (int)(intptr_t)lv_event_get_user_data(e);
  g_obd = i == 1;
  G.obd.select(i);
  if (g_cb.setObd) g_cb.setObd(g_obd);
}
// One button, two meanings: with a dongle stored it forgets it, with none it starts pairing.
// The label follows the stored state, not the tap (review finding 34).
void forgetCb(lv_event_t *) {
  if (g_vs.diag.obdBound) {
    if (g_cb.forgetDongle) g_cb.forgetDongle();
  } else if (g_cb.pairDongle) {
    g_cb.pairDongle();
  }
}
void unitsCb(lv_event_t *e) {
  int i = (int)(intptr_t)lv_event_get_user_data(e);
  g_units_mph = i == 1;
  G.units.select(i);
  notifySettings();
}
void calibCb(lv_event_t *) {
  if (g_cb.calibrateMounting) g_cb.calibrateMounting();
  setText(G.calibL, "Taken");
}
void accelCalCb(lv_event_t *) {
  if (g_vs.diag.accCalState == 1) return;  // already collecting
  if (g_cb.calibrateAccel) g_cb.calibrateAccel();
  setText(G.accelCalL, "Turn it");
}
void resetTripCb(lv_event_t *) {
  if (g_cb.resetTrip) g_cb.resetTrip();
  D.plot.resetHistory();
  Y.plot.resetHistory();
}
void clearRecordsCb(lv_event_t *) {
  if (g_cb.clearRecords) g_cb.clearRecords();
}
constexpr int SROW = 50;  // 7 rows of 50 from y 8 fill the 366 px page
int srowY(int row) { return 8 + row * SROW; }
lv_obj_t *settingRow(lv_obj_t *p, int col, int row, const char *name) {
  int x = col == 0 ? 20 : 414, y = srowY(row);
  blabel(p, &inter_rg_18, FG, x, y + 31, name);
  rule(p, x, y + 48, 366);
  return p;
}
lv_obj_t *rowValue(lv_obj_t *p, int col, int row, const char *text, uint32_t color = MUTED) {
  int y = srowY(row);
  return brlabel(p, &inter_rg_16, color, col == 0 ? 386 : 780, y + 31, 250, text);
}
void buildSettings(lv_obj_t *p) {
  static const char *NIGHT[3] = {"Auto", "On", "Off"};
  static const char *DEMO[2] = {"Off", "Replay"};
  static const char *UNITS[2] = {"km/h", "mph"};
  settingRow(p, 0, 0, "IMU calibration");
  G.calib = button(p, 206, srowY(0) + 8, 86, 34, "Mounting", &inter_rg_14, LIME, PANEL, calibCb, nullptr);
  G.calibL = buttonLabel(G.calib);
  G.accelCal = button(p, 300, srowY(0) + 8, 86, 34, "Offsets", &inter_rg_14, LIME, PANEL, accelCalCb, nullptr);
  G.accelCalL = buttonLabel(G.accelCal);
  settingRow(p, 0, 1, "Units");
  G.units = segmented(p, 246, srowY(1) + 8, 68, 34, UNITS, 2, unitsCb, &inter_rg_14);
  G.units.select(g_units_mph ? 1 : 0);
  settingRow(p, 0, 2, "Night mode");
  G.night = segmented(p, 196, srowY(2) + 8, 62, 34, NIGHT, 3, nightCb, &inter_rg_14);
  G.night.select(g_nightMode == -1 ? 0 : (g_nightMode == 1 ? 1 : 2));
  settingRow(p, 0, 3, "Brightness");
  G.bright = rowValue(p, 0, 3, "Auto · 100 %");
  static const char *OBD[2] = {"Off", "On"};
  settingRow(p, 0, 4, "OBD");
  G.forget = button(p, 132, srowY(4) + 8, 106, 34, "Pair dongle", &inter_rg_14, FG, PANEL, forgetCb, nullptr);
  G.forgetL = buttonLabel(G.forget);
  G.obd = segmented(p, 246, srowY(4) + 8, 68, 34, OBD, 2, obdCb, &inter_rg_14);
  G.obd.select(g_obd ? 1 : 0);
  settingRow(p, 0, 5, "Storage");
  G.dongle = rowValue(p, 0, 5, "");
  settingRow(p, 0, 6, "Start screen");
  rowValue(p, 0, 6, "Drive");

  settingRow(p, 1, 0, "Trip");
  button(p, 660, srowY(0) + 8, 120, 34, "Reset this trip", &inter_rg_14, FG, PANEL, resetTripCb, nullptr);
  settingRow(p, 1, 1, "All-time records");
  button(p, 700, srowY(1) + 8, 80, 34, "Clear", &inter_rg_14, RED, PANEL, clearRecordsCb, nullptr);
  settingRow(p, 1, 2, "Timezone");
  rowValue(p, 1, 2, "Australia/Melbourne");
  settingRow(p, 1, 3, "Diagnostics");
  G.diag = rowValue(p, 1, 3, "");
  settingRow(p, 1, 4, "Demo mode");
  G.demo = segmented(p, 630, srowY(4) + 8, 72, 34, DEMO, 2, demoCb, &inter_rg_14);
  G.demo.select(g_demo ? 1 : 0);
  settingRow(p, 1, 5, "WiFi update");
  rowValue(p, 1, 5, "Off · 10 min timeout");
  settingRow(p, 1, 6, "About");
  G.about = rowValue(p, 1, 6, "");
}
void updateSettings() {
  static uint8_t lastCal = 255;
  uint8_t cal = g_vs.diag.accCalState;
  if (cal == 1) setTextF(G.accelCalL, "Turn %d/%d", g_vs.diag.accCalDirs, 10);
  else if (cal != lastCal) setText(G.accelCalL, cal == 2 ? "Done" : (cal == 3 ? "Failed" : (g_vs.diag.accCalibrated ? "Done" : "Offsets")));
  lastCal = cal;
  static const char *const OBD_WORD[8] = {"off", "scan", "connect", "set-up", "", "asleep", "not paired", "pairing"};
  uint8_t os = g_vs.diag.obdState < 8 ? g_vs.diag.obdState : 0;
  setText(G.forgetL, g_vs.diag.obdState == 7 ? "Pairing..." : (g_vs.diag.obdBound ? "Forget dongle" : "Pair dongle"));
  if (os == 4) setTextF(G.diag, "GPS %d · IMU %.0f Hz · OBD %.1f Hz", g_vs.gps.sats, g_vs.imu.rateHz, g_vs.diag.obdHz);
  else setTextF(G.diag, "GPS %d · IMU %.0f Hz · OBD %s", g_vs.gps.sats, g_vs.imu.rateHz, OBD_WORD[os]);
  setTextF(G.bright, "%s · %u %%", g_nightMode == -1 ? "Auto" : "Manual", g_vs.disp.backlightPct);
  setTextF(G.about, "Mini Dash %s · touch %s", g_version, g_vs.diag.touchId);
  if (g_vs.store.fsOk) {
    setTextF(G.dongle, "%u trips · %lu of %lu KB", g_vs.store.storedTrips, (unsigned long)g_vs.store.usedKb,
             (unsigned long)g_vs.store.totalKb);
  } else {
    setText(G.dongle, "file system unavailable");
  }
}

// ---- shell ---------------------------------------------------------------------------------
void selectTab(Ui::Page p) {
  for (int i = 0; i < Ui::PAGE_COUNT; i++) {
    bool on = (int)p == i;
    lv_obj_set_style_bg_opa(H.tab[i], on ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
    show(H.tabInd[i], on);
    setColor(buttonLabel(H.tab[i]), on ? FG : MUTED);
  }
}
void tabCb(lv_event_t *e) { Ui::setPage((Ui::Page)(intptr_t)lv_event_get_user_data(e)); }
void headerCb(lv_event_t *) {
  if (g_page == Ui::PAGE_SETTINGS) Ui::setPage(g_pageBeforeSettings);
  else Ui::setPage(Ui::PAGE_SETTINGS);
}
void gestureCb(lv_event_t *e) {
  lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
  if (g_page == Ui::PAGE_SETTINGS) return;
  if (dir == LV_DIR_LEFT) Ui::setPage((Ui::Page)((g_page + 1) % Ui::PAGE_COUNT));
  else if (dir == LV_DIR_RIGHT) Ui::setPage((Ui::Page)((g_page + Ui::PAGE_COUNT - 1) % Ui::PAGE_COUNT));
  (void)e;
}

void buildShell() {
  H.scr = lv_obj_create(nullptr);
  lv_obj_remove_style_all(H.scr);
  lv_obj_set_style_bg_color(H.scr, c(BG), 0);
  lv_obj_set_style_bg_opa(H.scr, LV_OPA_COVER, 0);
  lv_obj_clear_flag(H.scr, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(H.scr, gestureCb, LV_EVENT_GESTURE, nullptr);

  H.header = box(H.scr, 0, 0, 800, HEADER_H);
  lv_obj_add_flag(H.header, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(H.header, headerCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t *brand = blabel(H.header, &inter_sb_16, FG, 20, 32, "MINI");
  lv_obj_set_style_text_letter_space(brand, 3, 0);
  lv_obj_t *model = blabel(H.header, &inter_sb_16, LIME, 82, 32, "COOPER SE");
  lv_obj_set_style_text_letter_space(model, 2, 0);
  rule(H.scr, 0, HEADER_H - 1, 800);
  lv_obj_t *gear = label(H.header, &lv_font_montserrat_16, MUTED, 760, 16, LV_SYMBOL_SETTINGS);
  (void)gear;
  H.clock = brlabel(H.header, &inter_rg_18, FG, 746, 32, 70, "--:--");
  H.night = brlabel(H.header, &inter_rg_16, MUTED, 672, 32, 50, "day");
  H.obd = pill(H.header, 0, 13, "OBD OFF", MUTED);
  H.obdL = buttonLabel(H.obd);
  lv_obj_align(H.obd, LV_ALIGN_TOP_RIGHT, -184, 13);
  H.gps = brlabel(H.header, &inter_rg_16, MUTED, 504, 32, 130, "GPS —");
  H.demo = pill(H.header, 0, 13, "DEMO", AMBER);
  lv_obj_align(H.demo, LV_ALIGN_TOP_RIGHT, -386, 13);
  H.back = brlabel(H.header, &inter_rg_16, MUTED, 326, 32, 80, "← back");
  show(H.back, false);

  H.content = box(H.scr, 0, HEADER_H, 800, CONTENT_H);
  for (int i = 0; i < Ui::PAGE_COUNT + 1; i++) {
    H.page[i] = box(H.content, 0, 0, 800, CONTENT_H);
    show(H.page[i], false);
  }
  buildDrive(H.page[Ui::PAGE_DRIVE]);
  buildTrip(H.page[Ui::PAGE_TRIP]);
  buildDynamics(H.page[Ui::PAGE_DYNAMICS]);
  buildEnergy(H.page[Ui::PAGE_ENERGY]);
  buildBattery(H.page[Ui::PAGE_BATTERY]);
  buildSettings(H.page[Ui::PAGE_COUNT]);

  H.tabbar = box(H.scr, 0, HEADER_H + CONTENT_H, 800, TABBAR_H);
  rule(H.tabbar, 0, 0, 800);
  for (int i = 0; i < Ui::PAGE_COUNT; i++) {
    lv_obj_t *t = box(H.tabbar, i * 160, 0, 160, TABBAR_H);
    lv_obj_set_style_bg_color(t, c(PANEL), 0);
    lv_obj_add_flag(t, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(t, tabCb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    lv_obj_t *l = label(t, &inter_rg_16, MUTED, 0, 0, TAB_NAMES[i]);
    lv_obj_set_style_text_letter_space(l, 2, 0);
    lv_obj_set_width(l, 160);
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_y(l, yForBaseline(&inter_rg_16, 38));
    H.tab[i] = t;
    H.tabInd[i] = panel(t, 0, 0, 160, 3, LIME, 0);
  }
  show(H.page[Ui::PAGE_DRIVE], true);
  selectTab(Ui::PAGE_DRIVE);
}

void updateHeader() {
  if (gpsFresh()) {
    setTextF(H.gps, "GPS %d", g_vs.gps.sats);
    setColor(H.gps, MUTED);
  } else if (g_vs.gps.fixOk && fresh(g_vs.gps.stampMs, 2000)) {
    setTextF(H.gps, "GPS %d", g_vs.gps.sats);  // a fix, but older than the 500 ms rule: amber
    setColor(H.gps, AMBER);
  } else {
    setText(H.gps, "GPS acquiring");
    setColor(H.gps, AMBER);
  }
  const char *obd = g_vs.diag.obdState == 0   ? "OBD OFF"
                    : g_vs.diag.obdState == 6 ? "NO DONGLE"
                    : g_vs.diag.obdState == 7 ? "PAIRING"
                    : !g_vs.car.linked        ? "OBD SCAN"
                    : g_vs.car.asleep         ? "CAR ASLEEP"
                                              : "OBD LINK";
  uint32_t oc = (!g_vs.car.linked || g_vs.car.asleep) ? MUTED : LIME;
  setText(H.obdL, obd);
  setColor(H.obdL, oc);
  setBorder(H.obd, oc);
  setText(H.night, g_vs.disp.night ? "night" : "day");
  show(H.demo, g_vs.demo);
  if (g_vs.utcNow) {
    char hm[8];
    Clock::formatHM(g_vs.utcNow, hm, sizeof(hm));
    setText(H.clock, hm);
    setColor(H.clock, FG);
  } else {
    setText(H.clock, "--:--");
    setColor(H.clock, MUTED);
  }
}

void tickCb(lv_timer_t *) {
  g_now = millis();
  uint32_t tStart = g_now;
  Vs::snapshot(g_vs);
  struct Done { uint32_t t; ~Done() { uint32_t d = millis() - t; if (d > g_tickMaxMs) g_tickMaxMs = d; } } done{tStart};
  if (!g_shellShown) {
    uint32_t up = g_now - g_bootMs;
    if (H.scr == nullptr && up >= 300) buildShell();  // behind the splash, so the hand-over is instant
    if ((g_vs.sensorsReady && up >= 1500) || up >= 3000) Ui::showShell();
    else {
      if (S.clock && g_vs.gps.timeValid) setTextF(S.clock, "%02u:%02u UTC", g_vs.gps.hour, g_vs.gps.minute);
      return;
    }
  }
  updateHeader();
  switch (g_page) {
    case Ui::PAGE_DRIVE: updateDrive(); break;
    case Ui::PAGE_TRIP: updateTrip(); break;
    case Ui::PAGE_DYNAMICS: updateDynamics(); break;
    case Ui::PAGE_ENERGY: updateEnergy(); break;
    case Ui::PAGE_BATTERY: updateBattery(); break;
    case Ui::PAGE_SETTINGS: updateSettings(); break;
    default: break;
  }
}
}  // namespace

namespace Ui {

void begin(const Callbacks &cb, const char *version) {
  g_cb = cb;
  g_version = version;
  g_bootMs = millis();
  buildSplash();
  g_tick = lv_timer_create(tickCb, 100, nullptr);
}

void applySettings(const UiSettings &st) {
  g_units_mph = st.mph;
  g_nightMode = st.nightMode;
  g_demo = st.demo;
  g_obd = st.obd;
  VsLock lk;
  if (lk.ok()) Vs::raw().disp.bestField = st.bestField % REC_COUNT;
}

void setBootStatus(const char *gps, const char *obd, const char *imu) {
  if (gps) setText(S.gps, gps);
  if (obd) setText(S.obd, obd);
  if (imu) setText(S.imu, imu);
}

void showShell() {
  if (g_shellShown) return;
  if (H.scr == nullptr) buildShell();
  lv_scr_load_anim(H.scr, LV_SCR_LOAD_ANIM_FADE_ON, 250, 0, true);
  g_shellShown = true;
  S.scr = nullptr;
  S.gps = S.obd = S.imu = S.clock = nullptr;
}

void setPage(Page p) {
  if (!g_shellShown) return;
  if (p == PAGE_SETTINGS) {
    if (g_page != PAGE_SETTINGS) g_pageBeforeSettings = g_page;
  }
  for (int i = 0; i < PAGE_COUNT + 1; i++) show(H.page[i], false);
  g_page = p;
  show(H.page[p == PAGE_SETTINGS ? PAGE_COUNT : p], true);
  show(H.back, p == PAGE_SETTINGS);
  if (p != PAGE_SETTINGS) selectTab(p);
  else for (int i = 0; i < PAGE_COUNT; i++) { lv_obj_set_style_bg_opa(H.tab[i], LV_OPA_TRANSP, 0); show(H.tabInd[i], false); }
  if (p == PAGE_TRIP) tripShowView(T.view == TV_EVENT ? TV_RECORDS : T.view);
}

Page page() { return g_page; }

void openEvent(int rec) {
  if (!g_shellShown) return;
  setPage(PAGE_TRIP);
  T.eventIdx = rec % REC_COUNT;
  tripShowView(TV_EVENT);
}

void timing(uint32_t *tickMaxMs, uint32_t *plotMaxMs) {
  *tickMaxMs = g_tickMaxMs;
  *plotMaxMs = g_plotMaxMs;
  g_tickMaxMs = g_plotMaxMs = 0;
}

}  // namespace Ui
