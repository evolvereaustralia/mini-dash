// Palette and type scale from CLAUDE.md section 4 (the design handoff's palette, sizes raised
// for the 0.117 mm pixel). Fonts are the converted Inter faces in src/ui/fonts.
#pragma once

#include "lvgl.h"

LV_FONT_DECLARE(inter_md_88);
LV_FONT_DECLARE(inter_md_60);
LV_FONT_DECLARE(inter_md_40);
LV_FONT_DECLARE(inter_md_34);
LV_FONT_DECLARE(inter_md_30);
LV_FONT_DECLARE(inter_md_28);
LV_FONT_DECLARE(inter_md_26);
LV_FONT_DECLARE(inter_rg_18);
LV_FONT_DECLARE(inter_rg_16);
LV_FONT_DECLARE(inter_rg_14);
LV_FONT_DECLARE(inter_sb_16);
LV_FONT_DECLARE(inter_sb_26);
LV_IMG_DECLARE(img_mini_wings);

namespace Theme {

constexpr uint32_t BG = 0x0B1114;
constexpr uint32_t PANEL = 0x121C21;
constexpr uint32_t PANEL2 = 0x182429;
constexpr uint32_t FG = 0xEDF4EE;
constexpr uint32_t MUTED = 0x99ACB4;
constexpr uint32_t LINE = 0x2B3A42;
constexpr uint32_t LIME = 0xC7F46B;
constexpr uint32_t CYAN = 0x64D9E0;
constexpr uint32_t AMBER = 0xF2B84B;
constexpr uint32_t RED = 0xF0645A;
constexpr uint32_t CLOUD = 0x5F7F8C;

constexpr int HEADER_H = 50;
constexpr int CONTENT_H = 366;
constexpr int TABBAR_H = 64;
constexpr int PAD_X = 20;

inline lv_color_t c(uint32_t hex) { return lv_color_hex(hex); }

// Apply the base look once after lv_init: no default theme decoration anywhere.
void init();

// A plain container with no padding, border or background.
lv_obj_t *box(lv_obj_t *parent, int x, int y, int w, int h);
// A label with font and colour, text left-aligned, no wrap.
lv_obj_t *label(lv_obj_t *parent, const lv_font_t *font, uint32_t color, int x, int y, const char *text);
// Fixed-width label whose text is right-aligned, ending at rightX. Updating its text never
// moves it, so a tick invalidates only when the text changes.
lv_obj_t *rlabel(lv_obj_t *parent, const lv_font_t *font, uint32_t color, int rightX, int y, int width, const char *text);
// Uppercase, letter-spaced label in the muted colour (the design's "label" class).
lv_obj_t *caption(lv_obj_t *parent, int x, int y, const char *text, const lv_font_t *font = &inter_rg_16);
// A 1 px horizontal rule in the line colour.
lv_obj_t *rule(lv_obj_t *parent, int x, int y, int w);
// A rounded filled panel (record tiles, pills).
lv_obj_t *panel(lv_obj_t *parent, int x, int y, int w, int h, uint32_t fill = PANEL, int radius = 10);
// Pill with a 1 px border and small uppercase text.
lv_obj_t *pill(lv_obj_t *parent, int x, int y, const char *text, uint32_t color);

}  // namespace Theme
