#include "theme.h"

namespace Theme {

static lv_style_t s_plain;

void init() {
  lv_style_init(&s_plain);
  lv_style_set_pad_all(&s_plain, 0);
  lv_style_set_border_width(&s_plain, 0);
  lv_style_set_radius(&s_plain, 0);
  lv_style_set_bg_opa(&s_plain, LV_OPA_TRANSP);
  lv_style_set_outline_width(&s_plain, 0);
  lv_style_set_shadow_width(&s_plain, 0);
  lv_disp_set_bg_color(nullptr, c(BG));
}

lv_obj_t *box(lv_obj_t *parent, int x, int y, int w, int h) {
  lv_obj_t *o = lv_obj_create(parent);
  lv_obj_remove_style_all(o);
  lv_obj_add_style(o, &s_plain, 0);
  lv_obj_set_pos(o, x, y);
  lv_obj_set_size(o, w, h);
  lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
  return o;
}

lv_obj_t *label(lv_obj_t *parent, const lv_font_t *font, uint32_t color, int x, int y, const char *text) {
  lv_obj_t *l = lv_label_create(parent);
  lv_obj_set_style_text_font(l, font, 0);
  lv_obj_set_style_text_color(l, c(color), 0);
  lv_label_set_long_mode(l, LV_LABEL_LONG_CLIP);
  lv_obj_set_pos(l, x, y);
  lv_label_set_text(l, text);
  return l;
}

lv_obj_t *rlabel(lv_obj_t *parent, const lv_font_t *font, uint32_t color, int rightX, int y, int width, const char *text) {
  lv_obj_t *l = label(parent, font, color, rightX - width, y, text);
  lv_obj_set_width(l, width);
  lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_RIGHT, 0);
  return l;
}

int yForBaseline(const lv_font_t *font, int baseline) { return baseline - (font->line_height - font->base_line); }

lv_obj_t *blabel(lv_obj_t *parent, const lv_font_t *font, uint32_t color, int x, int baseline, const char *text) {
  return label(parent, font, color, x, yForBaseline(font, baseline), text);
}

lv_obj_t *brlabel(lv_obj_t *parent, const lv_font_t *font, uint32_t color, int rightX, int baseline, int width, const char *text) {
  return rlabel(parent, font, color, rightX, yForBaseline(font, baseline), width, text);
}

lv_obj_t *caption(lv_obj_t *parent, int x, int y, const char *text, const lv_font_t *font) {
  lv_obj_t *l = label(parent, font, MUTED, x, y, text);
  lv_obj_set_style_text_letter_space(l, 1, 0);
  return l;
}

lv_obj_t *rule(lv_obj_t *parent, int x, int y, int w) {
  lv_obj_t *o = box(parent, x, y, w, 1);
  lv_obj_set_style_bg_color(o, c(LINE), 0);
  lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
  return o;
}

lv_obj_t *panel(lv_obj_t *parent, int x, int y, int w, int h, uint32_t fill, int radius) {
  lv_obj_t *o = box(parent, x, y, w, h);
  lv_obj_set_style_bg_color(o, c(fill), 0);
  lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(o, radius, 0);
  return o;
}

lv_obj_t *pill(lv_obj_t *parent, int x, int y, const char *text, uint32_t color) {
  lv_obj_t *o = box(parent, x, y, LV_SIZE_CONTENT, 24);
  lv_obj_set_style_border_width(o, 1, 0);
  lv_obj_set_style_border_color(o, c(color), 0);
  lv_obj_set_style_radius(o, 6, 0);
  lv_obj_set_style_pad_hor(o, 8, 0);
  lv_obj_set_style_pad_ver(o, 3, 0);
  lv_obj_t *l = label(o, &inter_rg_14, color, 0, 0, text);
  lv_obj_set_style_text_letter_space(l, 1, 0);
  lv_obj_center(l);
  return o;
}

}  // namespace Theme
