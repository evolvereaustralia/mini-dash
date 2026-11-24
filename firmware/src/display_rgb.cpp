#include "display_rgb.h"

#include <Arduino.h>

#include "board_config.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_rgb.h"

namespace {
esp_lcd_panel_handle_t g_panel = nullptr;
}

namespace DisplayRgb {

esp_lcd_panel_handle_t begin() {
  esp_lcd_rgb_panel_config_t cfg = {};
  cfg.clk_src = LCD_CLK_SRC_DEFAULT;
  cfg.timings.pclk_hz = LCD_PIXEL_CLOCK_HZ;
  cfg.timings.h_res = LCD_H_RES;
  cfg.timings.v_res = LCD_V_RES;
  cfg.timings.hsync_pulse_width = LCD_HSYNC_PULSE;
  cfg.timings.hsync_back_porch = LCD_HSYNC_BACK;
  cfg.timings.hsync_front_porch = LCD_HSYNC_FRONT;
  cfg.timings.vsync_pulse_width = LCD_VSYNC_PULSE;
  cfg.timings.vsync_back_porch = LCD_VSYNC_BACK;
  cfg.timings.vsync_front_porch = LCD_VSYNC_FRONT;
  cfg.timings.flags.pclk_active_neg = 1;
  cfg.data_width = 16;
  cfg.bits_per_pixel = 16;
  cfg.num_fbs = LCD_NUM_FBS;
  cfg.bounce_buffer_size_px = LCD_BOUNCE_BUFFER_PX;
  cfg.dma_burst_size = 64;  // IDF 5.5 name for what Waveshare set as psram_trans_align = 64
  cfg.hsync_gpio_num = PIN_LCD_HSYNC;
  cfg.vsync_gpio_num = PIN_LCD_VSYNC;
  cfg.de_gpio_num = PIN_LCD_DE;
  cfg.pclk_gpio_num = PIN_LCD_PCLK;
  cfg.disp_gpio_num = PIN_LCD_DISP;
  for (int i = 0; i < 16; i++) cfg.data_gpio_nums[i] = PIN_LCD_DATA[i];
  cfg.flags.fb_in_psram = 1;

  esp_err_t err = esp_lcd_new_rgb_panel(&cfg, &g_panel);
  if (err != ESP_OK) {
    log_e("esp_lcd_new_rgb_panel failed: %s", esp_err_to_name(err));
    g_panel = nullptr;
    return nullptr;
  }
  err = esp_lcd_panel_init(g_panel);
  if (err != ESP_OK) {
    log_e("esp_lcd_panel_init failed: %s", esp_err_to_name(err));
    return nullptr;
  }
  return g_panel;
}

bool getFrameBuffers(void **fb0, void **fb1) {
  if (g_panel == nullptr) return false;
  return esp_lcd_rgb_panel_get_frame_buffer(g_panel, 2, fb0, fb1) == ESP_OK;
}

void fill(uint16_t rgb565) {
  void *fb0 = nullptr, *fb1 = nullptr;
  if (!getFrameBuffers(&fb0, &fb1)) return;
  uint16_t *px = static_cast<uint16_t *>(fb0);
  for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) px[i] = rgb565;
  esp_lcd_panel_draw_bitmap(g_panel, 0, 0, LCD_H_RES, LCD_V_RES, fb0);
}

void colourBars() {
  void *fb0 = nullptr, *fb1 = nullptr;
  if (!getFrameBuffers(&fb0, &fb1)) return;
  // White, yellow, cyan, green, magenta, red, blue, black: the SMPTE order, left to right.
  static const uint16_t bars[8] = {0xFFFF, 0xFFE0, 0x07FF, 0x07E0, 0xF81F, 0xF800, 0x001F, 0x0000};
  uint16_t *px = static_cast<uint16_t *>(fb0);
  const int barW = LCD_H_RES / 8;
  for (int y = 0; y < LCD_V_RES; y++) {
    for (int x = 0; x < LCD_H_RES; x++) {
      px[y * LCD_H_RES + x] = bars[(x / barW) & 7];
    }
  }
  esp_lcd_panel_draw_bitmap(g_panel, 0, 0, LCD_H_RES, LCD_V_RES, fb0);
}

}  // namespace DisplayRgb
