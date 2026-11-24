// RGB panel bring-up: the ESP-IDF RGB LCD driver configured exactly as Waveshare's
// rgb_lcd_port.cpp, with pins and timing from board_config.h. Framebuffers live in PSRAM,
// which is why this is part of phase 3a: a working panel proves the flash and PSRAM
// settings. LVGL takes over the framebuffers in phase 3b.
#pragma once

#include <stdint.h>

#include "esp_lcd_panel_ops.h"

namespace DisplayRgb {

// Install the panel driver. Returns the panel handle, or nullptr on failure.
esp_lcd_panel_handle_t begin();

// Fill the whole screen with one RGB565 colour.
void fill(uint16_t rgb565);

// Eight vertical colour bars, for a visible backlight ramp and a colour-order check.
void colourBars();

// Both PSRAM framebuffers, for LVGL later.
bool getFrameBuffers(void **fb0, void **fb1);

constexpr uint16_t RGB565_BLACK = 0x0000;
constexpr uint16_t RGB565_WHITE = 0xFFFF;
constexpr uint16_t RGB565_RED = 0xF800;
constexpr uint16_t RGB565_GREEN = 0x07E0;
constexpr uint16_t RGB565_BLUE = 0x001F;

}  // namespace DisplayRgb
