// GT911 capacitive touch at I2C 0x5D on Wire, through the I2cBus mutex (CLAUDE.md section 2:
// LVGL's touch read goes through the same mutex as every other I2C access). Reset is driven
// through the IO expander (IO1) with INT (GPIO4) held low so the controller takes address 0x5D,
// the same sequence as Waveshare's touch_gt911_init.
#pragma once

#include <stdint.h>

namespace TouchGt911 {

struct Point {
  uint16_t x, y, strength;
};

// Reset the controller and read its product id. Returns false if it does not answer.
bool begin();

// Read the current touch state. Returns the number of points (0 when nothing is touched),
// filling up to maxPoints entries. Returns -1 on a bus error or if the bus was busy.
int read(Point *points, uint8_t maxPoints);

// Product id string read at begin(), for the diagnostics page ("911" expected).
const char *productId();

}  // namespace TouchGt911
