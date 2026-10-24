#include "touch_gt911.h"

#include <Arduino.h>
#include <Wire.h>

#include "board_config.h"
#include "i2c_bus.h"
#include "io_expander.h"

namespace {
constexpr uint16_t REG_PRODUCT_ID = 0x8140;
constexpr uint16_t REG_STATUS = 0x814E;  // bit 7 = buffer ready, bits 0..3 = point count
constexpr uint16_t REG_POINTS = 0x8150;  // 8 bytes per point: id, x lo, x hi, y lo, y hi, size lo, size hi, pad
char g_productId[5] = "----";

bool writeReg(uint16_t reg, const uint8_t *data, uint8_t len) {
  Wire.beginTransmission(I2C_ADDR_GT911);
  Wire.write(reg >> 8);
  Wire.write(reg & 0xFF);
  for (uint8_t i = 0; i < len; i++) Wire.write(data[i]);
  return Wire.endTransmission() == 0;
}

bool readReg(uint16_t reg, uint8_t *data, uint8_t len) {
  Wire.beginTransmission(I2C_ADDR_GT911);
  Wire.write(reg >> 8);
  Wire.write(reg & 0xFF);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(I2C_ADDR_GT911, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) data[i] = Wire.read();
  return true;
}
}  // namespace

namespace TouchGt911 {

bool begin() {
  // Reset with INT low: expander IO1 low, INT low, IO1 high, then release INT as an input.
  pinMode(PIN_TOUCH_INT, OUTPUT);
  digitalWrite(PIN_TOUCH_INT, LOW);
  IoExpander::setOutput(IOX_PIN_TOUCH_RST, false);
  delay(20);
  IoExpander::setOutput(IOX_PIN_TOUCH_RST, true);
  delay(60);
  pinMode(PIN_TOUCH_INT, INPUT);
  delay(100);

  I2cLock guard(50, "touch.begin");
  if (!guard.ok()) return false;
  uint8_t id[4] = {0};
  if (!readReg(REG_PRODUCT_ID, id, 4)) return false;
  for (int i = 0; i < 4; i++) g_productId[i] = (id[i] >= 0x20 && id[i] < 0x7F) ? id[i] : 0;
  g_productId[4] = 0;
  uint8_t zero = 0;
  writeReg(REG_STATUS, &zero, 1);
  return true;
}

const char *productId() { return g_productId; }

int read(Point *points, uint8_t maxPoints) {
  I2cLock guard(5, "touch");  // LVGL polls every 30 ms; if the bus is busy, report "unchanged" quickly
  if (!guard.ok()) return -1;
  uint8_t status = 0;
  if (!readReg(REG_STATUS, &status, 1)) return -1;
  if ((status & 0x80) == 0) return 0;  // nothing new; do not clear, the controller is still writing
  uint8_t count = status & 0x0F;
  uint8_t zero = 0;
  if (count == 0 || count > 5) {
    writeReg(REG_STATUS, &zero, 1);
    return 0;
  }
  if (count > maxPoints) count = maxPoints;
  uint8_t buf[5 * 8];
  if (!readReg(REG_POINTS, buf, count * 8)) {
    writeReg(REG_STATUS, &zero, 1);
    return -1;
  }
  writeReg(REG_STATUS, &zero, 1);
  for (uint8_t i = 0; i < count; i++) {
    const uint8_t *p = buf + i * 8;
    points[i].x = p[1] | (p[2] << 8);
    points[i].y = p[3] | (p[4] << 8);
    points[i].strength = p[5] | (p[6] << 8);
    if (points[i].x >= LCD_H_RES) points[i].x = LCD_H_RES - 1;
    if (points[i].y >= LCD_V_RES) points[i].y = LCD_V_RES - 1;
  }
  return count;
}

}  // namespace TouchGt911
