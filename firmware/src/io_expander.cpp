#include "io_expander.h"

#include <Wire.h>

#include "board_config.h"
#include "i2c_bus.h"

namespace {
uint8_t g_outputShadow = 0xFF;
}

namespace IoExpander {

bool writeReg(uint8_t reg, uint8_t value) {
  I2cLock guard;
  if (!guard.ok()) return false;
  Wire.beginTransmission(I2C_ADDR_IO_EXPANDER);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readReg(uint8_t reg, uint8_t *value, uint8_t len) {
  I2cLock guard;
  if (!guard.ok()) return false;
  Wire.beginTransmission(I2C_ADDR_IO_EXPANDER);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t got = Wire.requestFrom(I2C_ADDR_IO_EXPANDER, len);
  if (got != len) return false;
  for (uint8_t i = 0; i < len; i++) value[i] = Wire.read();
  return true;
}

bool begin() {
  // Waveshare: mode 0xFF (all outputs), then outputs all high. That leaves the backlight
  // enabled, touch out of reset, audio PA on (we turn that off), SD CS high, USB selected.
  if (!writeReg(IOX_REG_MODE, 0xFF)) return false;
  g_outputShadow = 0xFF;
  if (!writeReg(IOX_REG_IO_OUTPUT, g_outputShadow)) return false;
  // Audio amplifier off: nothing is connected to the speaker header.
  setOutput(IOX_PIN_AUDIO_PA, false);
  return true;
}

bool setOutput(uint8_t pin, bool high) {
  if (pin > 7) return false;
  if (high) {
    g_outputShadow |= (1u << pin);
  } else {
    g_outputShadow &= ~(1u << pin);
  }
  return writeReg(IOX_REG_IO_OUTPUT, g_outputShadow);
}

bool readInput(uint8_t pin, bool *high) {
  if (pin > 7 || high == nullptr) return false;
  uint8_t v = 0;
  if (!readReg(IOX_REG_IO_INPUT, &v, 1)) return false;
  *high = (v >> pin) & 1u;
  return true;
}

bool setBacklightPercent(uint8_t brightPercent) {
  if (brightPercent > 100) brightPercent = 100;
  // Waveshare's driver takes an "off percentage" 0..100 and caps it at 95 so the panel never
  // fully blanks, then scales to the 8-bit register.
  uint8_t offPercent = 100 - brightPercent;
  if (offPercent > IOX_BACKLIGHT_MAX_OFF_PERCENT) offPercent = IOX_BACKLIGHT_MAX_OFF_PERCENT;
  uint8_t reg = static_cast<uint8_t>(offPercent * (255 / 100.0f));
  return writeReg(IOX_REG_PWM, reg);
}

bool setBacklightEnable(bool on) { return setOutput(IOX_PIN_BACKLIGHT, on); }

}  // namespace IoExpander
