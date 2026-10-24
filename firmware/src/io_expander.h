// Waveshare 4.3C board controller at I2C 0x24 (CH32V003 running Waveshare's register
// protocol). Ported from the 4.3C repo, examples/arduino/*/src/io_extension, onto Wire and
// the I2cBus mutex. Registers and pin roles are in board_config.h.
#pragma once

#include <stdint.h>

namespace IoExpander {

// Set all pins to output and drive them high (Waveshare's power-on state). Returns false
// if the expander did not acknowledge.
bool begin();

// Drive one output pin. Keeps a shadow of the output register like Waveshare's driver.
bool setOutput(uint8_t pin, bool high);

// Read one input pin.
bool readInput(uint8_t pin, bool *high);

// Backlight. The PWM register is inverted (0 = brightest). brightPercent is what a human
// means: 100 = full, 0 = as dim as Waveshare's driver allows (never fully off).
bool setBacklightPercent(uint8_t brightPercent);
bool setBacklightEnable(bool on);

// Raw register access for bring-up diagnostics.
bool writeReg(uint8_t reg, uint8_t value);
bool readReg(uint8_t reg, uint8_t *value, uint8_t len);

}  // namespace IoExpander
