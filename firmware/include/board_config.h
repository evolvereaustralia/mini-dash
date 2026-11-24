// Board facts for the Waveshare ESP32-S3-Touch-LCD-4.3C (standard, SKU 33799) and the
// Mini Dash sensors. Pin names, I2C addresses and board facts live here and nowhere else
// (CLAUDE.md section 8). Sources: Waveshare 4.3C repo HARDWARE_REFERENCE.md and the
// examples' rgb_lcd_port.h / io_extension.h / i2c.h (V1.1 schematic), CLAUDE.md section 2.
#pragma once

#include <stdint.h>

// ---------------------------------------------------------------------------------------
// I2C: the one bus. GPIO8/9 at 400 kHz, shared by touch, IO expander, RTC, both audio
// codecs and all three Mini Dash sensors. Every access goes through I2cBus (i2c_bus.h).
// ---------------------------------------------------------------------------------------
constexpr int PIN_I2C_SDA = 8;
constexpr int PIN_I2C_SCL = 9;
constexpr uint32_t I2C_FREQ_HZ = 400000;
constexpr int PIN_TOUCH_INT = 4;  // GT911 interrupt (touch reset is on the IO expander)

// Expected I2C map (7-bit). Verified by the bring-up scan; result recorded below.
constexpr uint8_t I2C_ADDR_VEML7700 = 0x10;   // Adafruit ADA5378 light sensor
constexpr uint8_t I2C_ADDR_ES8311 = 0x18;     // speaker codec, unused (Waveshare es8311_codec.h: 0x30 8-bit)
constexpr uint8_t I2C_ADDR_IO_EXPANDER = 0x24; // CH32V003 board controller (Waveshare "CH422G" in older docs)
constexpr uint8_t I2C_ADDR_ES7210 = 0x40;     // microphone ADC, unused (Waveshare es7210_adc.h: 0x80 8-bit)
constexpr uint8_t I2C_ADDR_NEO_M9N = 0x42;    // SparkFun GPS-17285
constexpr uint8_t I2C_ADDR_PCF85063A = 0x51;  // RTC with CR927 cell
constexpr uint8_t I2C_ADDR_GT911 = 0x5D;      // touch (0x14 is the alternate strap address)
constexpr uint8_t I2C_ADDR_ISM330DHCX = 0x6B; // SparkFun SEN-19764

struct I2cExpected {
  uint8_t addr;
  const char *name;
  bool onBoard;  // true: part of the display board; false: Mini Dash sensor on the terminal
};
constexpr I2cExpected I2C_EXPECTED[] = {
    {I2C_ADDR_VEML7700, "VEML7700 light", false},
    {I2C_ADDR_ES8311, "ES8311 codec", true},
    {I2C_ADDR_IO_EXPANDER, "IO expander", true},
    {I2C_ADDR_ES7210, "ES7210 mic ADC", true},
    {I2C_ADDR_NEO_M9N, "NEO-M9N GPS", false},
    {I2C_ADDR_PCF85063A, "PCF85063A RTC", true},
    {I2C_ADDR_GT911, "GT911 touch", true},
    {I2C_ADDR_ISM330DHCX, "ISM330DHCX IMU", false},
};
// Bring-up scan result (CLAUDE.md section 2), 24 Oct 2026, all three sensors on the terminal:
// 0x10, 0x18, 0x24, 0x40, 0x42, 0x51, 0x5D, 0x6B found, nothing unexpected, nothing missing.
// The map above is the real map.

// ---------------------------------------------------------------------------------------
// IO expander at 0x24: Waveshare's own register protocol (ported from the 4.3C repo,
// examples/arduino/*/src/io_extension). Do not use ESP32_IO_Expander's CH422G class.
// ---------------------------------------------------------------------------------------
constexpr uint8_t IOX_REG_MODE = 0x02;      // bit per pin, 1 = output
constexpr uint8_t IOX_REG_IO_OUTPUT = 0x03; // bit per pin
constexpr uint8_t IOX_REG_IO_INPUT = 0x04;
constexpr uint8_t IOX_REG_PWM = 0x05;       // backlight duty, INVERTED: 0 = brightest, 255 = off
constexpr uint8_t IOX_REG_ADC = 0x06;
constexpr uint8_t IOX_REG_RTC_INT = 0x07;

constexpr uint8_t IOX_PIN_DI0 = 0;        // isolated digital input 0
constexpr uint8_t IOX_PIN_TOUCH_RST = 1;  // GT911 reset
constexpr uint8_t IOX_PIN_BACKLIGHT = 2;  // backlight enable (1 = on); brightness is the PWM register
constexpr uint8_t IOX_PIN_AUDIO_PA = 3;   // speaker amplifier enable (keep low)
constexpr uint8_t IOX_PIN_SD_CS = 4;      // TF card chip select
constexpr uint8_t IOX_PIN_USB_CAN_SEL = 5; // 0 = USB, 1 = CAN (no CAN on this variant); also DI1
constexpr uint8_t IOX_PIN_DO0 = 6;        // isolated digital output 0
constexpr uint8_t IOX_PIN_DO1 = 7;        // isolated digital output 1

// Waveshare's driver never lets the PWM register reach fully off (it caps the "off" side at
// 95 to 97 percent). Keep the same floor so the screen never goes black by accident.
constexpr uint8_t IOX_BACKLIGHT_MAX_OFF_PERCENT = 95;

// ---------------------------------------------------------------------------------------
// 4.3" 800x480 RGB565 panel (ST7262) on the S3's LCD peripheral. Timing from Waveshare's
// rgb_lcd_port.h: 16 MHz pixel clock, two framebuffers in PSRAM, 10-line bounce buffer.
// ---------------------------------------------------------------------------------------
constexpr int LCD_H_RES = 800;
constexpr int LCD_V_RES = 480;
constexpr uint32_t LCD_PIXEL_CLOCK_HZ = 16 * 1000 * 1000;
constexpr int LCD_HSYNC_PULSE = 4, LCD_HSYNC_BACK = 8, LCD_HSYNC_FRONT = 8;
constexpr int LCD_VSYNC_PULSE = 4, LCD_VSYNC_BACK = 8, LCD_VSYNC_FRONT = 8;
constexpr int LCD_NUM_FBS = 2;
constexpr int LCD_BOUNCE_BUFFER_PX = LCD_H_RES * 10;

constexpr int PIN_LCD_VSYNC = 3;
constexpr int PIN_LCD_HSYNC = 46;
constexpr int PIN_LCD_DE = 5;
constexpr int PIN_LCD_PCLK = 7;
constexpr int PIN_LCD_DISP = -1;  // not wired
constexpr int PIN_LCD_RST = -1;   // not wired
// Data bits 0..15 = B3..B7, G2..G7, R3..R7
constexpr int PIN_LCD_DATA[16] = {
    14, 38, 18, 17, 10,       // B3..B7
    39, 0, 45, 48, 47, 21,    // G2..G7
    1, 2, 42, 41, 40,         // R3..R7
};

// ---------------------------------------------------------------------------------------
// Other on-board peripherals we do not drive but must not collide with.
// ---------------------------------------------------------------------------------------
constexpr int PIN_SD_CLK = 12, PIN_SD_CMD = 11, PIN_SD_D0 = 13;  // one-bit SDMMC, unused
constexpr int PIN_I2S_MCLK = 6, PIN_I2S_BCLK = 44, PIN_I2S_WS = 16;  // audio, unused
constexpr int PIN_I2S_DOUT = 15, PIN_I2S_DIN = 43;                   // (GPIO15/16/43/44 are I2S, not CAN)

// ---------------------------------------------------------------------------------------
// Sensor configuration (CLAUDE.md section 5).
// ---------------------------------------------------------------------------------------
constexpr uint8_t GPS_NAV_RATE_HZ = 10;
constexpr uint16_t IMU_ODR_HZ = 208;
// Nominal forward axis of the car in the IMU's frame once the board is screwed into the shell
// (CLAUDE.md section 6: board X along the connector edge, Y along the header edge; the shell
// mounts the board with Y pointing forward). Pitch and roll of the mount are handled by the
// attitude filter; only this axis and its sign matter, and the sign is confirmed on the first
// drive (acceleration must read positive).
constexpr float IMU_FORWARD_SENSOR[3] = {0.0f, 1.0f, 0.0f};
constexpr uint16_t LUX_PERIOD_MS = 1000;
