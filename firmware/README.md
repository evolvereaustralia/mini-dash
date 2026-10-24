# Firmware

PlatformIO project, Arduino framework, toolchain pinned to the Waveshare 4.3C repo's `config/ci.json` (Arduino core 3.3.11). `platformio.ini` maps every FQBN option to a board override; do not change them without reading CLAUDE.md section 2.

## Toolchain (set up 24 Oct 2026)

- Platform: pioarduino `platform-espressif32` release 55.03.311 (Arduino core 3.3.11 on ESP-IDF 5.5.5). The official PlatformIO espressif32 platform stopped at Arduino core 2.x.
- PlatformIO lives in `~/.platformio/penv`, a virtualenv on Homebrew `python@3.12`, with `pio` symlinked into `/opt/homebrew/bin`. The pioarduino build scripts refuse Python outside 3.10 to 3.14 (Homebrew's own `platformio` formula runs on 3.15 and fails with "Python version must be 3.10, 3.11, 3.12, 3.13, 3.14").
- Waveshare's own `examples/arduino/03_lcd` was built first with the same `platformio.ini` settings to prove the flash and PSRAM options (1.19 MB flash, success, 24 Oct 2026).

## Build, flash, watch

```bash
cd firmware && pio run
```

```bash
cd firmware && pio run -t upload && pio device monitor
```

The 4.3C's single USB-C port is native USB with auto-reset. If upload fails: hold BOOT, press and release RESET, release BOOT, upload, press RESET.

## Phase 3a: bring-up (`env:bringup`)

`src/main.cpp` proves, in order: USB CDC serial, flash and PSRAM, the I2C map against `include/board_config.h`, the IO expander (backlight ramp 100 to 0 to 100), the RGB panel (colour bars, white, yellow, cyan, green, magenta, red, blue, black left to right), the NEO-M9N at 10 Hz PVT, the ISM330DHCX at 208 Hz, the VEML7700 at 1 Hz. Then one line per second:

```
[   12 s] GPS fix 3 sats 9 spd 0.02 m/s (+-0.15) hAcc 2.1 m alt 48.3 m pvt 118 2026-10-24 05:12:33Z | IMU a +0.012 -0.004 +0.998 g  w +0.15 -0.08 +0.02 dps  208/s | lux 312.0 | BL 100% | i2c hold max 4 ms over 0 | heap ... psram ...
```

Serial keys: `s` rescan I2C, `1`..`9` backlight 10 to 90 percent, `0` 100 percent, `c` colour bars, `w` white, `k` black, `r` reboot.

What to record from the first run (CLAUDE.md sections 2 and 9): the real I2C map into `board_config.h` (the scan prints UNEXPECTED or MISSING against the expected list and says to stop and ask), the IMU sample rate (expect about 208/s), satellites and speed accuracy on the bench, lux indoors, and that `i2c hold max` stays under 25 ms after start-up.

## Layout

```
firmware/
  platformio.ini        pinned toolchain, board overrides, library versions
  include/
    board_config.h      pins, I2C map, expander registers, panel timing (the only place)
    obd_pids.h          OBD ECU addresses and DIDs with citations (empty until phase 3c)
  src/
    main.cpp            phase 3a bring-up
    i2c_bus.*           the one I2C mutex, RAII lock, hold-time diagnostics, scan
    io_expander.*       Waveshare board controller at 0x24 ported onto Wire
    display_rgb.*       RGB panel init as Waveshare's rgb_lcd_port, framebuffers in PSRAM
  lib/ test/            empty until phase 3b
```

## Modules

- `i2c_bus`: `I2cBus::begin()` once, then every I2C access inside an `I2cLock`. The destructor logs any hold over 25 ms and counts them; `I2cBus::longestHoldMs()` and `overHoldCount()` go in the status line.
- `io_expander`: registers mode 0x02, output 0x03, input 0x04, PWM 0x05, ADC 0x06, RTC INT 0x07. The PWM register is inverted (0 = brightest). `setBacklightPercent(100)` is full brightness; 0 is as dim as Waveshare's driver allows (it never fully blanks the panel). The audio amplifier (IO3) is switched off at init.
- `display_rgb`: `begin()`, `fill()`, `colourBars()`, `getFrameBuffers()` for LVGL in 3b.

## Libraries (pinned in `platformio.ini`)

| Library | Version | Used for |
|---|---|---|
| SparkFun u-blox GNSS v3 | 3.1.15 | NEO-M9N over I2C, UBX, autoPVT |
| SparkFun 6DoF ISM330DHCX | 1.0.6 | IMU |
| Adafruit VEML7700 Library | 2.1.6 | light sensor (its SSD1306 and GFX dependencies are ignored) |
| Adafruit BusIO | 1.17.4 | VEML7700 dependency |

Phase 3b adds `lvgl/lvgl @ 8.4.0` with Waveshare's `lv_conf.h`; phase 3c adds `h2zero/NimBLE-Arduino` (2.5.1 current).
