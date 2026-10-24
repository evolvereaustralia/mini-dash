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

## Phase 3b: the product firmware (`env:minidash`, the default)

`src/main.cpp` boots in this order so the screen is on within a second (splash at about 410 ms on the bench): serial, I2C, IO expander (retried, it is slow to wake), RGB panel, GT911, LVGL and the splash; then the sensor task on core 0 and the demo replay. LVGL runs on core 1 with a 100 ms UI tick.

Serial keys: `d` demo replay on/off, `1`..`9`/`0` backlight, `p` screenshot, `n` next tab, `D` `R` `Y` `E` `B` a page (Drive, Trip, Dynamics, Energy, Battery), `x` Settings, `a` log the next frame's invalidated areas, `T` bypass the touch read (diagnostic), `i` I2C scan, `r` reboot. A status line prints every 5 s with render and I2C statistics.

Screenshots from the Mac, over the same USB cable (needs pyserial and Pillow):

```bash
python3 firmware/tools/screenshot.py /dev/cu.usbmodem1101 shot.png --key Y
```

### Display pipeline and what it costs

LVGL 8.4 renders dirty areas in partial mode into a 40-row draw buffer in internal RAM; each flushed chunk is copied into the back framebuffer in PSRAM; on the last chunk the panel is told to switch to that buffer, we wait for the frame-complete event, then copy the same rows into the new back buffer so the two stay identical. No tearing, no full-screen redraw. LVGL's hot draw routines are placed in IRAM (`LV_ATTRIBUTE_FAST_MEM`). Measured 24 Oct 2026 on Drive with the demo running: about 55 ms of render per frame, one frame per 100 ms tick, longest I2C hold 24 ms. Two earlier approaches were measured and dropped: direct mode into the PSRAM framebuffers (about 150 ms of per-pixel blending per frame) and a canvas-only G plot redrawn every tick (whole-plot invalidation each frame). The remaining cost is dominated by anti-aliased text and lines; the 30 fps target in CLAUDE.md section 7 of the handoff is not met yet and the tick stays at 10 Hz until it is.

### Layout rules in the UI code

Text is placed by baseline, never by box top: `Theme::blabel` and `brlabel` take a baseline and derive the y from the font's `line_height` and `base_line`. This matters because lv_font_conv sizes a face's line box from the glyphs it contains, so the digit-only 88 and 60 px faces have 1 px of baseline and no leading while the text faces carry 4 to 9 px; placing them by box top put the big digits straight onto the captions above them (seen on the bench, 24 Oct 2026). Labels whose text changes are fixed-width and right-aligned where they end at a column edge, so updating them never moves them. Each page keeps a small baseline grid, noted at the top of its build function; columns start at x 20 and end at x 780, the Drive plot is centred on x 400, chart axis labels are centred on value lines the page draws itself.

### Layout

```
firmware/
  include/lv_conf.h         Waveshare's 4.3C config, adapted (see its header)
  include/vehicle_state.h   the shared state and stale thresholds
  src/app/
    lvgl_port.*             display pipeline above, touch input, tick, LVGL task and lock
    touch_gt911.*           GT911 on Wire through the I2C mutex
    vehicle_state.cpp       the state lock
    sensors.*               sensor task: GPS, IMU, lux, night mode, static mounting reference
    trip_engine.*           trip integration, timers, records (source agnostic)
    demo_source.*           scripted drive feeding the same state and engine
  src/ui/
    theme.*                 palette, type scale, widget helpers
    gplot.*                 the G plot (canvas background and cloud, object trail and dot)
    ui.*                    splash, header, tab bar, the five pages, Settings
    fonts/                  Inter converted with lv_font_conv (see below)
    img_mini_wings.c        the splash emblem (RGB565 plus alpha)
  src/bringup/              phase 3a bring-up program (env:bringup)
  tools/screenshot.py       screenshot over serial
```

### Fonts

Inter 4.1 (SIL OFL) with the tabular-figures feature frozen into the default digits with fontTools (Inter's default digits are proportional), then converted with lv_font_conv 1.5.3 at 4 bpp, uncompressed: Medium 88 and 60 (digits and signs only), Medium 40/34/30/28/26, Regular 18/16/14, SemiBold 26/16 (ASCII plus – — · ° ± − ← → ↻ ☼). About 600 KB of flash. Regenerate with the recipe in docs/design/README.md.

### Done and not done in 3b (24 Oct 2026)

Done: LVGL on the panel with touch, splash with the emblem, header status cluster, five tabs with swipe, Settings by header tap, Drive, Trip (records, journey totals, event page without traces), Dynamics, Energy, Battery, demo replay, the trip and records engine, GPS, IMU and lux into the state, night mode from lux, screenshot tool.

Not yet: records and trip history storage (Today and All time are since power-on only), event trace capture, the gyro-propagated attitude (the IMU path uses a static mounting reference and a 5 Hz low-pass), the GPS and accelerometer speed fusion (displayed speed is wheel speed when OBD is live, else raw GPS), timezone and DST (the clock adds 11 h), start-screen preference, WiFi OTA, units persistence, the 30 fps target, and the GPS poll's 80 to 108 ms bus holds seen after two minutes (docs/test-log.md).

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
