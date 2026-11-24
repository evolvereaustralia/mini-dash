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

## Framework rebuild (`env:minidash-xip`, the default since 24 Oct 2026)

The product firmware is built on an Arduino core rebuilt from ESP-IDF 5.5.5 with four options the prebuilt core lacks: `CONFIG_SPIRAM_XIP_FROM_PSRAM`, `CONFIG_SPIRAM_FETCH_INSTRUCTIONS`, `CONFIG_SPIRAM_RODATA` (code and read-only data are copied into PSRAM at boot and executed from there, so a flash write no longer takes the PSRAM cache down with the flash cache) and `CONFIG_LCD_RGB_ISR_IRAM_SAFE` (the panel's bounce-buffer refill interrupt keeps running during a flash write). Without them every flash write froze the panel refill for the duration of a block erase and the glass repeated stale ten-line strips (banded digits, ghost text; docs/test-log.md, 24 Oct 2026). Measured with the bench proxy, serial key `Z` (a 1 ms task touching the PSRAM frame buffer records its longest stall while five event files are written): prebuilt core 19 to 20 ms, rebuilt core 1.5 to 1.8 ms, the same as idle. The rebuilt core costs about 1 MB of PSRAM for the copied code (free PSRAM 5.0 MB instead of 6.0 MB), renders faster (the UI handler averages 26 to 30 ms instead of 39 to 80 ms, flash cache misses being slower than PSRAM) and boots in the same time. On this env flash writes are immediate: event traces on acceptance, the checkpoint every 30 s. `env:minidash` is the same firmware on the prebuilt core, kept as a fallback that needs no rebuild; on it writes are deferred to standstills and traces held back (`MINIDASH_XIP` is the switch, in `src/main.cpp`).

pioarduino does the rebuild itself ("hybrid build", `custom_sdkconfig` in `platformio.ini`): the first build downloads ESP-IDF and compiles the libraries, about three minutes and a few hundred megabytes; later builds take 20 s. The rebuilt libraries replace the prebuilt `framework-arduinoespressif32-libs` package in place, so every switch between `minidash-xip` and `minidash` (or `bringup`) swaps the package and triggers another three-minute rebuild. Stay on one env. The hybrid build ignores `build_src_filter` (the bring-up program guards itself with `MINIDASH_BRINGUP`), needs the partition CSV inside the project folder (`app3M_fat9M_16MB.csv`, copied from the core) and leaves `.dummy/`, `managed_components/`, `CMakeLists.txt`, `sdkconfig*` and `dependencies.lock` in the project, all ignored by git.

Per-machine fix, needed once per PlatformIO installation (24 Oct 2026, platform 55.03.311, PlatformIO Core 6.2.0): the hybrid build runs the platform's own tool version check on `tool-scons`. The installed package is 4.41101.0 (SCons 4.11.1, what Core 6.2 runs on) but the platform's `platform.json` lists 4.40801.0, so the check deletes the package directory to reinstall it, neither reinstall path applies, and the link step dies with `ModuleNotFoundError: No module named 'SCons.Tool.FortranCommon'` although nothing is wrong with the project. Point the platform at the version it actually runs:

```bash
python3 - <<'EOF'
import json, pathlib, shutil
p = pathlib.Path.home() / ".platformio/platforms/espressif32/platform.json"
shutil.copy2(p, p.with_suffix(".json.orig"))
d = json.loads(p.read_text())
d["packages"]["tool-scons"]["package-version"] = "4.41101.0"
d["packages"]["tool-scons"]["version"] = "https://github.com/pioarduino/scons/releases/download/4.11.1/scons-local-4.11.1.tar.gz"
p.write_text(json.dumps(d, indent=2) + "\n")
EOF
```

A platform upgrade overwrites the file; if the error above comes back, run it again. Worth reporting upstream to pioarduino (platform.py, `install_tool` on `tool-scons` for `custom_sdkconfig` builds) when there is time.

## Phase 3b: the product firmware (`env:minidash`, prebuilt core)

`src/main.cpp` boots in this order so the screen is on within a second (splash at about 410 ms on the bench): serial, I2C, IO expander (retried, it is slow to wake), RGB panel, GT911, LVGL and the splash; then the sensor task on core 0 and the demo replay. LVGL runs on core 1 with a 100 ms UI tick.

Serial keys: `d` demo replay on/off, `1`..`9`/`0` backlight, `p` screenshot, `n` next tab, `D` `R` `Y` `E` `B` a page (Drive, Trip, Dynamics, Energy, Battery), `x` Settings, `a` log the next frame's invalidated areas, `T` bypass the touch read (diagnostic), `i` I2C scan, `r` reboot, `Z` flash-write stall test (see "Framework rebuild"), `Q` dump the stored trace of this trip's 0-60 record as CSV, `c` start the accelerometer offset calibration, `g` 10 Hz motion log (lat, lon, yaw, tilt, correction, bias, speed, trust, still, |a|, |w|, rest |a|), `V` and `I` toggle the hold-back and immediate-write behaviours of flash writes, `W` and `X` write and remove storage self-test files, `e` open the first record's event page. A status line prints every 5 s with lux, night state, backlight, render and I2C statistics; the boot banner prints the reset reason (11 is a USB reset, which an upload causes).

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

### Storage and clock

`src/app/attitude.*`: the gyro-propagated quaternion of CLAUDE.md section 5, plain C++ with no Arduino dependency; `vehicleFrameG()` turns one accelerometer sample into longitudinal and lateral g from the current up and the nominal forward axis (`IMU_FORWARD_SENSOR` in board_config.h). `src/app/speed_fusion.*`: displayed speed from GPS and the integrated longitudinal g, also plain C++. `src/app/accel_cal.*`: the once-per-sensor accelerometer offset calibration (sphere fit over ten orientations, Settings or key `c`, docs/calibration.md); the offsets and radius live in the settings blob and every raw sample is corrected before anything else sees it. Both are tested on the Mac with `test/native/run.sh` (clang++, a synthetic drive with bounds on the G and speed errors; run it after touching either module). The sensor task feeds the engine two speeds: the displayed one (wheels, else fused GPS) and the raw GPS sample the timers run on. The status line shows the filter's state: `att tilt` (sensor Z from upright), `corr` (correction per sample, settles at zero), `bias` (learnt gyro bias), `lonbias` (residual learnt at standstill), `trust`/`hold` and `still`; and `refused`, the GPS polls the receiver declined by stretching the clock past the chip's 26 ms timeout.

`src/app/obd_client.*`: the car's data over BLE from the vLinker BM+ (CLAUDE.md section 3): one task on core 0 that scans for the dongle or reconnects to the stored one, finds its serial characteristics (the layout is logged, known Vgate ones preferred), sets the ELM327 up for BMW extended addressing and polls the identifiers of `include/obd_pids.h`, one ECU per visit. `src/app/obd_proto.*` parses the adapter's replies (ISO-TP frames as text) and `src/app/obd_decode.*` turns payloads into `CarData` with bounds checks; both are plain C++ and run in `test/native/run.sh` against replies recorded from cars. It uses the core's own BLE library on the NimBLE host; the stack takes 72 KB of internal RAM, so LVGL's object pool is allocated in PSRAM (`include/lv_conf.h`). Serial keys: `o` OBD on or off (stored), `O` status with per-identifier counts and round-trip times, `L` every request and reply, `F` forget the dongle. The first reply to each identifier is printed as an `OBD first:` line for docs/obd-notes.md.

`src/app/storage.*`: NVS (Preferences, namespace `minidash`) holds settings, the mounting reference, the lifetime records and the trip id counter; LittleFS on the `ffat` partition (the 9 MB the partition scheme sets aside, formatted on first use) holds `/trips/<id>.trp` (last 60, binary with a magic and version), `/events/<id>-<rec>.evt` (12 s of speed, G and power at 10 Hz) and `/active.trp`, the checkpoint written every 30 s while a non-demo trip has distance. At boot a checkpoint is closed into a trip file: a trip ends at power loss by design. Demo trips are never stored. On the prebuilt core all file writes are deferred until the displayed speed has been under 2 km/h for 2 s (or the trip ends, or a checkpoint is 10 min overdue): there a flash write blocks PSRAM reads and the panel's bounce buffers then repeat stale ten-line strips, which is visible as banded digits and ghost text unless the picture is static. Key `V` suppresses event saves entirely for diagnosis; `I` forces immediate writes. On the rebuilt framework (env `minidash-xip`, the default) writes are immediate; the deferral applies to the prebuilt core only (`MINIDASH_XIP`). At boot, event files whose trip has no trip file (demo trips, trips lost before their first checkpoint) are removed. `src/app/clock.*`: PCF85063A RTC read at boot, set from GPS once, UTC everywhere, Melbourne local time with the DST rule for display and day grouping. Serial keys `W` and `X` write and remove self-test trip and event files; `e` opens the first record's event page.

### Fonts

Inter 4.1 (SIL OFL) with the tabular-figures feature frozen into the default digits with fontTools (Inter's default digits are proportional), then converted with lv_font_conv 1.5.3 at 4 bpp, uncompressed: Medium 88 and 60 (digits and signs only), Medium 40/34/30/28/26, Regular 18/16/14, SemiBold 26/16 (ASCII plus – — · ° ± − ← → ↻ ☼). About 600 KB of flash. Regenerate with the recipe in docs/design/README.md.

### Done and not done in 3b (24 Oct 2026)

Done: LVGL on the panel with touch, splash with the emblem, header status cluster, five tabs with swipe, Settings by header tap, Drive, Trip (records, journey totals, event page without traces), Dynamics, Energy, Battery, demo replay, the trip and records engine, GPS, IMU and lux into the state, night mode from lux, screenshot tool.

Not yet: the gyro-propagated attitude (the IMU path uses a static mounting reference and a 5 Hz low-pass), the GPS and accelerometer speed fusion (displayed speed is wheel speed when OBD is live, else raw GPS), start-screen preference, WiFi OTA, the 30 fps target, the 30-minute I2C soak, and the first real-data drive for the trip and checkpoint paths. Done since: records and trip storage, event traces, the clock with DST, settings persistence, the GPS bus holds.

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
