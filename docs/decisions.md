# Decisions

One line of reasoning per decision, dated. Newest at the bottom. Hardware and architecture decisions made in chat before the repo existed are summarised in CLAUDE.md; this file starts at the repo.

- 2026-10-14: Hardware locked (4.3C, NEO-M9N SMA, ISM330DHCX, VEML7700, Qwiic). Reasons in CLAUDE.md section 2.
- 2026-10-23: OBD input (vLinker BM+, BLE, BMW UDS) brought into the first release instead of a later phase; bring-up stays staged because the dongle has not arrived.
- 2026-10-24: Dovetail flipped: rail on the shell, groove in the plate. A dovetail groove in a face that is vertical in print has a lip that starts in mid-air and cannot print; a rail with its wide end on the bed can.
- 2026-10-24: Rail leans back by RAIL_LEAN_DEG = 10 (nominal dash tilt) so the plate groove is vertical in print; the shell no longer changes when the tilt is measured, the plate absorbs it.
- 2026-10-24: Printability is checked layer by layer (check_overhangs.py, 45-degree step) before any STL is released; PrusaSlicer's warnings had been catching what a face-normal check missed.
- 2026-10-24: GPS board on 3 mm bosses, components toward the display, so its edge-mount SMA jack and plug clear the display PCB back by 1 mm. IMU moved to x = 26 to clear a connector row on the display back. Clearances taken from Waveshare's STEP, not the bare PCB outline.
- 2026-10-24: Second notch through the visor floor for the light sensor cable; the coax notch is under the antenna.
- 2026-10-24: Two Qwiic-to-header cables into the one screw terminal (IMU chain and light sensor) because the light sensor is out of 50 mm reach and the GPS board's second port faces the wall.
- 2026-10-24: M2 x 4 heat-set inserts and M2 x 6 screws for all sensor mounts: the CE10192 kit has no M2.5 and M3 inserts do not fit a 6 mm boss.
- 2026-10-24: USB-C opening is a notch from the shell's seam edge rather than a window, removing a 16 mm bridge.
- 2026-10-24: Plate adhesive is genuine 3M VHB 5952, not a general-purpose mounting tape: the F56 dash top is a grained low-surface-energy plastic (electrical tape would not stick), Gorilla's data sheet excludes polypropylene, and 5952 is the grade phone-mount makers use on dashes. Promoter only if a hidden test patch fails, because promoter makes removal leave a mark.
- 2026-10-24: Firmware toolchain is PlatformIO on the pioarduino platform release 55.03.311 (Arduino core 3.3.11, the version Waveshare's ci.json pins); the official PlatformIO espressif32 platform stopped at Arduino core 2.x.
- 2026-10-24: PlatformIO runs from a virtualenv on Homebrew python@3.12 (`~/.platformio/penv`, `pio` symlinked into /opt/homebrew/bin) because the pioarduino build scripts refuse Python 3.15, which is what Homebrew's own platformio formula ships.
- 2026-10-24: Waveshare's Arduino io_extension driver was ported onto Wire and the one I2cBus mutex instead of ESP-IDF's i2c_master API, so the SparkFun and Adafruit sensor libraries (which use Wire) and the board controller share one bus driver and one lock.
- 2026-10-24: The IO expander's PWM register is inverted (0 = brightest, Waveshare caps the dim end at 95 percent so the panel never blanks). IoExpander::setBacklightPercent takes human brightness and hides the inversion.
- 2026-10-24: Expected I2C map gains 0x40 (ES7210 microphone ADC, Waveshare es7210_adc.h 0x80 in 8-bit form) and confirms 0x18 for the ES8311 codec (es8311_codec.h 0x30 in 8-bit form). Both are on the bus whether we use audio or not, so the scan must expect them.
- 2026-10-24: Phase 3a drives the RGB panel with esp_lcd_panel_rgb directly, copied from Waveshare's rgb_lcd_port (IDF 5.5 renames psram_trans_align to dma_burst_size). Whether phase 3b uses ESP32_Display_Panel as CLAUDE.md section 5 says, or Waveshare's own esp_lcd_touch GT911 port, is still to decide.
