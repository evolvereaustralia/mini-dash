# Firmware

Created in phase 3a. First build Waveshare's own LCD example with the pinned toolchain (CLAUDE.md section 2) to prove the flash/PSRAM settings, then the bring-up firmware: I2C scan, backlight PWM ramp through the IO expander, GPS fix and satellite count, raw IMU, lux, printed once a second.
