# Test log

Dated pass/fail against CLAUDE.md section 9. Nothing is done until it has a dated pass here.

| Date | Test | Result | Notes |
|---|---|---|---|
| 2026-10-24 | Sensor supply: VOUT on the I2C terminal reads 3.3 V with no sensors connected | PASS | 3.43 V measured unloaded by Logan. Within the 3.6 V absolute maximum of all three sensors (ISM330DHCX, NEO-M9N, VEML7700). Header cable A wired black GND, red VOUT, blue SDA, yellow SCL in the side-entry block; pins seated and tug-tested. |
| 2026-10-24 | I2C scan matches the expected map | PASS | Bring-up firmware, GPS, IMU and light sensor on header cable A: 0x10 VEML7700, 0x18 ES8311, 0x24 IO expander, 0x40 ES7210, 0x42 NEO-M9N, 0x51 RTC, 0x5D GT911, 0x6B ISM330DHCX. Nothing unexpected, nothing missing. Earlier the same day with nothing on the terminal the five on-board devices alone were found. Scan holds the bus 17 ms. |
| 2026-10-24 | Toolchain: flash and PSRAM options | PASS | esptool reports 16 MB flash and 8 MB embedded PSRAM; firmware reports PSRAM 8388608 bytes found, Arduino core 3.3.11, IDF 5.5.5; RGB panel framebuffers allocated in PSRAM. |
| 2026-10-24 | IO expander and backlight PWM | PASS (serial) | Expander acks, mode and output writes ok, PWM ramp 100 to 0 to 100 written without error. Visible dimming to be confirmed by eye. |
| 2026-10-24 | Display: colour bars visible, colour order correct; backlight dims visibly | PASS | Logan confirmed by eye: bars in SMPTE order (white, yellow, cyan, green, magenta, red, blue, black left to right), so the RGB data pin order is right, and the PWM ramp dimmed the panel visibly and back. Brightness in cd/m2 and the midday-sun and night tests remain. |
| 2026-10-24 | IMU streams at 208 Hz | PASS | WHO_AM_I 0x6B. 211 to 217 samples/s polled by data-ready (ISM330DHCX ODR runs a few percent high, within spec). Bench, board tilted: a = (-0.214, +0.031, +0.993) g, magnitude 1.016 g; gyro within about 2 dps at rest. 45 s with no bus errors. |
| 2026-10-24 | GPS answers over I2C at 10 Hz | PASS | NEO-M9N protocol version 32.01 (M9); SparkFun u-blox GNSS v3 3.1.15 accepts it, no fall-back to v2 needed. Nav rate 10 Hz confirmed: PVT count rises 10 per second. UTC date and time valid once fixed. |
| 2026-10-24 | GPS fix on the bench (not the section 9 open-road test) | PASS | Indoors at the bench with the patch antenna: 3D fix within 10 s of boot (hot start), 5 to 6 satellites, speed accuracy 0.28 to 0.32 m/s, horizontal accuracy 5.7 to 7.5 m, speed 0.03 to 0.05 m/s at rest. The 8-satellite, 0.3 m/s open-road test is still to run. |
| 2026-10-24 | Light sensor reads | PASS | VEML7700 gain 1/8, 100 ms: 162 to 164 lux indoors at the bench, steady to 0.5 lux. |
| 2026-10-24 | I2C hold time | PASS (bench) | GPS begin() holds the bus 266 ms and VEML7700 begin() 217 ms, once at start-up, before the stats reset. Steady state with touch idle, GPS 10 Hz, IMU 208 Hz and lux 1 Hz: longest hold 6 ms, none over the 25 ms limit. The 30-minute zero-error test with touch active is still to run. |
