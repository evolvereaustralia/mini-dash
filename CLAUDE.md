# Mini Dash: dash-top speed, G-force and EV display for a 2022 Mini Cooper SE

This file is the single source of truth for the project. Read it fully before doing anything. When a decision changes, update this file in the same commit.

Owner: Logan. Orchestration and research happen in Claude chat; CAD-as-code and firmware happen here in Claude Code. Australian English, "program" not "programme", no em dashes in docs or UI strings.

Version 2 brief, 23 Oct 2026. The original plan split GPS/IMU (phase 1) from OBD (phase 2). Logan has decided to build the OBD input into the first release, so this brief covers both. Bring-up is still staged (section 10) because the OBD path cannot be tested until the dongle arrives, but the architecture, data model and UI are designed for the full feature set from the start.

## 1. What we are building

A dash-top device for a 2022 Mini Cooper SE (F56 electric) that shows live speed, lateral and longitudinal G, acceleration and braking timers, trip stats, and the car's own EV data (battery state of charge, power in and out, battery and motor temperatures, torque, pedal, wheel speeds) on a 4.3" touch IPS screen. Powered from the car's USB-C port. GPS and IMU are inside the device on one I2C bus; the car data arrives over Bluetooth LE from a vLinker BM+ dongle in the OBD port. Printed enclosure in ASA or PETG, no supports, no glue; the device clips onto a thin adhesive plate on the dash and lifts off to be stowed. No custom PCB, no soldering, no car wiring beyond USB-C and the dongle.

Finished device about 122 x 100 x 55 mm including the visor and dovetail rail (the lens alone is 112.4 x 75.1 mm). It sits on the dash top directly above the instrument cluster, where the HUD combiner would be on HUD-equipped cars (this car has none), tilted back toward the driver. Fallback position: dash top between the cluster hood and the centre dial.

## 2. Locked hardware (do not substitute without asking)

All parts are in hand as of 23 Oct 2026 except the dongle (ordered from Amazon AU 21 Oct 2026, Amazon US stock), the USB-C cable, the M2 screws and the second header cable (all in carts 24 Oct 2026).

| Part | Detail |
|---|---|
| Main board | Waveshare ESP32-S3-Touch-LCD-4.3C, standard version, SKU 33799, no case. ESP32-S3-WROOM-1-N16R8 (16 MB flash, 8 MB OPI PSRAM), 4.3" 800x480 IPS, ST7262 RGB panel, GT911 5-point touch at 0x5D, PCF85063A RTC at 0x51 with CR927 cell, TF slot, USB-C (native USB, no UART bridge) and DC 7 to 36 V inputs, 16-pin 3.5 mm screw terminal (both rear-wiring and bottom-wiring blocks supplied) carrying I2C (VOUT 3.3 V by default: schematic H8 "I2C_VCC", selection resistors R88 to R92 all NC, note "the default paste is 3V3"), 5 V out, isolated DI/DO. Also carries ES8311/ES7210 audio and mics, which we ignore. No CAN or RS485 on this variant (GPIO15/16/43/44 are I2S). Panel brightness in cd/m2 is not stated by Waveshare: measure against a phone and record in docs/decisions.md. Operating temperature not stated; assume 0 to 65 C. Ships with a factory demo image; our firmware replaces it. |
| IO expander | At I2C 0x24. Waveshare's older docs call it CH422G; their current BSP identifies it as a CH32V003 running Waveshare's own register protocol. The working driver is in the 4.3C repo at `examples/esp-idf/03_lcd/components/io_extension/` (registers: mode 0x02, IO output 0x03, IO input 0x04, PWM 0x05, ADC 0x06, RTC_INT 0x07; IO1 touch reset, IO2 backlight enable, IO3 audio PA, IO4 SD CS, IO5 USB/CAN select). Backlight brightness is `IO_EXTENSION_Pwm_Output(uint8_t)` on register 0x05. Port this driver; do not use ESP32_IO_Expander's CH422G class. |
| GPS | SparkFun GPS Breakout NEO-M9N, SMA, Qwiic (GPS-17285). u-blox M9, up to 25 Hz, Doppler speed, four constellations, I2C 0x42, SMA, backup battery. UBX over I2C via SparkFun u-blox GNSS v3 library (confirm M9N support in the pinned version at bring-up; fall back to v2 if v3 refuses it). Run at 10 Hz PVT. Board 40.64 x 36.8 mm, holes at (2.54, 2.54), (38.1, 2.54), (2.54, 34.26), (38.1, 34.26) from its corner. |
| GPS antenna | SparkFun Antenna GPS Embedded SMA (GPS-00177): active ceramic patch, 26 dB LNA, 3.3 V bias, 5 inch lead to SMA. Inside the visor facing the sky through the windscreen. SparkFun flags the cable joint for strain relief in vibration; the enclosure provides it. |
| IMU | SparkFun 6DoF IMU ISM330DHCX, Qwiic (SEN-19764, 1 x 1 inch, two Qwiic ports). I2C 0x6B. Screwed rigidly to the shell; the shell is referenced to the dash through the dovetail and adhesive plate. |
| Light sensor | Adafruit Right Angle VEML7700, STEMMA QT/Qwiic (ADA5378). I2C 0x10. Mounted under the visor roof, lens facing forward through a slot in the visor cover (cabin light, which is what the driver's eyes see). |
| Cabling | Two Qwiic-to-male-header cables (ADA4209, 150 mm; one more to buy, 24 Oct 2026) both landing in the same four I2C screw-terminal contacts: cable A to the IMU right port, cable B up through the visor floor to the light sensor. One 50 mm Qwiic cable (ADA4399) from the IMU left port to the GPS board's J4; the other two 50 mm cables are spares. The light sensor is too far from any other sensor for a 50 mm cable, and the GPS board's second port (J2) faces the shell wall, so a single daisy chain does not work. Standard Qwiic colours: black GND, red 3.3 V, blue SDA, yellow SCL. Fit the bottom-entry (side-wiring) terminal block so the wires leave downward. |
| OBD dongle | Vgate vLinker BM+ (Amazon AU ASIN B08BR4KY1G, Vgate brand only, not the Xiuganpo or other reseller listings). Bluetooth 4.0 dual-mode: BLE (used by us) and Classic. ELM327 v2.2 command set. 51 mA working, 3 mA sleep, auto-sleep on bus idle, wake on bus activity, battery-drain protection. BMW-specific firmware, recommended by BimmerCode/BimmerLink for F-series. Update its firmware with Vgate's Windows tool before first use. |
| Power | 1 m right-angle USB-C to USB-C cable from the console port (lower of the two ports; the upper is USB-A). Route forward along the console, up the driver's side of the centre stack, across the dash behind the cluster hood. If the port sags under load, a 12 V socket USB-C PD adapter (3 A). |
| Enclosure | ASA or PETG, FDM, three parts (bezel, shell, plate), all printed flat with no supports. Heat-set inserts and screws: the CE10192 kit in hand has M2, M3, M4, M5, M6 and no M2.5, so all sensor bosses take its M2 x 4 mm inserts (3.5 mm OD, 3.2 mm hole) with M2 x 6 mm pan-head machine screws, 10 needed (GPS 4, IMU 4, light sensor 2), none in hand as of 24 Oct 2026. M2 passes through SparkFun's 3.3 mm and Adafruit's 2.5 mm holes alike. No other screws anywhere: display trapped by the bezel, bezel snapped to the shell, plate glued to the dash. Plate adhesive: genuine 3M VHB 5952 (12 mm, black), three strips under the 90 x 60 plate, on a dash wiped with isopropyl and left 24 h before loading. The F56 dash top is a grained plastic that ordinary tapes do not wet (electrical tape would not stick at all, 24 Oct 2026), so general-purpose mounting tapes such as Gorilla are out (Gorilla's own data sheet excludes polypropylene). Test a patch on a hidden spot first; if it fails, add 3M Adhesion Promoter 4298 and accept that removal will then leave a mark. The weighted-base idea from the first brief is dead. |

### Hardware facts that constrain the design

- The RGB panel consumes almost every GPIO. Exposed on the 4.3C: I2C on GPIO8 (SDA) / GPIO9 (SCL), shared with touch (0x5D), the IO expander (0x24), the RTC (0x51) and the ES8311 codec (read its address from the schematic, 0x18 is typical); isolated DI/DO through the expander; TF card on GPIO11/12/13. There is no free UART, so all sensors are on the one I2C bus, and the car data comes in over BLE.
- Expected I2C map: 0x10 VEML7700, 0x18 (probably) ES8311, 0x24 IO expander, 0x42 NEO-M9N, 0x51 PCF85063A, 0x5D GT911, 0x6B ISM330DHCX. Run a scan at bring-up and record the real map in `board_config.h`; stop and ask if anything unexpected appears. Bus at 400 kHz.
- VOUT on the I2C terminal must read 3.3 V with a multimeter before any sensor is connected. All three sensors are 3.3 V parts.
- Every I2C access, including LVGL's touch read, the IO expander (backlight PWM) and the RTC, goes through one FreeRTOS mutex with a bounded hold time. No driver may touch the bus outside it.
- The ESP32-S3 has BLE only, no Bluetooth Classic. The dongle's BLE side is a GATT serial service (Vgate uses service 0xFFF0 with notify characteristic 0xFFF1 and write characteristic 0xFFF2 on most of their adapters; confirm by scanning the dongle's services at bring-up). WiFi and BLE share the S3's radio; keep WiFi off while driving and only enable it in the OTA settings mode.
- Toolchain pinned from the 4.3C repo's `config/ci.json`: Arduino core 3.3.11, FQBN options `USBMode=hwcdc, CDCOnBoot=cdc, CPUFreq=240, FlashMode=qio, FlashSize=16M, PartitionScheme=app3M_fat9M_16MB, PSRAM=opi, UploadSpeed=921600`. In PlatformIO use `board = esp32-s3-devkitc-1` with these as board overrides. Build and flash Waveshare's own LCD example first; only then build ours. Wrong PSRAM or flash mode is the classic cause of "flashed fine, screen black".
- Flashing: the single USB-C port is native USB, auto-reset normally works. Recovery: hold BOOT, press and release RESET, release BOOT, flash, press RESET. Factory image restore: `esptool --chip esp32s3 erase-flash` then `write-flash 0x0 firmware/ESP32-S3-Touch-LCD-4.3C-Test.bin` from the 4.3C repo.
- Links: docs https://docs.waveshare.com/ESP32-S3-Touch-LCD-4.3C, GitHub https://github.com/waveshareteam/ESP32-S3-Touch-LCD-4.3C (schematic in `hardware/schematics/`, 2D PDF, DXF and STEP in `hardware/dimensions/`, IO expander driver in the ESP-IDF examples, factory image in `firmware/`). SparkFun NEO-M9N hardware: https://github.com/sparkfun/SparkFun_u-Blox_NEO_M9N.

## 3. Car data over OBD (new in v2)

### What the Cooper SE exposes

Almost none of it is standard OBD. It is BMW UDS "ReadDataByIdentifier" (service 0x22) traffic to individual ECUs. Two open projects have mapped it and are the references: OVMS `vehicle_minise` (working C++ on ESP32, read-only, derived from their BMW i3 module, https://docs.openvehicles.com/en/latest/components/vehicle_minise/docs/index.html, source in the OVMS v3 repo under `vehicle/OVMS.V3/components/vehicle_minise`) and the OBDb MINI Cooper parameter list (https://pelican.clutch.engineering/cars/mini/cooper/, repo github.com/OBDb/MINI-Cooper). Pull ECU addresses, DIDs, scaling and units from those sources into `firmware/include/obd_pids.h` with a comment citing where each came from. Do not invent DIDs.

Metrics to read, in priority order (first-release set marked *):

- * HV battery SOC (actual, plus high and low usable limits); displayed SOC = scaled between the limits so it matches the dash
- * HV battery voltage and current; power kW = V x I / 1000, positive discharging
- * Battery temperature min and max; cell voltage avg/min/max (spread = max minus min)
- * Motor speed, actual torque, requested torque; motor temperatures 1 and 2
- * Accelerator pedal position
- * Wheel speeds (four) and their average; this is the true-speedo speed, display it alongside GPS speed
- * 12 V battery voltage
- Inverter and drive-electronics temperatures, DC-DC board temperature
- State of health, battery capacity
- Range by mode, consumption per 100 km, auxiliary power
- Charging telemetry (plug, cable rating, AC per phase, DC V and I, charger temperature, derating). Only when stationary; a "charging" page is optional and last.
- Not available from the car per OVMS: per-cell voltages, trip regen kWh (compute regen from current integration instead).

### How it is read

- BLE central on the ESP32-S3 (NimBLE, not Bluedroid: smaller and more stable with LVGL and PSRAM). Scan for the dongle by name prefix "vLinker", connect, discover the serial service, subscribe to notify, write AT and UDS commands. Store the dongle's address in NVS after first pairing and reconnect to it directly on later boots.
- ELM327 setup sequence: `ATZ`, `ATE0`, `ATL0`, `ATS0`, `ATH1`, `ATSP6` (ISO 15765-4 CAN 11-bit 500 kbit) or `ATSP7` as the BMW references require, `ATAT1`. BMW F-series diagnostics use a tester ID of 0x6F1 with extended addressing: `ATSH 6F1`, `ATCEA xx` (xx = target ECU address), `ATCRA 6xx` (response filter), `ATFCSH 6F1`, `ATFCSD xx 30 00 00` (flow control must carry the ECU address byte), `ATFCSM 1`. Then send `22 DD DD` requests and parse the `62 DD DD ...` replies. Take the exact ECU addresses (SME battery, EME drive electronics, KOMBI, DSC) from the OVMS source; the i3 heritage suggests SME at 0x07 and EME at 0x0F but verify against the car.
- Poll scheduling: fast group (voltage, current, torque, pedal, wheel speeds) at 5 Hz as a round-robin, slow group (temperatures, SOC limits, 12 V) at 0.5 Hz, static group (SOH, capacity, counts) once per connection. Bound each request with a 300 ms timeout; after three consecutive timeouts drop to a reconnect state.
- The car's buses are awake with ignition on and for about 40 s after. The device shows "car asleep" when the dongle connects but requests time out; it never blocks the GPS/IMU UI on OBD.
- Treat everything the dongle returns as untrusted input: bounds-check every decoded value against physical limits before display or storage, and never let a malformed frame crash the parser.

## 4. Features (first release)

Pages, swipe left/right; tap the status bar for Settings. Status bar: GPS fix and sat count, BLE/OBD state (off, scanning, connected, car asleep), night-mode indicator, clock from RTC (set from GPS time once a fix is held).

1. Dash: speed (fused GPS, or wheel-speed average when OBD is live, with a small indicator of which), G-ball with fading 2 to 3 s trace and peak markers, battery SOC bar, power kW as a horizontal bar centred on zero (regen left, drive right), heading.
2. EV: power kW large, SOC with usable window, battery V and A, battery temp min/max, cell spread, motor temps, torque actual vs requested, pedal position, 12 V. Greyed with "no car data" when OBD is not live.
3. G detail: large G-ball, numeric lateral, longitudinal, combined, session peaks with reset.
4. Timers: 0-60, 0-100 km/h, 100-0 braking time and distance. Auto-arm, auto-start, best runs in NVS with RTC timestamps. When OBD is live, also log peak power kW and peak torque for the run.
5. Trip: max speed, distance, moving time, average moving speed, altitude; when OBD is live add energy used kWh (integrate power), regen kWh, Wh/km, and SOC used.
6. Settings: IMU mounting calibration, units, night mode auto/on/off plus manual brightness, OBD on/off and "forget dongle", reset bests, about/version, WiFi OTA mode (off by default, times out after 10 min).

## 5. Firmware architecture

- PlatformIO, Arduino framework, core and board options as pinned in section 2. LVGL 8.4 with Waveshare's `lv_conf.h` as the starting point; ESP32_Display_Panel for the RGB panel and GT911; the ported IO expander driver for backlight and touch reset.
- Tasks: core 1 runs LVGL (5 ms tick) and nothing else heavy. Core 0 runs the sensor task (I2C: IMU FIFO at 208 Hz, GPS PVT at 10 Hz, lux at 1 Hz) and the OBD task (NimBLE client, poll scheduler, parser). All three share a single `VehicleState` struct protected by a mutex, with a monotonic timestamp on every field so the UI can grey out stale values (GPS stale after 500 ms, OBD fast group after 1 s, slow group after 5 s).
- Attitude and gravity: static mounting calibration is not enough (1 degree of pitch error projects 0.017 g into the longitudinal axis). Keep a gyro-propagated quaternion at 208 Hz, corrected slowly toward the accelerometer gravity vector only when near-stationary or in steady motion (GPS speed stable, low gyro rates), never during sustained acceleration or cornering. Vehicle-frame G = rotate(accel) minus gravity. Estimate accelerometer bias at standstill.
- Three speed/G paths: (1) displayed speed = GPS Doppler speed fused with integrated longitudinal accel in a complementary filter, time-aligned on UBX iTOW; when OBD wheel speed is live and fresh, it becomes the display source and GPS is the cross-check; (2) timing path = raw GPS speed samples with crossing-time interpolation, no smoothing, unchanged by OBD; (3) G graphics = 2nd-order low-pass about 5 Hz, peak hold with slow decay, ring buffer.
- Timers: fun and personal comparison, tolerance about 0.1 s. km/h thresholds, mph display only. Arm when GPS speed under 1 km/h and accel under 0.05 g for 1 s with 3D fix, 7 or more sats, speed accuracy under 0.5 m/s. Start on fused speed crossing 1 km/h. Invalidate on fix loss, degraded accuracy or a missed GPS epoch. Braking timer arms above 100 km/h.
- Night mode: VEML7700 lux with hysteresis (below 15 lux for 5 s enters, above 40 lux for 5 s exits) drives real backlight PWM through the expander, plus a dark palette. Manual override in Settings.
- State: NVS (Preferences) for calibration, settings, dongle address, best runs, trip history. Atomic writes (write, then commit a version key). Nothing on the TF card.
- Boot: dash visible within 1 s target, before any sensor or BLE is ready. Explicit "acquiring" and "connecting" states. Timers locked until fix gates pass. BLE scan starts at boot and retries with backoff; it must never delay the UI.
- Power loss is a reboot; only an in-progress timer run is lost.

## 6. Enclosure (CadQuery, parametric)

- Draft 2.5 exists: `hardware/enclosure/mini_dash_enclosure.py` with `check_overhangs.py` beside it. Run `python mini_dash_enclosure.py export/` to regenerate `export/mini-dash-{bezel,shell,plate,visor-cover}.stl` and `mini-dash-assembly.step`, then `python3 -I check_overhangs.py export/mini-dash-*.stl`; the check must report nothing but the accepted bridges listed below before any STL is released. Board numbers were measured from Waveshare's 4.3C STEP and 2D drawing (lens 112.4 x 75.1, PCB 104 x 56, holes at x +-46 / y -22 and +28 in STEP frame, glass front to PCB back 10.2, terminal plug to 17.2, USB-C on the +X end at y -2.5, z 9.4 to 13.5) and from SparkFun's NEO-M9N SMA .brd.
- Printing rule: every part prints flat on its largest face with no supports. Bezel face down, shell back down, plate adhesive-face down, cover flat. The test is the slicer's: slice at 0.2 mm and no layer may contain material that is not on the layer below (45-degree step), other than short two-sided bridges. Accepted bridges: the four snap windows (8 mm), the vent slot tops (1.8 mm) and a one-layer sliver where the stop rib's foot crosses the coax notch (5.6 mm). Everything else was redesigned to pass: snap catches 45 degrees, horizontal bosses carry a web down to the back wall, visor cover grooves have 45-degree front faces, the cover entry slot and coax notch have chevron tops, the antenna stop rib is gabled, the SMA knockout is a teardrop, the USB-C opening is a notch from the seam edge rather than a window. Lessons: a square 119 mm groove reads as a 119 mm bridge (draft 2.2); a dovetail GROOVE in a face that is vertical in print has a lip whose underside starts as a knife edge in mid-air, whatever the flank angle, so it can never print (drafts 1 to 2.3, caught by Logan from the PrusaSlicer preview); a horizontal peg off a vertical wall starts as a line in mid-air even as a teardrop. Keep it that way when editing.
- Joints, no glue: bezel to shell by four cantilever snap fingers (45-degree catches, about 170 N total retention; release by pulling, or with a flat blade through the windows). Device to plate by a 42-degree dovetail with the RAIL on the shell and the GROOVE in the plate: the rail (7 mm neck, 3.2 mm deep, 56 mm long) hangs under the shell's bottom wall with its wide end flush with the shell back so its lowest corner is on the bed; it leans back by RAIL_LEAN_DEG (10, the nominal dash tilt) so the plate groove is vertical in print and both lips are 42-degree overhangs. The groove is open at the plate's left end and blind at the right, so the groove end is the stop. Visor cover slides into 0.8 mm grooves in the visor roof and floor from the +X end, with a finger notch to pull it.
- Parts: bezel (102 x 61 window, 2 mm pocket walls, snap fingers, USB-C notch); shell (25 mm tray; four bosses with 4.5 mm recesses press the display PCB forward against the bezel lip; M2 insert bosses for the GPS board at (-57, -13) origin (3 mm bosses, components toward the display, SMA jack pointing inward: the jack and its plug then clear the display PCB back by 1 mm) and for the IMU centred at (26, 2) (5 mm bosses, components toward the display, no standoff; x = 26 keeps its screw heads clear of the connector row on the display back); insert holes run 1.5 mm into the back wall so a 4 mm insert seats fully; open-fronted 30 mm visor whose 26 x 26 antenna pocket sits at the BACK against the visor rear wall between two side ribs and a gabled front stop rib, antenna looking up through the 2 mm roof; two chevron-topped notches through the floor: a 6 mm coax notch under the antenna front edge and an 8 mm Qwiic notch at x = 22 for the light sensor cable; light sensor board flat under the roof on two insert bosses (each on a web to the back wall, insert hole open on the board side) with its lens facing forward through a 7 x 3.5 slot in the cover; vents; 6.6 mm teardrop SMA knockout in the -X wall; USB-C notch open at the seam edge; dovetail rail); visor cover (1 mm plate, 119.9 x 10.7); plate (90 x 60, TILT_DEG default 10, dovetail groove centred on plate Y = +7 so the device back face lands about 14 mm from the windscreen edge, 4.3 mm of plate under the groove floor). Finished shell 121.6 x 100.3 x 55.
- Measured 24 Oct 2026 and now in the model: GPS-00177 antenna 25.0 x 25.0 ceramic, 8.2 thick bare, 9.2 with its 3M pad (pad stays on and sticks to the pocket floor), coax exits mid-edge under the LNA can, about 135 mm to the SMA plug; pocket is 26 x 26 with a rear stop rib and a 4 x 2.5 coax channel. IMU board 25.4 square, 1.57 thick, holes 20.32 square (confirmed from SparkFun's brd), Qwiic connectors 3.1 proud of the board top, so the IMU sits on 1.5 mm standoffs; board axes X along the connector edge, Y along the header edge, Z out of the board. ADA5378 light sensor 25.49 x 17.80, holes 2.5 on 20.3 x 12.7, right-angle lens centred on the long edge; it mounts flat under the visor roof, component side down, against the ends of two 4 mm insert bosses (each on a web to the back wall), lens facing the 7 x 3.5 slot in the cover. Display-back clearances checked against Waveshare's STEP on 24 Oct 2026: the screw terminal block reaches z -17.2 (x -29 to 28, y -22.7 to -13.5), a connector row reaches -14.1 (x -9 to 12.6, y -10.9 to -0.9), the left-edge connector -13.7 (x -52 to -46, y -14.7 to -5.7), the ESP32 can -13.3, USB-C -13.45, TF slot -12.05; the GPS and IMU positions and boss heights were set to clear all of these.
- Open items: BOOT/RESET access not cut; visor fillets only on edges that stay vertical in print; antenna strain relief is the coax channel only, add a zip-tie anchor; TILT_DEG to be set from the cardboard mock-up at position A (the shell is designed for 10 degrees and does not change with TILT_DEG; the plate groove leans by TILT_DEG - RAIL_LEAN_DEG, so if the measured tilt is outside 5 to 15 degrees change RAIL_LEAN_DEG and reprint the shell); USB-C handedness to be confirmed against the physical board before the bezel is printed.
- Layout drawings: `hardware/enclosure/mini-dash-layout-{1-tray,2-visor,3-sections,4-exploded}.png`, generated by `diagrams.py` from the model and the Waveshare STEP. Regenerate them whenever the layout changes.
- Print order: bezel test fit first (under an hour); shell after bring-up confirms the I2C map and antenna reception; plate after the tilt is measured.

## 7. Repo layout

```
mini-dash/
  CLAUDE.md
  docs/            bom.md, decisions.md, assembly.md, calibration.md, test-log.md, obd-notes.md
  hardware/
    enclosure/     mini_dash_enclosure.py, export/
    datasheets/    4.3C schematic, 2D drawing, STEP; NEO-M9N brd; sensor datasheets
  firmware/        platformio.ini, src/, include/ (board_config.h, obd_pids.h), lib/, test/
```

## 8. Working conventions

- Small commits, each buildable. `pio run` must pass before any firmware commit; `python mini_dash_enclosure.py export/` must succeed before any enclosure commit.
- Pin names, I2C addresses and board facts live in `firmware/include/board_config.h`; OBD ECU addresses, DIDs and scaling live in `firmware/include/obd_pids.h` with a source citation per line. Nowhere else.
- When a hardware fact is uncertain, check the schematic, the 4.3C repo or the OVMS/OBDb sources in `hardware/datasheets/` rather than guessing; if still unsure, stop and ask Logan.
- Log decisions in `docs/decisions.md` with a date and one line of reasoning. Log every OBD request/response pair seen during bring-up to `docs/obd-notes.md`.
- Keep UI strings short; the screen is read at a glance while driving. Nothing on screen requires more than one tap while moving.

## 9. Acceptance tests (pass/fail)

Record results in `docs/test-log.md`. Nothing is "done" until it has a dated pass.

- Power: measured current at full backlight with GPS, antenna and BLE live, plus boot peak; the Mini's USB-C port holds 4.75 to 5.25 V under that load through the chosen cable.
- Sensor supply: VOUT on the I2C terminal measures 3.3 V before any sensor is connected.
- I2C: a scan shows every expected address and no unexpected one; 30 minutes of simultaneous touch, GPS at 10 Hz and IMU FIFO reads with zero bus errors.
- Display: readable at arm's length at midday with sun on the windscreen, visor fitted; at night the backlight dims to a non-glare level with no visible flicker; measured brightness recorded.
- Startup: dash visible within 1 s; "acquiring" and "connecting" states shown; hot-start fix in under 10 s on the dash.
- GPS: internal patch antenna gives 8 or more satellites and speed accuracy under 0.3 m/s on an open road; else relocate the antenna, then fall back to the external puck.
- BLE/OBD: connects to the dongle within 10 s of ignition on; fast group holds 5 Hz with under 2 percent timeouts over a 20 minute drive; reconnects unaided after a bus sleep/wake cycle; power kW sign and magnitude agree with the car's own power gauge; wheel-speed average agrees with the speedo; SOC matches the dash within 1 percent after scaling.
- Mount: no movement relative to the plate under a hard stop from 60 km/h; IMU zero shifts under 0.02 g over a drive.
- Timing: three back-to-back 0-60 runs on the same road agree within 0.1 s; displayed GPS speed within the speedo's usual 2 to 3 km/h over-read; with OBD live, wheel speed and GPS speed agree within 2 km/h at steady 60.
- Power loss: 20 abrupt power cuts, including during a settings write, leave calibration, settings, dongle address and best runs intact.
- 12 V drain: with the dongle left in for 7 days of not driving, the car starts normally.
- Thermal: after 20 minutes in the sun with the device running, screen readable and touch working.

## 10. Build order and status

- Phase 0 and 1 (chat): decisions, BOM, independent review, revisions, parts ordered. Done 14 Oct 2026. All parts except the dongle and USB-C cable in hand 23 Oct 2026.
- Phase 2: repo set-up, datasheets copied into `hardware/datasheets/`, enclosure draft 2.5 with checker and layout drawings committed. Done 24 Oct 2026 from chat. Bezel test-fit print done by Logan 24 Oct 2026 (PETG, brim); a shell with the old 3.6 mm insert holes was also printed and serves as a fit-check shell only.
- Phase 3a (Claude Code, local, next): board bring-up. Build Waveshare's LCD example to pin versions. Then bring-up firmware: I2C scan against the expected map, backlight PWM ramp through the expander, GPS fix state and sat count, raw IMU, lux, printed once a second. VOUT check by Logan with a meter before sensors are connected.
- Phase 3b: LVGL UI skeleton with all six pages and the status bar, driven by `VehicleState`, with the EV page greyed. Sensor task, attitude, fusion, timers, trip, settings, night mode, NVS. This is drivable and testable before the dongle arrives.
- Phase 3c (when the dongle arrives): update its firmware; pair with BimmerLink on a phone and confirm it reads SE battery data (that validates the dongle before any of our code touches it); NimBLE client; ELM327 session; first DID reads logged to `docs/obd-notes.md`; poll scheduler; EV page live; trip energy; timer peaks.
- Phase 4: shell and plate prints after the I2C map, antenna reception and dash tilt are confirmed; assembly notes; calibration procedure; acceptance tests; iterate.
