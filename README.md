# Mini Dash

Dash-top speed, G-force and EV display for a 2022 Mini Cooper SE. Waveshare ESP32-S3-Touch-LCD-4.3C, u-blox NEO-M9N GPS, ISM330DHCX IMU, VEML7700 light sensor, vLinker BM+ OBD dongle over BLE, printed enclosure with no supports and no glue.

`CLAUDE.md` is the single source of truth: what is being built, the locked hardware, the firmware architecture, the enclosure, the acceptance tests and the build order. Read it first.

```
docs/                 decisions, BOM, assembly, calibration, test log, OBD notes
hardware/enclosure/   CadQuery model, printability checker, layout drawings, STL/STEP exports
hardware/datasheets/  Waveshare 4.3C schematic, 2D drawing, STEP; SparkFun board files
firmware/             PlatformIO project (phase 3a onward)
```

Enclosure: `cd hardware/enclosure && python3 mini_dash_enclosure.py export/ && python3 -I check_overhangs.py export/mini-dash-*.stl`. Drawings: `python3 diagrams.py`.
