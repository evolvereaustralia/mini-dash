# Datasheets and board files

- `ESP32-S3-Touch-LCD-4.3C-3D.stp`: Waveshare's STEP of the display board, from https://github.com/waveshareteam/ESP32-S3-Touch-LCD-4.3C (hardware/dimensions). The enclosure clearances and `diagrams.py` read this file.
- `waveshare-4.3c/`: 2D drawing (PDF, DXF), schematic PDF, and `repo-config/ci.json` (the Arduino core and FQBN options the firmware toolchain is pinned to).
- `sparkfun-neo-m9n-sma/`: Eagle board and schematic from https://github.com/sparkfun/SparkFun_u-Blox_NEO_M9N. Component positions (SMA jack, Qwiic ports) in the layout drawings come from this.
- `sparkfun-ism330dhcx/`: Eagle board and schematic from https://github.com/sparkfun/SparkFun_Qwiic_6DoF_ISM330DHCX.

Still to add: ADA5378 (VEML7700 right angle) drawing from Adafruit, GPS-00177 antenna datasheet, vLinker BM+ documentation.
