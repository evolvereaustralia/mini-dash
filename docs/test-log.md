# Test log

Dated pass/fail against CLAUDE.md section 9. Nothing is done until it has a dated pass here.

| Date | Test | Result | Notes |
|---|---|---|---|
| 2026-10-24 | Sensor supply: VOUT on the I2C terminal reads 3.3 V with no sensors connected | PASS | 3.43 V measured unloaded by Logan. Within the 3.6 V absolute maximum of all three sensors (ISM330DHCX, NEO-M9N, VEML7700). Header cable A wired black GND, red VOUT, blue SDA, yellow SCL in the side-entry block; pins seated and tug-tested. |
| | I2C scan matches the expected map | | |
