# Calibration

Three calibrations, in this order. The first is per sensor and done once on the bench; the second is per mount and repeated whenever the device is refitted; the third is a road check.

## 1. Accelerometer offsets (once per IMU, on the bench)

Why: the ISM330DHCX has tens of mg of zero-g offset per axis (this unit: -2.7, -25.5, +17.4 mg, 25 Oct 2026). The attitude filter absorbs an uncorrected offset into its idea of "up" while the car stands still, which is harmless until the car turns: the absorbed error swings with the heading and a long corner reads up to 0.05 g of lateral error until the next cruise or stop corrects it. Measured once, the offsets are subtracted from every raw sample and the problem is gone for good.

How: with the IMU board loose or the device in hand, open Settings, IMU calibration, and tap Offsets (or send `c` over serial). The button reads "Turn n/10". Turn the device slowly through at least six orientations, resting it against the table or a box for about two seconds in each: flat, upside down, standing on each of its four edges, then a few tilts in between. The collector stores a direction whenever the last 0.3 s of samples lie within 30 mg of each other and the direction is more than 25 degrees from every one already stored; it stops at ten directions or after 90 s, needs at least six with each axis seen both ways, fits a sphere (offset and radius) and stores the result in NVS. The button then reads "Done"; the serial line shows the offsets, the radius and the residual (under 5 mg is a good fit, this unit gave 2.9). "Failed" means too few directions or an axis not seen both ways: start again and make sure the upside-down and the four edge positions are each held still.

Result on this unit: offset -2.7, -25.5, +17.4 mg, radius 1.0027 g, residual 2.9 mg (25 Oct 2026, ten directions in 50 s).

## 2. Mounting reference (per mount)

Why: the attitude filter needs a starting orientation so the G display is right from the first sample; it then tracks the real orientation on its own. The reference is the gravity direction in the sensor frame with the device fitted and the car level and still.

How: device on its plate, car parked on level ground, engine off, nobody moving in the car. Settings, IMU calibration, Mounting. Takes one sample and stores it. Repeat after refitting the device or moving the plate. The forward axis is not calibrated: it is the nominal axis of the board in the shell (`IMU_FORWARD_SENSOR` in board_config.h), and the filter projects it onto the horizontal plane, so the mount's pitch and roll do not matter. Its sign is checked on the first drive: acceleration must read positive on Drive.

## 3. Road checks (first drive)

- Forward sign: a gentle acceleration from rest reads positive longitudinal g, braking negative; if reversed, flip the sign of `IMU_FORWARD_SENSOR` and rebuild.
- Lateral sign: a left turn reads negative (positive is right); the G plot dot moves right in a right-hand bend.
- Zero at rest after a drive: within 0.02 g on both axes within two seconds of stopping. If not, the standstill gate (0.3 m/s GPS) or the gyro bias learning is suspect; key `g` prints the filter state at 10 Hz.
- Hard stop: during a firm brake from 60 km/h the longitudinal reading should be steady, not creeping, and return to zero when stopped; creep means the attitude is being corrected toward the accelerometer while it should not be (check the trust flag in the `g` log stays off during the stop).
- Speed: fused speed against the speedo at a steady 60 km/h (the speedo over-reads by 2 to 3 km/h) and against GPS raw; a lag between the two on launch means the GPS latency constant (`SpeedFusion::GPS_LATENCY_S`, 100 ms) needs refining.
