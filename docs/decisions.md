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
