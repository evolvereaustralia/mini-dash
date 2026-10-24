# Assembly

Order follows the exploded drawing `hardware/enclosure/mini-dash-layout-4-exploded.png`; the tray and visor drawings show where everything sits.

1. Heat-set the ten M2 x 4 inserts: four GPS bosses, four IMU bosses, two light-sensor bosses under the visor roof.
2. GPS board onto its bosses, components toward the display, SMA jack pointing inward. Plug the 50 mm Qwiic cable into J4 (and nothing into J2) before the screws go in.
3. IMU onto its bosses, components toward the display. 50 mm cable into its left port, header cable A into its right port.
4. Antenna: peel the 3M pad, press it onto the visor floor against the back wall between the side ribs. Drop the coax through the coax notch and connect the SMA plug to the GPS jack.
5. Light sensor against the ends of its two bosses, component side down, lens forward, two M2 screws from below. Header cable B through the Qwiic notch into one of its STEMMA ports.
6. Header cables A and B into the bottom-entry screw terminal block: black GND, red 3V3, blue SDA, yellow SCL, two wires per contact. Check VOUT is 3.3 V with a meter before this step (see test-log.md).
7. Display into the bezel pocket from behind. Shell over the bezel from behind until all four fingers click. The display is now trapped between the bezel lip and the shell bosses.
8. Visor cover slides in from the right along the roof and floor grooves until it stops.
9. Plate onto the dash with VHB, groove open end to the left. Slide the device on from the left until the rail hits the groove end.

To stow: slide the device left off the plate.
