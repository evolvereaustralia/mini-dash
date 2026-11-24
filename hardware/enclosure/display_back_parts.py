"""
Parts on the back of the Waveshare ESP32-S3-Touch-LCD-4.3C, measured from Logan's square-on photo of 9 Nov 2026
(hardware/datasheets/waveshare-4.3c/display-back-photo-2026-11-09.png), device frame: x right (USB-C end = +X),
y up, in mm about the glass centre. Heights are mm proud of the PCB back; those marked ~ are typical values, not
measured. Waveshare's STEP omits several of these, so THIS list, not the STEP, is what layouts are checked against.
Each entry: (name, x0, x1, y0, y1, height).
"""
PCB = (-52.0, 52.0, -23.0, 32.94)          # PCB outline; notch for the touch flex at x 29..43.5, y -24..-16
PCB_NOTCH = (29.0, 43.5, -24.0, -16.0)
PARTS = [
    ('screw heads (4)',        None, None, None, None, 1.0),          # at PCB_HOLES; see mini_dash_enclosure.PCB_HOLES
    ('TF slot',                37.2, 53.4, 6.3, 24.5, 1.9),
    ('ESP32 can',              14.0, 34.0, 6.3, 27.7, 3.2),
    ('USB-C',                  47.5, 56.0, -6.0, 4.4, 3.3),
    ('MIC1',                   49.0, 51.0, 5.0, 6.5, 1.0),
    ('BOOT switch',            50.0, 54.0, -16.0, -9.5, 3.5),
    ('RESET switch',           50.0, 54.0, -23.0, -17.0, 3.5),
    ('regulators',             34.0, 46.0, -8.0, 0.5, 1.5),
    ('inductor 4R7',           22.3, 30.7, -6.6, 0.5, 5.0),            # ~
    ('regulator IC',           17.0, 22.0, -3.4, 0.5, 1.2),
    ('6-pin JST (1..6)',       32.0, 39.0, -13.0, -9.0, 7.0),           # ~ vertical JST with plug room
    ('capacitors',             14.0, 19.0, -12.0, -8.0, 1.8),
    ('optocouplers (4)',       -10.0, 13.0, -9.0, 0.0, 4.0),            # ~ DIP-4 SMD
    ('terminal block + plug',  -31.0, 29.0, -34.0, -14.4, 7.0),         # header on the PCB; plug reaches ~17.2 behind the glass
    ('speaker',                -37.2, -11.0, 6.3, 33.2, 0.0),           # flush with the PCB back (Logan, 9 Nov 2026)
    ('speaker wires',          -47.0, -37.0, 5.0, 22.0, 1.5),
    ('LCD FPC connector',      -37.5, -13.5, -12.3, -6.0, 2.0),
    ('LCD flex notch in PCB',  -40.0, -10.0, -1.4, 2.5, 0.0),           # cut-out; flex lies on the PCB back here, ~0.3
    ('SPK connector',          -47.5, -41.0, 21.9, 27.7, 7.0),          # ~ JST PH with plug and wires
    ('coin cell holder',       -50.0, -41.0, -6.0, 4.4, 3.5),           # ~
    ('battery connector',      -45.0, -41.0, -14.3, -7.3, 7.0),         # ~ JST PH
    ('battery switch',         -51.5, -48.0, -15.0, -7.0, 4.0),         # ~
    ('strapping pads',         -44.0, -31.0, -22.0, -17.0, 0.8),
    ('GT911 flex + chip',      29.0, 43.5, -24.0, -16.0, 1.5),          # in the PCB notch, on the LCD frame
]
