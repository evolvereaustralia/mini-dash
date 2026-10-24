"""
Mini Dash enclosure, draft 2.5 (parametric, CadQuery).

Four printed parts, no supports, no glue:
  1. bezel   : printed FACE DOWN. Plate with the screen window; pocket walls hold the
               display stack; four cantilever snap fingers on the top/bottom walls.
  2. shell   : printed BACK DOWN. Tray that slides over the bezel from behind and snaps
               onto the fingers; carries the sensor bosses, the visor (with the GPS
               antenna pocket), vents, USB-C notch, and the dovetail RAIL underneath.
  3. plate   : printed FLAT (adhesive face on the bed). Wedge sets the tilt; dovetail
               GROOVE the rail slides into from the left; the blind end is the stop.
  4. cover   : printed FLAT. 1 mm plate that slides into the visor front from the right.

Printability rule, checked layer by layer (0.2 mm) before any STL is released: no layer may
contain material that is not on the layer below, other than two-sided bridges of 10 mm or
less (snap windows, antenna stop rib, vent slots), and no supported overhang steeper than
45 degrees from vertical.

Device frame (all three parts are designed in it, then moved to print orientation):
  X = left-right, Y = up (+) / down (-), Z = toward the driver (+) / toward the windscreen (-).
  Origin = centre of the display glass, Z = 0 at the glass front face.

Board facts come from Waveshare's ESP32-S3-Touch-LCD-4.3C-3D.stp and -2D.pdf and SparkFun's
NEO-M9N SMA .brd. The light sensor and the SparkFun embedded antenna were not measured; their
pockets are parameters (ANT_*, see below).
"""
import math
import cadquery as cq

# ---------------- parameters (mm) ----------------
CLR = 0.3                      # sliding clearance
WALL = 2.0
# display board (4.3C)
GLASS_W, GLASS_H, GLASS_R = 112.4, 75.1, 4.0
GLASS_CY = -1.94               # glass centre in STEP frame (we re-origin on it)
STACK = 10.2                   # glass front to PCB back
TERM_DEPTH = 17.2              # glass front to the back of the screw-terminal plug
PCB_HOLES = [(-46, -22 - GLASS_CY), (46, -22 - GLASS_CY), (-46, 28 - GLASS_CY), (46, 28 - GLASS_CY)]
PCB_HOLE_HEAD = 4.5            # the screw heads / standoffs proud of the PCB back
LCD_CY = -2.4 - GLASS_CY       # active-area centre offset
WIN_W, WIN_H, WIN_R = 102.0, 61.0, 2.0
USBC_Y, USBC_Z0, USBC_Z1 = -2.5 - GLASS_CY, 9.4, 13.5   # USB-C receptacle on the +X end
# enclosure
DEPTH = 25.0                   # bezel front face to shell back face
BEZ_T = 2.0                    # bezel plate thickness
BEZ_WALL_H = 13.0              # pocket wall height from the bezel front face
SHELL_T = 2.0
VISOR_FWD = 30.0               # how far the visor projects in front of the bezel face
# GPS-00177 measured 24 Oct 2026: 25.0 x 25.0 ceramic, 8.2 thick bare (9.2 with 3M pad), cable exits mid-edge under the LNA can
ANT_SIZE, ANT_T, ANT_PAD = 25.0, 8.2, 0.9
ANT_W, ANT_D = ANT_SIZE + 1.0, ANT_SIZE + 1.0        # pocket, 0.5 clearance a side
VISOR_CAV = ANT_T + ANT_PAD + 0.6                     # 9.7: pad stays on, sticks to the pocket floor
# ADA5378 measured 24 Oct 2026: 25.49 x 17.80 board, holes 2.5 mm on 20.3 x 12.7, right-angle sensor centred on the long edge
LIGHT_X, LIGHT_FWD = 42.0, 14.6                      # board centre (x, forward of the bezel face); front edge 2.7 mm behind the cover
LIGHT_BOARD_L, LIGHT_BOARD_W, LIGHT_HOLE_PITCH = 25.49, 17.80, 20.3
LIGHT_SLOT_W, LIGHT_SLOT_D = 7.0, 3.5                 # slot in the cover in front of the lens
QWIIC_NOTCH_X = 22.0                                  # second floor notch, for the light sensor's Qwiic cable
COVER_T, COVER_GROOVE = 1.0, 0.8                      # slide-in visor cover
# snaps
FINGER_W, FINGER_L, FINGER_GAP = 8.0, 9.0, 0.8
FINGER_X = 35.0                # fingers at +-FINGER_X on top and bottom walls
BUMP = 1.0                     # bump projection
BUMP_Z0, BUMP_Z1 = 10.0, 12.0  # catch face at Z0, ramp from Z1 to wall top
TOP_CATCH_RISE = 1.0           # top fingers: 45-degree catch so the bezel can be pulled off; bottom fingers: flat catch, released through the windows
# sensors on the shell back (inside face)
BOSS_H, BOSS_OD, INSERT_HOLE = 5.0, 6.0, 3.2   # IMU bosses; hole for the kit's M2 x 4 mm inserts (3.5 OD). The CE10192 kit has no M2.5.
GPS_BOSS_H = 3.0               # GPS board low: its edge-mount SMA jack and plug (7.9 dia) must clear the display PCB back at z -10.2
INSERT_DEPTH_INTO_WALL = 1.5   # insert holes run on into the 2 mm back wall so a 4 mm insert seats fully in a 3 mm boss
GPS_ORIGIN = (-57.0, -13.0)    # SparkFun NEO-M9N SMA board local origin (its corner) in device frame
GPS_HOLES = [(2.54, 2.54), (38.1, 2.54), (2.54, 34.2646), (38.1, 34.2646)]
IMU_C = (26.0, 2.0)            # SparkFun 1x1" ISM330DHCX centre; x = 26 keeps its screw heads clear of the connector row on the display back (x up to 12.6)
IMU_PITCH = 20.32              # confirmed from SparkFun brd 24 Oct 2026: 25.4 sq board, holes 2.54 in
IMU_STANDOFF = 0.0             # components face the display PCB; 5 mm bosses leave 1.4 mm over the Qwiic plugs under the PCB-back parts
# dovetail (draft 2.4): RAIL on the shell, GROOVE in the plate.
# A dovetail groove in a face that is vertical in print always has a lip whose underside starts at a knife edge in
# mid-air (the mouth edge), so the old strip under the shell could not print. A rail whose wide end stands on the
# bed can: its lower flank rests on the layers below it and its upper flank is a roof.
DT_NECK, DT_DEPTH, DT_FLANK_DEG = 7.0, 3.2, 42.0   # neck at the shell wall, protrusion, flank angle from the axis
DT_RAIL_L = 56.0               # clear of the bottom snap windows (|x| 30.7 to 39.3)
DT_CLR = 0.15                  # per side, rail to groove
RAIL_LEAN_DEG = 10.0           # rail leans back by the nominal tilt so the plate groove is vertical in print.
                               # The shell does not change when TILT_DEG is measured; the plate groove leans by
                               # (TILT_DEG - RAIL_LEAN_DEG), so keep TILT_DEG within 10 +- 5 or raise RAIL_LEAN_DEG.
# plate
PLATE_W, PLATE_D, PLATE_T = 90.0, 60.0, 3.0
TILT_DEG = 10.0                # measure the dash; positive tilts the screen back toward the driver
GROOVE_Y = 7.0                 # plate frame: groove centre line; the device back face then lands near Y = +14

# derived
POCKET_W, POCKET_H = GLASS_W + 2 * CLR, GLASS_H + 2 * CLR
BEZ_W, BEZ_H = POCKET_W + 2 * WALL, POCKET_H + 2 * WALL
SHELL_IN_W, SHELL_IN_H = BEZ_W + 2 * CLR, BEZ_H + 2 * CLR
SHELL_W, SHELL_H = SHELL_IN_W + 2 * SHELL_T, SHELL_IN_H + 2 * SHELL_T
SHELL_IN_BACK = -(DEPTH - SHELL_T)      # z of the shell's inside back face
SHELL_BACK = -DEPTH


def rrect(w, h, r, z0, z1, cx=0.0, cy=0.0):
    """rounded-rectangle prism from z0 to z1 (device frame)"""
    return (cq.Workplane("XY").workplane(offset=z0).center(cx, cy)
            .rect(w, h).extrude(z1 - z0).edges("|Z").fillet(r))


def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


# ======================= 1. BEZEL =======================
def make_bezel():
    b = rrect(BEZ_W, BEZ_H, GLASS_R + WALL + CLR, -BEZ_WALL_H, 0)
    # glass pocket from the back, leaving the front plate
    b = b.cut(rrect(POCKET_W, POCKET_H, GLASS_R + CLR, -BEZ_WALL_H - 1, -BEZ_T))
    # screen window through the plate
    b = b.cut(rrect(WIN_W, WIN_H, WIN_R, -BEZ_T - 1, 1, 0, LCD_CY))
    # USB-C notch in the +X wall, open at the wall's free (back) edge
    b = b.cut(box(POCKET_W / 2 - 1, BEZ_W / 2 + 1, USBC_Y - 7.5, USBC_Y + 7.5, -BEZ_WALL_H - 1, -(USBC_Z0 - 1)))
    # snap fingers on the top and bottom walls
    for sy in (1, -1):
        wall_y_in = sy * POCKET_H / 2
        wall_y_out = sy * BEZ_H / 2
        for sx in (1, -1):
            fx = sx * FINGER_X
            # two slots either side of the finger, from the free edge toward the plate
            for dx in (-(FINGER_W / 2 + FINGER_GAP / 2), (FINGER_W / 2 + FINGER_GAP / 2)):
                b = b.cut(box(fx + dx - FINGER_GAP / 2, fx + dx + FINGER_GAP / 2,
                              min(wall_y_in, wall_y_out) - 1, max(wall_y_in, wall_y_out) + 1,
                              -BEZ_WALL_H - 1, -(BEZ_WALL_H - FINGER_L)))
            # bump: flat catch face at -BUMP_Z0 (toward the plate), 45-degree lead-in to the wall top
            rise = TOP_CATCH_RISE   # 45-degree catch on all four fingers (was flat on the bottom pair; flat = floating overhang)
            prof = [(0, -(BUMP_Z0 - rise)), (sy * BUMP, -BUMP_Z0), (sy * BUMP, -BUMP_Z1), (0, -BEZ_WALL_H)]
            bump = (cq.Workplane("YZ").workplane(offset=fx - FINGER_W / 2 + 0.3)
                    .polyline([(wall_y_out + p[0], p[1]) for p in prof]).close()
                    .extrude(FINGER_W - 0.6))
            b = b.union(bump)
    return b


# ======================= 2. SHELL =======================
def make_shell():
    s = rrect(SHELL_W, SHELL_H, GLASS_R + WALL + CLR + CLR + SHELL_T, SHELL_BACK, 0)
    s = s.cut(rrect(SHELL_IN_W, SHELL_IN_H, GLASS_R + WALL + 2 * CLR, SHELL_IN_BACK, 1))

    # --- visor: open-fronted channel on top of the shell. Roof, floor, back and side walls are all vertical in
    # print, so there are no bridges. The antenna sits at the BACK of the channel (nearest the windscreen) on its
    # 3M pad, boxed by two side ribs and a front stop rib, looking up through the 2 mm roof. The front of the
    # channel is closed by a separate 1 mm cover that slides in from the +X end along grooves in roof and floor.
    vy0 = SHELL_H / 2 - SHELL_T                       # shares the top wall
    vy1 = SHELL_H / 2 + VISOR_CAV + SHELL_T
    floor_y, roof_y = SHELL_H / 2, SHELL_H / 2 + VISOR_CAV
    cav_x = SHELL_W / 2 - SHELL_T
    visor = box(-SHELL_W / 2, SHELL_W / 2, vy0, vy1, SHELL_BACK, VISOR_FWD)
    visor = visor.cut(box(-cav_x, cav_x, floor_y, roof_y, SHELL_BACK + SHELL_T, VISOR_FWD + 1))
    s = s.union(visor)
    # antenna pocket: back wall, two side ribs and a front stop rib (floor to roof, 9.7 mm bridge, four-side anchored)
    ant_z0 = SHELL_BACK + SHELL_T
    for sx in (1, -1):
        s = s.union(box(sx * (ANT_W / 2 + 1) - 1, sx * (ANT_W / 2 + 1) + 1, floor_y, roof_y, SHELL_BACK, ant_z0 + ANT_D + 2.0))
    # front stop rib floor to roof, 2 mm thick, with a gabled (chevron) section so it climbs from both walls at
    # 45 degrees instead of bridging the 9.7 mm cavity
    rz0, mid_y, hh = ant_z0 + ANT_D, (floor_y + roof_y) / 2, (roof_y - floor_y) / 2
    rib = (cq.Workplane("YZ").workplane(offset=-ANT_W / 2 - 2)
           .polyline([(floor_y, rz0), (mid_y, rz0 + hh), (roof_y, rz0), (roof_y, rz0 + 2.0),
                      (mid_y, rz0 + hh + 2.0), (floor_y, rz0 + 2.0)]).close()
           .extrude(ANT_W + 4))
    s = s.union(rib)
    # two chevron-topped notches through the floor (= the tray's top wall), so they print without a bridge:
    # coax notch under the antenna's front edge, just behind the stop rib; Qwiic notch outside the pocket, between the
    # side rib and the light sensor, for the light sensor's cable (a JST SH plug is about 6 x 3.5 mm)
    nz0, nz1 = ant_z0 + ANT_D - 8.0, ant_z0 + ANT_D - 0.2
    def notch(xc, hw):
        return (cq.Workplane("XZ").workplane(offset=-(floor_y + 1)).center(xc, 0)
                .polyline([(-hw, nz0), (hw, nz0), (hw, nz1), (0, nz1 + hw), (-hw, nz1)]).close()
                .extrude(SHELL_T + 2))
    s = s.cut(notch(0.0, 3.0)).cut(notch(QWIIC_NOTCH_X, 4.0))
    # cover grooves in roof and floor, and the entry slot through the +X side wall
    # Grooves are cut with a 45-degree front face (trapezoid section) so the wall above them is never a flat
    # overhang: a square groove 119 mm long reads to the slicer as a 119 mm bridge. The side-wall entry slot gets
    # a chevron top for the same reason.
    cz0, cz1 = VISOR_FWD - 3.0, VISOR_FWD - 3.0 + COVER_T + 2 * CLR
    g = COVER_GROOVE
    def groove(y_wall, into):   # into = +1 cuts upward into the roof, -1 cuts downward into the floor
        prof = [(y_wall - into * 0.01, cz0), (y_wall + into * g, cz0), (y_wall + into * g, cz1), (y_wall - into * 0.01, cz1 + g)]
        return (cq.Workplane("YZ").workplane(offset=-cav_x - 1).polyline(prof).close().extrude(2 * cav_x + 2))
    s = s.cut(groove(roof_y, +1)).cut(groove(floor_y, -1))
    hw = (roof_y - floor_y) / 2 + g
    slot = (cq.Workplane("YZ").workplane(offset=cav_x - 1)
            .polyline([(floor_y - g, cz0), (roof_y + g, cz0), (roof_y + g, cz1), (floor_y + hw - g, cz1 + hw), (floor_y - g, cz1)]).close()
            .extrude(SHELL_T + 2))
    s = s.cut(slot)
    # light sensor: board flat against the roof underside on two teardrop bosses, lens on its front edge looking
    # forward through a slot in the cover (cabin light, which is what the driver's eyes see)
    lz = VISOR_FWD - LIGHT_FWD
    def teardrop(wp, r, up, length=4.0):
        # hole with its point upward so the ceiling is two 45-degree faces instead of a bridge
        k = r / math.sqrt(2); sgn = 1 if up else -1
        return (wp.circle(r).extrude(length)
                .union(wp.polyline([(-k, sgn * k), (k, sgn * k), (0, sgn * r * math.sqrt(2))]).close().extrude(length)))
    # Each boss is a horizontal peg off a vertical wall, so its underside would start as a line in mid-air. A web of
    # the same depth runs from the boss down to the shell back wall, so the boss stands on something.
    for dx in (-LIGHT_HOLE_PITCH / 2, LIGHT_HOLE_PITCH / 2):
        bz = lz - LIGHT_BOARD_W / 2 + 2.5          # holes on the header edge, 2.5 in from the board edge
        bx = LIGHT_X + dx
        wp = cq.Workplane("XZ").workplane(offset=-roof_y).center(bx, bz)
        b = wp.circle(BOSS_OD / 2).extrude(4.0)
        b = b.union(box(bx - BOSS_OD / 2, bx + BOSS_OD / 2, roof_y - 4.0, roof_y + 0.01, SHELL_BACK + SHELL_T - 0.01, bz))
        # insert hole open at the free end (the board side), blind 0.6 mm short of the roof
        wph = cq.Workplane("XZ").workplane(offset=-(roof_y - 0.6)).center(bx, bz)
        b = b.cut(teardrop(wph, INSERT_HOLE / 2, up=True, length=4.4))
        s = s.union(b)

    # --- snap windows in top/bottom walls
    for sy in (1, -1):
        for sx in (1, -1):
            s = s.cut(box(sx * FINGER_X - (FINGER_W / 2 + CLR), sx * FINGER_X + (FINGER_W / 2 + CLR),
                          sy * (SHELL_IN_H / 2 - 1), sy * (SHELL_H / 2 + (0.5 if sy > 0 else 1.0)),
                          -(BUMP_Z1 + 0.2), -(BUMP_Z0 - 0.2)))

    # --- USB-C opening in the +X end wall: a notch open at the shell's front (seam) edge rather than a window, so
    # there is no 16 mm bridge. The bezel wall behind it still closes the part above the plug.
    s = s.cut(box(SHELL_IN_W / 2 - 1, SHELL_W / 2 + 1, USBC_Y - 8.0, USBC_Y + 8.0, -(USBC_Z1 + 1.5), 1.0))
    # --- SMA knockout for a future external antenna, -X end wall (teardrop so its ceiling is not a bridge)
    sma = cq.Workplane("YZ").workplane(offset=-SHELL_W / 2 - 1).center(-25.0, -12.0)
    k = 3.3 / math.sqrt(2)
    s = s.cut(sma.circle(3.3).extrude(SHELL_T + 2)
              .union(sma.polyline([(-k, k), (k, k), (0, 3.3 * math.sqrt(2))]).close().extrude(SHELL_T + 2)))

    # --- vents: vertical slots in both end walls, slots in the back plate
    for sx in (1, -1):
        for y in (-34, -28, 22, 28, 34):
            s = s.cut(box(sx * (SHELL_IN_W / 2 - 1), sx * (SHELL_W / 2 + 1), y - 0.9, y + 0.9, -20.0, -6.0))
    for x in (40, 44, 48, 52, 56):
        s = s.cut(box(x - 1.0, x + 1.0, -13.0, 7.0, SHELL_BACK - 1, SHELL_IN_BACK + 1))
    for x in (-36, -40, -44, -48, -52):
        s = s.cut(box(x - 1.0, x + 1.0, -36.0, -26.0, SHELL_BACK - 1, SHELL_IN_BACK + 1))

    # --- bosses that press the display PCB forward against the bezel lip (recess seats over the screw heads)
    for (x, y) in PCB_HOLES:
        h = cq.Workplane("XY").workplane(offset=SHELL_IN_BACK).center(x, y).circle(3.5).extrude(-(STACK + 1.2) - SHELL_IN_BACK)
        h = h.cut(cq.Workplane("XY").workplane(offset=-(STACK + 1.2) - 0.01).center(x, y).circle(PCB_HOLE_HEAD / 2).extrude(1.2 + 0.1))
        s = s.union(h)

    # --- sensor bosses (heat-set M2 x 4 inserts, M2 x 6 screws): GPS board and IMU
    def boss_h(x, y, h):
        b = cq.Workplane("XY").workplane(offset=SHELL_IN_BACK).center(x, y).circle(BOSS_OD / 2).extrude(h)
        return b.cut(cq.Workplane("XY").workplane(offset=SHELL_IN_BACK - INSERT_DEPTH_INTO_WALL).center(x, y)
                     .circle(INSERT_HOLE / 2).extrude(h + INSERT_DEPTH_INTO_WALL + 0.01))
    for (hx, hy) in GPS_HOLES:
        s = s.union(boss_h(GPS_ORIGIN[0] + hx, GPS_ORIGIN[1] + hy, GPS_BOSS_H))
    for dx in (-1, 1):
        for dy in (-1, 1):
            s = s.union(boss_h(IMU_C[0] + dx * IMU_PITCH / 2, IMU_C[1] + dy * IMU_PITCH / 2, BOSS_H + IMU_STANDOFF))

    # --- dovetail rail under the bottom wall, running along X, wide end (tip) at the back so its lowest corner
    # sits on the bed in print. The rail leans back by RAIL_LEAN_DEG (tip toward the windscreen) so that on a dash
    # tilted by the same angle the plate groove is vertical. Flanks: lower one is a (42 - 10) degree overhang
    # resting on the layers below, upper one is a roof, tip face leans 10 degrees. Nothing starts in mid-air.
    s = s.union(dovetail_rail())
    return s


def dovetail_rail():
    """rail profile in the device frame, extruded along X. d = along the rail axis from the wall toward the tip,
    s = across the rail, positive toward the driver (+Z)."""
    y_out = -SHELL_H / 2
    L = math.radians(RAIL_LEAN_DEG)
    ta = math.tan(math.radians(DT_FLANK_DEG))
    half_neck, half_tip = DT_NECK / 2, DT_NECK / 2 + DT_DEPTH * ta
    a = (-math.cos(L), -math.sin(L))          # axis (y, z): away from the wall and toward the back
    sd = (-math.sin(L), math.cos(L))          # across (y, z)
    # axis height at the wall such that the tip's back corner lands exactly on the shell back face
    zc = SHELL_BACK - DT_DEPTH * a[1] + half_tip * sd[1]
    def P(d, s):
        return (y_out + d * a[0] + s * sd[0], zc + d * a[1] + s * sd[1])
    prof = [P(-1.5, -half_neck), P(-1.5, half_neck), P(0, half_neck), P(DT_DEPTH, half_tip),
            P(DT_DEPTH, -half_tip), P(0, -half_neck)]
    return (cq.Workplane("YZ").workplane(offset=-DT_RAIL_L / 2).polyline(prof).close().extrude(DT_RAIL_L))


# ======================= 3. PLATE =======================
def make_plate():
    # plate frame: X left-right, Y toward the windscreen (+), Z up. Printed as-is.
    t = math.tan(math.radians(TILT_DEG))
    h0 = PLATE_T + 6.0                      # wedge height at the plate centre line
    y0, y1 = -PLATE_D / 2, PLATE_D / 2
    wedge = (cq.Workplane("YZ").polyline([(y0, 0), (y1, 0), (y1, h0 - y1 * t), (y0, h0 - y0 * t)]).close()
             .extrude(PLATE_W / 2, both=True))
    wedge = wedge.edges("|Z").fillet(6.0)
    # dovetail groove cut into the tilted surface, running along X, open at the -X end (the device slides in from
    # the left), blind at +X so the groove end is the stop. Its axis is vertical when TILT_DEG == RAIL_LEAN_DEG,
    # otherwise it leans by the difference toward the driver, so both lips stay close to a 42-degree overhang
    # whatever the measured tilt (within reason, see RAIL_LEAN_DEG).
    phi = math.radians(TILT_DEG - RAIL_LEAN_DEG)
    hs = h0 - GROOVE_Y * t                   # surface height at the groove centre line
    ta = math.tan(math.radians(DT_FLANK_DEG))
    half_mouth = DT_NECK / 2 + DT_CLR
    depth = DT_DEPTH + 2 * DT_CLR
    half_floor = half_mouth + depth * ta
    a = (-math.sin(phi), -math.cos(phi))     # down the groove axis (y, z); phi > 0 leans toward the driver (-Y)
    sd = (math.cos(phi), -math.sin(phi))     # across the groove (y, z), toward the windscreen (+Y)
    def P(d, s):
        return (GROOVE_Y + d * a[0] + s * sd[0], hs + d * a[1] + s * sd[1])
    prof = [P(-6, -half_mouth), P(-6, half_mouth), P(0, half_mouth), P(depth, half_floor),
            P(depth, -half_floor), P(0, -half_mouth)]
    groove = (cq.Workplane("YZ").workplane(offset=-PLATE_W / 2 - 1).polyline(prof).close()
              .extrude(PLATE_W / 2 + 1 + DT_RAIL_L / 2 + 0.5))
    return wedge.cut(groove)


# ======================= 4. VISOR COVER =======================
def make_cover():
    """1 mm plate, printed flat, slides into the visor grooves from the +X end. Slot for the light sensor lens."""
    floor_y, roof_y = SHELL_H / 2, SHELL_H / 2 + VISOR_CAV
    cav_x = SHELL_W / 2 - SHELL_T
    y0, y1 = floor_y - COVER_GROOVE + CLR, roof_y + COVER_GROOVE - CLR
    x0, x1 = -cav_x - COVER_GROOVE + CLR, SHELL_W / 2 - 0.2
    c = cq.Workplane("XY").box(x1 - x0, y1 - y0, COVER_T, centered=False).translate((x0, y0, 0))
    # lens slot: lens centre sits about 7 mm below the roof underside (4 boss + 1.6 board + 1.5 lens)
    ly = roof_y - 7.1
    c = c.cut(cq.Workplane("XY").box(LIGHT_SLOT_W, LIGHT_SLOT_D, COVER_T + 2, centered=(True, True, False))
              .translate((LIGHT_X, ly, -1)))
    # finger notch at the +X end for pulling it out
    c = c.cut(cq.Workplane("XY").box(6, 3, COVER_T + 2, centered=(True, True, False)).translate((x1 - 3, y0, -1)))
    return c


# ======================= export =======================
if __name__ == "__main__":
    import sys
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    bezel = make_bezel()
    shell = make_shell()
    plate = make_plate()
    cover = make_cover()
    # print orientations: bezel face down (front face on the bed), shell back down, plate as modelled
    bezel_p = bezel.rotate((0, 0, 0), (1, 0, 0), 180).translate((0, 0, 0))            # front face at z=0, walls grow +z
    shell_p = shell.translate((0, 0, DEPTH))                                             # back face at z=0, walls grow +z
    cq.exporters.export(bezel_p, f"{out}/mini-dash-bezel.stl", tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(shell_p, f"{out}/mini-dash-shell.stl", tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(plate, f"{out}/mini-dash-plate.stl", tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(cover, f"{out}/mini-dash-visor-cover.stl", tolerance=0.02, angularTolerance=0.1)
    asm = cq.Assembly()
    asm.add(bezel, name="bezel", color=cq.Color("gray"))
    asm.add(shell, name="shell", color=cq.Color("gray50"))
    asm.save(f"{out}/mini-dash-assembly.step")
    for n, p in (("bezel", bezel_p), ("shell", shell_p), ("plate", plate), ("cover", cover)):
        bb = p.val().BoundingBox()
        print(f"{n}: {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, zmin {bb.zmin:.2f}")
