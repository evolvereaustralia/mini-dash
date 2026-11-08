"""
Layout and attachment drawings for the Mini Dash enclosure, drawn to scale in the device frame from the CadQuery
model and Waveshare's STEP of the 4.3C display board. Run from this directory:

    python3 diagrams.py

Writes mini-dash-layout-{1-tray,2-visor,3-sections,4-exploded}.png beside this file. Needs cadquery, trimesh,
shapely, matplotlib and vtk, and ../datasheets/ESP32-S3-Touch-LCD-4.3C-3D.stp. Takes a minute or two (the STEP
import is the slow part).
"""
import os, sys, math, tempfile
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mini_dash_enclosure as md
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.patches import Rectangle, Circle
import trimesh, shapely
from shapely.ops import unary_union
import cadquery as cq

OUT = HERE
STEP_43C = os.path.join(HERE, '..', 'datasheets', 'ESP32-S3-Touch-LCD-4.3C-3D.stp')
S = tempfile.mkdtemp(prefix='mini-dash-diagrams-')
os.makedirs(f'{S}/dev')

C_SHELL, C_BEZ, C_COVER, C_PLATE = '#9ccc9c', '#b0bec5', '#c5e1a5', '#cfd8dc'
C_PCB, C_PCBPART = '#e0e0e0', '#bdbdbd'
C_GPS, C_IMU, C_LIGHT, C_ANT = '#64b5f6', '#ffb74d', '#ba68c8', '#4db6ac'
C_CABLE, C_COAX = '#1565c0', '#424242'

# ---- geometry from the model
y_out = -md.SHELL_H / 2
floor_y, roof_y = md.SHELL_H / 2, md.SHELL_H / 2 + md.VISOR_CAV
cav_x = md.SHELL_W / 2 - md.SHELL_T
ant_z0 = md.SHELL_BACK + md.SHELL_T
GPS_W, GPS_H = 40.64, 36.8046
gx0, gy0 = md.GPS_ORIGIN                                # board unrotated: local (lx, ly) -> device (gx0 + lx, gy0 + ly)
gcx, gcy = gx0 + GPS_W, gy0 + GPS_H
def G(lx, ly):
    return gx0 + lx, gy0 + ly
JACK_X, JACK_Y = md.GPS_JACK                            # SMA jack axis, pointing +X
LANE_X1 = JACK_X + 50.0                                 # end of the plug (20) plus the stiff 30 mm of lead
gps_z = md.SHELL_IN_BACK + md.GPS_BOSS_H               # board underside
imu_z = md.SHELL_IN_BACK + md.IMU_BOSS_H
IMU_HALF = 25.4 / 2
lz = md.VISOR_FWD - md.LIGHT_FWD
light_bz = lz - md.LIGHT_BOARD_W / 2 + 2.5
PCB_CY = 3 - md.GLASS_CY
GLASS_FRONT_STEP_Z = 5.3                                # z of the glass front face in Waveshare's STEP frame
GLASS_Z = -md.BEZ_T                                     # glass front sits behind the bezel plate (device frame)


def plate_in_device_frame(plate):
    """Plate frame (X, Y toward windscreen, Z up) to device frame, seated under the shell with the rail in the groove."""
    L = math.radians(md.RAIL_LEAN_DEG); ta = math.tan(math.radians(md.DT_FLANK_DEG))
    half_tip = md.DT_NECK / 2 + md.DT_DEPTH * ta
    zc = md.SHELL_BACK + md.DT_DEPTH * math.sin(L) + half_tip * math.cos(L)
    t = math.tan(math.radians(md.TILT_DEG)); h0 = md.PLATE_T + 6.0; hs = h0 - md.GROOVE_Y * t
    def rotx(v, deg):
        a = math.radians(deg); c, s_ = math.cos(a), math.sin(a); x, y, z = v
        return (x, y * c - z * s_, y * s_ + z * c)
    p = rotx(rotx((0, md.GROOVE_Y, hs), -90), md.TILT_DEG)
    return (plate.rotate((0, 0, 0), (1, 0, 0), -90).rotate((0, 0, 0), (1, 0, 0), md.TILT_DEG)
            .translate((-p[0], y_out - p[1], zc - p[2])))


def build_meshes():
    """Device-frame STLs of the four printed parts and the display board, plus the display's back-side parts."""
    parts = {'bezel': md.make_bezel(), 'shell': md.make_shell(), 'plate': plate_in_device_frame(md.make_plate()),
             'cover': md.make_cover().translate((0, 0, md.VISOR_FWD - 3.0 + md.CLR))}
    for n, o in parts.items():
        cq.exporters.export(o, f'{S}/dev/{n}.stl', tolerance=0.05, angularTolerance=0.2)
    w = cq.importers.importStep(STEP_43C)
    sols = w.solids().vals()
    comp = cq.Compound.makeCompound(sols).translate((0, -md.GLASS_CY, -GLASS_FRONT_STEP_Z + GLASS_Z))
    cq.exporters.export(cq.Workplane().add(comp), f'{S}/dev/board43c.stl', tolerance=0.1, angularTolerance=0.3)
    back = []
    for so in sols:
        b = so.BoundingBox()
        if b.zmin - GLASS_FRONT_STEP_Z + GLASS_Z < md.PCB_BACK_Z - 0.3 and so.Volume() > 3:     # behind the PCB back face
            back.append((b.zmin - GLASS_FRONT_STEP_Z + GLASS_Z, b.zmax - GLASS_FRONT_STEP_Z + GLASS_Z, b.xmin, b.xmax,
                         b.ymin - md.GLASS_CY, b.ymax - md.GLASS_CY, so.Volume()))
    return back


back_parts = build_meshes()   # (zmin, zmax, xmin, xmax, ymin, ymax, volume), device frame


def rect(ax, x0, x1, y0, y1, **kw):
    ax.add_patch(Rectangle((min(x0, x1), min(y0, y1)), abs(x1 - x0), abs(y1 - y0), **kw))


def label(ax, text, xy, xytext, color='k', fs=8.5, ha='left'):
    ax.annotate(text, xy=xy, xytext=xytext, fontsize=fs, ha=ha, va='center', color=color,
                arrowprops=dict(arrowstyle='-', color=color, lw=0.8, shrinkA=0, shrinkB=1))


# =====================================================================================
# Figure 1: inside the tray, seen from the front with the display removed
# =====================================================================================
def fig_tray():
    fig, ax = plt.subplots(figsize=(20, 9))
    # shell outline and walls
    rect(ax, -md.SHELL_W / 2, md.SHELL_W / 2, -md.SHELL_H / 2, md.SHELL_H / 2, fc=C_SHELL, ec='k', lw=1.2)
    rect(ax, -md.SHELL_IN_W / 2, md.SHELL_IN_W / 2, -md.SHELL_IN_H / 2, md.SHELL_IN_H / 2, fc='#eef7ee', ec='k', lw=0.8)
    # visor above (seen edge-on here)
    rect(ax, -md.SHELL_W / 2, md.SHELL_W / 2, md.SHELL_H / 2 - md.SHELL_T, roof_y + md.SHELL_T, fc=C_SHELL, ec='k', lw=1.2)
    rect(ax, -cav_x, cav_x, floor_y, roof_y, fc='#eef7ee', ec='k', lw=0.8, hatch='////', alpha=0.6)
    # back-plate vent slots
    for x in (40, 44, 48, 52, 56):
        rect(ax, x - 1, x + 1, -36, -26, fc='white', ec='k', lw=0.5)
    for x in (-36, -40, -44, -48, -52):
        rect(ax, x - 1, x + 1, -36, -26, fc='white', ec='k', lw=0.5)
    # end-wall vents
    for sx in (1, -1):
        for y in (-34, -28, 22, 28, 34):
            rect(ax, sx * (md.SHELL_IN_W / 2), sx * (md.SHELL_W / 2), y - 0.9, y + 0.9, fc='white', ec='k', lw=0.5)
    # snap windows in top and bottom walls
    for sy in (1, -1):
        for sx in (1, -1):
            rect(ax, sx * md.FINGER_X - 4.3, sx * md.FINGER_X + 4.3, sy * md.SHELL_IN_H / 2, sy * md.SHELL_H / 2,
                 fc='white', ec='k', lw=0.6)
    # USB-C notch (+X wall) and SMA knockout (-X wall)
    rect(ax, md.SHELL_IN_W / 2, md.SHELL_W / 2, md.USBC_Y - 8, md.USBC_Y + 8, fc='white', ec='k', lw=0.6)
    ax.add_patch(Circle((-md.SHELL_W / 2 + md.SHELL_T / 2, -25), 3.3, fc='white', ec='k', lw=0.6))
    # coax notch and Qwiic notch through the top wall
    rect(ax, md.COAX_NOTCH_X - 5, md.COAX_NOTCH_X + 5, md.SHELL_IN_H / 2, md.SHELL_H / 2, fc='white', ec='k', lw=0.6)
    rect(ax, md.QWIIC_NOTCH_X - 4, md.QWIIC_NOTCH_X + 4, md.SHELL_IN_H / 2, md.SHELL_H / 2, fc='white', ec='k', lw=0.6)
    # rail under the bottom wall (behind the wall in this view)
    rect(ax, -md.DT_RAIL_L / 2, md.DT_RAIL_L / 2, y_out, y_out - 4.3, fc=C_SHELL, ec='k', lw=1.0, ls='--')
    # display PCB outline and the parts on its back (what the sensors must clear)
    rect(ax, -52, 52, PCB_CY - 28, PCB_CY + 28, fc='none', ec='#616161', lw=1.0, ls='--')
    rect(ax, -md.GLASS_W / 2, md.GLASS_W / 2, -md.GLASS_H / 2, md.GLASS_H / 2, fc='none', ec='#9e9e9e', lw=0.8, ls=':')
    for (z0, z1, x0, x1, yy0, yy1, vol) in back_parts:
        if vol < 20 or (x1 - x0) < 3:
            continue
        rect(ax, x0, x1, yy0, yy1, fc=C_PCBPART, ec='#757575', lw=0.6, alpha=0.75)
    # display press pads (on bare PCB) and, dotted, Waveshare's corner standoffs they avoid
    for (x, y) in md.PCB_PRESS:
        ax.add_patch(Circle((x, y), md.PCB_PRESS_R, fc=C_SHELL, ec='k', lw=0.8))
    for (x, y) in md.PCB_HOLES:
        ax.add_patch(Circle((x, y), 1.75, fc='none', ec='#616161', lw=0.6, ls=':'))
    # GPS board
    rect(ax, gx0, gcx, gy0, gcy, fc=C_GPS, ec='k', lw=1.0, alpha=0.85)
    for (hx, hy) in md.GPS_HOLES:
        ax.add_patch(Circle(G(hx, hy), md.BOSS_OD / 2, fc='white', ec='k', lw=0.6))
        ax.add_patch(Circle(G(hx, hy), 1.3, fc='#333', ec='none'))
    rect(ax, gx0 + 14.2, gx0 + 26.4, gy0 + 8.2, gy0 + 24.2, fc='#1e88e5', ec='k', lw=0.5)    # NEO-M9N module
    rect(ax, gcx - 9.5, gcx + 3.2, JACK_Y - 3.2, JACK_Y + 3.2, fc='#555', ec='k', lw=0.5)      # SMA jack body, pointing +X
    rect(ax, gcx + 3.2, gcx + 3.2 + 20, JACK_Y - 3.95, JACK_Y + 3.95, fc='#888', ec='k', lw=0.5)   # SMA plug
    rect(ax, gcx + 3.2 + 20, LANE_X1, JACK_Y - 1.5, JACK_Y + 1.5, fc='#aaa', ec='k', lw=0.5)  # 30 mm stiff section of the lead
    rect(ax, gx0 + 35.69 - 3, gcx, gy0 + 15.6 - 3, gy0 + 15.6 + 3, fc='#fff', ec='k', lw=0.5)  # J4 Qwiic, faces +X
    rect(ax, gx0, gx0 + 5.08 + 3, gy0 + 15.6 - 3, gy0 + 15.6 + 3, fc='#fff', ec='k', lw=0.5)   # J2 Qwiic, faces the left wall
    rect(ax, gx0 + 10.5 - 4.5, gx0 + 10.5 + 4.5, gcy - 7, gcy, fc='#fff', ec='k', lw=0.5)      # USB-C, top edge
    # IMU board
    ix, iy = md.IMU_C
    rect(ax, ix - IMU_HALF, ix + IMU_HALF, iy - IMU_HALF, iy + IMU_HALF, fc=C_IMU, ec='k', lw=1.0, alpha=0.9)
    for dx in (-1, 1):
        for dy in (-1, 1):
            ax.add_patch(Circle((ix + dx * md.IMU_PITCH / 2, iy + dy * md.IMU_PITCH / 2), md.BOSS_OD / 2, fc='white', ec='k', lw=0.6))
            ax.add_patch(Circle((ix + dx * md.IMU_PITCH / 2, iy + dy * md.IMU_PITCH / 2), 1.3, fc='#333', ec='none'))
    rect(ax, ix - 1.25, ix + 1.25, iy - 1.5, iy + 1.5, fc='#333', ec='none')
    rect(ax, ix - 3, ix + 3, iy - IMU_HALF, iy - IMU_HALF + 7.5, fc='#fff', ec='k', lw=0.5)    # Qwiic ports face down and up
    rect(ax, ix - 3, ix + 3, iy + IMU_HALF - 7.5, iy + IMU_HALF, fc='#fff', ec='k', lw=0.5)
    # cables
    # A: header cable from the terminal, right along the bottom, up into the IMU's BOTTOM port
    ax.plot([-3, 8, ix, ix], [-22, -28, -28, iy - IMU_HALF - 1], color=C_CABLE, lw=2)
    # 50 mm (or 100 mm): IMU TOP port, left across the top of the display back to GPS J4 (faces +X)
    ax.plot([ix, ix, 10, gcx + 1], [iy + IMU_HALF + 1, 27, 27, gy0 + 15.6], color=C_CABLE, lw=2)
    # B: header cable from the terminal, right and up the right side, to the Qwiic notch and into the visor
    ax.plot([6, 20, 55, 55, md.QWIIC_NOTCH_X, md.QWIIC_NOTCH_X], [-22, -31, -31, 44, 44, 48], color=C_CABLE, lw=2, ls=(0, (4, 2)))
    # coax: from the coax notch down to the end of the stiff section
    ax.plot([md.COAX_NOTCH_X, md.COAX_NOTCH_X, LANE_X1 - 3, LANE_X1], [48, 44, JACK_Y + 3, JACK_Y], color=C_COAX, lw=1.6)
    # terminal wiring points
    ax.plot([-3, 0, 3, 6], [-18, -18, -18, -18], 'o', color=C_CABLE, ms=4)

    # labels: left column (right-aligned at x = -70), right column (left-aligned at x = 70), top and bottom rows
    L, R = -70, 70
    label(ax, 'Visor cavity above the tray: antenna, light sensor\nand slide-in cover (see the plan view)', (-50, 47), (L, 50), ha='right')
    label(ax, 'GPS NEO-M9N board, left half, components toward the display,\nfour M2 x 3 inserts in 2 mm bosses, M2 x 4 screws. Its top edge\nis 0.45 mm under the top wall; J2 (left) faces the wall, unused.',
          (gx0 + 10, gy0 + 32), (L, 36), ha='right')
    label(ax, 'Antenna lead: plug (dark grey, 20 mm) and the 30 mm STIFF section\nbeside it run dead straight along y = 37, in the band between the\ndisplay board edge (y 33) and the bezel wall (y 46.5); nothing may\nsit under the display back here because the plug is 7.9 mm across',
          (JACK_X + 14, JACK_Y + 3.9), (L, 22), ha='right')
    label(ax, 'Cable A: 150 mm Qwiic-to-header cable, terminal\nalong the bottom to the IMU bottom port', (ix - 3, -27), (L, 10), ha='right')
    label(ax, 'Grey blocks: parts on the BACK of the display board\n(screw terminal 7 mm deep, USB-C, DC in, TF slot, ESP32\nmodule, FPC, switches). Sensor parts must clear them.',
          (-49, -10), (L, -4), ha='right')
    label(ax, 'SMA knockout, future external antenna', (-60, -25), (L, -18), ha='right')
    label(ax, 'Vents in the back plate and both end walls', (-44, -31), (L, -28), ha='right')
    label(ax, '16-pin screw terminal: both header cables land in\nthe same four contacts (SDA, SCL, 3V3, GND). Fit\nthe bottom-entry block so the wires exit downward.',
          (-14, -18), (L, -40), ha='right')
    label(ax, 'Dovetail rail on the outside of the bottom wall,\nflush with the shell back', (-20, y_out - 2.2), (L, -54), ha='right')
    label(ax, 'Display board (dashed outline): pressed forward against the bezel lip by four\n5 mm pads (green discs) on BARE PCB, placed from the STEP to miss every part on\nits back and the corner standoffs (dotted). No screws into the display board.',
          (md.PCB_PRESS[3][0], md.PCB_PRESS[3][1]), (R, 50), ha='left')
    label(ax, 'Grey blocks are now drawn 2 mm deeper than in drafts up to 2.6 (the glass sits\nbehind the 2 mm bezel plate); only 10.8 mm separates the shell back from the PCB back',
          (-20, -8), (-30, -58), ha='left')
    label(ax, 'IMU ISM330DHCX on the right under the TF slot, rotated 90 degrees so its\nQwiic ports face up and down; components toward the display; four M2 x 3\ninserts in 2.5 mm bosses, M2 x 4 screws. Board X is now the device Y axis.',
          (ix + 8, iy + 10), (R, 36), ha='left')
    label(ax, 'IMU top port to GPS J4: 50 mm Qwiic cable (48 mm needed; use a 100 mm\none from the elechawk kit if the 50 mm will not reach)', (10, 27), (R, 24), ha='left')
    label(ax, 'USB-C notch, open at the bezel seam', (60, md.USBC_Y), (R, 6), ha='left')
    label(ax, 'Cable B (dashed): second 150 mm Qwiic-to-header cable, terminal, along the\nbottom and up the right side, through the Qwiic notch to the light sensor', (55, -10), (R, -8), ha='left')
    label(ax, 'Snap windows: the bezel fingers click in here\n(two on top, two underneath)', (35, -41.5), (R, -24), ha='left')
    label(ax, 'Display press pad, one of four', (36, -21), (R, -36), ha='left')
    label(ax, 'Coax notch (10 mm, x = 21) and Qwiic notch (8 mm, x = 40) through the top wall into the visor,\nchevron tops. Both sit BEHIND the bezel pocket wall (z -22 to -13.5), where the tray is open to the floor.',
          (md.COAX_NOTCH_X, 41), (-30, 58), ha='left')
    ax.set_xlim(-150, 150); ax.set_ylim(-62, 62); ax.set_aspect('equal'); ax.axis('off')
    ax.set_title('Mini Dash shell, looking in from the front with the display removed (device frame, mm; X right, Y up)', fontsize=12)
    fig.tight_layout(); fig.savefig(f'{OUT}/mini-dash-layout-1-tray.png', dpi=110); plt.close(fig)


# =====================================================================================
# Figure 2: visor plan view from above (X across, Z toward the driver)
# =====================================================================================
def fig_visor():
    fig, ax = plt.subplots(figsize=(20, 6.2))
    # shell footprint seen from above: tray (z -25..0), bezel (0..-13 overlaps), visor (z -25..30) sits on top of the tray
    rect(ax, -md.SHELL_W / 2, md.SHELL_W / 2, md.SHELL_BACK, 0, fc='#e8f5e9', ec='k', lw=0.6, ls=':')
    rect(ax, -md.SHELL_W / 2, md.SHELL_W / 2, md.SHELL_BACK, md.VISOR_FWD, fc=C_SHELL, ec='k', lw=1.2)
    rect(ax, -cav_x, cav_x, ant_z0, md.VISOR_FWD, fc='#eef7ee', ec='k', lw=0.8)          # cavity (open at the front)
    rect(ax, -cav_x, cav_x, md.VISOR_FWD - 0.01, md.VISOR_FWD + 0.6, fc='#eef7ee', ec='none')  # open front
    # antenna pocket: side ribs and gabled stop rib
    for sx, zend in ((-1, ant_z0 + md.ANT_D + 2), (1, md.SIDE_RIB_GAP_Z)):
        rect(ax, sx * (md.ANT_W / 2 + 1) - 1, sx * (md.ANT_W / 2 + 1) + 1, md.SHELL_BACK, zend, fc=C_SHELL, ec='k', lw=0.8)
    rect(ax, -md.ANT_W / 2 - 2, md.ANT_W / 2 + 2, ant_z0 + md.ANT_D, ant_z0 + md.ANT_D + 2, fc=C_SHELL, ec='k', lw=0.8, hatch='xx')
    # antenna on its pad
    rect(ax, -12.5, 12.5, ant_z0 + 0.5, ant_z0 + 25.5, fc=C_ANT, ec='k', lw=1.0)
    ax.add_patch(Circle((0, ant_z0 + 13), 6, fc='#80cbc4', ec='k', lw=0.5))
    rect(ax, -3, 3, ant_z0 + 20, ant_z0 + 25.5, fc='#555', ec='k', lw=0.5)                   # LNA can / cable exit
    # notch through the floor (seen from above as an opening)
    rect(ax, md.COAX_NOTCH_X - 5, md.COAX_NOTCH_X + 5, md.NOTCH_Z0, md.NOTCH_Z1, fc='white', ec='k', lw=0.8)
    rect(ax, md.QWIIC_NOTCH_X - 4, md.QWIIC_NOTCH_X + 4, md.NOTCH_Z0, md.NOTCH_Z1, fc='white', ec='k', lw=0.8)
    # antenna lead: out of the front-left corner of the pocket (the -X rib stops at z -3), back along the floor, down the notch
    ax.plot([0, 11, 15.5, md.COAX_NOTCH_X, md.COAX_NOTCH_X], [ant_z0 + 25.5, ant_z0 + 24, -1, -10, md.NOTCH_Z1 - 2], color=C_COAX, lw=1.6)
    # cover grooves and cover
    cz0 = md.VISOR_FWD - 3.0
    rect(ax, -cav_x - 0.8, cav_x + 0.8, cz0, cz0 + md.COVER_T + 2 * md.CLR, fc='white', ec='k', lw=0.6)
    rect(ax, -cav_x - 0.8 + md.CLR, md.SHELL_W / 2 - 0.2, cz0 + md.CLR, cz0 + md.CLR + md.COVER_T, fc=C_COVER, ec='k', lw=1.0)
    rect(ax, md.SHELL_W / 2 - 0.2 - 6, md.SHELL_W / 2 - 0.2, cz0 - 1.5, cz0 + md.CLR + md.COVER_T, fc='white', ec='k', lw=0.6)  # finger notch
    rect(ax, md.LIGHT_X - 3.5, md.LIGHT_X + 3.5, cz0 - 0.4, cz0 + 2.0, fc='white', ec='k', lw=0.6)   # lens slot (shown as an opening)
    # entry slot through the +X wall
    rect(ax, cav_x, md.SHELL_W / 2, cz0, cz0 + md.COVER_T + 2 * md.CLR, fc='white', ec='k', lw=0.6)
    # light sensor board (hangs under the roof; seen from above)
    rect(ax, md.LIGHT_X - md.LIGHT_BOARD_L / 2, md.LIGHT_X + md.LIGHT_BOARD_L / 2, lz - md.LIGHT_BOARD_W / 2, lz + md.LIGHT_BOARD_W / 2,
         fc=C_LIGHT, ec='k', lw=1.0, alpha=0.85)
    for dx in (-md.LIGHT_HOLE_PITCH / 2, md.LIGHT_HOLE_PITCH / 2):
        ax.add_patch(Circle((md.LIGHT_X + dx, light_bz), md.BOSS_OD / 2, fc='white', ec='k', lw=0.6))
        ax.add_patch(Circle((md.LIGHT_X + dx, light_bz), 1.3, fc='#333', ec='none'))
        rect(ax, md.LIGHT_X + dx - 3, md.LIGHT_X + dx + 3, md.SHELL_BACK + md.SHELL_T, light_bz, fc=C_SHELL, ec='k', lw=0.5, alpha=0.6)
    rect(ax, md.LIGHT_X - 2, md.LIGHT_X + 2, lz + md.LIGHT_BOARD_W / 2 - 2.5, lz + md.LIGHT_BOARD_W / 2 + 0.5, fc='#333', ec='k', lw=0.5)  # lens
    rect(ax, md.LIGHT_X - md.LIGHT_BOARD_L / 2, md.LIGHT_X - md.LIGHT_BOARD_L / 2 + 6, lz + 3, lz + 9, fc='#fff', ec='k', lw=0.5)   # STEMMA ports
    rect(ax, md.LIGHT_X + md.LIGHT_BOARD_L / 2 - 6, md.LIGHT_X + md.LIGHT_BOARD_L / 2, lz + 3, lz + 9, fc='#fff', ec='k', lw=0.5)
    ax.plot([md.QWIIC_NOTCH_X, md.QWIIC_NOTCH_X, 33, 28.5], [md.NOTCH_Z1 - 2, 4, 10, lz + 6], color=C_CABLE, lw=2, ls=(0, (4, 2)))
    # roof hole note: the roof is above everything here
    ax.annotate('', xy=(-70, 36), xytext=(-70, 28), arrowprops=dict(arrowstyle='->'))
    ax.text(-70, 38, 'toward the driver (+Z)', fontsize=8.5, va='bottom', ha='center')

    L, R = -66, 66
    label(ax, 'GPS patch antenna, 25 x 25 x 8.2 mm, stuck to the visor floor with\nits own 3M pad and boxed in by the back wall, two side ribs and the\nstop rib. Looks up through the 2 mm roof, 0.6 mm below it.',
          (-8, ant_z0 + 8), (L, -12), ha='right')
    label(ax, 'Side ribs, floor to roof; the right one stops at z = -3\nso the lead can leave the pocket at its front corner', (-14, ant_z0 + 10), (L, 2), ha='right')
    label(ax, 'Stop rib, gabled underside so it is not a bridge', (-10, ant_z0 + md.ANT_D + 1), (L, 10), ha='right')
    label(ax, 'Coax notch, 10 x 8.5 (chevron top), behind the bezel wall:\nthe plug and the 30 mm stiff lead are fed down through it\nat an angle, then the lead runs back along the floor', (md.COAX_NOTCH_X - 5, md.NOTCH_Z0 + 4), (L, 20), ha='right')
    label(ax, 'Tray below (dotted outline): display, GPS, IMU', (-50, -20), (L, -26), ha='right')
    label(ax, 'Open front of the visor, closed by the cover', (-30, md.VISOR_FWD + 0.3), (-30, 40), ha='center')
    label(ax, 'Light sensor VEML7700 board, flat under the roof against two bosses\n(each with a web down to the back wall). Two M2 x 12 screws go down\nthrough the roof (counterbored heads on top) and the boss, nut under\nthe board. Lens on its front edge faces the slot in the cover.',
          (md.LIGHT_X + 10, lz), (R, 14), ha='left')
    label(ax, 'Qwiic notch: cable B comes up here, behind the\nbezel wall, between the light sensor boss webs', (md.QWIIC_NOTCH_X + 4, md.NOTCH_Z0 + 4), (R, -2), ha='left')
    label(ax, 'Lens slot, 7 x 3.5, in the cover', (md.LIGHT_X + 3.5, cz0 + 1.5), (R, 40), ha='left')
    label(ax, '1 mm cover slides in from the right along grooves in roof\nand floor, through a slot in the right wall; finger notch to pull it',
          (md.SHELL_W / 2 - 1, cz0 + 1), (R, 28), ha='left')
    label(ax, 'Boss webs down to the back wall', (md.LIGHT_X + md.LIGHT_HOLE_PITCH / 2 + 3, -10), (R, -16), ha='left')
    ax.set_xlim(-135, 135); ax.set_ylim(-34, 44); ax.set_aspect('equal'); ax.axis('off')
    ax.set_title('Visor, plan view from above with the roof removed (X right, Z toward the driver). Windscreen is at the bottom of the picture.', fontsize=12)
    fig.tight_layout(); fig.savefig(f'{OUT}/mini-dash-layout-2-visor.png', dpi=110); plt.close(fig)


# =====================================================================================
# Figure 3: side sections (Y up, Z toward the driver) at four X stations, from the real meshes
# =====================================================================================
def xsec(mesh, x):
    sec = mesh.section(plane_origin=[x, 0, 0], plane_normal=[1, 0, 0])
    if sec is None:
        return shapely.geometry.Polygon()
    p, _ = sec.to_2D(to_2D=trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]) @ trimesh.transformations.translation_matrix([-x, 0, 0]))
    return unary_union(list(p.polygons_full))      # coords come out as (z, y)


def draw_poly(ax, poly, **kw):
    for g in (poly.geoms if hasattr(poly, 'geoms') else [poly]):
        if g.is_empty:
            continue
        zz, yy = g.exterior.xy
        ax.fill(zz, yy, **kw)
        for i in g.interiors:
            zz, yy = i.xy
            ax.fill(zz, yy, color='white')


def fig_sections():
    meshes = {n: trimesh.load(f'{S}/dev/{n}.stl') for n in ('shell', 'bezel', 'cover', 'plate', 'board43c')}
    stations = [(-35, 'x = -35: GPS board, low on 2 mm bosses'), (-10, 'x = -10: SMA plug in the band above the display board'),
                (0, 'x = 0: antenna pocket, terminal block, rail'), (md.COAX_NOTCH_X, f'x = {md.COAX_NOTCH_X:.0f}: coax notch behind the bezel wall, stiff lead'),
                (md.IMU_C[0], f'x = {md.IMU_C[0]:.0f}: IMU under the TF slot, press pad, Qwiic notch'), (52.15, 'x = 52: light sensor boss, roof screw hole, cover slot')]
    fig, axs = plt.subplots(2, 3, figsize=(24, 13))
    for ax, (x, title) in zip(axs.flat, stations):
        draw_poly(ax, xsec(meshes['plate'], x), color=C_PLATE)
        draw_poly(ax, xsec(meshes['shell'], x), color=C_SHELL)
        draw_poly(ax, xsec(meshes['board43c'], x), color='#9e9e9e')
        draw_poly(ax, xsec(meshes['bezel'], x), color='#546e7a')
        draw_poly(ax, xsec(meshes['cover'], x), color='#7cb342')
        if x == -35:
            rect(ax, gps_z, gps_z + 1.6, gy0, gcy, fc=C_GPS, ec='k', lw=0.8)
            rect(ax, gps_z + 1.6, gps_z + 1.6 + 2.4, gy0 + 8.2, gy0 + 24.2, fc='#1e88e5', ec='k', lw=0.5)
            label(ax, 'GPS board on 2 mm bosses,\ncomponents toward the display', (gps_z + 1, gcy - 3), (-44, 60), ha='left')
        if x == -10:
            ax.add_patch(Circle((gps_z + 1.6 + 3.17, JACK_Y), 3.95, fc='#888', ec='k', lw=0.5))
            label(ax, 'SMA plug, 7.9 mm: its top is level with the\ndisplay PCB back, so it runs beside the PCB,\n0.3 mm above its top edge', (gps_z + 1.6 + 3.17, JACK_Y + 4), (-44, 62), ha='left')
        if x == md.COAX_NOTCH_X:
            ax.add_patch(Circle((gps_z + 1.6 + 3.17, JACK_Y), 1.5, fc='#aaa', ec='k', lw=0.5))
            label(ax, 'stiff lead', (gps_z + 1.6 + 3.17, JACK_Y + 1.5), (-44, 60), ha='left')
            label(ax, 'coax notch through the floor,\nbehind the bezel wall', (md.NOTCH_Z0 + 4, floor_y - 1), (8, 40), ha='left')
        if x == 0:
            rect(ax, ant_z0 + 0.5, ant_z0 + 25.5, floor_y, floor_y + 0.9, fc='#333', ec='none')
            rect(ax, ant_z0 + 0.5, ant_z0 + 25.5, floor_y + 0.9, floor_y + 0.9 + 8.2, fc=C_ANT, ec='k', lw=0.8)
            label(ax, 'antenna on its 3M pad,\n0.6 mm under the roof', (ant_z0 + 12, floor_y + 5), (-44, 62), ha='left')
            label(ax, 'screw terminal block, the deepest\npart on the display back (z -17.2)', (-12, -18), (6, -58), ha='left')
            label(ax, 'dovetail rail, flush with the shell back', (-20, y_out - 2.5), (-44, -62), ha='left')
            label(ax, 'gabled stop rib', (6, floor_y + 4.85), (14, 62), ha='left')
        if x == md.IMU_C[0]:
            rect(ax, imu_z, imu_z + 1.57, md.IMU_C[1] - IMU_HALF, md.IMU_C[1] + IMU_HALF, fc=C_IMU, ec='k', lw=0.8)
            for sy in (-1, 1):
                rect(ax, imu_z + 1.57, imu_z + 1.57 + 3.1, md.IMU_C[1] + sy * IMU_HALF, md.IMU_C[1] + sy * (IMU_HALF - 6), fc='#eee', ec='k', lw=0.5)
            label(ax, 'IMU on 2.5 mm bosses, components toward the display,\nQwiic ports (white) on its top and bottom edges', (imu_z + 1, md.IMU_C[1]), (-44, 60), ha='left')
            label(ax, 'TF card slot on the display back', (-13, 15), (-44, -58), ha='left')
            label(ax, 'press pad on bare PCB', (-14, 30), (6, 50), ha='left')
        if x == 52.15:
            rect(ax, lz - md.LIGHT_BOARD_W / 2, lz + md.LIGHT_BOARD_W / 2, roof_y - 4 - 1.57, roof_y - 4, fc=C_LIGHT, ec='k', lw=0.8)
            label(ax, 'light sensor board against the boss ends, 4 mm below the roof,\nheld by M2 x 12 screws down through the roof; lens looks forward',
                  (lz, roof_y - 4.8), (-44, 62), ha='left')
            label(ax, 'screw hole and counterbore through the roof and boss', (light_bz, roof_y + 1), (-44, -58), ha='left')
        ax.axhline(y_out, color='k', lw=0.3, ls=':')
        ax.set_aspect('equal'); ax.set_xlim(-46, 36); ax.set_ylim(-68, 68)
        ax.set_title(title, fontsize=10); ax.set_xlabel('Z, toward the driver (mm)'); ax.set_ylabel('Y, up (mm)')
        ax.grid(True, lw=0.3, alpha=0.5)
    fig.suptitle('Side sections through the assembled device on its plate. Green: shell; dark blue-grey: bezel; grey: display board; pale wedge: adhesive plate', fontsize=12)
    fig.tight_layout(); fig.savefig(f'{OUT}/mini-dash-layout-3-sections.png', dpi=110); plt.close(fig)


# =====================================================================================
# Figure 4: exploded view (VTK offscreen render, annotated in matplotlib)
# =====================================================================================
def fig_exploded():
    import vtk
    ren = vtk.vtkRenderer(); ren.SetBackground(1, 1, 1)

    def hexc(h):
        h = h.lstrip('#'); return tuple(int(h[i:i + 2], 16) / 255 for i in (0, 2, 4))

    def add_stl(path, color, off=(0, 0, 0)):
        r = vtk.vtkSTLReader(); r.SetFileName(path); r.Update()
        m = vtk.vtkPolyDataMapper(); m.SetInputConnection(r.GetOutputPort())
        a = vtk.vtkActor(); a.SetMapper(m); a.GetProperty().SetColor(*color)
        a.GetProperty().SetAmbient(0.3); a.GetProperty().SetDiffuse(0.7); a.SetPosition(*off); ren.AddActor(a)

    def add_box(x0, x1, y0, y1, z0, z1, color, off=(0, 0, 0)):
        c = vtk.vtkCubeSource(); c.SetBounds(min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1), min(z0, z1), max(z0, z1))
        m = vtk.vtkPolyDataMapper(); m.SetInputConnection(c.GetOutputPort())
        a = vtk.vtkActor(); a.SetMapper(m); a.GetProperty().SetColor(*color); a.GetProperty().SetAmbient(0.3)
        a.SetPosition(*off); ren.AddActor(a)

    EXP_BEZ, EXP_BOARD, EXP_SENS, EXP_COVER, EXP_PLATE, EXP_ANT, EXP_LIGHT = 130, 80, 22, 60, -45, 30, 75
    add_stl(f'{S}/dev/shell.stl', hexc('#81c784'))
    add_stl(f'{S}/dev/bezel.stl', hexc('#90a4ae'), (0, 0, EXP_BEZ))
    add_stl(f'{S}/dev/board43c.stl', hexc('#616161'), (0, 0, EXP_BOARD))
    add_stl(f'{S}/dev/cover.stl', hexc('#558b2f'), (EXP_COVER, 0, 0))
    add_stl(f'{S}/dev/plate.stl', hexc('#b0bec5'), (0, EXP_PLATE, 0))
    add_box(gx0, gcx, gy0, gcy, gps_z, gps_z + 1.6, hexc('#42a5f5'), (0, 0, EXP_SENS))
    add_box(gx0 + 14.2, gx0 + 26.4, gy0 + 8.2, gy0 + 24.2, gps_z + 1.6, gps_z + 4.0, hexc('#1565c0'), (0, 0, EXP_SENS))
    add_box(gcx - 9.5, gcx + 8, JACK_Y - 3.2, JACK_Y + 3.2, gps_z + 1.6, gps_z + 8.0, hexc('#333333'), (0, 0, EXP_SENS))
    ix, iy = md.IMU_C
    add_box(ix - IMU_HALF, ix + IMU_HALF, iy - IMU_HALF, iy + IMU_HALF, imu_z, imu_z + 1.57, hexc('#ffa726'), (0, 0, EXP_SENS))
    for sy in (-1, 1):
        add_box(ix - 3, ix + 3, iy + sy * IMU_HALF, iy + sy * (IMU_HALF - 7.5), imu_z + 1.57, imu_z + 1.57 + 3.1, hexc('#eeeeee'), (0, 0, EXP_SENS))
    add_box(-12.5, 12.5, floor_y, floor_y + 0.9, ant_z0 + 0.5, ant_z0 + 25.5, hexc('#333333'), (0, EXP_ANT, 0))
    add_box(-12.5, 12.5, floor_y + 0.9, floor_y + 9.1, ant_z0 + 0.5, ant_z0 + 25.5, hexc('#26a69a'), (0, EXP_ANT, 0))
    add_box(md.LIGHT_X - md.LIGHT_BOARD_L / 2, md.LIGHT_X + md.LIGHT_BOARD_L / 2, roof_y - 4 - 1.57, roof_y - 4,
            lz - md.LIGHT_BOARD_W / 2, lz + md.LIGHT_BOARD_W / 2, hexc('#ab47bc'), (0, 0, EXP_LIGHT))
    W, H = 2000, 1300
    win = vtk.vtkRenderWindow(); win.SetOffScreenRendering(1); win.AddRenderer(ren); win.SetSize(W, H)
    cam = ren.GetActiveCamera(); cam.SetFocalPoint(0, 10, 40); cam.SetPosition(420, 200, 300); cam.SetViewUp(0, 1, 0)
    ren.ResetCamera(); cam.Zoom(1.25); win.Render()
    w2i = vtk.vtkWindowToImageFilter(); w2i.SetInput(win); w2i.Update()
    pw = vtk.vtkPNGWriter(); pw.SetFileName(f'{S}/exploded_raw.png'); pw.SetInputConnection(w2i.GetOutputPort()); pw.Write()
    anchors = {'bezel': (0, 45, EXP_BEZ - 6), 'display': (0, 40, EXP_BOARD - 3), 'shell': (62, -30, -14),
               'cover': (EXP_COVER + 50, 47, 28), 'plate': (40, EXP_PLATE - 50, -20), 'gps': (gx0 + 20, gcy - 6, gps_z + EXP_SENS),
               'imu': (ix, iy - 10, imu_z + EXP_SENS), 'antenna': (0, floor_y + 9 + EXP_ANT, ant_z0 + 12),
               'light': (md.LIGHT_X, roof_y - 4, lz + EXP_LIGHT), 'rail': (0, y_out - 2, -20)}
    pts = {}
    for k, (x, y, z) in anchors.items():
        ren.SetWorldPoint(x, y, z, 1.0); ren.WorldToDisplay(); dx, dy, _ = ren.GetDisplayPoint(); pts[k] = (dx, H - dy)

    notes = {
        'bezel': ('1. Bezel (printed face down). The display drops into its pocket from behind;\nfour cantilever fingers on its top and bottom walls.', (60, 150)),
        'display': ('2. Waveshare 4.3C display. Not screwed: trapped between the bezel lip\nand four press pads on bare PCB when the shell snaps on.', (60, 420)),
        'shell': ('3. Shell (printed back down) slides over the bezel from behind and clicks\nonto the fingers. Carries every sensor.', (1250, 820)),
        'gps': ('4. GPS NEO-M9N board, left half: four M2 x 3 heat-set inserts in 2 mm bosses,\ncomponents toward the display, SMA jack pointing right along the top.', (1250, 150)),
        'imu': ('5. IMU ISM330DHCX, right, ports up and down: four M2 x 3 inserts\nin 2.5 mm bosses, components toward the display.', (1250, 560)),
        'antenna': ('6. GPS patch antenna: 3M pad onto the visor floor, boxed by ribs. Its lead leaves\nthe pocket at the front-right corner and drops through the coax notch to the SMA jack.', (1250, 30)),
        'light': ('7. Light sensor: two M2 x 12 screws down through the roof and bosses,\nnuts under the board, lens facing the cover slot.', (60, 620)),
        'cover': ('8. Visor cover slides in from the right along grooves in the\nroof and floor, after the antenna and light sensor are in.', (1250, 430)),
        'plate': ('9. Adhesive plate (3M VHB underneath) with the dovetail groove.\nThe device slides on from the left until the rail hits the groove end.', (60, 1000)),
        'rail': ('Dovetail rail under the shell', (900, 1230)),
    }
    img = mpimg.imread(f'{S}/exploded_raw.png')
    fig, ax = plt.subplots(figsize=(20, 13)); ax.imshow(img); ax.axis('off')
    for k, (text, (tx, ty)) in notes.items():
        ax.annotate(text, xy=pts[k], xytext=(tx, ty), fontsize=11, ha='left', va='center',
                    arrowprops=dict(arrowstyle='-', color='#333', lw=0.9),
                    bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='#999', lw=0.5))
    ax.set_title('Mini Dash exploded view, numbered in assembly order (device frame: screen faces the viewer, windscreen is behind)', fontsize=13)
    fig.tight_layout(); fig.savefig(f'{OUT}/mini-dash-layout-4-exploded.png', dpi=100); plt.close(fig)


if __name__ == '__main__':
    fig_tray(); fig_visor(); fig_sections(); fig_exploded()
    print('done')
