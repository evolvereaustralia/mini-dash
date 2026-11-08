"""
Pad test coupon: the shell's four display press pads on a thin back plate, with a short post at each
corner whose top sits where the GLASS FRONT should be (z -2, behind the bezel plate). Drop the display
in on its own, press it onto the pads, and the glass front should be flush with the four post tops.
Proud = pads too tall; recessed = pads too short. Prints back down in about half an hour.
Run: python3 pad_coupon.py export/
"""
import sys
import cadquery as cq
import mini_dash_enclosure as md

PLATE_T = 1.2
z_plate = md.SHELL_IN_BACK - PLATE_T
c = md.box(-md.SHELL_IN_W / 2, md.SHELL_IN_W / 2, -md.SHELL_IN_H / 2, md.SHELL_IN_H / 2, z_plate, md.SHELL_IN_BACK)
for (x, y) in md.PCB_PRESS:
    c = c.union(cq.Workplane("XY").workplane(offset=md.SHELL_IN_BACK).center(x, y).circle(md.PCB_PRESS_R)
                .extrude(md.PCB_BACK_Z - md.SHELL_IN_BACK))
# corner posts: L-shaped, 2 mm thick, 12 mm legs, top at the glass front (z = -BEZ_T)
for sx in (1, -1):
    for sy in (1, -1):
        x0, y0 = sx * md.SHELL_IN_W / 2, sy * md.SHELL_IN_H / 2
        c = c.union(md.box(x0, x0 - sx * 12, y0, y0 - sy * 2, md.SHELL_IN_BACK, -md.BEZ_T))
        c = c.union(md.box(x0, x0 - sx * 2, y0, y0 - sy * 12, md.SHELL_IN_BACK, -md.BEZ_T))
out = sys.argv[1] if len(sys.argv) > 1 else "export/"
cq.exporters.export(c.translate((0, 0, -z_plate)), f"{out}/mini-dash-pad-coupon.stl", tolerance=0.02, angularTolerance=0.1)
bb = c.val().BoundingBox()
print(f"pad coupon: {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm; pads {md.PCB_BACK_Z - md.SHELL_IN_BACK:.1f} tall, posts {-md.BEZ_T - md.SHELL_IN_BACK:.1f} tall")
