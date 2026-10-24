"""
Layer-by-layer printability check for the Mini Dash STLs (what PrusaSlicer does, without PrusaSlicer).

Slices each mesh at LAYER intervals in its print orientation and reports any part of a layer that does not sit on
the layer below once that layer is grown by the allowed overhang step (0.2 mm at 45 degrees). Each report is one
contiguous unsupported region: bounds, area, and the z range over which it persists.

What is acceptable: two-sided bridges of about 10 mm or less (snap windows, antenna stop rib, vent slots), listed
in ACCEPTED below. Anything else means the model has to change. Run before every STL release:

    python3 -I check_overhangs.py export/mini-dash-*.stl
"""
import sys
import trimesh
import shapely
from shapely.ops import unary_union

LAYER = 0.2
STEP = LAYER * 1.05          # 45 degrees: each layer may extend one layer height past the one below
MIN_AREA = 0.5               # mm2, ignore tessellation noise

ACCEPTED = {                 # name: (approx size, reason)
    "snap window": "8 mm bridge, anchored both ends",
    "stop rib": "9.7 mm bridge floor to roof, anchored four sides",
    "vent slot": "1.8 mm bridge",
}


def layer(mesh, z):
    sec = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    if sec is None:
        return shapely.geometry.Polygon()
    p, _ = sec.to_2D(to_2D=trimesh.transformations.translation_matrix([0, 0, -z]))
    return unary_union(list(p.polygons_full))


def check(path):
    m = trimesh.load(path)
    zmax = m.bounds[1][2]
    prev = layer(m, LAYER / 2)
    z = LAYER * 1.5
    found = []
    while z < zmax:
        cur = layer(m, z)
        d = cur.difference(prev.buffer(STEP))
        for g in (d.geoms if hasattr(d, "geoms") else [d]):
            if g.area > MIN_AREA:
                b = g.bounds
                found.append((round(z, 1), round(g.area, 1), tuple(round(x, 1) for x in b)))
        prev = cur
        z += LAYER
    # group by bounds so a region that persists over several layers is one line
    groups = {}
    for z, a, b in found:
        groups.setdefault(b, []).append((z, a))
    print(f"\n{path}: {len(groups)} unsupported region(s)")
    for b, zs in sorted(groups.items(), key=lambda kv: kv[1][0][0]):
        w, h = b[2] - b[0], b[3] - b[1]
        print(f"  z {zs[0][0]:5.1f}..{zs[-1][0]:5.1f}  {w:5.1f} x {h:5.1f} mm  area {max(a for _, a in zs):6.1f}  at x {b[0]}..{b[2]}, y {b[1]}..{b[3]}")
    return groups


if __name__ == "__main__":
    for p in sys.argv[1:]:
        check(p)
