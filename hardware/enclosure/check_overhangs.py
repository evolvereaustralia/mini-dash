"""
Layer-by-layer printability check for the Mini Dash STLs (what PrusaSlicer does, without PrusaSlicer).

Slices each mesh at LAYER intervals in its print orientation and reports any part of a layer that does not sit on
the layer below once that layer is grown by the allowed overhang step (0.2 mm at 45 degrees). Each report is one
contiguous unsupported region: bounds, area, and the z range over which it persists.

What is acceptable: the short two-sided bridges listed in ACCEPTED below (snap windows, vent slot tops), judged by
size. Anything else means the model has to change, and the script then exits non-zero. Run before every STL release:

    python3 -I check_overhangs.py export/mini-dash-*.stl
"""
import sys
import trimesh
import shapely
from shapely.ops import unary_union

LAYER = 0.2
STEP = LAYER * 1.05          # 45 degrees: each layer may extend one layer height past the one below
MIN_AREA = 0.5               # mm2, ignore tessellation noise

# The bridges the design accepts, by size (longer side, shorter side, mm) and by how many
# layers they may persist: a real bridge is one layer, then the layers above sit on it. A
# region is accepted only if it fits one of these; anything else fails the check.
ACCEPTED = [                 # (name, max longer side, max shorter side, max layers, reason)
    ("snap window", 8.6, 2.3, 1, "8 mm bridge, anchored both ends"),
    ("vent slot top", 2.4, 1.7, 1, "1.8 mm bridge"),
]


def accepted(w, h, layers):
    long_side, short_side = max(w, h), min(w, h)
    for name, max_long, max_short, max_layers, _ in ACCEPTED:
        if long_side <= max_long and short_side <= max_short and layers <= max_layers:
            return name
    return None


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
    rejected = 0
    lines = []
    for b, zs in sorted(groups.items(), key=lambda kv: kv[1][0][0]):
        w, h = b[2] - b[0], b[3] - b[1]
        name = accepted(w, h, len(zs))
        if name is None:
            rejected += 1
        lines.append(f"  {'ok  ' if name else 'FAIL'} z {zs[0][0]:5.1f}..{zs[-1][0]:5.1f}  {w:5.1f} x {h:5.1f} mm  area {max(a for _, a in zs):6.1f}  "
                     f"at x {b[0]}..{b[2]}, y {b[1]}..{b[3]}  {name or 'NOT an accepted bridge'}")
    print(f"\n{path}: {len(groups)} unsupported region(s), {rejected} not accepted")
    print("\n".join(lines)) if lines else None
    return rejected


if __name__ == "__main__":
    # Exit status: 0 only if every file was checked and nothing but accepted bridges was found;
    # 1 if any region is not an accepted bridge; 2 if nothing was checked or a file could not
    # be read. Until 24 Nov 2026 this always exited 0, so a script or a tired eye could take a
    # failing part for a passing one (review finding 42).
    if len(sys.argv) < 2:
        print("usage: python3 -I check_overhangs.py export/mini-dash-*.stl")
        sys.exit(2)
    bad, unread = 0, 0
    for p in sys.argv[1:]:
        try:
            bad += check(p)
        except Exception as e:  # a missing or broken STL is a failure, not a skip
            print(f"\n{p}: COULD NOT BE CHECKED ({e})")
            unread += 1
    print("\nRESULT:", "FAIL" if (bad or unread) else "CLEAN",
          f"({bad} region(s) not accepted, {unread} file(s) unreadable, {len(sys.argv) - 1} file(s) given)")
    sys.exit(2 if unread else (1 if bad else 0))
