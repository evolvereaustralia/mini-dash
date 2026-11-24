"""Clearance check of the shell layout against the display-back parts measured from the photo (device frame).
Run: python3 check_layout.py   (after any change to the layout constants)"""
import math
import mini_dash_enclosure as md
import display_back_parts as dbp

Z_PCB = md.PCB_BACK_Z
parts = []
for (n, x0, x1, y0, y1, h) in dbp.PARTS:
    if x0 is None:
        for (hx, hy) in md.PCB_HOLES:
            parts.append((n, hx - 2.0, hx + 2.0, hy - 2.0, hy + 2.0, Z_PCB - h))
    else:
        parts.append((n, x0, x1, y0, y1, Z_PCB - h))
parts.append(('PCB', *dbp.PCB, Z_PCB))
WX, WY = md.SHELL_IN_W / 2, md.SHELL_IN_H / 2

def ov(a, b):
    return a[0] < b[1] and b[0] < a[1] and a[2] < b[3] and b[2] < a[3]

env = []   # (name, x0, x1, y0, y1, z_top, touches)  touches = allowed to meet the PCB/heads
for (x, y) in md.PCB_PRESS:
    env.append(('press pad', x - md.PCB_PRESS_R, x + md.PCB_PRESS_R, y - md.PCB_PRESS_R, y + md.PCB_PRESS_R, Z_PCB - md.SCREW_HEAD_H, True))
cx, cy = md.GPS_CORNER
gz = md.SHELL_IN_BACK + md.GPS_BOSS_H + 1.6          # GPS board top (component side)
env.append(('GPS board + module', cx - 40.64, cx, cy - 36.8, cy, gz + 2.4, False))
jx, jy = cx - 40.41, cy - 25.65
env.append(('SMA jack', jx - 3.2, jx + 3.2, jy - 3.2, jy + 3.2, gz + 6.35, False))
env.append(('SMA plug + stiff lead (hex 9.1)', jx - 50, jx, jy - 4.55, jy + 4.55, gz + 3.17 + 4.55, False))
env.append(('GPS J4 plug', cx - 35.69 - 8, cx - 35.69, cy - 15.6 - 3.5, cy - 15.6 + 3.5, gz + 4.0, False))
env.append(('GPS J3 USB-C', cx - 10.5 - 4.5, cx - 10.5 + 4.5, cy - 36.8, cy - 36.8 + 7.5, gz + 3.2, False))
for (hx, hy) in md.GPS_HOLES:
    env.append(('GPS screw head', cx - hx - 1.9, cx - hx + 1.9, cy - hy - 1.9, cy - hy + 1.9, gz + 1.3, False))
ix, iy = md.IMU_C
iz = md.SHELL_IN_BACK + md.IMU_BOSS_H + 1.57
env.append(('IMU board', ix - 12.7, ix + 12.7, iy - 12.7, iy + 12.7, iz + 1.0, False))
for sx in (-1, 1):
    env.append(('IMU Qwiic plug', ix + sx * 12.7 + (0 if sx > 0 else -8), ix + sx * 12.7 + (8 if sx > 0 else 0), iy - 3.5, iy + 3.5, iz + 4.0, False))
    for sy in (-1, 1):
        env.append(('IMU screw head', ix + sx * 10.16 - 1.9, ix + sx * 10.16 + 1.9, iy + sy * 10.16 - 1.9, iy + sy * 10.16 + 1.9, iz + 1.3, False))

bad = 0
worst = []
for (n, x0, x1, y0, y1, zt, touches) in env:
    if x0 < -WX or x1 > WX or y0 < -WY or y1 > WY:
        print(f'WALL   {n}: x {x0:.1f}..{x1:.1f} y {y0:.1f}..{y1:.1f}'); bad += 1
    for (pn, px0, px1, py0, py1, pz) in parts:
        if ov((x0, x1, y0, y1), (px0, px1, py0, py1)):
            gap = pz - zt
            if touches and pn in ('PCB', 'screw heads (4)'):
                continue
            if gap < 0.5:
                print(f'CLASH  {n} under {pn}: gap {gap:.2f} mm'); bad += 1
            else:
                worst.append((gap, n, pn))
# shell features against each other
names = [e[0] for e in env]
for i in range(len(env)):
    for j in range(i + 1, len(env)):
        a, b = env[i], env[j]
        if a[0].split()[0] == b[0].split()[0] and a[0].split()[0] in ('GPS', 'IMU', 'SMA'):
            continue
        if a[0].startswith(('GPS', 'SMA')) and b[0].startswith(('GPS', 'SMA')):
            continue
        if ov(a[1:5], b[1:5]):
            print(f'OVERLAP {a[0]} / {b[0]}'); bad += 1
worst.sort()
print('tightest z gaps:', ', '.join(f'{g:.1f} mm {n} under {p}' for g, n, p in worst[:5]))
print('RESULT:', 'FAIL' if bad else 'CLEAN', f'(DEPTH {md.DEPTH}, plug top {gz + 3.17 + 4.55 - Z_PCB:.1f} mm behind the PCB)')
