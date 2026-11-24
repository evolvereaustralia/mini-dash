#!/usr/bin/env python3
# Split a drive log into its capture records and say whether the log is complete.
#
#   python3 tools/capture_parse.py ../docs/drive-logs/drive-2026-11-28-101500.log
#
# The device's capture stream (src/app/capture.h) is text lines beginning with '~' among the
# ordinary serial output. This writes one CSV per record type beside the log, in a folder named
# after it, and prints a summary: how many of each record, how long the stream ran, how many
# records the device dropped, how regular the GPS epochs and IMU samples were, every OBD
# identifier's rate and timing, the timer runs the engine judged, and the least free stack of
# each task. Standard library only.
import csv, os, sys
from collections import Counter, defaultdict

FIELDS = {
    "G": ["sats", "flags", "itow_ms", "speed_mm_s", "h_acc_mm", "alt_cm", "speed_acc_mm_s", "heading_cdeg"],
    "I": ["a", "b", "ax_ug", "ay_ug", "az_ug", "gx_dmdps", "gy_dmdps", "gz_dmdps"],
    "A": ["flags", "gyro_norm_ddps", "lon_ug", "lat_ug", "yaw_mdps", "fused_mm_s", "centripetal_ug", "acc_norm_ug"],
    "F": ["a", "b", "lat_lp_ug", "lon_lp_ug", "tilt_cdeg", "rest_norm_ug", "gyro_bias_mdps", "lon_bias_ug"],
    "S": ["flags", "sats", "shown_m_h", "gps_m_h", "wheels_m_h", "power_w", "soc_cpct", "trip_cm"],
    "C": ["record", "b", "value_e3", "aux_e3", "engine_ms", "v3", "v4", "v5"],
    "O": ["ecu", "did", "duration_ms", "outcome", "frames", "link_state", "v4", "v5"],
    "B": ["state", "b", "v0", "v1", "v2", "v3", "v4", "v5"],
    "T": ["task", "b", "stack_free", "heap_free", "heap_min", "psram_free", "v4", "v5"],
    "D": ["a", "b", "dropped", "v1", "v2", "v3", "v4", "v5"],
}
RECORDS = ["0-60", "0-100", "60-0", "lateral", "draw", "regen"]
OUTCOMES = ["good", "unanswered", "silent", "refused", "bad"]
TASKS = ["loop", "sensors", "OBD", "display"]


def parse(path):
    """Returns (records by type, config lines, malformed count). Each record is [t_seconds, a, b, v0..v5]."""
    recs, config, bad = defaultdict(list), [], 0
    # The device stamps records with a 32-bit microsecond clock that wraps every 71.6 minutes, and
    # records from different tasks reach the log slightly out of order. Each stamp is therefore
    # placed by its signed 32-bit distance from the latest time seen so far, which is right for
    # anything within 35 minutes either side of it, wrap or no wrap.
    latest = None  # unwrapped microseconds
    with open(path, "rb") as f:
        for raw in f:
            i = raw.find(b"~")
            if i < 0:
                continue
            line = raw[i:].decode("ascii", "replace").strip()
            if line.startswith("~K,"):
                config.append(line)
                continue
            p = line.split(",")
            if len(p) != 10 or len(p[0]) != 2 or p[0][1] not in FIELDS:
                bad += 1  # cut short (the last line of a log usually is) or run into another line
                continue
            try:
                n = [int(x) for x in p[1:]]
            except ValueError:
                bad += 1
                continue
            if not 0 <= n[0] < 2**32:
                bad += 1
                continue
            if latest is None:
                t = n[0]
            else:
                d = (n[0] - latest) % 2**32
                if d >= 2**31:
                    d -= 2**32
                t = latest + d
            if latest is None or t > latest:
                latest = t
            recs[p[0][1]].append([t / 1e6] + n[1:])
    return recs, config, bad


def summary(recs, config, bad, out):
    def p(*a):
        print(*a, file=out)

    total = sum(len(v) for v in recs.values())
    times = [r[0] for v in recs.values() for r in v]
    span = (max(times) - min(times)) if times else 0
    p(f"capture records: {total} over {span:.1f} s; {bad} malformed line(s) skipped")
    for t in sorted(recs):
        p(f"  {t}: {len(recs[t])}")
    dropped = recs["D"][-1][3] if recs.get("D") else 0
    p(f"dropped by the device (ring full): {dropped}" + ("  <-- THE LOG IS INCOMPLETE" if dropped else ""))
    for k in config[:1] + config[-1:] if len(config) > 1 else config:
        p("config:", k)

    g = recs.get("G", [])
    if len(g) > 1:
        steps = Counter()
        repeats = sum(1 for r in g if r[2] & 0x40)
        fixed = [r for r in g if r[2] & 0x10]
        for a, b in zip(g, g[1:]):
            if not (b[2] & 0x40):
                steps[(b[3] - a[3]) % 604800000] += 1
        p(f"GPS: {len(g)} PVT read, {len(fixed)} with a valid 3D fix, {repeats} repeated epochs")
        p("  iTOW step between successive reads (ms: count): " + ", ".join(f"{k}: {v}" for k, v in sorted(steps.items())[:8]))
        if fixed:
            # arrival time minus epoch, relative to its smallest value: the jitter the fusion has to ride out
            d = [r[0] * 1000.0 - r[3] for r in fixed if not r[2] & 0x40]  # a repeated epoch is a re-read, not an arrival
            base = min(d)
            late = sorted(x - base for x in d)
            p(f"  arrival delay over the earliest: median {late[len(late) // 2]:.1f} ms, 95th percentile {late[int(len(late) * 0.95)]:.1f} ms, max {late[-1]:.1f} ms")
            p(f"  satellites {min(r[1] for r in fixed)}..{max(r[1] for r in fixed)}, speed accuracy up to {max(r[7] for r in fixed)} mm/s, top speed {max(r[4] for r in fixed) * 3.6 / 1000:.1f} km/h")
    im = recs.get("I", [])
    if len(im) > 1:
        gaps = [b[0] - a[0] for a, b in zip(im, im[1:])]
        dur = im[-1][0] - im[0][0]
        p(f"IMU: {len(im)} samples, {(len(im) - 1) / dur if dur else 0:.1f} Hz mean, longest gap {max(gaps) * 1000:.1f} ms, {sum(1 for x in gaps if x > 0.012)} gaps over 12 ms")
    a = recs.get("A", [])
    if a:
        n = len(a)
        flag = lambda bit: 100.0 * sum(1 for r in a if r[1] & bit) / n
        p(f"attitude: accelerometer trusted {flag(1):.0f} %, at rest {flag(2):.0f} %, held by the turn gate {flag(4):.0f} %, bias learning {flag(8):.0f} %, forward axis usable {flag(64):.0f} %")
        p(f"  centripetal up to {max(r[7] for r in a) / 1e6:.3f} g; lateral {min(r[4] for r in a) / 1e6:+.2f}..{max(r[4] for r in a) / 1e6:+.2f} g; longitudinal {min(r[3] for r in a) / 1e6:+.2f}..{max(r[3] for r in a) / 1e6:+.2f} g")
    s = recs.get("S", [])
    if s:
        both = [r for r in s if (r[1] & 3) == 3]
        p(f"engine feed: {len(s)} updates; wheels fresh in {100.0 * sum(1 for r in s if r[1] & 2) / len(s):.0f} %, car power fresh in {100.0 * sum(1 for r in s if r[1] & 4) / len(s):.0f} %; trip {s[-1][8] / 100000:.2f} km")
        if both:
            d = sorted((r[5] - r[4]) / 1000.0 for r in both if r[4] > 20000)
            if d:
                p(f"  wheels minus GPS above 20 km/h: median {d[len(d) // 2]:+.2f} km/h, 5th..95th percentile {d[int(len(d) * 0.05)]:+.2f}..{d[int(len(d) * 0.95)]:+.2f} km/h")
    o = recs.get("O", [])
    if o:
        dur = o[-1][0] - o[0][0]
        p(f"OBD: {len(o)} exchanges in {dur:.0f} s")
        by = defaultdict(list)
        for r in o:
            by[(r[1], r[2])].append(r)
        for (ecu, did), rs in sorted(by.items()):
            good = [r for r in rs if r[4] == 0]
            outs = Counter(OUTCOMES[r[4]] if 0 <= r[4] < 5 else "?" for r in rs)
            ms = sorted(r[3] for r in rs)
            p(f"  ECU {ecu:02X} {did:04X}: {len(rs)} asked, {len(good) / dur if dur else 0:.2f} Hz good, {ms[len(ms) // 2]} ms median, {ms[-1]} ms max; " + ", ".join(f"{k} {v}" for k, v in outs.items()))
    for r in recs.get("B", []):
        p(f"OBD link state {r[1]} at {r[0]:.1f} s")
    for r in recs.get("C", []):
        name = RECORDS[r[1]] if 0 <= r[1] < len(RECORDS) else str(r[1])
        p(f"candidate {name}: {r[3] / 1000:.3f} (aux {r[4] / 1000:.3f}) at {r[0]:.1f} s")
    marks = defaultdict(list)
    for r in recs.get("T", []):
        marks[r[1]].append(r)
    for task, rs in sorted(marks.items()):
        name = TASKS[task] if task < len(TASKS) else str(task)
        p(f"stack, {name} task: least free {min(r[3] for r in rs)} bytes; heap least free {min(r[5] for r in rs)} bytes")
    return dropped


def write_csvs(recs, folder):
    os.makedirs(folder, exist_ok=True)
    for t, rows in recs.items():
        with open(os.path.join(folder, f"{t}.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s"] + FIELDS[t])
            for r in rows:
                w.writerow([f"{r[0]:.6f}"] + r[1:])


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit("usage: capture_parse.py <drive log> [output folder]")
    log = sys.argv[1]
    recs, config, bad = parse(log)
    if not sum(len(v) for v in recs.values()):
        sys.exit(f"{log}: no capture records (was the stream started with key C?)")
    folder = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(log)[0] + "-capture"
    write_csvs(recs, folder)
    with open(os.path.join(folder, "summary.txt"), "w") as f:
        summary(recs, config, bad, f)
    dropped = summary(recs, config, bad, sys.stdout)
    print(f"CSV files and summary.txt in {folder}")
    sys.exit(3 if dropped else 0)
