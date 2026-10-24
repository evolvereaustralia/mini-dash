# Design

The UI specification lives in CLAUDE.md section 4; this folder holds the design sources and the pictures behind it.

- `mini-dash-ui.html`: the merged interactive prototype (24 Oct 2026), fixed at 800 x 480, one CSS pixel per panel pixel, synthetic data. Open it in a browser: tabs, header (Settings), Trip scopes, tiles, Journey totals and the Drive trip-best field all work, and swiping left or right changes tab. Query parameters pick a state for rendering, listed in the file header.
- `assets/`: `mini-wings-source.png` is the emblem Logan supplied (600 x 600 on white); `mini-wings.png` (420 px wide) and `mini-wings-560.png` are the transparent cut-outs the splash uses and the firmware will convert to an LVGL image.
- `renders/merged/`: every state of the merged prototype at true panel size. Regenerate with the command below after editing the prototype.
- `handoff/`: the telemetry design handoff Logan approved on 24 Oct 2026 (implementation brief, original fragment, standalone preview). The merged prototype supersedes it as the visual reference; the brief remains useful for its record rules, validity rules and OBD research notes.
- `renders/handoff/`: the handoff prototype at true panel size, for comparison.

Render (Google Chrome, headless):

```bash
cd docs/design && for s in "0-splash screen=Splash" "1-drive-corner screen=Drive&scene=Corner&obd=1" "2-trip-records screen=Trip&scope=trip"; do set -- $s; "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" --headless=new --disable-gpu --hide-scrollbars --window-size=800,480 --screenshot="renders/merged/$1.png" "file://$PWD/mini-dash-ui.html?$2"; done
```

Legibility rule of thumb used throughout: the panel is 93.7 mm wide, so a pixel is 0.117 mm. At about 75 cm from the driver's eyes, text needs roughly 3.5 mm capitals (a 40 px font) to be read at a glance. Everything smaller is orientation or parked reading.
