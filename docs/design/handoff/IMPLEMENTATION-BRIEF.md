# MINI Cooper SE telemetry display — implementation handoff

Prepared 24 October 2026. This document is self-contained; the implementing model does not need the preceding chat.

## 1. Assignment and approved direction

Build an embedded telemetry display for an Australian, right-hand-drive **2022 MINI Cooper SE (F56 electric)**. The user wants an equal mix of everyday EV efficiency and performance telemetry, with sophisticated, beautiful visuals. They explicitly approved the supplied interactive design, including the newly added Trip records screen.

Preserve the design direction and information hierarchy. Implement it on the specified hardware; do not replace it with a generic grid of OBD gauges. Start by inspecting any existing firmware repository and its instructions. No firmware repository was supplied or modified during design.

The product answers three questions:

- Drive: what is the car doing now?
- Dynamics/Energy/Battery/Charge: what explains that behaviour?
- Trip: what were the highlights, and how do they compare with previous sessions?

### Hardware confirmed by the user

| Component | Confirmed selection |
|---|---|
| Display/controller | Waveshare **ESP32-S3-Touch-LCD-4.3C**, 800 × 480 capacitive touchscreen |
| Motion sensor | SparkFun **6DoF IMU Breakout — ISM330DHCX** |
| OBD adapter | Vgate **vLinker BM+** |
| Vehicle | Australian 2022 MINI Cooper SE, F56 |
| Position/speed | GPS receiver available/planned; exact module, connection and update rate not specified |

Use documentation for the **4.3C**, not the similarly named 4.3 board. Waveshare documents 8 MB PSRAM, 16 MB flash, an SD/TF slot and LVGL support. Pin mappings and peripheral availability must be checked against the exact board revision.

## 2. Handoff files and maturity

- `IMPLEMENTATION-BRIEF.md`: requirements, architecture, signal research, validation and acceptance criteria.
- `design-preview.html`: standalone browser preview of the approved design, with interactive navigation and synthetic data.
- `design-fragment.html`: editable original HTML/CSS/JavaScript reference, independent of the standalone wrapper.

The preview is **a design prototype, not firmware**. It has no Bluetooth, GPS, IMU or vehicle connection. Its data, event curves, timestamps, records, energy figures and battery readings are invented examples. The source contains browser/SVG constructs that need to be translated to LVGL or another suitable embedded renderer, not executed as the product UI.

The prototype starts on Trip to expose the latest addition. The actual product should normally start on **Drive**, with an optional last-screen preference. Scope switches, record details, journey totals, screen navigation and the Drive trip-best selector work in the prototype. They do not implement real recording, persistent lifetime records or validation of runs.

### Prototype details that must not become production requirements

- Repeated/hardcoded detail values across time scopes are illustrative, not a coherent telemetry dataset.
- Synthetic event speed, acceleration and power curves are not physically synchronized measurements.
- Browser state persistence is not a substitute for device storage.
- HTML reflow on narrow chat windows is for preview usability. The physical display remains fixed at 800 × 480.
- Some prototype annotations are too small for a 4.3-inch panel viewed at driving distance. Preserve hierarchy but verify actual readability.
- The charging example draws a continuous curve. Real missing samples must create visible gaps.
- A record comparison must name its baseline: last trip, previous lifetime record, or another explicit comparison. Do not imply every improved trip metric is a lifetime personal best.

## 3. Visual system and navigation

Aim for a polished motorsport instrument with EV-specific intelligence. Use graphite surfaces, warm white numerals, lime for propulsion and selected emphasis, cyan for regeneration and secondary motion traces. Avoid excessive glow, decoration, spinning gauges or constantly moving backgrounds.

Reference dark palette: background `#0B1114`, raised surface `#121C21`, foreground `#EDF4EE`, secondary text `#99ACB4`, structural lines `#2B3A42`, lime `#C7F46B`, cyan `#64D9E0`. The reference also contains a light palette. Production day/night mode should be explicit or use a verified ambient/daylight source; do not assume an ambient light sensor exists.

### Layout at 800 × 480

- Header: approximately 50 px, product identity and concise connection/status information.
- Main content: approximately 366 px.
- Bottom navigation: approximately 64 px.
- Six persistent tabs, in order: **Drive · Trip · Dynamics · Energy · Battery · Charge**.
- Keep connection loss/invalid measurement information visible without displacing navigation.
- Native controls must have generous touch targets. Six bottom tabs are approximately 133 px wide each.
- Use consistent numeric alignment and tabular numerals. Main figures roughly 48–66 px, record figures 32–36 px, supporting values 18–24 px. Prefer 14–16 px labels where possible; assess actual viewing distance.
- No scrolling needed for the primary Drive or Trip records view. Use drill-down pages for detail.
- A settings/diagnostics entry may be secondary; do not add a seventh permanent bottom tab.

Preserve restrained motion: traces and indicator positions respond to measurements, transitions are short, and stored bests do not continually animate. A new valid personal best may get one brief accent highlight. Do not implement a distracting celebration while driving.

## 4. Screens and interactions

### Drive

Three principal regions:

1. Left: GPS speed in km/h; signed battery electrical power in kW; clear draw/regen direction.
2. Centre: the signature two-axis G-force plot.
3. Right: displayed SOC, range estimate, battery average temperature.

The footer contains combined G, trip efficiency in Wh/km, and a selectable trip-best field. Tapping the trip-best field cycles through **maximum lateral G, peak regeneration, best 0–60 km/h, peak battery draw**. Remember the user's choice. Records must not change simply because the currently displayed field changes.

Power must be bidirectional and labelled, rather than relying on colour alone. Positive normalized power means battery discharge; negative means battery charging/regen. If using car speed as fallback for GPS, change the source label explicitly.

### G-force visual shared by Drive and Dynamics

- X axis: lateral acceleration; right positive and left negative in display coordinates.
- Y axis: acceleration upward; braking downward.
- Establish the sensor-to-vehicle rotation and convert signs explicitly.
- Fixed, labelled reference rings: 0.5 g and 1.0 g in the prototype.
- Current sample: crisp bright point, subtle halo and optional vector from origin.
- Recent samples: a fading 3–5 second trail.
- Trip history: a subdued density cloud; use bins or bounded samples rather than retaining every point forever.
- Show overflow honestly if samples exceed the chosen scale. Do not silently clamp peaks.
- Do not automatically zoom during a manoeuvre.
- A historical envelope is not a tyre grip limit. Never label a derived number as remaining grip percentage.

### Trip — approved headline screen

Top selector: **This trip / Today / All time**. Show the selected session/date context. Six large, tappable record tiles in a 3 × 2 grid:

| Record | Unit | Required interpretation |
|---|---|---|
| Best 0–60 km/h | seconds | Fastest valid stationary-start run |
| Best 0–100 km/h | seconds | Fastest valid stationary-start run |
| Maximum lateral G | g | Greatest filtered magnitude; retain left/right values |
| Maximum braking G | g | Greatest filtered longitudinal deceleration magnitude |
| Peak battery draw | kW | Greatest one-second mean discharge power |
| Peak battery regen | kW | Greatest magnitude of one-second mean charging power while driving |

Each tile shows value/unit, time or date, and an explicitly named comparison. Use a restrained new-best accent only when a valid lifetime record was actually beaten. There must be an empty state (`—`, no valid run yet), not zero seconds.

Footer: distance, average Wh/km, recovered kWh, and **Journey totals →**.

Tapping a record opens a detail page inside Trip with:

- Record name/value, scope and event timestamp.
- Synchronized speed, relevant G-force and battery power traces with independent labelled units/scales and a common time axis.
- SOC, battery temperature, data source and validity context captured at that event.
- For acceleration: stationary-start/no-rollout convention, valid run count, GPS quality and timing uncertainty where available.
- For power: one-second mean and instantaneous maximum, both clearly distinguished.
- For lateral G: separate left/right peaks; for braking: related combined G.
- A prominent **← Records** button restoring the selected scope.

The reference details are static illustrations. Implement real event buffers. A linked scrubber is a useful subsequent enhancement; the approved prototype does not yet contain one.

### Journey totals within Trip

Include distance, driving time, average speed, net battery energy, recovered energy, average Wh/km, valid acceleration-run counts, and best 60–100 km/h. Also expose peak acceleration G and peak combined G, which were discussed but are not separate headline tiles. Record a 60–100 rolling run independently of whether there was a preceding 0–100 run; state that convention consistently.

### Dynamics

Expanded G-force plot; lateral, longitudinal and combined readings; left/right and acceleration/braking peaks; yaw rate if calibrated. Keep source and filtering consistent with Trip records. Detailed synchronized pedal/speed traces can be added once the relevant signals are validated. Closed-course performance testing is the intended context for timed runs.

### Energy

Rolling consumption history, trip-average Wh/km, net energy and recovered kWh. A later view can show signed battery power over time or route segments coloured by consumption. At low speed, instantaneous Wh/km becomes unstable: suppress it and show kW. Show stationary consumption as energy, not a fabricated distance efficiency.

### Battery

Pack voltage/current, average/minimum/maximum temperature, cell-voltage spread, and BMS health estimate. Treat health as a slow estimate with timestamp/history, not a calibrated battery warranty result. Individual cell heatmaps and module temperatures are later features requiring a more involved acquisition path.

### Charge

Charging power versus SOC, energy added, elapsed time, pack temperatures and session comparison. Distinguish battery-side power from wall/charger input. Show 10–80% elapsed only if both crossings were observed with sufficient data. Estimated completion time is optional and must be labelled as estimated.

Vehicle sleep may make charging telemetry intermittent. Display last-sample age and gaps. Do not retain stale power under a live label or extrapolate unseen charging energy as measured.

### Secondary settings/diagnostics

IMU calibration, BLE/GPS connection diagnostics, sample rates/age, logging status, brightness/day-night settings, trip management, clock/timezone and firmware version. Default to observation; coding, ECU actuator commands, fault clearing and charging control are outside this project scope.

## 5. Metrics, validity and record rules

### Time and trip lifecycle

Use a monotonic clock for intervals, integration and event ordering. Use UTC for stored wall-clock timestamps and a configured timezone for display/day grouping. The user currently uses Australia/Melbourne; handle DST and allow configuration. Do not hardcode the sample October dates.

Suggested initial policy: begin a trip on a validated vehicle-ready transition; finish after confirmed shutdown with a reconnect grace period. Short BLE dropouts must not end a trip. GPS movement alone may be a fallback with explicitly documented limitations. Allow manual trip reset while parked. Persist active-trip state periodically without excessive flash wear. Today aggregates sessions using the selected local-day rule; define handling of midnight crossings. All-time records must survive restart and ordinary trip reset. Destructive record clearing requires a deliberate user action.

### Acceleration timing

Implement a state machine: idle → stationary/armed → running → target crossed → accepted/rejected. Require a stable stationary interval, valid fresh speed, forward movement and continuity. Configure thresholds to suit the actual GPS receiver, not arbitrary UI animation timing. Use interpolated target crossings between valid timestamped samples; do not claim millisecond accuracy from slow GPS. Avoid rollover of noisy near-zero readings into repeated false starts. Reject or flag dropouts, interrupted runs, poor fixes and invalid sensor time. Default to no rollout. Store measured run metadata and uncertainty. Grade compensation is not assumed; record grade only if independently reliable.

### G-force

Calibrate accelerometer offsets/scales as appropriate and gyro zero bias while stationary. Rigidly mount the sensor, establish vehicle axes, and compensate orientation/gravity using sensor fusion. A simplistic accelerometer-only tilt estimate is corrupted by sustained acceleration. A six-axis IMU does not provide drift-free absolute heading. Do not infer road gradient from acceleration alone.

Maintain separate processing paths for raw logging, filtered live display and validated peaks. Reject clipping and isolated shock/pothole spikes. Define the filter/window and retain its version with records. Combined G is `sqrt(lateral_g² + longitudinal_g²)` from the same sample, not the combination of independently recorded maxima.

### Power and energy

Normalize battery power to `P_kW = V × I / 1000`, positive for discharge. Validate raw current sign first. Align voltage/current timestamps and reject stale pairs.

- Draw energy = integral of `max(P, 0)` over the driving session.
- Recovered energy = integral of `max(-P, 0)` while driving; exclude plugged charging.
- Net trip energy = draw energy minus recovered energy.
- Wh/km = `1000 × net_kWh / distance_km`, only above a minimum usable distance.
- Peak draw/regen headline values use valid one-second averages; retain instantaneous maxima separately.

Use real time deltas, not assumed polling periods. Do not integrate across large data gaps. Mark incomplete coverage and never silently invent missing energy. Pack power includes auxiliary loads and losses; it is not motor mechanical output or wheel horsepower. Recovered energy is not a direct measurement of friction-braking contribution or regeneration efficiency.

## 6. Vehicle signal research and confidence

Research found F56 Cooper SE-specific definitions in Iternio/ABRP, OBDb and OVMS. These support implementation discovery but were **not tested on this user's vehicle or dongle**. Avoid treating generic OBD emissions PIDs or broad 2020–2026 website listings as proof for every electric MINI generation.

The BM+ transports requests; it does not itself provide a complete semantic signal catalogue. Implement manufacturer-specific UDS requests, correct ECU addressing and ISO-TP/ELM flow control. Never send only the bare request below without configuring routing and parsing correctly.

| Signal | Request | Source and qualification |
|---|---|---|
| Display SOC and associated limits | `22 DDBC` | Iternio/OBDb/OVMS; distinguish from physical SOC and compare with dashboard |
| HV pack voltage | `22 DD68` | Iternio/OBDb/OVMS |
| HV pack current | `22 DD69` | All three; byte/sign discrepancy requires validation |
| Pack temperature min/max/mean | `22 DDC0` | Iternio provides mean; OVMS decodes all three |
| BMS capacity/health estimate | `22 6335` | All three; slow estimate |
| Vehicle speed | `22 D107` | OBDb/OVMS |
| Odometer | `22 D10D` | OBDb/OVMS; verify returned fields and scaling |
| Range by driving mode/current estimate | `22 420C` | OBDb/OVMS |
| Battery summary statistics | `22 DFA0` | OVMS; capacity/voltage/SOC summaries |
| Motor temperature | `22 DEA6` | OVMS |
| Motor RPM | `22 DEA7` | OVMS; torque fields decoded in source but not a validated wheel-power solution |
| Inverter/power electronics temperatures | `22 DE8C` | OVMS |
| Accelerator position | `22 DE9C` | OVMS |
| Wheel speeds | `22 D030` | OVMS; availability does not imply high-rate access |
| Charging operating state | `22 DE84` | OVMS |
| Charger voltage/current | `22 DE86`, `22 DE87` | OVMS; identify AC/DC fields correctly |
| 12 V battery information | `22 402B` | OVMS; distinguish ECU/adapter supply voltage from battery measurements |

Further charging interface, cabin and body signals exist in OVMS. They are lower priority. Do not promise steering angle, brake pressure, traction-control intervention, friction-brake contribution, or available drive/regen power limits without validating additional definitions.

### Core addressing and parsing starting points

Iternio's initialization uses ELM commands including `ATPBE101`, `ATSPB`, transmit header `6F1`, receive filter `607`, extended address `07`, and explicit flow-control settings. Read the entire current upstream file before implementing; these are an interdependent setup, not a generic auto-protocol example.

OBDb routes the cited battery requests through `6F1` / response `607` / extended address `07`; its instrument-cluster speed/range/odometer requests use response `660` / extended address `60`. Other ECU addresses must be taken from the relevant definitions, not assumed to match the battery ECU.

For payload decoding, byte indices below mean decoded DID data after response headers, lengths, service and DID bytes are removed:

- `DD68`: initial unsigned 16-bit big-endian value divided by 100 → volts.
- `DDBC`: initial unsigned 16-bit / 10 → displayed SOC; following fields describe associated upper/lower values. Validate semantics before exposing them.
- `DDC0`: OVMS uses signed 16-bit / 100 at offsets 0, 2, 4 → min/max/mean °C.
- `DD69`: OVMS reads signed 32-bit / 100 then reverses sign. Iternio uses a signed lower-word expression with reversal; OBDb defines signed 32-bit / 100 without that reversal. Do not casually copy one formula. Capture raw payloads at rest, discharge and regen, establish a normalized convention, and test range/overflow.
- `6335`: source implementations also differ in full-word versus low-byte interpretation. Verify payload length and plausible meaning; label as BMS estimate.

Individual cell voltage/module temperature acquisition in OVMS uses UDS RoutineControl select/probe sequences, not simply another read DID. Defer until after the observation-only core is stable and those routines have been independently reviewed and tested. Do not describe routine-control traffic as passive CAN listening.

### Primary sources

Use these as implementation references, inspect licenses before copying code, and pin source revisions used by the firmware:

1. [Iternio Cooper SE definitions and ELM initialization](https://github.com/Iternio-Planning-AB/ev-obd-pids/blob/main/Mini/MiniCooperSE.json)
2. [OBDb Cooper SE signal set](https://github.com/OBDb/MINI-Cooper-SE/blob/main/signalsets/v3/default.json)
3. [OVMS Cooper SE implementation](https://github.com/openvehicles/Open-Vehicle-Monitoring-System-3/blob/master/vehicle/OVMS.V3/components/vehicle_minise/src/vehicle_minise.cpp)
4. [OVMS Cooper SE documentation, including sleep/access limitations](https://docs.openvehicles.com/en/latest/components/vehicle_minise/docs/index.html)
5. [Vgate vLinker BM+ product documentation](https://www.vgatemall.com/products-detail/i-15/?s=1)
6. [Espressif ESP32-S3 BLE architecture](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/ble/overview.html)
7. [Waveshare ESP32-S3-Touch-LCD-4.3C documentation](https://docs.waveshare.com/ESP32-S3-Touch-LCD-4.3C)
8. [SparkFun ISM330DHCX hookup guide](https://learn.sparkfun.com/tutorials/qwiic-6dof---ism330dhcx-hookup-guide/all)
9. [Analog Devices: acceleration and inclination limitations](https://www.analog.com/en/resources/app-notes/an-1057.html)

## 7. Embedded architecture recommendation

Adapt to an existing project if present. Otherwise, ESP-IDF + LVGL + a BLE-capable stack such as NimBLE is a reasonable starting point. Use the exact Waveshare board support package and compatible library versions; do not upgrade blindly before the board demo works.

**Critical compatibility point:** ESP32-S3 supports BLE, not Bluetooth Classic. Connect to the BM+'s BLE service. `BluetoothSerial`/SPP examples are not suitable. The BLE service/characteristic UUIDs were not established during this design work; discover and verify writes, notifications, MTU/fragmentation and reconnect behaviour on the actual BM+ firmware. Do not invent UUIDs or assume Nordic UART.

Separate responsibilities:

- BLE transport and serialized ELM request/response queue.
- Vehicle protocol/decoder layer with payload length checks, signedness and units.
- IMU acquisition and fusion independent of BLE or drawing.
- GPS parser with timestamps, fix quality, speed and receiver update rate.
- Timestamped normalized telemetry store/snapshot.
- Trip integration, run detector, peaks and event capture.
- Bounded logging queue and persistent record store.
- UI consuming snapshots; UI must never block sampling or send diagnostic commands directly.

Each normalized sample should carry signal ID, value/unit, source, monotonic capture/receive timestamps, UTC when known, validity, age and relevant quality metadata. Maintain raw responses for decoder validation without placing technical strings in the driving UI.

### Initial rate targets, not promises

| Layer | Starting target |
|---|---|
| IMU acquisition | 104 or 208 Hz, hardware-supported setting |
| Motion processing/logging | 50–100 Hz as useful after filtering |
| Screen rendering | Stable 30 fps first |
| Current/voltage/speed OBD group | Benchmark; aim for approximately 2–5 updates/s per signal if achievable |
| SOC and temperatures | Every 2–10 seconds |
| Health and detailed battery scans | Much slower; on demand where appropriate |

These rates are not additive guarantees. Poll sequentially and adapt to response latency, ECU routing changes, retries and screen priority. Show sample age and never equate interpolated visual smoothness with measurement freshness. Avoid polling hundreds of cell values during performance capture.

Use cached static backgrounds, bounded chart buffers, density bins and selective redraws. An RGB565 800 × 480 framebuffer is 768,000 bytes; two consume 1,536,000 bytes before other buffers. Budget PSRAM and display bandwidth, avoid allocation in fast loops and benchmark the actual board under BLE + IMU + GPS + logging load. Do not promise 60 fps or infer performance from the HTML preview.

## 8. Logging and persistence

Store continuous telemetry in bounded sessions, preferably on verified SD storage if available. Use wear-conscious NVS or another appropriate store for preferences, lifetime record indices and crash recovery. Handle absent/full/failed storage without blocking the UI or losing live acquisition.

Suggested record fields: schema version, metric ID, value/unit, trip/event IDs, timestamp, source, sample coverage, filter/decoder version, SOC, battery temperature, validity/uncertainty, and event trace reference. Save pre-event and post-event windows with a ring buffer rather than logging only the maximum value. Aggregate daytime totals from energy/distance sums, not the arithmetic mean of session Wh/km.

Keep records comparable across firmware changes. If filtering or timing conventions change, flag the older baseline or migrate explicitly. Test recovery after abrupt power loss and prevent one event from being accepted multiple times.

## 9. Power, sleep and fault behaviour

The product power supply, ignition detection and shutdown strategy are unknown. Inspect the actual installation before prescribing wiring. The display board must receive a suitable regulated supply, not an assumed direct vehicle 12 V connection.

OVMS reports limited/intermittent OBD accessibility while the car sleeps or charges. The vehicle's 12 V supply and sleep behaviour must be tested with this actual device. Do not keep modules awake just to maintain a pretty live graph. Verify the dongle/device combination when parked, locked, charging and reconnecting; handle possible OBD-related alarm behaviour without changing vehicle coding as part of this project.

When a source is lost: mark its readings stale/unavailable, stop invalid integration, retain completed records, and reconnect with bounded backoff. G-force remains usable if the IMU is healthy even when BLE disconnects. GPS failure must not masquerade as zero speed.

## 10. Implementation stages and acceptance criteria

### Stage A — hardware and protocol proof

Run the exact board's display/touch demo. Confirm IMU axes and rates, identify GPS receiver/interface, connect to BM+ over BLE and capture raw read responses. Validate SOC, voltage, current and temperature plausibility and current sign in several operating states. Record achieved per-signal rates and timestamps.

### Stage B — polished live core

Implement shared theme, six-tab shell and Drive/Dynamics. Separate sampling from rendering. Verify readable labels at physical size, correct G signs, correct regen direction, stale-data treatment, responsive touch and no display tearing under load.

### Stage C — trips, records and event details

Implement trip lifecycle, integration, run detection, one-second power records, all time scopes, record comparisons and event buffers. Then populate the approved Trip and journey/detail pages with real data. Preserve completed records through reconnects and restart.

### Stage D — energy, battery and charge

Implement real curves and session logs. Validate source-side meanings, current/voltage alignment, charging sleep gaps and temperature decoding. Add advanced cells/maps only after the core is reliable.

### Meaningful verification

- Decoder fixtures from captured payloads: positive/negative current, signed temperature, malformed/short replies, multi-frame responses and wrong ECU/DID.
- Integration replay: known power trace gives expected draw/recovered/net energy; gaps do not create phantom energy; charge sessions do not inflate driving regen.
- Run detection replay: valid crossing, GPS noise near zero, target crossed between samples, dropout, interrupted run and no-rollout convention.
- Peak tests: short shocks versus sustained events; combined G from simultaneous axes; one-second versus instantaneous power.
- Trip persistence: reconnect, shutdown grace, manual reset, power interruption, midnight/DST and lifetime-record retention.
- UI checks at **800 × 480**: no clipped controls, all six tabs, all three scopes, empty states, event back navigation and Drive metric cycling.
- Hardware checks: actual sensor response, realistic update rates, touch latency, logging load, memory use and parked power/sleep behaviour.

No unverified sensor should display a plausible dummy number in production. Demo mode must be explicit, visibly labelled and isolated from genuine trip/lifetime records.

## 11. Unresolved inputs and how to proceed

First inspect existing code and hardware documentation. Ask the user only for information that remains necessary:

- Firmware repository and preferred toolchain, if one already exists.
- GPS model, interface, update rate and timestamp/PPS capabilities.
- Exact board revision, sensor wiring, power supply and mounting orientation.
- BM+ firmware and discoverable BLE services.
- Available logging storage and desired export method.

Default to the approved design and sensible implementation choices. Do not ask the user to redesign the screens or reconfirm decisions already recorded here. If a signal or update rate is unavailable, explain the measured constraint and degrade that feature honestly.

## 12. Suggested kickoff prompt for the implementing model

> Implement the MINI Cooper SE telemetry device described in this brief. Inspect the attached design preview and original fragment as the approved visual reference. Target the Waveshare ESP32-S3-Touch-LCD-4.3C, SparkFun ISM330DHCX and Vgate vLinker BM+ over BLE. Preserve the six screens, Trip record scopes/detail views, and selectable Drive trip-best field. Treat all prototype values as synthetic. Inspect the existing firmware and board support first, establish and validate the real data path before claiming vehicle compatibility, then implement in the stages and acceptance criteria above. Keep live sampling, signal validity, logging and UI separate. Ask only for genuinely missing hardware/repository information; do not substitute another design or fabricate unavailable telemetry.
