# Independent review of commit dc9e9b4: what was done about each finding

An independent model reviewed the whole codebase on 24 November 2026 (42 findings and an appendix of further observations). Every finding was checked against the source and found correct. This file records what happened to each one. It is the ledger the second review should start from.

**State of the evidence, read this first.** Everything marked Fixed below was built with `pio run` and, where the logic could be separated from the hardware, tested natively on the Mac (`firmware/test/native/run.sh`: 226 checks and the synthetic drive, all passing). **None of it has run on the device.** No board was attached while this work was done. Until the firmware has been flashed and driven, "Fixed" means "changed, compiles, and passes the tests named", not "seen working".

Commits, on branch `review-fixes`: `d595355` speed fusion, `65d948a` car data freshness, `a230e54` OBD client, `4cc8dce` storage and the single writer, `13052d9` attitude, `eac0047` smaller findings, `3705a7c` capture mode, and the commit that adds this file.

## Owner's decisions that the fixes implement

Logan, 24 November 2026:

- **D1.** A 60 to 0 braking record is a continuous braking stop: at least 0.1 g of braking throughout from the downward crossing of 60 km/h to standstill; coasting for more than 2 s in total, or any rise in speed, cancels it. Not implemented yet: it is part of the timer work that waits for the logged drive (findings 11 to 13).
- **D2.** A trip shorter than 50 m is discarded; exactly 50 m qualifies. Implemented in `4cc8dce`.
- **D3.** A new dongle is paired only after a deliberate Pair tap in Settings while parked; the stored dongle stays paired. Implemented in `a230e54`, except that the screen does not yet show the name it found before storing it.
- **D4.** A drive that crosses midnight belongs to the day it started. Implemented in `4cc8dce`.

## The 42 findings

Status: **Fixed**, **Part** (the undisputed part is fixed, the rest waits), **Waits** (needs the logged drive or the device).

| # | Finding | Status | What was done, and how it was checked |
|---|---|---|---|
| 01 | First GPS fix in motion overstates the displayed speed | Fixed, `d595355` | The fusion seeds on the first fix, after a gap of more than 1 s, and when no IMU step has arrived since the last fix; history is read by time; a repeated epoch is ignored. Native `test_speed.cpp`, 70 checks, including a first fix at 72 km/h with and without the IMU. |
| 02 | Learning the clock restarts the active trip | Fixed, `4cc8dce` | A late clock dates the trip from its elapsed time and rebuilds Today; the trip is not restarted (D4). The engine side is in `test_trip.cpp`; the loop-task side is build only. |
| 03 | Replacing the checkpoint can erase the last good copy | Fixed, `4cc8dce` | Every trip, checkpoint and trace file is written to a second file, synced, read back and compared, then renamed over the old one. My earlier "atomic on close" comment was wrong for this LittleFS port. Build only: the power-cut test of section 9 has still to be run on the device. |
| 04 | Recovery deletes the checkpoint even when archiving fails | Fixed, `4cc8dce` | Checkpoints are per trip (`/active/<id>.trp`). One is removed only after its trip file is safely written; otherwise it stays for the next boot. An unreadable one is set aside as `.bad`. Build only. |
| 05 | A failed mount formats the trip partition | Fixed, `4cc8dce` | The partition is formatted on its own only if it is completely blank. Otherwise storage stays off for the boot, nothing is erased, and serial `KK` is the one deliberate way to format. Build only. |
| 06 | Queued event data can be mixed or lost across tasks | Fixed, `4cc8dce` | Trace slots with their own mutex; a slot is marked pending only once complete; the writer takes its own copy before writing. Build only. |
| 07 | Lifetime writes and trip closure read a changing engine | Fixed, `4cc8dce` | The engine is read and rotated only under the state lock; the copy is written after the lock is released. Build only. |
| 08 | Switching the replay destroys a real session | Fixed, `4cc8dce` | The replay no longer resets the engine. The loop task closes the running trip properly, then switches the source. A replay trip is judged by its own flag, never archived. Build only. |
| 09 | First dongle chosen by name and signal strength | Part, `a230e54` | D3: nothing is paired without a Pair tap, and the binding is stored only after the car has answered through that dongle. Waits: showing the found name for confirmation before storing it. |
| 10 | Bench stall probe reads past the frame buffer | Fixed, `4cc8dce` | The probe now reads 64 points inside the buffer. **The stall figures taken with the old probe (1.5 ms against 20 ms) are preliminary until the test is repeated on the device.** |
| 11 | Timer uses scheduler ticks as GPS epochs | Waits | The fusion is now fed one distinct epoch at a time (`d595355`), but the timers still run on a 20 Hz tick with a cached fix. To be rebuilt on GPS epochs once the capture log shows the real epoch timing. In capture mode timer runs are logged, never recorded. |
| 12 | Launch detector rejects a clean launch or continues an abandoned one | Waits | Same work as 11. |
| 13 | Rolling and braking records span low-quality fixes and unrelated motion | Waits | Same work as 11, with the D1 rule for braking. |
| 14 | Unrelated OBD replies keep absent measurements apparently fresh | Fixed, `65d948a` | One time stamp per measurement; the SOC baseline waits for a real SOC reading. Native `test_obd.cpp` and `test_trip.cpp`. |
| 15 | OBD can stay asleep while other ECUs are awake | Fixed, `a230e54` | The asleep probe rotates across the ECUs and a refusal counts as awake. Build only; no sleep and wake cycle has been observed. |
| 16 | Transient OBD failures persistently change scheduling | Fixed, `a230e54` | Defaults are constants and are restored; a demoted item gets its rate back when it answers; the ECU cache is cleared before a switch; the shared filter is judged by re-asking. Build only. |
| 17 | UI trip reset blocks the display and sensor publication on flash I/O | Fixed, `4cc8dce` | No UI callback touches flash. Requests go to the loop task; file system figures are read outside the state lock. Build only. |
| 18 | A failed AT send passes uninitialised stack text to the logger | Fixed, `a230e54` | The reply buffer is initialised before any return. |
| 19 | Unchecked allocation and task creation | Fixed, `a230e54` and this commit | Stream buffer, client and every task creation are checked; "sensors ready" is said only once the sensor task exists. |
| 20 | A sustained gentle turn becomes the gravity or bias reference | Fixed, `13052d9` | Turn gate: speed times yaw rate above 0.05 g holds the accelerometer off, released after 0.5 s under 0.03 g. Native `test_attitude.cpp`: a 0.089 g bend for 60 s reads 0.003 g without the gate and 0.089 g with it. The thresholds are the reviewer's starting values, to be tuned from the logged drive. |
| 21 | The 3 percent lockout can return on the next boot | Fixed, `13052d9` | The rest magnitude is seeded from the sensor, not from the unit-length stored reference; gyro bias is learnt only on GPS evidence of a standstill. Native test. |
| 22 | Near-vertical forward reference is ill-conditioned | Fixed, `13052d9` | The forward axis is a stored setting (serial `A`); a setting within 30 degrees of vertical for the mounting is refused and the G figures show dashes. Native test, including the enclosure mounting with forward minus Z. |
| 23 | Bad calibration fits pass acceptance | Fixed, `13052d9` | Double precision about the sample mean, a conditioning test, and acceptance at 10 mg RMS and 20 mg worst direction. Native test. Note: the synthetic sensor in `test_fusion.cpp`, with datasheet-typical scale errors, fits at 8.2 mg, so the 10 mg limit is close for a part with large scale errors; this unit measured 2.9 mg. |
| 24 | Event charts show the wrong run or never load a saved trace | Part, `eac0047` | The page reloads when the record changes and asks again for a trace that is not written yet. Waits: a generation number tying a trace to its record. |
| 25 | Run counts in Today and lifetime are wrong | Fixed, `4cc8dce` | Records and counts are merged independently. Native `test_trip.cpp`. |
| 26 | Power record pairs a mean with a peak from another event | Fixed, `eac0047` | Time-weighted one-second mean with that second's own extreme; a gap starts the window afresh. Native `test_trip.cpp`. |
| 27 | Charts only collect history on the visible tab | Fixed, `eac0047` | Both charts are sampled every 5 s whatever page is showing. Build only. |
| 28 | Mounting calibration says "Taken" before it is saved | Fixed, `4cc8dce` | The button reads Saving, then Stored or Failed from the result of the write. Build only. |
| 29 | OBD request deadline does not bound a whole exchange | Part, `a230e54` | Application waits are bounded and writes use write without response. Waits: the BLE library's own waits in discovery and subscription have no deadline of ours; they matter at connection time, on the OBD task only. |
| 30 | Road log can overwrite the only copy of a drive | Fixed, `eac0047` | File named to the second and opened so that an existing file is an error. |
| 31 | An isolated shock can set a lasting lateral record | Waits | Needs a second-order filter and a shock test chosen from real road data; the capture log carries the raw and the filtered path for that. |
| 32 | Long uptimes break absolute timestamp arithmetic | Fixed, `eac0047` | Crossing times in double. Native test: exact to half a millisecond at 8 hours of uptime. |
| 33 | Decoder rejection contract has an integer hole | Fixed, `65d948a` | Negation done in double; extremes must be ordered around the mean. Native `test_obd.cpp`. |
| 34 | Forgetting a dongle while OBD is off is acknowledged, not committed | Fixed, `a230e54` | Forget is handled before the on/off test. |
| 35 | Bus hold limit observed only after the fact | Fixed, `eac0047` | The scan takes the bus per address; a GPS that answers but refuses its settings is reported as not configured. Build only. |
| 36 | UI strings and trace labels lose information | Fixed, `65d948a` | Text buffer 256 bytes; the event context line built separately. |
| 37 | Storage formats accept incompatible same-size layouts | Fixed, `eac0047` | Trip and event files carry a CRC-32 (trip version 3; versions 1 and 2 still read); unreadable trip files are set aside; build-time asserts fail if a stored layout changes size. The NVS blobs (settings, lifetime records) still rely on size and version alone. |
| 38 | A net-regenerating trip hides its efficiency | Fixed, `65d948a` | Signed efficiency is shown; the moving average uses distance covered while moving. |
| 39 | Pairing and trip IDs persisted only in part | Fixed, `a230e54`, `4cc8dce` | The binding is one record; a trip number is checked against the files on disk and verified after it is stored. |
| 40 | Chosen speed can stay on screen past its validity | Fixed, `65d948a` | Drive shows a speed only while its source is fresh; a snapshot that timed out is reported. |
| 41 | Overlapping `strncpy` on reconnect | Fixed, `a230e54` | |
| 42 | Host clearance checker prints FAIL and exits 0 | Fixed, `eac0047` | Both enclosure checkers exit non-zero on failure. `check_overhangs.py` was run on the four released STLs: CLEAN, 14 accepted bridges on the shell. `check_layout.py` compiles but was not run (no CadQuery on this machine). |

## The appendix observations

| Area | Observation | Status |
|---|---|---|
| Clock | Base time read and written with no lock | Fixed, `4cc8dce` |
| Boot | UI can reach storage before it is open | Fixed, `4cc8dce`: requests are held until the first trip has started; a trace read before the mount returns nothing |
| Display | `flushCb` swaps buffers after a wait timeout and ignores the draw result | Open. Not touched without the device in hand: it is the display path that took a night to get right |
| Loop | A due checkpoint cleared even when the write fails | Fixed, `4cc8dce` |
| Loop | A checkpoint can wait behind traces | Fixed, `4cc8dce`: the checkpoint is written first |
| Loop | Trip id read unlocked | Fixed, `4cc8dce` |
| Storage | Trip names counted without validating the files | Fixed, `eac0047` |
| Storage | Event header and samples written separately; reader accepts a prefix | Fixed, `4cc8dce` |
| Wear | Lifetime records written up to ten times a second | Fixed, `4cc8dce`: at most every 30 s |
| Sensors | Calibration and brightness state crosses tasks with no lock; counters can lose counts | Part: calibration results are handed over under a lock. Brightness state and the diagnostic counters are unchanged |
| Sensors | Backlight write failure ignored | Fixed, `eac0047` |
| Demo | Real GPS and the replay both write the GPS state | Fixed, `eac0047` |
| OBD | Backoff not capped at 12 s | Fixed, `a230e54` |
| OBD | Frame count learnt before the reply is accepted | Fixed, `a230e54` |
| OBD | Receive overflow dropped silently | Fixed, `a230e54`: counted and reported |
| OBD | Status print reads the poll table while the task changes it | Fixed, `a230e54`: printed by the OBD task itself |
| Fusion | One history slot per call; duplicates manufacture a standstill | Fixed, `d595355` |
| Timers | Armed state can start across a gap; a segment crossing 60 and 100 postpones 100; exactly 1 km/h does not finish braking | Waits, with findings 11 to 13 |
| I2C | Holder tested before the mutex is taken; diagnostics unlocked; expander shadow outside the lock | Open, low risk |
| Stacks | High-water paths on the four tasks | Instrumented: the capture stream carries each task's least free stack every 10 s and the status line shows the loop task's. The large buffers of the storage self-test are off the stack. To be read from the first log |
| Allocations | Long `printf` output allocates | Part: the capture stream formats into fixed buffers. The 10 Hz motion log (key `g`) and the status line still allocate |
| Lock graph | No circular deadlock found | Unchanged in principle: the trace-slot mutex is taken inside the state lock by the engine's task and alone by the writer, never the other way round |

## What waits, and for what

1. **The first logged drive in capture mode** (`firmware/README.md`, "Capture"). It gives the raw record the reviewer asked for and found missing: every GPS epoch with its arrival time, every IMU sample, the gate decisions, the three speeds, every OBD exchange, the candidate timer runs and the stack margins.
2. **From that log:** timers on GPS epochs and the D1 braking rule (11, 12, 13 and the appendix), turn-gate thresholds (20), shock rejection (31), the forward axis sign and the GPS latency constant, wheel speed against GPS.
3. **On the device:** the stall test repeated with the corrected probe (10), the power-cut test of section 9 against the new storage path (03, 04), an OBD sleep and wake cycle (15, 29), `flushCb` (appendix).
4. **Design still to do:** the Pair confirmation screen (09), trace generations (24).
5. **Then** a second independent review, with the capture log as evidence.

## Still to be agreed with the owner

Recommended, not yet confirmed: the trace window is the whole manoeuvre plus 2 s each side (the brief says a fixed 10 s); traces of lifetime records are kept beyond the 60-trip limit; arming tests horizontal acceleration under 0.05 g; a missed epoch is an iTOW step other than 100 ms; a completed 0 to 60 stands if a later gap cancels the 0 to 100; braking thresholds of 0.30 m/s of rebound over two epochs and 0.5 s of dwell; comparison lines name their baseline.
