# OBD notes

Every request/response pair seen during bring-up goes here, with the ECU address, DID, raw bytes and the decoded value, so `firmware/include/obd_pids.h` can cite them as [car].

## Sources (24 Nov 2026)

- OVMS v3 `vehicle_minise`, commit 6399a607 (22 Nov 2026): ECU addresses and DIDs in `ecu_definitions/*_defines.h`, scaling in `src/vehicle_minise.cpp`. Runs on real Cooper SEs.
- OBDb `MINI-Cooper`, commit ac00b0bc (20 Nov 2026): `signalsets/v3/default.json` for offsets and scaling, `tests/test_cases/20xx/commands/*.yaml` for recorded replies. Those replies are the vectors in `firmware/test/native/test_obd.cpp`.

Both agree on every address and scaling we use. Differences: the DSC wheel speeds (DBE4, 0.01 km/h) are in OBDb only; the head unit's (D030, whole km/h) are in both and proven on the SE by OVMS.

## Transport

ISO 15765 on 11-bit CAN at 500 kbit with extended addressing: we send on 0x6F1 with the ECU address as the first data byte, the ECU answers on 0x600 + its address with 0xF1 first. Adapter set-up as sent by the firmware: `ATZ`, `ATE0`, `ATL0`, `ATS0`, `ATH1`, `ATSP6`, `ATAT1`, `ATST32`, `ATSH6F1`, `ATFCSH6F1`; then per ECU `ATCEAxx`, `ATCRA6xx`, `ATFCSDxx300000` and once `ATFCSM1`. A reply line looks like `607F1100762DD690000`: id 607, address F1, first frame of 7 bytes, `62 DD 69 00 00`.

## Identifiers in use

| ECU | DID | What | Scaling | Group |
|---|---|---|---|---|
| SME 07 | DD68 | pack voltage | u16 / 100 V | fast |
| SME 07 | DD69 | pack current | s32 / 100 A, charging positive in the car, negated by us | fast |
| EME 1A | DEA7 | motor speed, torque actual and requested | u16 * 0.5 - 5000 rpm; s16 * 0.5 Nm | fast |
| DSC 29 | DBE4 | wheel speeds FL FR RL RR | s16 / 100 km/h | fast, tried first |
| NBT 63 | D030 | wheel speeds | s16 km/h | fast, fallback |
| EDM 12 | DE9C | accelerator pedal | u16 at 4 * 0.0625 % | fast |
| SME 07 | DDBC | display SOC (as the dash shows it); a maximum and a minimum that are not limits | u16 / 10 % each | slow |
| SME 07 | DDC0 | pack temperature min, max, mean | s16 / 100 C | slow |
| SME 07 | DFA0 | cell voltage min, max, mean | u16 / 10000 V at 6, 8, 10 | slow |
| EME 1A | DEA6 | motor temperatures 1 and 2 | s16 * 0.0156 C | slow |
| EDM 12 | 402B | 12 V battery voltage | u16 at 2 / 4000 + 6 V | slow |
| KOM 60 | D111 | electric range | u16 / 10 km | every 5 s |
| SME 07 | 6335 | state of health | u32 % | once |

## To record from the first connection in the car

The firmware prints these on serial; `firmware/tools/drive_log.py` captures them. Close BimmerLink on the phone first: the dongle takes one connection.

1. `OBD scan: saw '...'` and `OBD: connecting to ...`: the dongle's BLE name and address.
2. `OBD gatt: service ... characteristic ...`: the GATT layout, and `OBD gatt: using ...`: the pair chosen.
3. `OBD adapter: ...`: the adapter's identity string and port voltage.
4. One `OBD first: ECU xx DID xxxx, n bytes ... = ...` line per identifier: the raw payload and the decode. These are the [car] citations.
5. Any `OBD: ECU xx refuses DID xxxx, NRC xx` line, and which wheel-speed source ended up in use.
6. Key `O`: the achieved rates and round-trip time per identifier.

## Checks against the car (open)

- SOC: done 24 Nov 2026. The ECU said 50.7 %, the dash showed 51 %, the scaled figure would have been 42 %. The first value of DDBC is shown as it is; the other two (99.9 and 14.5 here) are not a usable window, most likely the highest and lowest SOC the car has recorded.
- Power sign: with the car in READY and stationary the figure should be a small positive draw; under acceleration it grows positive, lifting off it goes negative. Compare with the car's power gauge.
- Wheel speed against the speedo at a steady 60 km/h.

## Log

### 24 Nov 2026, first connection, car in READY and stationary (drive-2026-11-24-1402.log)

- Dongle: BLE address dd:0d:30:89:1b:a7. Adapter identifies as `ELM327 v2.2`, port voltage 14.2 V.
- GATT: service 18F0 with 2AF0 (notify, indicate) and 2AF1 (write, write without response), which is the pair in use; also a vendor service e7810a71-73ae-499d-8c15-faa9aef0c3f2 with one characteristic bef8d6c9-9c21-4c9e-b632-bd58c1009f9f that does everything. Not the FFF0 layout the brief guessed.
- Every set-up command accepted, including one receive filter for all ECUs (`ATCF600`, `ATCM700`) and the "expected responses" digit after a request.

| ECU | DID | Payload (after 62 DD DD) | Decoded |
|---|---|---|---|
| 07 | DD68 | 8C03 | pack 358.43 V |
| 07 | DD69 | FFFFFCFC | -7.72 A in the car's convention, +7.72 A discharge, 2.77 kW |
| 07 | DDBC | 01FB03E70091 | SOC 50.7 % (dash: 51 %), maximum 99.9 %, minimum 14.5 % |
| 07 | DDC0 | 06A5076B0735 | pack temperature min 17.01, max 18.99, mean 18.45 C |
| 07 | DFA0 | 25DD26B3264791AA921691D906A5076B0735FFFFFFFF268F15B3167E1614920492729235268F | cells min 3.7290, max 3.7398, mean 3.7337 V |
| 07 | 6335 | 00000064 | SOH 100 % |
| 1A | DEA7 | 27100000000000 | motor 0 rpm, torque 0.0 Nm, requested 0.0 Nm |
| 1A | DEA6 | 05350532 | motor temperatures 20.8 and 20.7 C |
| 29 | DBE4 | 00000000000000000000030303030064646464 | wheels 0.00 km/h each: the DSC source answers on the SE |
| 12 | DE9C | 0093004A0000 | pedal 0.0 % |
| 12 | 402B | 00E183E8097100000700004B00020000000000000000000200000000 | 12 V battery 14.442 V |
| 60 | D111 | 02C6060E02C6FFFF3C740071 | range 71.0 km (maximum 155.0 km) |

Nothing refused, nothing unanswered in 1400 requests across four sessions. Plausibility: 96 cells at 3.7337 V is 358.4 V, the pack reading; the 12 V figure agrees with the adapter's own 14.2 V.

Rates (requests answered per second, all identifiers): 5.2 as first written; 8.2 with a 7.5 to 15 ms connection interval, the expected-responses digit and a 100 ms adapter timeout; 9 with the shared receive filter. A request costs 45 to 130 ms depending on the ECU (the battery ECU is the slowest), a change of ECU about 50 ms. Result with the schedule as committed: battery power 1.9 Hz, wheel speeds 1.9 Hz, pack voltage 1.4 Hz, torque and pedal 0.45 Hz, slow group every 2 s. The brief's 5 Hz for the whole fast group is not reachable through this adapter.

Still open: the power sign under acceleration and regen, wheel speed against the speedo.
