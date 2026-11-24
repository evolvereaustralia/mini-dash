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
| SME 07 | DDBC | SOC, maximum, minimum | u16 / 10 % each | slow |
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

- SOC: the `OBD first` line for DDBC prints the raw SOC, the limits and the scaled figure. The dash should show the scaled one (CLAUDE.md section 3); if it shows the raw one, the scaling is wrong for this car and `obd_decode.cpp` changes.
- Power sign: with the car in READY and stationary the figure should be a small positive draw; under acceleration it grows positive, lifting off it goes negative. Compare with the car's power gauge.
- Wheel speed against the speedo at a steady 60 km/h.

## Log

(nothing from the car yet)
