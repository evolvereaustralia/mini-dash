// OBD ECU addresses, DIDs and scaling for the 2022 Mini Cooper SE (F56 BEV). Nothing here is
// invented (CLAUDE.md section 3); every line cites one of:
//   [OVMS]  OVMS v3 vehicle_minise, commit 6399a607 (22 Nov 2026): ecu_definitions/*_defines.h
//           for the addresses and DIDs, src/vehicle_minise.cpp for the scaling (a module that
//           runs on real Cooper SEs).
//   [OBDb]  OBDb/MINI-Cooper, commit ac00b0bc (20 Nov 2026): signalsets/v3/default.json for
//           bit offsets and scaling, tests/test_cases/20xx/commands/*.yaml for recorded
//           request and response pairs from cars (the vectors in firmware/test/native).
//   [car]   a pair logged from Logan's car in docs/obd-notes.md.
// Where the two sources disagree or only one has a value, the comment says so.
// [car] 24 Nov 2026: every identifier below answered on Logan's car at the first connection
// with plausible values (docs/obd-notes.md), including the DSC wheel speeds.
//
// Transport: BMW F-series diagnostics, ISO 15765 on 11-bit CAN at 500 kbit with extended
// addressing. We transmit on 0x6F1 with the ECU address as the first data byte; the ECU
// answers on 0x600 + its address with 0xF1 (us) as the first data byte. [OVMS] I3_ECU_x_TX
// 0x06F1aa and _RX 0x06aaF1; [OBDb] hdr 6F1, rax 6aa, eax aa.
#pragma once

#include <stdint.h>

constexpr uint16_t OBD_TESTER_ID = 0x6F1;
constexpr uint8_t OBD_TESTER_ADDR = 0xF1;
constexpr uint16_t OBD_RX_BASE = 0x600;

constexpr uint8_t UDS_READ_DID = 0x22;
constexpr uint8_t UDS_READ_DID_OK = 0x62;
constexpr uint8_t UDS_NEGATIVE = 0x7F;
constexpr uint8_t UDS_NRC_PENDING = 0x78;

// ECU addresses.
constexpr uint8_t ECU_SME = 0x07;  // battery management          [OVMS] ecu_sme_defines.h; [OBDb] eax 07
constexpr uint8_t ECU_EME = 0x1A;  // drive electronics           [OVMS] ecu_eme_defines.h; [OBDb] eax 1A
constexpr uint8_t ECU_EDM = 0x12;  // powertrain control (EDME)   [OVMS] ecu_edm_defines.h; [OBDb] eax 12
constexpr uint8_t ECU_DSC = 0x29;  // stability control           [OVMS] ecu_dsc_defines.h; [OBDb] eax 29
constexpr uint8_t ECU_KOM = 0x60;  // instrument cluster          [OVMS] ecu_kom_defines.h; [OBDb] eax 60
constexpr uint8_t ECU_NBT = 0x63;  // head unit                   [OVMS] ecu_nbt_defines.h; [OBDb] eax 63

// SME, battery. Offsets are into the payload after 62 DD DD, big-endian.
constexpr uint16_t DID_SME_VOLTS = 0xDD68;  // u16 / 100 V. [OVMS] HV_SPANNUNG_BERECHNET; [OBDb] COOPER_HVBAT_VOLT
constexpr uint16_t DID_SME_AMPS = 0xDD69;   // s32 / 100 A, positive while charging; negated here so
                                            // positive is discharge. [OVMS] HV_STROM (negates too); [OBDb] COOPER_HVBAT_CURR
constexpr uint16_t DID_SME_SOC = 0xDDBC;    // u16 / 10 % at 0: the display SOC, shown by the dash as it is ([car]: 50.7 against
                                            // 51 on the dash; [OVMS] ANZEIGE_SOC uses it unscaled too). At 2 and 4: a maximum and
                                            // a minimum that are not a usable window ([car] 99.9 and 14.5; [OBDb] 97.0 and 11.5)
constexpr uint16_t DID_SME_TEMPS = 0xDDC0;  // s16 / 100 C at 0 (min), 2 (max), 4 (mean). [OVMS] TEMPERATUREN; [OBDb] _CT_MIN, _CT_MAX
constexpr uint16_t DID_SME_CELLS = 0xDFA0;  // u16 / 10000 V at 6 (min), 8 (max), 10 (mean); 38 bytes or more.
                                            // [OVMS] ZUSTAND_SPEICHER; [OBDb] COOPER_HVBAT_C_V_MIN, _MAX, _AVG
constexpr uint16_t DID_SME_SOH = 0x6335;    // u32 % (the value sits in the last byte). [OVMS] ALTERUNG_KAPAZITAET; [OBDb] COOPER_HVBAT_SOH

// EME, drive.
constexpr uint16_t DID_EME_MACHINE = 0xDEA7;      // u16 * 0.5 - 5000 rpm at 0; s16 * 0.5 Nm actual at 2, requested at 4.
                                                  // [OVMS] AE_ELEKTRISCHE_MASCHINE; [OBDb] COOPER_EM_RPM, _TQ, _TQ_TGT
constexpr uint16_t DID_EME_MOTOR_TEMPS = 0xDEA6;  // s16 * 0.0156 C at 0 and 2. [OVMS] AE_TEMP_EMASCHINE; [OBDb] COOPER_EM_T_1, _2 (39/2500)

// EDM, powertrain control.
constexpr uint16_t DID_EDM_PEDAL = 0xDE9C;  // u16 * 0.0625 % at 4. [OVMS] PEDALWERTGEBER; [OBDb] COOPER_APP (bix 32, / 16)
constexpr uint16_t DID_EDM_IBS = 0x402B;    // 12 V battery sensor: u16 / 4000 + 6 V at 2. [OVMS] ecu_edm_extra_defines.h
                                            // STAT_U_BATT (defined, not polled there); [OBDb] COOPER_12V_V

// Wheel speeds. Two candidates; the client probes the first at connection and falls back.
constexpr uint16_t DID_DSC_WHEELS = 0xDBE4;  // s16 / 100 km/h at 0 FL, 2 FR, 4 RL, 6 RR. [OBDb] only (COOPER_FL_VSS..., recorded
                                             // on 2014+ cars); not in [OVMS]; [car] answers on the SE
constexpr uint16_t DID_NBT_WHEELS = 0xD030;  // s16 km/h (whole numbers) at 0, 2, 4, 6. [OVMS] NBT_STATUS_SPEED; [OBDb] COOPER_NBT_WHL_SPD_n

// KOM, cluster.
constexpr uint16_t DID_KOM_RANGE = 0xD111;  // u16 / 10 km at 0 (current electric range). [OVMS] KOMBI_REICHWEITE_BEV_PHEV; [OBDb] COOPER_RANGE_E_CUR
