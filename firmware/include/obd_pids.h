// OBD ECU addresses, DIDs and scaling for the 2022 Mini Cooper SE (F56 BEV).
// Filled in phase 3c. Every line must cite its source: OVMS vehicle_minise, the OBDb
// MINI-Cooper repo, or a request/response pair logged in docs/obd-notes.md.
// Do not invent DIDs (CLAUDE.md section 3).
#pragma once

#include <stdint.h>

// Tester (our) CAN ID for BMW F-series diagnostics over ISO 15765 with extended addressing.
// Source: CLAUDE.md section 3 (BimmerCode/OVMS convention). Verify against the car in 3c.
constexpr uint16_t OBD_TESTER_ID = 0x6F1;

// UDS service and positive-response byte for ReadDataByIdentifier.
constexpr uint8_t UDS_READ_DID = 0x22;
constexpr uint8_t UDS_READ_DID_OK = 0x62;

// ECU addresses and DIDs: none yet. See docs/obd-notes.md once the vLinker BM+ arrives.
