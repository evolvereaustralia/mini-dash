// Turns one UDS payload (the bytes after 62 DD DD) into car data, using the offsets and
// scaling of include/obd_pids.h. Every value is checked against physical limits before it is
// written (CLAUDE.md section 3: the dongle is untrusted input); a payload that is too short or
// out of range changes nothing and returns false. Plain C++, tested natively against
// replies recorded from cars.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "vehicle_state.h"

namespace ObdDecode {

// Applies the payload and stamps the group it belongs to with nowMs.
bool apply(uint8_t ecu, uint16_t did, const uint8_t *p, size_t n, uint32_t nowMs, CarData &car);

// A one-line human-readable decode for the bring-up log (docs/obd-notes.md).
void describe(uint8_t ecu, uint16_t did, const CarData &car, char *out, size_t outLen);

}  // namespace ObdDecode
