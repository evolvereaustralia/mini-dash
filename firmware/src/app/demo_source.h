// Demo replay (CLAUDE.md section 4, Settings): a labelled, scripted drive that feeds the same
// VehicleState and TripEngine as the real sensors, so the whole UI can be developed and
// reviewed on the bench. Isolated from real records: switching demo off resets the trip.
//
// The script, about 95 s per loop: stand still, 0-100 in about 7.3 s, cruise with left and
// right corners, a hard stop from 100, a gentle lap with regen, back to standstill.
#pragma once

#include <stdint.h>

class TripEngine;

namespace Demo {

void start(TripEngine *engine);  // creates the 50 Hz task
void setEnabled(bool on);
bool enabled();

}  // namespace Demo
