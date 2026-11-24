// The car's own data over Bluetooth LE from the vLinker BM+ in the OBD port (CLAUDE.md
// section 3). One task on core 0: scan for the dongle (or reconnect to the stored address),
// find its serial characteristics, set the ELM327 up for BMW extended addressing, then poll
// the identifiers of include/obd_pids.h in a round robin and write the decoded values into
// VehicleState. It never blocks the GPS and IMU side: everything here waits on its own task,
// the adapter's prompt is awaited for at most 300 ms per request, three unanswered requests in
// a row mean "car asleep" (the dongle answers, the car does not; each ECU is then probed in
// turn and the adapter set up again if none answers) and a silent dongle means reconnect.
//
// Protocol text and decoding live in obd_proto.* and obd_decode.* (plain C++, tested
// natively); this file is the transport, the session and the schedule.
#pragma once

#include <stdint.h>

namespace Obd {

// Unpaired: OBD is on but no dongle is stored, and none is looked for until the owner asks.
enum State : uint8_t { Off = 0, Scanning, Connecting, Setup, Live, Asleep, Unpaired, Pairing };

void start();              // loads the stored on/off and dongle, starts the task
void setEnabled(bool on);  // stored in NVS
bool enabled();
bool paired();
void forgetDongle();       // drops the stored dongle and the link, whether or not OBD is on
// Pairing is a deliberate act (owner's decision D3, 24 Nov 2026): for 30 s the dash looks for a
// dongle, connects, and stores it only once the car has answered a read through it. Ignored
// while a dongle is stored.
void requestPairing();
void setTrafficLog(bool on);  // every request and reply on serial
bool trafficLog();
void printStatus();        // serial: state, adapter, characteristics, rates, per-identifier counts
const char *stateName(uint8_t s);

}  // namespace Obd
