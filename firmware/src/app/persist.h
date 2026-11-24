// The one owner of flash writes for trips, records, traces and settings (review findings 02,
// 04, 06, 07, 08, 17, 28 and the storage notes, 24 Nov 2026). Everything here that writes runs
// on the Arduino loop task, from service(). The UI, the sensor task and the trip engine never
// write: they leave a request, or a finished trace in a slot, and service() picks it up. So a
// tap can no longer stall the display on flash I/O, two tasks can no longer write the same
// file, and nothing is written from an engine that is changing underneath the writer.
//
// What is copied under which lock: the trip engine is read and rotated only under the state
// lock (VsLock) and the copy is written after the lock is released. Trace slots have their
// own short mutex, taken by the engine's task while it holds the state lock and by the writer
// on its own; it is never held across a write.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "storage.h"

class TripEngine;

namespace Persist {

// Setup, in this order, before the sensor and demo tasks exist.
const Storage::Settings &loadSettings();
const Storage::Settings &settings();  // as last loaded or stored; loop task
// Archives checkpoints left by power cuts and loads the lifetime set. In capture mode nothing
// stored is touched: no recovery, no pruning, and no trip, record or trace is written after.
void begin(TripEngine *engine, bool captureMode);
void startFirstTrip();

void service(uint32_t nowMs);  // loop task

// Requests from any task. None touches flash; each is carried out by service().
void requestSettings(bool mph, int8_t nightMode, bool demo, uint8_t bestField);
void requestDemo(bool on);      // closes the running trip properly, then switches the source
void requestResetTrip();
void requestClearRecords();
void requestStats();           // refresh the storage figures shown in Settings

bool loadTrace(uint32_t tripId, int rec, Storage::TraceSample *out, size_t max, size_t *n);  // a read; any task

// Bench switches (serial keys V and I) and the capture build's master switch.
void setSuppressed(bool on);
bool suppressed();
void setImmediate(bool on);
bool immediate();
void setTripWrites(bool on);  // off: no trip, record or trace is written (capture builds)
bool tripWrites();
int pendingRecovery();        // checkpoints from earlier boots still waiting to be archived

// Writes five test traces and removes them (the bench stall test); loop task only.
uint32_t stallTestWrites();

}  // namespace Persist
