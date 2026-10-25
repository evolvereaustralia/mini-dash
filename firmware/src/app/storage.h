// Persistent state (CLAUDE.md section 5). NVS (Preferences) for settings, the mounting
// calibration and the all-time records, written atomically (value, then a version key).
// LittleFS on the 9 MB internal flash partition the app3M_fat9M scheme sets aside (label
// "ffat", formatted as LittleFS on first use) for bounded per-trip summaries (last 60), the
// active-trip checkpoint and 12 s event traces. Nothing on the TF card.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "vehicle_state.h"

namespace Storage {

constexpr int MAX_TRIPS = 60;
constexpr uint32_t CHECKPOINT_MS = 30000;

struct Settings {
  uint8_t version = 2;  // 2: accelerometer offset calibration added (25 Oct 2026)
  bool mph = false;
  int8_t nightMode = -1;  // -1 auto, 0 day, 1 night
  uint8_t brightness = 100;
  uint8_t startScreen = 0;  // 0 Drive, 1 last page
  uint8_t bestField = 0;
  bool demo = false;  // the replay is a bench tool; off by default from the first drive (25 Oct 2026)
  bool calibrated = false;
  float gRef[3] = {0, 0, 1};
  bool accCalibrated = false;       // sphere-fit offsets taken (docs/calibration.md)
  float accOffset[3] = {0, 0, 0};   // g, subtracted from every raw sample
  float accRadius = 1;              // g, the raw magnitude at rest; samples are divided by it
};

// One completed (or checkpointed) trip. Plain data, written as one block with a header.
struct TripSummary {
  uint32_t id = 0;
  uint32_t startUtc = 0;  // 0 when no clock was available
  uint32_t endUtc = 0;
  uint32_t durationS = 0;
  bool demo = false;
  bool closed = false;  // false while this is the active checkpoint
  TripData trip;
  RecordSet records;
  MotionData peaks;
};

// A trace sample at 10 Hz: 10 bytes. t10 is tenths of a second from the window start.
struct TraceSample {
  uint16_t t10;
  int16_t kph10;  // km/h x 10
  int16_t latMg;  // milli-g
  int16_t lonMg;
  int16_t kw10;   // kW x 10
};
constexpr size_t TRACE_MAX = 120;  // 12 s

bool begin();  // mounts; returns false if the file system is unusable (NVS still works)
uint32_t writeCount();  // flash writes (files and NVS) since boot, for the diagnostics line
bool fsOk();
size_t fsUsedBytes();
size_t fsTotalBytes();

void loadSettings(Settings &s);
void saveSettings(const Settings &s);

void loadLifetime(RecordSet &all);
void saveLifetime(const RecordSet &all);
void clearLifetime();

uint32_t nextTripId();  // monotonic, persisted in NVS

// Completed trips, newest first. index 0 is the most recent.
int tripCount();
bool loadTrip(int indexFromNewest, TripSummary &out);
bool saveTrip(const TripSummary &t);  // rotates beyond MAX_TRIPS (oldest deleted with its events)
void deleteEvents(uint32_t tripId);   // for a trip that is discarded rather than saved
int pruneOrphanEvents();              // at boot: remove event files whose trip has no trip file (demo trips, power cuts)

// Active trip checkpoint (every CHECKPOINT_MS). At boot, a checkpoint found on disk is the
// previous trip ended by power loss: finalise it with closeActive().
bool saveActive(const TripSummary &t);
bool loadActive(TripSummary &out);
void clearActive();

bool saveEvent(uint32_t tripId, int rec, const TraceSample *samples, size_t n);
bool loadEvent(uint32_t tripId, int rec, TraceSample *samples, size_t max, size_t *n);

// Aggregates. Local-day comparison uses the Melbourne offset for the given UTC.
struct DayStats {
  uint16_t trips = 0;
  float km = 0;
};
void aggregateDay(uint32_t dayUtc, RecordSet &best, DayStats &stats, uint32_t excludeTripId = 0);
void aggregateAll(DayStats &stats);
bool sameLocalDay(uint32_t utcA, uint32_t utcB);

}  // namespace Storage
