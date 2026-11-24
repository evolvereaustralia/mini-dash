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
constexpr int MAX_ACTIVE = 4;  // checkpoints that may wait in /active: the running trip and unarchived ones
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

// Mounts. A partition that has never held a littlefs is formatted once; one that has and fails
// to mount is left alone, storage stays off for the boot, and needsOwner() is true.
bool begin();
bool needsOwner();
bool formatNow();       // the deliberate format (serial KK): every trip and trace is erased
uint32_t faultCount();  // writes that failed or were refused since boot
// Bench self-test (serial W): one block written through the same safe path as every trip file,
// read back and removed. It never touches /trips. removeTestTrips clears the test trips older
// firmware left in the history (ids from 900000 up); those ids are ignored when numbering trips.
bool selfTest();
int quarantineUnreadableTrips();  // boot: unreadable trip files renamed to <id>.bad, kept, no longer counted
int removeTestTrips();
uint32_t writeCount();  // flash writes (files and NVS) since boot, for the diagnostics line
bool fsOk();
size_t fsUsedBytes();
size_t fsTotalBytes();

void loadSettings(Settings &s);
bool saveSettings(const Settings &s);  // true only if NVS accepted it

void loadLifetime(RecordSet &all);
bool saveLifetime(const RecordSet &all);
void clearLifetime();

// A new trip number, stored before it is used and always above every trip and checkpoint on
// the disk, so a failed write can never hand out a number that already has files.
uint32_t reserveTripId();
int listActive(uint32_t *ids, int max);  // checkpoints waiting in /active, ascending

// OBD: on/off and the paired dongle (address as text, BLE address type). Separate keys, not
// part of the settings blob, so pairing never touches the calibration.
// The car's forward axis in the IMU frame (board_config.h IMU_AXES), likewise its own key.
uint8_t loadForwardAxis(uint8_t fallback);
bool saveForwardAxis(uint8_t axis);
bool loadObdEnabled();
void saveObdEnabled(bool on);
// The paired dongle as ONE record, so an address can never be stored with another dongle's
// address type (review finding 39). An empty address is the explicit "none" state, written
// by Forget, so an older build's keys cannot bring a forgotten dongle back.
struct ObdBinding {
  uint8_t version = 1;
  uint8_t type = 0;      // BLE address type
  char addr[18] = "";    // "aa:bb:cc:dd:ee:ff"
  char name[24] = "";
};
bool loadObdBinding(ObdBinding &out);   // true when a dongle is stored; migrates the two-key form once
bool saveObdBinding(const ObdBinding &b);  // true only if the write was accepted
bool clearObdBinding();

// Completed trips, newest first. index 0 is the most recent.
int tripCount();
bool loadTrip(int indexFromNewest, TripSummary &out);
bool saveTrip(const TripSummary &t);  // rotates beyond MAX_TRIPS (oldest deleted with its events)
void deleteEvents(uint32_t tripId);   // for a trip that is discarded rather than saved
int pruneOrphanEvents();              // at boot: remove event files whose trip has no trip file (demo trips, power cuts)

// The running trip's checkpoint (every CHECKPOINT_MS), one file per trip. A checkpoint found at
// boot is a trip ended by power loss: it is archived and only then removed; if archiving
// fails it stays for the next boot.
bool saveActive(const TripSummary &t);
bool loadActive(uint32_t id, TripSummary &out);
void clearActive(uint32_t id);
void quarantineActive(uint32_t id);  // unreadable: set aside, not retried, not destroyed
bool loadLegacyActive(TripSummary &out);  // the single /active.trp of builds before 24 Nov 2026
void clearLegacyActive();

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
