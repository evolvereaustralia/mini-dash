#include "storage.h"

#include <Arduino.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <string.h>

#include "clock.h"

namespace {
constexpr const char *NVS_NS = "minidash";
constexpr const char *FS_LABEL = "ffat";  // the 9 MB data partition of app3M_fat9M_16MB
constexpr uint32_t TRIP_MAGIC = 0x54524950;  // "TRIP"
constexpr uint16_t TRIP_VERSION = 1;
constexpr uint32_t EVENT_MAGIC = 0x45564E54;  // "EVNT"

Preferences g_prefs;
bool g_fsOk = false;
uint32_t g_writes = 0;

struct TripFile {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  Storage::TripSummary body;
};

bool writeBlock(const char *path, const void *data, size_t n) {
  // LittleFS updates a file atomically on close (copy on write), so a power cut leaves the
  // old or the new content. No temporary file and rename: each extra step costs flash erases,
  // and every erase stalls the panel refill (see main.cpp, deferred writes).
  g_writes++;
  File f = LittleFS.open(path, "w");
  if (!f) return false;
  size_t w = f.write(static_cast<const uint8_t *>(data), n);
  f.close();
  return w == n;
}

bool readTrip(const char *path, Storage::TripSummary &out) {
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  TripFile tf;
  size_t r = f.read(reinterpret_cast<uint8_t *>(&tf), sizeof(tf));
  f.close();
  if (r != sizeof(tf) || tf.magic != TRIP_MAGIC || tf.version != TRIP_VERSION || tf.size != sizeof(Storage::TripSummary)) return false;
  out = tf.body;
  return true;
}

bool writeTrip(const char *path, const Storage::TripSummary &t) {
  TripFile tf = {TRIP_MAGIC, TRIP_VERSION, (uint16_t)sizeof(Storage::TripSummary), t};
  return writeBlock(path, &tf, sizeof(tf));
}

// Trip ids in /trips, sorted descending (newest first). Returns count.
int listTrips(uint32_t *ids, int max) {
  int n = 0;
  File dir = LittleFS.open("/trips");
  if (!dir || !dir.isDirectory()) return 0;
  File e;
  while ((e = dir.openNextFile())) {
    const char *name = e.name();
    const char *base = strrchr(name, '/');
    base = base ? base + 1 : name;
    uint32_t id = strtoul(base, nullptr, 10);
    if (id && strstr(base, ".trp") && n < max) ids[n++] = id;
    e.close();
  }
  dir.close();
  for (int i = 1; i < n; i++) {  // insertion sort, descending
    uint32_t v = ids[i];
    int j = i - 1;
    while (j >= 0 && ids[j] < v) {
      ids[j + 1] = ids[j];
      j--;
    }
    ids[j + 1] = v;
  }
  return n;
}

void tripPath(char *buf, size_t n, uint32_t id) { snprintf(buf, n, "/trips/%lu.trp", (unsigned long)id); }
void eventPath(char *buf, size_t n, uint32_t id, int rec) { snprintf(buf, n, "/events/%lu-%d.evt", (unsigned long)id, rec); }

void deleteTripAndEvents(uint32_t id) {
  char path[40];
  tripPath(path, sizeof(path), id);
  LittleFS.remove(path);
  for (int r = 0; r < REC_COUNT; r++) {
    eventPath(path, sizeof(path), id, r);
    LittleFS.remove(path);
  }
}
}  // namespace

namespace Storage {

bool begin() {
  g_prefs.begin(NVS_NS, false);
  g_fsOk = LittleFS.begin(true, "/lfs", 8, FS_LABEL);
  if (g_fsOk) {
    if (!LittleFS.exists("/trips")) LittleFS.mkdir("/trips");
    if (!LittleFS.exists("/events")) LittleFS.mkdir("/events");
    Serial.printf("storage: LittleFS on '%s', %u KB used of %u KB, %d trips\n", FS_LABEL,
                  (unsigned)(LittleFS.usedBytes() / 1024), (unsigned)(LittleFS.totalBytes() / 1024), tripCount());
  } else {
    Serial.println("storage: LittleFS mount FAILED, trips and events will not persist");
  }
  return g_fsOk;
}

bool fsOk() { return g_fsOk; }
uint32_t writeCount() { return g_writes; }
size_t fsUsedBytes() { return g_fsOk ? LittleFS.usedBytes() : 0; }
size_t fsTotalBytes() { return g_fsOk ? LittleFS.totalBytes() : 0; }

void loadSettings(Settings &s) {
  Settings d;
  if (g_prefs.getUChar("set_v", 0) != d.version) return;  // nothing valid stored
  size_t n = g_prefs.getBytes("set", &s, sizeof(s));
  if (n != sizeof(s)) s = d;
}

void saveSettings(const Settings &s) {
  g_writes++;
  g_prefs.putBytes("set", &s, sizeof(s));
  g_prefs.putUChar("set_v", s.version);
}

void loadLifetime(RecordSet &all) {
  if (g_prefs.getUChar("life_v", 0) != 1) return;
  RecordSet tmp;
  if (g_prefs.getBytes("life", &tmp, sizeof(tmp)) == sizeof(tmp)) all = tmp;
}

void saveLifetime(const RecordSet &all) {
  g_writes++;
  g_prefs.putBytes("life", &all, sizeof(all));
  g_prefs.putUChar("life_v", 1);
}

void clearLifetime() {
  g_prefs.remove("life");
  g_prefs.remove("life_v");
}

uint32_t nextTripId() {
  uint32_t id = g_prefs.getULong("trip_id", 0) + 1;
  g_prefs.putULong("trip_id", id);
  return id;
}

int tripCount() {
  if (!g_fsOk) return 0;
  uint32_t ids[MAX_TRIPS + 4];
  return listTrips(ids, MAX_TRIPS + 4);
}

bool loadTrip(int indexFromNewest, TripSummary &out) {
  if (!g_fsOk) return false;
  uint32_t ids[MAX_TRIPS + 4];
  int n = listTrips(ids, MAX_TRIPS + 4);
  if (indexFromNewest < 0 || indexFromNewest >= n) return false;
  char path[40];
  tripPath(path, sizeof(path), ids[indexFromNewest]);
  return readTrip(path, out);
}

bool saveTrip(const TripSummary &t) {
  if (!g_fsOk) return false;
  char path[40];
  tripPath(path, sizeof(path), t.id);
  if (!writeTrip(path, t)) return false;
  uint32_t ids[MAX_TRIPS + 4];
  int n = listTrips(ids, MAX_TRIPS + 4);
  for (int i = MAX_TRIPS; i < n; i++) deleteTripAndEvents(ids[i]);
  return true;
}

void deleteEvents(uint32_t tripId) {
  if (!g_fsOk) return;
  char path[40];
  for (int r = 0; r < REC_COUNT; r++) {
    eventPath(path, sizeof(path), tripId, r);
    LittleFS.remove(path);
  }
}

// Event traces are written when a record is accepted, before anyone knows whether the trip
// will be saved. Demo trips never are, and a trip ended by a power cut before its first
// checkpoint leaves its traces behind too. At boot, after the checkpoint has been closed into
// a trip file, every event file whose trip id has no trip file is an orphan. A bounded batch
// per pass; the directory is not iterated while files are removed from it.
int pruneOrphanEvents() {
  if (!g_fsOk) return 0;
  uint32_t ids[MAX_TRIPS + 4];
  int n = listTrips(ids, MAX_TRIPS + 4);
  int removed = 0;
  for (int pass = 0; pass < 16; pass++) {
    char victims[32][40];
    int v = 0;
    File dir = LittleFS.open("/events");
    if (!dir || !dir.isDirectory()) break;
    File e;
    while (v < 32 && (e = dir.openNextFile())) {
      const char *name = e.name();
      const char *base = strrchr(name, '/');
      base = base ? base + 1 : name;
      uint32_t id = strtoul(base, nullptr, 10);
      bool kept = false;
      for (int i = 0; i < n && !kept; i++) kept = ids[i] == id;
      if (!kept) snprintf(victims[v++], sizeof(victims[0]), "/events/%s", base);
      e.close();
    }
    dir.close();
    for (int i = 0; i < v; i++) removed += LittleFS.remove(victims[i]) ? 1 : 0;
    if (v < 32) break;
  }
  if (removed) Serial.printf("storage: %d orphaned event files removed, %u KB used\n", removed, (unsigned)(LittleFS.usedBytes() / 1024));
  return removed;
}

bool saveActive(const TripSummary &t) { return g_fsOk && writeTrip("/active.trp", t); }
bool loadActive(TripSummary &out) { return g_fsOk && LittleFS.exists("/active.trp") && readTrip("/active.trp", out); }
void clearActive() {
  if (g_fsOk) LittleFS.remove("/active.trp");
}

bool saveEvent(uint32_t tripId, int rec, const TraceSample *samples, size_t n) {
  if (!g_fsOk || n == 0) return false;
  char path[40];
  eventPath(path, sizeof(path), tripId, rec);
  struct {
    uint32_t magic;
    uint32_t count;
  } hdr = {EVENT_MAGIC, (uint32_t)n};
  g_writes++;
  File f = LittleFS.open(path, "w");
  if (!f) return false;
  bool ok = f.write(reinterpret_cast<const uint8_t *>(&hdr), sizeof(hdr)) == sizeof(hdr) &&
            f.write(reinterpret_cast<const uint8_t *>(samples), n * sizeof(TraceSample)) == n * sizeof(TraceSample);
  f.close();
  return ok;
}

bool loadEvent(uint32_t tripId, int rec, TraceSample *samples, size_t max, size_t *n) {
  *n = 0;
  if (!g_fsOk) return false;
  char path[40];
  eventPath(path, sizeof(path), tripId, rec);
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  struct {
    uint32_t magic;
    uint32_t count;
  } hdr;
  if (f.read(reinterpret_cast<uint8_t *>(&hdr), sizeof(hdr)) != sizeof(hdr) || hdr.magic != EVENT_MAGIC) {
    f.close();
    return false;
  }
  size_t count = hdr.count < max ? hdr.count : max;
  size_t r = f.read(reinterpret_cast<uint8_t *>(samples), count * sizeof(TraceSample));
  f.close();
  *n = r / sizeof(TraceSample);
  return *n > 0;
}

bool sameLocalDay(uint32_t utcA, uint32_t utcB) {
  if (utcA == 0 || utcB == 0) return false;
  return Clock::localDayNumber(utcA) == Clock::localDayNumber(utcB);
}

void aggregateDay(uint32_t dayUtc, RecordSet &best, DayStats &stats, uint32_t excludeTripId) {
  best = RecordSet();
  stats = DayStats();
  if (!g_fsOk || dayUtc == 0) return;
  uint32_t ids[MAX_TRIPS + 4];
  int n = listTrips(ids, MAX_TRIPS + 4);
  for (int i = 0; i < n; i++) {
    if (ids[i] == excludeTripId) continue;
    char path[40];
    tripPath(path, sizeof(path), ids[i]);
    TripSummary t;
    if (!readTrip(path, t) || t.demo || !sameLocalDay(t.startUtc, dayUtc)) continue;
    stats.trips++;
    stats.km += t.trip.distanceKm;
    for (int r = 0; r < REC_COUNT; r++) {
      const Record &cand = t.records.r[r];
      if (!cand.valid) continue;
      bool lower = r <= REC_100_0;
      Record &cur = best.r[r];
      if (!cur.valid || (lower ? cand.value < cur.value : cand.value > cur.value)) {
        cur = cand;
        cur.beatLifetime = false;
        cur.hasPrevious = false;
      }
      cur.runs += cand.runs;
    }
  }
}

void aggregateAll(DayStats &stats) {
  stats = DayStats();
  if (!g_fsOk) return;
  uint32_t ids[MAX_TRIPS + 4];
  int n = listTrips(ids, MAX_TRIPS + 4);
  for (int i = 0; i < n; i++) {
    char path[40];
    tripPath(path, sizeof(path), ids[i]);
    TripSummary t;
    if (!readTrip(path, t) || t.demo) continue;
    stats.trips++;
    stats.km += t.trip.distanceKm;
  }
}

}  // namespace Storage
