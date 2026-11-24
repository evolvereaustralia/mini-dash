#include "storage.h"

#include <Arduino.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_partition.h"

#include "clock.h"

namespace {
constexpr const char *NVS_NS = "minidash";
constexpr const char *FS_LABEL = "ffat";  // the 9 MB data partition of app3M_fat9M_16MB
constexpr const char *FS_MOUNT = "/lfs";  // where LittleFS is mounted in the VFS, for the POSIX calls below
constexpr uint32_t TRIP_MAGIC = 0x54524950;  // "TRIP"
constexpr uint32_t TEST_ID_BASE = 900000;  // trips written by the old self-test; never part of the sequence
// Trip file versions. 1: braking record was 100-0. 2: braking record is 60-0. 3: the same body
// as 2 followed by a CRC-32 of everything before it (review finding 37). Versions 1 and 2 are
// still read, by their exact size; they carry no checksum, so only their size, magic and
// version can be tested.
constexpr uint16_t TRIP_VERSION = 3;
constexpr uint32_t EVENT_MAGIC = 0x45564E54;   // "EVNT": header and samples, no checksum (read only)
constexpr uint32_t EVENT_MAGIC2 = 0x45564E32;  // "EVN2": header, samples, CRC-32 of both

// The stored layouts are these structs byte for byte. If one of these fails, a field was added
// or moved: give the file a new version (and say how the old one is read) before changing the
// number here. A layout of the same size cannot be told apart by the reader, so it is caught
// here, at build time, or not at all.
static_assert(sizeof(Record) == 52, "Record layout changed: bump TRIP_VERSION and the lifetime version");
static_assert(sizeof(RecordSet) == 312, "RecordSet layout changed: bump TRIP_VERSION and the lifetime version");
static_assert(sizeof(Storage::TripSummary) == 424, "TripSummary layout changed: bump TRIP_VERSION");
static_assert(sizeof(Storage::TraceSample) == 10, "TraceSample layout changed: new event magic needed");

uint32_t crc32(const void *data, size_t n, uint32_t crc = 0) {
  const uint8_t *p = static_cast<const uint8_t *>(data);
  crc = ~crc;
  for (size_t i = 0; i < n; i++) {
    crc ^= p[i];
    for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1)));
  }
  return ~crc;
}

Preferences g_prefs;
bool g_fsOk = false, g_fsNeedsOwner = false;
uint32_t g_writes = 0, g_faults = 0;

struct TripFile {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  Storage::TripSummary body;
  uint32_t crc;  // version 3 on; versions 1 and 2 end before it
};
constexpr size_t TRIP_FILE_LEGACY = offsetof(TripFile, crc);

// A durable replacement of one file (review finding 03). The LittleFS port in this build syncs a
// file the moment it is opened for writing, so opening the target itself commits an EMPTY file
// before a byte is written: a power cut there used to lose the last good copy. Instead the new
// content goes to a second file beside the target, with every result checked; it is read back
// and compared; only then is it renamed over the target, which littlefs does atomically. A cut
// at any point leaves either the old complete file or the new complete file. POSIX calls are
// used because Arduino's File::close() and flush() throw the results away.
bool writeBlock(const char *path, const void *data, size_t n) {
  g_writes++;
  char full[64], tmp[72];
  snprintf(full, sizeof(full), "%s%s", FS_MOUNT, path);
  snprintf(tmp, sizeof(tmp), "%s.tmp", full);
  bool ok = false;
  int fd = open(tmp, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd >= 0) {
    const uint8_t *b = static_cast<const uint8_t *>(data);
    size_t done = 0;
    while (done < n) {
      ssize_t w = write(fd, b + done, n - done);
      if (w <= 0) break;
      done += (size_t)w;
    }
    bool synced = done == n && fsync(fd) == 0;
    bool closed = close(fd) == 0;
    ok = synced && closed;
  }
  if (ok) {  // read back and compare before the old copy is given up
    ok = false;
    int rd = open(tmp, O_RDONLY);
    if (rd >= 0) {
      const uint8_t *b = static_cast<const uint8_t *>(data);
      uint8_t chunk[128];
      size_t done = 0;
      bool same = true;
      while (same && done < n) {
        size_t want = n - done < sizeof(chunk) ? n - done : sizeof(chunk);
        ssize_t r = read(rd, chunk, want);
        if (r != (ssize_t)want || memcmp(chunk, b + done, want) != 0) same = false;
        done += want;
      }
      uint8_t extra;
      if (same && read(rd, &extra, 1) != 0) same = false;  // nothing may follow
      close(rd);
      ok = same;
    }
  }
  if (ok) ok = rename(tmp, full) == 0;
  if (!ok) {
    g_faults++;
    unlink(tmp);
    Serial.printf("storage: write of %s FAILED (errno %d); the previous copy, if any, is untouched\n", path, errno);
  }
  return ok;
}

bool readTrip(const char *path, Storage::TripSummary &out) {
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  TripFile tf;
  size_t total = f.size();
  size_t r = f.read(reinterpret_cast<uint8_t *>(&tf), sizeof(tf));
  f.close();
  // A file is the whole record for its version or it is nothing: no prefix, nothing appended.
  if (r != total || r < TRIP_FILE_LEGACY || tf.magic != TRIP_MAGIC || tf.size != sizeof(Storage::TripSummary)) return false;
  if (tf.version == TRIP_VERSION) {
    if (total != sizeof(tf) || tf.crc != crc32(&tf, TRIP_FILE_LEGACY)) return false;
  } else if (tf.version == 1 || tf.version == 2) {
    if (total != TRIP_FILE_LEGACY) return false;
  } else {
    return false;
  }
  out = tf.body;
  if (tf.version == 1) out.records.r[REC_60_0] = Record();  // a 100-0 figure: not comparable
  return true;
}

bool writeTrip(const char *path, const Storage::TripSummary &t) {
  static TripFile tf;  // the loop task is the only writer; static keeps 430 bytes off its stack
  memset(&tf, 0, sizeof(tf));  // padding bytes are part of the checksum, so they are defined
  tf.magic = TRIP_MAGIC;
  tf.version = TRIP_VERSION;
  tf.size = (uint16_t)sizeof(Storage::TripSummary);
  tf.body = t;
  tf.crc = crc32(&tf, TRIP_FILE_LEGACY);
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
    size_t len = strlen(base);
    bool isTrip = len > 4 && strcmp(base + len - 4, ".trp") == 0;  // the name ends in .trp: not .trp.tmp, not .bad
    if (id && isTrip && n < max) ids[n++] = id;
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

// Checkpoint ids in /active, ascending.
int listActiveIds(uint32_t *ids, int max) {
  int n = 0;
  File dir = LittleFS.open("/active");
  if (!dir || !dir.isDirectory()) return 0;
  File e;
  while ((e = dir.openNextFile())) {
    const char *name = e.name();
    const char *base = strrchr(name, '/');
    base = base ? base + 1 : name;
    uint32_t id = strtoul(base, nullptr, 10);
    const char *dot = strrchr(base, '.');
    if (id && dot && strcmp(dot, ".trp") == 0 && n < max) ids[n++] = id;
    e.close();
  }
  dir.close();
  for (int i = 1; i < n; i++) {
    uint32_t v = ids[i];
    int j = i - 1;
    while (j >= 0 && ids[j] > v) {
      ids[j + 1] = ids[j];
      j--;
    }
    ids[j + 1] = v;
  }
  return n;
}

void tripPath(char *buf, size_t n, uint32_t id) { snprintf(buf, n, "/trips/%lu.trp", (unsigned long)id); }
void activePath(char *buf, size_t n, uint32_t id) { snprintf(buf, n, "/active/%lu.trp", (unsigned long)id); }
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

namespace {
// Does the partition already hold a littlefs? Its superblock carries the ASCII tag "littlefs" in
// one of the first two blocks. Used to tell a partition that has never been set up from one
// that has data and merely failed to mount.
// True only if the data partition is certainly unused: found, readable, and its first two
// blocks (where littlefs keeps both copies of its superblock) fully erased. Anything else,
// a littlefs that will not mount, some other content, a read error, counts as "may hold data"
// and is never formatted without the owner's say-so.
bool partitionIsBlank() {
  const esp_partition_t *part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, FS_LABEL);
  if (!part) return false;
  static uint8_t block[512];
  for (uint32_t off = 0; off < 2 * 4096; off += sizeof(block)) {
    if (esp_partition_read(part, off, block, sizeof(block)) != ESP_OK) return false;
    for (size_t k = 0; k < sizeof(block); k++)
      if (block[k] != 0xFF) return false;
  }
  return true;
}

void removeLeftovers(const char *dirPath) {  // second files left by a cut during a write
  File dir = LittleFS.open(dirPath);
  if (!dir || !dir.isDirectory()) return;
  char victims[8][48];
  int v = 0;
  File e;
  while (v < 8 && (e = dir.openNextFile())) {
    const char *name = e.name();
    const char *base = strrchr(name, '/');
    base = base ? base + 1 : name;
    if (strstr(base, ".tmp")) snprintf(victims[v++], sizeof(victims[0]), "%s/%s", dirPath, base);
    e.close();
  }
  dir.close();
  for (int i = 0; i < v; i++) LittleFS.remove(victims[i]);
}
}  // namespace

bool begin() {
  g_prefs.begin(NVS_NS, false);
  // A mount failure must never format a partition that has trips on it (review finding 05):
  // format only when the partition is blank. Otherwise storage stays off for this boot,
  // nothing is erased, and formatting takes a deliberate serial command.
  g_fsOk = LittleFS.begin(false, FS_MOUNT, 8, FS_LABEL);
  if (!g_fsOk) {
    if (partitionIsBlank()) {
      Serial.println("storage: the data partition is blank, formatting it once");
      g_fsOk = LittleFS.begin(true, FS_MOUNT, 8, FS_LABEL);
    } else {
      g_fsNeedsOwner = true;
      g_faults++;
      Serial.println("storage: the data partition is not blank and did NOT mount. Nothing was erased. Trips and traces are off "
                     "this boot; if this repeats, the serial command KK formats the partition (all trips lost).");
    }
  }
  if (g_fsOk) {
    for (const char *d : {"/trips", "/events", "/active"})
      if (!LittleFS.exists(d)) LittleFS.mkdir(d);
    for (const char *d : {"/trips", "/events", "/active", "/"}) removeLeftovers(d);
    Serial.printf("storage: LittleFS on '%s', %u KB used of %u KB, %d trips\n", FS_LABEL,
                  (unsigned)(LittleFS.usedBytes() / 1024), (unsigned)(LittleFS.totalBytes() / 1024), tripCount());
  }
  return g_fsOk;
}

bool formatNow() {
  LittleFS.end();  // the label is kept from begin(), so format() addresses the same partition whether or not it mounted
  g_fsOk = LittleFS.format() && LittleFS.begin(false, FS_MOUNT, 8, FS_LABEL);
  if (g_fsOk) {
    for (const char *d : {"/trips", "/events", "/active"}) LittleFS.mkdir(d);
    g_fsNeedsOwner = false;
  }
  Serial.printf("storage: format %s\n", g_fsOk ? "done, the partition is empty" : "FAILED");
  return g_fsOk;
}

bool needsOwner() { return g_fsNeedsOwner; }
uint32_t faultCount() { return g_faults; }
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

bool saveSettings(const Settings &s) {
  g_writes++;
  bool ok = g_prefs.putBytes("set", &s, sizeof(s)) == sizeof(s) && g_prefs.putUChar("set_v", s.version) == 1;
  if (!ok) g_faults++;
  return ok;
}

void loadLifetime(RecordSet &all) {
  // The braking slot held 100-0 figures before 24 Nov 2026: drop a stored one, once. The flag
  // is set even when nothing is stored, so a 60-0 record saved later is never dropped.
  bool migrated = g_prefs.getBool("brk60", false);
  if (g_prefs.getUChar("life_v", 0) == 1) {
    RecordSet tmp;
    if (g_prefs.getBytes("life", &tmp, sizeof(tmp)) == sizeof(tmp)) {
      if (!migrated) {
        tmp.r[REC_60_0] = Record();
        g_prefs.putBytes("life", &tmp, sizeof(tmp));
      }
      all = tmp;
    }
  }
  if (!migrated) g_prefs.putBool("brk60", true);
}

bool saveLifetime(const RecordSet &all) {
  g_writes++;
  bool ok = g_prefs.putBytes("life", &all, sizeof(all)) == sizeof(all) && g_prefs.putUChar("life_v", 1) == 1;
  if (!ok) g_faults++;
  return ok;
}

void clearLifetime() {
  g_prefs.remove("life");
  g_prefs.remove("life_v");
}

bool selfTest() {
  if (!g_fsOk) return false;
  static uint8_t block[1200];
  for (size_t i = 0; i < sizeof(block); i++) block[i] = (uint8_t)(i * 31 + 7);
  bool ok = writeBlock("/selftest.bin", block, sizeof(block));  // written, synced, read back, renamed
  File f = LittleFS.open("/selftest.bin", "r");
  bool sized = f && f.size() == sizeof(block);
  if (f) f.close();
  LittleFS.remove("/selftest.bin");
  return ok && sized;
}
int removeTestTrips() {
  if (!g_fsOk) return 0;
  uint32_t ids[MAX_TRIPS + 4];
  int n = listTrips(ids, MAX_TRIPS + 4), removed = 0;
  for (int i = 0; i < n; i++)
    if (ids[i] >= TEST_ID_BASE) {
      deleteTripAndEvents(ids[i]);
      removed++;
    }
  return removed;
}

uint32_t reserveTripId() {
  uint32_t id = g_prefs.getULong("trip_id", 0);
  if (g_fsOk) {  // the counter is checked against what is actually on the disk
    uint32_t ids[MAX_TRIPS + 4];
    int n = listTrips(ids, MAX_TRIPS + 4);
    for (int i = 0; i < n; i++)  // newest first; test trips from older firmware do not count
      if (ids[i] < TEST_ID_BASE) {
        if (ids[i] > id) id = ids[i];
        break;
      }
    uint32_t act[MAX_ACTIVE];
    int a = listActive(act, MAX_ACTIVE);
    for (int i = 0; i < a; i++)
      if (act[i] > id && act[i] < TEST_ID_BASE) id = act[i];
  }
  id++;
  g_writes++;
  if (g_prefs.putULong("trip_id", id) != sizeof(uint32_t) || g_prefs.getULong("trip_id", 0) != id) {
    g_faults++;
    Serial.println("storage: the trip counter could not be stored; this trip's number is taken from the files on disk");
  }
  return id;
}

int listActive(uint32_t *ids, int max) { return g_fsOk ? listActiveIds(ids, max) : 0; }

bool loadCaptureMode() { return g_prefs.getBool("capture", true); }
bool saveCaptureMode(bool on) {
  g_writes++;
  bool ok = g_prefs.putBool("capture", on) == 1;
  if (!ok) g_faults++;
  return ok;
}
uint8_t loadForwardAxis(uint8_t fallback) {
  uint8_t a = g_prefs.getUChar("fwd_axis", fallback);
  return a < 6 ? a : fallback;
}
bool saveForwardAxis(uint8_t axis) {
  g_writes++;
  bool ok = axis < 6 && g_prefs.putUChar("fwd_axis", axis) == 1;
  if (!ok) g_faults++;
  return ok;
}
bool loadObdEnabled() { return g_prefs.getBool("obd_on", true); }
void saveObdEnabled(bool on) {
  g_writes++;
  g_prefs.putBool("obd_on", on);
}
bool saveObdBinding(const ObdBinding &b) {
  g_writes++;
  return g_prefs.putBytes("obd_bind", &b, sizeof(b)) == sizeof(b);
}
bool clearObdBinding() {
  ObdBinding none;
  bool ok = saveObdBinding(none);
  g_prefs.remove("obd_addr");
  g_prefs.remove("obd_type");
  return ok;
}
bool loadObdBinding(ObdBinding &out) {
  out = ObdBinding();
  ObdBinding b;
  if (g_prefs.isKey("obd_bind")) {
    if (g_prefs.getBytes("obd_bind", &b, sizeof(b)) != sizeof(b) || b.version != 1) return false;
    b.addr[sizeof(b.addr) - 1] = 0;
    b.name[sizeof(b.name) - 1] = 0;
    out = b;
    return out.addr[0] != 0;
  }
  // The form used until 24 Nov 2026: address and type as two keys. Carried over once; the old
  // keys go only after the new record is in place.
  if (!g_prefs.isKey("obd_addr")) return false;
  char addr[20] = "";
  g_prefs.getString("obd_addr", addr, sizeof(addr));
  if (!addr[0] || strlen(addr) >= sizeof(b.addr)) return false;
  strncpy(b.addr, addr, sizeof(b.addr) - 1);
  b.type = g_prefs.getUChar("obd_type", 0);
  if (saveObdBinding(b)) {
    g_prefs.remove("obd_addr");
    g_prefs.remove("obd_type");
  }
  out = b;
  return true;
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
// Trip files that cannot be read (damaged, or from a layout this build does not know) are set
// aside as <id>.bad so they stop counting as trips and stop holding a place among the 60, and
// are not destroyed. Run once at boot before any task writes.
int quarantineUnreadableTrips() {
  if (!g_fsOk) return 0;
  uint32_t ids[MAX_TRIPS + 4];
  int n = listTrips(ids, MAX_TRIPS + 4), moved = 0;
  static TripSummary t;
  for (int i = 0; i < n; i++) {
    char path[40], bad[40];
    tripPath(path, sizeof(path), ids[i]);
    if (readTrip(path, t)) continue;
    snprintf(bad, sizeof(bad), "/trips/%lu.bad", (unsigned long)ids[i]);
    LittleFS.remove(bad);
    if (LittleFS.rename(path, bad)) moved++;
    g_faults++;
    Serial.printf("storage: trip file %lu cannot be read; set aside as %lu.bad\n", (unsigned long)ids[i], (unsigned long)ids[i]);
  }
  return moved;
}

int pruneOrphanEvents() {
  if (!g_fsOk) return 0;
  uint32_t ids[MAX_TRIPS + 4 + MAX_ACTIVE];
  int n = listTrips(ids, MAX_TRIPS + 4);
  n += listActiveIds(ids + n, MAX_ACTIVE);  // a trip not yet archived keeps its traces
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

// The running trip's checkpoint, one file per trip in /active (review finding 04): a trip that
// could not be archived keeps its checkpoint for the next boot, and the next trip cannot
// overwrite it. The single file /active.trp used until 24 Nov 2026 is still recognised.
bool saveActive(const TripSummary &t) {
  if (!g_fsOk) return false;
  char path[40];
  activePath(path, sizeof(path), t.id);
  return writeTrip(path, t);
}
bool loadActive(uint32_t id, TripSummary &out) {
  if (!g_fsOk) return false;
  char path[40];
  activePath(path, sizeof(path), id);
  return LittleFS.exists(path) && readTrip(path, out);
}
void clearActive(uint32_t id) {
  if (!g_fsOk) return;
  char path[40];
  activePath(path, sizeof(path), id);
  LittleFS.remove(path);
}
// Sets a checkpoint aside when it cannot be read, so it is neither retried at every boot nor
// silently destroyed. At most three are kept.
void quarantineActive(uint32_t id) {
  if (!g_fsOk) return;
  char from[40], to[40];
  activePath(from, sizeof(from), id);
  snprintf(to, sizeof(to), "/active/%lu.bad", (unsigned long)id);
  LittleFS.rename(from, to);
}
bool loadLegacyActive(TripSummary &out) { return g_fsOk && LittleFS.exists("/active.trp") && readTrip("/active.trp", out); }
void clearLegacyActive() {
  if (g_fsOk) LittleFS.remove("/active.trp");
}

namespace {
struct EventFile {
  uint32_t magic;
  uint32_t count;
  TraceSample samples[TRACE_MAX];
  uint32_t crcRoom;  // the CRC follows the last sample actually stored, wherever that is
};
}  // namespace

bool saveEvent(uint32_t tripId, int rec, const TraceSample *samples, size_t n) {
  if (!g_fsOk || n == 0 || n > TRACE_MAX) return false;
  char path[40];
  eventPath(path, sizeof(path), tripId, rec);
  static EventFile ef;  // the loop task is the only writer
  ef.magic = EVENT_MAGIC2;
  ef.count = (uint32_t)n;
  memcpy(ef.samples, samples, n * sizeof(TraceSample));
  size_t body = 2 * sizeof(uint32_t) + n * sizeof(TraceSample);
  uint32_t crc = crc32(&ef, body);
  memcpy(reinterpret_cast<uint8_t *>(&ef) + body, &crc, sizeof(crc));
  return writeBlock(path, &ef, body + sizeof(crc));
}

// A trace is either the whole file as written or nothing: a count beyond capacity, a file
// shorter or longer than its count says, or a short read all mean "no trace".
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
  size_t total = f.size();
  bool ok = f.read(reinterpret_cast<uint8_t *>(&hdr), sizeof(hdr)) == sizeof(hdr) && (hdr.magic == EVENT_MAGIC || hdr.magic == EVENT_MAGIC2) &&
            hdr.count >= 1 && hdr.count <= TRACE_MAX && hdr.count <= max;
  bool checked = ok && hdr.magic == EVENT_MAGIC2;
  size_t bytes = ok ? hdr.count * sizeof(TraceSample) : 0;
  if (ok) ok = total == sizeof(hdr) + bytes + (checked ? sizeof(uint32_t) : 0);
  if (ok) ok = f.read(reinterpret_cast<uint8_t *>(samples), bytes) == bytes;
  if (ok && checked) {
    uint32_t stored = 0;
    ok = f.read(reinterpret_cast<uint8_t *>(&stored), sizeof(stored)) == sizeof(stored) && stored == crc32(samples, bytes, crc32(&hdr, sizeof(hdr)));
  }
  f.close();
  if (ok) *n = hdr.count;
  return ok;
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
      bool lower = r <= REC_60_0;
      Record &cur = best.r[r];
      uint32_t runs = (uint32_t)(cur.valid ? cur.runs : 0) + cand.runs;
      if (!cur.valid || (lower ? cand.value < cur.value : cand.value > cur.value)) {
        cur = cand;
        cur.beatLifetime = false;
        cur.hasPrevious = false;
      }
      cur.runs = runs > 65535 ? 65535 : (uint16_t)runs;
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
