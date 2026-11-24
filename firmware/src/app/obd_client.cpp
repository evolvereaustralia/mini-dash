#include "obd_client.h"

#include <Arduino.h>
#include <BLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include <freertos/task.h>

#include <map>
#include <string>

#include "demo_source.h"
#include "obd_decode.h"
#include "obd_pids.h"
#include "obd_proto.h"
#include "storage.h"
#include "vehicle_state.h"

namespace {
// The time allowed for the adapter's prompt after one request. It is not a bound on a whole
// exchange: a change of ECU first costs up to three configuration commands, and the BLE
// library's discovery and subscription calls wait on the stack without a deadline of ours
// (review finding 29). The second matters at connection time only, on this task only.
constexpr uint32_t REQUEST_TIMEOUT_MS = 300;
constexpr uint32_t SELECT_TIMEOUT_MS = 400;   // one configuration command; about 25 ms in practice
constexpr uint32_t AT_TIMEOUT_MS = 1500;      // set-up commands, where time does not matter
constexpr uint32_t RESET_TIMEOUT_MS = 4000;
constexpr uint32_t SCAN_SECONDS = 4;
constexpr uint32_t CONNECT_TIMEOUT_MS = 6000;
constexpr uint32_t PAIR_WINDOW_MS = 30000;    // how long a Pair request keeps looking
constexpr uint8_t ASLEEP_AFTER = 3;           // consecutive unanswered requests: the car is asleep
constexpr uint8_t RECONNECT_AFTER = 3;        // consecutive requests the dongle itself ignored
constexpr uint32_t ASLEEP_PROBE_MS = 1500, ASLEEP_PROBE_SLOW_MS = 5000;
constexpr uint8_t PROBES_BEFORE_SETUP = 8;    // two rounds of the four probes, then the adapter is set up again
constexpr uint32_t SETUP_AGAIN_MIN_MS = 30000;
constexpr uint32_t MAX_BACKOFF_MS = 12000;
// Periods are targets. What the Drive page shows moving (battery power, wheel speed) is asked
// for as fast as the link allows; torque and pedal (Dynamics page) at about 1 Hz, so they do
// not cost the first two their rate: every change of ECU costs adapter commands.
// Pack voltage moves slowly next to the current, so it is refreshed twice a second and the
// power figure follows the current.
constexpr uint32_t FAST_MS = 150, VOLTS_MS = 500, DRIVE_MS = 900, SLOW_MS = 2000, RANGE_MS = 5000;
constexpr uint32_t DEMOTED_MS = 10000;

// The schedule's defaults are immutable; what a session learns or changes sits beside them and
// is put back at every connection and when an identifier recovers (review finding 16).
struct PollItem {
  const uint8_t ecu;
  const uint16_t did;
  const uint32_t defaultPeriodMs;  // 0: once per connection
  const bool defaultEnabled;
  uint32_t periodMs;
  bool enabled;
  uint32_t lastMs = 0, ok = 0, fail = 0, rttSumMs = 0;
  uint8_t failRun = 0;
  uint8_t frames = 0, countFails = 0;  // frames the answer takes, learnt; 0 until known or if the digit misfired
  bool seen = false, done = false, demoted = false;
  ObdProto::Reply lastReply = ObdProto::Reply::NoData;
  uint8_t lastNrc = 0;
  PollItem(uint8_t e, uint16_t d, uint32_t p, bool en = true) : ecu(e), did(d), defaultPeriodMs(p), defaultEnabled(en), periodMs(p), enabled(en) {}
};

// Fast group first (CLAUDE.md: voltage, current, torque, pedal, wheel speeds), then the slow
// group, then the once-per-connection values. Both wheel-speed sources are listed; the DSC
// one (0.01 km/h) is tried first and the head unit's (1 km/h) takes over if the car refuses it.
PollItem g_items[] = {
    {ECU_SME, DID_SME_VOLTS, VOLTS_MS},
    {ECU_SME, DID_SME_AMPS, FAST_MS},
    {ECU_EME, DID_EME_MACHINE, DRIVE_MS},
    {ECU_DSC, DID_DSC_WHEELS, FAST_MS},
    {ECU_NBT, DID_NBT_WHEELS, FAST_MS, false},
    {ECU_EDM, DID_EDM_PEDAL, DRIVE_MS},
    {ECU_SME, DID_SME_SOC, SLOW_MS},
    {ECU_SME, DID_SME_TEMPS, SLOW_MS},
    {ECU_SME, DID_SME_CELLS, SLOW_MS},
    {ECU_EME, DID_EME_MOTOR_TEMPS, SLOW_MS},
    {ECU_EDM, DID_EDM_IBS, SLOW_MS},
    {ECU_KOM, DID_KOM_RANGE, RANGE_MS},
    {ECU_SME, DID_SME_SOH, 0},
};
constexpr int ITEM_COUNT = sizeof(g_items) / sizeof(g_items[0]);
constexpr int ITEM_VOLTS = 0, ITEM_MACHINE = 2, ITEM_WHEELS_DSC = 3, ITEM_WHEELS_NBT = 4, ITEM_PEDAL = 5;

volatile bool g_enabled = true, g_forget = false, g_pairRequest = false, g_printRequest = false, g_trafficLog = false;
volatile bool g_connected = false;
volatile uint8_t g_state = Obd::Off;
volatile uint32_t g_rxDropped = 0;
bool g_bleUp = false, g_unavailable = false;  // unavailable: the stack or our own resources could not start this boot
BLEClient *g_client = nullptr;
BLERemoteCharacteristic *g_notifyChar = nullptr, *g_writeChar = nullptr;
bool g_writeNoResponse = false;
StreamBufferHandle_t g_rx = nullptr;
Storage::ObdBinding g_binding;  // the paired dongle; addr empty when none
char g_adapterId[40] = "", g_chars[96] = "";
CarData g_car;
uint8_t g_curEcu = 0xFF;
bool g_fcModeSet = false;
// One receive filter for every ECU (ids 0x600 to 0x6FF) saves a command per change of ECU.
// g_wideFilter is what we want; g_adapterWide is what the adapter currently holds.
bool g_wideFilter = false, g_adapterWide = false;
uint8_t g_wideOk = 0;
uint32_t g_visitStartMs = 0;
uint8_t g_silentRun = 0, g_unansweredRun = 0;
uint32_t g_requests = 0, g_timeouts = 0, g_fastCount = 0, g_rateWindowMs = 0, g_scans = 0;
float g_fastHz = 0;
uint32_t g_lastAsleepProbeMs = 0, g_asleepSinceMs = 0, g_connectedAtMs = 0, g_lastSetupMs = 0, g_pairUntilMs = 0;
uint8_t g_probeIdx = 0, g_probesSinceSetup = 0;

bool bound() { return g_binding.addr[0] != 0; }

class ClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient *) override {}
  void onDisconnect(BLEClient *) override { g_connected = false; }
} g_callbacks;

void onNotify(BLERemoteCharacteristic *, uint8_t *data, size_t len, bool) {
  if (!g_rx || !len) return;
  if (xStreamBufferSend(g_rx, data, len, 0) != len) g_rxDropped = g_rxDropped + 1;  // counted, shown in the status
}

void setState(uint8_t s) {
  if (g_state != s) Serial.printf("OBD: %s\n", Obd::stateName(s));
  g_state = s;
}

// Car data and diagnostics into the shared state. The demo replay owns the car data while it
// runs; the diagnostics are ours either way.
void publish() {
  g_car.linked = g_state == Obd::Live || g_state == Obd::Asleep;
  g_car.asleep = g_state == Obd::Asleep;
  VsLock lk;
  if (!lk.ok()) return;
  VehicleState &vs = Vs::raw();
  if (!Demo::enabled()) vs.car = g_car;
  vs.diag.obdState = g_state;
  vs.diag.obdBound = bound();
  vs.diag.obdHz = g_fastHz;
  vs.diag.obdRequests = g_requests;
  vs.diag.obdTimeouts = g_timeouts;
}

void drain() {
  if (g_rx) xStreamBufferReset(g_rx);
}

bool sendText(const char *s) {
  if (!g_connected || !g_writeChar || !g_rx) return false;
  return g_writeChar->writeValue((uint8_t *)s, strlen(s), !g_writeNoResponse);
}

// Everything the adapter says up to its '>' prompt. Returns the length, or -1 when the prompt
// did not arrive in time (buf then holds what did).
int readReply(char *buf, size_t max, uint32_t timeoutMs) {
  size_t n = 0;
  uint32_t t0 = millis();
  buf[0] = 0;
  while (g_connected) {
    uint32_t el = millis() - t0;
    if (el >= timeoutMs) break;
    uint8_t tmp[64];
    size_t got = xStreamBufferReceive(g_rx, tmp, sizeof(tmp), pdMS_TO_TICKS(timeoutMs - el));
    for (size_t i = 0; i < got; i++) {
      if (tmp[i] == 0) continue;  // the ELM documents stray nulls
      if (n + 1 < max) buf[n++] = (char)tmp[i];
      if (tmp[i] == '>') {
        buf[n] = 0;
        return (int)n;
      }
    }
  }
  buf[n] = 0;
  return -1;
}

void logText(const char *tag, const char *text) {
  char clean[200];
  size_t k = 0;
  for (size_t i = 0; text[i] && k + 2 < sizeof(clean); i++) clean[k++] = (text[i] == '\r' || text[i] == '\n') ? '|' : text[i];
  clean[k] = 0;
  Serial.printf("OBD %s %s\n", tag, clean);
}

enum class AtResult : uint8_t { Ok, NotOk, NoPrompt, SendFailed };

// One adapter command. replyOut is always a terminated string on return, empty when nothing
// was received: a failed send used to leave it untouched and callers logged stack garbage
// (review finding 18).
AtResult atEx(const char *cmd, char *replyOut, size_t replyMax, uint32_t timeoutMs) {
  if (replyOut && replyMax) replyOut[0] = 0;
  char line[40], reply[160];
  reply[0] = 0;
  snprintf(line, sizeof(line), "%s\r", cmd);
  drain();
  if (!sendText(line)) return AtResult::SendFailed;
  int n = readReply(reply, sizeof(reply), timeoutMs);
  if (g_trafficLog || n < 0) {
    logText(">", cmd);
    logText(n < 0 ? "< (no prompt)" : "<", reply);
  }
  if (replyOut && replyMax) {
    strncpy(replyOut, reply, replyMax - 1);
    replyOut[replyMax - 1] = 0;
  }
  if (n < 0) return AtResult::NoPrompt;
  return ObdProto::replyOk(reply) ? AtResult::Ok : AtResult::NotOk;
}
bool at(const char *cmd, uint32_t timeoutMs = SELECT_TIMEOUT_MS) { return atEx(cmd, nullptr, 0, timeoutMs) == AtResult::Ok; }

// The adapter's own identity lines are informational and carry no OK: keep the first printable
// line whenever a prompt came back.
void atInfo(const char *cmd, char *out, size_t max) {
  char reply[160];
  out[0] = 0;
  AtResult r = atEx(cmd, reply, sizeof(reply), AT_TIMEOUT_MS);
  if (r == AtResult::SendFailed || r == AtResult::NoPrompt) return;
  size_t k = 0;
  for (size_t i = 0; reply[i] && k + 1 < max; i++) {
    char ch = reply[i];
    if (ch == '>') break;
    if (ch == '\r' || ch == '\n') {
      if (k) break;
      continue;
    }
    out[k++] = ch;
  }
  out[k] = 0;
}

bool elmSetup() {
  char reply[160];
  // ATZ takes a second and prints the banner; one retry after a bare return clears a half-typed line.
  AtResult reset = atEx("ATZ", reply, sizeof(reply), RESET_TIMEOUT_MS);
  if (reset != AtResult::Ok) {
    sendText("\r");
    delay(200);
    reset = atEx("ATZ", reply, sizeof(reply), RESET_TIMEOUT_MS);
  }
  if (reset != AtResult::Ok) {
    logText("setup: no ELM banner:", reply);
    return false;
  }
  // Echo off, linefeeds off, spaces off, headers on, ISO 15765-4 CAN 11-bit 500 kbit, adaptive
  // timing, 100 ms adapter timeout (under our 300 ms prompt wait), tester id on transmit and on
  // the flow control frames. The ECU-specific parts follow in selectEcu().
  static const char *const SEQ[] = {"ATE0", "ATL0", "ATS0", "ATH1", "ATSP6", "ATAT1", "ATST19", "ATSH6F1", "ATFCSH6F1"};
  for (const char *cmd : SEQ) {
    if (atEx(cmd, reply, sizeof(reply), AT_TIMEOUT_MS) != AtResult::Ok) {
      Serial.printf("OBD setup: %s not accepted\n", cmd);
      logText("<", reply);
      return false;
    }
  }
  atInfo("ATI", g_adapterId, sizeof(g_adapterId));
  char volts[24];
  atInfo("ATRV", volts, sizeof(volts));
  Serial.printf("OBD adapter: %s, port voltage %s\n", g_adapterId, volts);
  // Everything the adapter held is gone after ATZ: our record of it goes too.
  g_curEcu = 0xFF;
  g_fcModeSet = false;
  g_adapterWide = false;
  g_wideOk = 0;
  g_wideFilter = at("ATCF600", AT_TIMEOUT_MS) && at("ATCM700", AT_TIMEOUT_MS);
  g_adapterWide = g_wideFilter;
  Serial.printf("OBD setup: one receive filter for all ECUs %s\n", g_wideFilter ? "accepted, on trial" : "not accepted, per-ECU filter");
  g_lastSetupMs = millis();
  for (PollItem &it : g_items) it.frames = 0, it.countFails = 0;
  return true;
}

// Point the adapter at one ECU: extended address on transmit, its reply id as the receive
// filter (or the shared filter), and its address in the flow control frame (BMW ECUs ignore a
// flow control without it). The cache is cleared BEFORE the first command: if any of them
// fails the adapter is half-changed and must not be taken for the old ECU (finding 16).
bool selectEcu(uint8_t ecu) {
  if (ecu == g_curEcu && g_adapterWide == g_wideFilter) return true;
  g_curEcu = 0xFF;
  char cmd[24];
  snprintf(cmd, sizeof(cmd), "ATCEA%02X", ecu);
  if (!at(cmd)) return false;
  if (g_wideFilter) {
    if (!g_adapterWide) {
      if (!at("ATCF600") || !at("ATCM700")) return false;
      g_adapterWide = true;
    }
  } else {
    snprintf(cmd, sizeof(cmd), "ATCRA%03X", OBD_RX_BASE + ecu);
    if (!at(cmd)) return false;
    g_adapterWide = false;
  }
  snprintf(cmd, sizeof(cmd), "ATFCSD%02X300000", ecu);
  if (!at(cmd)) return false;
  if (!g_fcModeSet) {
    if (!at("ATFCSM1")) return false;
    g_fcModeSet = true;
  }
  g_curEcu = ecu;
  g_visitStartMs = millis();
  return true;
}

enum class Outcome : uint8_t { Good, Unanswered, Silent, Refused, Bad };

Outcome request(PollItem &it) {
  uint32_t t0 = millis();
  it.lastMs = t0;
  if (!selectEcu(it.ecu)) return Outcome::Silent;
  it.lastMs = millis();  // after the visit began, so the first identifier of a visit is not asked twice in it
  char req[12], reply[400];
  bool counted = it.frames >= 1 && it.frames <= 15 && it.countFails < 2;
  ObdProto::buildRequest(it.did, req, sizeof(req), counted ? it.frames : 0);
  drain();
  g_requests++;
  if (!sendText(req)) return Outcome::Silent;
  int n = readReply(reply, sizeof(reply), REQUEST_TIMEOUT_MS + 100);
  uint32_t rtt = millis() - t0;
  if (g_trafficLog) {
    Serial.printf("OBD > %02X %s", it.ecu, req);
    Serial.println();
    logText(n < 0 ? "< (no prompt)" : "<", reply);
  }
  if (n < 0) {
    g_timeouts++;
    it.fail++;
    it.lastReply = ObdProto::Reply::NoData;
    if (counted) {  // the digit may be why the adapter never finished: ask plainly next time
      it.frames = 0;
      it.countFails++;
    }
    sendText("\r");  // stops a search in progress so the next command starts clean
    char dump[64];
    readReply(dump, sizeof(dump), 150);
    return Outcome::Silent;
  }
  uint8_t payload[ObdProto::MAX_PAYLOAD];
  size_t len = 0;
  uint8_t nrc = 0;
  int frames = 0;
  ObdProto::Reply r = ObdProto::parseReply(reply, (size_t)n, it.ecu, it.did, payload, sizeof(payload), &len, &nrc, &frames);
  it.lastReply = r;
  it.lastNrc = nrc;
  if (r != ObdProto::Reply::Ok && counted) {
    it.frames = 0;
    it.countFails++;
  }
  if (r == ObdProto::Reply::Ok) {
    uint32_t now = millis();
    uint32_t fastBefore = g_car.fastStampMs;
    if (ObdDecode::apply(it.ecu, it.did, payload, len, now, g_car)) {
      it.ok++;
      it.rttSumMs += rtt;
      // The frame count is learnt only from a reply the decoder accepted.
      if (it.frames == 0 && frames > 0 && frames <= 15) it.frames = (uint8_t)frames;
      if (g_car.fastStampMs != fastBefore) g_fastCount++;
      if (!it.seen) {  // the bring-up record for docs/obd-notes.md
        it.seen = true;
        char hex[2 * 48 + 1], what[120];
        size_t k = 0;
        for (size_t i = 0; i < len && i < 48; i++) k += snprintf(hex + k, sizeof(hex) - k, "%02X", payload[i]);
        ObdDecode::describe(it.ecu, it.did, g_car, what, sizeof(what));
        Serial.printf("OBD first: ECU %02X DID %04X, %u bytes %s%s = %s (%lu ms)\n", it.ecu, it.did, (unsigned)len, hex, len > 48 ? "..." : "", what,
                      (unsigned long)rtt);
      }
      return Outcome::Good;
    }
    it.fail++;
    if (it.fail <= 3) {
      Serial.printf("OBD: ECU %02X DID %04X answered but the value is out of range or short (%u bytes)\n", it.ecu, it.did, (unsigned)len);
      logText("<", reply);
    }
    return Outcome::Bad;
  }
  it.fail++;
  if (r == ObdProto::Reply::NoData) {
    g_timeouts++;
    return Outcome::Unanswered;
  }
  if (r == ObdProto::Reply::Negative) {
    if (it.fail <= 2) Serial.printf("OBD: ECU %02X refuses DID %04X, NRC %02X\n", it.ecu, it.did, nrc);
    return Outcome::Refused;
  }
  if (it.fail <= 3) {
    Serial.printf("OBD: ECU %02X DID %04X: %s\n", it.ecu, it.did, ObdProto::replyName(r));
    logText("<", reply);
  }
  return Outcome::Bad;
}

bool due(const PollItem &it, uint32_t now) {
  if (!it.enabled) return false;
  if (it.periodMs == 0) return !it.done;
  return it.lastMs == 0 || now - it.lastMs >= it.periodMs;
}
float overdue(const PollItem &it, uint32_t now) {
  if (it.lastMs == 0) return 1000.0f;
  return (float)(now - it.lastMs) / (float)(it.periodMs ? it.periodMs : SLOW_MS);
}

// Next identifier to ask for. Changing ECU costs three adapter commands, so a visit asks for
// everything due on the current ECU (each identifier once per visit), then moves to the ECU
// holding the most overdue one.
int pickNext(uint32_t now) {
  int best = -1;
  float bestScore = 0;
  for (int i = 0; i < ITEM_COUNT; i++) {
    const PollItem &it = g_items[i];
    if (!due(it, now) || it.ecu != g_curEcu) continue;
    if (it.lastMs != 0 && it.lastMs >= g_visitStartMs) continue;  // already asked this visit
    float s = overdue(it, now);
    if (best < 0 || s > bestScore) best = i, bestScore = s;
  }
  if (best >= 0) return best;
  for (int i = 0; i < ITEM_COUNT; i++) {
    const PollItem &it = g_items[i];
    if (!due(it, now) || it.ecu == g_curEcu) continue;
    float s = overdue(it, now);
    if (best < 0 || s > bestScore) best = i, bestScore = s;
  }
  if (best >= 0) return best;
  // Nothing due elsewhere: a new visit on the same ECU if something there is due again.
  for (int i = 0; i < ITEM_COUNT; i++) {
    if (due(g_items[i], now)) {
      g_visitStartMs = now;
      return i;
    }
  }
  return -1;
}

// Back to the table's defaults: a new connection, or a new dongle, starts with no memory of the
// last one's failures.
void resetItems() {
  for (PollItem &it : g_items) {
    it.periodMs = it.defaultPeriodMs;
    it.enabled = it.defaultEnabled;
    it.lastMs = 0;
    it.failRun = 0;
    it.done = false;
    it.demoted = false;
    it.frames = 0;
    it.countFails = 0;
  }
  g_silentRun = g_unansweredRun = 0;
  g_curEcu = 0xFF;
}

void account(PollItem &it, Outcome o) {
  if (o == Outcome::Good) {
    it.failRun = 0;
    g_silentRun = g_unansweredRun = 0;
    if (it.defaultPeriodMs == 0) it.done = true;
    if (it.demoted) {  // it answers again: its own rate back
      it.demoted = false;
      it.periodMs = it.defaultPeriodMs;
    }
    return;
  }
  if (it.failRun < 255) it.failRun++;
  if (o == Outcome::Silent) {
    if (g_silentRun < 255) g_silentRun++;
  } else {
    g_silentRun = 0;
  }
  if (o == Outcome::Unanswered || o == Outcome::Silent) {
    if (g_unansweredRun < 255) g_unansweredRun++;
  } else {
    g_unansweredRun = 0;  // the car is talking, this identifier is the problem
  }
  if (g_state != Obd::Live) return;
  int idx = (int)(&it - g_items);
  // The DSC wheel speeds are the less proven source: four misses while the car answers others
  // (or one refusal) hand over to the head unit's, for this connection.
  if (idx == ITEM_WHEELS_DSC && (o == Outcome::Refused || (it.failRun >= 4 && g_unansweredRun < ASLEEP_AFTER))) {
    Serial.println("OBD: DSC wheel speeds (DBE4) not available on this car, using the head unit's (D030)");
    it.enabled = false;
    g_items[ITEM_WHEELS_NBT].enabled = true;
    return;
  }
  // Any other identifier the car keeps declining is asked for rarely, so it does not cost the
  // fast group its rate; a once-per-connection one is given up. A later answer restores it.
  if (g_unansweredRun < ASLEEP_AFTER && (o == Outcome::Refused || it.failRun >= 8)) {
    if (it.defaultPeriodMs == 0) {
      it.done = true;
    } else if (!it.demoted) {
      Serial.printf("OBD: ECU %02X DID %04X keeps failing (%s), asking every 10 s from now\n", it.ecu, it.did, ObdProto::replyName(it.lastReply));
      it.demoted = true;
      it.periodMs = DEMOTED_MS;
    }
  }
}

// While the car sleeps, one probe at a time, each to a different ECU in turn: the battery ECU
// alone must not decide whether the car is awake (review finding 15).
int probeItem() {
  static const int ROUND[4] = {ITEM_VOLTS, ITEM_MACHINE, -1, ITEM_PEDAL};
  int k = ROUND[g_probeIdx++ % 4];
  if (k < 0) k = g_items[ITEM_WHEELS_DSC].enabled ? ITEM_WHEELS_DSC : ITEM_WHEELS_NBT;
  return k;
}

// --- Bluetooth ---------------------------------------------------------------------------

bool nameLooksLikeDongle(const String &name) {
  String n = name;
  n.toLowerCase();
  return n.indexOf("vlink") >= 0;  // "vLinker BM+", "vLinker BM-IOS", "IOS-Vlink"
}

void disconnect() {
  if (g_client && g_client->isConnected()) g_client->disconnect();
  g_connected = false;
  g_notifyChar = g_writeChar = nullptr;
}

// Find a notify and a write characteristic for the serial stream. Known Vgate layouts first,
// then any vendor service that has both. Everything found is logged for docs/obd-notes.md.
bool findSerial() {
  std::map<std::string, BLERemoteService *> *services = g_client->getServices();
  if (!services) return false;
  static const uint16_t SKIP[] = {0x1800, 0x1801, 0x180A};
  static const uint16_t KNOWN[] = {0x18F0, 0xFFF0, 0xFFE0};
  BLERemoteCharacteristic *nBest = nullptr, *wBest = nullptr;
  int bestRank = 99;
  g_chars[0] = 0;
  for (auto &sv : *services) {
    BLERemoteService *svc = sv.second;
    BLEUUID su = svc->getUUID();
    std::map<std::string, BLERemoteCharacteristic *> *chars = svc->getCharacteristics();
    BLERemoteCharacteristic *n = nullptr, *w = nullptr;
    if (chars) {
      for (auto &ch : *chars) {
        BLERemoteCharacteristic *c = ch.second;
        Serial.printf("OBD gatt: service %s characteristic %s%s%s%s%s\n", su.toString().c_str(), c->getUUID().toString().c_str(),
                      c->canNotify() ? " notify" : "", c->canIndicate() ? " indicate" : "", c->canWrite() ? " write" : "",
                      c->canWriteNoResponse() ? " write-no-response" : "");
        if (!n && (c->canNotify() || c->canIndicate())) n = c;
        if (!w && (c->canWrite() || c->canWriteNoResponse())) w = c;
        // Prefer a characteristic dedicated to its direction when a service has several.
        if (c->canNotify() && !(c->canWrite() || c->canWriteNoResponse())) n = c;
        if ((c->canWrite() || c->canWriteNoResponse()) && !c->canNotify() && !c->canIndicate()) w = c;
      }
    }
    if (!n || !w) continue;
    bool skip = false;
    for (uint16_t s : SKIP) skip = skip || su.equals(BLEUUID(s));
    if (skip) continue;
    int rank = 10;
    for (int k = 0; k < 3; k++)
      if (su.equals(BLEUUID(KNOWN[k]))) rank = k;
    if (rank < bestRank) bestRank = rank, nBest = n, wBest = w;
  }
  if (!nBest || !wBest) return false;
  g_notifyChar = nBest;
  g_writeChar = wBest;
  g_writeNoResponse = wBest->canWriteNoResponse();
  snprintf(g_chars, sizeof(g_chars), "notify %s, write %s%s", nBest->getUUID().toString().c_str(), wBest->getUUID().toString().c_str(),
           g_writeNoResponse ? " (no response)" : "");
  Serial.printf("OBD gatt: using %s\n", g_chars);
  g_notifyChar->registerForNotify(onNotify, nBest->canNotify());
  return true;
}

// Connect, find the serial pair and set the adapter up. Nothing is stored here: a binding is
// only written by the pairing flow, after the car has answered through this dongle.
bool connectTo(const String &addr, uint8_t type, const char *name) {
  setState(Obd::Connecting);
  if (!g_client) {
    g_client = BLEDevice::createClient();
    if (!g_client) {
      Serial.println("OBD: no memory for a BLE client");
      return false;
    }
    g_client->setClientCallbacks(&g_callbacks);
  }
  Serial.printf("OBD: connecting to %s (%s)\n", addr.c_str(), name);
  if (!g_client->connect(BLEAddress(addr, type), type, CONNECT_TIMEOUT_MS)) {
    Serial.println("OBD: connect failed");
    return false;
  }
  g_connected = true;
  // 7.5 to 15 ms connection interval (the default is 20 to 40): every command is a write and a
  // notification, so the interval is paid twice per exchange.
  g_client->updateConnParams(6, 12, 0, 200);
  drain();
  if (!findSerial()) {
    Serial.println("OBD: no serial characteristics on this device");
    disconnect();
    return false;
  }
  setState(Obd::Setup);
  publish();
  if (!elmSetup()) {
    disconnect();
    return false;
  }
  resetItems();
  g_connectedAtMs = millis();
  g_rateWindowMs = g_connectedAtMs;
  g_fastCount = 0;
  return true;
}

// One scan. With a binding, only that address is taken. Without one this is only ever called
// from a pairing request, and the strongest device that looks like the dongle is offered.
bool scanFor(String *addr, uint8_t *type, String *name) {
  BLEScan *scan = BLEDevice::getScan();
  if (!scan) return false;
  // A 30 percent listening duty: the radio shares core 0 with the sensor task, and a denser
  // scan cost the IMU up to 7 percent of its samples on the bench (24 Nov 2026).
  scan->setActiveScan(true);
  scan->setInterval(160);
  scan->setWindow(48);
  BLEScanResults *res = scan->start(SCAN_SECONDS, false);
  g_scans++;
  int bestRssi = -999;
  bool found = false;
  int count = res ? res->getCount() : 0;
  for (int i = 0; i < count; i++) {
    BLEAdvertisedDevice d = res->getDevice(i);
    String n = d.haveName() ? d.getName() : String("");
    String a = d.getAddress().toString();
    bool take = bound() ? a.equalsIgnoreCase(g_binding.addr) : nameLooksLikeDongle(n);
    if (!bound() && n.length()) Serial.printf("OBD pairing: saw '%s' %s rssi %d%s\n", n.c_str(), a.c_str(), d.getRSSI(), take ? " (candidate)" : "");
    if (!take || d.getRSSI() <= bestRssi) continue;
    bestRssi = d.getRSSI();
    *addr = a;
    *name = n.length() ? n : String("dongle");
    *type = d.getAddressType();
    found = true;
  }
  scan->clearResults();
  return found;
}

void goLive() {
  setState(Obd::Live);
  publish();
}

// Pairing, on the owner's request only (decision D3, 24 Nov 2026): find a dongle, connect, set
// it up, and store it only after the car has answered a read through it. Returns true when
// connected and bound.
bool pairOnce() {
  String addr, name;
  uint8_t type = 0;
  if (!scanFor(&addr, &type, &name)) return false;
  if (!connectTo(addr, type, name.c_str())) return false;
  bool answered = false;
  for (int k = 0; k < 4 && !answered && g_connected; k++) answered = request(g_items[probeItem()]) == Outcome::Good;
  if (!answered) {
    Serial.printf("OBD pairing: '%s' connected but the car did not answer through it; not stored (is the car awake?)\n", name.c_str());
    disconnect();
    return false;
  }
  Storage::ObdBinding b;
  b.type = type;
  strncpy(b.addr, addr.c_str(), sizeof(b.addr) - 1);
  strncpy(b.name, name.c_str(), sizeof(b.name) - 1);
  if (!Storage::saveObdBinding(b)) {
    Serial.println("OBD pairing: could not store the dongle; not paired");
    disconnect();
    return false;
  }
  g_binding = b;
  Serial.printf("OBD pairing: paired with '%s' %s\n", g_binding.name, g_binding.addr);
  return true;
}

void printStatusNow() {
  Serial.printf("OBD status: %s, dongle %s '%s', adapter '%s', %s\n", Obd::stateName(g_state), bound() ? g_binding.addr : "none", g_binding.name, g_adapterId,
                g_chars);
  Serial.printf("OBD status: power updates %.1f Hz, %lu requests, %lu unanswered (%.1f %%), %lu received bytes dropped, linked %lu s\n", g_fastHz,
                (unsigned long)g_requests, (unsigned long)g_timeouts, g_requests ? 100.0f * g_timeouts / g_requests : 0.0f, (unsigned long)g_rxDropped,
                g_connected ? (unsigned long)((millis() - g_connectedAtMs) / 1000) : 0UL);
  for (const PollItem &it : g_items) {
    Serial.printf("OBD item: ECU %02X DID %04X %s period %lu ms, %u frames, ok %lu fail %lu, mean round trip %lu ms, last %s%s\n", it.ecu, it.did,
                  it.enabled ? "on " : "off", (unsigned long)it.periodMs, it.frames, (unsigned long)it.ok, (unsigned long)it.fail,
                  it.ok ? (unsigned long)(it.rttSumMs / it.ok) : 0UL, ObdProto::replyName(it.lastReply), it.demoted ? " (demoted)" : "");
  }
}

void obdTask(void *) {
  uint32_t backoffMs = 1000;
  for (;;) {
    if (g_printRequest) {  // printed here, by the task that owns the table (no torn reads)
      g_printRequest = false;
      printStatusNow();
    }
    // Forgetting is a storage operation and happens whether or not the radio is in use
    // (review finding 34).
    if (g_forget) {
      g_forget = false;
      if (g_connected) disconnect();
      if (Storage::clearObdBinding()) {
        g_binding = Storage::ObdBinding();
        Serial.println("OBD: dongle forgotten");
      } else {
        Serial.println("OBD: could not clear the stored dongle; still paired");
      }
      g_car = CarData();
      publish();
    }
    if (!g_enabled || g_unavailable) {
      if (g_connected) disconnect();
      g_pairRequest = false;
      if (g_state != Obd::Off) {
        setState(Obd::Off);
        g_car = CarData();
      }
      publish();
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }
    if (!g_bleUp) {
      uint32_t before = ESP.getFreeHeap();
      if (!BLEDevice::init("Mini Dash")) {
        // Not enough internal RAM for the stack, most likely. Nothing else depends on OBD:
        // say so, stay off until the next boot, and leave the rest of the dash alone.
        Serial.printf("OBD: Bluetooth did not start (heap %lu), OBD off until the next boot\n", (unsigned long)before);
        g_unavailable = true;
        continue;
      }
      g_bleUp = true;
      Serial.printf("OBD: Bluetooth up, heap %lu to %lu\n", (unsigned long)before, (unsigned long)ESP.getFreeHeap());
    }
    if (!g_connected) {
      if (g_state == Obd::Live || g_state == Obd::Asleep) {
        Serial.println("OBD: link lost");
        g_car = CarData();
      }
      if (!bound()) {
        // No dongle is stored: the dash connects to nothing until the owner asks it to pair.
        if (!g_pairRequest) {
          setState(Obd::Unpaired);
          publish();
          vTaskDelay(pdMS_TO_TICKS(300));
          continue;
        }
        if (g_state != Obd::Pairing) {
          setState(Obd::Pairing);
          g_pairUntilMs = millis() + PAIR_WINDOW_MS;
          publish();
        }
        if (pairOnce()) {
          g_pairRequest = false;
          goLive();
        } else if ((int32_t)(millis() - g_pairUntilMs) >= 0) {
          g_pairRequest = false;
          Serial.println("OBD pairing: nothing paired in 30 s; tap Pair to try again");
          setState(Obd::Unpaired);
          publish();
        }
        continue;
      }
      g_pairRequest = false;
      // The stored dongle is tried directly first (no scan needed), then found by scanning.
      bool ok = false;
      if (g_scans % 2 == 0) ok = connectTo(String(g_binding.addr), g_binding.type, g_binding.name[0] ? g_binding.name : "stored dongle");
      if (!ok && g_enabled) {
        setState(Obd::Scanning);
        publish();
        String addr, name;
        uint8_t type = 0;
        if (scanFor(&addr, &type, &name)) ok = connectTo(addr, type, g_binding.name[0] ? g_binding.name : name.c_str());
        else g_scans++;  // so the direct attempt comes round again
      }
      if (!ok) {
        setState(Obd::Scanning);
        publish();
        vTaskDelay(pdMS_TO_TICKS(backoffMs));
        backoffMs = backoffMs * 2 > MAX_BACKOFF_MS ? MAX_BACKOFF_MS : backoffMs * 2;
        continue;
      }
      backoffMs = 1000;
      goLive();
    }

    uint32_t now = millis();
    if (now - g_rateWindowMs >= 2000) {
      g_fastHz = (float)g_fastCount * 1000.0f / (float)(now - g_rateWindowMs);
      g_fastCount = 0;
      g_rateWindowMs = now;
      publish();
    }
    int idx;
    if (g_state == Obd::Asleep) {
      uint32_t every = now - g_asleepSinceMs < 60000 ? ASLEEP_PROBE_MS : ASLEEP_PROBE_SLOW_MS;
      if (now - g_lastAsleepProbeMs < every) {
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
      }
      // The adapter may have lost its configuration while the car slept (it has its own sleep
      // mode): after two rounds of probes with no answer, set it up again, at most every 30 s.
      if (g_probesSinceSetup >= PROBES_BEFORE_SETUP && now - g_lastSetupMs >= SETUP_AGAIN_MIN_MS) {
        Serial.println("OBD: still no answer, setting the adapter up again");
        g_probesSinceSetup = 0;
        if (!elmSetup()) {
          Serial.println("OBD: adapter set-up failed, reconnecting");
          disconnect();
          g_car = CarData();
          setState(Obd::Scanning);
          publish();
          continue;
        }
      }
      g_lastAsleepProbeMs = millis();
      g_probesSinceSetup++;
      idx = probeItem();
    } else {
      idx = pickNext(now);
      if (idx < 0) {
        vTaskDelay(pdMS_TO_TICKS(5));
        continue;
      }
    }
    PollItem &it = g_items[idx];
    Outcome o = request(it);
    // While the shared receive filter is on trial, a failure is put to the question directly:
    // the same identifier again with this ECU's own filter. Only if that succeeds is the
    // shared filter the cause, and it is then dropped for this connection (finding 16).
    if (g_state == Obd::Live && g_wideFilter && g_wideOk < 20 && (o == Outcome::Unanswered || o == Outcome::Bad)) {
      g_wideFilter = false;
      Outcome again = request(it);
      if (again == Outcome::Good) {
        Serial.println("OBD: the shared receive filter loses replies on this adapter, using one filter per ECU");
        o = again;
      } else {
        g_wideFilter = true;  // not the filter: selectEcu() puts it back on the next request
      }
    }
    if (o == Outcome::Good && g_wideFilter && g_wideOk < 20) g_wideOk++;
    account(it, o);
    if (g_silentRun >= RECONNECT_AFTER) {
      Serial.println("OBD: the dongle stopped answering, reconnecting");
      disconnect();
      g_car = CarData();
      setState(Obd::Scanning);
      publish();
      continue;
    }
    if (g_state == Obd::Asleep && (o == Outcome::Good || o == Outcome::Refused)) {
      // Any ECU answering, even with a refusal, means the car is awake.
      for (PollItem &p : g_items) p.lastMs = 0;
      setState(Obd::Live);
    } else if (g_state == Obd::Live && g_unansweredRun >= ASLEEP_AFTER) {
      setState(Obd::Asleep);
      g_asleepSinceMs = g_lastAsleepProbeMs = millis();
      g_probesSinceSetup = 0;
      g_curEcu = 0xFF;  // reselect on wake
    }
    if (o == Outcome::Good || g_state == Obd::Asleep) publish();
  }
}
}  // namespace

namespace Obd {

const char *stateName(uint8_t s) {
  switch (s) {
    case Off: return "off";
    case Scanning: return "scanning";
    case Connecting: return "connecting";
    case Setup: return "setting up the adapter";
    case Live: return "live";
    case Asleep: return "car asleep";
    case Unpaired: return "no dongle paired";
    case Pairing: return "pairing";
  }
  return "?";
}

void start() {
  g_enabled = Storage::loadObdEnabled();
  Storage::loadObdBinding(g_binding);
  Serial.printf("OBD: %s, dongle %s\n", g_enabled ? "on" : "off", bound() ? g_binding.addr : "none paired (Settings, Pair dongle)");
  // Without the receive buffer or the task there is no OBD this boot; the rest of the dash is
  // unaffected and the state says off (review finding 19).
  g_rx = xStreamBufferCreate(1024, 1);
  if (!g_rx) {
    Serial.println("OBD: no memory for the receive buffer, OBD off until the next boot");
    g_unavailable = true;
    return;
  }
  if (xTaskCreatePinnedToCore(obdTask, "obd", 8 * 1024, nullptr, 2, nullptr, 0) != pdPASS) {
    Serial.println("OBD: the task could not start, OBD off until the next boot");
    g_unavailable = true;
  }
}

void setEnabled(bool on) {
  if (g_unavailable) return;
  if (on == g_enabled) return;
  g_enabled = on;
  Storage::saveObdEnabled(on);
}
bool enabled() { return g_enabled && !g_unavailable; }
bool paired() { return bound(); }
void forgetDongle() { g_forget = true; }
void requestPairing() {
  if (!bound()) g_pairRequest = true;
}
void setTrafficLog(bool on) { g_trafficLog = on; }
bool trafficLog() { return g_trafficLog; }
void printStatus() {
  if (g_unavailable) Serial.println("OBD status: unavailable this boot");
  else g_printRequest = true;
}

}  // namespace Obd
