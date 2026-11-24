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
constexpr uint32_t REQUEST_TIMEOUT_MS = 300;  // CLAUDE.md section 3: every request bounded
constexpr uint32_t AT_TIMEOUT_MS = 1500;
constexpr uint32_t RESET_TIMEOUT_MS = 4000;
constexpr uint32_t SCAN_SECONDS = 4;
constexpr uint32_t CONNECT_TIMEOUT_MS = 6000;
constexpr uint8_t ASLEEP_AFTER = 3;           // consecutive unanswered requests: the car is asleep
constexpr uint8_t RECONNECT_AFTER = 3;        // consecutive requests the dongle itself ignored
constexpr uint32_t ASLEEP_PROBE_MS = 1500;
constexpr uint32_t FAST_MS = 200, SLOW_MS = 2000, RANGE_MS = 5000;

struct PollItem {
  uint8_t ecu;
  uint16_t did;
  uint32_t periodMs;  // 0: once per connection
  bool enabled;
  uint32_t lastMs = 0, ok = 0, fail = 0, rttSumMs = 0;
  uint8_t failRun = 0;
  bool seen = false, done = false, demoted = false;
  ObdProto::Reply lastReply = ObdProto::Reply::NoData;
  uint8_t lastNrc = 0;
  PollItem(uint8_t e, uint16_t d, uint32_t p, bool en = true) : ecu(e), did(d), periodMs(p), enabled(en) {}
};

// Fast group first (CLAUDE.md: voltage, current, torque, pedal, wheel speeds), then the slow
// group, then the once-per-connection values. Both wheel-speed sources are listed; the DSC
// one (0.01 km/h, proven on other F56s only) is tried first and the head unit's (1 km/h,
// proven on the SE by OVMS) takes over if the car refuses it.
PollItem g_items[] = {
    {ECU_SME, DID_SME_VOLTS, FAST_MS},
    {ECU_SME, DID_SME_AMPS, FAST_MS},
    {ECU_EME, DID_EME_MACHINE, FAST_MS},
    {ECU_DSC, DID_DSC_WHEELS, FAST_MS},
    {ECU_NBT, DID_NBT_WHEELS, FAST_MS, false},
    {ECU_EDM, DID_EDM_PEDAL, FAST_MS},
    {ECU_SME, DID_SME_SOC, SLOW_MS},
    {ECU_SME, DID_SME_TEMPS, SLOW_MS},
    {ECU_SME, DID_SME_CELLS, SLOW_MS},
    {ECU_EME, DID_EME_MOTOR_TEMPS, SLOW_MS},
    {ECU_EDM, DID_EDM_IBS, SLOW_MS},
    {ECU_KOM, DID_KOM_RANGE, RANGE_MS},
    {ECU_SME, DID_SME_SOH, 0},
};
constexpr int ITEM_COUNT = sizeof(g_items) / sizeof(g_items[0]);
constexpr int ITEM_PROBE = 0;      // pack volts: the probe while the car sleeps
constexpr int ITEM_WHEELS_DSC = 3, ITEM_WHEELS_NBT = 4;

volatile bool g_enabled = true, g_forget = false, g_trafficLog = false;
volatile bool g_connected = false;
volatile uint8_t g_state = Obd::Off;
bool g_bleUp = false, g_bleFailed = false;
BLEClient *g_client = nullptr;
BLERemoteCharacteristic *g_notifyChar = nullptr, *g_writeChar = nullptr;
bool g_writeNoResponse = false;
StreamBufferHandle_t g_rx = nullptr;
char g_dongleAddr[20] = "";
uint8_t g_dongleType = 0;
char g_dongleName[24] = "", g_adapterId[40] = "", g_chars[96] = "";
CarData g_car;
uint8_t g_curEcu = 0xFF;
bool g_fcModeSet = false;
uint32_t g_visitStartMs = 0;
uint8_t g_silentRun = 0, g_unansweredRun = 0;
uint32_t g_requests = 0, g_timeouts = 0, g_fastCount = 0, g_rateWindowMs = 0, g_scans = 0;
float g_fastHz = 0;
uint32_t g_lastAsleepProbeMs = 0, g_connectedAtMs = 0;

class ClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient *) override {}
  void onDisconnect(BLEClient *) override { g_connected = false; }
} g_callbacks;

void onNotify(BLERemoteCharacteristic *, uint8_t *data, size_t len, bool) {
  if (g_rx && len) xStreamBufferSend(g_rx, data, len, 0);
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
  vs.diag.obdHz = g_fastHz;
  vs.diag.obdRequests = g_requests;
  vs.diag.obdTimeouts = g_timeouts;
}

void drain() {
  if (g_rx) xStreamBufferReset(g_rx);
}

bool sendText(const char *s) {
  if (!g_connected || !g_writeChar) return false;
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

bool at(const char *cmd, char *replyOut = nullptr, size_t replyMax = 0, uint32_t timeoutMs = AT_TIMEOUT_MS) {
  char line[40], reply[160];
  snprintf(line, sizeof(line), "%s\r", cmd);
  drain();
  if (!sendText(line)) return false;
  int n = readReply(reply, sizeof(reply), timeoutMs);
  if (g_trafficLog || n < 0) {
    logText(">", cmd);
    logText(n < 0 ? "< (no prompt)" : "<", reply);
  }
  if (replyOut && replyMax) {
    strncpy(replyOut, reply, replyMax - 1);
    replyOut[replyMax - 1] = 0;
  }
  return n >= 0 && ObdProto::replyOk(reply);
}

// The adapter's own identity lines are informational: keep the first printable line.
void atInfo(const char *cmd, char *out, size_t max) {
  char reply[160];
  out[0] = 0;
  at(cmd, reply, sizeof(reply));
  size_t k = 0;
  for (size_t i = 0; reply[i] && k + 1 < max; i++) {
    char ch = reply[i];
    if (ch == '>' ) break;
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
  bool reset = at("ATZ", reply, sizeof(reply), RESET_TIMEOUT_MS);
  if (!reset) {
    sendText("\r");
    delay(200);
    reset = at("ATZ", reply, sizeof(reply), RESET_TIMEOUT_MS);
  }
  if (!reset) {
    logText("setup: no ELM banner:", reply);
    return false;
  }
  // Echo off, linefeeds off, spaces off, headers on, ISO 15765-4 CAN 11-bit 500 kbit, adaptive
  // timing, 200 ms adapter timeout (under our 300 ms bound), tester id on transmit and on the
  // flow control frames. The ECU-specific parts follow in selectEcu().
  static const char *const SEQ[] = {"ATE0", "ATL0", "ATS0", "ATH1", "ATSP6", "ATAT1", "ATST32", "ATSH6F1", "ATFCSH6F1"};
  for (const char *cmd : SEQ) {
    if (!at(cmd, reply, sizeof(reply))) {
      Serial.printf("OBD setup: %s not accepted\n", cmd);
      logText("<", reply);
      return false;
    }
  }
  atInfo("ATI", g_adapterId, sizeof(g_adapterId));
  char volts[24];
  atInfo("ATRV", volts, sizeof(volts));
  Serial.printf("OBD adapter: %s, port voltage %s\n", g_adapterId, volts);
  g_curEcu = 0xFF;
  g_fcModeSet = false;
  return true;
}

// Point the adapter at one ECU: extended address on transmit, its reply id as the receive
// filter, and its address in the flow control frame (BMW ECUs ignore a flow control without it).
bool selectEcu(uint8_t ecu) {
  if (ecu == g_curEcu) return true;
  char cmd[24];
  snprintf(cmd, sizeof(cmd), "ATCEA%02X", ecu);
  if (!at(cmd)) return false;
  snprintf(cmd, sizeof(cmd), "ATCRA%03X", OBD_RX_BASE + ecu);
  if (!at(cmd)) return false;
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
  if (!selectEcu(it.ecu)) return g_connected ? Outcome::Silent : Outcome::Silent;
  char req[12], reply[400];
  ObdProto::buildRequest(it.did, req, sizeof(req));
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
    sendText("\r");  // stops a search in progress so the next command starts clean
    char dump[64];
    readReply(dump, sizeof(dump), 150);
    return Outcome::Silent;
  }
  uint8_t payload[ObdProto::MAX_PAYLOAD];
  size_t len = 0;
  uint8_t nrc = 0;
  ObdProto::Reply r = ObdProto::parseReply(reply, (size_t)n, it.ecu, it.did, payload, sizeof(payload), &len, &nrc);
  it.lastReply = r;
  it.lastNrc = nrc;
  if (r == ObdProto::Reply::Ok) {
    uint32_t now = millis();
    uint32_t fastBefore = g_car.fastStampMs;
    if (ObdDecode::apply(it.ecu, it.did, payload, len, now, g_car)) {
      it.ok++;
      it.rttSumMs += rtt;
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

void resetItems() {
  for (PollItem &it : g_items) {
    it.lastMs = 0;
    it.failRun = 0;
    it.done = false;
    if (it.demoted) {
      it.periodMs = it.periodMs == 10000 ? SLOW_MS : it.periodMs;
      it.demoted = false;
    }
  }
  g_silentRun = g_unansweredRun = 0;
  g_curEcu = 0xFF;
}

void account(PollItem &it, Outcome o) {
  if (o == Outcome::Good) {
    it.failRun = 0;
    g_silentRun = g_unansweredRun = 0;
    if (it.periodMs == 0) it.done = true;
    if (g_state == Obd::Asleep) setState(Obd::Live);
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
  int idx = (int)(&it - g_items);
  // The DSC wheel speeds are the unproven source: four misses while the car answers others
  // (or one refusal) hand over to the head unit's.
  if (idx == ITEM_WHEELS_DSC && g_state == Obd::Live && (o == Outcome::Refused || (it.failRun >= 4 && g_unansweredRun < ASLEEP_AFTER))) {
    Serial.println("OBD: DSC wheel speeds (DBE4) not available on this car, using the head unit's (D030)");
    it.enabled = false;
    g_items[ITEM_WHEELS_NBT].enabled = true;
    return;
  }
  // Any other identifier the car keeps declining is asked for rarely, so it does not cost the
  // fast group its rate; a once-per-connection one is given up.
  if (g_state == Obd::Live && g_unansweredRun < ASLEEP_AFTER && (o == Outcome::Refused || it.failRun >= 8)) {
    if (it.periodMs == 0) {
      it.done = true;
    } else if (!it.demoted && idx != ITEM_PROBE) {
      Serial.printf("OBD: ECU %02X DID %04X keeps failing (%s), asking every 10 s from now\n", it.ecu, it.did, ObdProto::replyName(it.lastReply));
      it.demoted = true;
      it.periodMs = 10000;
    }
  }
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

bool connectTo(const String &addr, uint8_t type, const char *name) {
  setState(Obd::Connecting);
  if (!g_client) {
    g_client = BLEDevice::createClient();
    g_client->setClientCallbacks(&g_callbacks);
  }
  Serial.printf("OBD: connecting to %s (%s)\n", addr.c_str(), name);
  if (!g_client->connect(BLEAddress(addr, type), type, CONNECT_TIMEOUT_MS)) {
    Serial.println("OBD: connect failed");
    return false;
  }
  g_connected = true;
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
  strncpy(g_dongleName, name, sizeof(g_dongleName) - 1);
  if (strcmp(g_dongleAddr, addr.c_str()) != 0 || g_dongleType != type) {
    strncpy(g_dongleAddr, addr.c_str(), sizeof(g_dongleAddr) - 1);
    g_dongleType = type;
    Storage::saveObdDongle(g_dongleAddr, g_dongleType);
    Serial.printf("OBD: dongle %s stored\n", g_dongleAddr);
  }
  resetItems();
  g_connectedAtMs = millis();
  g_rateWindowMs = g_connectedAtMs;
  g_fastCount = 0;
  setState(Obd::Live);
  publish();
  return true;
}

bool scanAndConnect() {
  setState(Obd::Scanning);
  publish();
  BLEScan *scan = BLEDevice::getScan();
  if (!scan) return false;
  // A 30 percent listening duty: the radio shares core 0 with the sensor task, and a denser
  // scan cost the IMU up to 7 percent of its samples on the bench (24 Nov 2026).
  scan->setActiveScan(true);
  scan->setInterval(160);
  scan->setWindow(48);
  BLEScanResults *res = scan->start(SCAN_SECONDS, false);
  g_scans++;
  String bestAddr, bestName;
  uint8_t bestType = 0;
  int bestRssi = -999;
  int count = res ? res->getCount() : 0;
  for (int i = 0; i < count; i++) {
    BLEAdvertisedDevice d = res->getDevice(i);
    String name = d.haveName() ? d.getName() : String("");
    String addr = d.getAddress().toString();
    bool stored = g_dongleAddr[0] && addr.equalsIgnoreCase(g_dongleAddr);
    bool looks = nameLooksLikeDongle(name);
    if (g_scans <= 2 && name.length()) Serial.printf("OBD scan: saw '%s' %s rssi %d\n", name.c_str(), addr.c_str(), d.getRSSI());
    if (g_dongleAddr[0] ? !stored : !looks) continue;  // a stored dongle is the only one we take
    if (d.getRSSI() > bestRssi) {
      bestRssi = d.getRSSI();
      bestAddr = addr;
      bestName = name.length() ? name : String("stored dongle");
      bestType = d.getAddressType();
    }
  }
  scan->clearResults();
  if (!bestAddr.length()) return false;
  return connectTo(bestAddr, bestType, bestName.c_str());
}

void obdTask(void *) {
  uint32_t backoffMs = 1000;
  for (;;) {
    if (!g_enabled) {
      if (g_connected) disconnect();
      if (g_state != Obd::Off) {
        setState(Obd::Off);
        g_car = CarData();
        publish();
      }
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }
    if (!g_bleUp) {
      uint32_t before = ESP.getFreeHeap();
      if (!BLEDevice::init("Mini Dash")) {
        // Not enough internal RAM for the stack, most likely. Nothing else depends on OBD:
        // say so, stay off until the next boot, and leave the rest of the dash alone.
        Serial.printf("OBD: Bluetooth did not start (heap %lu), OBD off until the next boot\n", (unsigned long)before);
        g_bleFailed = true;
        g_enabled = false;
        continue;
      }
      g_bleUp = true;
      Serial.printf("OBD: Bluetooth up, heap %lu to %lu\n", (unsigned long)before, (unsigned long)ESP.getFreeHeap());
    }
    if (g_forget) {
      g_forget = false;
      disconnect();
      g_dongleAddr[0] = 0;
      Storage::clearObdDongle();
      Serial.println("OBD: dongle forgotten");
    }
    if (!g_connected) {
      if (g_state == Obd::Live || g_state == Obd::Asleep) {
        Serial.println("OBD: link lost");
        g_car = CarData();
      }
      // A stored dongle is tried directly first (no scan needed), then found by scanning.
      bool ok = false;
      if (g_dongleAddr[0] && g_scans % 2 == 0) ok = connectTo(String(g_dongleAddr), g_dongleType, g_dongleName[0] ? g_dongleName : "stored dongle");
      if (!ok && g_enabled) ok = scanAndConnect();
      if (!ok) {
        setState(Obd::Scanning);
        publish();
        vTaskDelay(pdMS_TO_TICKS(backoffMs));
        backoffMs = backoffMs < 12000 ? backoffMs * 2 : 12000;
        continue;
      }
      backoffMs = 1000;
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
      if (now - g_lastAsleepProbeMs < ASLEEP_PROBE_MS) {
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
      }
      g_lastAsleepProbeMs = now;
      idx = ITEM_PROBE;
    } else {
      idx = pickNext(now);
      if (idx < 0) {
        vTaskDelay(pdMS_TO_TICKS(5));
        continue;
      }
    }
    PollItem &it = g_items[idx];
    Outcome o = request(it);
    account(it, o);
    if (g_silentRun >= RECONNECT_AFTER) {
      Serial.println("OBD: the dongle stopped answering, reconnecting");
      disconnect();
      g_car = CarData();
      setState(Obd::Scanning);
      publish();
      continue;
    }
    if (g_state == Obd::Live && g_unansweredRun >= ASLEEP_AFTER) {
      setState(Obd::Asleep);
      g_curEcu = 0xFF;  // reselect on wake: the adapter may have reset its filters meanwhile
      g_lastAsleepProbeMs = millis();
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
  }
  return "?";
}

void start() {
  g_enabled = Storage::loadObdEnabled();
  Storage::loadObdDongle(g_dongleAddr, sizeof(g_dongleAddr), &g_dongleType);
  g_rx = xStreamBufferCreate(1024, 1);
  Serial.printf("OBD: %s, dongle %s\n", g_enabled ? "on" : "off", g_dongleAddr[0] ? g_dongleAddr : "not paired yet (any vLinker)");
  xTaskCreatePinnedToCore(obdTask, "obd", 8 * 1024, nullptr, 2, nullptr, 0);
}

void setEnabled(bool on) {
  if (g_bleFailed) return;  // the stack could not start this boot
  if (on == g_enabled) return;
  g_enabled = on;
  Storage::saveObdEnabled(on);
}
bool enabled() { return g_enabled; }
void forgetDongle() { g_forget = true; }
void setTrafficLog(bool on) { g_trafficLog = on; }
bool trafficLog() { return g_trafficLog; }

void printStatus() {
  Serial.printf("OBD status: %s, dongle %s '%s', adapter '%s', %s\n", stateName(g_state), g_dongleAddr[0] ? g_dongleAddr : "none", g_dongleName, g_adapterId,
                g_chars);
  Serial.printf("OBD status: power updates %.1f Hz, %lu requests, %lu unanswered (%.1f %%), linked %lu s\n", g_fastHz, (unsigned long)g_requests,
                (unsigned long)g_timeouts, g_requests ? 100.0f * g_timeouts / g_requests : 0.0f,
                g_connected ? (unsigned long)((millis() - g_connectedAtMs) / 1000) : 0UL);
  for (const PollItem &it : g_items) {
    Serial.printf("OBD item: ECU %02X DID %04X %s period %lu ms, ok %lu fail %lu, mean round trip %lu ms, last %s%s\n", it.ecu, it.did,
                  it.enabled ? "on " : "off", (unsigned long)it.periodMs, (unsigned long)it.ok, (unsigned long)it.fail,
                  it.ok ? (unsigned long)(it.rttSumMs / it.ok) : 0UL, ObdProto::replyName(it.lastReply), it.demoted ? " (demoted)" : "");
  }
}

}  // namespace Obd
