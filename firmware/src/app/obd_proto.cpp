#include "obd_proto.h"

#include <stdio.h>
#include <string.h>

#include "obd_pids.h"

namespace {
int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// One line without spaces into bytes after the 3-digit CAN id. Returns the byte count, or -1
// if the line is not a hex frame.
int parseFrame(const char *line, size_t len, uint16_t *canId, uint8_t *bytes, size_t max) {
  char buf[40];
  size_t n = 0;
  for (size_t i = 0; i < len && n < sizeof(buf); i++) {
    if (line[i] == ' ' || line[i] == '\t') continue;
    if (hexVal(line[i]) < 0) return -1;
    buf[n++] = line[i];
  }
  if (n < 3 + 4 || ((n - 3) & 1)) return -1;  // id, then at least the address and one PCI byte, whole bytes
  *canId = (uint16_t)((hexVal(buf[0]) << 8) | (hexVal(buf[1]) << 4) | hexVal(buf[2]));
  size_t count = (n - 3) / 2;
  if (count > max) count = max;
  for (size_t i = 0; i < count; i++) bytes[i] = (uint8_t)((hexVal(buf[3 + 2 * i]) << 4) | hexVal(buf[4 + 2 * i]));
  return (int)count;
}

bool contains(const char *text, size_t len, const char *needle) {
  size_t m = strlen(needle);
  if (m == 0 || len < m) return false;
  for (size_t i = 0; i + m <= len; i++)
    if (memcmp(text + i, needle, m) == 0) return true;
  return false;
}
}  // namespace

namespace ObdProto {

void buildRequest(uint16_t did, char *out, size_t n, int frames) {
  if (frames >= 1 && frames <= 15) snprintf(out, n, "%02X%04X%X\r", UDS_READ_DID, did, frames);
  else snprintf(out, n, "%02X%04X\r", UDS_READ_DID, did);
}

bool replyOk(const char *text) {
  size_t len = strlen(text);
  return contains(text, len, "OK") || contains(text, len, "ELM327") || contains(text, len, "vLinker");
}

const char *replyName(Reply r) {
  switch (r) {
    case Reply::Ok: return "ok";
    case Reply::NoData: return "no data";
    case Reply::Negative: return "negative";
    case Reply::Malformed: return "malformed";
    case Reply::Adapter: return "adapter error";
  }
  return "?";
}

Reply parseReply(const char *text, size_t textLen, uint8_t ecu, uint16_t did, uint8_t *payload, size_t payloadMax,
                 size_t *payloadLen, uint8_t *nrc, int *frames) {
  *payloadLen = 0;
  if (frames) *frames = 0;
  int msgFrames = 0;
  if (nrc) *nrc = 0;
  const uint16_t wantId = (uint16_t)(OBD_RX_BASE + ecu);
  uint8_t msg[MAX_PAYLOAD + 3];
  size_t msgLen = 0, msgTotal = 0;
  int nextSeq = -1;         // -1: no multi-frame message in progress
  bool sawFrame = false, sawPartial = false, adapterError = false, noData = false;
  Reply result = Reply::NoData;
  bool done = false;

  auto finish = [&](const uint8_t *m, size_t n) {
    // A complete UDS message: ours, a refusal of ours, or something else (a stale reply).
    if (n >= 3 && m[0] == UDS_READ_DID_OK && m[1] == (did >> 8) && m[2] == (did & 0xFF)) {
      size_t k = n - 3;
      if (k > payloadMax) k = payloadMax;
      memcpy(payload, m + 3, k);
      *payloadLen = k;
      result = Reply::Ok;
      done = true;
    } else if (n >= 3 && m[0] == UDS_NEGATIVE && m[1] == UDS_READ_DID) {
      if (m[2] == UDS_NRC_PENDING) return;  // the real answer follows
      if (nrc) *nrc = m[2];
      result = Reply::Negative;
      done = true;
    }
  };

  size_t pos = 0;
  while (pos < textLen && !done) {
    size_t end = pos;
    while (end < textLen && text[end] != '\r' && text[end] != '\n' && text[end] != '>') end++;
    const char *line = text + pos;
    size_t len = end - pos;
    pos = end + 1;
    while (len && (line[0] == ' ')) { line++; len--; }
    if (len == 0) continue;

    uint16_t id = 0;
    uint8_t b[12];
    int n = parseFrame(line, len, &id, b, sizeof(b));
    if (n < 0) {
      // Not a frame: an adapter message. SEARCHING... and echoes are harmless.
      if (contains(line, len, "NO DATA")) noData = true;
      else if (contains(line, len, "ERROR") || contains(line, len, "BUFFER FULL") || contains(line, len, "STOPPED") ||
               contains(line, len, "UNABLE") || contains(line, len, "BUS") || contains(line, len, "?"))
        adapterError = true;
      continue;
    }
    if (id != wantId || n < 2 || b[0] != OBD_TESTER_ADDR) continue;  // another ECU's traffic
    sawFrame = true;
    uint8_t pci = b[1];
    switch (pci >> 4) {
      case 0: {  // single frame
        size_t l = pci & 0x0F;
        if (l == 0 || (size_t)n < 2 + l) { sawPartial = true; break; }
        nextSeq = -1;
        msgFrames = 1;
        finish(b + 2, l);
        break;
      }
      case 1: {  // first frame
        if (n < 3) { sawPartial = true; break; }
        msgTotal = ((size_t)(pci & 0x0F) << 8) | b[2];
        msgLen = 0;
        if (msgTotal < 7 || msgTotal > sizeof(msg)) { nextSeq = -1; sawPartial = true; break; }
        for (int i = 3; i < n && msgLen < msgTotal; i++) msg[msgLen++] = b[i];
        nextSeq = 1;
        msgFrames = 1;
        break;
      }
      case 2: {  // consecutive frame
        if (nextSeq < 0) break;  // none in progress: the tail of something we never started
        if ((pci & 0x0F) != (nextSeq & 0x0F)) { nextSeq = -1; sawPartial = true; break; }
        for (int i = 2; i < n && msgLen < msgTotal; i++) msg[msgLen++] = b[i];
        nextSeq++;
        msgFrames++;
        if (msgLen >= msgTotal) {
          nextSeq = -1;
          finish(msg, msgTotal);
        }
        break;
      }
      default: break;  // flow control and anything else
    }
  }
  if (done) {
    if (frames && result == Reply::Ok) *frames = msgFrames;
    return result;
  }
  if (nextSeq >= 0 || sawPartial) return Reply::Malformed;
  if (adapterError) return Reply::Adapter;
  if (noData || !sawFrame) return Reply::NoData;
  return Reply::Malformed;  // frames from the right ECU, none of them our answer
}

}  // namespace ObdProto
