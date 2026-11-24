// The text side of talking to the car through an ELM327-style adapter (CLAUDE.md section 3):
// building a ReadDataByIdentifier request, and turning the adapter's reply (everything up to
// its '>' prompt, headers on, spaces off, ISO-TP frames one per line) into the UDS payload.
// Everything the dongle returns is untrusted: lengths are checked, unknown lines are skipped,
// a frame that does not belong to the request is ignored, and nothing here can overrun.
// Plain C++, tested natively against replies recorded from cars (test/native/test_obd.cpp).
//
// A reply line with headers on and extended addressing looks like  607F1100762DD690000 :
//   607  CAN id (0x600 + ECU)      F1  extended address (us)
//   10 07  ISO-TP first frame, 7 bytes in total     62 DD 69 00 00  the first of them
// then  607F121006FFFFFFFFF : 21 consecutive frame 1, 00 6F, padding. Single frames carry
// 0n (length) instead. The adapter sends the flow control itself (ATFCSM1).
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace ObdProto {

enum class Reply : uint8_t {
  Ok,        // payload holds the bytes after 62 DD DD
  NoData,    // the adapter timed out waiting for the ECU (NO DATA), or the reply was empty
  Negative,  // the ECU refused (7F 22 nrc); nrc is set
  Malformed, // frames for this request arrived but did not assemble
  Adapter,   // the adapter reported an error (CAN ERROR, BUFFER FULL, STOPPED, ?, ...)
};

constexpr size_t MAX_PAYLOAD = 96;

// "22DD68\r" into out (at least 8 bytes).
void buildRequest(uint16_t did, char *out, size_t n);

// Parse one reply. text need not be terminated by the prompt; lines end in \r or \n.
Reply parseReply(const char *text, size_t textLen, uint8_t ecu, uint16_t did, uint8_t *payload, size_t payloadMax,
                 size_t *payloadLen, uint8_t *nrc);

// True when an AT command's reply contains OK (or, for ATZ and ATI, the ELM banner).
bool replyOk(const char *text);

const char *replyName(Reply r);

}  // namespace ObdProto
