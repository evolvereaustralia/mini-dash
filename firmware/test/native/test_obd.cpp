// Native test of the OBD protocol layer: the ELM reply parser and the value decoder, against
// replies recorded from cars in the OBDb MINI-Cooper repo (commit ac00b0bc, tests/test_cases;
// each case names its file) and against the failure shapes an adapter produces. Run with
// test/native/run.sh. Non-zero exit on any failure.
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../../include/obd_pids.h"
#include "../../src/app/obd_decode.h"
#include "../../src/app/obd_proto.h"

static int g_fail = 0, g_run = 0;
#define CHECK(cond, ...)                         \
  do {                                           \
    g_run++;                                     \
    if (!(cond)) {                               \
      g_fail++;                                  \
      printf("FAIL line %d: ", __LINE__);        \
      printf(__VA_ARGS__);                       \
      printf("\n");                              \
    }                                            \
  } while (0)
static bool near(float a, float b, float tol) { return fabsf(a - b) <= tol; }

static ObdProto::Reply run(const char *text, uint8_t ecu, uint16_t did, CarData &car, bool *applied = nullptr, uint8_t *nrc = nullptr) {
  uint8_t pl[ObdProto::MAX_PAYLOAD];
  size_t n = 0;
  ObdProto::Reply r = ObdProto::parseReply(text, strlen(text), ecu, did, pl, sizeof(pl), &n, nrc);
  bool ok = r == ObdProto::Reply::Ok && ObdDecode::apply(ecu, did, pl, n, 1000, car);
  if (applied) *applied = ok;
  return r;
}

int main() {
  using ObdProto::Reply;
  CarData car;
  bool ap;

  // --- Recorded replies -------------------------------------------------------------------
  // 2023/6F1.607.22DD68: single frame, 345.53 V
  CHECK(run("607F10562DD6886F9\r\r>", ECU_SME, DID_SME_VOLTS, car, &ap) == Reply::Ok && ap, "DD68 parse");
  CHECK(near(car.packV, 345.53f, 0.005f), "DD68 volts %.2f", car.packV);
  // 2020/6F1.607.22DD69: two frames; OBDb's signed value -95.56 A is the car's convention
  // (charging positive), so ours is +95.56 A discharge. Vector rebuilt from that case.
  CHECK(run("607F1100762DD69FFFF\r607F121DAACFFFFFFFF\r>", ECU_SME, DID_SME_AMPS, car, &ap) == Reply::Ok && ap, "DD69 parse");
  CHECK(near(car.packA, 95.56f, 0.005f), "DD69 amps %.2f", car.packA);
  CHECK(near(car.powerKw, 345.53f * 95.56f / 1000.0f, 0.01f) && car.fastStampMs == 1000, "power %.2f", car.powerKw);
  // 2020/6F1.607.22DD69: 1.11 A in the car's convention
  CHECK(run("607F1100762DD690000\r607F121006FFFFFFFFF\r>", ECU_SME, DID_SME_AMPS, car, &ap) == Reply::Ok && ap && near(car.packA, -1.11f, 0.005f),
        "DD69 small %.2f", car.packA);
  // 2023/6F1.607.22DDBC: SOC 74.4, max 97.0, min 11.5
  CHECK(run("607F1100962DDBC02E8\r607F12103CA0073FFFF\r>", ECU_SME, DID_SME_SOC, car, &ap) == Reply::Ok && ap, "DDBC parse");
  CHECK(near(car.socRawPct, 74.4f, 0.01f) && near(car.socMaxPct, 97.0f, 0.01f) && near(car.socMinPct, 11.5f, 0.01f), "DDBC values");
  CHECK(near(car.socPct, (74.4f - 11.5f) / (97.0f - 11.5f) * 100.0f, 0.01f), "DDBC shown %.2f", car.socPct);
  // 2023/6F1.607.22DDC0: min 9.15, max 11.98
  CHECK(run("607F1100962DDC00393\r607F12104AE042BFFFF\r>", ECU_SME, DID_SME_TEMPS, car, &ap) == Reply::Ok && ap, "DDC0 parse");
  CHECK(near(car.tempMin, 9.15f, 0.005f) && near(car.tempMax, 11.98f, 0.005f) && near(car.tempAvg, 10.67f, 0.005f), "DDC0 values %.2f %.2f %.2f",
        car.tempMin, car.tempMax, car.tempAvg);
  // 2020/6F1.607.22DFA0: 41 bytes over seven frames, preceded by a stale DD68 single frame
  // exactly as recorded. Cell min 3.4496, max 3.4718, mean 3.4613.
  CHECK(run("607F10562DD6881CA\r607F1102962DFA023BB\r607F121255124A486C0\r607F122879E87350A9B\r607F1230CE50BDFFFFF\r607F124FFFF2C750AEE\r"
            "607F1250BE70B3DFFFF\r607F126FFFFFFFF2C75\r>",
            ECU_SME, DID_SME_CELLS, car, &ap) == Reply::Ok && ap, "DFA0 parse");
  CHECK(near(car.cellMinV, 3.4496f, 0.00005f) && near(car.cellMaxV, 3.4718f, 0.00005f) && near(car.cellAvgV, 3.4613f, 0.00005f), "DFA0 cells %.4f %.4f %.4f",
        car.cellMinV, car.cellMaxV, car.cellAvgV);
  // 2023/6F1.607.226335: SOH 97
  CHECK(run("607F110076263350000\r607F1210061FFFFFFFF\r>", ECU_SME, DID_SME_SOH, car, &ap) == Reply::Ok && ap && car.sohPct == 97.0f, "6335 %.0f", car.sohPct);
  // 2022/6F1.61A.22DEA7: 169 rpm, -12.5 Nm, requested -13 Nm
  CHECK(run("61AF1100A62DEA72862\r61AF121FFE7FFE605FF\r>", ECU_EME, DID_EME_MACHINE, car, &ap) == Reply::Ok && ap, "DEA7 parse");
  CHECK(near(car.motorRpm, 169.0f, 0.01f) && near(car.torqueNm, -12.5f, 0.01f) && near(car.torqueReqNm, -13.0f, 0.01f), "DEA7 %.1f %.1f %.1f", car.motorRpm,
        car.torqueNm, car.torqueReqNm);
  // 2022/6F1.61A.22DEA6: 30.108 and 30.0144 C
  CHECK(run("61AF1100762DEA6078A\r61AF1210784FFFFFFFF\r>", ECU_EME, DID_EME_MOTOR_TEMPS, car, &ap) == Reply::Ok && ap, "DEA6 parse");
  CHECK(near(car.motorTemp1, 30.108f, 0.01f) && near(car.motorTemp2, 30.0144f, 0.01f), "DEA6 %.3f %.3f", car.motorTemp1, car.motorTemp2);
  // 2025/6F1.629.22DBE4: wheels 10.39, 11.99, 9.98, 11.69 km/h
  CHECK(run("629F1101662DBE4040F\r629F12104AF03E60491\r629F122045F01010101\r629F1230164646464\r>", ECU_DSC, DID_DSC_WHEELS, car, &ap) == Reply::Ok && ap,
        "DBE4 parse");
  CHECK(near(car.wheelKph[0], 10.39f, 0.005f) && near(car.wheelKph[1], 11.99f, 0.005f) && near(car.wheelKph[2], 9.98f, 0.005f) &&
            near(car.wheelKph[3], 11.69f, 0.005f) && near(car.wheelAvgKph, 11.0125f, 0.005f) && car.wheelStampMs == 1000,
        "DBE4 wheels");
  // 2022/6F1.612.22DE9C: pedal 17.25 %
  CHECK(run("612F1100962DE9C011E\r612F121008E0114FFFF\r>", ECU_EDM, DID_EDM_PEDAL, car, &ap) == Reply::Ok && ap && near(car.pedalPct, 17.25f, 0.005f),
        "DE9C %.2f", car.pedalPct);
  // 2022/6F1.612.22402B: 12 V battery 14.508 V
  CHECK(run("612F1101F62402B00B4\r612F12184F009680000\r612F122080000810002\r612F123000000000000\r612F124000000050000\r612F1250000FFFFFFFF\r>", ECU_EDM,
            DID_EDM_IBS, car, &ap) == Reply::Ok && ap && near(car.v12, 14.508f, 0.001f),
        "402B %.3f", car.v12);
  // 2023/6F1.660.22D111: range 42 km
  CHECK(run("660F1100F62D11101A4\r660F12106E001A4FFFF\r660F1223C660027FFFF\r>", ECU_KOM, DID_KOM_RANGE, car, &ap) == Reply::Ok && ap &&
            near(car.rangeKm, 42.0f, 0.01f),
        "D111 %.1f", car.rangeKm);

  // --- Adapter and ECU failure shapes -----------------------------------------------------
  CarData before = car;
  uint8_t nrc = 0;
  CHECK(run("NO DATA\r\r>", ECU_SME, DID_SME_VOLTS, car) == Reply::NoData, "NO DATA");
  CHECK(run("\r>", ECU_SME, DID_SME_VOLTS, car) == Reply::NoData, "empty");
  CHECK(run("CAN ERROR\r\r>", ECU_SME, DID_SME_VOLTS, car) == Reply::Adapter, "CAN ERROR");
  CHECK(run("BUFFER FULL\r>", ECU_SME, DID_SME_VOLTS, car) == Reply::Adapter, "BUFFER FULL");
  CHECK(run("?\r>", ECU_SME, DID_SME_VOLTS, car) == Reply::Adapter, "?");
  CHECK(run("607F1037F2231\r>", ECU_SME, DID_SME_VOLTS, car, nullptr, &nrc) == Reply::Negative && nrc == 0x31, "negative response nrc %02X", nrc);
  // response pending, then the answer
  CHECK(run("607F1037F2278\r607F10562DD6886F9\r>", ECU_SME, DID_SME_VOLTS, car, &ap) == Reply::Ok && ap, "pending then answer");
  // a reply from another ECU only, and a reply to another DID only
  CHECK(run("61AF10562DD6886F9\r>", ECU_SME, DID_SME_VOLTS, car) == Reply::NoData, "wrong ECU");
  CHECK(run("607F10562DD6986F9\r>", ECU_SME, DID_SME_VOLTS, car) == Reply::Malformed, "wrong DID");
  // truncated multi-frame, wrong sequence number, garbage, absurd length
  CHECK(run("607F1100962DDBC02E8\r>", ECU_SME, DID_SME_SOC, car) == Reply::Malformed, "missing consecutive frame");
  CHECK(run("607F1100962DDBC02E8\r607F12203CA0073FFFF\r>", ECU_SME, DID_SME_SOC, car) == Reply::Malformed, "sequence skip");
  CHECK(run("607F1ZZ62DD68\r>", ECU_SME, DID_SME_VOLTS, car) != Reply::Ok, "non-hex");
  CHECK(run("607F11FFF62DFA0\r>", ECU_SME, DID_SME_CELLS, car) == Reply::Malformed, "4095-byte first frame");
  CHECK(run("607F10562DD68\r>", ECU_SME, DID_SME_VOLTS, car) == Reply::Malformed, "single frame shorter than its length");
  // spaces on (ATS1) must parse the same
  CHECK(run("607 F1 05 62 DD 68 86 F9\r>", ECU_SME, DID_SME_VOLTS, car, &ap) == Reply::Ok && ap, "spaces on");
  // out-of-range values are refused and change nothing: 20 V pack, SOC limits reversed, 0xFFFF range
  car = before;
  CHECK(run("607F10562DD6807D0\r>", ECU_SME, DID_SME_VOLTS, car, &ap) == Reply::Ok && !ap && car.packV == before.packV, "20 V refused");
  CHECK(run("607F1100962DDBC02E8\r607F1210073" "03CAFFFF\r>", ECU_SME, DID_SME_SOC, car, &ap) == Reply::Ok && !ap && car.socPct == before.socPct,
        "reversed SOC limits refused");
  CHECK(run("660F1100F62D111FFFF\r660F12106E001A4FFFF\r660F1223C660027FFFF\r>", ECU_KOM, DID_KOM_RANGE, car, &ap) == Reply::Ok && !ap, "range FFFF refused");
  // request text
  char req[12];
  ObdProto::buildRequest(DID_SME_VOLTS, req, sizeof(req));
  CHECK(strcmp(req, "22DD68\r") == 0, "request '%s'", req);
  CHECK(ObdProto::replyOk("OK\r\r>") && ObdProto::replyOk("\rELM327 v2.2\r\r>") && !ObdProto::replyOk("?\r>"), "replyOk");

  printf("obd protocol: %d checks, %d failed\n", g_run, g_fail);
  printf(g_fail ? "RESULT: FAIL\n" : "RESULT: PASS\n");
  return g_fail ? 1 : 0;
}
