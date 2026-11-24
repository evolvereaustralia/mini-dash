#include "obd_decode.h"

#include <math.h>
#include <stdio.h>

#include "obd_pids.h"

namespace {
uint16_t u16(const uint8_t *p, size_t at) { return (uint16_t)((p[at] << 8) | p[at + 1]); }
int16_t s16(const uint8_t *p, size_t at) { return (int16_t)u16(p, at); }
uint32_t u32(const uint8_t *p, size_t at) {
  return ((uint32_t)p[at] << 24) | ((uint32_t)p[at + 1] << 16) | ((uint32_t)p[at + 2] << 8) | p[at + 3];
}
bool in(float v, float lo, float hi) { return v >= lo && v <= hi; }  // false for NaN too
}  // namespace

namespace ObdDecode {

bool apply(uint8_t ecu, uint16_t did, const uint8_t *p, size_t n, uint32_t nowMs, CarData &car) {
  if (ecu == ECU_SME && did == DID_SME_VOLTS) {
    if (n < 2) return false;
    float v = u16(p, 0) / 100.0f;
    if (!in(v, 150.0f, 450.0f)) return false;  // 96 cells: 240 V empty, 403 V full
    car.packV = v;
    return true;  // the fast stamp goes with the current, which completes the power figure
  }
  if (ecu == ECU_SME && did == DID_SME_AMPS) {
    if (n < 4) return false;
    float a = -((int32_t)u32(p, 0)) / 100.0f;  // the car counts charging positive; we count discharge
    if (!in(a, -600.0f, 600.0f)) return false;
    if (fabsf(a) < 0.01f) a = 0;
    car.packA = a;
    if (car.packV > 0) {
      car.powerKw = car.packV * a / 1000.0f;
      car.fastStampMs = nowMs;
    }
    return true;
  }
  if (ecu == ECU_SME && did == DID_SME_SOC) {
    if (n < 6) return false;
    float raw = u16(p, 0) / 10.0f, hi = u16(p, 2) / 10.0f, lo = u16(p, 4) / 10.0f;
    if (!in(raw, 0, 100) || !in(hi, 50, 100) || !in(lo, 0, 50) || hi - lo < 20) return false;
    car.socRawPct = raw;
    car.socMaxPct = hi;
    car.socMinPct = lo;
    float shown = (raw - lo) / (hi - lo) * 100.0f;
    car.socPct = shown < 0 ? 0 : (shown > 100 ? 100 : shown);
    car.slowStampMs = nowMs;
    return true;
  }
  if (ecu == ECU_SME && did == DID_SME_TEMPS) {
    if (n < 6) return false;
    float lo = s16(p, 0) / 100.0f, hi = s16(p, 2) / 100.0f, mean = s16(p, 4) / 100.0f;
    if (!in(lo, -40, 90) || !in(hi, -40, 90) || !in(mean, -40, 90) || lo > hi + 0.5f) return false;
    car.tempMin = lo;
    car.tempMax = hi;
    car.tempAvg = mean;
    car.slowStampMs = nowMs;
    return true;
  }
  if (ecu == ECU_SME && did == DID_SME_CELLS) {
    if (n < 12) return false;
    float lo = u16(p, 6) / 10000.0f, hi = u16(p, 8) / 10000.0f, mean = u16(p, 10) / 10000.0f;
    if (!in(lo, 1.5f, 4.5f) || !in(hi, 1.5f, 4.5f) || !in(mean, 1.5f, 4.5f) || lo > hi + 0.001f) return false;
    car.cellMinV = lo;
    car.cellMaxV = hi;
    car.cellAvgV = mean;
    return true;
  }
  if (ecu == ECU_SME && did == DID_SME_SOH) {
    if (n < 4) return false;
    uint32_t v = u32(p, 0);
    if (v < 20 || v > 110) return false;
    car.sohPct = (float)v;
    return true;
  }
  if (ecu == ECU_EME && did == DID_EME_MACHINE) {
    if (n < 6) return false;
    float rpm = u16(p, 0) * 0.5f - 5000.0f, tq = s16(p, 2) * 0.5f, req = s16(p, 4) * 0.5f;
    if (!in(rpm, -5000, 20000) || !in(tq, -500, 500) || !in(req, -500, 500)) return false;
    car.motorRpm = rpm;
    car.torqueNm = tq;
    car.torqueReqNm = req;
    car.driveStampMs = nowMs;
    return true;
  }
  if (ecu == ECU_EME && did == DID_EME_MOTOR_TEMPS) {
    if (n < 4) return false;
    float t1 = s16(p, 0) * 0.0156f, t2 = s16(p, 2) * 0.0156f;
    if (!in(t1, -40, 220) || !in(t2, -40, 220)) return false;
    car.motorTemp1 = t1;
    car.motorTemp2 = t2;
    return true;
  }
  if (ecu == ECU_EDM && did == DID_EDM_PEDAL) {
    if (n < 6) return false;
    float pct = u16(p, 4) * 0.0625f;
    if (!in(pct, 0, 110)) return false;
    car.pedalPct = pct > 100 ? 100 : pct;
    return true;
  }
  if (ecu == ECU_EDM && did == DID_EDM_IBS) {
    if (n < 4) return false;
    float v = u16(p, 2) / 4000.0f + 6.0f;
    if (!in(v, 6.5f, 17.0f)) return false;
    car.v12 = v;
    return true;
  }
  if ((ecu == ECU_DSC && did == DID_DSC_WHEELS) || (ecu == ECU_NBT && did == DID_NBT_WHEELS)) {
    if (n < 8) return false;
    float div = ecu == ECU_DSC ? 100.0f : 1.0f;
    float w[4], sum = 0;
    for (int i = 0; i < 4; i++) {
      w[i] = s16(p, (size_t)i * 2) / div;
      if (!in(w[i], -60, 300)) return false;
      sum += fabsf(w[i]);
    }
    for (int i = 0; i < 4; i++) car.wheelKph[i] = fabsf(w[i]);
    car.wheelAvgKph = sum / 4.0f;
    car.wheelStampMs = nowMs;
    return true;
  }
  if (ecu == ECU_KOM && did == DID_KOM_RANGE) {
    if (n < 2) return false;
    uint16_t raw = u16(p, 0);
    if (raw == 0xFFFF) return false;
    float km = raw / 10.0f;
    if (!in(km, 0, 600)) return false;
    car.rangeKm = km;
    return true;
  }
  return false;
}

void describe(uint8_t ecu, uint16_t did, const CarData &car, char *out, size_t outLen) {
  if (ecu == ECU_SME && did == DID_SME_VOLTS) snprintf(out, outLen, "pack %.2f V", car.packV);
  else if (ecu == ECU_SME && did == DID_SME_AMPS) snprintf(out, outLen, "pack %+.2f A (discharge positive), %+.2f kW", car.packA, car.powerKw);
  else if (ecu == ECU_SME && did == DID_SME_SOC)
    snprintf(out, outLen, "SOC raw %.1f %%, limits %.1f to %.1f %%, shown %.1f %%", car.socRawPct, car.socMinPct, car.socMaxPct, car.socPct);
  else if (ecu == ECU_SME && did == DID_SME_TEMPS) snprintf(out, outLen, "pack temp min %.2f max %.2f mean %.2f C", car.tempMin, car.tempMax, car.tempAvg);
  else if (ecu == ECU_SME && did == DID_SME_CELLS) snprintf(out, outLen, "cells min %.4f max %.4f mean %.4f V", car.cellMinV, car.cellMaxV, car.cellAvgV);
  else if (ecu == ECU_SME && did == DID_SME_SOH) snprintf(out, outLen, "SOH %.0f %%", car.sohPct);
  else if (ecu == ECU_EME && did == DID_EME_MACHINE)
    snprintf(out, outLen, "motor %.0f rpm, torque %.1f Nm, requested %.1f Nm", car.motorRpm, car.torqueNm, car.torqueReqNm);
  else if (ecu == ECU_EME && did == DID_EME_MOTOR_TEMPS) snprintf(out, outLen, "motor temps %.1f and %.1f C", car.motorTemp1, car.motorTemp2);
  else if (ecu == ECU_EDM && did == DID_EDM_PEDAL) snprintf(out, outLen, "pedal %.1f %%", car.pedalPct);
  else if (ecu == ECU_EDM && did == DID_EDM_IBS) snprintf(out, outLen, "12 V battery %.3f V", car.v12);
  else if (did == DID_DSC_WHEELS || did == DID_NBT_WHEELS)
    snprintf(out, outLen, "wheels %.2f %.2f %.2f %.2f km/h, mean %.2f", car.wheelKph[0], car.wheelKph[1], car.wheelKph[2], car.wheelKph[3], car.wheelAvgKph);
  else if (ecu == ECU_KOM && did == DID_KOM_RANGE) snprintf(out, outLen, "range %.1f km", car.rangeKm);
  else snprintf(out, outLen, "(no decoder)");
}

}  // namespace ObdDecode
