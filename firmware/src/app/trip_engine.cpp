#include "trip_engine.h"

#include <math.h>
#include <string.h>

namespace {
constexpr float ARM_KPH = 1.0f;
constexpr float ARM_G = 0.05f;
constexpr uint32_t ARM_MS = 1000;
constexpr uint32_t MAX_GAP_MS = 300;
constexpr float GATE_SPEED_ACC = 0.5f;  // m/s
constexpr uint8_t GATE_SATS = 7;
constexpr float MOVING_KPH = 2.0f;

void stamp(char *out, size_t n, const char *clock) {
  strncpy(out, clock && clock[0] ? clock : "--:--", n - 1);
  out[n - 1] = 0;
}
}  // namespace

void TripEngine::reset(uint32_t nowMs) {
  trip_ = TripData();
  trip_.startMs = nowMs;
  session_ = RecordSet();
  accel_ = Timer();
  brake_ = Brake();
  roll_ = Rolling();
  powerAcc_ = 0;
  powerN_ = 0;
  powerWindowStartMs_ = nowMs;
  instMaxDraw_ = instMaxRegen_ = 0;
  lastMs_ = nowMs;
  lastSoc_ = -1;
  resetSessionPeaks();
}

void TripEngine::resetSessionPeaks() { peaks_ = MotionData(); }

float TripEngine::crossingMs(float v0, uint32_t t0, float v1, uint32_t t1, float target) {
  if (v1 == v0) return (float)t1;
  float f = (target - v0) / (v1 - v0);
  if (f < 0) f = 0;
  if (f > 1) f = 1;
  return (float)t0 + f * (float)(t1 - t0);
}

void TripEngine::accept(RecordId id, float value, float aux, uint32_t nowMs, float soc, float temp,
                        bool lowerIsBetter) {
  RecordSet *sets[3] = {&session_, &today_, &all_};
  bool beatAll = !all_.r[id].valid || (lowerIsBetter ? value < all_.r[id].value : value > all_.r[id].value);
  for (int s = 0; s < 3; s++) {
    Record &r = sets[s]->r[id];
    r.runs++;
    bool better = !r.valid || (lowerIsBetter ? value < r.value : value > r.value);
    if (!better) continue;
    if (r.valid) {
      r.previous = r.value;
      r.hasPrevious = true;
    }
    r.valid = true;
    r.value = value;
    r.aux = aux;
    r.socPct = soc;
    r.packTemp = temp;
    stamp(r.when, sizeof(r.when), clock_);
    r.beatLifetime = beatAll;
  }
}

void TripEngine::checkSpeedTimers(uint32_t nowMs, float kph, bool valid, float lonG, float speedAccMps,
                                  uint8_t sats, float soc, float temp) {
  Timer &t = accel_;
  bool gated = valid && speedAccMps < GATE_SPEED_ACC && sats >= GATE_SATS;
  bool gap = t.lastMs != 0 && nowMs - t.lastMs > MAX_GAP_MS;

  if (!valid || gap) {
    if (t.state == RunState::Running) t.state = RunState::Idle;  // invalidated run
    if (!valid) t.state = RunState::Idle;
  }

  switch (t.state) {
    case RunState::Idle:
    case RunState::Done:
      if (gated && kph < ARM_KPH && fabsf(lonG) < ARM_G) {
        if (t.stillSinceMs == 0) t.stillSinceMs = nowMs;
        if (nowMs - t.stillSinceMs >= ARM_MS) {
          t.state = RunState::Armed;
          t.crossed60 = t.crossed100 = false;
        }
      } else {
        t.stillSinceMs = 0;
      }
      break;
    case RunState::Armed:
      if (!gated || kph >= 5.0f) {  // moved off without a clean start, or lost the gate
        t.state = RunState::Idle;
        t.stillSinceMs = 0;
      } else if (kph >= ARM_KPH && t.lastKph < ARM_KPH) {
        t.startMs = (uint32_t)crossingMs(t.lastKph, t.lastMs, kph, nowMs, ARM_KPH);
        t.state = RunState::Running;
      }
      break;
    case RunState::Running:
      if (!gated) {
        t.state = RunState::Idle;
        break;
      }
      if (!t.crossed60 && kph >= 60.0f) {
        t.crossed60 = true;
        float ms = crossingMs(t.lastKph, t.lastMs, kph, nowMs, 60.0f) - (float)t.startMs;
        accept(REC_0_60, ms / 1000.0f, 0, nowMs, soc, temp, true);
      }
      if (!t.crossed100 && kph >= 100.0f) {
        t.crossed100 = true;
        float ms = crossingMs(t.lastKph, t.lastMs, kph, nowMs, 100.0f) - (float)t.startMs;
        accept(REC_0_100, ms / 1000.0f, 0, nowMs, soc, temp, true);
        t.state = RunState::Done;
      }
      if (kph < t.lastKph - 3.0f && !t.crossed100) t.state = RunState::Done;  // backed off: run over
      break;
  }

  // 60-100 rolling run, counted on its own.
  if (valid && !gap) {
    if (!roll_.running && t.lastKph < 60.0f && kph >= 60.0f) {
      roll_.running = true;
      roll_.startMs = (uint32_t)crossingMs(t.lastKph, t.lastMs, kph, nowMs, 60.0f);
    } else if (roll_.running) {
      if (kph >= 100.0f) {
        float s = (crossingMs(t.lastKph, t.lastMs, kph, nowMs, 100.0f) - (float)roll_.startMs) / 1000.0f;
        if (!(trip_.best60to100S > 0) || s < trip_.best60to100S) trip_.best60to100S = s;
        roll_.running = false;
      } else if (kph < t.lastKph - 2.0f || kph < 58.0f) {
        roll_.running = false;
      }
    }
  } else {
    roll_.running = false;
  }

  // 100-0 braking: time and distance from the downward 100 crossing to under 1 km/h.
  Brake &b = brake_;
  if (valid && !gap) {
    if (!b.armed && t.lastKph >= 100.0f && kph < 100.0f && lonG < -0.05f) {
      b.armed = true;
      b.startMs = (uint32_t)crossingMs(t.lastKph, t.lastMs, kph, nowMs, 100.0f);
      float dtS = (float)(nowMs - b.startMs) / 1000.0f;
      b.distanceM = (100.0f + kph) / 2.0f / 3.6f * dtS;
    } else if (b.armed) {
      float dtS = (float)(nowMs - b.lastMs) / 1000.0f;
      b.distanceM += (b.lastKph + kph) / 2.0f / 3.6f * dtS;
      if (kph > b.lastKph + 1.0f) {
        b.armed = false;  // speed rose again: not a stop
      } else if (kph < ARM_KPH) {
        float s = (crossingMs(b.lastKph, b.lastMs, kph, nowMs, ARM_KPH) - (float)b.startMs) / 1000.0f;
        accept(REC_100_0, s, b.distanceM, nowMs, soc, temp, true);
        b.armed = false;
      }
    }
    b.lastKph = kph;
    b.lastMs = nowMs;
  } else {
    b.armed = false;
  }

  t.lastKph = kph;
  t.lastMs = nowMs;
}

void TripEngine::checkPower(uint32_t nowMs, float powerKw, bool carLive, bool moving, float soc, float temp) {
  if (!carLive) {
    powerAcc_ = 0;
    powerN_ = 0;
    powerWindowStartMs_ = nowMs;
    return;
  }
  if (powerKw > instMaxDraw_) instMaxDraw_ = powerKw;
  if (-powerKw > instMaxRegen_) instMaxRegen_ = -powerKw;
  powerAcc_ += powerKw;
  powerN_++;
  if (nowMs - powerWindowStartMs_ >= 1000 && powerN_ > 0) {
    float mean = powerAcc_ / (float)powerN_;
    if (mean > 0 && moving) accept(REC_DRAW, mean, instMaxDraw_, nowMs, soc, temp, false);
    if (mean < 0 && moving) accept(REC_REGEN, -mean, instMaxRegen_, nowMs, soc, temp, false);
    powerAcc_ = 0;
    powerN_ = 0;
    powerWindowStartMs_ = nowMs;
  }
}

void TripEngine::update(uint32_t nowMs, float speedKph, bool speedValid, float speedAccMps, uint8_t sats,
                        float latG, float lonG, bool motionValid, float powerKw, bool carLive,
                        float socPct, float packTemp, float altM) {
  if (lastMs_ == 0) lastMs_ = nowMs;
  float dtS = (float)(nowMs - lastMs_) / 1000.0f;
  if (dtS < 0) dtS = 0;
  if (dtS > 0.5f) dtS = 0;  // a gap: do not integrate across it
  lastMs_ = nowMs;
  bool moving = speedValid && speedKph > MOVING_KPH;

  // Trip integration.
  if (speedValid) {
    trip_.distanceKm += speedKph / 3600.0f * dtS;
    if (moving) trip_.movingS += dtS;
    if (speedKph > trip_.maxKph) trip_.maxKph = speedKph;
    trip_.avgMovingKph = trip_.movingS > 0 ? trip_.distanceKm / (trip_.movingS / 3600.0f) : 0;
  }
  trip_.altM = altM;
  if (carLive) {
    if (powerKw > 0) trip_.drawKwh += powerKw * dtS / 3600.0f;
    else trip_.recoveredKwh += -powerKw * dtS / 3600.0f;
    trip_.netKwh = trip_.drawKwh - trip_.recoveredKwh;
    trip_.whPerKm = trip_.distanceKm > 0.5f ? 1000.0f * trip_.netKwh / trip_.distanceKm : 0;
    if (trip_.socStartPct < 0) trip_.socStartPct = socPct;
    trip_.socUsedPct = trip_.socStartPct - socPct;
    trip_.energyValid = true;
  }

  // Session peaks and lateral record from the display G path.
  if (motionValid) {
    if (latG < 0 && -latG > peaks_.peakLeft) peaks_.peakLeft = -latG;
    if (latG > 0 && latG > peaks_.peakRight) peaks_.peakRight = latG;
    if (lonG > 0 && lonG > peaks_.peakAccel) peaks_.peakAccel = lonG;
    if (lonG < 0 && -lonG > peaks_.peakBrake) peaks_.peakBrake = -lonG;
    float lat = fabsf(latG);
    if (moving && lat > 0.15f && (!session_.r[REC_LAT].valid || lat > session_.r[REC_LAT].value)) {
      accept(REC_LAT, lat, latG, nowMs, socPct, packTemp, false);
    }
  }

  checkSpeedTimers(nowMs, speedKph, speedValid, lonG, speedAccMps, sats, socPct, packTemp);
  checkPower(nowMs, powerKw, carLive, moving, socPct, packTemp);
}

void TripEngine::publish(VehicleState &vs, const char *clockHHMM) const {
  if (clockHHMM) strncpy(clock_, clockHHMM, sizeof(clock_) - 1);
  vs.trip = trip_;
  vs.motion.peakLeft = peaks_.peakLeft;
  vs.motion.peakRight = peaks_.peakRight;
  vs.motion.peakAccel = peaks_.peakAccel;
  vs.motion.peakBrake = peaks_.peakBrake;
  vs.recTrip = session_;
  vs.recToday = today_;
  vs.recAll = all_;
}
