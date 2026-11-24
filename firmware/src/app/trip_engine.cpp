#include "trip_engine.h"

#include <math.h>
#include <string.h>

#include "clock.h"

namespace {
constexpr float ARM_KPH = 1.0f;
constexpr float ARM_G = 0.05f;
constexpr uint32_t ARM_MS = 1000;
constexpr float BRAKE_FROM_KPH = 60.0f;
constexpr float ELIGIBLE_KM = 0.05f;  // a trip of 50 m or more is kept (owner's decision D2)
constexpr uint32_t MAX_GAP_MS = 300;
constexpr float GATE_SPEED_ACC = 0.5f;  // m/s
constexpr uint8_t GATE_SATS = 7;
constexpr float MOVING_KPH = 2.0f;
constexpr uint32_t EVENT_TAIL_MS = 2000;

void stampWhen(Record &r, bool dateNotTime) {
  uint32_t utc = Clock::nowUtc();
  r.utc = utc;
  if (utc == 0) {
    strncpy(r.when, "--:--", sizeof(r.when) - 1);
  } else if (dateNotTime) {
    Clock::formatDayMonth(utc, r.when, sizeof(r.when));
  } else {
    Clock::formatHM(utc, r.when, sizeof(r.when));
  }
}
}  // namespace

void TripEngine::startTrip(uint32_t tripId, uint32_t nowMs, uint32_t startUtc) {
  movingKm_ = 0;
  eligible_ = eligibleEvent_ = false;
  tripId_ = tripId;
  startUtc_ = startUtc;
  startMs_ = nowMs;
  trip_ = TripData();
  trip_.startMs = nowMs;
  session_ = RecordSet();
  accel_ = Timer();
  brake_ = Brake();
  roll_ = Rolling();
  powerAcc_ = powerSpanMs_ = 0;
  powerWindowStartMs_ = nowMs;
  powerLastMs_ = 0;
  instMaxDraw_ = instMaxRegen_ = 0;
  ringHead_ = ringCount_ = 0;
  lastTraceMs_ = 0;
  memset(pendingMs_, 0, sizeof(pendingMs_));
  lastMs_ = nowMs;
  resetSessionPeaks();
}

void TripEngine::resetSessionPeaks() { peaks_ = MotionData(); }

double TripEngine::crossingMs(float v0, uint32_t t0, float v1, uint32_t t1, float target) {
  if (v1 == v0) return (double)t1;
  double f = ((double)target - v0) / ((double)v1 - v0);
  if (f < 0) f = 0;
  if (f > 1) f = 1;
  return (double)t0 + f * (double)(t1 - t0);
}

void TripEngine::accept(RecordId id, float value, float aux, uint32_t nowMs, float soc, float temp,
                        bool lowerIsBetter) {
  bool beatAll = !isolated_ && (!all_.r[id].valid || (lowerIsBetter ? value < all_.r[id].value : value > all_.r[id].value));
  RecordSet *sets[3] = {&session_, &today_, &all_};
  // Until the trip has covered 50 m it may yet be discarded (owner's decision D2), so nothing
  // it does reaches Today or the lifetime set; promote() carries the session over when it
  // qualifies. Captured runs are logged, never recorded (capture builds).
  if (candidateLog_) candidateLog_(id, value, aux, nowMs);
  if (captureOnly_) return;
  int nSets = (isolated_ || !eligible_) ? 1 : 3;
  bool newBest = false;  // the session record changed: its trace is the one worth keeping
  for (int s = 0; s < nSets; s++) {
    Record &r = sets[s]->r[id];
    r.runs++;
    bool better = !r.valid || (lowerIsBetter ? value < r.value : value > r.value);
    if (!better) continue;
    if (s == 0) newBest = true;
    if (r.valid) {
      r.previous = r.value;
      r.hasPrevious = true;
    } else if (s == 0 && lastTrip_.r[id].valid) {
      r.previous = lastTrip_.r[id].value;  // first of this trip: compare with the last trip
      r.hasPrevious = true;
    }
    r.valid = true;
    r.value = value;
    r.aux = aux;
    r.socPct = soc;
    r.packTemp = temp;
    r.tripId = tripId_;
    stampWhen(r, s == 2);
    r.beatLifetime = beatAll;
  }
  // The lifetime set changed if a run was counted in it, record or not (review finding 25).
  if (nSets == 3) lifetimeDirty_ = true;
  // Hand the trace over once the tail has been captured, but only for a new session best: every
  // valid run counts (r.runs), yet the stored trace must stay the record's own. Before this
  // check, the demo's slow 0-50-60 cruise replaced the 2.6 s run's trace with 12 s of cruising
  // (found 24 Oct 2026 from a trace dump, key Q).
  if (newBest) pendingMs_[id] = nowMs + EVENT_TAIL_MS;
}

// The session's records into another set: the better value wins, the run counts add. Used when
// a trip qualifies, when Today is rebuilt, and at boot to fold a recovered trip into the
// lifetime set (the same merge twice changes nothing, so a repeat after a power cut is safe).
bool TripEngine::merge(RecordSet &into, const RecordSet &from, bool addRuns, bool dateStamp) {
  bool changed = false;
  for (int id = 0; id < REC_COUNT; id++) {
    const Record &f = from.r[id];
    if (!f.valid) continue;
    Record &t = into.r[id];
    bool lower = id <= REC_60_0;
    uint32_t runs = (uint32_t)(t.valid ? t.runs : 0) + (addRuns ? f.runs : 0);
    if (!t.valid || (lower ? f.value < t.value : f.value > t.value)) {
      float prev = t.value;
      bool hadPrev = t.valid;
      t = f;
      t.previous = prev;
      t.hasPrevious = hadPrev;
      if (dateStamp && t.utc) Clock::formatDayMonth(t.utc, t.when, sizeof(t.when));
      changed = true;
    }
    if (addRuns) {
      uint16_t r16 = runs > 65535 ? 65535 : (uint16_t)runs;
      if (t.runs != r16) changed = true;
      t.runs = r16;
    } else if (!t.runs) {
      t.runs = f.runs;
    }
  }
  return changed;
}

void TripEngine::promote() {
  eligible_ = true;
  eligibleEvent_ = true;
  if (isolated_ || captureOnly_) return;
  for (int id = 0; id < REC_COUNT; id++)
    if (session_.r[id].valid) session_.r[id].beatLifetime = !all_.r[id].valid || (id <= REC_60_0 ? session_.r[id].value < all_.r[id].value : session_.r[id].value > all_.r[id].value);
  merge(today_, session_, true);
  if (merge(all_, session_, true, true)) lifetimeDirty_ = true;
}

void TripEngine::rebaseToday(const RecordSet &fromFiles) {
  today_ = fromFiles;
  if (eligible_ && !isolated_ && !captureOnly_) merge(today_, session_, true);
}

bool TripEngine::takeLifetimeIfDirty(RecordSet &out) {
  if (!lifetimeDirty_) return false;
  lifetimeDirty_ = false;
  out = all_;
  return true;
}

void TripEngine::traceSample(uint32_t nowMs, float kph, float latG, float lonG, float kw) {
  if (lastTraceMs_ != 0 && nowMs - lastTraceMs_ < 100) return;
  lastTraceMs_ = nowMs;
  Storage::TraceSample &t = ring_[ringHead_];
  t.t10 = (uint16_t)((nowMs / 100) & 0xFFFF);
  t.kph10 = (int16_t)lroundf(kph * 10.0f);
  t.latMg = (int16_t)lroundf(latG * 1000.0f);
  t.lonMg = (int16_t)lroundf(lonG * 1000.0f);
  t.kw10 = (int16_t)lroundf(kw * 10.0f);
  ringHead_ = (ringHead_ + 1) % Storage::TRACE_MAX;
  if (ringCount_ < (int)Storage::TRACE_MAX) ringCount_++;
}

void TripEngine::flushEvents(uint32_t nowMs) {
  if (sink_ == nullptr) return;
  for (int id = 0; id < REC_COUNT; id++) {
    if (pendingMs_[id] == 0 || nowMs < pendingMs_[id]) continue;
    pendingMs_[id] = 0;
    Storage::TraceSample out[Storage::TRACE_MAX];
    int n = 0;
    uint16_t t0 = 0;
    for (int i = 0; i < ringCount_; i++) {
      int idx = (ringHead_ - ringCount_ + i + Storage::TRACE_MAX) % Storage::TRACE_MAX;
      out[n] = ring_[idx];
      if (n == 0) t0 = out[n].t10;
      out[n].t10 = (uint16_t)(out[n].t10 - t0);  // relative to the window start
      n++;
    }
    if (n > 0) sink_(tripId_, id, out, (size_t)n);
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
        t.startMs = crossingMs(t.lastKph, t.lastMs, kph, nowMs, ARM_KPH);
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
        double ms = crossingMs(t.lastKph, t.lastMs, kph, nowMs, 60.0f) - t.startMs;
        accept(REC_0_60, (float)(ms / 1000.0), 0, nowMs, soc, temp, true);
      }
      if (!t.crossed100 && kph >= 100.0f) {
        t.crossed100 = true;
        double ms = crossingMs(t.lastKph, t.lastMs, kph, nowMs, 100.0f) - t.startMs;
        accept(REC_0_100, (float)(ms / 1000.0), 0, nowMs, soc, temp, true);
        t.state = RunState::Done;
      }
      if (kph < t.lastKph - 3.0f && !t.crossed100) t.state = RunState::Done;  // backed off: run over
      break;
  }

  // 60-100 rolling run, counted on its own.
  if (valid && !gap) {
    if (!roll_.running && t.lastKph < 60.0f && kph >= 60.0f) {
      roll_.running = true;
      roll_.startMs = crossingMs(t.lastKph, t.lastMs, kph, nowMs, 60.0f);
    } else if (roll_.running) {
      if (kph >= 100.0f) {
        float s = (float)((crossingMs(t.lastKph, t.lastMs, kph, nowMs, 100.0f) - roll_.startMs) / 1000.0);
        if (!(trip_.best60to100S > 0) || s < trip_.best60to100S) trip_.best60to100S = s;
        roll_.running = false;
      } else if (kph < t.lastKph - 2.0f || kph < 58.0f) {
        roll_.running = false;
      }
    }
  } else {
    roll_.running = false;
  }

  // 60-0 braking: time and distance from the downward 60 crossing to under 1 km/h. It was
  // 100-0 until 24 Nov 2026: a stop from 100 km/h is not something Logan's roads allow, a
  // stop from 60 happens on every drive.
  Brake &b = brake_;
  if (valid && !gap) {
    if (!b.armed && t.lastKph >= BRAKE_FROM_KPH && kph < BRAKE_FROM_KPH && lonG < -0.05f) {
      b.armed = true;
      b.startMs = crossingMs(t.lastKph, t.lastMs, kph, nowMs, BRAKE_FROM_KPH);
      float dtS = (float)(((double)nowMs - b.startMs) / 1000.0);
      b.distanceM = (BRAKE_FROM_KPH + kph) / 2.0f / 3.6f * dtS;
    } else if (b.armed) {
      float dtS = (float)(nowMs - b.lastMs) / 1000.0f;
      b.distanceM += (b.lastKph + kph) / 2.0f / 3.6f * dtS;
      if (kph > b.lastKph + 1.0f) {
        b.armed = false;  // speed rose again: not a stop
      } else if (kph < ARM_KPH) {
        float s = (float)((crossingMs(b.lastKph, b.lastMs, kph, nowMs, ARM_KPH) - b.startMs) / 1000.0);
        accept(REC_60_0, s, b.distanceM, nowMs, soc, temp, true);
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
  // No car data, the first sample, or a gap: the window starts afresh at this sample, so a
  // reading never stands in for time it was not seen.
  if (!carLive || powerLastMs_ == 0 || nowMs - powerLastMs_ > 500) {
    powerAcc_ = powerSpanMs_ = 0;
    instMaxDraw_ = instMaxRegen_ = 0;
    powerWindowStartMs_ = nowMs;
    powerLastMs_ = carLive ? nowMs : 0;
    return;
  }
  float dtMs = (float)(nowMs - powerLastMs_);
  powerLastMs_ = nowMs;
  powerAcc_ += powerKw * dtMs;  // each reading weighted by the time it stood for
  powerSpanMs_ += dtMs;
  if (powerKw > instMaxDraw_) instMaxDraw_ = powerKw;
  if (-powerKw > instMaxRegen_) instMaxRegen_ = -powerKw;
  if (nowMs - powerWindowStartMs_ >= 1000 && powerSpanMs_ > 0) {
    float mean = powerAcc_ / powerSpanMs_;
    if (mean > 0 && moving) accept(REC_DRAW, mean, instMaxDraw_, nowMs, soc, temp, false);
    if (mean < 0 && moving) accept(REC_REGEN, -mean, instMaxRegen_, nowMs, soc, temp, false);
    powerAcc_ = powerSpanMs_ = 0;
    instMaxDraw_ = instMaxRegen_ = 0;  // the extremes belong to the second they were seen in
    powerWindowStartMs_ = nowMs;
  }
}

void TripEngine::update(uint32_t nowMs, float speedKph, bool speedValid, float timingKph, bool timingValid,
                        float speedAccMps, uint8_t sats, float latG, float lonG, bool motionValid,
                        float powerKw, bool carLive, float socPct, float packTemp, float altM) {
  if (lastMs_ == 0) lastMs_ = nowMs;
  float dtS = (float)(nowMs - lastMs_) / 1000.0f;
  if (dtS < 0) dtS = 0;
  if (dtS > 0.5f) dtS = 0;  // a gap: do not integrate across it
  lastMs_ = nowMs;
  bool moving = speedValid && speedKph > MOVING_KPH;

  // Trip integration.
  if (speedValid) {
    trip_.distanceKm += speedKph / 3600.0f * dtS;
    if (moving) {
      trip_.movingS += dtS;
      movingKm_ += speedKph / 3600.0f * dtS;  // the same intervals as the time, so creep does not inflate the average
    }
    if (speedKph > trip_.maxKph) trip_.maxKph = speedKph;
    if (!eligible_ && trip_.distanceKm >= ELIGIBLE_KM) promote();
    trip_.avgMovingKph = trip_.movingS > 0 ? movingKm_ / (trip_.movingS / 3600.0f) : 0;
  }
  trip_.altM = altM;
  if (carLive) {
    if (powerKw > 0) trip_.drawKwh += powerKw * dtS / 3600.0f;
    else trip_.recoveredKwh += -powerKw * dtS / 3600.0f;
    trip_.netKwh = trip_.drawKwh - trip_.recoveredKwh;
    trip_.whPerKm = trip_.distanceKm > 0.5f ? 1000.0f * trip_.netKwh / trip_.distanceKm : 0;
    // The baseline waits for a real SOC reading: power arrives first on a new link, and a
    // baseline of zero made "SOC used" read about minus 50 (review finding 14).
    if (socPct >= 0) {
      if (trip_.socStartPct < 0) trip_.socStartPct = socPct;
      trip_.socUsedPct = trip_.socStartPct - socPct;
    }
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

  checkSpeedTimers(nowMs, timingKph, timingValid, lonG, speedAccMps, sats, socPct, packTemp);
  checkPower(nowMs, powerKw, carLive, moving, socPct, packTemp);
  traceSample(nowMs, speedValid ? speedKph : 0, motionValid ? latG : 0, motionValid ? lonG : 0, carLive ? powerKw : 0);
  flushEvents(nowMs);
}

void TripEngine::publish(VehicleState &vs) const {
  vs.trip = trip_;
  vs.motion.peakLeft = peaks_.peakLeft;
  vs.motion.peakRight = peaks_.peakRight;
  vs.motion.peakAccel = peaks_.peakAccel;
  vs.motion.peakBrake = peaks_.peakBrake;
  vs.recTrip = session_;
  vs.recToday = today_;
  vs.recAll = all_;
  vs.tripId = tripId_;
  vs.tripStartUtc = startUtc_;
}

void TripEngine::summary(Storage::TripSummary &out, uint32_t nowMs, bool demo) const {
  out.id = tripId_;
  out.startUtc = startUtc_;
  out.durationS = (nowMs - startMs_) / 1000;
  out.endUtc = startUtc_ ? startUtc_ + out.durationS : 0;
  out.demo = demo;
  out.closed = false;
  out.trip = trip_;
  out.records = session_;
  out.peaks = peaks_;
}
