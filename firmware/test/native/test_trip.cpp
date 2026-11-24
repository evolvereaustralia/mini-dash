// Native test of the trip engine's record bookkeeping (review findings 04, 25 and the owner's
// decision D2): nothing a trip does reaches Today or the lifetime set until it has covered 50 m,
// a replay never reaches them at all, a capture build records nothing, and merging a recovered
// trip twice changes nothing. Build and run with test/native/run.sh.
#include <stdio.h>
#include <string.h>

#include "../../src/app/clock.h"
#include "../../src/app/trip_engine.h"

// The engine only needs the clock to stamp records.
static uint32_t g_utc = 1795000000UL;
namespace Clock {
uint32_t nowUtc() { return g_utc; }
void formatHM(uint32_t, char *out, size_t n) { snprintf(out, n, "14:12"); }
void formatDayMonth(uint32_t, char *out, size_t n) { snprintf(out, n, "24 Nov"); }
}  // namespace Clock

static int g_checks = 0, g_failed = 0;
#define CHECK(cond, what)                                   \
  do {                                                      \
    g_checks++;                                             \
    if (!(cond)) {                                          \
      g_failed++;                                           \
      printf("FAIL line %d: %s\n", __LINE__, what);         \
    }                                                       \
  } while (0)

static uint32_t g_now = 1000;
// Drive at a steady speed for a distance, 20 Hz, with a given lateral g and battery power.
static void drive(TripEngine &e, float kph, float metres, float latG, float kw = 0, bool car = false) {
  float steps = metres / (kph / 3.6f) / 0.05f;
  for (int i = 0; i < (int)(steps + 0.5f); i++) {
    g_now += 50;
    e.update(g_now, kph, true, kph, true, 0.2f, 12, latG, 0, true, kw, car, car ? 60.0f : -1.0f, car ? 25.0f : -273.0f, 40.0f);
  }
}
static int g_candidates = 0;
static void candidate(int, float, float, uint32_t) { g_candidates++; }

int main() {
  static TripEngine e;
  static VehicleState vs;
  RecordSet life;
  life.r[REC_LAT].valid = true;
  life.r[REC_LAT].value = 0.50f;
  life.r[REC_LAT].runs = 9;
  RecordSet out;

  // --- a trip that stops short of 50 m leaves no trace in Today or the lifetime set
  e.setLifetime(life);
  e.startTrip(7, g_now, g_utc);
  drive(e, 20, 30, 0.80f);
  e.publish(vs);
  CHECK(vs.recTrip.r[REC_LAT].valid && vs.recTrip.r[REC_LAT].value > 0.79f, "the trip itself shows its lateral peak");
  CHECK(vs.recAll.r[REC_LAT].value == 0.50f && vs.recAll.r[REC_LAT].runs == 9, "lifetime untouched before 50 m");
  CHECK(!vs.recToday.r[REC_LAT].valid, "today untouched before 50 m");
  CHECK(!e.eligible() && !e.takeEligibleEvent(), "not eligible at 30 m");
  CHECK(!e.takeLifetimeIfDirty(out), "lifetime not marked for writing before 50 m");
  CHECK(vs.trip.distanceKm > 0.029f && vs.trip.distanceKm < 0.031f, "30 m integrated");

  // --- the same trip carried on past 50 m: promoted once, with its earlier record
  drive(e, 20, 25, 0.0f);
  e.publish(vs);
  CHECK(e.eligible(), "eligible past 50 m");
  CHECK(e.takeEligibleEvent(), "eligibility reported");
  CHECK(!e.takeEligibleEvent(), "eligibility reported once only");
  CHECK(vs.recAll.r[REC_LAT].value > 0.79f, "lifetime takes the record on promotion");
  CHECK(vs.recAll.r[REC_LAT].hasPrevious && vs.recAll.r[REC_LAT].previous == 0.50f, "the lifetime comparison names the previous record");
  CHECK(vs.recAll.r[REC_LAT].runs == 9 + vs.recTrip.r[REC_LAT].runs, "lifetime run count is the old count plus this trip's");
  CHECK(strcmp(vs.recAll.r[REC_LAT].when, "24 Nov") == 0, "a lifetime record carries a date");
  CHECK(strcmp(vs.recToday.r[REC_LAT].when, "14:12") == 0, "today's record carries a time");
  CHECK(vs.recToday.r[REC_LAT].valid && vs.recToday.r[REC_LAT].value > 0.79f, "today takes the record on promotion");
  CHECK(vs.recTrip.r[REC_LAT].beatLifetime, "the trip tile knows it beat the lifetime record");
  CHECK(e.takeLifetimeIfDirty(out) && out.r[REC_LAT].value > 0.79f, "lifetime handed to the writer");
  CHECK(!e.takeLifetimeIfDirty(out), "and only once");

  // --- after promotion a better run goes straight to all three sets
  drive(e, 30, 20, -0.95f);
  e.publish(vs);
  CHECK(vs.recAll.r[REC_LAT].value > 0.94f && vs.recToday.r[REC_LAT].value > 0.94f, "a later record reaches today and lifetime at once");
  CHECK(vs.recAll.r[REC_LAT].aux < 0, "the side of the peak is kept");
  CHECK(e.takeLifetimeIfDirty(out), "lifetime marked again");
  e.markLifetimeDirty();
  CHECK(e.takeLifetimeIfDirty(out), "a failed write can be retried");

  // --- a new trip that never reaches 50 m: the earlier lifetime stays, nothing is marked
  RecordSet before = out;
  e.setLastTrip(before);  // what Persist does once a trip has been archived
  e.startTrip(8, g_now, g_utc);
  drive(e, 15, 10, 1.20f);
  e.publish(vs);
  CHECK(vs.recTrip.r[REC_LAT].value > 1.19f, "the short trip shows its own peak");
  CHECK(vs.recAll.r[REC_LAT].value == before.r[REC_LAT].value, "a short trip never reaches lifetime");
  CHECK(!e.takeLifetimeIfDirty(out), "and marks nothing for writing");
  CHECK(vs.recTrip.r[REC_LAT].hasPrevious, "the first record of a trip can be compared with something");

  // --- a replay (isolated trip) never reaches today or lifetime, however far it goes
  e.startTrip(9, g_now, g_utc);
  e.setIsolated(true);
  drive(e, 60, 400, 1.40f, 80.0f, true);
  e.publish(vs);
  CHECK(e.eligible(), "a replay still counts its distance");
  CHECK(vs.recTrip.r[REC_LAT].value > 1.39f && vs.recTrip.r[REC_DRAW].valid, "the replay shows its own records");
  CHECK(vs.recAll.r[REC_LAT].value == before.r[REC_LAT].value && !vs.recAll.r[REC_DRAW].valid, "a replay never reaches lifetime");
  CHECK(!e.takeLifetimeIfDirty(out), "a replay marks nothing for writing");
  CHECK(!vs.recTrip.r[REC_LAT].beatLifetime, "a replay never claims a lifetime record");
  e.setIsolated(false);

  // --- power records: one-second means, counted, and the energy integrates
  e.startTrip(10, g_now, g_utc);
  drive(e, 72, 200, 0.0f, 90.0f, true);  // 10 s at 20 m/s drawing 90 kW
  drive(e, 72, 100, 0.0f, -30.0f, true);  // 5 s recovering 30 kW
  e.publish(vs);
  CHECK(vs.recTrip.r[REC_DRAW].valid && vs.recTrip.r[REC_DRAW].value > 89.0f && vs.recTrip.r[REC_DRAW].value < 91.0f, "peak draw is the one-second mean");
  CHECK(vs.recTrip.r[REC_REGEN].valid && vs.recTrip.r[REC_REGEN].value > 29.0f && vs.recTrip.r[REC_REGEN].value < 31.0f, "peak regen is the one-second mean, positive");
  CHECK(vs.trip.drawKwh > 0.24f && vs.trip.drawKwh < 0.26f, "10 s at 90 kW is 0.25 kWh");
  CHECK(vs.trip.recoveredKwh > 0.040f && vs.trip.recoveredKwh < 0.043f, "5 s at 30 kW is 0.042 kWh");
  CHECK(vs.trip.socStartPct == 60.0f, "SOC baseline taken from the first reading");
  CHECK(vs.recAll.r[REC_DRAW].valid && vs.recAll.r[REC_DRAW].runs >= 9, "power runs counted in lifetime after 50 m");

  // --- capture build: runs are logged as candidates and never become records
  CHECK(e.takeLifetimeIfDirty(out), "the power trip marked lifetime for writing");
  e.startTrip(11, g_now, g_utc);
  e.setCaptureOnly(true, candidate);
  RecordSet allBefore = vs.recAll;
  drive(e, 40, 300, 1.60f, 100.0f, true);
  e.publish(vs);
  CHECK(g_candidates > 0, "capture: candidates are logged");
  CHECK(!vs.recTrip.r[REC_LAT].valid && !vs.recTrip.r[REC_DRAW].valid, "capture: nothing is recorded for the trip");
  CHECK(vs.recAll.r[REC_LAT].value == allBefore.r[REC_LAT].value, "capture: lifetime unchanged");
  CHECK(!e.takeLifetimeIfDirty(out), "capture: nothing marked for writing");
  e.setCaptureOnly(false, nullptr);

  // --- merging a recovered trip into the lifetime set: better wins, twice changes nothing
  RecordSet into, from;
  into.r[REC_0_60].valid = true;
  into.r[REC_0_60].value = 7.4f;
  into.r[REC_0_60].runs = 3;
  from.r[REC_0_60].valid = true;
  from.r[REC_0_60].value = 6.9f;
  from.r[REC_0_60].runs = 2;
  from.r[REC_0_60].utc = g_utc;
  from.r[REC_LAT].valid = true;
  from.r[REC_LAT].value = 0.7f;
  from.r[REC_LAT].runs = 1;
  CHECK(TripEngine::merge(into, from, false, true), "a faster recovered run changes the lifetime set");
  CHECK(into.r[REC_0_60].value == 6.9f && into.r[REC_0_60].previous == 7.4f && into.r[REC_0_60].hasPrevious, "lower is better for a timed run");
  CHECK(into.r[REC_LAT].valid && into.r[REC_LAT].value == 0.7f && !into.r[REC_LAT].hasPrevious, "a first record has no previous");
  CHECK(!TripEngine::merge(into, from, false, true), "merging the same trip again changes nothing");
  RecordSet slower;
  slower.r[REC_0_60].valid = true;
  slower.r[REC_0_60].value = 8.8f;
  slower.r[REC_0_60].runs = 4;
  uint16_t runsBefore = into.r[REC_0_60].runs;
  CHECK(TripEngine::merge(into, slower, true) && into.r[REC_0_60].value == 6.9f, "a slower run only adds to the count");
  CHECK(into.r[REC_0_60].runs == runsBefore + 4, "run counts add");

  // --- today rebuilt from files (late clock) keeps this trip's records only if it qualifies
  e.startTrip(12, g_now, 0);
  drive(e, 20, 20, 0.6f);
  RecordSet files;
  files.r[REC_LAT].valid = true;
  files.r[REC_LAT].value = 0.3f;
  files.r[REC_LAT].runs = 2;
  e.rebaseToday(files);
  e.publish(vs);
  CHECK(vs.recToday.r[REC_LAT].value == 0.3f, "before 50 m a rebuilt Today holds only the stored trips");
  drive(e, 20, 40, 0.0f);
  e.rebaseToday(files);
  e.publish(vs);
  CHECK(vs.recToday.r[REC_LAT].value > 0.59f, "after 50 m a rebuilt Today includes this trip");
  e.setStartUtc(g_utc - 60);
  e.publish(vs);
  CHECK(vs.tripStartUtc == g_utc - 60 && vs.trip.distanceKm > 0.059f, "a late clock dates the trip without restarting it");

  // --- finding 26: the power record is a time-weighted mean and carries its own second's extreme
  {
    static TripEngine pe;
    static VehicleState pv;
    g_now += 5000;
    pe.startTrip(20, g_now, g_utc);
    auto step = [&](uint32_t dtMs, float kw) {
      g_now += dtMs;
      pe.update(g_now, 50, true, 50, true, 0.2f, 12, 0, 0, true, kw, true, 60.0f, 25.0f, 40.0f);
    };
    step(50, 100);                                  // the first reading opens the window
    for (int i = 0; i < 19; i++) step(50, i == 5 ? 140.0f : 100.0f);  // one second at 100 kW with a 140 kW spike
    step(50, 100);
    pe.publish(pv);
    CHECK(pv.recTrip.r[REC_DRAW].valid && pv.recTrip.r[REC_DRAW].aux == 140.0f, "the first second's record carries its 140 kW spike");
    for (int i = 0; i < 40; i++) step(50, 110);     // two steadier, higher seconds
    pe.publish(pv);
    CHECK(pv.recTrip.r[REC_DRAW].value > 109.0f && pv.recTrip.r[REC_DRAW].value < 111.0f, "a higher one-second mean takes the record");
    CHECK(pv.recTrip.r[REC_DRAW].aux < 111.0f, "and its extreme is that second's own, not the earlier spike");
    // uneven sampling: 0.1 s at 200 kW seen ten times, then 0.9 s at 20 kW seen three times
    static TripEngine ue;
    g_now += 5000;
    ue.startTrip(21, g_now, g_utc);
    auto ustep = [&](uint32_t dtMs, float kw) {
      g_now += dtMs;
      ue.update(g_now, 50, true, 50, true, 0.2f, 12, 0, 0, true, kw, true, 60.0f, 25.0f, 40.0f);
    };
    ustep(10, 200);
    for (int i = 0; i < 10; i++) ustep(10, 200);
    for (int i = 0; i < 3; i++) ustep(300, 20);
    ue.publish(pv);
    CHECK(pv.recTrip.r[REC_DRAW].valid && pv.recTrip.r[REC_DRAW].value > 36.0f && pv.recTrip.r[REC_DRAW].value < 40.0f,
          "readings are weighted by the time they stood for (38 kW, not the 158 kW a per-sample mean gives)");
    // a gap in the car data does not let one reading stand in for the missing time
    static TripEngine ge;
    g_now += 5000;
    ge.startTrip(22, g_now, g_utc);
    g_now += 50;
    ge.update(g_now, 50, true, 50, true, 0.2f, 12, 0, 0, true, 150, true, 60.0f, 25.0f, 40.0f);
    g_now += 2000;  // nothing for two seconds
    ge.update(g_now, 50, true, 50, true, 0.2f, 12, 0, 0, true, 150, true, 60.0f, 25.0f, 40.0f);
    ge.publish(pv);
    CHECK(!pv.recTrip.r[REC_DRAW].valid, "two readings two seconds apart are not a one-second mean");
  }

  // --- finding 32: a timed run late in a long session keeps its precision
  {
    static TripEngine te;
    static VehicleState tv;
    uint32_t t = 30000000UL;  // 8 h 20 min of uptime: a float can no longer hold single milliseconds here
    te.startTrip(30, t, g_utc);
    for (int i = 0; i < 15; i++) {  // standing still with a good fix: arms after 1 s
      t += 100;
      te.update(t, 0, true, 0, true, 0.2f, 12, 0, 0, true, 0, false, -1, -273, 40);
    }
    for (int i = 1; i <= 31; i++) {  // 2 km/h more every 100 ms sample: 0.57 g, a brisk launch
      t += 100;
      te.update(t, i * 2.0f, true, i * 2.0f, true, 0.2f, 12, 0, 0.57f, true, 0, false, -1, -273, 40);
    }
    te.publish(tv);
    // the start is the 1 km/h crossing, 50 ms into the first step; 60 km/h is reached at 3000 ms
    CHECK(tv.recTrip.r[REC_0_60].valid, "the run is timed");
    printf("0-60 at 8 h uptime: %.4f s (exact 2.9500)\n", tv.recTrip.r[REC_0_60].value);
    CHECK(tv.recTrip.r[REC_0_60].value > 2.9495f && tv.recTrip.r[REC_0_60].value < 2.9505f, "to within half a millisecond");
  }

  printf("trip engine: %d checks, %d failed\nRESULT: %s\n", g_checks, g_failed, g_failed ? "FAIL" : "PASS");
  return g_failed ? 1 : 0;
}
