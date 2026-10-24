// The Mini Dash UI (CLAUDE.md section 4): splash, header with the status cluster, five tabs,
// Settings by header tap, swipe between tabs. Built once; a 100 ms LVGL timer takes a
// VehicleState snapshot and updates the visible page. Nothing here touches I2C or sensors
// directly; everything comes through VehicleState and a few callbacks into the app layer.
#pragma once

#include <stdint.h>

#include "lvgl.h"
#include "../app/storage.h"

namespace Ui {

enum Page : uint8_t { PAGE_DRIVE = 0, PAGE_TRIP, PAGE_DYNAMICS, PAGE_ENERGY, PAGE_BATTERY, PAGE_COUNT, PAGE_SETTINGS };

// The settings the UI owns; the app persists them when settingsChanged fires.
struct UiSettings {
  bool mph = false;
  int8_t nightMode = -1;
  bool demo = true;
  uint8_t bestField = 0;
};

struct Callbacks {
  void (*setDemo)(bool on) = nullptr;
  void (*setNightMode)(int mode) = nullptr;  // -1 auto, 0 day, 1 night
  void (*calibrateMounting)() = nullptr;
  void (*calibrateAccel)() = nullptr;  // sphere-fit offsets, the user turns the device
  void (*resetTrip)() = nullptr;
  void (*resetPeaks)() = nullptr;
  void (*clearRecords)() = nullptr;
  void (*settingsChanged)(const UiSettings &s) = nullptr;
  // Load the trace captured for a record (by the trip it belongs to). Returns false if none.
  bool (*loadTrace)(uint32_t tripId, int rec, Storage::TraceSample *out, size_t max, size_t *n) = nullptr;
};

// Build the splash immediately (call right after LvglPort::begin, under the LVGL lock).
void begin(const Callbacks &cb, const char *version);
// Apply stored settings before the shell is shown.
void applySettings(const UiSettings &s);

// Leave the splash for the shell. The tick timer calls this on its own once the sensors are
// ready and at least 1.5 s have passed, or after 3 s regardless.
void showShell();

void setPage(Page p);
Page page();
// Open a record's event page on Trip (diagnostics key, and later deep links).
void openEvent(int rec);

// Longest UI tick and plot redraw since the last call, ms.
void timing(uint32_t *tickMaxMs, uint32_t *plotMaxMs);

// Boot status lines shown on the splash.
void setBootStatus(const char *gps, const char *obd, const char *imu);

}  // namespace Ui
