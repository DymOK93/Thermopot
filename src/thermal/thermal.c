#include "thermal.h"

typedef struct {
  uint16_t current_temperature;
  uint16_t temperature_point;
  TmMode mode;
} TmState;

static TmState g_tm_state = {0};

static void TmpStart(void) {}
static void TmpStop(void) {}

static bool TmpIsRunning(void) {
  return true;
}

TpStatus TmInitialize(void) {
  return TpSuccess;
}

void TmSetState(bool enable) {
  if (!enable) {
    TmpStop();
  } else {
    TmpStart();
  }
}

uint16_t TmGetCurrentTemperature(void) {
  return g_tm_state.current_temperature;
}


TpStatus TmSetMode(TmMode mode) {
  if (TmpIsRunning()) {
    return TpAlreadyRunning;
  }
  g_tm_state.mode = mode;
  return TpSuccess;
}

TpStatus TmSetTemperaturePoint(uint16_t value) {
  if (value > TM_TEMPERATURE_MAX) {
    return TpOverflow;
  }
  g_tm_state.temperature_point = value;
  return TpSuccess;
}

