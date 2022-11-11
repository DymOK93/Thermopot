#pragma once
#include <tools/status.h>

#include <stdbool.h>
#include <stdint.h>

#define TM_TEMPERATURE_MIN 0     // 0.0
#define TM_TEMPERATURE_MAX 1000  // 100.0
#define TM_TEMPERATURE_STEP 5    // 0.5

typedef enum { TmModeRelay, TmModePid, TmModeMax } TmMode;

TpStatus TmInitialize(void);
void TmSetState(bool enable);

uint16_t TmGetCurrentTemperature(void);
TpStatus TmSetMode(TmMode mode);
TpStatus TmSetTemperaturePoint(uint16_t value);
