#pragma once
#include <tools/fixed_point.h>
#include <tools/status.h>

#include <stdbool.h>

#define SSI_PANEL_SIZE 4
#define SSI_NUMBER_MIN (-99)  // -99.0
#define SSI_NUMBER_MAX 999    // 999.0

typedef struct {
  char str[SSI_PANEL_SIZE];
} SsiValue;

TpStatus SsiInitialize(void);
void SsiSetState(bool enable);

SsiValue SsiGetValue(void);
TpStatus SsiClearValue(void);
TpStatus SsiSetValue(SsiValue value);
TpStatus SsiSetNumber(FixedPoint16 number);
