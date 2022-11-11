#pragma once
#include <tools/status.h>

#include <stdbool.h>
#include <stdint.h>

#define SSI_PANEL_SIZE 4
#define SSI_NUMBER_MIN (-999)  // -99.9
#define SSI_NUMBER_MAX (9999)  // 9999

typedef struct {
  char str[SSI_PANEL_SIZE];
} SsiValue;

TpStatus SsiInitialize(void);
void SsiSetState(bool enable);

SsiValue SsiGetValue(void);
TpStatus SsiSetValue(SsiValue value);
TpStatus SsiSetNumber(int16_t number);
