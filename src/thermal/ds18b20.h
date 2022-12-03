#pragma once
#include <tools/fixed_point.h>
#include <tools/status.h>

#include <stdint.h>

TpStatus DsInitialize(void);

TpStatus DsStartMeasurement(void);
TpStatus DsQueryTemperature(FixedPoint16* temperature);
