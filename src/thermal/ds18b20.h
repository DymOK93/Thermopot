#pragma once
#include <tools/fixed_point.h>
#include <tools/status.h>

TpStatus DsInitialize(void);

TpStatus DsPrepare(void);
TpStatus DsConvertTemperature(void);
TpStatus DsReadTemperature(FixedPoint16* temperature);
