#pragma once
#include <tools/fixed_point.h>
#include <tools/status.h>

#include <stdbool.h>

typedef bool (*temperature_reader_t)(TpStatus, FixedPoint16);

TpStatus DsInitialize(void);
TpStatus DsStartMeasurement(void);
TpStatus DsReadTemperatureAsync(temperature_reader_t reader);
