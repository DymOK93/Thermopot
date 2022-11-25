#pragma once

typedef enum {
  TpSuccess,
  TpHardwareFault = -1,
  TpAlreadyRunning = -2,
  TpInvalidParameter = -3
} TpStatus;

#define TP_ERROR(status) ((status) < 0)
#define TP_SUCCESS(status) (!TP_ERROR(status))
