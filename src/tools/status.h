#pragma once

typedef enum {
  TpSuccess,
  TpUnderflow = -1,
  TpOverflow = -2,
  TpUnsupportedSymbol = -3,
  TpHardwareFault = -4,
  TpAlreadyRunning = -5
} TpStatus;

#define TP_ERROR(status) ((status) < 0)
#define TP_SUCCESS(status) (!TP_ERROR(status))
