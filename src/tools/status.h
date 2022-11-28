#pragma once

typedef enum {
  TpSuccess,
  TpAlreadyRunning = -1,
  TpInvalidParameter = -2,
  TpDeviceNotConnected = -3,
  TpNotReady = -4
} TpStatus;

#define TP_ERROR(status) ((status) < 0)
#define TP_SUCCESS(status) (!TP_ERROR(status))
#define RET_IF_ERROR(expr)                    \
  {                                           \
    const TpStatus status_from_expr = (expr); \
    if (!TP_SUCCESS(status_from_expr))        \
      return status_from_expr;                \
  }
