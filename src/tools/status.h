/**
 * @file
 * @brief Status/error codes and user-friendly macro for its checking
 *
 * Like NTSTATUS, but TpStatus
 */
#pragma once

/**
 * @enum TpStatus
 * @brief Status and error codes
 */
typedef enum {
  TpSuccess = 0,
  TpPending = 1,
  TpAlreadyRunning = -1,
  TpInvalidParameter = -2,
  TpDeviceNotConnected = -3,
  TpNotReady = -4,
  TpNotSupported = -5,
  TpCrcError = -6,
  TpCancelled = -7
} TpStatus;

#define TP_ERROR(status) ((status) < 0)
#define TP_SUCCESS(status) (!TP_ERROR(status))
