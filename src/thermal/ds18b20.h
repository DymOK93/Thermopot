#pragma once
#include <tools/fixed_point.h>
#include <tools/status.h>

#include <stdint.h>

typedef enum {
  DsResolution9Bit = 0x0,
  DsResolution10Bit = 0x1,
  DsResolution11Bit = 0x2,
  DsResolution12Bit = 0x3
} DsResolution;

/**
 * @brief Initializes the temperature sensor interface
 * @return TpSuccess
 */
TpStatus DsInitialize(void);

/**
 * @brief Sets the measurement resolution
 * @param[in] resolution Sensor resolution
 * @return
 *    - TpDeviceNotConnected if the temperature sensor is not connected
 *    - TpSuccess otherwise
 */
TpStatus DsPrepare(DsResolution resolution);

/**
 * @brief Starts temperature conversion
 * @return
 *    - TpDeviceNotConnected if the temperature sensor is not connected
 *    - TpSuccess otherwise
 */
TpStatus DsConvertTemperature(void);

/**
 * @brief Reads the latest converted temperature value
 * @param[out] temperature Pointer to a variable to store the temperature
 * @return
 *    - TpDeviceNotConnected if the temperature sensor is not connected
 *    - TpInvalidParameter if temperature is NULL
 *    - TpSuccess otherwise
 */
TpStatus DsReadTemperature(FixedPoint16* temperature);
