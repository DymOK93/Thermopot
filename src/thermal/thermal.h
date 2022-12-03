#pragma once
#include <tools/fixed_point.h>
#include <tools/status.h>

#include <stdbool.h>

extern const FixedPoint16 TmTemperatureHysteresis;
extern const FixedPoint16 TmTemperatureMin;
extern const FixedPoint16 TmTemperatureMax;

/**
 * @enum TmMode
 * @brief Heater power control mode
 */
typedef enum { TmModeRelay, TmModePid, TmModeCount } TmMode;

/**
 * @brief Initializes the thermal manager
 * @return TpSuccess
 */
TpStatus TmInitialize(void);

/**
 * @brief Starts or stops the thermal manager
 * @remark Thermal manager stopping automatically turns off the heater
 */
void TmSetState(bool enable);

/**
 * @brief Requests the current temperature
 * @param[out] temperature Pointer to a variable to store the temperature
 * @param[in] wait Wait for temperature measurement
 * @return
 *    - TpNotReady if thermal manager is stopped
 *    - TpDeviceNotConnected if the temperature sensor is not connected
 *    - TpInvalidParameter if temperature is NULL
 *    - TpSuccess otherwise
 */
TpStatus TmQueryTemperature(FixedPoint16* temperature, bool wait);

/**
 * @brief Sets power control mode and temperature point
 * @warning The thermal manager must be manually stopped first
 * @param[in] mode New mode
 * @param[in] temperature_point Temperature in fixed-point format
 * @return
 *    - TpAlreadyRunning if thermal manager isn't stopped
 *    - TpInvalidParameter if value > TM_TEMPERATURE_MAX
 *    - TpSuccess otherwise
 */
TpStatus TmSetup(TmMode mode, FixedPoint16 temperature_point);
