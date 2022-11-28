#pragma once
#include <tools/fixed_point.h>
#include <tools/status.h>

#include <stdbool.h>

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
 * @param[in] enable New state
 * @return
 *    - TpDeviceNotConnected if the temperature sensor is not connected
 *    - TpSuccess otherwise
 */
TpStatus TmSetState(bool enable);

/**
 * @brief Requests the current temperature
 * @param[out] temperature Pointer to a variable to store the temperature
 * @return
 *    - TpNotReady if thermal manager is stopped
 *    - TpDeviceNotConnected if the temperature sensor is not connected
 *    - TpInvalidParameter if temperature is NULL
 *    - TpSuccess otherwise
 */
TpStatus TmQueryTemperature(FixedPoint16* temperature);

/**
 * @brief Sets power control mode
 * @warning The thermal manager must be manually stopped first
 * @param[in] mode New mode
 * @return
 *    - TpAlreadyRunning if thermal manager isn't stopped
 *    - TpInvalidParameter if mode is invalid
 *    - TpSuccess otherwise
 */
TpStatus TmSetMode(TmMode mode);

/**
 * @brief Sets the temperature point
 * @param[in] value Temperature in fixed-point format (xx.x * 10)
 * @return
 *    - TpInvalidParameter if value > TM_TEMPERATURE_MAX
 *    - TpSuccess otherwise
 */
TpStatus TmSetTemperaturePoint(FixedPoint16 value);
