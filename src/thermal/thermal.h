/**
 * @file
 * @brief Thermal Manager interface
 *
 * The thermal manager encapsulates the algorithms for changing and maintaining
 * the temperature (using Relay or Proportional-Integral-Derivative a.k.a. PID
 * controller), mediates between the Control and the Heater Manager
 */
#pragma once
#include <tools/fixed_point.h>
#include <tools/status.h>

#include <stdbool.h>

// clang-format off
extern const FixedPoint16 TmTemperatureMin;  //!< Minimum supported temperature point
extern const FixedPoint16 TmTemperatureMax;  //!< Maximum supported temperature point
// clang-format on

/**
 * @enum TmMode
 * @brief Temperature control mode
 */
typedef enum { TmModeRelay, TmModePid, TmModeCount } TmMode;

/**
 * @brief Initializes the Thermal Manager
 * @return TpSuccess
 */
TpStatus TmInitialize(void);

/**
 * @brief Starts or stops the Thermal Manager
 * @param[in] enable Starts if true, false otherwise
 * @remark Thermal manager stopping automatically turns off the heater
 */
void TmSetState(bool enable);

/**
 * @brief Requests the current temperature
 * @param[out] temperature Pointer to a variable to store the temperature
 * @param[in] wait Wait for temperature measurement
 * @return
 *    - TpNotReady if Thermal Manager is stopped
 *    - TpDeviceNotConnected if the temperature sensor is not connected
 *    - TpInvalidParameter if temperature is NULL
 *    - TpSuccess otherwise
 */
TpStatus TmQueryTemperature(FixedPoint16* temperature, bool wait);

/**
 * @brief Sets temperature control mode and temperature point
 * @warning The Thermal Manager must be manually stopped first
 * @param[in] mode Temperature control mode
 * @param[in] temperature_point Temperature point in fixed point format
 * @return
 *    - TpAlreadyRunning if Thermal Manager isn't stopped
 *    - TpInvalidParameter if not (TmTemperatureMin <= temperature_point <=
 * TmTemperatureMax)
 *    - TpSuccess otherwise
 */
TpStatus TmSetup(TmMode mode, FixedPoint16 temperature_point);
