#pragma once
#include <tools/status.h>

#include <stdbool.h>
#include <stdint.h>

#define TM_TEMPERATURE_MIN 0     // 0.0
#define TM_TEMPERATURE_MAX 1000  // 100.0
#define TM_TEMPERATURE_STEP 5    // 0.5

#define TM_TIMER_INTERRUPT_PRIORITY 2
#define TM_USART_INTERRUPT_PRIORITY 2

/**
 * @enum TmMode
 * @brief Heater power control mode
 */
typedef enum { TmModeRelay, TmModePid, TmModeMax } TmMode;

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
TpStatus TmQueryTemperature(uint16_t* temperature);

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
TpStatus TmSetTemperaturePoint(uint16_t value);
