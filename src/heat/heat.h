/**
 * @file
 * @brief Heater Manager interface
 *
 * The Heater Manager abstracts the power control of the heater element,
 * allowing a generic "power factor"  to be set in percentages in the
 * range [HM_POWER_FACTOR_MIN, HM_POWER_FACTOR_MAX]
 */
#pragma once
#include <tools/status.h>

#include <stdint.h>

#define HM_VOLTAGE_FREQUENCY 50  //!< Mains voltage frequency in Hz
#define HM_POWER_FACTOR_MIN 0    //!< Minimum allowed power factor in %
#define HM_POWER_FACTOR_MAX 100  //!< Maximum allowed power factor in %

/**
 * @brief Initializes the Heater Manager
 * @return TpSuccess
 */
TpStatus HmInitialize(void);

/**
 * @brief Retrieves the heater power factor
 * @return Power factor in range [HM_POWER_FACTOR_MIN, HM_POWER_FACTOR_MAX]
 */
uint8_t HmGetPowerFactor(void);

/**
 * @brief Sets the heater power factor
 * @param[in] power_factor Power factor in range [HM_POWER_FACTOR_MIN,
 * HM_POWER_FACTOR_MAX]
 * @remark If power_factor == HM_POWER_FACTOR_MIN, turns off the heating
 * @return
 *    - TpInvalidParameter if power_factor > HM_POWER_FACTOR_MAX
 *    - TpSuccess otherwise
 */
TpStatus HmSetPowerFactor(uint8_t power_factor);
