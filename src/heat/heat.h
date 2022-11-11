#pragma once
#include <tools/status.h>

#include <stdint.h>

#define HM_POWER_FACTOR_MIN 0
#define HM_POWER_FACTOR_MAX 100

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
 * @remark If power_factor == HM_POWER_FACTOR_MIN, turns off the heating,
 * otherwise - turns on
 * @return TpSuccess on success of TpOverflow if power_factor >
 * HM_POWER_FACTOR_MAX
 */
TpStatus HmSetPowerFactor(uint8_t power_factor);
