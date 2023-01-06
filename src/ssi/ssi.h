#pragma once
#include <tools/fixed_point.h>
#include <tools/status.h>

#include <stdbool.h>

#define SSI_PANEL_SIZE 4
#define SSI_NUMBER_MIN (-99)  // -99.0
#define SSI_NUMBER_MAX 999    // 999.0

typedef struct {
  char str[SSI_PANEL_SIZE];
} SsiValue;

/**
 * @brief Initializes the seven-segment indicator (SSI) panel
 * @return TpSuccess
 */
TpStatus SsiInitialize(void);

/**
 * @brief Starts or stops the SSI controller
 * @param[in] enable Starts if true, false otherwise
 * @remark Thermal manager stopping automatically turns off the heater
 */
void SsiSetState(bool enable);

/**
 * @brief Requests the current value of the SSI panel
 * @remark Raw value string
 */
SsiValue SsiGetValue(void);

/**
 * @brief Clears the SSI panel
 * @remark TpSuccess
 */
TpStatus SsiClearValue(void);

/**
 * @brief Sets the value of the SSI panel
 * @param[in] value Raw value string (only 'a'-'z', 'A'-'Z', '0'-'9', '-', ' '
 * and '\0' are supported)
 * @return
 *    - TpInvalidParameter if value contains invalid characters
 *    - TpSuccess otherwise
 */
TpStatus SsiSetValue(SsiValue value);

/**
 * @brief Sets a fixed point number as the value of the SSI panel
 * @param[in] number Fixed-point number in range [SSI_NUMBER_MIN; SSI_NUMBER_MAX]
 * @return
 *    - TpInvalidParameter if number is out of range
 *    - TpSuccess otherwise
 */
TpStatus SsiSetNumber(FixedPoint16 number);
