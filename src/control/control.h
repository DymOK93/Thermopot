#pragma once
#include <tools/status.h>

/**
 * @brief Initializes the control panel and setup interface
 * @return TpSuccess
 */
TpStatus CtrlInitialize(void);

/**
 * @brief Updates image on the SSI panel and processes requests from external
 * devices (Bluetooth module, etc.)
 * @return TpSuccess
 */
TpStatus CtrlProcessRequests(void);
