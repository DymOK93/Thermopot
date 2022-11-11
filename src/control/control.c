#include "control.h"

TpStatus CtrlInitialize(void) {
  return TpSuccess;
}

TpStatus CtrlProcessRequests(void) {
  /**
   * 1. Update SSI value via TmGetCurrentTemperature() query
   * 2. After closing the setup transaction, call TmSetTemperaturePoint()
   * 3. Process Bluetooth/USART data
   */
  return TpSuccess;
}



