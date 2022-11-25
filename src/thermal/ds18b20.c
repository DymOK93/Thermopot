#include "ds18b20.h"

#include <tools/utils.h>

#include <stdbool.h>

#define DS_LISTEN_BAUD 9600
#define DS_SEND_RECV_BAUD 115200

void DspUpdateBaudrate(USART_TypeDef* transceiver, uint32_t baud) {
  CLEAR_BIT(transceiver->CR1, USART_CR1_UE);
  transceiver->BRR = (uint16_t)(SystemCoreClock / baud);
  SET_BIT(transceiver->CR1, USART_CR1_UE);
}

void DspSetupTransceiver(USART_TypeDef* transceiver) {
  SET_BIT(transceiver->CR3, USART_CR3_HDSEL);
  SET_BIT(transceiver->CR1, USART_CR1_RE | USART_CR1_TE);
}

void DspSend(USART_TypeDef* transceiver, uint8_t value) {}
uint8_t DspEchoRead(USART_TypeDef* transceiver) {}

bool DspDetectDevice(USART_TypeDef* transceiver) {
  DspUpdateBaudrate(transceiver, DS_LISTEN_BAUD);
  DspSend(transceiver, 0xF0);
  DspUpdateBaudrate(transceiver, DS_SEND_RECV_BAUD);
  return true;
}

TpStatus DsInitialize(Ds18b20* sensor) {
  if (!sensor || !sensor->transceiver) {
    return TpInvalidParameter;
  }
  DspSetupTransceiver(sensor->transceiver);
  if (!DspDetectDevice(sensor->transceiver)) {
    return TpDeviceNotConnected;
  }
  return TpSuccess;
}

TpStatus DsStartMeasurement(Ds18b20* sensor) {
  (void)sensor;
  return TpSuccess;
}

uint16_t DsReadTemperature(Ds18b20* sensor) {
  (void)sensor;
  return 0;
}
