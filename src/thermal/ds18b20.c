#include "ds18b20.h"

#include <tools/utils.h>

#include <string.h>

#include <stm32f0xx.h>

#define DS_LISTEN_BAUD 9600
#define DS_SEND_RECV_BAUD 115200

typedef struct {
  temperature_reader_t temperature_reader;
} DsState;

static DsState g_ds_state = {NULL};

//static void DspUpdateBaud(USART_TypeDef* transceiver, uint32_t baud) {
//  CLEAR_BIT(transceiver->CR1, USART_CR1_UE);
//  transceiver->BRR = (uint16_t)(SystemCoreClock / baud);
//  SET_BIT(transceiver->CR1, USART_CR1_UE);
//}

//static void DspSetupTransceiver(USART_TypeDef* transceiver) {
//  SET_BIT(transceiver->CR3, USART_CR3_HDSEL);
//  SET_BIT(transceiver->CR1, USART_CR1_RE | USART_CR1_TE);
//}
//
//static void DspSend(USART_TypeDef* transceiver, uint8_t value) {}

//static bool DspDetectDevice(USART_TypeDef* transceiver) {
//  DspUpdateBaud(transceiver, DS_LISTEN_BAUD);
//  DspSend(transceiver, 0xF0);
//  DspUpdateBaud(transceiver, DS_SEND_RECV_BAUD);
//  return true;
//}

static void DspInitiateTemperatureRequest(void) {}

TpStatus DsInitialize(void) {
  return TpSuccess;
}

TpStatus DsStartMeasurement(void) {
  return TpSuccess;
}

TpStatus DsReadTemperatureAsync(temperature_reader_t reader) {
  if (g_ds_state.temperature_reader) {
    return TpAlreadyRunning;
  }
  g_ds_state.temperature_reader = reader;
  DspInitiateTemperatureRequest();
  return TpSuccess;
}
