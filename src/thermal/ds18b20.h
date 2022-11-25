#pragma once
#include <tools/status.h>

#include <stdint.h>

#include <stm32f0xx.h>

typedef struct {
  USART_TypeDef* transceiver;
} Ds18b20;

TpStatus DsInitialize(Ds18b20* sensor);
TpStatus DsStartMeasurement(Ds18b20* sensor);
uint16_t DsReadTemperature(Ds18b20* sensor);
