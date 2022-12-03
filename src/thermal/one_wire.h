#pragma once
#include <tools/status.h>

#include <stdint.h>

#include <stm32f0xx.h>

#define OW_IO_GRANULARITY 8

#define OW_SKIP_ROM 0xCC
#define OW_CONVERT_TEMPERATURE 0x44

void OwBytesToRaw(const uint8_t* bytes,
                  uint32_t bytes_count,
                  unsigned char* raw);

void OwRawToBytes(const unsigned char* raw,
                  uint32_t bytes_count,
                  uint8_t* bytes);

TpStatus OwBroadcastCommand(USART_TypeDef* transceiver, uint8_t command);

TpStatus OwBroadcastCommandEx(USART_TypeDef* transceiver,
                              uint8_t command,
                              const unsigned char* payload,
                              uint32_t bytes_count);
