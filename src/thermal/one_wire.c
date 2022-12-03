#include "one_wire.h"

#include <tools/utils.h>

#include <limits.h>
#include <stdbool.h>

#define OW_POLLING_SHIFT 10
#define OW_RESET_BAUD 9600
#define OW_IO_BAUD 115200
#define OW_RESET 0xF0
#define OW_BIT 0xFF

#define OwConvertRaw(frame) ((frame) == OW_BIT ? 1 : 0)
#define OwConvertBit(bit) ((bit) ? OW_BIT : 0)

static void OwpUpdateTransceiverBaud(USART_TypeDef* transceiver,
                                     uint32_t baud) {
  CLEAR_BIT(transceiver->CR1, USART_CR1_UE);
  transceiver->BRR = (uint16_t)(SystemCoreClock / baud);
  SET_BIT(transceiver->CR1, USART_CR1_UE);
}

static void OwpSendRaw(USART_TypeDef* transceiver, uint8_t value) {
  *(volatile uint8_t*)&transceiver->TDR = value;
  while (!READ_BIT(transceiver->ISR, USART_ISR_TC))
    ;
}

static uint8_t OwpRecvRaw(USART_TypeDef* transceiver) {
  volatile uint32_t timeout = SystemCoreClock >> OW_POLLING_SHIFT;
  while (!READ_BIT(transceiver->ISR, USART_ISR_RXNE | USART_ISR_FE) &&
         timeout--)
    ;
  return *(const volatile uint8_t*)&transceiver->RDR;
}

void OwpSend(USART_TypeDef* transceiver, uint8_t value) {
  uint8_t idx = 0;
  for (; idx < OW_IO_GRANULARITY; ++idx) {
    OwpSendRaw(transceiver, OwConvertBit(value & 1 << idx));
  }
}

uint8_t OwpRecv(USART_TypeDef* transceiver) {
  uint8_t value = 0;
  uint8_t idx = 0;
  for (; idx < OW_IO_GRANULARITY; ++idx) {
    const uint8_t bit = OwConvertBit(OwpRecvRaw(transceiver));
    value |= bit << idx;
  }
  return value;
}

void OwBytesToRaw(const uint8_t* bytes,
                  uint32_t bytes_count,
                  unsigned char* raw) {
  while (bytes_count--) {
    const uint8_t value = *bytes++;
    uint8_t idx = 0;
    for (; idx < OW_IO_GRANULARITY; ++idx) {
      const uint8_t bit = value & 1 << idx;
      *raw++ = !bit ? 0 : OW_BIT;
    }
  }
}

void OwRawToBytes(const unsigned char* raw,
                  uint32_t bytes_count,
                  uint8_t* bytes) {
  while (bytes_count--) {
    uint8_t value = 0, idx = 0;
    for (; idx < OW_IO_GRANULARITY; ++idx) {
      value |= OwConvertRaw(*raw++) << idx;
    }
    *bytes++ = value;
  }
}

TpStatus OwpReset(USART_TypeDef* transceiver) {
  OwpUpdateTransceiverBaud(transceiver, OW_RESET_BAUD);

  OwpSendRaw(transceiver, OW_RESET);  // Send RESET
  const uint8_t echo = OwpRecvRaw(transceiver);

  OwpUpdateTransceiverBaud(transceiver, OW_IO_BAUD);

  return echo != OW_RESET ? TpSuccess : TpDeviceNotConnected;
}

TpStatus OwBroadcastCommand(USART_TypeDef* transceiver, uint8_t command) {
  return OwBroadcastCommandEx(transceiver, command, NULL, 0);
}

TpStatus OwBroadcastCommandEx(USART_TypeDef* transceiver,
                              uint8_t command,
                              const unsigned char* payload,
                              uint32_t bytes_count) {
  const TpStatus status = OwpReset(transceiver);
  if (TP_SUCCESS(status)) {
    OwpSend(transceiver, OW_SKIP_ROM);
    OwpSend(transceiver, command);
    while (bytes_count--) {
      OwpSend(transceiver, *payload++);
    }
  }
  return status;
}
