#include "ds18b20.h"

#include <tools/break_on.h>
#include <tools/utils.h>

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <stm32f0xx.h>

#define PACKED __attribute__((packed))

#define DS_OW_TRANSFER_GRANULARITY 8
#define DS_OW_RESET_BAUD 9600
#define DS_OW_TRANSFER_BAUD 115200
#define DS_OW_RESET_TIMEOUT_SHIFT 12
#define DS_OW_TRANSFER_TIMEOUT_SHIFT 10
#define DS_OW_ZERO 0x00
#define DS_OW_RESET 0xF0
#define DS_OW_BIT 0xFF

#define DS_TH_MAX 125
#define DS_TL_MIN (-55)
#define DS_ACCURACY_MASK 0x20  // 10-bit accuracy

#define DS_SKIP_ROM 0xCC
#define DS_CONVERT_TEMPERATURE 0x44
#define DS_WRITE_SCRATCHPAD 0x4E
#define DS_READ_SCRATCHPAD 0xBE

#define DS_TEMPERATURE_SHIFT 3

typedef struct PACKED {
  int16_t temperature;
  uint8_t th;
  uint8_t tl;
  uint8_t configuration;
  unsigned char reserved[3];
  uint8_t crc;
} DsScratchpad;

typedef struct {
  int8_t th;
  int8_t tl;
  unsigned char accuracy;
} DsConfig;

static const DsConfig g_ds_config = {.th = DS_TH_MAX,
                                     .tl = DS_TL_MIN,
                                     .accuracy = DS_ACCURACY_MASK | 0x1F};

static void DspPrepareGpio(void) {
  /**
   * 1. Activate PA2 in the alternative function mode
   * 2. Setup PA2 as open-drain; external 4.7 kOhm pull-up is required
   * 3. Select AF1 (USARTx_TX) for PA2 and AF3
   */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
  SET_BIT(GPIOA->MODER, GPIO_MODER_MODER2_1);  // (1)
  SET_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_2);    // (2)
  SET_BIT(GPIOA->AFR[0], 0x00000100);          // (3)
}

static void DspSetupTransceiver(void) {
  /**
   * Setup USART2 in half-duplex mode
   */
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
  SET_BIT(USART2->CR3, USART_CR3_HDSEL);
  SET_BIT(USART2->CR1, USART_CR1_RE | USART_CR1_TE | USART_CR1_UE);
}

static void DspUpdateTransceiverBaud(uint32_t baud) {
  CLEAR_BIT(USART2->CR1, USART_CR1_UE);
  USART2->BRR = (uint16_t)(SystemCoreClock / baud);
  SET_BIT(USART2->CR1, USART_CR1_UE);
}

static uint8_t DspPumpRawSequence(uint8_t value, uint32_t timeout) {
  while (!READ_BIT(USART2->ISR, USART_ISR_TXE))
    ;
  *(volatile uint8_t*)&USART2->TDR = value;

  while (timeout-- && !READ_BIT(USART2->ISR, USART_ISR_RXNE | USART_ISR_FE))
    ;
  return *(const volatile uint8_t*)&USART2->RDR;
}

static void DspSend(const unsigned char* buffer, uint32_t bytes_count) {
  const uint32_t timeout = SystemCoreClock >> DS_OW_TRANSFER_TIMEOUT_SHIFT;

  while (bytes_count--) {
    uint8_t value = *buffer++;
    uint8_t idx = 0;
    for (; idx < DS_OW_TRANSFER_GRANULARITY; ++idx) {
      DspPumpRawSequence(value & 1 ? DS_OW_BIT : DS_OW_ZERO, timeout);
      value >>= 1;
    }
  }
}

static void DspRecv(unsigned char* buffer, uint32_t bytes_count) {
  const uint32_t timeout = SystemCoreClock >> DS_OW_TRANSFER_TIMEOUT_SHIFT;

  while (bytes_count--) {
    uint8_t value = 0;
    uint8_t idx = 0;
    for (; idx < DS_OW_TRANSFER_GRANULARITY; ++idx) {
      value >>= 1;
      const uint8_t echo = DspPumpRawSequence(DS_OW_BIT, timeout);
      if (echo == DS_OW_BIT) {
        value |= 0x80;
      }
    }
    *buffer++ = value;
  }
}

static TpStatus DspReset() {
  DspUpdateTransceiverBaud(DS_OW_RESET_BAUD);

  const uint8_t echo = DspPumpRawSequence(
      DS_OW_RESET, SystemCoreClock >> DS_OW_RESET_TIMEOUT_SHIFT);

  DspUpdateTransceiverBaud(DS_OW_TRANSFER_BAUD);

  return echo != DS_OW_RESET ? TpSuccess : TpDeviceNotConnected;
}

static TpStatus DspSendCommandWithPayload(uint8_t command,
                                          const unsigned char* payload,
                                          uint32_t bytes_count) {
  const TpStatus status = DspReset();
  if (TP_SUCCESS(status)) {
    const unsigned char broadcast[] = {DS_SKIP_ROM, command};
    DspSend(broadcast, sizeof broadcast);
    DspSend(payload, bytes_count);
  }
  return status;
}

static TpStatus DspSendCommand(uint8_t command) {
  return DspSendCommandWithPayload(command, NULL, 0);
}

static bool DspVerifyCrc(const DsScratchpad* scratchpad) {
  uint8_t bytes_count = sizeof(DsScratchpad);
  const unsigned char* raw_data = (const unsigned char*)scratchpad;

  uint8_t crc = 0;
  while (bytes_count--) {
    unsigned char value = *raw_data++;
    uint8_t idx = 0;
    for (; idx < CHAR_BIT; ++idx) {
      if ((value ^ crc) & 0x01) {
        crc = (crc ^ 0x18) >> 1 | 0x80;
      } else {
        crc >>= 1;
      }
      value >>= 1;
    }
  }
  return crc == 0;
}

TpStatus DspReadTemperature(FixedPoint16* temperature) {
  DsScratchpad scratchpad;
  DspRecv((unsigned char*)&scratchpad, sizeof(DsScratchpad));

  if (!DspVerifyCrc(&scratchpad)) {
    return TpCrcError;
  }

  Fp16WriteAsNumber(*temperature, scratchpad.temperature >> DS_TEMPERATURE_SHIFT);

  return TpSuccess;
}

TpStatus DsInitialize(void) {
  DspPrepareGpio();
  DspSetupTransceiver();
  return TpSuccess;
}

TpStatus DsPrepare(void) {
  return DspSendCommandWithPayload(DS_WRITE_SCRATCHPAD,
                                   (const unsigned char*)&g_ds_config,
                                   sizeof(DsConfig));
}

TpStatus DsConvertTemperature(void) {
  return DspSendCommand(DS_CONVERT_TEMPERATURE);
}

TpStatus DsReadTemperature(FixedPoint16* temperature) {
  if (!temperature) {
    return TpInvalidParameter;
  }
  const TpStatus status = DspSendCommand(DS_READ_SCRATCHPAD);
  if (!TP_SUCCESS(status)) {
    return status;
  }
  return DspReadTemperature(temperature);
}
