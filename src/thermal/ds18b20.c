/**
 * @file
 * @brief DS18B20 digital thermal sensor implementation
 */
#include "ds18b20.h"

#include <tools/break_on.h>
#include <tools/utils.h>

#include <limits.h>
#include <stdint.h>
#include <stdlib.h>

#include <stm32f0xx.h>

#define PACKED __attribute__((packed))

/**
 * 1-Wire transceiver implemented on hardware USART
 * @see https://www.rotr.info/electronics/interface/one_wire/ow_over_uart.htm
 */
// clang-format off
#define DS_OW_TRANSFER_GRANULARITY 8     //!< Number of 1-Wire transactions to transfer single byte of data
#define DS_OW_RESET_BAUD_RATE 9600       //!< USART baud rate for 1-Wire reset 
#define DS_OW_TRANSFER_BAUD_RATE 115200  //!< USART baud rate for 1-Wire transfer
#define DS_OW_RESET_TIMEOUT_SHIFT 12     //!< 1-Wire reset timeout is SystemCoreClock >> DS_OW_RESET_TIMEOUT_SHIFT
#define DS_OW_TRANSFER_TIMEOUT_SHIFT 10  //!< 1-Wire transfer timeout is SystemCoreClock >> DS_OW_TRANSFER_TIMEOUT_SHIFT
#define DS_OW_RESET 0xF0                 //!< 1-Wire reset sequence
#define DS_OW_ZERO 0x00                  //!< 1-Wire '0' bit
#define DS_OW_BIT 0xFF                   //!< 1-Wire '1' bit

#define DS_TH_MAX 125    //!< Maximum supported temperature
#define DS_TL_MIN (-55)  //!< Minimum supported temperature

#define DS_RESOLUTION_MASK DsResolution12Bit  //!< Valid resolution mask
#define DS_RESOLUTION_SHIFT 4                 //!< Shift resolution to the left in the configuration byte
#define DS_FP16_TEMPERATURE_SHIFT 3           //!< Shift resolution to the right to convert into FixedPoint16
// clang-format on

#define DS_VALID_CRC 0               //!< Valid CRC calculation result
#define DS_SKIP_ROM 0xCC             //!< 'Skip Rom' command
#define DS_CONVERT_TEMPERATURE 0x44  //!< 'Convert T' command
#define DS_WRITE_SCRATCHPAD 0x4E     //!< 'Write Scratchpad' command
#define DS_READ_SCRATCHPAD 0xBE      //!< 'Read Scratchpad' command

/**
 * @struct DsScratchpad
 * @brief DS18B20 internal state (scratchpad)
 * @see
 * https://datasheet.lcsc.com/szlcsc/1912111437_UMW-Youtai-Semiconductor-Co-Ltd-DS18B20_C376006.pdf
 */
typedef struct PACKED {
  int16_t temperature;        //!< Last converted temperature
  uint8_t th;                 //!< High alarm trigger
  uint8_t tl;                 //!< Low alarm trigger
  uint8_t configuration;      //!< Configuration
  unsigned char reserved[3];  //!< Reserved
  uint8_t crc;                //!< CRC
} DsScratchpad;

/**
 * @struct DsScratchpad
 * @brief DS18B20 configuration update request
 */
typedef struct {
  int8_t th;           //!< High alarm trigger
  int8_t tl;           //!< Low alarm trigger
  uint8_t resolution;  //!< Configuration
} DsConfig;

/**
 * @brief Configures GPIO pins for 1-Wire bus
 */
static void DspPrepareGpio(void) {
  /**
   * 1. Activate PA2 in the alternative function mode
   * 2. Setup PA2 as open-drain; external 4.7 kOhm pull-up is required
   * 3. Select AF1 (USARTx_TX) for PA2
   */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
  SET_BIT(GPIOA->MODER, GPIO_MODER_MODER2_1);  // (1)
  SET_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_2);    // (2)
  SET_BIT(GPIOA->AFR[0], 0x00000100);          // (3)
}

/**
 * @brief Configures the hardware USART to communicate with 1-Wire devices
 */
static void DspSetupTransceiver(void) {
  /**
   * Setup USART2 in half-duplex mode
   */
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
  SET_BIT(USART2->CR3, USART_CR3_HDSEL);
  SET_BIT(USART2->CR1, USART_CR1_RE | USART_CR1_TE | USART_CR1_UE);
}

/**
 * @brief Sets the USART baud rate
 * @param[in] baud Baud rate
 */
static void DspUpdateTransceiverBaudRate(uint32_t baud) {
  CLEAR_BIT(USART2->CR1, USART_CR1_UE);
  USART2->BRR = (uint16_t)(SystemCoreClock / baud);
  SET_BIT(USART2->CR1, USART_CR1_UE);
}

/**
 * @brief Sends raw byte to USART and reads echo
 * @warning USART must be configured in half duplex mode or with shorted RX/TX
 * @param[in] value Raw data
 * @param[in] timeout Timeout in CPU tics
 * @return Echo
 */
static uint8_t DspPumpRawSequence(uint8_t value, uint32_t timeout) {
  while (!READ_BIT(USART2->ISR, USART_ISR_TXE))
    ;
  *(volatile uint8_t*)&USART2->TDR = value;

  while (timeout-- && !READ_BIT(USART2->ISR, USART_ISR_RXNE | USART_ISR_FE))
    ;
  return *(const volatile uint8_t*)&USART2->RDR;
}

/**
 * @brief Sends a sequence of bytes over 1-Wire bus
 * @remark LSB sent first
 * @param[in] buffer Source buffer
 * @param[in] bytes_count Buffer size in bytes
 */
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

/**
 * @brief Receives a sequence of bytes on the 1-Wire bus
 * @remark LSB received first
 * @param[in] buffer Destination buffer
 * @param[in] bytes_count Buffer size in bytes
 */
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

/**
 * @brief Resets all devices on the 1-Wire bus
 * @return
 *    - TpDeviceNotConnected if the temperature sensor is not connected
 *    - TpSuccess otherwise
 */
static TpStatus DspReset() {
  DspUpdateTransceiverBaudRate(DS_OW_RESET_BAUD_RATE);

  const uint8_t echo = DspPumpRawSequence(
      DS_OW_RESET, SystemCoreClock >> DS_OW_RESET_TIMEOUT_SHIFT);

  DspUpdateTransceiverBaudRate(DS_OW_TRANSFER_BAUD_RATE);

  return echo != DS_OW_RESET ? TpSuccess : TpDeviceNotConnected;
}

/**
 * @brief Resets all devices on the 1-Wire bus and broadcast the command with
 * the payload
 * @param[in] command Command
 * @param[in] payload Source buffer
 * @param[in] bytes_count Buffer size in bytes
 * @return
 *    - TpDeviceNotConnected if the temperature sensor is not connected
 *    - TpSuccess otherwise
 */
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

/**
 * @brief Resets all devices on the 1-Wire bus and broadcast the given command
 * @param[in] command Command
 * @return
 *    - TpDeviceNotConnected if the temperature sensor is not connected
 *    - TpSuccess otherwise
 */
static TpStatus DspSendCommand(uint8_t command) {
  return DspSendCommandWithPayload(command, NULL, 0);
}

/**
 * @brief Calculates the CRS of the DS18B20 scratchpad
 * @remark CRC = X^8 + X^5 + X^4 + 1
 * @param[in] scratchpad Scratchpad read using the 'Read Scratchpad' command
 * @return CRC
 */
static uint8_t DspCalcCrc(const DsScratchpad* scratchpad) {
  uint8_t bytes_count = sizeof(DsScratchpad);
  const unsigned char* raw_data = (const unsigned char*)scratchpad;

  uint8_t crc = 0;
  while (bytes_count--) {
    unsigned char value = *raw_data++;
    uint8_t idx = 0;
    for (; idx < CHAR_BIT; ++idx) {
      if ((value ^ crc) & 1) {
        crc = (crc ^ 0x18) >> 1 | 0x80;
      } else {
        crc >>= 1;
      }
      value >>= 1;
    }
  }
  return crc;
}

/**
 * @brief Reads the latest converted temperature value
 * @param[out] temperature Non-NULL pointer to a variable to store the
 * temperature
 * @return
 *    - TpDeviceNotConnected if the temperature sensor is not connected
 *    - TpCrcError if scratchpad CRC is invalid (so request must be repeated)
 *    - TpSuccess otherwise
 */
TpStatus DspReadTemperature(FixedPoint16* temperature) {
  DsScratchpad scratchpad;
  DspRecv((unsigned char*)&scratchpad, sizeof(DsScratchpad));

  if (DspCalcCrc(&scratchpad) != DS_VALID_CRC) {
    return TpCrcError;
  }

  Fp16WriteAsNumber(*temperature,
                    scratchpad.temperature >> DS_FP16_TEMPERATURE_SHIFT);

  return TpSuccess;
}

TpStatus DsInitialize(void) {
  DspPrepareGpio();
  DspSetupTransceiver();
  return TpSuccess;
}

TpStatus DsPrepare(DsResolution resolution) {
  /*
   * Resolution mask is stored in 6th and 5th bits of resolution configuration
   * byte
   */
  const uint8_t raw_resolution =
      (uint8_t)((resolution & DS_RESOLUTION_MASK) << DS_RESOLUTION_SHIFT);
  const DsConfig ds_config = {
      .th = DS_TH_MAX, .tl = DS_TL_MIN, .resolution = raw_resolution | 0x1F};

  return DspSendCommandWithPayload(
      DS_WRITE_SCRATCHPAD, (const unsigned char*)&ds_config, sizeof(DsConfig));
}

TpStatus DsConvertTemperature(void) {
  return DspSendCommand(DS_CONVERT_TEMPERATURE);
}

TpStatus DsReadTemperature(FixedPoint16* temperature) {
  if (!temperature) {
    return TpInvalidParameter;
  }

  TpStatus status;

  do {
    status = DspSendCommand(DS_READ_SCRATCHPAD);
    BREAK_ON_ERROR(status);
    status = DspReadTemperature(temperature);

  } while (status == TpCrcError);

  return status;
}
