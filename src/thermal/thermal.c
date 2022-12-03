#include "thermal.h"
#include "one_wire.h"

#include <heat/heat.h>
#include <tools/break_on.h>
#include <tools/utils.h>

#include <limits.h>
#include <string.h>

#include <stm32f0xx.h>

#define TM_TEMPERATURE_HYSTERESIS 5
#define TM_TEMPERATURE_MIN 0    // 5.0
#define TM_TEMPERATURE_MAX 100  // 100.0

#define TM_MODE_INACTIVE TmModeCount

#define TM_TEMPERATURE_POLLING_PERIOD 200
#define TM_TIMER_INTERRUPT_PRIORITY 1
#define TM_DMA_INTERRUPT_PRIORITY 1

#define TM_DS18B20_ACCURACY_MASK 0x20  // 10-bit accuracy
#define TM_DS18B20_TEMPERATURE_LSB_MASK 1
#define TM_DS18B20_TEMPERATURE_LSB_SHIFT 1
#define TM_DS18B20_TEMPERATURE_MSB_SHIFT \
  (CHAR_BIT - TM_DS18B20_TEMPERATURE_LSB_SHIFT)

#define TM_DS18B20_START_MEASUREMENT 0x44
#define TM_DS18B20_WRITE_SCRATCHPAD 0x4E
#define TM_DS18B20_READ_SCRATCHPAD 0xBE

typedef void (*tm_heater_handler_t)(void);
static void TmpRelayHeaterHandler(void);
static void TmpPidHeaterHandler(void);

typedef void (*tm_setup_handler_t)(FixedPoint16);
static void TmpRelaySetupHandler(FixedPoint16 temperature_point);
static void TmpPidSetupHandler(FixedPoint16 temperature_point);

typedef struct {
  FixedPoint16 max_temperature;
  FixedPoint16 min_temperature;
} TmRelayInfo;

typedef struct {
  FixedPoint16 max_temperature;
} TmPidInfo;

typedef struct {
  uint8_t temperature_lsb;
  uint8_t temperature_msb;
  uint8_t th;
  uint8_t tl;
  uint8_t configuration;
  unsigned char reserved[3];
  uint8_t crc;
} TmDs18b20Scratchpad;

typedef struct {
  unsigned char th;
  unsigned char tl;
  unsigned char configuration;
} TmDs18b20Config;

// NOLINTNEXTLINE(clang-diagnostic-padded)
typedef struct {
  TpStatus status;
  TmMode mode;
  tm_heater_handler_t heater_handlers[TmModeCount];
  tm_setup_handler_t setup_handlers[TmModeCount];
  FixedPoint16 current_temperature;
  union {
    TmRelayInfo relay;
    TmPidInfo pid;
  };
  const TmDs18b20Config config;
  unsigned char raw_scratchpad[sizeof(TmDs18b20Scratchpad) * OW_IO_GRANULARITY];
} TmState;

const FixedPoint16 TmTemperatureHysteresis = {TM_TEMPERATURE_HYSTERESIS, 0};
const FixedPoint16 TmTemperatureMin = {TM_TEMPERATURE_MIN, 0};
const FixedPoint16 TmTemperatureMax = {TM_TEMPERATURE_MAX, 0};

static TmState g_tm_state = {
    .status = TpNotReady,
    .mode = TM_MODE_INACTIVE,
    .heater_handlers = {&TmpRelayHeaterHandler, &TmpPidHeaterHandler},
    .setup_handlers = {&TmpRelaySetupHandler, &TmpPidSetupHandler},
    .config = {.th = TM_TEMPERATURE_MAX,
               .tl = TM_TEMPERATURE_MIN,
               .configuration = TM_DS18B20_ACCURACY_MASK | 0x1F}};

static void TmpPrepareGpio(void) {
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

static void TmpSetupTransceiver(void) {
  /**
   * 1. Setup USART2 in half-duplex mode with DMA receiver
   * 2. Activate DMA transfer error and transfer complete interrupts
   * 3. Specify transfer direction (from USART2->RDR to raw_scratchpad) and
   * data size in bytes
   * 4. Enable DMA interrupt
   */
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
  SET_BIT(USART2->CR3, USART_CR3_DMAR | USART_CR3_HDSEL);  // (1)
  SET_BIT(USART2->CR1, USART_CR1_RE | USART_CR1_TE);

  SET_BIT(RCC->AHBENR, RCC_AHBENR_DMAEN);
  SET_BIT(DMA1_Channel5->CCR,
          DMA_CCR_MINC | DMA_CCR_TEIE | DMA_CCR_TCIE);  // (2)
  DMA1_Channel5->CNDTR = sizeof(TmDs18b20Scratchpad) * OW_IO_GRANULARITY;
  DMA1_Channel5->CPAR = (uint32_t)&USART2->RDR;
  DMA1_Channel5->CMAR = (uint32_t)&g_tm_state.raw_scratchpad;
  NVIC_SetPriority(DMA1_Channel4_5_IRQn, TM_DMA_INTERRUPT_PRIORITY);
  NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
}

static void TmpSetupTimer() {
  /**
   * 1. Tick period is 1ms
   * 2. Enable timer interrupts
   * 3. Priority must be higher than user input interrupt handlers (i.e. less
   * absolute value)
   */
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM16EN);
  TIM16->PSC = (uint16_t)(SystemCoreClock / 1000 - 1);  // (1)
  TIM16->ARR = TM_TEMPERATURE_POLLING_PERIOD;

  SET_BIT(TIM16->DIER, TIM_DIER_UIE);  // (2)
  NVIC_EnableIRQ(TIM16_IRQn);
  NVIC_SetPriority(TIM16_IRQn, TM_TIMER_INTERRUPT_PRIORITY);  // (3)
}

static void TmpRelayHeaterHandler(void) {
  const FixedPoint16 current_temperature = g_tm_state.current_temperature;
  if (FpGreaterEqual(current_temperature, g_tm_state.relay.max_temperature)) {
    HmSetPowerFactor(HM_POWER_FACTOR_MIN);
  } else if (FpLessEqual(current_temperature,
                         g_tm_state.relay.min_temperature)) {
    HmSetPowerFactor(HM_POWER_FACTOR_MAX);
  }
}

static void TmpPidHeaterHandler(void) {
  HmSetPowerFactor(HM_POWER_FACTOR_MIN);
}

static void TmpRelaySetupHandler(FixedPoint16 temperature_point) {
  g_tm_state.relay.max_temperature = temperature_point;
  FpSub(temperature_point, TmTemperatureHysteresis,
        &g_tm_state.relay.min_temperature);
}

static void TmpPidSetupHandler(FixedPoint16 temperature_point) {
  (void)temperature_point;
}

static void TmpStart(void) {
  g_tm_state.status = TpNotReady;
  SET_BIT(USART2->CR1, USART_CR1_UE);
  SET_BIT(TIM16->CR1, TIM_CR1_CEN);
}

static void TmpStop(void) {
  CLEAR_BIT(TIM16->CR1, TIM_CR1_CEN);
  CLEAR_BIT(USART2->CR1, USART_CR1_UE);
  TIM16->CNT = 0;
  HmSetPowerFactor(HM_POWER_FACTOR_MIN);
}

bool TmpValidateSettings(TmMode mode, FixedPoint16 value) {
  return mode < TmModeCount && FpGreaterEqual(value, TmTemperatureMin) &&
         FpLessEqual(value, TmTemperatureMax);
}

TpStatus TmpSetup(TmMode mode, FixedPoint16 value) {
  g_tm_state.mode = mode;
  const tm_setup_handler_t setup_handler = g_tm_state.setup_handlers[mode];
  setup_handler(value);
  return TpSuccess;
}

static void TmpRaiseDeviceError() {
  g_tm_state.status = TpDeviceNotConnected;
  TmpStop();
}

static TpStatus TmpGetMeasurementStatus(bool wait) {
  TpStatus status;
  do {
    status = g_tm_state.status;
  } while (wait && status == TpNotReady);
  return status;
}

static void TmpStartMeasurement(void) {
  do {
    TpStatus status = OwBroadcastCommandEx(
        USART2, TM_DS18B20_WRITE_SCRATCHPAD,
        (const unsigned char*)&g_tm_state.config, sizeof(TmDs18b20Config));
    BREAK_ON_ERROR(status);

    status = OwBroadcastCommand(USART2, TM_DS18B20_START_MEASUREMENT);
    BREAK_ON_ERROR(status);

    g_tm_state.status = status;
    return;

  } while (false);

  TmpRaiseDeviceError();
}

static void TmpConvertTemperature(void) {
  TmDs18b20Scratchpad scratchpad;
  OwRawToBytes(g_tm_state.raw_scratchpad, sizeof(TmDs18b20Scratchpad),
               (unsigned char*)&scratchpad);

  FixedPoint16* temperature = &g_tm_state.current_temperature;
  temperature->whole =
      (int16_t)(scratchpad.temperature_msb << TM_DS18B20_TEMPERATURE_MSB_SHIFT |
                scratchpad.temperature_lsb >> TM_DS18B20_TEMPERATURE_LSB_SHIFT);
  temperature->fractional =
      (uint16_t)(scratchpad.temperature_lsb & TM_DS18B20_TEMPERATURE_LSB_MASK);
}

TpStatus TmInitialize(void) {
  TmpPrepareGpio();
  TmpSetupTransceiver();
  TmpSetupTimer();
  return TpSuccess;
}

TpStatus TmQueryTemperature(FixedPoint16* temperature, bool wait) {
  if (!temperature) {
    return TpInvalidParameter;
  }

  const TpStatus status = TmpGetMeasurementStatus(wait);
  if (!TP_SUCCESS(status)) {
    return status;
  }

  *temperature = g_tm_state.current_temperature;
  return TpSuccess;
}

void TmSetState(bool enable) {
  if (!enable) {
    TmpStop();
  } else {
    TmpStart();
  }
}

TpStatus TmSetup(TmMode mode, FixedPoint16 temperature_point) {
  if (!TmpValidateSettings(mode, temperature_point)) {
    return TpInvalidParameter;
  }
  if (TP_SUCCESS(g_tm_state.status)) {
    return TpAlreadyRunning;
  }
  return TmpSetup(mode, temperature_point);
}

void TIM16_IRQHandler(void) {
  CLEAR_BIT(TIM16->SR, TIM_SR_UIF);

  if (!TP_SUCCESS(g_tm_state.status)) {
    TmpStartMeasurement();

  } else {
    const TpStatus status =
        OwBroadcastCommand(USART2, TM_DS18B20_READ_SCRATCHPAD);
    if (!TP_SUCCESS(status)) {
      TmpRaiseDeviceError();
    } else {
      SET_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
    }
  }
}

void DMA1_Channel4_5_IRQHandler(void) {
  if (READ_BIT(DMA1->ISR, DMA_ISR_TEIF5)) {
    DMA1->IFCR = DMA_IFCR_CTEIF5;
    TmpRaiseDeviceError();

  } else if (READ_BIT(DMA1->ISR, DMA_ISR_TCIF5)) {
    DMA1->IFCR = DMA_IFCR_CTCIF5;
    TmpConvertTemperature();

    const tm_heater_handler_t heater_handler =
        g_tm_state.heater_handlers[g_tm_state.mode];
    heater_handler();

    TmpStartMeasurement();
  }
  CLEAR_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
}
