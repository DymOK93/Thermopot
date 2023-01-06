#include "thermal.h"
#include "ds18b20.h"

#include <heat/heat.h>
#include <tools/break_on.h>
#include <tools/utils.h>

#include <stm32f0xx.h>

#define TM_TEMPERATURE_HYSTERESIS_UP (-2)
#define TM_TEMPERATURE_HYSTERESIS_DOWN_WHOLE 2
#define TM_TEMPERATURE_HYSTERESIS_DOWN_FRACTIONAL 5
#define TM_TEMPERATURE_MIN 0    // 5.0
#define TM_TEMPERATURE_MAX 100  // 100.0

#define TM_MODE_INACTIVE TmModeCount

#define TM_TEMPERATURE_POLLING_PERIOD 100
#define TM_TIMER_INTERRUPT_PRIORITY 2

#define TM_PID_SAMPLE_TIME (TM_TEMPERATURE_POLLING_PERIOD / 1000.0f)
#define TM_PID_P_FACTOR (5.5f)
#define TM_PID_I_FACTOR 1

#define TM_DEBUG_INTERFACE_INTERRUPT_PRIORITY 3
#define TM_DBG_TRANSFER_BAUD 115200

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
  FixedPoint16 temperature_hysteresis_up;
  FixedPoint16 temperature_hysteresis_down;
} TmRelaySettings;

typedef struct {
  float temperature_point;
  float accumulated_difference;
} TmPidInfo;

typedef struct {
  float p_factor;
  float i_factor;
} TmPidSettings;

typedef struct {
  TmMode type;
  union {
    TmRelaySettings relay_settings;
    TmPidSettings pid_settings;
  };
} TmSettingsPacket;

typedef struct {
  volatile TpStatus status;
  volatile TmMode mode;
  const tm_heater_handler_t heater_handlers[TmModeCount];
  const tm_setup_handler_t setup_handlers[TmModeCount];
  volatile FixedPoint16 current_temperature;

  // NOLINTNEXTLINE(clang-diagnostic-padded)
  union {
    TmRelayInfo relay_info;
    TmPidInfo pid_info;
  };

  volatile TmRelaySettings relay_settings;
  volatile TmPidSettings pid_settings;
  TmSettingsPacket settings_packet;
} TmState;

const FixedPoint16 TmTemperatureMin = Fp16Initialize(TM_TEMPERATURE_MIN, 0);
const FixedPoint16 TmTemperatureMax = Fp16Initialize(TM_TEMPERATURE_MAX, 0);

static TmState g_tm_state = {
    .status = TpNotReady,
    .mode = TM_MODE_INACTIVE,
    .heater_handlers = {&TmpRelayHeaterHandler, &TmpPidHeaterHandler},
    .setup_handlers = {&TmpRelaySetupHandler, &TmpPidSetupHandler},
    .relay_settings = {.temperature_hysteresis_up =
                           Fp16Initialize(TM_TEMPERATURE_HYSTERESIS_UP, 0),
                       .temperature_hysteresis_down =
                           Fp16Initialize(TM_TEMPERATURE_HYSTERESIS_DOWN_WHOLE,
                                          TM_TEMPERATURE_HYSTERESIS_DOWN_FRACTIONAL)},
    .pid_settings = {.p_factor = TM_PID_P_FACTOR,
                     .i_factor = TM_PID_I_FACTOR}};

static void TmpPrepareGpio(void) {
  /**
   * 1. Activate PA10 in the alternative function mode
   * 2. Setup weak pull-up on PA10
   * 3. Select AF1 (USARTx_RX) for PA10 (3)
   */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
  SET_BIT(GPIOA->MODER, GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1);  // (1)
  SET_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR10_0);  // (2)
  SET_BIT(GPIOA->AFR[1], 0x00000110);                                 // (3)
}

static void TmpSetupTimer() {
  /**
   * 1. Tick period is 1ms
   * 2. Enable update interrupt
   * 3. Only counter overflow/underflow generates an update interrupt
   * 4. Timer interrupt priority must be higher than user input interrupt
   * handlers (i.e. less absolute value)
   */
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM16EN);
  TIM16->PSC = (uint16_t)(SystemCoreClock / 1000 - 1);  // (1)
  TIM16->ARR = TM_TEMPERATURE_POLLING_PERIOD;

  SET_BIT(TIM16->DIER, TIM_DIER_UIE);  // (2)
  SET_BIT(TIM16->CR1, TIM_CR1_URS);    // (3)

  NVIC_EnableIRQ(TIM16_IRQn);
  NVIC_SetPriority(TIM16_IRQn, TM_TIMER_INTERRUPT_PRIORITY);  // (4)
}

static void TmpSetupDebugInterface(void) {
  SET_BIT(RCC->AHBENR, RCC_AHBENR_DMAEN);
  SET_BIT(DMA1_Channel3->CCR,
          DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TEIE | DMA_CCR_TCIE);
  DMA1_Channel3->CPAR = (uint32_t)&USART1->RDR;
  DMA1_Channel3->CNDTR = sizeof(TmSettingsPacket);
  DMA1_Channel3->CMAR = (uint32_t)&g_tm_state.settings_packet;
  SET_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);

  NVIC_SetPriority(DMA1_Channel2_3_IRQn, TM_DEBUG_INTERFACE_INTERRUPT_PRIORITY);
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
  USART1->BRR = SystemCoreClock / TM_DBG_TRANSFER_BAUD;
  SET_BIT(USART1->CR3, USART_CR3_DMAR | USART_CR3_DMAT);
  SET_BIT(USART1->CR1, USART_CR1_RE | USART_CR1_TE | USART_CR1_UE);
}

static void TmpRelayHeaterHandler(void) {
  const FixedPoint16 current_temperature = g_tm_state.current_temperature;
  if (Fp16GreaterEqual(current_temperature,
                       g_tm_state.relay_info.max_temperature)) {
    HmSetPowerFactor(HM_POWER_FACTOR_MIN);
  } else if (Fp16LessEqual(current_temperature,
                           g_tm_state.relay_info.min_temperature)) {
    HmSetPowerFactor(HM_POWER_FACTOR_MAX);
  }
}

static float TmpConvertToFloat(FixedPoint16 value) {
  return (float)Fp16ReadAsNumber(value) / (1 << FP16_FRACTIONAL_BITS);
}

static void TmpPidSetPowerFactor(float difference,
                                 float accumulated_difference) {
  const float result = difference * TM_PID_P_FACTOR +
                       accumulated_difference * TM_PID_I_FACTOR;

  int32_t power_factor = (int32_t)result;

  if (power_factor > HM_POWER_FACTOR_MAX) {
    power_factor = HM_POWER_FACTOR_MAX;
  } else if (power_factor < HM_POWER_FACTOR_MIN) {
    power_factor = HM_POWER_FACTOR_MIN;
  }

  HmSetPowerFactor((uint8_t)power_factor);
}

static void TmpPidHeaterHandler(void) {
  const float difference = g_tm_state.pid_info.temperature_point -
                           TmpConvertToFloat(g_tm_state.current_temperature);

  const float accumulated_difference =
      g_tm_state.pid_info.accumulated_difference;
  g_tm_state.pid_info.accumulated_difference += difference * TM_PID_SAMPLE_TIME;

  TmpPidSetPowerFactor(difference, accumulated_difference);
}

static void TmpRelaySetupHandler(FixedPoint16 temperature_point) {
  Fp16Add(g_tm_state.relay_info.max_temperature, temperature_point,
          g_tm_state.relay_settings.temperature_hysteresis_up);
  Fp16Sub(g_tm_state.relay_info.min_temperature, temperature_point,
          g_tm_state.relay_settings.temperature_hysteresis_down);
}

static void TmpPidSetupHandler(FixedPoint16 temperature_point) {
  g_tm_state.pid_info.temperature_point = TmpConvertToFloat(temperature_point);
  g_tm_state.pid_info.accumulated_difference = 0;
}

static void TmpStart(void) {
  g_tm_state.status = TpNotReady;
  SET_BIT(USART2->CR1, USART_CR1_UE);
  SET_BIT(TIM16->CR1, TIM_CR1_CEN);
}

static void TmpStop(void) {
  CLEAR_BIT(TIM16->CR1, TIM_CR1_CEN);
  CLEAR_BIT(USART2->CR1, USART_CR1_UE);
  HmSetPowerFactor(HM_POWER_FACTOR_MIN);
}

static bool TmpValidateSettings(TmMode mode, FixedPoint16 value) {
  return mode < TmModeCount && Fp16GreaterEqual(value, TmTemperatureMin) &&
         Fp16LessEqual(value, TmTemperatureMax);
}

static TpStatus TmpSetup(TmMode mode, FixedPoint16 value) {
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
  } while (wait && (status == TpNotReady || status == TpPending));
  return status;
}

static void TmpStartMeasurement(void) {
  do {
    TpStatus status = DsPrepare(DsResolution9Bit);
    BREAK_ON_ERROR(status);

    status = DsConvertTemperature();
    BREAK_ON_ERROR(status);

    g_tm_state.status = TpPending;
    return;

  } while (false);

  TmpRaiseDeviceError();
}

static void TmpContinueMeasurement(void) {
  if (!TP_SUCCESS(DsConvertTemperature())) {
    TmpRaiseDeviceError();
  } else {
    g_tm_state.status = TpSuccess;
  }
}

static void TmpUpdateSettings(void) {
  const TmSettingsPacket* settings_packet = &g_tm_state.settings_packet;
  if (settings_packet->type == TmModePid) {
    g_tm_state.pid_settings = settings_packet->pid_settings;
  } else if (settings_packet->type == TmModeRelay) {
    g_tm_state.relay_settings = settings_packet->relay_settings;
  }
}

TpStatus TmInitialize(void) {
  TmpPrepareGpio();
  TmpSetupTimer();
  TmpSetupDebugInterface();
  return DsInitialize();
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
  if (READ_BIT(TIM16->CR1, TIM_CR1_CEN)) {
    return TpAlreadyRunning;
  }
  return TmpSetup(mode, temperature_point);
}

void TIM16_IRQHandler(void) {
  CLEAR_BIT(TIM16->SR, TIM_SR_UIF);

  if (!TP_SUCCESS(g_tm_state.status)) {
    TmpStartMeasurement();

  } else {
    FixedPoint16 temperature;
    const TpStatus status = DsReadTemperature(&temperature);
    if (!TP_SUCCESS(status)) {
      TmpRaiseDeviceError();
    } else {
      g_tm_state.current_temperature = temperature;

      const tm_heater_handler_t heater_handler =
          g_tm_state.heater_handlers[g_tm_state.mode];
      heater_handler();

      TmpContinueMeasurement();
    }
  }
}

void DMA1_Channel2_3_IRQHandler(void) {
  const uint32_t status = DMA1->ISR;
  if (READ_BIT(status, DMA_ISR_TEIF3)) {
    DMA1->IFCR = DMA_IFCR_CTEIF3;
  } else if (READ_BIT(status, DMA_ISR_TCIF3)) {
    DMA1->IFCR = DMA_IFCR_CTCIF3;
    TmpUpdateSettings();
  }
}
