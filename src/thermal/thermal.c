#include "thermal.h"
#include "ds18b20.h"

#include <heat/heat.h>
#include <tools/utils.h>

#include <stm32f0xx.h>

#define TM_TEMPERATURE_HYSTERESIS 5
#define TM_TEMPERATURE_MIN 0    // 5.0
#define TM_TEMPERATURE_MAX 100  // 100.0

#define TM_MODE_INACTIVE TmModeCount

#define TM_TEMPERATURE_POLLING_PERIOD 500
#define TM_TIMER_INTERRUPT_PRIORITY 2

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

// NOLINTNEXTLINE(clang-diagnostic-padded)
typedef struct {
  volatile TpStatus status;
  TmMode mode;
  tm_heater_handler_t heater_handlers[TmModeCount];
  tm_setup_handler_t setup_handlers[TmModeCount];
  volatile FixedPoint16 current_temperature;
  union {
    TmRelayInfo relay;
    TmPidInfo pid;
  };
} TmState;

const FixedPoint16 TmTemperatureHysteresis = {TM_TEMPERATURE_HYSTERESIS, 0};
const FixedPoint16 TmTemperatureMin = {TM_TEMPERATURE_MIN, 0};
const FixedPoint16 TmTemperatureMax = {TM_TEMPERATURE_MAX, 0};

static TmState g_tm_state = {
    .status = TpNotReady,
    .mode = TM_MODE_INACTIVE,
    .heater_handlers = {&TmpRelayHeaterHandler, &TmpPidHeaterHandler},
    .setup_handlers = {&TmpRelaySetupHandler, &TmpPidSetupHandler}};

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
  } while (wait && (status == TpNotReady || status == TpPending));
  return status;
}

static void TmpStartMeasurement(TpStatus wait_status) {
  if (!TP_SUCCESS(DsStartMeasurement())) {
    TmpRaiseDeviceError();
  } else {
    g_tm_state.status = wait_status;
  }
}

TpStatus TmInitialize(void) {
  TmpSetupTimer();
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
    TmpStartMeasurement(TpPending);

  } else {
    FixedPoint16 temperature;
    const TpStatus status = DsQueryTemperature(&temperature);
    if (!TP_SUCCESS(status)) {
      TmpRaiseDeviceError();
    } else {
      g_tm_state.current_temperature = temperature;

      const tm_heater_handler_t heater_handler =
          g_tm_state.heater_handlers[g_tm_state.mode];
      heater_handler();

      TmpStartMeasurement(TpSuccess);
    }
  }
}
