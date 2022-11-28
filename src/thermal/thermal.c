#include "thermal.h"
#include "ds18b20.h"

#include <heat/heat.h>
#include <tools/utils.h>

#include <stm32f0xx.h>

#define TM_TEMPERATURE_HYSTERESIS 5
#define TM_TEMPERATURE_MIN (0 + TM_TEMPERATURE_HYSTERESIS)  // 5.0
#define TM_TEMPERATURE_MAX 100                              // 100.0

#define TM_TEMPERATURE_POLLING_PERIOD 500
#define TM_TIMER_INTERRUPT_PRIORITY 1

typedef struct {
  TmMode mode;
  TpStatus status;
  FixedPoint16 current_temperature;
  FixedPoint16 temperature_point;
} TmState;

const FixedPoint16 TmTemperatureMin = {TM_TEMPERATURE_MIN, 0};
const FixedPoint16 TmTemperatureMax = {TM_TEMPERATURE_MAX, 0};

static TmState g_tm_state = {.mode = TmModeRelay, .status = TpSuccess};

static void TmpPrepareGpio(void) {
  /**
   * 1. Activate PA14 in the alternative function mode as open-drain
   * 2. Using external 4.7 kOhm pull-up is highly recommended
   * 3. Select AF1 (USARTx_TX) for PA14
   */
  //SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
  //SET_BIT(GPIOA->MODER, GPIO_MODER_MODER14_1);  // (1)
  //SET_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_14);
  //SET_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR14_0);  // (2)
  //SET_BIT(GPIOA->AFR[1], 0x01000000);           // (3)
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

static TpStatus TmpGetStatus(void) {
  return g_tm_state.status;
}

static void TmpStart(void) {
  SET_BIT(TIM16->CR1, TIM_CR1_CEN);
}

static void TmpStop(void) {
  CLEAR_BIT(TIM16->CR1, TIM_CR1_CEN);
  HmSetPowerFactor(0);
}

static void TmpUpdateHeaterState(void) {}

static bool TmpUpdateTemperature(TpStatus status, FixedPoint16 value) {
  g_tm_state.status = status;
  if (!TP_SUCCESS(status)) {
    TmpStop();
    return false;
  }
  g_tm_state.current_temperature = value;
  TmpUpdateHeaterState();
  return true;
}

TpStatus TmInitialize(void) {
  g_tm_state.current_temperature = TmTemperatureMin;
  g_tm_state.temperature_point = TmTemperatureMin;

  TmpPrepareGpio();
  RET_IF_ERROR(DsInitialize());
  TmpSetupTimer();

  return TpSuccess;
}

TpStatus TmQueryTemperature(FixedPoint16* temperature) {
  /*const TpStatus status = TmpGetStatus();
  if (!TP_SUCCESS(status)) {
    return status;
  }*/
  if (!temperature) {
    return TpInvalidParameter;
  }
  *temperature = g_tm_state.current_temperature;
  return TpSuccess;
}

TpStatus TmSetState(bool enable) {
  if (enable) {
    TmpStart();
    return TmpGetStatus();
  }
  const TpStatus status = TmpGetStatus();
  RET_IF_ERROR(status);
  TmpStop();
  return TpSuccess;
}

TpStatus TmSetMode(TmMode mode) {
  if (TP_SUCCESS(TmpGetStatus())) {
    return TpAlreadyRunning;
  }
  if (mode >= TmModeCount) {
    return TpInvalidParameter;
  }
  g_tm_state.mode = mode;
  return TpSuccess;
}

TpStatus TmSetTemperaturePoint(FixedPoint16 value) {
  if (FpGreater(value, TmTemperatureMax)) {
    return TpInvalidParameter;
  }
  g_tm_state.temperature_point = value;
  return TpSuccess;
}

void TIM16_IRQHandler(void) {
  DsReadTemperatureAsync(&TmpUpdateTemperature);
}
