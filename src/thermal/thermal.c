#include "thermal.h"
#include "ds18b20.h"

#include <heat/heat.h>
#include <tools/utils.h>

#include <stm32f0xx.h>

#define TM_TEMPERATURE_POLLING_PERIOD 500

typedef struct {
  uint16_t current_temperature;
  uint16_t temperature_point;
  TmMode mode;
  Ds18b20 temperature_sensor;
} TmState;

static TmState g_tm_state = {.current_temperature = TM_TEMPERATURE_MIN,
                             .temperature_point = TM_TEMPERATURE_MIN,
                             .mode = TmModeRelay,
                             .temperature_sensor = {.transceiver = USART2}};

static void TmpPrepareGpio(void) {
  /**
   * 1. Activate PA14 in the alternative function mode as open-drain
   * 2. Using external 4.7 kOhm pull-up is highly recommended
   * 3. Select AF1 (USARTx_TX) for PA14
   */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
  SET_BIT(GPIOB->MODER, GPIO_MODER_MODER14_1);  // (1)
  SET_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_14);
  SET_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR14_0);  // (2)
  SET_BIT(GPIOB->AFR[1], 0x01000000);           // (3)
}

static TpStatus TmpSetupSensor(void) {
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
  return DsInitialize(&g_tm_state.temperature_sensor);
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
  return TpSuccess;
}

static TpStatus TmpTryStart(void) {
  SET_BIT(TIM16->CR1, TIM_CR1_CEN);
  return TmpGetStatus();
}

static TpStatus TmpTryStop(void) {
  const TpStatus status = TmpGetStatus();
  if (!TP_SUCCESS(status)) {
    return status;
  }
  CLEAR_BIT(TIM16->CR1, TIM_CR1_CEN);
  HmSetPowerFactor(0);
  return TpSuccess;
}

TpStatus TmInitialize(void) {
  TmpPrepareGpio();
  RET_IF_ERROR(TmpSetupSensor());
  TmpSetupTimer();
  return TpSuccess;
}

TpStatus TmQueryTemperature(uint16_t* temperature) {
  const TpStatus status = TmpGetStatus();
  if (!TP_SUCCESS(status)) {
    return status;
  }
  if (!temperature) {
    return TpInvalidParameter;
  }
  *temperature = g_tm_state.current_temperature;
  return TpSuccess;
}

TpStatus TmSetState(bool enable) {
  if (!enable) {
    return TmpTryStop();
  }
  return TmpTryStart();
}

uint16_t TmGetCurrentTemperature(void) {
  return g_tm_state.current_temperature;
}

TpStatus TmSetMode(TmMode mode) {
  if (TP_SUCCESS(TmpGetStatus())) {
    return TpAlreadyRunning;
  }
  if (mode >= TmModeMax) {
    return TpInvalidParameter;
  }
  g_tm_state.mode = mode;
  return TpSuccess;
}

TpStatus TmSetTemperaturePoint(uint16_t value) {
  if (value > TM_TEMPERATURE_MAX) {
    return TpInvalidParameter;
  }
  g_tm_state.temperature_point = value;
  return TpSuccess;
}

void TIM16_IRQHandler(void) {
  g_tm_state.current_temperature = DsReadTemperature(&g_tm_state.temperature_sensor);
}
