#include "heat.h"

#include <tools/utils.h>

#include <stm32f0xx.h>

#define HM_PULSE_DELAY_MIN 50
#define HM_PULSE_DELAY_MAX 9300
#define HM_PULSE_DELAY_STEP \
  (HM_PULSE_DELAY_MAX / (HM_POWER_FACTOR_MAX - HM_POWER_FACTOR_MIN))
#define HM_PULSE_DELAY_LIMIT (HM_PULSE_DELAY_MAX + HM_PULSE_DELAY_MIN)

#define HM_WAVE_DURATION \
  (((1000000 / (HM_VOLTAGE_FREQUENCY * 2)) - HM_PULSE_DELAY_LIMIT) / 2)

typedef struct {
  uint8_t power_factor;
  uint16_t pulse_width;  // NOLINT(clang-diagnostic-padded)
  uint16_t pulse_delay;
} HmState;

static HmState g_hm_state = {.power_factor = (uint8_t)0,
                             .pulse_width = HM_WAVE_DURATION,
                             .pulse_delay = HM_PULSE_DELAY_MIN};

static void HmpPrepareGpio() {
  /**
   * 1. Activate PB14, PB15 in the alternative function mode
   * 2. Select AF1 (TIM15_CHx) for PB14, PB15
   * 3. Pull-up TIM_CH2 (trigger input)
   */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);
  SET_BIT(GPIOB->MODER, GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1);  // (1)
  SET_BIT(GPIOB->AFR[1], 0x11000000);                                  // (2)
  SET_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPDR15_0);                         // (3)
}

static void HmpSetupTimer() {
  /*
   * 1. Tick period is 1us
   * 2. One-pulse mode
   * 3. PWM mode 2 (_|_) without fast output
   * 4. OC1 configured as output with active high, OC2 configured as input
   * 5. Select input polarity (detect low level only)
   * 6. Trigger Mode - the counter starts at a rising edge on the TI2
   * 7. Main output enable
   */
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM15EN);
  TIM15->PSC = (uint16_t)(SystemCoreClock / 1000000 - 1);  // (1)
  SET_BIT(TIM15->CR1, TIM_CR1_OPM);                        // (2)
  SET_BIT(TIM15->CCMR1, TIM_CCMR1_OC1M);                   // (3)
  SET_BIT(TIM15->CCER, TIM_CCER_CC1E);                     // (4)
  CLEAR_BIT(TIM15->CCER, TIM_CCER_CC2P);                   // (5)
  SET_BIT(TIM15->SMCR, TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2 | TIM_SMCR_TS_2 |
                           TIM_SMCR_TS_1);  // (6)
  SET_BIT(TIM15->BDTR, TIM_BDTR_MOE);       // (7)
}

static void HmpStart(void) {
  /**
   * 1. Pulse delay is (TIM15_CCR1 - TIM15_CNT)
   * 2. Pulse width is (TIM15_ARR - TIM15_CCR1)
   */
  TIM15->CCR1 = g_hm_state.pulse_delay;                          // (1)
  TIM15->ARR = g_hm_state.pulse_delay + g_hm_state.pulse_width;  // (2)
  SET_BIT(TIM15->CR1, TIM_CR1_CEN);
}

static void HmpStop(void) {
  CLEAR_BIT(TIM15->CR1, TIM_CR1_CEN);
  TIM15->CNT = 0;
}

static void HmpSetup(uint8_t power_factor) {
  g_hm_state.power_factor = power_factor;
  if (power_factor == HM_POWER_FACTOR_MIN) {
    HmpStop();
  } else {
    g_hm_state.pulse_delay =
        HM_PULSE_DELAY_LIMIT - power_factor * HM_PULSE_DELAY_STEP;
    HmpStart();
  }
}

TpStatus HmInitialize(void) {
  HmpPrepareGpio();
  HmpSetupTimer();
  return TpSuccess;
}

uint8_t HmGetPowerFactor(void) {
  return g_hm_state.power_factor;
}

TpStatus HmSetPowerFactor(uint8_t power_factor) {
  if (power_factor > HM_POWER_FACTOR_MAX) {
    return TpInvalidParameter;
  }
  if (power_factor != HmGetPowerFactor()) {
    HmpSetup(power_factor);
  }
  return TpSuccess;
}
