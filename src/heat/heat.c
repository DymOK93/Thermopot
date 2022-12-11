#include "heat.h"

#include <tools/utils.h>

#include <stm32f0xx.h>

/**
 * At HM_POWER_FACTOR_MIN heater is always off
 *
 * Maximum zero detection delay is (28us + t_rise) ~ 350us so maximum pulse
 * delay is (10000 - (HM_PULSE_WIDTH + 350))
 */
#define HM_PULSE_WIDTH 50
#define HM_PULSE_DELAY_COUNT (HM_POWER_FACTOR_MAX - HM_POWER_FACTOR_MIN)

typedef struct {
  uint8_t power_factor;
  uint8_t turn_on_point;
  const uint16_t pulse_delay[HM_PULSE_DELAY_COUNT];
} HmState;

static HmState g_hm_state = {
    .power_factor = (uint8_t)0,
    .pulse_delay = {9602, 9554, 9505, 9457, 9408, 9360, 9311, 9262, 9213, 9163,
                    9114, 9064, 9014, 8963, 8912, 8861, 8809, 8757, 8704, 8650,
                    8596, 8542, 8486, 8430, 8372, 8314, 8255, 8195, 8134, 8071,
                    8006, 7940, 7872, 7802, 7730, 7655, 7578, 7497, 7412, 7324,
                    7229, 7129, 7021, 6904, 6774, 6627, 6455, 6243, 5944, 4825,
                    3706, 3407, 3195, 3023, 2876, 2746, 2629, 2521, 2421, 2326,
                    2238, 2153, 2072, 1995, 1920, 1848, 1778, 1710, 1644, 1579,
                    1516, 1455, 1395, 1336, 1277, 1220, 1164, 1108, 1054, 1000,
                    946,  893,  841,  789,  738,  687,  636,  586,  536,  487,
                    437,  388,  339,  290,  242,  193,  145,  96,   48,   0}};

static void HmpPrepareGpio() {
  /**
   * 1. Activate PB14, PB15 in the alternative function mode
   * 2. Pull-down TIM_CH1 (main output) and pull-up TIM_CH2 (trigger input)
   */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);
  SET_BIT(GPIOB->MODER, GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1);  // (1)
  SET_BIT(GPIOB->AFR[1], 0x11000000);
  SET_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPDR14_1 | GPIO_PUPDR_PUPDR15_0);  // (2)
}

static void HmpSetupTimer() {
  /*
   * 1. Tick period is 1us
   * 2. One-pulse mode
   * 3. PWM mode 2 (_|_) without fast output
   * 4. OC1 configured as output with active high, OC2 configured as input
   * sensitive to failing edge
   * 5. Trigger Mode - the counter starts at a rising edge on the TI2
   */
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM15EN);
  TIM15->PSC = (uint16_t)(SystemCoreClock / 1000000 - 1);    // (1)
  SET_BIT(TIM15->CR1, TIM_CR1_OPM);                          // (2)
  SET_BIT(TIM15->CCMR1, TIM_CCMR1_CC2S_0 | TIM_CCMR1_OC1M);  // (3)
  SET_BIT(TIM15->CCER, TIM_CCER_CC1E | TIM_CCER_CC2P);       // (4)
  SET_BIT(TIM15->SMCR, TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2 | TIM_SMCR_TS_2 |
                           TIM_SMCR_TS_1);  // (5)
}

static void HmpStart(void) {
  /*
   * Main output enable
   */
  SET_BIT(TIM15->BDTR, TIM_BDTR_MOE);
}

static void HmpStop(void) {
  /*
   * Main output disable
   */
  CLEAR_BIT(TIM15->BDTR, TIM_BDTR_MOE);
}

static void HmpSetPowerFactor(uint8_t power_factor) {
  /**
   * 1. Pulse delay is (TIM15_CCR1 - TIM15_CNT)
   * 2. Pulse width is (TIM15_ARR - TIM15_CCR1)
   */
  g_hm_state.power_factor = power_factor;
  if (power_factor == HM_POWER_FACTOR_MIN) {
    HmpStop();
  } else {
    const uint16_t pulse_delay = g_hm_state.pulse_delay[power_factor - 1];
    TIM15->CCR1 = pulse_delay;                  // (1)
    TIM15->ARR = pulse_delay + HM_PULSE_WIDTH;  // (2)
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
    HmpSetPowerFactor(power_factor);
  }
  return TpSuccess;
}
