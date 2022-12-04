#include "heat.h"

#include <tools/utils.h>

#include <stm32f0xx.h>

/**
 * At HM_POWER_FACTOR_MIN heater is always off
 *
 * Zero detection delay is ~28us so HM_POWER_FACTOR_OFFSET can start from
 * position where (10000 - (HM_PULSE_WIDTH + 28)) > 0
 */
#define HM_PULSE_WIDTH 100
#define HM_POWER_FACTOR_OFFSET 2
#define HM_PULSE_DELAY_COUNT \
  (HM_POWER_FACTOR_MAX - HM_POWER_FACTOR_MIN - HM_POWER_FACTOR_OFFSET)

typedef struct {
  uint8_t power_factor;
  uint8_t turn_on_point;
  const uint16_t pulse_delay[HM_PULSE_DELAY_COUNT];
} HmState;

static HmState g_hm_state = {
    .power_factor = (uint8_t)0,
    .pulse_delay = {9850, 9800, 9749, 9699, 9649, 9598, 9547, 9496, 9445, 9393,
                    9341, 9288, 9235, 9182, 9128, 9074, 9020, 8964, 8908, 8852,
                    8794, 8736, 8676, 8616, 8555, 8492, 8429, 8363, 8296, 8228,
                    8158, 8085, 8011, 7933, 7853, 7769, 7681, 7589, 7491, 7388,
                    7276, 7154, 7020, 6868, 6690, 6469, 6160, 5000, 3840, 3531,
                    3310, 3132, 2980, 2846, 2724, 2612, 2509, 2411, 2319, 2231,
                    2147, 2067, 1990, 1915, 1842, 1772, 1704, 1637, 1571, 1508,
                    1445, 1384, 1324, 1264, 1206, 1148, 1092, 1036, 980,  926,
                    872,  818,  765,  712,  659,  607,  556,  504,  453,  402,
                    351,  301,  250,  200,  150,  100,  50,   0}};

static void HmpPrepareGpio() {
  /**
   * 1. Activate PB14, PB15 in the alternative function mode
   * 2. Pull-up TIM_CH2 (trigger input)
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
   * sensitive to raising edge
   * 5. Trigger Mode - the counter starts at a rising edge on the TI2
   */
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM15EN);
  TIM15->PSC = (uint16_t)(SystemCoreClock / 1000000 - 1);    // (1)
  SET_BIT(TIM15->CR1, TIM_CR1_OPM);                          // (2)
  SET_BIT(TIM15->CCMR1, TIM_CCMR1_CC2S_0 | TIM_CCMR1_OC1M);  // (3)
  SET_BIT(TIM15->CCER, TIM_CCER_CC1E);                       // (4)
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
  if (power_factor <= HM_POWER_FACTOR_OFFSET) {
    HmpStop();
  } else {
    const uint16_t pulse_delay =
        g_hm_state.pulse_delay[power_factor - HM_POWER_FACTOR_OFFSET - 1];
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
