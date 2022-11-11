#include "heat.h"

#define HM_WAVE_DELAY_MIN 50
#define HM_WAVE_DELAY_MAX 9300
#define HM_WAVE_DELAY_STEP \
  (HM_WAVE_DELAY_MAX / (HM_POWER_FACTOR_MAX - HM_POWER_FACTOR_MIN))

typedef struct {
  uint8_t power_factor;
  uint8_t pulse_width;
  uint16_t wave_delay;
} HmState;

static HmState g_hm_state = {0};

static void HmpStart(void) {}
static void HmpStop(void) {}

static uint16_t HmpCalcWaveDelay(uint8_t power_factor) {
  return HM_WAVE_DELAY_MIN + HM_WAVE_DELAY_MAX -
         power_factor * HM_WAVE_DELAY_STEP;
}

static void HmpSetup(uint8_t power_factor) {
  g_hm_state.power_factor = power_factor;
  if (power_factor == HM_POWER_FACTOR_MIN) {
    HmpStop();
  } else {
    g_hm_state.wave_delay = HmpCalcWaveDelay(power_factor);
    HmpStart();
  }
}

TpStatus HmInitialize(void) {
  return TpSuccess;
}

uint8_t HmGetPowerFactor(void) {
  return g_hm_state.power_factor;
}

TpStatus HmSetPowerFactor(uint8_t power_factor) {
  if (power_factor > HM_POWER_FACTOR_MAX) {
    return TpOverflow;
  }
  if (power_factor != HmGetPowerFactor()) {
    HmpSetup(power_factor);
  }
  return TpSuccess;
}
