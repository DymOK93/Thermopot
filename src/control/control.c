#include "control.h"

#include <ssi/ssi.h>
#include <thermal/thermal.h>
#include <tools/fixed_point.h>
#include <tools/utils.h>

#include <stm32f0xx.h>

#define CTRL_TEMPERATURE_STEP 1
#define CTRL_BUTTON_INTERRUPT_PRIORITY 3
#define CTRL_TIMER_INTERRUPT_PRIORITY 3
#define CTRL_BUTTON_DEBOUNCE_DELAY 100  // Milliseconds

typedef enum {
  CtrlStageInitialized = -1,
  CtrlStageSetupTemperature,
  CtrlStageSetupMode,
  CtrlStageSetupFinished,
  CtrlStageError,
  CtrlStageCount
} CtrlStage;

typedef enum {
  CtrlButtonUser,
  CtrlButtonUp,
  CtrlButtonDown,
  CtrlButtonCount
} CtrlButton;

typedef void (*ctrl_button_handler_t)(CtrlButton);
static void CtrlpSetupTemperature(CtrlButton key);
static void CtrlpSetupMode(CtrlButton key);
static void CtrlpDummyKeyHandler(CtrlButton key);

// NOLINTNEXTLINE(clang-diagnostic-padded)
typedef struct {
  ctrl_button_handler_t button_handler[CtrlStageCount];
  CtrlStage stage;
  TmMode mode;
  CtrlButton scheduled_button;
  FixedPoint16 temperature_point;
  const FixedPoint16 temperature_step;
  const SsiValue mode_label[TmModeCount];
  const SsiValue error_msg;
} CtrlState;

static CtrlState g_ctrl_state = {
    .button_handler = {&CtrlpSetupTemperature, &CtrlpSetupMode,
                       &CtrlpDummyKeyHandler, &CtrlpDummyKeyHandler},
    .stage = CtrlStageInitialized,
    .mode = TmModeRelay,
    .scheduled_button = CtrlButtonCount,
    .temperature_step = {CTRL_TEMPERATURE_STEP, 0},
    .mode_label = {{"ler"},   // "rel"
                   {"dip"}},  // "pid"
    .error_msg = {"rre"}      // "err"
};

static void CtrlpPrepareGpio(void) {
  /*
   * Configure PA0 (user button) as weak pull-down input, PA11, PA12 (up/down
   * buttons) - as weak pull-up inputs
   */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
  SET_BIT(GPIOA->PUPDR,
          GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR11_0 | GPIO_PUPDR_PUPDR12_0);
}

static void CtrlpSetupKeyInterrupts(void) {
  /**
   * 1. Enable interrupts on pins PA0 (user button), PA11 and PA12 (up/down)
   * 2. Operation mode: failing edge
   * 3. Button interrupt priority must be lower than temperature sensor timer
   * interrupt priority (i.e. higher in absolute priority value)
   */
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
  SET_BIT(EXTI->IMR, EXTI_IMR_MR0 | EXTI_IMR_MR11 | EXTI_IMR_MR12);  // (1)
  SET_BIT(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI0_PA);
  SET_BIT(SYSCFG->EXTICR[2], SYSCFG_EXTICR3_EXTI11_PA);
  SET_BIT(SYSCFG->EXTICR[3], SYSCFG_EXTICR4_EXTI12_PA);
  SET_BIT(EXTI->FTSR, EXTI_FTSR_TR0 | EXTI_FTSR_TR11 | EXTI_FTSR_TR12);  // (2)
  NVIC_SetPriority(EXTI0_1_IRQn, CTRL_BUTTON_INTERRUPT_PRIORITY);        // (3)
  NVIC_SetPriority(EXTI4_15_IRQn, CTRL_BUTTON_INTERRUPT_PRIORITY);
  NVIC_EnableIRQ(EXTI0_1_IRQn);
  NVIC_EnableIRQ(EXTI4_15_IRQn);
}

static void CtrlpSetupTimer(void) {
  /*
   * 1. Tick period is 1ms
   * 2. One-pulse mode
   * 3. User interrupt enable
   */
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM14EN);
  TIM14->PSC = (uint16_t)(SystemCoreClock / 1000 - 1);  // (1)
  TIM14->ARR = CTRL_BUTTON_DEBOUNCE_DELAY;

  SET_BIT(TIM14->CR1, TIM_CR1_OPM);    // (2)
  SET_BIT(TIM14->DIER, TIM_DIER_UIE);  // (3)

  NVIC_SetPriority(TIM14_IRQn, CTRL_TIMER_INTERRUPT_PRIORITY);
  NVIC_EnableIRQ(TIM14_IRQn);
}

static void CtrlpSetupTemperature(CtrlButton button) {
  if (button == CtrlButtonUp &&
      FpLess(g_ctrl_state.temperature_point, TmTemperatureMax)) {
    FpAdd(g_ctrl_state.temperature_point, g_ctrl_state.temperature_step,
          &g_ctrl_state.temperature_point);
  } else if (button == CtrlButtonDown &&
             FpGreater(g_ctrl_state.temperature_point, TmTemperatureMin)) {
    FpSub(g_ctrl_state.temperature_point, g_ctrl_state.temperature_step,
          &g_ctrl_state.temperature_point);
  }
}

static void CtrlpSetupMode(CtrlButton button) {
  /*
   * It doesn't matter which button was clicked as there are only 2 possible
   * values
   */
  (void)button;
  g_ctrl_state.mode =
      g_ctrl_state.mode == TmModeRelay ? TmModePid : TmModeRelay;
}

static void CtrlpDummyKeyHandler(CtrlButton button) {
  /*
   * Do nothing - setup is finished or not started yet
   */
  (void)button;
}

static void CtrlpNextMode(void) {
  const CtrlStage stage = g_ctrl_state.stage;
  switch (stage) {  // NOLINT(clang-diagnostic-switch-enum)
    case CtrlStageSetupTemperature:
    case CtrlStageSetupMode:
      g_ctrl_state.stage = (CtrlStage)(stage + 1);
      break;
    case CtrlStageSetupFinished:
      g_ctrl_state.stage = CtrlStageSetupTemperature;
      break;
    default:
      break;
  }
}

static void CtrlpScheduleButtonHandler(CtrlButton button) {
  if (!READ_BIT(TIM14->CR1, TIM_CR1_CEN)) {
    g_ctrl_state.scheduled_button = button;
    SET_BIT(TIM14->CR1, TIM_CR1_CEN);
  }
}

TpStatus CtrlInitialize(void) {
  g_ctrl_state.temperature_point = TmTemperatureMin;

  CtrlpPrepareGpio();
  CtrlpSetupTimer();
  CtrlpSetupKeyInterrupts();

  return TpSuccess;
}

TpStatus CtrlProcessRequests(void) {
  FixedPoint16 temperature;
  switch (g_ctrl_state.stage) {  // NOLINT(clang-diagnostic-switch-enum)
    case CtrlStageInitialized:
      SsiSetState(true);
      g_ctrl_state.stage = CtrlStageSetupFinished;
      break;
    case CtrlStageSetupTemperature:
      SsiSetNumber(g_ctrl_state.temperature_point);
      break;
    case CtrlStageSetupMode:
      SsiSetValue(g_ctrl_state.mode_label[g_ctrl_state.mode]);
      break;
    case CtrlStageSetupFinished:
      if (TP_SUCCESS(TmQueryTemperature(&temperature))) {
        SsiSetNumber(temperature);
      } else {
        g_ctrl_state.stage = CtrlStageError;
      }
      break;
    case CtrlStageError:
      SsiSetValue(g_ctrl_state.error_msg);
      break;
    default:
      break;
  }
  return TpSuccess;
}

void EXTI0_1_IRQHandler(void) {
  SET_BIT(EXTI->PR, EXTI_PR_PR0);
  CtrlpScheduleButtonHandler(CtrlButtonUser);
}

void EXTI4_15_IRQHandler(void) {
  const uint32_t interrupt_mask = EXTI->PR;
  if (READ_BIT(interrupt_mask, EXTI_PR_PR11)) {
    EXTI->PR = EXTI_PR_PR11;
    CtrlpScheduleButtonHandler(CtrlButtonUp);
  } else if (READ_BIT(interrupt_mask, EXTI_PR_PR12)) {
    EXTI->PR = EXTI_PR_PR12;
    CtrlpScheduleButtonHandler(CtrlButtonDown);
  }
}

void TIM14_IRQHandler(void) {
  CLEAR_BIT(TIM14->SR, TIM_SR_UIF);

  const CtrlButton button = g_ctrl_state.scheduled_button;
  if (button == CtrlButtonUser) {
    CtrlpNextMode();
  } else {
    const ctrl_button_handler_t handler =
        g_ctrl_state.button_handler[g_ctrl_state.stage];
    handler(button);
  }
}