#include "control.h"

#include <ssi/ssi.h>
#include <thermal/thermal.h>
#include <tools/fixed_point.h>
#include <tools/utils.h>

#include <stm32f0xx.h>

#define CTRL_TEMPERATURE_STEP 1
#define CTRL_KEY_INTERRUPT_PRIORITY 3

typedef enum {
  CtrlStageSetupTemperature,
  CtrlStageSetupMode,
  CtrlStageSetupFinished,
  CtrlStageError,
  CtrlStageCount
} CtrlStage;

typedef enum { CtrlSetupKeyUp, CtrlSetupKeyDown } CtrlSetupKey;

typedef void (*ctrl_setup_key_handler_t)(CtrlSetupKey);
static void CtrlpSetupTemperature(CtrlSetupKey key);
static void CtrlpSetupMode(CtrlSetupKey key);
static void CtrlpDummyKeyHandler(CtrlSetupKey key);

typedef struct {
  ctrl_setup_key_handler_t setup_key_handler[CtrlStageCount];
  CtrlStage stage;
  TmMode mode;
  FixedPoint16 temperature_point;
  const FixedPoint16 temperature_step;
  const SsiValue mode_label[TmModeCount];
  const SsiValue error_msg;
} CtrlState;

static CtrlState g_ctrl_state = {
    .setup_key_handler = {&CtrlpSetupTemperature, &CtrlpSetupMode,
                          &CtrlpDummyKeyHandler, &CtrlpDummyKeyHandler},
    .stage = CtrlStageSetupFinished,
    .mode = TmModeRelay,
    .temperature_step = {CTRL_TEMPERATURE_STEP, 0},
    .mode_label = {{"rel"}, {"pid"}},
    .error_msg = {"err"}};

static void CtrlpPrepareGpio(void) {
  /*
   * Configure GPIOs as weak pull-up inputs
   */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
  SET_BIT(GPIOA->PUPDR,
          GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR11_0 | GPIO_PUPDR_PUPDR12_0);
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
  NVIC_SetPriority(EXTI0_1_IRQn, CTRL_KEY_INTERRUPT_PRIORITY);           // (3)
  NVIC_SetPriority(EXTI4_15_IRQn, CTRL_KEY_INTERRUPT_PRIORITY);
  NVIC_EnableIRQ(EXTI0_1_IRQn);
  NVIC_EnableIRQ(EXTI4_15_IRQn);
}

static void CtrlpSetupTemperature(CtrlSetupKey key) {
  if (key == CtrlSetupKeyUp &&
      FpLess(g_ctrl_state.temperature_point, TmTemperatureMax)) {
    FpAdd(g_ctrl_state.temperature_point, g_ctrl_state.temperature_step,
          &g_ctrl_state.temperature_point);
  } else if (key == CtrlSetupKeyDown &&
             FpGreater(g_ctrl_state.temperature_point, TmTemperatureMin)) {
    FpSub(g_ctrl_state.temperature_point, g_ctrl_state.temperature_step,
          &g_ctrl_state.temperature_point);
  }
}

static void CtrlpSetupMode(CtrlSetupKey key) {
  /*
   * It doesn't matter which button was clicked as there are only 2 possible
   * values
   */
  (void)key;
  g_ctrl_state.mode =
      g_ctrl_state.mode == TmModeRelay ? TmModePid : TmModeRelay;
}

static void CtrlpDummyKeyHandler(CtrlSetupKey key) {
  /*
   * Do nothing - setup is finished or not started yet
   */
  (void)key;
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

TpStatus CtrlInitialize(void) {
  g_ctrl_state.temperature_point = TmTemperatureMin;

  CtrlpPrepareGpio();
  CtrlpSetupKeyInterrupts();

  return TpSuccess;
}

TpStatus CtrlProcessRequests(void) {
  const FixedPoint16 temperature = {0, 0};
  switch (g_ctrl_state.stage) {  // NOLINT(clang-diagnostic-switch-enum)
    case CtrlStageSetupTemperature:
      SsiSetNumber(g_ctrl_state.temperature_point);
      break;
    case CtrlStageSetupMode:
      SsiSetValue(g_ctrl_state.mode_label[g_ctrl_state.mode]);
      break;
    case CtrlStageSetupFinished:
      /*if (TP_SUCCESS(TmQueryTemperature(&temperature))) {
        SsiSetNumber(temperature);
      } else {
        g_ctrl_state.stage = CtrlStageError;
      }*/
      SsiSetNumber(temperature);
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
  CtrlpNextMode();
}

void EXTI4_15_IRQHandler(void) {
  const ctrl_setup_key_handler_t handler =
      g_ctrl_state.setup_key_handler[g_ctrl_state.stage];

  if (READ_BIT(EXTI->PR, EXTI_PR_PR11)) {
    SET_BIT(EXTI->PR, EXTI_PR_PR11);
    handler(CtrlSetupKeyUp);
  } else if (READ_BIT(EXTI->PR, EXTI_PR_PR12)) {
    SET_BIT(EXTI->PR, EXTI_PR_PR12);
    handler(CtrlSetupKeyDown);
  }
}