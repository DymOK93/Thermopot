#include "control.h"

#include <ssi/ssi.h>
#include <thermal/thermal.h>
#include <tools/break_on.h>
#include <tools/fixed_point.h>
#include <tools/utils.h>

#include <stdbool.h>

#include <stm32f0xx.h>

#define CTRL_TEMPERATURE_STEP 1
#define CTRL_BUTTON_INTERRUPT_PRIORITY 3
#define CTRL_TIMER_INTERRUPT_PRIORITY 3
#define CTRL_BUTTON_DEBOUNCE_DELAY 50  // Milliseconds

#define CTRL_DEFAULT_MODE TmModePid
#define CTRL_DEFAULT_TEMPERATURE_POINT TmTemperatureMax

typedef enum {
  CtrlStageError = -2,
  CtrlStageInitialized = -1,
  CtrlStageSetupMode,
  CtrlStageSetupTemperature,
  CtrlStageHeat,
  CtrlStageCount
} CtrlStage;

#define CTRL_SETUP_STAGE_COUNT (CtrlStageHeat - (CtrlStageInitialized + 1))

typedef enum {
  CtrlButtonUser,
  CtrlButtonUp,
  CtrlButtonDown,
  CtrlButtonCount
} CtrlButton;

typedef void (*ctrl_button_handler_t)(CtrlButton);
static void CtrlpSetupMode(CtrlButton key);
static void CtrlpSetupTemperature(CtrlButton key);

// NOLINTNEXTLINE(clang-diagnostic-padded)
typedef struct {
  TmMode mode;
  FixedPoint16 temperature_point;
} CtrlSettings;

// NOLINTNEXTLINE(clang-diagnostic-padded)
typedef struct {
  const ctrl_button_handler_t button_handler[CTRL_SETUP_STAGE_COUNT];
  volatile CtrlStage stage;
  volatile CtrlButton scheduled_button;
  volatile CtrlSettings settings;
  const FixedPoint16 temperature_step;
  const SsiValue mode_label[TmModeCount];
  const SsiValue init_msg;
  const SsiValue wait_msg;
  const SsiValue error_msg;
} CtrlState;

static CtrlState g_ctrl_state = {
    .button_handler = {&CtrlpSetupMode, &CtrlpSetupTemperature},
    .stage = CtrlStageInitialized,
    .scheduled_button = CtrlButtonCount,
    .temperature_step = Fp16Initialize(CTRL_TEMPERATURE_STEP, 0),
    .mode_label = {{"ler"},   // "rel"
                   {"dip"}},  // "pid"
    .init_msg = {"tini"},     // "init"
    .wait_msg = {"tiaw"},     // "wait"
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

static void CtrlpSetupKeyInterrupts(void) {
  /**
   * 1. Enable interrupts on pins PA0 (user button), PA11 and PA12 (up/down)
   * 2. Operation mode: raising edge for user button, failing edge for up/down
   * 3. Button interrupt priority must be lower than temperature sensor timer
   * interrupt priority (i.e. higher in absolute priority value)
   */
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
  SET_BIT(EXTI->IMR, EXTI_IMR_MR0 | EXTI_IMR_MR11 | EXTI_IMR_MR12);  // (1)
  SET_BIT(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI0_PA);
  SET_BIT(SYSCFG->EXTICR[2], SYSCFG_EXTICR3_EXTI11_PA);
  SET_BIT(SYSCFG->EXTICR[3], SYSCFG_EXTICR4_EXTI12_PA);
  SET_BIT(EXTI->RTSR, EXTI_RTSR_TR0);  // (2)
  SET_BIT(EXTI->FTSR, EXTI_FTSR_TR11 | EXTI_FTSR_TR12);
  NVIC_SetPriority(EXTI0_1_IRQn, CTRL_BUTTON_INTERRUPT_PRIORITY);  // (3)
  NVIC_SetPriority(EXTI4_15_IRQn, CTRL_BUTTON_INTERRUPT_PRIORITY);
  NVIC_EnableIRQ(EXTI0_1_IRQn);
  NVIC_EnableIRQ(EXTI4_15_IRQn);
}

static void CtrlpUpdateDefaults(void) {
  g_ctrl_state.settings.mode = CTRL_DEFAULT_MODE;
  g_ctrl_state.settings.temperature_point = CTRL_DEFAULT_TEMPERATURE_POINT;
}

static void CtrlpSetupMode(CtrlButton button) {
  /*
   * It doesn't matter which button was clicked as there are only 2 possible
   * values
   */
  (void)button;
  volatile TmMode* mode = &g_ctrl_state.settings.mode;
  if (*mode == TmModeRelay) {
    *mode = TmModePid;
  } else {
    *mode = TmModeRelay;
  }
}

static void CtrlpSetupTemperature(CtrlButton button) {
  volatile CtrlSettings* settings = &g_ctrl_state.settings;
  FixedPoint16 temperature_point;

  if (button == CtrlButtonUp) {
    Fp16Add(temperature_point, settings->temperature_point,
            g_ctrl_state.temperature_step);
    if (Fp16LessEqual(temperature_point, TmTemperatureMax)) {
      settings->temperature_point = temperature_point;
    }
  } else if (button == CtrlButtonDown) {
    Fp16Sub(temperature_point, settings->temperature_point,
            g_ctrl_state.temperature_step);
    if (Fp16GreaterEqual(temperature_point, TmTemperatureMin)) {
      settings->temperature_point = temperature_point;
    }
  }
}

static bool CtrpSetupInProgress(void) {
  const CtrlStage stage = g_ctrl_state.stage;
  return stage > CtrlStageInitialized && stage < CtrlStageHeat;
}

static CtrlStage CtrlpConfigureTm(void) {
  const volatile CtrlSettings* settings = &g_ctrl_state.settings;

  TmSetState(false);
  if (!TP_SUCCESS(TmSetup(settings->mode, settings->temperature_point))) {
    return CtrlStageError;
  }
  TmSetState(true);

  return CtrlStageHeat;
}

static void CtrlpNextMode(void) {
  const CtrlStage stage = g_ctrl_state.stage;
  switch (stage) {  // NOLINT(clang-diagnostic-switch-enum)
    case CtrlStageSetupTemperature:
      g_ctrl_state.stage = CtrlpConfigureTm();
      break;
    case CtrlStageHeat:
      g_ctrl_state.stage = CtrlStageSetupMode;
      break;
    default:
      g_ctrl_state.stage = (CtrlStage)(stage + 1);
      break;
  }
}

static bool CtrpCheckButtonPressed(CtrlButton button) {
  bool pressed = false;
  switch (button) {  // NOLINT(clang-diagnostic-switch-enum)
    case CtrlButtonUser:
      pressed = READ_BIT(GPIOA->IDR, GPIO_IDR_0);
      break;
    case CtrlButtonUp:
      pressed = READ_BIT(GPIOA->IDR, GPIO_IDR_11) == 0;
      break;
    case CtrlButtonDown:
      pressed = READ_BIT(GPIOA->IDR, GPIO_IDR_12) == 0;
      break;
    default:
      break;
  }
  return pressed;
}

static void CtrlpScheduleButtonHandler(CtrlButton button) {
  if (!READ_BIT(TIM14->CR1, TIM_CR1_CEN)) {
    g_ctrl_state.scheduled_button = button;
    SET_BIT(TIM14->CR1, TIM_CR1_CEN);
  }
}

static void CtrlpDisplayCurrentTemperature(void) {
  FixedPoint16 temperature;

  do {
    TpStatus status = TmQueryTemperature(&temperature, false);
    if (status == TpNotReady || status == TpPending) {
      status = SsiSetValue(g_ctrl_state.wait_msg);
      BREAK_ON_ERROR(status);

      status = TmQueryTemperature(&temperature, true);
    }
    BREAK_ON_ERROR(status);

    status = SsiSetNumber(temperature);
    BREAK_ON_ERROR(status);

    return;

  } while (false);

  TmSetState(false);
  g_ctrl_state.stage = CtrlStageError;
}

TpStatus CtrlInitialize(void) {
  CtrlpPrepareGpio();
  CtrlpSetupTimer();
  CtrlpSetupKeyInterrupts();
  CtrlpUpdateDefaults();
  return TpSuccess;
}

TpStatus CtrlProcessRequests(void) {
  switch (g_ctrl_state.stage) {  // NOLINT(clang-diagnostic-switch-enum)
    case CtrlStageInitialized:
      SsiSetState(true);
      SsiSetValue(g_ctrl_state.init_msg);
      break;
    case CtrlStageSetupTemperature:
      SsiSetNumber(g_ctrl_state.settings.temperature_point);
      break;
    case CtrlStageSetupMode:
      SsiSetValue(g_ctrl_state.mode_label[g_ctrl_state.settings.mode]);
      break;
    case CtrlStageHeat:
      CtrlpDisplayCurrentTemperature();
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
  if (!CtrpCheckButtonPressed(button)) {
    return;
  }

  if (button == CtrlButtonUser) {
    CtrlpNextMode();
  } else if (CtrpSetupInProgress()) {
    const ctrl_button_handler_t handler =
        g_ctrl_state.button_handler[g_ctrl_state.stage];
    handler(button);
  }
}
