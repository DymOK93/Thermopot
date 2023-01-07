/**
 * @file
 * @brief Thermal Manager implementation
 *
 */
#include "thermal.h"
#include "ds18b20.h"

#include <heat/heat.h>
#include <tools/break_on.h>
#include <tools/utils.h>

#include <stm32f0xx.h>

/**
 * Relay control hysteresis window: turn on the heater at (temperature_point
 * - 2.5C), turn off the heater at (temperature_point - 2C) taking into account
 * thermal inertia
 */
#define TM_TEMPERATURE_HYSTERESIS_UP (-2)
#define TM_TEMPERATURE_HYSTERESIS_DOWN_WHOLE 2
#define TM_TEMPERATURE_HYSTERESIS_DOWN_FRACTIONAL 5

#define TM_TEMPERATURE_MIN 0    // Minimum supported temperature point (5.0)
#define TM_TEMPERATURE_MAX 100  // Maximum supported temperature point  (100.0)

#define TM_MODE_INACTIVE TmModeCount  //!< Initial mode

#define TM_TEMPERATURE_SAMPLING_PERIOD \
  100  //!< Temperature sampling period in ms
#define TM_TIMER_INTERRUPT_PRIORITY 2

/**
 * PID (technically PI) regulator settings
 */
#define TM_PID_SAMPLING_PERIOD (TM_TEMPERATURE_SAMPLING_PERIOD / 1000.0f)
#define TM_PID_P_FACTOR (5.5f)  //!< Proportional coefficient
#define TM_PID_I_FACTOR 1       //!< Integral coefficient

/**
 * USART debug interface settings
 */
#define TM_DEBUG_INTERFACE_INTERRUPT_PRIORITY 3
#define TM_DEBUG_INTERFACE_TRANSFER_BAUD_RATE 115200

typedef void (*tm_heater_handler_t)(void);
static void TmpRelayHeaterHandler(void);
static void TmpPidHeaterHandler(void);

typedef void (*tm_setup_handler_t)(FixedPoint16);
static void TmpRelaySetupHandler(FixedPoint16 temperature_point);
static void TmpPidSetupHandler(FixedPoint16 temperature_point);

/**
 * @struct TmRelayState
 * @brief Relay controller state
 */
typedef struct {
  FixedPoint16 turn_off_temperature;  //!< Turn off temperature
  FixedPoint16 turn_on_temperature;   //!< Turn on temperature
} TmRelayState;

/**
 * @struct TmRelaySettings
 * @brief Relay controller configuration data
 */
// clang-format off
typedef struct {
  FixedPoint16 temperature_hysteresis_up; //!< Upper limit of the hysteresis window
  FixedPoint16 temperature_hysteresis_down; //!< Lower limit of the hysteresis window
} TmRelaySettings;
// clang-format on

/**
 * @struct TmPidState
 * @brief PID controller state
 */
typedef struct {
  float temperature_point;
  float accumulated_difference;
} TmPidState;

/**
 * @struct TmPidSettings
 * @brief PID controller configuration data
 */
typedef struct {
  float p_factor;
  float i_factor;
} TmPidSettings;

/**
 * @struct TmSettings
 * @brief Configuration data sent via the debug interface
 */
typedef struct {
  TmMode type;  //!< Target controller type
  union {
    TmRelaySettings relay_settings;  //!< Relay controller settings
    TmPidSettings pid_settings;      //!< PID controller settings
  };
} TmSettings;

/**
 * @struct TmState
 * @brief Thermal Manager global state
 */
// clang-format off
typedef struct {
  volatile TpStatus status;  //!< Current status of Thermal Manager
  volatile TmMode mode;      //!< Active temperature control mode (controller type)
  const tm_heater_handler_t heater_handlers[TmModeCount]; //!< Controllers' temperature changing & maintaining handlers
  const tm_setup_handler_t setup_handlers[TmModeCount];   //!< Controllers' temperature point setup handlers
  volatile FixedPoint16 current_temperature;              //!< Last read temperature

  // NOLINTNEXTLINE(clang-diagnostic-padded)
  union {
    TmRelayState relay_state;               //!< Relay controller state
    TmPidState pid_state;                   //!< PID controller state
  };

  volatile TmRelaySettings relay_settings;  //!< Relay controller settings
  volatile TmPidSettings pid_settings;      //!< PID controller settings
  TmSettings settings;                      //!< Debug interface DMA buffer
} TmState;
// clang-format on

const FixedPoint16 TmTemperatureMin = Fp16Initialize(TM_TEMPERATURE_MIN, 0);
const FixedPoint16 TmTemperatureMax = Fp16Initialize(TM_TEMPERATURE_MAX, 0);

static TmState g_tm_state = {
    .status = TpNotReady,
    .mode = TM_MODE_INACTIVE,
    .heater_handlers = {&TmpRelayHeaterHandler, &TmpPidHeaterHandler},
    .setup_handlers = {&TmpRelaySetupHandler, &TmpPidSetupHandler},
    .relay_settings = {.temperature_hysteresis_up =
                           Fp16Initialize(TM_TEMPERATURE_HYSTERESIS_UP, 0),
                       .temperature_hysteresis_down = Fp16Initialize(
                           TM_TEMPERATURE_HYSTERESIS_DOWN_WHOLE,
                           TM_TEMPERATURE_HYSTERESIS_DOWN_FRACTIONAL)},
    .pid_settings = {.p_factor = TM_PID_P_FACTOR, .i_factor = TM_PID_I_FACTOR}};

/**
 * @brief Configures GPIO pins for 1-Wire digital thermal sensor (DS18B20) and
 * USART debug interface
 */
static void TmpPrepareGpio(void) {
  /**
   * 1. Activate PA10 in the alternative function mode
   * 2. Setup weak pull-up on PA10
   * 3. Select AF1 (USARTx_RX) for PA10 (3)
   */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
  SET_BIT(GPIOA->MODER, GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1);  // (1)
  SET_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR10_0);  // (2)
  SET_BIT(GPIOA->AFR[1], 0x00000110);                                 // (3)
}

/**
 * @brief Configures the DS18B20 polling timer
 */
static void TmpSetupTimer() {
  /**
   * 1. Tick period is 1ms
   * 2. Enable update interrupt
   * 3. Timer interrupt priority must be higher than user input interrupt
   * handlers (i.e. less absolute value)
   */
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM16EN);
  TIM16->PSC = (uint16_t)(SystemCoreClock / 1000 - 1);  // (1)
  TIM16->ARR = TM_TEMPERATURE_SAMPLING_PERIOD;

  SET_BIT(TIM16->DIER, TIM_DIER_UIE);  // (2)
  NVIC_EnableIRQ(TIM16_IRQn);
  NVIC_SetPriority(TIM16_IRQn, TM_TIMER_INTERRUPT_PRIORITY);  // (4)
}

/**
 * @brief Configures the USART transceiver for debug interface
 */
static void TmpSetupDebugInterface(void) {
  /**
   * 1. Setup DMA peripheral-to-memory in circular mode
   * 2. Enable DMA interrupt
   * 3. Configure USART as receiver only
   */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_DMAEN);  // (1)
  SET_BIT(DMA1_Channel3->CCR,
          DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TEIE | DMA_CCR_TCIE);
  DMA1_Channel3->CPAR = (uint32_t)&USART1->RDR;
  DMA1_Channel3->CNDTR = sizeof(TmSettings);
  DMA1_Channel3->CMAR = (uint32_t)&g_tm_state.settings;
  SET_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);

  NVIC_SetPriority(DMA1_Channel2_3_IRQn, TM_DEBUG_INTERFACE_INTERRUPT_PRIORITY);
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);  // (2)

  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
  USART1->BRR = SystemCoreClock / TM_DEBUG_INTERFACE_TRANSFER_BAUD_RATE;
  SET_BIT(USART1->CR3, USART_CR3_DMAR);
  SET_BIT(USART1->CR1, USART_CR1_RE | USART_CR1_UE);  // (3)
}

/**
 * @brief Relay controller temperature changing & maintaining handler
 * @warning Called from a timer interrupt routine
 */
static void TmpRelayHeaterHandler(void) {
  /**
   * 1. If the temperature is above or equal to the upper limit of the
   * hysteresis window, turn off the heater
   * 2. If the temperature is below or equal to the lower limit of the
   * hysteresis window, turn on the heater
   */
  const FixedPoint16 current_temperature = g_tm_state.current_temperature;
  if (Fp16GreaterEqual(current_temperature,
                       g_tm_state.relay_state.turn_off_temperature)) {
    HmSetPowerFactor(HM_POWER_FACTOR_MIN);  // (1)
  } else if (Fp16LessEqual(current_temperature,
                           g_tm_state.relay_state.turn_on_temperature)) {
    HmSetPowerFactor(HM_POWER_FACTOR_MAX);  // (2)
  }
}

/**
 * @brief Converts 16-bit fixed point value to 32-bit floating point value
 * @param[in] value Fixed point value
 * @return Floating point value
 */
static float TmpConvertToFloat(FixedPoint16 value) {
  return (float)Fp16ReadAsNumber(value) / (1 << FP16_FRACTIONAL_BITS);
}

/**
 * @brief Calculates and sets the power factor via the Heater Manager interface
 * @warning Called from a timer interrupt routine
 * @param[in] difference Difference between temperature point and current
 * temperature
 * @param[in] accumulated_difference Cumulative (integral) temperature
 * difference
 * @remark Calculated value is limited to [HM_POWER_FACTOR_MIN,
 * HM_POWER_FACTOR_MAX]
 */
static void TmpPidSetPowerFactor(float difference,
                                 float accumulated_difference) {
  const float result =
      difference * TM_PID_P_FACTOR + accumulated_difference * TM_PID_I_FACTOR;

  int32_t power_factor = (int32_t)result;

  if (power_factor > HM_POWER_FACTOR_MAX) {
    power_factor = HM_POWER_FACTOR_MAX;
  } else if (power_factor < HM_POWER_FACTOR_MIN) {
    power_factor = HM_POWER_FACTOR_MIN;
  }

  HmSetPowerFactor((uint8_t)power_factor);
}

/**
 * @brief PID controller temperature changing & maintaining handler
 * @warning Called from a timer interrupt routine
 */
static void TmpPidHeaterHandler(void) {
  const float difference = g_tm_state.pid_state.temperature_point -
                           TmpConvertToFloat(g_tm_state.current_temperature);

  const float accumulated_difference =
      g_tm_state.pid_state.accumulated_difference;
  g_tm_state.pid_state.accumulated_difference +=
      difference * TM_PID_SAMPLING_PERIOD;

  TmpPidSetPowerFactor(difference, accumulated_difference);
}

/**
 * @brief Relay controller setup handler
 * @param[in] temperature_point Temperature point in fixed point format
 */
static void TmpRelaySetupHandler(FixedPoint16 temperature_point) {
  Fp16Add(g_tm_state.relay_state.turn_off_temperature, temperature_point,
          g_tm_state.relay_settings.temperature_hysteresis_up);
  Fp16Sub(g_tm_state.relay_state.turn_on_temperature, temperature_point,
          g_tm_state.relay_settings.temperature_hysteresis_down);
}

/**
 * @brief PID controller setup handler
 * @param[in] temperature_point Temperature point in fixed point format
 */
static void TmpPidSetupHandler(FixedPoint16 temperature_point) {
  g_tm_state.pid_state.temperature_point = TmpConvertToFloat(temperature_point);
  g_tm_state.pid_state.accumulated_difference = 0;
}

/**
 * @brief Starts polling the DS18B20 and interacting with the Heater Manager
 */
static void TmpStart(void) {
  g_tm_state.status = TpNotReady;
  SET_BIT(USART2->CR1, USART_CR1_UE);
  SET_BIT(TIM16->CR1, TIM_CR1_CEN);
}

/**
 * @brief Stops polling the DS18B20 and turns off the heater
 */
static void TmpStop(void) {
  CLEAR_BIT(TIM16->CR1, TIM_CR1_CEN);
  CLEAR_BIT(USART2->CR1, USART_CR1_UE);
  HmSetPowerFactor(HM_POWER_FACTOR_MIN);
}

/**
 * @brief Validates settings
 * @param[in] mode Temperature control mode
 * @param[in] temperature_point Temperature point in fixed point format
 * @return
 *    - true if mode is valid and (TmTemperatureMin <= temperature_point <=
 * TmTemperatureMax)
 *    - false otherwise
 */
static bool TmpValidateSettings(TmMode mode, FixedPoint16 temperature_point) {
  return mode < TmModeCount &&
         Fp16GreaterEqual(temperature_point, TmTemperatureMin) &&
         Fp16LessEqual(temperature_point, TmTemperatureMax);
}

static TpStatus TmpSetup(TmMode mode, FixedPoint16 value) {
  g_tm_state.mode = mode;
  const tm_setup_handler_t setup_handler = g_tm_state.setup_handlers[mode];
  setup_handler(value);
  return TpSuccess;
}

/**
 * @brief Sets the status of no communication with the temperature sensor and
 * stops the Thermal Manager with forced shutdown of the heater
 * @warning Called from a timer interrupt routine
 */
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

/**
 * @brief Configures DS18B20 and starts initial temperature conversion
 * @warning Called from a timer interrupt routine
 * @warning Raises communication error if temperature sensor not found
 */
static void TmpStartMeasurement(void) {
  /**
   * 1. Set 9-bit resolution of DS18B20
   * 2. Start temperature conversion
   * @see
   * https://datasheet.lcsc.com/szlcsc/1912111437_UMW-Youtai-Semiconductor-Co-Ltd-DS18B20_C376006.pdf
   * (in particular - conversion time table)
   */
  do {
    TpStatus status = DsPrepare(DsResolution9Bit);  // (1)
    BREAK_ON_ERROR(status);

    status = DsConvertTemperature();  // (2)
    BREAK_ON_ERROR(status);

    g_tm_state.status = TpPending;
    return;

  } while (false);

  TmpRaiseDeviceError(); 
}

/**
 * @brief Starts temperature conversion
 * @warning Called from a timer interrupt routine
 * @warning Raises communication error if temperature sensor not found
 */
static void TmpContinueMeasurement(void) {
  if (!TP_SUCCESS(DsConvertTemperature())) {
    TmpRaiseDeviceError();
  } else {
    g_tm_state.status = TpSuccess;
  }
}

/**
 * @brief Updates the controller configuration
 * @warning Called from a DMA interrupt routine
 */
static void TmpUpdateSettings(void) {
  const TmSettings* settings = &g_tm_state.settings;
  if (settings->type == TmModePid) {
    g_tm_state.pid_settings = settings->pid_settings;
  } else if (settings->type == TmModeRelay) {
    g_tm_state.relay_settings = settings->relay_settings;
  }
}

TpStatus TmInitialize(void) {
  TmpPrepareGpio();
  TmpSetupTimer();
  TmpSetupDebugInterface();
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

/**
 * @brief Reads the DS18B20 measurements and executes the algorithm for changing
 * and maintaining the temperature for the current controller type
 * @remark Timer interrupt routine
 */
void TIM16_IRQHandler(void) {
  CLEAR_BIT(TIM16->SR, TIM_SR_UIF);

  if (!TP_SUCCESS(g_tm_state.status)) {
    TmpStartMeasurement();

  } else {
    FixedPoint16 temperature;
    const TpStatus status = DsReadTemperature(&temperature);
    if (!TP_SUCCESS(status)) {
      TmpRaiseDeviceError();
    } else {
      g_tm_state.current_temperature = temperature;

      const tm_heater_handler_t heater_handler =
          g_tm_state.heater_handlers[g_tm_state.mode];
      heater_handler();

      TmpContinueMeasurement();
    }
  }
}

/**
 * @brief Applies settings passed through the debug interface
 * @remark DMA interrupt routine
 */
void DMA1_Channel2_3_IRQHandler(void) {
  const uint32_t status = DMA1->ISR;
  if (READ_BIT(status, DMA_ISR_TEIF3)) {
    DMA1->IFCR = DMA_IFCR_CTEIF3;
  } else if (READ_BIT(status, DMA_ISR_TCIF3)) {
    DMA1->IFCR = DMA_IFCR_CTCIF3;
    TmpUpdateSettings();
  }
}
