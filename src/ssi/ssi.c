#include "ssi.h"

#include <tools/utils.h>

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include <stm32f0xx.h>

#define SSI_FP_INDEX 1
#define SSI_FP_DOT 0x80

typedef struct {
  SsiValue value;
  bool is_number;
  uint8_t idx;
  const FixedPoint16 min_number;
  const FixedPoint16 max_number;
} SsiState;

static SsiState g_ssi_state = {.value = {{0}},
                               .is_number = false,
                               .idx = 0,
                               .min_number = {SSI_NUMBER_MIN, 0},
                               .max_number = {SSI_NUMBER_MAX, 0}};

static void SsipPrepareGpio(void) {
  /**
   * 1. Activate PA5, PA7 in the alternative function mode; AF0
   * (SPI1_SCK/SPI1_MOSI) already selected
   */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
  SET_BIT(GPIOA->MODER, GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1);  // (1)
}

static void SsipSetupSpi(void) {
  /**
   * 1. The maximum frequency of 74HC595 is about 4MHz, the maximum frequency of
   * AHB and APB2 is 48MHz, so set SCK as APB2_CLK/16
   */
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);
  SET_BIT(SPI1->CR1, SPI_CR1_BR_0 | SPI_CR1_BR_1);  // (1)

}

static void SsipSetupTimer(void) {
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM6EN);
  TIM6->ARR = 2;  // 500 Hz / 4 => 125 FPS
  SET_BIT(TIM6->DIER, TIM_DIER_UIE);
  NVIC_SetPriority(TIM6_DAC_IRQn, 3);
}

static void SsipStart() {
  TIM6->CNT = 0;
  TIM6->PSC = (uint16_t)(SystemCoreClock / 1000 - 1);
  NVIC_EnableIRQ(TIM6_DAC_IRQn);
  SET_BIT(TIM6->CR1, TIM_CR1_CEN);
}

static void SsipStop(void) {
  CLEAR_BIT(TIM6->CR1, TIM_CR1_CEN);
  NVIC_DisableIRQ(TIM6_DAC_IRQn);
}

static bool SsipValidateValue(SsiValue value) {
  uint8_t idx = 0;
  for (; idx < SSI_PANEL_SIZE; ++idx) {
    const char ch = value.str[idx];
    const bool valid = IS_NULL_TERMINATOR(ch) || IS_SPACE(ch) || IS_DASH(ch) ||
                       IS_DIGIT(ch) || IS_LETTER(ch);
    if (!valid) {
      return false;
    }
  }
  return true;
}

static TpStatus SsipSetValue(SsiValue value, bool is_number) {
  if (!SsipValidateValue(value)) {
    return TpInvalidParameter;
  }
  g_ssi_state.value = value;
  g_ssi_state.is_number = is_number;
  return TpSuccess;
}

static SsiValue SsipParseNumber(FixedPoint16 number) {
  uint16_t unsigned_number = (uint16_t)abs(number.whole);
  uint8_t idx = 1;
  SsiValue value = {{number.fractional ? '5' : '0', 0, 0, 0}};

  if (unsigned_number == 0) {
    value.str[idx] = '0';
  } else {
    for (; unsigned_number != 0; ++idx) {
      value.str[idx] = (char)('0' + unsigned_number % 10);
      unsigned_number /= 10;
    }
    if (number.whole < 0) {
      value.str[idx] = '-';
    }
  }
  return value;
}

static uint8_t SsipGetSegmentMask(char value) {
  // TODO

  uint8_t mask;
  switch (value) {
    case '\0':
    case ' ':
      mask = 0x0;  // 0b0000000
      break;
    case '-':
      mask = 0x40;  // 0b1000000
      break;
    case '0':
      mask = 0x3F;  // 0b0111111
      break;
    case '1':
      mask = 0x6;  // 0b0000110
      break;
    case '2':
      mask = 0x5B;  // 0b1011011
      break;
    case '3':
      mask = 0x4F;  // 0b1001111
      break;
    case '4':
      mask = 0x66;  // 0b1100110
      break;
    case '5':
      mask = 0x6D;  // 0b1111101
      break;
    case '6':
      mask = 0x7D;  // 0b1111101
      break;
    case '7':
      mask = 0x7;  // 0b0000111
      break;
    case '8':
      mask = 0x7F;  // 0b1111111
      break;
    case '9':
      mask = 0x6F;  // 0b1101111
      break;
    case 'a':
    case 'A':
    case 'b':
    case 'B':
    case 'c':
    case 'C':
    case 'd':
    case 'D':
    case 'e':
    case 'E':
    case 'f':
    case 'F':
    case 'g':
    case 'G':
    case 'h':
    case 'H':
    case 'i':
    case 'I':
    case 'j':
    case 'J':
    case 'k':
    case 'K':
    case 'l':
    case 'L':
    case 'm':
    case 'M':
    case 'n':
    case 'N':
    case 'p':
    case 'P':
    case 'q':
    case 'Q':
    case 'r':
    case 'R':
    case 's':
    case 'S':
    case 't':
    case 'T':
    case 'u':
    case 'U':
    case 'v':
    case 'V':
    case 'w':
    case 'W':
    case 'x':
    case 'X':
    case 'y':
    case 'Y':
    case 'z':
    case 'Z':
    default:
      mask = 0x0;  // 0b0000000
      break;
  }
  return mask;
}

static void SsipDrawSegment(uint8_t segment_mask) {
  while (!READ_BIT(SPI1->SR, SPI_SR_TXE))
    ;
  SPI1->DR = segment_mask;
}

static void SsipActivateSegment(uint8_t idx) {
  // TODO
  if (idx == 0) {
  } else if (idx == 1) {
  } else if (idx == 2) {
  } else if (idx == 3) {
  } else {
    // Fixed-point dot
  }
}

TpStatus SsiInitialize(void) {
  SsipPrepareGpio();
  SsipSetupSpi();
  SsipSetupTimer();
  return TpSuccess;
}

void SsiSetState(bool enable) {
  if (!enable) {
    SsipStop();
  } else {
    SsipStart();
  }
}

SsiValue SsiGetValue(void) {
  return g_ssi_state.value;
}

TpStatus SsiSetValue(SsiValue value) {
  return SsipSetValue(value, false);
}

TpStatus SsiSetNumber(FixedPoint16 number) {
  if (FpLess(number, g_ssi_state.min_number) ||
      FpGreater(number, g_ssi_state.max_number)) {
    return TpInvalidParameter;
  }
  return SsipSetValue(SsipParseNumber(number), true);
}

void TIM6_DAC_IRQHandler(void) {
  CLEAR_BIT(TIM6->SR, TIM_SR_UIF);

  const uint8_t idx = g_ssi_state.idx;
  uint8_t segment_mask = SsipGetSegmentMask(g_ssi_state.value.str[idx]);
  if (g_ssi_state.is_number && idx == SSI_FP_INDEX) {
    SET_BIT(segment_mask, SSI_FP_DOT);
  }
  SsipDrawSegment(segment_mask);
}

void SPI1_IRQHandler(void) {

  const uint8_t idx = g_ssi_state.idx;
  SsipActivateSegment(idx);
  g_ssi_state.idx = idx + 1 == SSI_PANEL_SIZE ? 0 : idx + 1;
}
