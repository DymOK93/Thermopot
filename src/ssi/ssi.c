#include "ssi.h"

#include <tools/utils.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <stm32f0xx.h>

#define SSI_FP_INDEX 1
#define SSI_FP_DOT 0x80
#define SSI_DIGIT_SHIFT 2

#define SSI_TIMER_INTERRUPT_PRIORITY 1
#define SSI_SPI_INTERRUPT_PRIORITY 1
#define SSI_FRAMES_PER_SECOND 60

typedef struct {
  SsiValue value;
  volatile uint8_t segment_mask[SSI_PANEL_SIZE];
  volatile bool is_number;
  volatile uint8_t idx;
  const FixedPoint16 min_number;
  const FixedPoint16 max_number;
} SsiState;

static SsiState g_ssi_state = {.value = {{0}},
                               .segment_mask = {0},
                               .is_number = false,
                               .idx = 0,
                               .min_number = Fp16Initialize(SSI_NUMBER_MIN, 0),
                               .max_number = Fp16Initialize(SSI_NUMBER_MAX, 0)};

static void SsipPrepareGpio(void) {
  /**
   * 1. Activate PA5, PA7 in the alternative function mode; AF0
   * (SPI1_SCK/SPI1_MOSI) already selected
   * 2. Configure PA6 as output with weak pull-down
   * 3. Configure PB2-5 as open-drain outputs to control the indicator digits
   */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN);
  SET_BIT(GPIOA->MODER, GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1);  // (1)
  SET_BIT(GPIOA->MODER, GPIO_MODER_MODER6_0);                        // (2)

  SET_BIT(GPIOB->MODER, GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0 |
                            GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0);  // (3)
  SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_2 | GPIO_OTYPER_OT_3 |
                             GPIO_OTYPER_OT_4 | GPIO_OTYPER_OT_5);
}

static void SsipSetupSpi(void) {
  /**
   * The maximum frequency of 74HC595 is about 4MHz, the maximum frequency of
   * AHB and APB2 is 48MHz, so set SCK as APB2_CLK/16 in Master mode
   * 8-bit transfer is configured by default (explicit 8-bit write access to DR
   * is required); also enable TX-only bidirectional mode
   */
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);
  SET_BIT(SPI1->CR1, SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_SSM |
                         SPI_CR1_SSI | SPI_CR1_BR_0 | SPI_CR1_BR_1 |
                         SPI_CR1_MSTR);
  NVIC_SetPriority(SPI1_IRQn, SSI_SPI_INTERRUPT_PRIORITY);
  NVIC_EnableIRQ(SPI1_IRQn);
}

static void SsipSetupTimer(void) {
  /*
   * 1. Tick period is 1ms
   * 2. Enable update interrupt
   */
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM6EN);
  TIM6->PSC = (uint16_t)(SystemCoreClock / 1000 - 1);  // (1)
  TIM6->ARR = (uint16_t)(1000 / (SSI_FRAMES_PER_SECOND * SSI_PANEL_SIZE));

  SET_BIT(TIM6->DIER, TIM_DIER_UIE);  // (2)

  NVIC_SetPriority(TIM6_DAC_IRQn, SSI_TIMER_INTERRUPT_PRIORITY);
  NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

static void SsipStart() {
  SET_BIT(TIM6->CR1, TIM_CR1_CEN);
  SET_BIT(SPI1->CR1, SPI_CR1_SPE);
}

static void SsipStop(void) {
  CLEAR_BIT(SPI1->CR1, SPI_CR1_SPE);
  CLEAR_BIT(TIM6->CR1, TIM_CR1_CEN);
  TIM6->CNT = 0;
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

static SsiValue SsipParseNumber(FixedPoint16 number) {
  const int16_t whole_part = Fp16Whole(number);
  uint16_t unsigned_number = (uint16_t)abs(whole_part);
  uint8_t idx = 1;
  SsiValue value = {{Fp16Fractional(number) ? '5' : '0', 0, 0, 0}};

  if (unsigned_number == 0) {
    value.str[idx] = '0';
  } else {
    for (; unsigned_number != 0; ++idx) {
      value.str[idx] = (char)('0' + unsigned_number % 10);
      unsigned_number /= 10;
    }
    if (whole_part < 0) {
      value.str[idx] = '-';
    }
  }
  return value;
}

static uint8_t SsipGetSegmentMask(char value) {
  /*
   * Bits 0..7 -> segments A...G (H is always empty)
   */
  uint8_t mask;
  switch (value) {
    case '\0':
    case ' ':
      mask = 0x0;  // 0b00000000
      break;
    case '-':
      mask = 0x40;  // 0b01000000
      break;
    case '0':
      mask = 0x3F;  // 0b00111111
      break;
    case '1':
      mask = 0x6;  // 0b00000110
      break;
    case '2':
      mask = 0x5B;  // 0b01011011
      break;
    case '3':
      mask = 0x4F;  // 0b01001111
      break;
    case '4':
      mask = 0x66;  // 0b01100110
      break;
    case '5':
      mask = 0x6D;  // 0b01111101
      break;
    case '6':
      mask = 0x7D;  // 0b01111101
      break;
    case '7':
      mask = 0x7;  // 0b00000111
      break;
    case '8':
      mask = 0x7F;  // 0b01111111
      break;
    case '9':
      mask = 0x6F;  // 0b01101111
      break;
    case 'a':
    case 'A':
      mask = 0x77;  // 0b01110111
      break;
    case 'b':
    case 'B':
      mask = 0xFC;  // 0b01110111
      break;
    case 'c':
    case 'C':
      mask = 0x39;  // 0b00111001
      break;
    case 'd':
    case 'D':
      mask = 0x5E;  // 0b01011110
      break;
    case 'e':
    case 'E':
      mask = 0x79;  // 0b01111001
      break;
    case 'f':
    case 'F':
      mask = 0x71;  // 0b01110001
      break;
    case 'g':
    case 'G':
      mask = 0x6F;  // 0b01101111
      break;
    case 'h':
    case 'H':
      mask = 0x76;  // 0b01110110
      break;
    case 'i':
    case 'I':
      mask = 0x30;  // 0b00110000
      break;
    case 'j':
    case 'J':
      mask = 0x1E;  // 0b00011110
      break;
    case 'k':
    case 'K':
      mask = 0x76;  // 0b01110110
      break;
    case 'l':
    case 'L':
      mask = 0x38;  // 0b00111000
      break;
    case 'm':
    case 'M':
      mask = 0x15;  // 0b00010101
      break;
    case 'n':
    case 'N':
      mask = 0x54;  // 0b01010100
      break;
    case 'o':
    case 'O':
      mask = 0x3F;  // 0b00111111
      break;
    case 'p':
    case 'P':
      mask = 0x73;  // 0b01110011
      break;
    case 'q':
    case 'Q':
      mask = 0x67;  // 0b01100111
      break;
    case 'r':
    case 'R':
      mask = 0x50;  // 0b01010000
      break;
    case 's':
    case 'S':
      mask = 0x6D;  // 0b01111101
      break;
    case 't':
    case 'T':
      mask = 0x78;  // 0b01111000
      break;
    case 'u':
    case 'U':
      mask = 0x3E;  // 0b00111110
      break;
    case 'v':
    case 'V':
      mask = 0x1C;  // 0b00011100
      break;
    case 'w':
    case 'W':
      mask = 0x2A;  // 0b00101010
      break;
    case 'x':
    case 'X':
      mask = 0x76;  // 0b01110110
      break;
    case 'y':
    case 'Y':
      mask = 0x6E;  // 0b01101110
      break;
    case 'z':
    case 'Z':
      mask = 0x5B;  // 0b01011011
      break;
    default:
      mask = 0x0;  // 0b00000000
      break;
  }
  return mask;
}

static bool SsipStaleValue(SsiValue value, bool is_number) {
  if (is_number != g_ssi_state.is_number) {
    return false;
  }
  return memcmp(&value, &g_ssi_state.value, sizeof(SsiValue)) == 0;
}

static void SsipFillSegmentMask(SsiValue value, bool is_number) {
  uint8_t idx = 0;
  for (; idx < SSI_PANEL_SIZE; ++idx) {
    const uint8_t segment_mask =
        SsipGetSegmentMask(value.str[idx]) |
        (is_number && idx == SSI_FP_INDEX ? SSI_FP_DOT : 0);
    g_ssi_state.segment_mask[idx] = segment_mask;
  }
}

static TpStatus SsipSetValue(SsiValue value, bool is_number) {
  if (!SsipValidateValue(value)) {
    return TpInvalidParameter;
  }
  if (!SsipStaleValue(value, is_number)) {
    g_ssi_state.value = value;
    g_ssi_state.is_number = is_number;
    SsipFillSegmentMask(value, is_number);
  }
  return TpSuccess;
}

static void SsipDrawSegment(uint8_t segment_mask) {
  /*
   * Explicit 8-bit write access is root of evil
   */
  *(volatile uint8_t*)&SPI1->DR = segment_mask;
  while (READ_BIT(SPI1->SR, SPI_SR_BSY))
    ;
}

static void SsipActivateSegment(uint8_t idx) {
  /*
   * 1. Transfer data to storage register by setting STCP
   * 2. Clear all digits
   * 3. Activate the corresponding output (0...3 -> PB2...5)
   * 4. Reset STCP
   */
  SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_6);  // (1)
  SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_6);  // (4)

  SET_BIT(GPIOB->ODR, 0xF << SSI_DIGIT_SHIFT);                      // (2)
  CLEAR_BIT(GPIOB->ODR, (uint16_t)(1 << (idx + SSI_DIGIT_SHIFT)));  // (3)
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

TpStatus SsiClearValue(void) {
  const SsiValue value = {0};
  return SsiSetValue(value);
}

TpStatus SsiSetValue(SsiValue value) {
  return SsipSetValue(value, false);
}

TpStatus SsiSetNumber(FixedPoint16 number) {
  if (Fp16Less(number, g_ssi_state.min_number) ||
      Fp16Greater(number, g_ssi_state.max_number)) {
    return TpInvalidParameter;
  }
  return SsipSetValue(SsipParseNumber(number), true);
}

void TIM6_DAC_IRQHandler(void) {
  CLEAR_BIT(TIM6->SR, TIM_SR_UIF);

  const uint8_t idx = g_ssi_state.idx;
  const uint8_t segment_mask = g_ssi_state.segment_mask[idx];
  SsipDrawSegment(segment_mask);
  SsipActivateSegment(idx);
  g_ssi_state.idx = idx + 1 == SSI_PANEL_SIZE ? 0 : idx + 1;
}
