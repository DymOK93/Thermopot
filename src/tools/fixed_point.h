/**
 * @file
 * @brief Fixed point arithmetic
 */
#pragma once
#include <stdint.h>

#define FP16_FRACTIONAL_BITS 1
#define FP16_FRACTIONAL_MASK ((1 << FP16_FRACTIONAL_BITS) - 1)
#define FP16_WHOLE_BITS (16 - FP16_FRACTIONAL_BITS)

/**
 * @struct FixedPoint16
 * @brief 16-bit fixed point value
 */
typedef struct {
  int16_t value;
} FixedPoint16;

#define Fp16Pack(whole, fractional)                      \
  ((int16_t)((uint16_t)(whole) << FP16_FRACTIONAL_BITS | \
             ((fractional)&FP16_FRACTIONAL_MASK)))

#define Fp16Initialize(whole, fractional) \
  { Fp16Pack((whole), (fractional)) }

#define Fp16ReadAsNumber(fp) ((fp).value)
#define Fp16WriteAsNumber(fp, number) ((fp).value = (int16_t)(number))
#define Fp16Zero(fp) Fp16WriteAsNumber((fp), 0)

#define Fp16Whole(fp) (Fp16ReadAsNumber(fp) >> 1)
#define Fp16Fractional(fp) (Fp16ReadAsNumber(fp) & 1)

#define Fp16Positive(fp) (Fp16ReadAsNumber(fp) > 0)
#define Fp16Negative(fp) (Fp16ReadAsNumber(fp) < 0)

#define Fp16Equal(lhs, rhs) (Fp16ReadAsNumber(lhs) == Fp16ReadAsNumber(rhs))
#define Fp16Greater(lhs, rhs) (Fp16ReadAsNumber(lhs) > Fp16ReadAsNumber(rhs))
#define Fp16GreaterEqual(lhs, rhs) \
  (Fp16ReadAsNumber(lhs) >= Fp16ReadAsNumber(rhs))
#define Fp16Less(lhs, rhs) (Fp16ReadAsNumber(lhs) < Fp16ReadAsNumber(rhs))
#define Fp16LessEqual(lhs, rhs) (Fp16ReadAsNumber(lhs) <= Fp16ReadAsNumber(rhs))

#define Fp16Add(fp, lhs, rhs) \
  Fp16WriteAsNumber((fp), Fp16ReadAsNumber(lhs) + Fp16ReadAsNumber(rhs))

#define Fp16Sub(fp, lhs, rhs) \
  Fp16WriteAsNumber((fp), Fp16ReadAsNumber(lhs) - Fp16ReadAsNumber(rhs))

#define Fp16Mul(fp, lhs, rhs) \
  Fp16WriteAsNumber((fp), Fp16ReadAsNumber(lhs) * Fp16ReadAsNumber(rhs))

#define Fp16Div(fp, lhs, rhs) \
  Fp16WriteAsNumber((fp), Fp16ReadAsNumber(lhs) / Fp16ReadAsNumber(rhs))

#define Fp16Min(lhs, rhs) (Fp16LessEqual((lhs), (rhs)) ? (lhs) : (rhs))
#define Fp16Max(lhs, rhs) (Fp16GreaterEqual((lhs), (rhs)) ? (lhs) : (rhs))
