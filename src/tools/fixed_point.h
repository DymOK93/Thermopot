#pragma once
#include <stdint.h>

typedef struct {
  int16_t value;
} FixedPoint16;

#define FpPack(whole, fractional) \
  ((int16_t)((uint16_t)(whole) << 1 | ((fractional)&1)))

#define FpInitialize(whole, fractional) \
  { FpPack((whole), (fractional)) }

#define FpReadAsNumber(fp) ((fp).value)
#define FpWriteAsNumber(fp, number) ((fp).value = (int16_t)(number))
#define FpZero(fp) FpWriteAsNumber((fp), 0)

#define FpWhole(fp) (FpReadAsNumber(fp) >> 1)
#define FpFractional(fp) (FpReadAsNumber(fp) & 1)

#define FpPositive(fp) ((fp).whole > 0)
#define FpNegative(fp) ((fp).whole < 0)

#define FpEqual(lhs, rhs) (FpReadAsNumber(lhs) == FpReadAsNumber(rhs))
#define FpGreater(lhs, rhs) (FpReadAsNumber(lhs) > FpReadAsNumber(rhs))
#define FpGreaterEqual(lhs, rhs) (FpReadAsNumber(lhs) >= FpReadAsNumber(rhs))
#define FpLess(lhs, rhs) (FpReadAsNumber(lhs) < FpReadAsNumber(rhs))
#define FpLessEqual(lhs, rhs) (FpReadAsNumber(lhs) <= FpReadAsNumber(rhs))

#define FpAdd(fp, lhs, rhs) \
  FpWriteAsNumber((fp), FpReadAsNumber(lhs) + FpReadAsNumber(rhs))

#define FpSub(fp, lhs, rhs) \
  FpWriteAsNumber((fp), FpReadAsNumber(lhs) - FpReadAsNumber(rhs))

#define FpMul(fp, lhs, rhs) \
  FpWriteAsNumber((fp), FpReadAsNumber(lhs) * FpReadAsNumber(rhs))

#define FpDiv(fp, lhs, rhs) \
  FpWriteAsNumber((fp), FpReadAsNumber(lhs) / FpReadAsNumber(rhs))

#define FpMin(lhs, rhs) (FpLessEqual((lhs), (rhs)) ? (lhs) : (rhs))
#define FpMax(lhs, rhs) (FpGreaterEqual((lhs), (rhs)) ? (lhs) : (rhs))
