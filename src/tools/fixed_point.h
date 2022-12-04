#pragma once
#include <stdint.h>

typedef struct {
  int16_t whole : 15;
  int16_t fractional : 1;
} FixedPoint16;

#define FpAsNumber(fp) (((fp).whole << 1) | (fp).fractional)

#define FpPositive(fp) ((fp).whole > 0)
#define FpNegative(fp) ((fp).whole < 0)

#define FpEqual(lhs, rhs) (FpAsNumber(lhs) == FpAsNumber(rhs))
#define FpGreater(lhs, rhs) (FpAsNumber(lhs) > FpAsNumber(rhs))
#define FpGreaterEqual(lhs, rhs) (FpAsNumber(lhs) >= FpAsNumber(rhs))
#define FpLess(lhs, rhs) (FpAsNumber(lhs) < FpAsNumber(rhs))
#define FpLessEqual(lhs, rhs) (FpAsNumber(lhs) <= FpAsNumber(rhs))

#define FpAdd(lhs, rhs, result) \
  (*(uint16_t*)(result) = (FpAsNumber(lhs) + FpAsNumber(rhs)))

#define FpSub(lhs, rhs, result) \
  (*(uint16_t*)(result) = (FpAsNumber(lhs) - FpAsNumber(rhs)))

#define FpMul(lhs, rhs, result) \
  (*(uint16_t*)(result) = (FpAsNumber(lhs) * FpAsNumber(rhs)))

#define FpDiv(lhs, rhs, result) \
  (*(uint16_t*)(result) = (FpAsNumber(lhs) / FpAsNumber(rhs)))

#define FpMin(lhs, rhs) (FpLessEqual((lhs), (rhs)) ? (lhs) : (rhs))
#define FpMax(lhs, rhs) (FpGreaterEqual((lhs), (rhs)) ? (lhs) : (rhs))

#define FpZero(fp) (*(uint16_t*)(fp) = 0)
