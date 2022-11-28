#pragma once
#include <stdint.h>

typedef struct {
  int16_t whole : 15;
  int16_t fractional : 1;
} FixedPoint16;

#define FpAsNumber(fp) (((fp).whole << 1) | (fp).fractional)

#define FpEqual(lhs, rhs) (FpAsNumber(lhs) == FpAsNumber(rhs))
#define FpGreater(lhs, rhs) (FpAsNumber(lhs) > FpAsNumber(rhs))
#define FpLess(lhs, rhs) (FpAsNumber(lhs) < FpAsNumber(rhs))

#define FpAdd(lhs, rhs, result)                                \
  ((result)->whole = (FpAsNumber(lhs) + FpAsNumber(rhs)) >> 1, \
   (result)->fractional = (FpAsNumber(lhs) + FpAsNumber(rhs)) & 1)

#define FpSub(lhs, rhs, result)                                \
  ((result)->whole = (FpAsNumber(lhs) - FpAsNumber(rhs)) >> 1, \
   (result)->fractional = (FpAsNumber(lhs) - FpAsNumber(rhs)) & 1)
