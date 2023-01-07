/**
 * @file
 * @brief Macro helpers for the "do-while(false)" idiom
 */
#pragma once
#include "status.h"

#define BREAK_ON_TRUE(cond) \
  if (cond)                 \
  break

#define BREAK_ON_FALSE(cond) BREAK_ON_TRUE(!(cond))
#define BREAK_ON_ERROR(status) BREAK_ON_TRUE(TP_ERROR(status))
#define BREAK_ON_SUCCESS(status) BREAK_ON_TRUE(TP_SUCCESS(status))
