/**
 * @file
 * @brief Macro helpers for the "do-while(false)" idiom
 */
#pragma once
#include "status.h"

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define BREAK_ON_TRUE(cond) \
  if (cond)                 \
  break

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define BREAK_ON_FALSE(cond) BREAK_ON_TRUE(!(cond))

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define BREAK_ON_ERROR(status) BREAK_ON_TRUE(TP_ERROR(status))

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define BREAK_ON_SUCCESS(cond) BREAK_ON_TRUE(TP_SUCCESS(status))
