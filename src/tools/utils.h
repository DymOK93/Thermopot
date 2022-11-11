#pragma once

#define IS_NULL_TERMINATOR(ch) ((ch) == '\0')
#define IS_SPACE(ch) ((ch) == ' ')
#define IS_DASH(ch) ((ch) == '-')
#define IS_DIGIT(ch) ((ch) >= '0' && (ch) <= '9')
#define IS_LETTER(ch) \
  (((ch) >= 'a' && (ch) <= 'z') || ((ch) >= 'A' && (ch) <= 'Z'))

#define SET_BIT(flag, mask) ((flag) |= (mask))
#define CLEAR_BIT(flag, mask) ((flag) &= ~(mask))
#define READ_BIT(flag, mask) ((flag) & (mask))
