
#ifndef __FIX16_MATH__
#define __FIX16_MATH__


#include <stdint.h>

typedef int32_t fix16_t;


static const fix16_t fix16_one = 0x00010000;


fix16_t fix16_from_float(float x) {
  float tmp = x * fix16_one;
  tmp += tmp < 0 ? -0.5f : 0.5f;
  return (fix16_t)tmp;
}

#define F16(x) ((fix16_t)(((x) >= 0) ? ((x) * 65536.0 + 0.5) : ((x) * 65536.0 - 0.5)))

static inline float fix16_to_float(fix16_t x) { return (float)x / fix16_one; }

static inline fix16_t fix16_min(fix16_t x, fix16_t y) { return x < y ? x : y; }

static inline fix16_t fix16_max(fix16_t x, fix16_t y) { return x > y ? x : y; }

static inline fix16_t fix16_clamp(fix16_t x, fix16_t min, fix16_t max) {
  return fix16_max(fix16_min(x, max), min);
}

static inline fix16_t fix16_clamp_zero_one(fix16_t x) {
  return fix16_max(fix16_min(x, fix16_one), 0);
}

static inline fix16_t fix16_mul(fix16_t x, fix16_t y) {
  // No rounding, no overflow check
  return ((int64_t)x * y) >> 16;
}

// TODO: temporary stub, check binary and replace if needed
static inline fix16_t fix16_div(fix16_t x, fix16_t y) {
  // No rounding, no overflow check
  return ((((int64_t)x) << 16) / y) >> 16;
}

#endif
