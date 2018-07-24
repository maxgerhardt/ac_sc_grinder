
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

static inline int fix16_to_int(fix16_t x)
{
  if (x > 0) return (x + (fix16_one >> 1)) >> 16;
  return (x - (fix16_one >> 1)) >> 16;
}

static inline fix16_t fix16_min(fix16_t x, fix16_t y) { return x < y ? x : y; }

static inline fix16_t fix16_max(fix16_t x, fix16_t y) { return x > y ? x : y; }

static inline fix16_t fix16_clamp(fix16_t x, fix16_t min, fix16_t max)
{
  return fix16_max(fix16_min(x, max), min);
}

// Clamp x to be in (0.0 <= x < 1.0) range
// !!! 1.0 is NOT included
static inline fix16_t fix16_clamp_zero_one(fix16_t x)
{
  return fix16_max(fix16_min(x, F16(1) - 1), 0);
}

static inline fix16_t fix16_mul(fix16_t x, fix16_t y)
{
  // No rounding, no overflow check
  return ((int64_t)x * y) >> 16;
}


#ifdef __GNUC__
#define clz(x) (__builtin_clzl(x))
#else
int clz(unsigned x) {
  int n;

  if (x == 0) return(0);
  n = 0;
  if (x <= 0x0000FFFF) {n = n +16; x = x <<16;}
  if (x <= 0x00FFFFFF) {n = n + 8; x = x << 8;}
  if (x <= 0x0FFFFFFF) {n = n + 4; x = x << 4;}
  if (x <= 0x3FFFFFFF) {n = n + 2; x = x << 2;}
  if (x <= 0x7FFFFFFF) {n = n + 1;}
  return 32 - n;
}
#endif

// https://stackoverflow.com/a/4771946/1031804
// http://www.hackersdelight.org/hdcodetxt/divmnu.c.txt
// https://github.com/PetteriAimonen/libfixmath/blob/master/libfixmath/fix16.c
static inline fix16_t fix16_div(fix16_t x, fix16_t y)
{
  // No rounding, no overflow check
  // return ((((int64_t)x) << 16) / y) >> 16;

	if (y == 0) return 0;

	uint32_t remainder = (x >= 0) ? x : (-x);
	uint32_t divider = (y >= 0) ? y : (-y);
	uint32_t quotient = 0;
	int bit_pos = 17;

	// Opt for worst case (big divider/reminder)
	if (divider & 0xFFF00000)
	{
		uint32_t shifted_div = ((divider >> 17) + 1);
		quotient = remainder / shifted_div;
		remainder -= ((uint64_t)quotient * divider) >> 17;
	}

	// Quick path for ...0000 (bits) tail
	while (!(divider & 0xF) && bit_pos >= 4)
	{
		divider >>= 4;
		bit_pos -= 4;
	}

	while (remainder && bit_pos >= 0)
	{
		// Shift remainder as much as we can without overflowing
		int shift = clz(remainder);
		if (shift > bit_pos) shift = bit_pos;
		remainder <<= shift;
		bit_pos -= shift;

		uint32_t div = remainder / divider;
		remainder = remainder % divider;
		quotient += div << bit_pos;

		remainder <<= 1;
		bit_pos--;
	}

	fix16_t result = quotient >> 1;

  // Restore sign
	if ((x ^ y) & 0x80000000) result = -result;

  return result;
}


#endif
