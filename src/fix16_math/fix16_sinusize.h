#ifndef __FIX16_SINUSIZE__
#define __FIX16_SINUSIZE__


#include "fix16_math.h"

// TODO: why PROGRAM/DATA does not depend on array size?
#include "fix16_sinusize_table.h"

// Convert linear requested "energy" to Sine-wave shift (used to
// calculate triac opening phase)
// - Input: [0.0..1.0), desired energy (equivalent to 0..100%)
// - Output: [0.0..1.0)
//
// NOTE: Don-t fogret to reverse range to get resl triac opening phasephase
fix16_t fix16_sinusize(fix16_t x)
{
  fix16_t tmp = fix16_clamp_zero_one(x);

  // 16 bits - fractional part. Need 9 bit for lookup
  return (fix16_t)(sinusize_table[ tmp >> (16 - SINUSIZE_TABLE_SIZE_BITS) ]);
}


#endif
