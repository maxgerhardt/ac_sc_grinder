#ifndef __FIX16_SINUSIZE__
#define __FIX16_SINUSIZE__


#include "fix16_math.h"

// TODO: replace implementation with fast one, table-based.
#include "math.h"

// Convert linear requested "energy" to triac opening phase
// - Input: [0.0..1.0], desired energy (equivalent to 0..100%)
// - Output: [0.0..1.0], triac opening phase (0..100%)
fix16_t fix16_sinusize(fix16_t x) {
  float tmp = fix16_to_float(fix16_clamp_zero_one(x));
  return fix16_clamp_zero_one(fix16_from_float(acos(tmp) * (2.0 / 3.1416)));
}

#endif
