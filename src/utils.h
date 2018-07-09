#ifndef __UTILS__
#define __UTILS__


float clamp(float value, float min, float max)
{
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

#endif
