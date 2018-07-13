#ifndef __UTILS__
#define __UTILS__


float clamp(float value, float min, float max)
{
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

float normalize(float value, float dead_zone_width, float min, float max)
{
  if (value < dead_zone_width) return 0.0;
  else return (value - dead_zone_width) / (100.0 - dead_zone_width) * (max - min) + min;
}

#endif
