#include <math.h>
#include "stm32f1xx_hal.h"

class Triac
{
public:

  float setpoint = 0.0;

  void tick()
  {
    /* angle - control angle in ticks

       output voltage is proportional to 2 * arccosinus of control angle

       setpoint - 0-100%
       tick_frequency - 40 kHz
       voltage_half_period - 1/100 sec, or 1/120 sec

       angle = 2 * arccosinus(setpoint / 100%) / pi * tick_frequency * voltage_half_period
    */
    float angle = 2 * acos(setpoint / 100.0) / 3.1416 * 40000.0 / 100.0;

    if (triac_tick_counter == angle)
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    else if (triac_tick_counter == angle + 1)
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

    triac_tick_counter++;
  }

  void reset()
  {
    triac_tick_counter = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  }

private:
  int triac_tick_counter = 0;
};
