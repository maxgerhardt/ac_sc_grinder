#ifndef __TRIAC_DRIVER__
#define __TRIAC_DRIVER__


#include <math.h>
#include "utils.h"
#include "stm32f1xx_hal.h"


// TODO: check polarity
#define TRIAC_OFF() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
#define TRIAC_ON()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)


class TriacDriver
{
public:
  // 0..100% of desired triac "power".
  // Will be used to calculate opening phase for each half sine wave
  float setpoint = 0.0;

  // 40 kHz
  void tick()
  {
    // TODO: at 100% power (start of sine wave) - wait for safe ignition voltage
    // (should be > 25v, measure at positive wave) before triac control signal remove.

    // If triac was activated (in prev tick) and still active - deactivate it.
    if (triac_open_done && !triac_close_done) {
      triac_close_done = true;
      TRIAC_OFF();
    }

    // If triac was not yet activated - check if we can to this
    if (!triac_open_done) {
      // "Linearize" setpoint to phase shift & scale to 0..1
      float normalized_setpoint = clamp(acos(setpoint / 100.0) * (2.0 / 3.1416), 0.0, 1.0);

      // TODO: measure period in ticks instead of hardcoding
      // Calculate ticks treshold when triac should be enabled
      int ticks_treshold = normalized_setpoint * (40000.0 / 100.0);

      if (phase_counter >= ticks_treshold) {
        triac_open_done = true;
        TRIAC_ON();
      }
    }

    phase_counter++;
  }

  void rearm()
  {
    phase_counter = 0;
    triac_open_done = false;
    triac_close_done = false;

    // Make sure to disable triac signal, if reset (zero cross) happens
    // immediately after triac enabled
    TRIAC_OFF();
  }

private:
  int phase_counter = 0; // increment every tick
  bool triac_open_done = false;
  bool triac_close_done = false;
};


#endif
