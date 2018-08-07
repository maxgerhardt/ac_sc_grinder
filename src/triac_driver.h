#ifndef __TRIAC_DRIVER__
#define __TRIAC_DRIVER__


#include "stm32f1xx_hal.h"
#include "fix16_math/fix16_math.h"


#define TRIAC_OFF() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
#define TRIAC_ON()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)

// Minimal voltage for guaranteed triac opening.
#define MIN_IGNITION_VOLTAGE (F16(25))

class TriacDriver
{
public:
  // 0..100% of desired triac "power".
  // Will be used to calculate opening phase for each half sine wave
  fix16_t setpoint = 0;
  fix16_t voltage = 0;

  // 40 kHz
  void tick()
  {
    // Poor man zero cross check
    if (((prev_voltage == 0) && (voltage > 0)) ||
        ((prev_voltage > 0) && (voltage == 0)))
    {
      rearm();
    }

    // Measure ticks after positive zero gross until voltage > MIN_IGNITION_VOLTAGE.
    // That's done on each positive wave and result is reused on negative wave.
    if ((voltage >= MIN_IGNITION_VOLTAGE) && (prev_voltage < MIN_IGNITION_VOLTAGE))
    {
      if (once_zero_crossed) safe_ignition_threshold = phase_counter;
    }

    // If period_in_ticks is not yet detected, only increment phase_counter,
    // don't turn on triac anyway.
    if (!once_period_counted)
    {
      phase_counter++;
      return;
    }

    // If triac was activated (in prev tick) and still active - deactivate it.
    if (triac_open_done && !triac_close_done) {
      triac_close_done = true;
      TRIAC_OFF();
    }

    // If triac was not yet activated - check if we can do this
    if (!triac_open_done && (phase_counter >= safe_ignition_threshold)) {
      // "Linearize" setpoint to phase shift & scale to 0..1
      fix16_t normalized_setpoint = fix16_sinusize(setpoint);

      // Calculate ticks treshold when triac should be enabled:
      // "mirror" and "enlarge" normalized setpoint
      uint32_t ticks_threshold = fix16_to_int(
          (fix16_one - normalized_setpoint) * correct_period_in_ticks
        ) + threshold_correction;
      
      if (phase_counter >= ticks_threshold) {
        triac_open_done = true;
        TRIAC_ON();
      }
    }

    phase_counter++;
    prev_voltage = voltage;
  }

private:
  uint32_t phase_counter = 0; // increment every tick
  bool triac_open_done = false;
  bool triac_close_done = false;

  // Holds measured number of ticks per positive half-period
  uint32_t positive_period_in_ticks = 0;
  // Holds measured number of ticks per negative half-period
  uint32_t negative_period_in_ticks = 0;
  // Due to filtration before zero-crossing detection 
  // measured positive half-period of voltage is bigger than
  // measured negative half-period. Real length of half-period
  // is (positive + negative / 2)
  // Holds real number of ticks per half-period
  uint32_t correct_period_in_ticks = 0;

  // Holds number of ticks by which zero-cross points are
  // shifted in time due to filtration before zero-crossing detection 
  int32_t threshold_correction = 0;

  // Holds ticks threshold when triac control signal is safe to turn off, vlotage > 25v
  // Initial value set to 0 because during the first period triac
  // won't turn on anyway
  uint32_t safe_ignition_threshold = 0;

  fix16_t prev_voltage = 0;

  bool once_zero_crossed = false;
  bool once_period_counted = false;


  // Happens on every zero cross
  void rearm()
  {
    if (once_zero_crossed) once_period_counted = true;

    once_zero_crossed = true;

    // If full half-period was counted at least once, save number of
    // ticks in half-period
    if (once_period_counted) 
    {
      if (voltage == 0) 
      {
        positive_period_in_ticks = phase_counter;
        // Zero-cross points are shifted in time due to filtration
        // For negative half-period correction must be negative
        threshold_correction = - threshold_correction;
      }

      if (voltage > 0)
      {
        negative_period_in_ticks = phase_counter;
        // Real length of half-period
        correct_period_in_ticks = (positive_period_in_ticks + negative_period_in_ticks) / 2;
        // Zero-cross points are shifted in time due to filtration, calculate this shift
        // For positive half-period correction is positive
        threshold_correction = (positive_period_in_ticks - correct_period_in_ticks) / 2;
      }
    }

    phase_counter = 0;
    triac_open_done = false;
    triac_close_done = false;

    // Make sure to disable triac signal, if reset (zero cross) happens
    // immediately after triac enabled
    TRIAC_OFF();
  }
};


#endif
