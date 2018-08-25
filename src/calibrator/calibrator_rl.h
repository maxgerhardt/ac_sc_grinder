#ifndef __CALIBRATOR_RL__
#define __CALIBRATOR_RL__

// Calculates motor's R and L.

#include "../fix16_math/fix16_math.h"

#include "../sensors.h"
#include "../triac_driver.h"

extern Sensors sensors;
extern TriacDriver triacDriver;

// Array size to record up to half of current & voltage positive wave.
// Full period is (APP_TICK_FREQUENCY / 50).
//
// I theory, we it would be enougth to save ~ 1/8 of voltage positive wave,
// and calc the rest on the fly. That will save a lot of memory, but we have
// no time for such optimization.
constexpr int calibrator_rl_buffer_length = APP_TICK_FREQUENCY / 50;


class CalibratorRL
{
public:

  bool tick(void) {

    fix16_t voltage = sensors.voltage;
    fix16_t current = sensors.current;
    fix16_t p_sum, i2_sum, R_motor;

    switch (state) {

    // Reset variables and wait 1 second to make sure motor stopped.
    case INIT:
      prev_current = 0;
      cal_rl_buffer_head = 0;
      last_positive_element = 0;

      triacDriver.voltage = voltage;
      triacDriver.setpoint = 0;
      triacDriver.tick();

      if (ticks_cnt++ >= (1 * APP_TICK_FREQUENCY)) set_state(WAIT_ZERO_CROSS);

      break;

    // Calibration should be started at the begining of positive period
    case WAIT_ZERO_CROSS:

      triacDriver.voltage = voltage;
      triacDriver.setpoint = 0;
      triacDriver.tick();

      if (sensors.zero_cross_up) set_state(MEASURE);

      break;

    // Save voltage and current data to buffers
    // until current crosses zero.
    case MEASURE:
      // turn on triac
      triacDriver.voltage = voltage;
      triacDriver.setpoint = fix16_one;
      triacDriver.tick();

      // When negative half-wave of voltage begins,
      // save buffer head to use in extrapolation
      // of negative voltage values.
      if (sensors.zero_cross_down)
      {
        last_positive_element = cal_rl_buffer_head;
      }

      // Save positive voltage values, extrapolate
      // negative voltage values from previously
      // saved positive values.
      if (voltage > 0)
      {
        voltage_buffer[cal_rl_buffer_head] = voltage;
      }
      else
      {
        voltage_buffer[cal_rl_buffer_head] =
          voltage_buffer[cal_rl_buffer_head - last_positive_element];
      }

      current_buffer[cal_rl_buffer_head] = current;

      // Protection from buffers overflow
      if (cal_rl_buffer_head < calibrator_rl_buffer_length) cal_rl_buffer_head++;

      // Stop saving data when current crosses zero,
      // turn off triac
      if ((prev_current > 0) && (current = 0))
      {
        set_state(CALCULATE);

        triacDriver.voltage = voltage;
        triacDriver.setpoint = 0;
        triacDriver.tick();
      }

      break;

    // Calculate resistance and inductance of motor
    case CALCULATE:

      // Holds active power
      p_sum = 0;
      // Holds square of current.
      i2_sum = 0;

      for (uint32_t i = 0; i < cal_rl_buffer_head; i++)
      {
        p_sum += fix16_mul(voltage_buffer[i], current_buffer[i]);
        i2_sum += fix16_mul(current_buffer[i], current_buffer[i]);
      }

      // Active power is equal to Joule power in this case
      // Current^2 * R = P
      // R = P / Current^2
      R_motor = fix16_div(p_sum, i2_sum);

      eeprom_float_write(CFG_MOTOR_RESISTANCE_ADDR, fix16_to_float(R_motor));

      // Reload sensor's config.
      sensors.configure();

      set_state(INIT);

      return true;
    }

    prev_current = current;

    return false;
  }

private:

  fix16_t prev_current = 0;

  fix16_t voltage_buffer[calibrator_rl_buffer_length];
  fix16_t current_buffer[calibrator_rl_buffer_length];

  // Holds head of voltage and current buffers
  uint32_t cal_rl_buffer_head = 0;
  // Holds last element with positive voltage
  // in buffer, used in extrapolation of
  // negative voltage values.
  uint32_t last_positive_element = 0;

  enum State {
    INIT,
    WAIT_ZERO_CROSS,
    MEASURE,
    CALCULATE
  } state = INIT;

  int ticks_cnt = 0;

  void set_state(State st)
  {
    ticks_cnt = 0;
    state = st;
  }
};

#endif
