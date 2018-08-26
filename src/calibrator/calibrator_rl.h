#ifndef __CALIBRATOR_RL__
#define __CALIBRATOR_RL__

// Calculates motor's R and L.

#include <algorithm>
#include <cmath>

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

    switch (state) {

    // Reset variables and wait 1 second to make sure motor stopped.
    case INIT:
      buffer_idx = 0;
      zero_cross_down_offset = 0;

      triacDriver.setpoint = 0;
      triacDriver.tick();

      if (ticks_cnt++ >= (1 * APP_TICK_FREQUENCY)) set_state(WAIT_ZERO_CROSS);

      break;

    // Calibration should be started at the begining of positive period
    case WAIT_ZERO_CROSS:

      triacDriver.setpoint = 0;
      triacDriver.tick();

      if (!sensors.zero_cross_up) break;

      // Fall down to recording immediately, we should not miss data for this tick
      set_state(RECORD_POSITIVE_WAVE);

    case RECORD_POSITIVE_WAVE:
      // turn on triac
      triacDriver.setpoint = fix16_one;
      triacDriver.tick();

      // Safety check. Restart on out of bounds.
      if (buffer_idx >= calibrator_rl_buffer_length) {
        set_state(INIT);
        break;
      }

      voltage_buffer[buffer_idx] = fix16_to_float(sensors.voltage);
      current_buffer[buffer_idx] = fix16_to_float(sensors.current);

      buffer_idx++;

      if (sensors.zero_cross_down) {
        zero_cross_down_offset = buffer_idx;
        set_state(RECORD_NEGATIVE_WAVE);
      }

      break;

    case RECORD_NEGATIVE_WAVE:
      // turn off triac
      triacDriver.setpoint = 0;
      triacDriver.tick();

      // If got enougth data (buffer ended or next zero cross found),
      // go to data processing
      if (sensors.zero_cross_up || buffer_idx >= calibrator_rl_buffer_length)
      {
        set_state(CALCULATE);
        break;
      }

      // Record current & emulate voltage
      voltage_buffer[buffer_idx] = - voltage_buffer[buffer_idx - zero_cross_down_offset];
      current_buffer[buffer_idx] = fix16_to_float(sensors.current);

      buffer_idx++;

      break;

    // Calculate resistance and inductance of motor
    // That may take a lot of time, but we don't care about triac at this moment
    case CALCULATE:
      process_data();
      set_state(INIT);

      return true;
    }

    return false;
  }

private:

  float voltage_buffer[calibrator_rl_buffer_length];
  float current_buffer[calibrator_rl_buffer_length];

  uint32_t buffer_idx = 0;
  uint32_t zero_cross_down_offset = 0;

  MedianIteratorTemplate<float, 32> median_filter;

  enum State {
    INIT,
    WAIT_ZERO_CROSS,
    RECORD_POSITIVE_WAVE,
    RECORD_NEGATIVE_WAVE,
    CALCULATE
  } state = INIT;

  int ticks_cnt = 0;

  void set_state(State st)
  {
    ticks_cnt = 0;
    state = st;
  }

  void process_data()
  {
    //
    // Process R
    //

    float p_sum = 0;  // active power
    float i2_sum = 0; // square of current

    for (uint32_t i = 0; i < buffer_idx; i++)
    {
      p_sum += voltage_buffer[i] * current_buffer[i];
      i2_sum += current_buffer[i] * current_buffer[i];
    }

    // Active power is equal to Joule power in this case
    // Current^2 * R = P
    // R = P / Current^2
    float R = p_sum / i2_sum;

    eeprom_float_write(CFG_MOTOR_RESISTANCE_ADDR, R);

    //
    // Process L
    //

    float max_current = *std::max_element(voltage_buffer, voltage_buffer + buffer_idx);
    float treshold = max_current * 0.1;

    median_filter.reset();

    for (uint32_t i = 1; i < buffer_idx; i++)
    {
      // Skip noisy data
      if (current_buffer[i] < treshold || current_buffer[i - 1] < treshold) continue;

      float di_dt = (current_buffer[i] - current_buffer[i - 1]) * APP_TICK_FREQUENCY;

      if (std::abs(di_dt) < 0.05) continue;

      // L = (V - R * I) / (dI/dt)
      float _l = (voltage_buffer[i] - R * current_buffer[i]) / di_dt;

      median_filter.add(_l);
    }

    float L = median_filter.result();

    eeprom_float_write(CFG_MOTOR_INDUCTANCE_ADDR, L);

    // Reload sensor's config.
    sensors.configure();
  }
};

#endif
