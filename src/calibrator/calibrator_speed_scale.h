#ifndef __CALIBRATOR_SPEED_SCALE__
#define __CALIBRATOR_SPEED_SCALE__

// Run motor at max speed and measure speed scaling factor.
// Max speed should have 1.0 at sensorss output.

#include "../fix16_math/fix16_math.h"

#include "../app.h"
#include "../sensors.h"
#include "../triac_driver.h"
#include "../speed_controller.h"

extern Sensors sensors;
extern TriacDriver triacDriver;
extern SpeedController speedController;

constexpr int calibrator_motor_startup_ticks = 3 * APP_TICK_FREQUENCY;


class CalibratorSpeedScale
{
public:

  bool tick() {
    switch (state) {

    case INIT:
      // Reset scaling factor
      sensors.cfg_rekv_to_speed_factor = fix16_one;
      setpoint = 0.05;
      setpoint_idx = 0;

      measure_attempts = 0;
      set_state(START_MEASURE);

    // Calc next setpoint & reset local vars
    case START_MEASURE:

      // Change setpoint from 0.0 to 1.0 and measure speed on each step.
      //
      // - step is not linear for optimal balance between speed and precision.
      // - some initial values are skipped, because useless anyway.

      if (setpoint < F16(0.2)) setpoint += F16(0.015625); // 1/64
      else if (setpoint < F16(0.375)) setpoint += F16(0.03125); // 1/32
      else setpoint += F16(0.125); // 1/8

      if (setpoint > fix16_one) setpoint = fix16_one; // clamp overflow

      triacDriver.setpoint = setpoint;

      // Init local iteration vars.
      speed_log[2] = fix16_minimum; // prevent false positives without data
      median_filter.reset();
      measure_attempts = 0;

      set_state(WAIT_STABLE_SPEED);

    // Wait for stable speed - measured speed should not
    // exceed max_speed long enougth
    case WAIT_STABLE_SPEED:
      triacDriver.tick();

      if (!sensors.zero_cross_up) break;

      median_filter.add(sensors.speed);
      ticks_cnt++;

      // ~ 0.25s
      if (ticks_cnt >= 12)
      {
        speed_log_push(median_filter.result());

        // if sepeed stable OR waited > 3 sec => record data
        if (sensors.speed == 0 || is_speed_stable() || measure_attempts > 13)
        {
          // Save setpoint data
          setpoints[setpoint_idx] = setpoint;
          rpms[setpoint_idx] = median_filter.result();
          setpoint_idx++;

          // End reached => go to processing
          if (setpoint >= fix16_one)
          {
            set_state(CALCULATE);
            break;
          }

          // Start with next setpoint
          set_state(START_MEASURE);
        }

        // Init for next measure attempt
        ticks_cnt = 0;
        median_filter.reset();

        break;
      }

      break;

    case CALCULATE:
      scale_factor = median_filter.result(); // last result => max RPMs at 1.0
      // Store speed scale factor and update seosons config
      eeprom_float_write(
        CFG_REKV_TO_SPEED_FACTOR_ADDR,
        fix16_to_float(scale_factor)
      );

      //
      // Generate correction table for scaled speed in [0.0..1.0] range
      // (0.0 and 1.0 points are skipped)
      //

      // Normalize setpoints to [0.0..1.0] range
      for (int i = 0; i < setpoint_idx; i++)
      {
        rpms[i] = fix16_div(rpms[i], scale_factor);
      }

      // Run down, skip first and last points
      for (int i = CFG_RPM_INTERP_TABLE_LENGTH - 1; i >= 0; i--)
      {
        fix16_t rpm = fix16_from_int(i + 1) / (CFG_RPM_INTERP_TABLE_LENGTH + 1);

        for (int idx = setpoint_idx - 2; idx >= 0; idx--)
        {
          if (rpms[idx] < rpm) // matching range found
          {
            // count point proportion (scale) between RPM values
            fix16_t sc = fix16_div(
              rpm - rpms[idx],
              rpms[idx + 1] - rpms[idx]
            );

            // apply to setpoints interval
            fix16_t result =
              fix16_mul(setpoints[idx], fix16_one - sc) +
              fix16_mul(setpoints[idx + 1], sc);

            // store
            eeprom_float_write(CFG_RPM_INTERP_TABLE_START_ADDR + i, fix16_to_float(result));

            break;
          }
        }
      }

      // Reload new config content
      sensors.configure();
      speedController.configure();

      set_state(STOP);

      break;

    // Motor off and wait 1 sec
    case STOP:
      triacDriver.setpoint = 0;
      triacDriver.tick();

      if (ticks_cnt++ > 1 * APP_TICK_FREQUENCY) {
        set_state(INIT);
        return true;
      }

      break;
    }

    return false;
  }

private:

  enum State {
    INIT,
    START_MEASURE,
    WAIT_STABLE_SPEED,
    CALCULATE,
    STOP
  } state = INIT;

  int ticks_cnt = 0;
  fix16_t max_speed = 0;
  fix16_t setpoint = 0;

  // History of measured speed. Used to detect stable values.
  fix16_t speed_log[3] = { 0, 0, fix16_minimum };
  int measure_attempts = 0;

  // We use ~20 intervals to collect rpm/volts
  // Reserve a bit more to skip bounds checks
  fix16_t setpoints[40];
  fix16_t rpms[40];
  int setpoint_idx = 0;

  fix16_t scale_factor;

  MedianIteratorTemplate<fix16_t, 32> median_filter;

  void set_state(State st)
  {
    state = st;
    ticks_cnt = 0;
  }

  bool is_speed_stable()
  {
    fix16_t a = speed_log[0], b = speed_log[1], c = speed_log[2];

    fix16_t min = (a <= b && a <= c) ? a : ((b <= a && b <= c) ? b : c);
    fix16_t max = (a >= b && a >= c) ? a : ((b >= a && b >= c) ? b : c);

    fix16_t diff = max - min;

    fix16_t abs_max = max > 0 ? max : - max;
    fix16_t abs_diff = diff > 0 ? diff : - diff;

    return abs_diff <= abs_max / 100;
  }

  void speed_log_push(fix16_t val)
  {
    speed_log[0] = speed_log[1];
    speed_log[1] = speed_log[2];
    speed_log[2] = val;
  }
};


#endif
