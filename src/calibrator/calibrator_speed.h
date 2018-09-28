#ifndef __CALIBRATOR_SPEED_SCALE__
#define __CALIBRATOR_SPEED_SCALE__

// Run motor at max speed and measure speed scaling factor.
// Max speed should have 1.0 at sensorss output.

#include "../math/fix16_math.h"
#include "../math/polyfit.h"

#include "../app.h"
#include "../sensors.h"
#include "../triac_driver.h"
#include "../speed_controller.h"

#include <math.h>

extern Sensors sensors;
extern TriacDriver triacDriver;
extern SpeedController speedController;

constexpr int calibrator_motor_startup_ticks = 3 * APP_TICK_FREQUENCY;


class CalibratorSpeed
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

      // Pick setpoint value from set of optimal values
      setpoint = setpoints_preset[setpoint_idx];

      triacDriver.setpoint = setpoint;

      // Init local iteration vars.
      speed_log[2] = fix16_minimum; // prevent false positives without data
      median_filter.reset();
      measure_attempts = 0;

      set_state(WAIT_STABLE_SPEED);

    // Wait for stable speed and record result.
    // Force end if waiting too long.
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
          // Save rpm value for current setpoint
          rpms[setpoint_idx] = fix16_to_float(median_filter.result());
          setpoints[setpoint_idx] = fix16_to_float(setpoint);
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
        measure_attempts++;
        // Init for next measure attempt
        ticks_cnt = 0;
        median_filter.reset();

        break;
      }

      break;

    case CALCULATE:
      process_data();

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
  fix16_t setpoint = 0;

  // History of measured speed. Used to detect stable values.
  fix16_t speed_log[3] = { 0, 0, fix16_minimum };
  int measure_attempts = 0;

  // Set of optimal to measurement setpoint values.
  fix16_t setpoints_preset[18] = {
    // ... hole
    F16(0.046875),
    F16(0.0625),
    F16(0.078125),
    F16(0.09375),
    F16(0.109375),
    F16(0.125),
    F16(0.140625),
    F16(0.15625),
    F16(0.171875),
    F16(0.1875),
    F16(0.203125),
    F16(0.234375),
    F16(0.265625),
    F16(0.296875),
    F16(0.328125),
    // ... hole
    F16(0.65),
    F16(0.75),
    F16(1.0)
  };

  // Array for measured setpoints values
  float setpoints[(sizeof(setpoints_preset) / sizeof(setpoints_preset[0]))];

  // Array for measured rpm values
  float rpms[(sizeof(setpoints) / sizeof(setpoints[0]))];
  int setpoint_idx = 0;

  // Approximated setpoint values.
  // 7 points used for approximation of measured data:
  // - 2 points for linear approximated interval in low-speed range
  // - 2 points for spline interpolated interval in mid-speed range
  // - 3 points in high-speed range
  // A lot more measured points is used to perform linear approxmation,
  // after which it is enough to have 2 points on linear low-speed interval.
  float setpoints_approx[7];
  // Approximated RPM values.
  float rpms_approx[7];

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

    // Speed stable if difference <= 1/3 %.
    // 1% is not always sufficient.
    return abs_diff <= abs_max / 300;
  }

  void speed_log_push(fix16_t val)
  {
    speed_log[0] = speed_log[1];
    speed_log[1] = speed_log[2];
    speed_log[2] = val;
  }

  // Max order of approximation polynomial
  enum { polynomial_order = 3 };
  // Order of approximation polynomial
  float polynomial_coeffs[polynomial_order + 1];


  // Calculates polynomial for given x and coefficients (array + order)
  float polynomial(float x, float coeffs[], int order)
  {
    float result = coeffs[0];
    for (int i = 1; i <= order; i++)
    {
      result += pow(x, i) * coeffs[i];
    }
    return result;
  }


  void process_data()
  {
    float scale_factor = fix16_to_float(median_filter.result()); // last result => max RPMs at 1.0
    // Store speed scale factor and update seosons config
    eeprom_float_write(
      CFG_REKV_TO_SPEED_FACTOR_ADDR,
      scale_factor
    );

    //
    // Generate correction table for scaled speed in [0.0..1.0] range
    // (0.0 and 1.0 points are skipped)
    //

    // Normalize setpoints to [0.0..1.0] range
    for (int i = 0; i < setpoint_idx; i++)
    {
      rpms[i] /= scale_factor;
      // clamp possible overflow at high speed
      if (rpms[i] > 1.0) rpms[i] = 1.0;
    }

    // Find last point in rpm range [0.17..0.4]
    // (for linear approximation)
    int last_linear_point = 0;
    for (int i = setpoint_idx - 1; i >= 0; i--)
    {
      if (rpms[i] <= 0.4)
      {
        last_linear_point = i;
        break;
      }
    }

    // Find first point in rpm range [0.17..0.4]
    // (for linear approximation)
    int first_linear_point = 0;
    for (int i = last_linear_point; i >= 0; i--)
    {
      if (rpms[i] < 0.17)
      {
        first_linear_point = i + 1;
        break;
      }
    }

    // Cut points below first point in rpm range [0.17..0.4]
    for (int i = 0; i < setpoint_idx - first_linear_point; i++)
    {
      setpoints[i] = setpoints[i + first_linear_point];
      rpms[i] = rpms[i + first_linear_point];
    }

    // Correct number of points after cut
    setpoint_idx -= first_linear_point;

    // Coefficients of linear approximation for
    // low-speed range
    float linear_approx_coeffs[2];

    // Linear approximation of points
    // in rpm range [0.17..0.4].
    polyfit(1, setpoints, rpms, last_linear_point - first_linear_point + 1, linear_approx_coeffs);

    // Calculate setpoint with rpm = 0
    // with linear approximation
    rpms_approx[0] = 0.0;
    setpoints_approx[0] = -linear_approx_coeffs[0] / linear_approx_coeffs[1];

    // Calculate setpoint with rpm = 0.5
    // with linear approximation
    rpms_approx[1] = 0.5;
    setpoints_approx[1] = (rpms_approx[1] - linear_approx_coeffs[0]) / linear_approx_coeffs[1];

    // Set 3 rpm and setpoint values in high-speed range to values of measured points
    rpms_approx[4] = rpms[setpoint_idx - 3];
    rpms_approx[5] = rpms[setpoint_idx - 2];
    rpms_approx[6] = rpms[setpoint_idx - 1];

    setpoints_approx[4] = setpoints[setpoint_idx - 3];
    setpoints_approx[5] = setpoints[setpoint_idx - 2];
    setpoints_approx[6] = setpoints[setpoint_idx - 1];

    // Calculate linear interpolation coefficient for helper point.
    // This coefficient is derivative of line through last point
    // and (last - 2) point.
    // This coefficient will be used in cubic spline interpolation
    // and during correction of (last - 1) point;
    float scale = (rpms[setpoint_idx - 1] - rpms[setpoint_idx - 3]) /
      (setpoints[setpoint_idx - 1] - setpoints[setpoint_idx - 3]);

    // Caclulate reasonable limits of rpm value of (last - 1) point.
    // Minimal limit is - not below line through last point
    // and (last - 2) point.
    // Maximal limit is - derivative of line through last and (last - 1)
    // points not less than 0.5 of derivative of line through
    // last and (last - 2) points. Too low derivative can make
    // speed control unstable, so we limit it.
    float middle_point_low_rpm_limit = rpms_approx[4] +
      scale * (setpoints_approx[5] - setpoints_approx[4]);
    float middle_point_high_rpm_limit = rpms_approx[6] -
      0.5 * scale * (setpoints_approx[6] - setpoints_approx[5]);

    // Correct rpm value of (last - 1) point.
    if (rpms_approx[5] < middle_point_low_rpm_limit)
    {
      rpms_approx[5] = middle_point_low_rpm_limit;
    }

    if (rpms_approx[5] > middle_point_high_rpm_limit)
    {
      rpms_approx[5] = middle_point_high_rpm_limit;
    }

    // In middle range speed calculator
    // has deviations due to nonlinear effects
    // in motor, so this points where excluded
    // from measuremets.
    // Now this points must be calculated by
    // cubic spline interpolation.
    // Spline will be built on 4 points - 2 in low-speed range,
    // first point in high-speed range and helper point, calculated
    // by linear interpolation.

    // Fill setpoints array for cubic spline calculation. Last
    // point is helper point 1/16 step forward from setpoints_approx[5].
    float spline_setpoints[] = {setpoints_approx[0], setpoints_approx[1],
      setpoints_approx[4], setpoints_approx[4] + 1.0f / 16.0f};

    // Fill rpms array for cubic spline calculation. Last
    // point is helper point calculated by linear interpolation.
    float spline_rpms[] = {rpms_approx[0], rpms_approx[1], rpms_approx[4],
      rpms_approx[4] + scale * (1.0f / 16.0f)};

    float spline_coeffs[4];
    // Calculate cubic spline to interpolate in middle-speed range.
    polyfit(3, spline_setpoints, spline_rpms, 4, spline_coeffs);

    // Calculate setpoints of 2 mid-range interpolated points.
    // Divide skipped mid-range interval into 3 parts.
    float spline_interp_setpoints_step = (setpoints_approx[4] - setpoints_approx[1]) / 3.0f;
    setpoints_approx[2] = setpoints_approx[1] + spline_interp_setpoints_step;
    setpoints_approx[3] = setpoints_approx[2] + spline_interp_setpoints_step;

    // Calculate 2 points in middle-speed range by cubic spline interpolation.
    rpms_approx[2] = polynomial(setpoints_approx[2], spline_coeffs, 3);
    rpms_approx[3] = polynomial(setpoints_approx[3], spline_coeffs, 3);

    // Build reversed interpolation table (X = rpm, Y = setpoint)
    // and store it in flash memory.
    // RPM = 0.0 and RPM = 1.0 points are skipped.
    for (int i = CFG_RPM_INTERP_TABLE_LENGTH - 1; i >= 0; i--)
    {
      float rpm = float(i + 1) / (CFG_RPM_INTERP_TABLE_LENGTH + 1);

      for (int idx = (7 - 2); idx >= 0; idx--)
      {
        if (rpms_approx[idx] < rpm) // matching range found
        {
          // count point proportion (scale) between RPM values
          float sc = (rpm - rpms_approx[idx]) / (rpms_approx[idx + 1] - rpms_approx[idx]);

          // apply to setpoints interval
          float result = setpoints_approx[idx] * (1.0 - sc) + setpoints_approx[idx + 1] * sc;

          // store
          eeprom_float_write(CFG_RPM_INTERP_TABLE_START_ADDR + i, result);
          break;
        }
      }
    }

    // Reload new config content
    sensors.configure();
    speedController.configure();
  }
};


#endif
