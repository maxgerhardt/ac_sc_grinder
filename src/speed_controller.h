#ifndef __SPEED_CONTROLLER__
#define __SPEED_CONTROLLER__


#include "app.h"
#include "eeprom_float.h"
#include "config_map.h"
#include "fix16_math/fix16_math.h"

// PID iteration frequency, Hz. To fit math in fix16 without overflow.
// Affects Ki "scale" of PID
#define APP_PID_FREQUENCY 40


constexpr int freq_divisor = APP_TICK_FREQUENCY / APP_PID_FREQUENCY;


class SpeedController
{
public:
  // Inputs
  fix16_t in_knob = 0;  // Knob position [0.0..1.0]
  fix16_t in_speed = 0; // Measured speed [0.0..1.0]

  // Output power [0..1] for triac control
  fix16_t out_power = 0;

  // 2 PIDs inside, but only one is active:
  //
  // - `pid_speed` used in normal mode
  // - `pid_limiter` used in current limit mode
  //
  // When motor current exceeds the limit, the `pid_limiter` output drops below
  // `pid_speed` output.
  //
  void tick()
  {
    // Downscale input frequency to avoid fixed poind overflow.
    // 40000Hz => 40Hz

    if (tick_freq_divide_counter >= freq_divisor) tick_freq_divide_counter = 0;

    if (tick_freq_divide_counter > 0) {
      tick_freq_divide_counter++;
      return;
    }

    tick_freq_divide_counter++;


    knob_normalized = normalize_knob(in_knob);

    if (!limiter_active)
    {
      pid_speed_out = speed_pid_tick();
    }

    fix16_t pid_limiter_out = limiter_pid_tick();

    if (pid_speed_out <= pid_limiter_out)
    {
      if (limiter_active)
      {
        // Recalculate `pid_speed_integral` to ensure smooth output change
        // after switch to normal mode
        pid_speed_integral = pid_speed_out - fix16_mul((knob_normalized - in_speed), cfg_pid_p);
        limiter_active = false;
      }
      out_power = pid_speed_out;
    }
    else
    {
      limiter_active = true;
      out_power = pid_limiter_out;
    }
  }

  // Load config from emulated EEPROM
  void configure()
  {
    cfg_dead_zone_width_norm = fix16_from_float(eeprom_float_read(CFG_DEAD_ZONE_WIDTH_ADDR,
       CFG_DEAD_ZONE_WIDTH_DEFAULT) / 100.0F);

    cfg_pid_p = fix16_from_float(eeprom_float_read(CFG_PID_P_ADDR,
       CFG_PID_P_DEFAULT));

    // CFG_PID_I in seconds, reverse and divide by tick frequency (40 Hz)
    cfg_pid_i_inv = fix16_from_float(
      1.0F
      / eeprom_float_read(CFG_PID_I_ADDR, CFG_PID_I_DEFAULT)
      / APP_PID_FREQUENCY
    );

    float _rpm_max = eeprom_float_read(CFG_RPM_MAX_ADDR, CFG_RPM_MAX_DEFAULT);

    cfg_rpm_max_limit_norm = fix16_from_float(
      eeprom_float_read(CFG_RPM_MAX_LIMIT_ADDR, CFG_RPM_MAX_LIMIT_DEFAULT) / _rpm_max
    );

    float _rpm_min_limit = eeprom_float_read(CFG_RPM_MIN_LIMIT_ADDR, CFG_RPM_MIN_LIMIT_DEFAULT);
    // Don't allow too small low limit
    // ~ 3000 for 35000 max limit
    if (_rpm_min_limit < _rpm_max * 0.085) _rpm_min_limit = _rpm_max * 0.085;

    cfg_rpm_min_limit_norm = fix16_from_float(_rpm_min_limit / _rpm_max);

    knob_norm_coeff =  fix16_div(
      cfg_rpm_max_limit_norm - cfg_rpm_min_limit_norm,
      fix16_one - cfg_dead_zone_width_norm
    );
  }

private:
  // Control dead zone width near 0, when motor should not run.
  fix16_t cfg_dead_zone_width_norm;
  // PID coefficients
  fix16_t cfg_pid_p;
  fix16_t cfg_pid_i_inv;
  // Config limits are now in normalized [0.0..1.0] form of max motor RPM.
  fix16_t cfg_rpm_max_limit_norm;
  fix16_t cfg_rpm_min_limit_norm;

  // Cache for knob normalization, calculated on config load
  fix16_t knob_norm_coeff = F16(1);
  // knob value normalized to range (cfg_rpm_min_limit..cfg_rpm_max_limit)
  fix16_t knob_normalized;


  fix16_t pid_speed_integral = 0;
  fix16_t pid_limiter_integral = 0;
  fix16_t pid_speed_out = 0;
  bool limiter_active = false;

  uint32_t tick_freq_divide_counter = 0;

  // Apply min/max limits to knob output
  fix16_t normalize_knob(fix16_t knob)
  {
    if (in_knob < cfg_dead_zone_width_norm) return 0;

    return fix16_mul(
      (in_knob - cfg_dead_zone_width_norm),
      knob_norm_coeff
    ) + cfg_rpm_min_limit_norm;
  }


  fix16_t speed_pid_tick()
  {
    fix16_t divergence = knob_normalized - in_speed;

    // pid_speed_integral += (1.0 / cfg_pid_i) * divergence;
    fix16_t tmp = pid_speed_integral + fix16_mul(cfg_pid_i_inv, divergence);
    pid_speed_integral = fix16_clamp(tmp, cfg_rpm_min_limit_norm, cfg_rpm_max_limit_norm);

    fix16_t proportional = fix16_mul(cfg_pid_p, divergence);

    return fix16_clamp(
      proportional + pid_speed_integral,
      cfg_rpm_min_limit_norm,
      cfg_rpm_max_limit_norm
    );
  }

  fix16_t limiter_pid_tick()
  {
    // Not implemented yet
    return F16(1.0);
    /*fix16_t divergence = fix16_one - in_current_average;

    // pid_limiter_integral += (1.0 / cfg_pid_i) * divergence;
    fix16_t tmp = pid_limiter_integral + fix16_mul(cfg_pid_i_inv, divergence);
    pid_limiter_integral = fix16_clamp_zero_one(tmp);

    fix16_t proportional = fix16_mul(cfg_pid_p, divergence);

    return fix16_clamp_zero_one(proportional + pid_limiter_integral);
    */
  }
};


#endif
