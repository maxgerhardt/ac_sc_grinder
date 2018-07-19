#ifndef __SPEED_CONTROLLER__
#define __SPEED_CONTROLLER__


#include "eeprom_float.h"
#include "config_map.h"
#include "fix16_math/fix16_math.h"


class SpeedController
{
public:
  // Inputs
  fix16_t in_knob = 0;  // Knob position
  fix16_t in_speed = 0; // Measured speed
  fix16_t in_power = 0; // Measured power

  // Output power 0..100% for triac control
  fix16_t out_power = 0;

  // Expected be called with 100/120Hz frequency
  // More frequent calls are useless, because we can not control triac faster

  // Contains two PIDs. pid_speed used in normal mode, pid_power - in power limit mode.
  // When motor power exceeds the limit, the pid_power output drops below
  // pid_speed output.
  void tick()
  {
    knob_normalized = normalize_knob(in_knob);

    if (!power_limit)
    {
      pid_speed_out = speed_pid_tick();
    }

    fix16_t pid_power_out = power_pid_tick();

    // TODO: check logic, isn't pid_power_out in [0..1]?
    if (pid_speed_out <= pid_power_out)
    {
      if (power_limit)
      {
        // Recalculate PID_speed_integral to ensure smooth switch to normal mode
        pid_speed_integral = pid_speed_out -
          fix16_mul((knob_normalized - in_speed), cfg_pid_p);
        power_limit = false;
      }
      out_power = pid_speed_out;
    }
    else
    {
      power_limit = true;
      out_power = pid_power_out;
    }
  }

  // Load config from emulated EEPROM
  void configure()
  {
    cfg_dead_zone_width_norm = fix16_from_float(eeprom_float_read(CFG_DEAD_ZONE_WIDTH_ADDR,
       CFG_DEAD_ZONE_WIDTH_DEFAULT) / 100.0);
    cfg_pid_p = fix16_from_float(eeprom_float_read(CFG_PID_P_ADDR,
       CFG_PID_P_DEFAULT));
    cfg_pid_i_inv = fix16_from_float(1.0 / eeprom_float_read(CFG_PID_I_ADDR,
       CFG_PID_P_DEFAULT));
    cfg_rpm_max_limit = fix16_from_float(eeprom_float_read(CFG_RPM_MAX_LIMIT_ADDR,
       CFG_RPM_MAX_LIMIT_DEFAULT));
    cfg_rpm_min_limit = fix16_from_float(eeprom_float_read(CFG_RPM_MIN_LIMIT_ADDR,
       CFG_RPM_MIN_LIMIT_DEFAULT));
    cfg_rpm_max = fix16_from_float(eeprom_float_read(CFG_RPM_MAX_ADDR,
       CFG_RPM_MAX_DEFAULT));

    out_min_clamp_norm = fix16_clamp_zero_one(
      fix16_div(cfg_rpm_min_limit, cfg_rpm_max)
    );
    out_max_clamp_norm = fix16_clamp_zero_one(
      fix16_div(cfg_rpm_max_limit, cfg_rpm_max)
    );
    knob_norm_coeff =  fix16_div(
      out_max_clamp_norm - out_min_clamp_norm,
      fix16_one - cfg_dead_zone_width_norm
    );
  }

private:
  // Control dead zone width near 0, when motor should not run.
  fix16_t cfg_dead_zone_width_norm;
  // PID coefficients
  fix16_t cfg_pid_p;
  fix16_t cfg_pid_i_inv;
  // In theory limits should be in % if max, but it's more convenient
  // for users to work with direct RPM values.
  fix16_t cfg_rpm_max_limit;
  fix16_t cfg_rpm_min_limit;
  fix16_t cfg_rpm_max;

  // Cache for clamping limits & normalization, calculated on config load
  fix16_t out_min_clamp_norm = 0;
  fix16_t out_max_clamp_norm = 1;
  fix16_t knob_norm_coeff = 1;


  fix16_t pid_speed_integral = 0;
  fix16_t pid_power_integral = 0;
  fix16_t pid_speed_out = 0;
  bool power_limit = false;

  // knob value normalized to range (cfg_rpm_min_limit..cfg_rpm_max_limit)
  fix16_t knob_normalized;

  // Apply min/max limits to knob output
  fix16_t normalize_knob(fix16_t knob)
  {
    if (in_knob < cfg_dead_zone_width_norm) return 0;

    return fix16_mul(
      (in_knob - cfg_dead_zone_width_norm),
      knob_norm_coeff
    ) + out_min_clamp_norm;
  }


  fix16_t speed_pid_tick()
  {
    fix16_t divergence = knob_normalized - in_speed;

    // TODO: ???? cfg_pid_i = 0 => result = infinity
    // pid_speed_integral += 1.0 / cfg_pid_i * divergence;
    fix16_t tmp = pid_speed_integral + fix16_mul(cfg_pid_i_inv, divergence);
    pid_speed_integral = fix16_clamp(tmp, out_min_clamp_norm, out_max_clamp_norm);

    fix16_t proportional = fix16_mul(cfg_pid_p, divergence);

    return fix16_clamp(
      proportional + pid_speed_integral,
      out_min_clamp_norm,
      out_max_clamp_norm
    );
  }

  fix16_t power_pid_tick()
  {
    // float divergence = 100.0 - in_power;
    fix16_t divergence = fix16_one - in_power;

    // TODO: ???? cfg_pid_i = 0 => result = infinity
    // pid_power_integral += 1.0 / cfg_pid_i * divergence;
    fix16_t tmp = pid_power_integral + fix16_mul(cfg_pid_i_inv, divergence);
    pid_power_integral = fix16_clamp_zero_one(tmp);

    fix16_t proportional = fix16_mul(cfg_pid_p, divergence);

    return fix16_clamp_zero_one(proportional + pid_power_integral);
  }
};


#endif
