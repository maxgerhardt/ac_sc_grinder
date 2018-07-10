#ifndef __SPEED_CONTROLLER__
#define __SPEED_CONTROLLER__


#include "eeprom_float.h"
#include "config_map.h"
#include "utils.h"


class SpeedController
{
public:
  // Inputs
  float in_knob = 0.0;  // Knob position
  float in_speed = 0.0; // Measured speed
  float in_power = 0.0; // Measured power

  // Output power 0..100% for triac control
  float out_power = 0.0;

  // Expected be called with 100/120Hz frequency
  // More frequent calls are useless, because we can not control triac faster
  void tick()
  {
    if (!lock)
    {
      pid_speed_out = speed_pid_tick();
    }

    float pid_power_out = power_pid_tick();

    if (pid_speed_out <= pid_power_out)
    {
      lock = false;
      out_power = pid_speed_out;
    }
    else
    {
      lock = true;
      out_power = pid_power_out;
    }
  }

  // Load config from emulated EEPROM
  void configure()
  {
    cfg_dead_zone_width = eeprom_float_read(CFG_DEAD_ZONE_WIDTH_ADDR, CFG_DEAD_ZONE_WIDTH_DEFAULT);
    cfg_pid_p = eeprom_float_read(CFG_PID_P_ADDR, CFG_PID_P_DEFAULT);
    cfg_pid_i = eeprom_float_read(CFG_PID_I_ADDR, CFG_PID_P_DEFAULT);
    cfg_rpm_max_limit = eeprom_float_read(CFG_RPM_MAX_LIMIT_ADDR, CFG_RPM_MAX_LIMIT_DEFAULT);
    cfg_rpm_min_limit = eeprom_float_read(CFG_RPM_MIN_LIMIT_ADDR, CFG_RPM_MIN_LIMIT_DEFAULT);
    cfg_rpm_max = eeprom_float_read(CFG_RPM_MAX_ADDR, CFG_RPM_MAX_DEFAULT);

    out_min_clamp = cfg_rpm_min_limit / cfg_rpm_max * 100.0;
    out_max_clamp = cfg_rpm_max_limit / cfg_rpm_max * 100.0;
  }

private:
  // Control dead zone width near 0, when motor should not run.
  float cfg_dead_zone_width;
  // PID coefficients
  float cfg_pid_p;
  float cfg_pid_i;
  // In theory limits should be in % if max, but it's more convenient
  // for users to work with direct RPM values.
  float cfg_rpm_max_limit;
  float cfg_rpm_min_limit;
  float cfg_rpm_max;

  // Cache for clamping limits, calculated on config load
  float out_min_clamp = 0.0;
  float out_max_clamp = 100.0;


  float PID_speed_integral = 0;
  float PID_power_integral = 0;
  float pid_speed_out = 0;
  bool lock = false;

  float speed_pid_tick()
  {
    // TODO: seems missed range normalization (rpm_min..rpm_max)
    float knob;
    if (in_knob < cfg_dead_zone_width)
    {
      knob = 0.0;
    }
    else
    {
      knob = clamp(in_knob, out_min_clamp, out_max_clamp);
    }

    float divergence = knob - in_speed;

    PID_speed_integral += 1.0 / cfg_pid_i * divergence;
    PID_speed_integral = clamp(PID_speed_integral, out_min_clamp, out_max_clamp);

    float proportional = cfg_pid_p * divergence;

    return clamp(proportional + PID_speed_integral, out_min_clamp, out_max_clamp);
  }

  float power_pid_tick()
  {
    float divergence = 100.0 - in_power;

    PID_power_integral += 1.0 / cfg_pid_i * divergence;
    PID_power_integral = clamp(PID_power_integral, 0.0, 100.0);

    float proportional = cfg_pid_p * divergence;

    return clamp(proportional + PID_power_integral, 0.0, 100.0);
  }
};


#endif
