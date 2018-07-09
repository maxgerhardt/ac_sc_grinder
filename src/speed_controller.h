class SpeedController
{
public:

  float cfg_dead_zone_width;
  float cfg_pid_p;
  float cfg_pid_i;
  float cfg_rpm_max_limit;
  float cfg_rpm_min_limit;
  float cfg_rpm_max;

  float setpoint = 0.0;

  void tick(float potentiometer, float speed, float power)
  {
    float control_speed = calculateControlSpeed(potentiometer,speed,lock);
    float control_power = calculateControlPower(power);
    setpoint = minControl(control_speed,control_power);
  }

private:

  float PID_speed_integral = 0;
  float PID_power_integral = 0;
  bool lock = false;

  float calculateControlSpeed(float potentiometer, float speed, bool lock)
  {
    if (!lock)
    {
      if (potentiometer < cfg_dead_zone_width)
        potentiometer = 0.0;

      float error = potentiometer - speed;
      float proportional = cfg_pid_p*error;

      PID_speed_integral += 1.0 / cfg_pid_i * error;

      if (PID_speed_integral > cfg_rpm_max_limit / cfg_rpm_max * 100.0)
      PID_speed_integral = cfg_rpm_max_limit / cfg_rpm_max * 100.0;

      if (PID_speed_integral < cfg_rpm_min_limit / cfg_rpm_max * 100.0)
      PID_speed_integral = cfg_rpm_min_limit / cfg_rpm_max * 100.0;

      float output = proportional + PID_speed_integral;

      if (output > cfg_rpm_max_limit / cfg_rpm_max * 100.0)
      output = cfg_rpm_max_limit / cfg_rpm_max * 100.0;

      if (output < cfg_rpm_min_limit / cfg_rpm_max * 100.0)
      output = cfg_rpm_min_limit / cfg_rpm_max * 100.0;

      return output;
    }
    return 0.0;
  }

  float calculateControlPower(float power)
  {
    float error = 100.0 - power;
    float proportional = cfg_pid_p * error;

    PID_power_integral += 1.0 / cfg_pid_i * error;

    if (PID_power_integral > 100.0)
      PID_power_integral = 100.0;

    if (PID_power_integral < 0.0)
      PID_power_integral = 0.0;

    float output = proportional + PID_power_integral;

    if (output > 100.0)
      output = 100.0;

    if (output < 0.0)
      output = 0.0;

    return output;
  }

  float minControl(float control_speed, float control_power)
  {
    if (control_speed <= control_power)
    {
      lock = false;
      return control_speed;
    }
    else
    {
      lock = true;
      return control_power;
    }
  }
};
