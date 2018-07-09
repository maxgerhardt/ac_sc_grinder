#ifndef __SENSORS__
#define __SENSORS__


#include "eeprom_float.h"
#include "config_map.h"

/*
  Sensors data source:

  - voltage: physical, immediate
  - current: physical, from shunt, immediate (but can have phase shift)
  - power: calculated
  - speed: calculated
*/
class Sensors
{
public:

  float power = 0.0;
  float speed = 0.0;
  float voltage = 0.0;
  float current = 0.0;

  float prev_voltage = 0.0;

  // Conig info
  float cfg_power_max;
  float cfg_motor_resistance;
  float cfg_rpm_max;

  // Should be called with 40kHz frequency
  void tick()
  {
    power_tick();
    speed_tick();
  }

  // Load config from emulated EEPROM
  void configure()
  {
    cfg_power_max = eeprom_float_read(CFG_POWER_MAX_ADDR, CFG_POWER_MAX_DEFAULT);
    cfg_motor_resistance = eeprom_float_read(CFG_MOTOR_RESISTANCE_ADDR, CFG_MOTOR_RESISTANCE_DEFAULT);
    cfg_rpm_max = eeprom_float_read(CFG_RPM_MAX_ADDR, CFG_RPM_MAX_DEFAULT);
  }

private:

  float p_sum = 0.0;
  int power_tick_counter = 0;
  // for extrapolation during the negative half-period of voltage
  float voltage_buffer[1024];
  int power_back_tick_counter = 0;

  float prev_current = 0.0;

  void power_tick()
  {
    // TODO: should detect & use phase shift
    if ((current > 0.0) && (voltage > 0.0))
    {
      p_sum += voltage*current;
      power_tick_counter++;
    }
    // voltage is negative - make the extrapolation
    else if ((current > 0.0) && (voltage == 0.0))
    {
      p_sum -= voltage_buffer[power_back_tick_counter] * current;
      power_back_tick_counter++;
      power_tick_counter++;
    }

    if ((prev_current > 0.0) && (current == 0.0))
    {
      power = p_sum / power_tick_counter / cfg_power_max * 100.0;
    }

    for (int i=0;i<1024-1;i++)
      voltage_buffer[i+1] = voltage_buffer[i];

    voltage_buffer[0] = voltage;
    prev_current = current;
  }


  float r_ekv_sum = 0.0;
  int speed_tick_counter = 0;


  void speed_tick()
  {
    if ((current > 0.0) && (voltage > 0.0))
    {
      r_ekv_sum += voltage/current - cfg_motor_resistance;
      speed_tick_counter++;
    }

    if ((prev_voltage > 0.0) && (voltage == 0.0))
    {
      float r_ekv = r_ekv_sum / speed_tick_counter;
      speed = 10.0 * r_ekv / cfg_rpm_max;
      speed_tick_counter = 0.0;
    }

    prev_voltage = voltage;
  }
};


#endif