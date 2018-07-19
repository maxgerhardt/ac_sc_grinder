#ifndef __SENSORS__
#define __SENSORS__


#include "eeprom_float.h"
#include "config_map.h"
#include "fix16_math/fix16_math.h"

// At 40000 Hz, 1/2 of sine wave at 50Hz should take ~400 ticks to record
#define VOLTAGE_BUFFER_SIZE 800

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

  fix16_t power = 0;
  fix16_t speed = 0;
  fix16_t voltage = 0;
  fix16_t current = 0;
  fix16_t knob = 0; // Speed knob physical value, 0..100%

  // Should be called with 40kHz frequency
  void tick()
  {
    power_tick();
    speed_tick();
  }

  // Load config from emulated EEPROM
  void configure()
  {
    fix16_cfg_power_max_inv = fix16_from_float(1.0 /
      eeprom_float_read(CFG_POWER_MAX_ADDR, CFG_POWER_MAX_DEFAULT));
    fix16_cfg_motor_resistance = fix16_from_float(
      eeprom_float_read(CFG_MOTOR_RESISTANCE_ADDR, CFG_MOTOR_RESISTANCE_DEFAULT)
    );
    fix16_cfg_rpm_max_inv = fix16_from_float(
      1.0 / eeprom_float_read(CFG_RPM_MAX_ADDR, CFG_RPM_MAX_DEFAULT)
    );
    // config shunt resistance - in mOhm (divide by 1000)
    // shunt amplifier gain - 50
    fix16_cfg_shunt_resistance_inv = fix16_from_float(1.0 /
      (eeprom_float_read(CFG_SHUNT_RESISTANCE_ADDR, CFG_SHUNT_RESISTANCE_DEFAULT)
        * 50.0
        / 1000.0)
    );
  }

  // Store raw ADC data to normalized values
  void adc_raw_data_load(uint16_t adc_voltage, uint16_t adc_current,
                         uint16_t adc_knob, uint16_t adc_v_refin)
  {
    // Vrefin is internal reference voltage 1.2v
    // Vref is ADC reference voltage, equal to ADC supply voltage (near 3.3v)
    // adc_vrefin = 1.2 / Vref * 4096
    // float v_ref = 1.2 * 4096 / adc_v_refin;

    fix16_t fix16_v_ref = fix16_div(F16(1.2), adc_v_refin << 2);

    // 4096 - maximum value of 12-bit integer
    // normalize to fix16_t[0.0..1.0]
    knob = adc_knob << 2;

    // maximum ADC input voltage - Vref
    // current = fix16_adc_current_norm * fix16_v_ref / cfg_shunt_resistance

    current = fix16_mul(
      fix16_mul(adc_current << 2, fix16_cfg_shunt_resistance_inv),
      fix16_v_ref
    );

    // resistors in voltage divider - [ 2*150 kOhm, 1.5 kOhm ]
    // (divider ratio => 201)
    // voltage = fix16_adc_voltage * fix16_v_ref * (301.5 / 1.5);
    voltage = fix16_mul(
      fix16_mul(adc_voltage << 2, fix16_v_ref),
      F16(301.5/1.5)
    );
  }

private:
  // Conig info
  fix16_t fix16_cfg_shunt_resistance_inv;
  fix16_t fix16_cfg_power_max_inv;
  fix16_t fix16_cfg_motor_resistance;
  fix16_t fix16_cfg_rpm_max_inv;

  // Buffer for extrapolation during the negative half-period of AC voltage
  // Record data on positive wave and replay on negative wave.
  fix16_t fix16_voltage_buffer[VOLTAGE_BUFFER_SIZE];

  fix16_t fix16_p_sum = 0;

  // Holds number of ticks during the period
  // Used to calculate the average power for the period
  int power_tick_counter = 0;

  // Holds number of tick when voltage crosses zero
  // Used to make the extrapolation during the interval
  // when voltage is negative
  int voltage_zero_cross_tick_count = 0;

  // Previous iteration values. Used to detect zero cross.
  fix16_t fix16_prev_voltage = 0;
  fix16_t fix16_prev_current = 0;

  void power_tick()
  {
    // TODO: should detect & use phase shift

    // Positive sine wave
    if ((current > 0) && (voltage > 0))
    {
      fix16_p_sum += fix16_mul(voltage, current);
      power_tick_counter++;
    }
    // Negative sine vave => extrapolate voltage
    else if ((current > 0) && (voltage == 0))
    {
      // If this is tick when voltage crosses zero (down), save tick number
      if (fix16_prev_voltage > 0)
      {
        voltage_zero_cross_tick_count = power_tick_counter;
      }

      // Now voltage is negative, but current is still positive
      // Inductance gives power back to the supply
      // This power must be substracted from power sum
      fix16_t extrapolated_voltage = fix16_voltage_buffer[power_tick_counter - voltage_zero_cross_tick_count];

      fix16_p_sum -= fix16_mul(extrapolated_voltage, current);
      power_tick_counter++;
    }

    if ((fix16_prev_current > 0) && (current == 0))
    {
      // Now we are at negative wave and shunt current ended
      // Time to calculate average power, and convert it to % or `cfg_power_max`

      // TODO: fix logic
      // power = p_sum / power_tick_counter / cfg_power_max * 100.0;
      power = fix16_mul(fix16_p_sum / power_tick_counter, fix16_cfg_power_max_inv);
      power_tick_counter = 0;
    }

    // We should never have bufer overrun, but let's keep protection from
    // memory corruption for safety.
    if (power_tick_counter < VOLTAGE_BUFFER_SIZE)
    {
      fix16_voltage_buffer[power_tick_counter] = voltage;
    }
    fix16_prev_current = current;
  }

  // Motor speed is proportional to the equivalent resistance `r_ekv`.
  // `r_ekv_sum` holds sum of calculated on each tick `r_ekv`
  // At the end of the period, arithmetic mean of `r_ekv_sum`
  // is calculated for noise reduction purpose.
  fix16_t fix16_r_ekv_sum = 0;
  // Holds number of ticks
  int speed_tick_counter = 0;


  void speed_tick()
  {
    if ((current > 0) && (voltage > 0))
    {
      // r_ekv_sum += voltage/current - cfg_motor_resistance;
      fix16_r_ekv_sum += fix16_div(voltage, current) - fix16_cfg_motor_resistance;
      speed_tick_counter++;
    }

    if ((fix16_prev_voltage > 0) && (voltage == 0))
    {
      // float r_ekv = r_ekv_sum / speed_tick_counter;
      fix16_t fix16_r_ekv = fix16_r_ekv_sum / speed_tick_counter;
      // TODO: hardcoded 10 => ?
      // speed = 10.0 * r_ekv / cfg_rpm_max;
      speed = fix16_mul(10 * fix16_r_ekv, fix16_cfg_rpm_max_inv);
      speed_tick_counter = 0;
    }

    fix16_prev_voltage = voltage;
  }
};


#endif
