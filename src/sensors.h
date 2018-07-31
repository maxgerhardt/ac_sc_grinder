#ifndef __SENSORS__
#define __SENSORS__

#include <string.h>

#include "eeprom_float.h"
#include "config_map.h"
#include "fix16_math/fix16_math.h"

static inline uint32_t sum_u16(uint16_t *uint16_array, uint32_t count)
{
  uint32_t sum = 0;
  while (count--) sum += *uint16_array++;
  return sum;
}

// At 40000 Hz, 1/2 of 50Hz sine wave should take ~400 ticks to record
#define VOLTAGE_BUFFER_SIZE 800

// Size of ADC circular & temporary buffers.
#define ADC_BUFFER_SIZE 32

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
    // Do preliminary filtering of raw data + normalize result
    fetch_adc_data();

    // Poor man zero cross check (both up and down)
    if (((prev_voltage == 0) && (voltage > 0)) ||
        ((prev_voltage > 0) && (voltage == 0)))
    {
      if (once_zero_crossed) once_period_counted = true;

      once_zero_crossed = true;

      // If full half-period was counted at least once, save number of
      // ticks in half-period
      if (once_period_counted) period_in_ticks = phase_counter;

      phase_counter = 0;
    }

    // Calculate average power only if number of ticks was counted
    // in full half-period
    if (once_period_counted) power_tick();

    speed_tick();

    prev_voltage = voltage;
  }

  // Load config from emulated EEPROM
  void configure()
  {
    cfg_power_max_inv = fix16_from_float(1.0F /
      eeprom_float_read(CFG_POWER_MAX_ADDR, CFG_POWER_MAX_DEFAULT));

    cfg_motor_resistance = fix16_from_float(
      eeprom_float_read(CFG_MOTOR_RESISTANCE_ADDR, CFG_MOTOR_RESISTANCE_DEFAULT)
    );

    cfg_rpm_max_inv = fix16_from_float(
      1.0F / eeprom_float_read(CFG_RPM_MAX_ADDR, CFG_RPM_MAX_DEFAULT)
    );

    // config shunt resistance - in mOhm (divide by 1000)
    // shunt amplifier gain - 50
    cfg_shunt_resistance_inv = fix16_from_float(1.0F /
      (eeprom_float_read(CFG_SHUNT_RESISTANCE_ADDR, CFG_SHUNT_RESISTANCE_DEFAULT)
        * 50
        / 1000)
    );
  }

  // Store raw ADC data to normalized values
  void adc_raw_data_load(uint16_t adc_voltage, uint16_t adc_current,
                         uint16_t adc_knob, uint16_t adc_v_refin)
  {
    adc_voltage_circular_buf[adc_circular_buffer_head] = adc_voltage;
    adc_current_circular_buf[adc_circular_buffer_head] = adc_current;
    adc_knob_circular_buf[adc_circular_buffer_head] = adc_knob;
    adc_v_refin_circular_buf[adc_circular_buffer_head] = adc_v_refin;

    adc_circular_buffer_head++;

    if (adc_circular_buffer_head >= ADC_BUFFER_SIZE) adc_circular_buffer_head = 0;
  }

private:
  // Circular buffers for adc data, filled by "interrupt" (DMA)
  // Should be at least +1 of required data size, to guarantee atomic use
  uint16_t adc_voltage_circular_buf[ADC_BUFFER_SIZE];
  uint16_t adc_current_circular_buf[ADC_BUFFER_SIZE];
  uint16_t adc_knob_circular_buf[ADC_BUFFER_SIZE];
  uint16_t adc_v_refin_circular_buf[ADC_BUFFER_SIZE];
  // Common circular buffer HEAD offset (the same for all buffers)
  uint8_t adc_circular_buffer_head = 0;

  // Temporary buffers to quick-copy data from circular one, before use.
  //
  // 1. Converts circular data to linear one
  // 2. Helps to avoid races, if heavy processing interrupted by ADC push new data
  uint16_t adc_voltage_tmp_buf[ADC_BUFFER_SIZE];
  uint16_t adc_current_tmp_buf[ADC_BUFFER_SIZE];
  uint16_t adc_knob_tmp_buf[ADC_BUFFER_SIZE];
  uint16_t adc_v_refin_tmp_buf[ADC_BUFFER_SIZE];


  void circ_to_linear(uint16_t *src, uint16_t *dst, uint8_t head, uint8_t count)
  {
    if (head == 0) head = ADC_BUFFER_SIZE;

    if (head >= count)
    {
      // Can copy as single slice
      memcpy(dst, src + (head - count), count * sizeof(uint16_t));
    }
    else {
      // Should copy as 2 slices
      uint8_t slice1_start = head + ADC_BUFFER_SIZE - count;
      uint8_t slice1_len = count - head;
      memcpy(dst, src + slice1_start, slice1_len * sizeof(uint16_t));
      memcpy(dst + slice1_len, src, head * sizeof(uint16_t));
    }
  }

  void fetch_adc_data()
  {
    // "Lock" raw data to temporary buffers
    uint8_t frozen_head = adc_circular_buffer_head;
    circ_to_linear(adc_voltage_circular_buf, adc_voltage_tmp_buf, frozen_head, 4);
    circ_to_linear(adc_current_circular_buf, adc_current_tmp_buf, frozen_head, 4);
    circ_to_linear(adc_knob_circular_buf, adc_knob_tmp_buf, frozen_head, 4);
    circ_to_linear(adc_v_refin_circular_buf, adc_v_refin_tmp_buf, frozen_head, 4);

    // Apply filters
    uint16_t adc_voltage = sum_u16(adc_voltage_tmp_buf, 4) >> 2;
    uint16_t adc_current = sum_u16(adc_current_tmp_buf, 4) >> 2;
    uint16_t adc_knob = sum_u16(adc_knob_tmp_buf, 4) >> 2;
    uint16_t adc_v_refin =  sum_u16(adc_v_refin_tmp_buf, 4) >> 2;

    // Now process the rest...

    // 4096 - maximum value of 12-bit integer
    // normalize to fix16_t[0.0..1.0]
    fix16_t knob_new = adc_knob << 4;

    // Use additional mean smoother for knob
    knob = (knob * 15 + knob_new) >> 4;


    // Vrefin - internal reference voltage, 1.2v
    // Vref - ADC reference voltage, equal to ADC supply voltage (~ 3.3v)
    // adc_vrefin = 1.2 / Vref * 4096
    fix16_t v_ref = fix16_div(F16(1.2), adc_v_refin << 4);

    // maximum ADC input voltage - Vref
    // current = adc_current_norm * v_ref / cfg_shunt_resistance
    current = fix16_mul(
      fix16_mul(adc_current << 4, cfg_shunt_resistance_inv),
      v_ref
    );

    // resistors in voltage divider - [ 2*150 kOhm, 1.5 kOhm ]
    // (divider ratio => 201)
    // voltage = adc_voltage * v_ref * (301.5 / 1.5);
    voltage = fix16_mul(fix16_mul(adc_voltage << 4, v_ref), F16(301.5/1.5));

  }

  // Conig info
  fix16_t cfg_shunt_resistance_inv;
  fix16_t cfg_power_max_inv;
  fix16_t cfg_motor_resistance;
  fix16_t cfg_rpm_max_inv;

  // Buffer for extrapolation during the negative half-period of AC voltage
  // Record data on positive wave and replay on negative wave.
  fix16_t voltage_buffer[VOLTAGE_BUFFER_SIZE];

  fix16_t p_sum = 0;

  // Holds number of ticks during the period
  // Used to calculate the average power for the period
  uint32_t power_tick_counter = 0;

  // Holds number of tick when voltage crosses zero
  // Used to make the extrapolation during the interval
  // when voltage is negative
  uint32_t voltage_zero_cross_tick_count = 0;

  // Previous iteration values. Used to detect zero cross.
  fix16_t prev_voltage = 0;
  fix16_t prev_current = 0;

  uint32_t phase_counter = 0; // increment every tick
  // Holds the number of ticks per half-period (between two zero crosses)
  // Will be near 400 for 50 Hz supply voltage or near 333.3333 for 60 Hz
  // Initial value -1 prevents triac from turning on during first period
  uint32_t period_in_ticks = 0;


  bool once_zero_crossed = false;
  bool once_period_counted = false;

  void power_tick()
  {
    // TODO: should detect & use phase shift

    // Positive sine wave
    if ((current > 0) && (voltage > 0))
    {
      p_sum += fix16_mul(voltage, current);
      power_tick_counter++;
    }
    // Negative sine vave => extrapolate voltage
    else if (voltage == 0)
    {
      // If this is tick when voltage crosses zero (down), save tick number
      if (prev_voltage > 0)
      {
        voltage_zero_cross_tick_count = power_tick_counter;
      }

      if (current > 0)
      {
        // Now voltage is negative, but current is still positive
        // Inductance gives power back to the supply
        // This power must be substracted from power sum
        fix16_t extrapolated_voltage = voltage_buffer[power_tick_counter - voltage_zero_cross_tick_count];

        p_sum -= fix16_mul(extrapolated_voltage, current);
        power_tick_counter++;
      }
    }

    if ((prev_current > 0) && (current == 0))
    {
      // Now we are at negative wave and shunt current ended
      // Time to calculate average power, and normalize it to [0.0..1.0]
      //
      // Normalized power = (p_sum / period_in_ticks) / cfg_power_max;
      // power > 1.0 => overload
      //
      // protect from zero div (that should never happen)
      if (period_in_ticks > 0)
      {
        power = fix16_mul(p_sum / period_in_ticks, cfg_power_max_inv);
      }
      power_tick_counter = 0;
    }

    // We should never have bufer overrun, but let's keep protection from
    // memory corruption for safety.
    if (power_tick_counter < VOLTAGE_BUFFER_SIZE)
    {
      voltage_buffer[power_tick_counter] = voltage;
    }

    prev_current = current;
  }

  // Motor speed is proportional to the equivalent resistance `r_ekv`.
  // `r_ekv_sum` holds sum of calculated on each tick `r_ekv`
  // At the end of the period, arithmetic mean of `r_ekv_sum`
  // is calculated for noise reduction purpose.
  fix16_t r_ekv_sum = 0;
  // Holds number of ticks
  uint32_t speed_tick_counter = 0;


  void speed_tick()
  {
    if (voltage > 0)
    {
      if (current > 0)
      {
        // r_ekv = voltage/current - cfg_motor_resistance;
        r_ekv_sum += fix16_div(voltage, current) - cfg_motor_resistance;
        speed_tick_counter++;
      }
    }

    if ((prev_voltage > 0) && (voltage == 0))
    {
      // Now we are at negative wave, calculate [normalized] speed

      // protect from zero div (that should never happen)
      if (speed_tick_counter > 0)
      {
        fix16_t r_ekv = r_ekv_sum / speed_tick_counter;
        // TODO: hardcoded 10 => ?
        // speed = 10.0 * r_ekv / cfg_rpm_max;
        speed = fix16_mul(10 * r_ekv, cfg_rpm_max_inv);
      }
      speed_tick_counter = 0;
    }
  }
};


#endif
