#ifndef __CALIBRATOR_SPEED_SCALE__
#define __CALIBRATOR_SPEED_SCALE__

// Run motor at max speed and measure speed scaling factor.
// Max speed should have 1.0 at sensorss output.

#include "../fix16_math/fix16_math.h"

#include "../app.h"
#include "../sensors.h"
#include "../triac_driver.h"

extern Sensors sensors;
extern TriacDriver triacDriver;

constexpr int calibrator_motor_startup_ticks = 3 * APP_TICK_FREQUENCY;

class CalibratorSpeedScale
{
public:

  bool tick() {
    fix16_t setpoint;

    switch (state) {

    case INIT:
      // Reset scaling factor
      sensors.cfg_rekv_to_speed_factor = fix16_one;

      max_speed = 0;
      median_filter.reset();

      set_state(START);
      break;

    // Gently run motor at max speed, in 3 sec
    case START:
      // Change setpoint from 0.0 to 1.0 in 3 seconds
      setpoint = fix16_div(
        fix16_from_int(ticks_cnt * 100 / calibrator_motor_startup_ticks),
        F16(100)
      );
      triacDriver.setpoint = setpoint;
      triacDriver.tick();

      // 3 secs ticked => continue with next state
      if (ticks_cnt++ >= calibrator_motor_startup_ticks) set_state(MEASURE);

      break;

    // Wait until speed not increaze 20 consequent waves (~ 0.4s)
    case WAIT_MAX:
      // Continue run at max speed
      triacDriver.setpoint = fix16_one;
      triacDriver.tick();

      if (!sensors.zero_cross_up) break;

      // If current max not exceeded - count attempts
      if (sensors.speed < max_speed) {
        ticks_cnt++;
        if (ticks_cnt > 20) set_state(MEASURE);
        break;
      }

      // If new max found - reset counter and try again
      max_speed = sensors.speed;
      ticks_cnt = 0;
      break;

    case MEASURE:
      // Continue run at max speed
      triacDriver.setpoint = fix16_one;
      triacDriver.tick();

      // Measure speed once per wave.
      if (!sensors.zero_cross_up) break;

      // Collect data and count attempts
      median_filter.add(sensors.speed);

      if (ticks_cnt++ < 32) break;

      // Save data to EEPROM and update sensors config
      eeprom_float_write(
        CFG_REKV_TO_SPEED_FACTOR_ADDR,
        fix16_to_float(median_filter.result())
      );
      sensors.cfg_rekv_to_speed_factor = sensors.speed;
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
    START,
    WAIT_MAX,
    MEASURE,
    STOP
  } state = INIT;

  fix16_t max_speed = 0;

  int ticks_cnt = 0;

  MedianIteratorTemplate<fix16_t, 32> median_filter;

  void set_state(State st)
  {
    state = st;
    ticks_cnt = 0;
  }
};


#endif
