#ifndef __CALIBRATOR_SPEED_SCALE__
#define __CALIBRATOR_SPEED_SCALE__

// Run motor at max speed and measure speed scaling factor.
// Max speed should have 1.0 at sensorss output.

#include <math.h>

#include "../fix16_math/fix16_math.h"

#include "../app.h"
#include "../sensors.h"
#include "../triac_driver.h"

extern Sensors sensors;
extern TriacDriver triacDriver;

constexpr int calibrator_motor_startup_ticks = 3 * APP_TICK_FREQUENCY;
constexpr int clibrator_motor_measure_ticks = 0.2 * APP_TICK_FREQUENCY;

class CalibratorSpeedScale
{
public:

  bool tick() {
    fix16_t setpoint;

    switch (state) {

    // Gently run motor at max speed, in 3 sec
    case START:
      // Reset scaling factor
      sensors.cfg_rekv_to_speed_factor = fix16_one;
      prev_speed = 0;

      // Change setpoint from 0.0 to 1.0 in 3 seconds
      setpoint = fix16_div(
        fix16_from_int(ticks_cnt * 100 / calibrator_motor_startup_ticks),
        F16(100)
      );
      triacDriver.voltage = sensors.voltage;
      triacDriver.setpoint = setpoint;
      triacDriver.tick();

      // 3 secs ticked => continue with next state
      if (ticks_cnt++ >= calibrator_motor_startup_ticks) set_state(MEASURE);

      break;

    // Wait until speed deviation < 3% in 0.2 sec
    case MEASURE:
      // Continue run at max speed
      triacDriver.voltage = sensors.voltage;
      triacDriver.setpoint = fix16_one;
      triacDriver.tick();

      // 0.2 secs ticked => continue on success or try again on fail.
      if (ticks_cnt++ >= clibrator_motor_measure_ticks) {
        // Keep only integer part, it's about 500-1000 for small
        // motors, and less for more powerful (but > 100 anyway)
        int current_speed = fix16_to_int(sensors.speed);

        if (abs(current_speed - prev_speed) * 100 / current_speed < 3)
        {
          // Speed deviation < 3% => max speed reached!
          // Save data to EEPROM and update sensors config
          eeprom_float_write(CFG_REKV_TO_SPEED_FACTOR_ADDR, fix16_to_float(sensors.speed));
          sensors.cfg_rekv_to_speed_factor = sensors.speed;
          set_state(STOP);
        }
        else
        {
          // Speed is not stable => try again
          set_state(MEASURE);
          // Store value for next compare
          prev_speed = current_speed;
        }
      }

      break;

    // Motor off and wait 1 sec
    case STOP:
      triacDriver.voltage = sensors.voltage;
      triacDriver.setpoint = 0;
      triacDriver.tick();

      if (ticks_cnt++ > 1 * APP_TICK_FREQUENCY) {
        set_state(START);
        return true;
      }

      break;
    }

    return false;
  }

private:

  enum State {
    START,
    MEASURE,
    STOP
  } state = START;

  int ticks_cnt = 0;

  // Previous speed value to compare stability
  int prev_speed;

  void set_state(State st)
  {
    state = st;
    ticks_cnt = 0;
  }
};


#endif
