#ifndef __CALIBRATOR_H__
#define __CALIBRATOR_H__


#include "fix16_math/fix16_math.h"

#include "app.h"
#include "sensors.h"
#include "triac_driver.h"

extern Sensors sensors;
extern TriacDriver triacDriver;


#define KNOB_TRESHOLD F16(0.05)

constexpr int knob_wait_min = APP_TICK_FREQUENCY * 0.2;
constexpr int knob_wait_max = APP_TICK_FREQUENCY * 1.0;

class Calibrator
{
public:

  bool tick(void) {
    fix16_t knob = sensors.knob;

    switch (cal_state) {

      case IDLE:

        if (knob < KNOB_TRESHOLD)
        {
          cal_state_cnt++;
          if (cal_state_cnt > knob_wait_min) set_state(UP_CHECK);
        }
        else reset();

        break;

      case UP_CHECK:
        if (knob < KNOB_TRESHOLD && cal_state_cnt == 0) break;

        if (knob >= KNOB_TRESHOLD)
        {
          cal_state_cnt++;
          if (cal_state_cnt > knob_wait_max) reset();
          break;
        }

        if (cal_state_cnt > knob_wait_min)
        {
          cal_knob_dials++;
          if (cal_knob_dials >= 3) set_state(PRE_PAUSE);
          else set_state(DOWN_CHECK);
        }
        else reset();

        break;

      case DOWN_CHECK:
        if (knob < KNOB_TRESHOLD) {
          cal_state_cnt++;
          if (cal_state_cnt > knob_wait_max) reset();
          break;
        }

        if (cal_state_cnt > knob_wait_min) set_state(UP_CHECK);
        else reset();

        break;

      // From now we are in calibration more. Should care about triac and
      // return true

      case PRE_PAUSE:
        // 1 sec pause to stop motor for sure
        triacDriver.voltage = sensors.voltage;
        triacDriver.setpoint = 0;
        triacDriver.tick();

        if (cal_state_cnt++ > 1 * APP_TICK_FREQUENCY) set_state(CALIBRATE_STATIC);

        return true;

      case CALIBRATE_STATIC:
        // Dummy stub, 2 sec max speed to show it works
        triacDriver.voltage = sensors.voltage;
        triacDriver.setpoint = fix16_one;
        triacDriver.tick();

        if (cal_state_cnt++ > 2 * APP_TICK_FREQUENCY) reset();

        return true;

      default:
        reset();
    }
    return false;
  }

private:

  enum CalibratorState {
    IDLE,
    UP_CHECK,
    DOWN_CHECK,
    PRE_PAUSE,
    CALIBRATE_STATIC,
    CALIBRATE_DYNAMIC
  } cal_state = IDLE;

  int cal_state_cnt = 0;
  int cal_knob_dials = 0;

  void set_state(CalibratorState st)
  {
    cal_state = st;
    cal_state_cnt = 0;
  }

  void reset(void)
  {
    cal_knob_dials = 0;
    set_state(IDLE);
  }
};

#endif
