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
    switch (cal_state) {
      case IDLE:        return idle();
      case UP_CHECK:    return up_check();
      case DOWN_CHECK:  return down_check();
      case PRE_PAUSE:   return pre_pause();
      case CALIBRATE_STATIC: return calibrate_static();
      default: reset(); return false;
    }
  }

private:

  enum CalibratorState {
    IDLE,
    UP_CHECK,
    DOWN_CHECK,
    PRE_PAUSE,
    CALIBRATE_STATIC
  } cal_state = IDLE;

  // Helper variables to reduce sub-states
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

  //////////////////////////////////////////////////////////////////////////////
  // States logic

  bool idle(void)
  {
    fix16_t knob = sensors.knob;

    if (knob < KNOB_TRESHOLD)
    {
      cal_state_cnt++;
      if (cal_state_cnt > knob_wait_min) set_state(UP_CHECK);
    }
    else reset();

    return false;
  }

  bool up_check(void)
  {
    fix16_t knob = sensors.knob;

    if (knob < KNOB_TRESHOLD && cal_state_cnt == 0) return false;

    if (knob >= KNOB_TRESHOLD)
    {
      cal_state_cnt++;
      if (cal_state_cnt > knob_wait_max) reset();
      return false;
    }

    if (cal_state_cnt > knob_wait_min)
    {
      cal_knob_dials++;
      if (cal_knob_dials >= 3) set_state(PRE_PAUSE);
      else set_state(DOWN_CHECK);
    }
    else reset();

    return false;
  }

  bool down_check(void)
  {
    fix16_t knob = sensors.knob;

    if (knob < KNOB_TRESHOLD) {
      cal_state_cnt++;
      if (cal_state_cnt > knob_wait_max) reset();
      return false;
    }

    if (cal_state_cnt > knob_wait_min) set_state(UP_CHECK);
    else reset();

    return false;
  }

  // From now we are in calibration mode. Should care about triac and
  // return true

  bool pre_pause(void)
  {
    // 1 sec pause to stop motor for sure
    triacDriver.voltage = sensors.voltage;
    triacDriver.setpoint = 0;
    triacDriver.tick();

    if (cal_state_cnt++ > 1 * APP_TICK_FREQUENCY) set_state(CALIBRATE_STATIC);

    return true;
  }

  bool calibrate_static(void)
  {
    // Dummy stub, 2 sec max speed to show it works
    triacDriver.voltage = sensors.voltage;
    triacDriver.setpoint = fix16_one;
    triacDriver.tick();

    if (cal_state_cnt++ > 2 * APP_TICK_FREQUENCY) reset();

    return true;
  }
};

#endif
