#ifndef __CALIBRATOR_H__
#define __CALIBRATOR_H__

// Detect when used dials knob 3 times, start calibration sequence and
// update configuration.

#include "fix16_math/fix16_math.h"

#include "app.h"
#include "triac_driver.h"
#include "calibrator/calibrator_wait_knob_dial.h"

extern TriacDriver triacDriver;


class Calibrator
{
public:

  // Returns:
  //
  // - false: we should continue in normal more
  // - true:  calibration started, we should stop other actions in main
  //          loop until finished.
  //
  bool tick() {

    switch (state) {

    case WAIT_START_CONDITION:
      // Wait until user dials knob 3 times to start calibration
      if (wait_knob_dial.tick()) {
        set_state(PRE_PAUSE);
        return true;
      }
      return false;

    case PRE_PAUSE:
      // 1 sec pause to stop motor for sure
      triacDriver.voltage = sensors.voltage;
      triacDriver.setpoint = 0;
      triacDriver.tick();

      if (ticks_cnt++ > 1 * APP_TICK_FREQUENCY) set_state(CALIBRATE_STATIC);

      return true;

    case CALIBRATE_STATIC:
      // Dummy stub, 2 sec max speed to show it works
      triacDriver.voltage = sensors.voltage;
      triacDriver.setpoint = fix16_one;
      triacDriver.tick();

      if (ticks_cnt++ > 2 * APP_TICK_FREQUENCY) set_state(WAIT_START_CONDITION);

      return true;
    }

    return false; // unreacheable, suppress warning
  }

private:

  enum CalibratorState {
    WAIT_START_CONDITION,
    PRE_PAUSE,
    CALIBRATE_STATIC
  } state = WAIT_START_CONDITION;

  int ticks_cnt = 0;

  void set_state(CalibratorState st) {
    state = st;
    ticks_cnt = 0;
  }

  // Nested FSM-s
  CalibratorWaitKnobDial wait_knob_dial;
};

#endif
