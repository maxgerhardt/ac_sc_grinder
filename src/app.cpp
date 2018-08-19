#include "app.h"

#include "adc.h"

#include "speed_controller.h"
#include "sensors.h"
#include "triac_driver.h"

SpeedController speedController;
Sensors sensors;
TriacDriver triacDriver;

////////////////////////////////////////////////////////////////////////////////

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

Calibrator calibrator;

////////////////////////////////////////////////////////////////////////////////

// ADC data is transfered to double size DMA buffer. Interrupts happen on half
// transfer and full transfer. So, we can process received data without risk
// of override. While half of buffer is processed, another half os used to
// collect next data.

uint16_t ADCBuffer[ADC_FETCH_PER_TICK * ADC_CHANNELS_COUNT * 2];

// Polling flag, set to true every time when new ADC data is ready.
bool adc_data_ready = false;
// Offset of actual ADC data in DMA buffer,
uint32_t adc_data_offset = 0;

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  adc_data_ready = true;
  adc_data_offset = 0;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  adc_data_ready = true;
  adc_data_offset = ADC_FETCH_PER_TICK * ADC_CHANNELS_COUNT;
}


// Main app loop.
void app_start(void)
{
  // Load config info from emulated EEPROM
  speedController.configure();
  sensors.configure();

  triacDriver.ref_sensors = &sensors;

  // Final hardware start: calibrate ADC & run cyclic DMA ops.
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCBuffer, ADC_FETCH_PER_TICK * ADC_CHANNELS_COUNT * 2);

  // Override loop in main.c to reduce patching
  while (1) {
    // Polling for flag which indicates that ADC data is ready
    while (!adc_data_ready) {
      // TODO: try to use volatile instead
      asm(""); // Prevent optimizer to drop empty loop
    }

    // Reset flag
    adc_data_ready = false;

    // Load samples from actual half of ADC buffer to sensors buffers
    sensors.adc_raw_data_load(ADCBuffer, adc_data_offset);

    sensors.tick();

    // Detect calibration mode & run calibration procedure if needed.
    // If calibration in progress - skip other steps.
    if (calibrator.tick()) continue;

    // Normal processing

    triacDriver.voltage = sensors.voltage;

    speedController.in_knob = sensors.knob;
    speedController.in_speed = sensors.speed;
    speedController.in_power = sensors.power;

    speedController.tick();

    triacDriver.setpoint = speedController.out_power;

    triacDriver.tick();
  }
}
