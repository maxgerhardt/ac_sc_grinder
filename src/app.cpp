#include "app.h"

#include "adc.h"
#include "tim.h"

#include "speed_controller.h"
#include "sensors.h"
#include "triac_driver.h"

SpeedController speedController;
Sensors sensors;
TriacDriver triacDriver;

uint16_t ADCBuffer[4];


// ADC data handler, ~ 40 kHz.
// Load handled values & normalize into `sensors` instanse
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
    // TODO: investigate interrupts priorities & atomic data r/w

    // TODO: add median filering to make zero cross checks less fragile.

    uint16_t adc_voltage = ADCBuffer[0];
    uint16_t adc_current = ADCBuffer[1];
    uint16_t adc_knob = ADCBuffer[2];
    uint16_t adc_vrefin = ADCBuffer[3];

    sensors.adc_raw_data_load(adc_voltage, adc_current, adc_knob, adc_vrefin);
}


// 40 kHz ticker for all logic
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static fix16_t prev_voltage = 0;

  sensors.tick();

  fix16_t voltage = sensors.voltage;

  triacDriver.voltage = voltage;
  triacDriver.tick();

  // Poor man zero cross check
  if (((prev_voltage == 0) && (voltage > 0)) ||
      ((prev_voltage > 0) && (voltage == 0)))
  {
    // 100/120 Hz, tick speed controller PIDs & rearm triac driver
    speedController.in_knob = sensors.knob;
    speedController.in_speed = sensors.speed;
    speedController.in_power = sensors.power;
    // TODO: PIDs should not drift on 50/60hz switch
    speedController.tick();

    triacDriver.setpoint = speedController.out_power;
  }

  prev_voltage = voltage;
}


void app_start(void)
{
  // Load config info from emulated EEPROM
  speedController.configure();
  sensors.configure();

  HAL_ADCEx_Calibration_Start(&hadc1);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCBuffer, 4);
  HAL_TIM_Base_Start_IT(&htim1);

  // Override loop in main.c to reduce patching
  while (1) {
    __WFI();
  }
}
