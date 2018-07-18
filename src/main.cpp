#include <math.h>

#include "stm32f1xx_hal.h"


#include "hw_init.h"
#include "fix16_math/fix16_math.h"
#include "eeprom_float.h"
#include "config_map.h"
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

    fix16_t adc_voltage = ADCBuffer[0];
    fix16_t adc_current = ADCBuffer[1];
    fix16_t adc_knob = ADCBuffer[2];
    fix16_t adc_vrefin = ADCBuffer[3];

    sensors.adc_raw_data_load(adc_voltage, adc_current, adc_knob, adc_vrefin);
}


// 40 kHz ticker for all logic
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static fix16_t fix16_prev_voltage = 0;

  sensors.tick();

  fix16_t fix16_voltage = sensors.voltage;

  triacDriver.voltage = fix16_voltage;
  triacDriver.tick();

  // Poor man zero cross check
  if (((fix16_prev_voltage == 0) && (fix16_voltage > 0)) ||
      ((fix16_prev_voltage > 0) && (fix16_voltage == 0)))
  {
    // 100/120 Hz, tick speed controller PIDs & rearm triac driver
    speedController.in_knob = sensors.knob;
    speedController.in_speed = sensors.speed;
    speedController.in_power = sensors.power;
    // TODO: PIDs should not drift on 50/60hz switch
    speedController.tick();

    triacDriver.setpoint = speedController.out_power;
    triacDriver.rearm();
  }

  fix16_prev_voltage = fix16_voltage;
}


int main(void)
{
  HAL_Init();
  SystemClock_Config();

  // Load config info from emulated EEPROM
  speedController.configure();
  sensors.configure();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DMA_Init();
  MX_TIM1_Init();

  HAL_ADCEx_Calibration_Start(&hadc1);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCBuffer, 3);
  HAL_TIM_Base_Start_IT(&htim1);

  while (1) {
    __WFI();
  }
}
