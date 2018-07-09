#include <math.h>

#include "stm32f1xx_hal.h"

#include "hw_init.h"
#include "eeprom_float.h"
#include "config_map.h"
#include "speed_controller.h"
#include "sensors.h"
#include "triac_driver.h"


struct AppData
{
  float cfg_shunt_resistance; // Current sense shunt value, mOhms
  float potentiometer;        // Speed knob physical value, 0..100%
};


AppData app_data;
SpeedController speedController;
Sensors sensors;
TriacDriver triacDriver;

uint16_t ADCBuffer[3];

// Load config from emulated EEPROM
void configure()
{
  // Application
  app_data.cfg_shunt_resistance = eeprom_float_read(CFG_SHUNT_RESISTANCE_ADDR, CFG_SHUNT_RESISTANCE_DEFAULT);

  // Components
  speedController.configure();
  sensors.configure();
}


// ADC data handler, ~ 40 kHz.
// Load handled values and put normalized data into objects
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
    // TODO: calibrate ADC instead of hardcoded 3.3v
    // TODO: investigate interrupts priorities & atomic data r/w

    uint16_t ADCVoltage = ADCBuffer[0];
    uint16_t ADCCurrent = ADCBuffer[1];
    uint16_t ADCPotentiometer = ADCBuffer[2];

    // 4096 - maximum value of 12-bit integer
    app_data.potentiometer = ADCPotentiometer * 100.0 / 4096.0;

    // app_data.cfg_shunt_resistance - in mOhm, divide by 1000
    // maximum ADC input voltage - 3.3 V
    // shunt amplifier gain - 50
    sensors.current = ADCCurrent / 4096.0 / app_data.cfg_shunt_resistance /
      1000.0 / 50.0 * 3.3;

    // resistors in voltage divider - 2*150 kOhm, 1.5 kOhm
    sensors.voltage = ADCVoltage * 3.3 / 4096.0 / 1.5 * 301.5;
}


// 40 kHz ticker for all logic
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  sensors.tick();
  triacDriver.tick();

  // Poor man zero cross check
  if (((sensors.prev_voltage == 0.0) && (sensors.voltage > 0.0)) ||
        ((sensors.prev_voltage > 0.0) && (sensors.voltage == 0.0)))
  {
    // 100/120 Hz, rearm triac driver & tick speed controller PIDs
    triacDriver.reset();
    speedController.tick(app_data.potentiometer, sensors.speed, sensors.power);
    triacDriver.setpoint = speedController.setpoint;
  }
}


int main(void)
{
  HAL_Init();
  SystemClock_Config();

  configure();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DMA_Init();
  MX_TIM1_Init();

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCBuffer, 3);
  HAL_TIM_Base_Start_IT(&htim1);

  // TODO: wait for interrupt
  while (1) {}
}
