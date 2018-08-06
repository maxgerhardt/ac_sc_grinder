#include "app.h"

#include "adc.h"

#include "speed_controller.h"
#include "sensors.h"
#include "triac_driver.h"

SpeedController speedController;
Sensors sensors;
TriacDriver triacDriver;

// In this buffer DMA will write data from ADC.
uint16_t ADCBuffer[ADC_CHANNELS_COUNT * ADC_FETCH_PER_TICK * 2];

// Flag indicates that ADC data is ready for procession
bool adc_data_ready = false;
// Offset of actual half of ADC DMA buffer,
uint32_t adc_data_offset = 0;


// ADC data handler, second half of ADC buffer filled ~ 17.9 kHz.
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  adc_data_ready = true;
  // Offcet of second half of ADC buffer is 4 channels * 8 samples = 32
  adc_data_offset = ADC_CHANNELS_COUNT * ADC_FETCH_PER_TICK;
}

// ADC data handler, first half of ADC buffer filled ~ 17.9 kHz.
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  adc_data_ready = true;
  // Offcet of first half of ADC buffer is 0
  adc_data_offset = 0;
}


void app_start(void)
{
  // Load config info from emulated EEPROM
  speedController.configure();
  sensors.configure();

  HAL_ADCEx_Calibration_Start(&hadc1);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCBuffer,
                    ADC_CHANNELS_COUNT * ADC_FETCH_PER_TICK * 2);
  
  // Override loop in main.c to reduce patching
  while (1) {
    // Polling for flag which indicates that ADC data is ready
    while (!adc_data_ready) {
      asm(""); // To prevent optimizer from deleting empty loop
    }
    
    // Reset flag
    adc_data_ready = false;

    // Load samples from actual half of ADC buffer to sensors buffers
    sensors.adc_raw_data_load(ADCBuffer, adc_data_offset);
      
    sensors.tick();

    triacDriver.voltage = sensors.voltage;

    speedController.in_knob = sensors.knob;
    speedController.in_speed = sensors.speed;
    speedController.in_power = sensors.power;

    speedController.tick();

    triacDriver.setpoint = speedController.out_power;

    triacDriver.tick();
  } 
}
