#include <math.h>

#include "stm32f1xx_hal.h"

#include "config.h"
#include "speed_controller.h"
#include "sensors.h"
#include "triac.h"

struct AppData
{
  float cfg_shunt_resistance;
  float potentiometer;
};

AppData app_data;
SpeedController speedController;
Sensors sensors;
Triac triac;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim1;

uint16_t ADCBuffer[3];

void _Error_Handler(const char * file, int line)
{
  while(1)
  {
  }
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /*Initializes the CPU, AHB and APB busses clocks*/
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /*Initializes the CPU, AHB and APB busses clocks*/
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /*Configure the Systick interrupt time*/
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /*Configure the Systick*/
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

  /*GPIO initialization*/
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void MX_TIM1_Init(void)
{

}

// 40 kHz
void zerocross(float voltage)
{
  if (((sensors.prev_voltage == 0.0) && (voltage > 0.0)) ||
        ((sensors.prev_voltage > 0.0) && (voltage == 0.0)))
  {
    // 100 - 120 Hz
    triac.reset();
    speedController.tick(app_data.potentiometer, sensors.speed, sensors.power);
    triac.setpoint = speedController.setpoint;
  }
}

float read_cfg(float *addr, float default_value)
{
  return default_value;
}

void cfg_init()
{
  app_data.cfg_shunt_resistance = read_cfg((float*)CFG_SHUNT_RESISTANCE_ADDR,
                                                   CFG_SHUNT_RESISTANCE_DEFAULT);

  speedController.cfg_dead_zone_width = read_cfg((float*)CFG_DEAD_ZONE_WIDTH_ADDR,
                                                     CFG_DEAD_ZONE_WIDTH_DEFAULT);
  speedController.cfg_pid_p = read_cfg((float*)CFG_PID_P_ADDR, CFG_PID_P_DEFAULT);
  speedController.cfg_pid_i = read_cfg((float*)CFG_PID_I_ADDR, CFG_PID_P_DEFAULT);
  speedController.cfg_rpm_max_limit = read_cfg((float*)CFG_RPM_MAX_LIMIT_ADDR,
                                                       CFG_RPM_MAX_LIMIT_DEFAULT);
  speedController.cfg_rpm_min_limit = read_cfg((float*)CFG_RPM_MIN_LIMIT_ADDR,
                                                       CFG_RPM_MIN_LIMIT_DEFAULT);
  speedController.cfg_rpm_max = read_cfg((float*)CFG_RPM_MAX_ADDR,
                                                 CFG_RPM_MAX_DEFAULT);

  sensors.cfg_p_max = read_cfg((float*)CFG_P_MAX_ADDR, CFG_P_MAX_DEFAULT);
  sensors.cfg_motor_resistance = read_cfg((float*)CFG_MOTOR_RESISTANCE_ADDR,
                                                  CFG_MOTOR_RESISTANCE_DEFAULT);
  sensors.cfg_rpm_max = read_cfg((float*)CFG_RPM_MAX_ADDR, CFG_RPM_MAX_DEFAULT);
}

// 40 kHz
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
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

// 40 kHz
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  sensors.tick();
  triac.tick();
  zerocross(sensors.voltage);
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  cfg_init();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DMA_Init();
  MX_TIM1_Init();

  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADCBuffer,3);
  HAL_TIM_Base_Start_IT(&htim1);

  while (1)
  {
  }
}
