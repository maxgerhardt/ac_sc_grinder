#include <math.h>
#include <stdbool.h>

#include "stm32f1xx_hal.h"

#include "config.h"

struct AppData
{
  float cfg_shunt_resistance;
  float cfg_motor_resistance;
  float cfg_rpm_max;
  float cfg_p_max;
  float cfg_rpm_min_limit;
  float cfg_rpm_max_limit;
  float cfg_pid_p;
  float cfg_pid_i;
  float cfg_dead_zone_width;

  float potentiometer;

  float current = 0.0;
  float prev_current;

  float voltage = 0.0;
  float prev_voltage;

  float power = 0.0;

  float r_ekv_sum = 0.0;
  float speed = 0.0;
  int speed_tick_counter;

  float p_sum = 0.0;
  int power_tick_counter;
  int power_back_tick_counter;

  float voltage_buffer[1024];

  int triac_tick_counter;

  bool PID_speed_lock = false;
  float PID_speed_integral = 0.0;
  float PID_power_integral = 0.0;

  float control_speed = 0.0;
  float control_power = 0.0;
  float control_voltage = 0.0;
};

AppData app_data;

uint16_t adc_current;

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

float current_value_converter(uint16_t adc_current)
{
  float current = adc_current / 4096 / app_data.cfg_shunt_resistance /
    1000.0 / 10.0 * 3.3;
  return current;
}

void speed_calculator(float current, float voltage)
{
  if ((current > 0.0) && (voltage > 0.0))
  {
    app_data.r_ekv_sum += voltage/current - app_data.cfg_motor_resistance;
    app_data.speed_tick_counter++;
  }

  if ((app_data.prev_voltage > 0.0) && (voltage == 0.0))
  {
    float r_ekv = app_data.r_ekv_sum / app_data.speed_tick_counter;
    app_data.speed = 10.0 * r_ekv / app_data.cfg_rpm_max;
    app_data.speed_tick_counter++;
  }
    app_data.prev_voltage = voltage;
}

void power_calculator(float current, float voltage)
{
  if ((current > 0.0) && (voltage > 0.0))
  {
    app_data.p_sum += voltage*current;
    app_data.power_tick_counter++;
  }
  else if ((current > 0.0) && (voltage == 0.0))
  {
    app_data.p_sum -= app_data.voltage_buffer[app_data.power_back_tick_counter] * current;
    app_data.power_back_tick_counter++;
    app_data.power_tick_counter++;
  }

  if ((app_data.prev_current > 0.0) && (current == 0.0))
  {
    app_data.power = app_data.p_sum / app_data.power_tick_counter /
      app_data.cfg_p_max * 100.0;
  }

  for (int i=0;i<1024-1;i++)
    app_data.voltage_buffer[i+1] = app_data.voltage_buffer[i];

  app_data.voltage_buffer[0] = voltage;
  app_data.prev_current = current;
}

void triac_on(void)
{

}

void triac_off(void)
{

}

void triac_controller(float control_voltage, float voltage)
{
  float angle = 2 * acos(control_voltage / 100.0) / 3.1416 * 40000.0 / 50.0 / 2;

  if (app_data.triac_tick_counter<angle)
    triac_off();
  else
    triac_on();

  app_data.triac_tick_counter++;

  if (((app_data.prev_voltage == 0.0) && (voltage > 0.0)) ||
      ((app_data.prev_voltage > 0.0) && (voltage == 0.0)))
    app_data.triac_tick_counter = 0;
}

void PID_speed(float potentiometer, float speed, bool lock)
{
  if (!lock)
  {
    if (potentiometer < app_data.cfg_dead_zone_width)
      potentiometer = 0.0;

    float error = potentiometer - speed;
    float proportional = app_data.cfg_pid_p*error;

    app_data.PID_speed_integral += 1.0 / app_data.cfg_pid_i * error / 50.0;

    if (app_data.PID_speed_integral > app_data.cfg_rpm_max_limit /
        app_data.cfg_rpm_max * 100.0)
      app_data.PID_speed_integral = app_data.cfg_rpm_max_limit /
        app_data.cfg_rpm_max * 100.0;

    if (app_data.PID_speed_integral < app_data.cfg_rpm_min_limit /
        app_data.cfg_rpm_max * 100.0)
      app_data.PID_speed_integral = app_data.cfg_rpm_min_limit /
        app_data.cfg_rpm_max * 100.0;

    float output = proportional + app_data.PID_speed_integral;

    if (output > app_data.cfg_rpm_max_limit /
        app_data.cfg_rpm_max * 100.0)
      output = app_data.cfg_rpm_max_limit /
        app_data.cfg_rpm_max * 100.0;

    if (output < app_data.cfg_rpm_min_limit /
        app_data.cfg_rpm_max * 100.0)
      output = app_data.cfg_rpm_min_limit /
        app_data.cfg_rpm_max * 100.0;

    app_data.control_speed = output;
  }
}

void PID_power(float power)
{
  float error = 100.0 - power;
  float proportional = app_data.cfg_pid_p * error;

  app_data.PID_power_integral += 1.0 / app_data.cfg_pid_i * error / 50.0;

  if (app_data.PID_power_integral > 100.0)
    app_data.PID_power_integral = 100.0;

  if (app_data.PID_power_integral < 0.0)
    app_data.PID_power_integral = 0.0;

  float output = proportional + app_data.PID_power_integral;

  if (output > 100.0)
    output = 100.0;

  if (output < 0.0)
    output = 0.0;

  app_data.control_power = output;
}

float min_PID(float control_speed, float control_power)
{
  if (control_speed <= control_power)
  {
    app_data.PID_speed_lock = false;
    return control_speed;
  }
  else
  {
    app_data.PID_speed_lock = true;
    return control_power;
  }
}

void timer1_callback(void)
{
  app_data.current = current_value_converter(adc_current);
  speed_calculator(app_data.current, app_data.voltage);
  power_calculator(app_data.current, app_data.voltage);
  triac_controller(app_data.control_voltage, app_data.voltage);
}

void timer2_callback(void)
{
  PID_speed(app_data.potentiometer,app_data.speed,app_data.PID_speed_lock);
  PID_power(app_data.power);
  app_data.control_voltage = min_PID(app_data.control_speed, app_data.control_power);
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  while (1)
  {
  }
}
