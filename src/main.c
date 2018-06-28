#include <math.h>
#include <stdbool.h>

#include "stm32f1xx_hal.h"

//Parameters from configurator

#define SHUNT_RESISTANCE 1.0
#define MOTOR_RESISTANCE 50.0
#define RPM_MAX 30000.0
#define P_MAX 230.0
#define RPM_MIN_LIMIT 3000.0
#define RPM_MAX_LIMIT 10000.0
#define PID_P 1.0
#define PID_I 1.0
#define DEAD_ZONE_WIDTH 10.0

//Internal parameters

#define MAX_12BIT 4096
#define SHUNT_AMPLIFIER_K 10.0
#define ADC_MAX_VOLTAGE 3.3
#define SPEED_COEFF 10.0
#define VOLTAGE_BUFFER_LENGTH 1024
#define POWER_SUPPLY_FREQ 50

#define TRIAC_TICK_FREQ 40000
#define PID_TICK_FREQ 50

uint16_t adc_current;

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

float voltage_buffer[VOLTAGE_BUFFER_LENGTH];

int triac_tick_counter;

bool PID_speed_lock = false;
float PID_speed_integral = 0.0;
float PID_power_integral = 0.0;

float control_speed = 0.0;
float control_power = 0.0;
float control_voltage = 0.0;

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

void current_value_converter(void)
{
  current = adc_current/MAX_12BIT/SHUNT_RESISTANCE/1000.0/SHUNT_AMPLIFIER_K*ADC_MAX_VOLTAGE;
}

void speed_calculator(void)
{
  if ((current>0.0)&&(voltage>0.0))
  {
    r_ekv_sum += voltage/current - MOTOR_RESISTANCE;
    speed_tick_counter++;
  }

  if ((prev_voltage>0.0)&&(voltage==0.0))
  {
    float r_ekv = r_ekv_sum/speed_tick_counter;
    speed = SPEED_COEFF*r_ekv/RPM_MAX;
    speed_tick_counter++;
  }
    prev_voltage = voltage;
}

void power_calculator(void)
{
  if ((current>0.0)&&(voltage>0.0))
  {
    p_sum += voltage*current;
    power_tick_counter++;
  }
  else if ((current>0.0)&&(voltage==0.0))
  {
    p_sum += -voltage_buffer[power_back_tick_counter]*current;
    power_back_tick_counter++;
    power_tick_counter++;
  }

  if ((prev_current>0.0)&&(current==0.0))
  {
    power = p_sum/power_tick_counter/P_MAX*100.0;
  }

  for (int i=0;i<VOLTAGE_BUFFER_LENGTH-1;i++)
    voltage_buffer[i+1] = voltage_buffer[i];

  voltage_buffer[0] = voltage;
  prev_current = current;
}

void triac_on(void)
{

}

void triac_off(void)
{

}

void triac_controller(void)
{
  float angle = 2*acos(control_voltage/100.0)/3.1416*TRIAC_TICK_FREQ/POWER_SUPPLY_FREQ/2;

  if (triac_tick_counter<angle)
    triac_off();
  else
    triac_on();

  triac_tick_counter++;

  if (((prev_voltage==0.0)&&(voltage>0.0))||((prev_voltage>0.0)&&(voltage==0.0)))
    triac_tick_counter = 0;
}

void PID_speed(void)
{
  if (!PID_speed_lock)
  {
    if (potentiometer<DEAD_ZONE_WIDTH)
      potentiometer = 0.0;

    float error = potentiometer - speed;
    float proportional = PID_P*error;

    PID_speed_integral += 1.0/PID_I*error/PID_TICK_FREQ;

    if (PID_speed_integral > RPM_MAX_LIMIT/RPM_MAX*100.0)
      PID_speed_integral = RPM_MAX_LIMIT/RPM_MAX*100.0;

    if (PID_speed_integral < RPM_MIN_LIMIT/RPM_MAX*100.0)
      PID_speed_integral = RPM_MIN_LIMIT/RPM_MAX*100.0;

    float output = proportional + PID_speed_integral;

    if (output > RPM_MAX_LIMIT/RPM_MAX*100.0)
      output = RPM_MAX_LIMIT/RPM_MAX*100.0;

    if (output < RPM_MIN_LIMIT/RPM_MAX*100.0)
      output = RPM_MIN_LIMIT/RPM_MAX*100.0;

    control_speed = output;
  }
}

void PID_power(void)
{
  float error = 100.0 - power;
  float proportional = PID_P*error;

  PID_power_integral += 1.0/PID_I*error/PID_TICK_FREQ;

  if (PID_power_integral > 100.0)
    PID_power_integral = 100.0;

  if (PID_power_integral < 0.0)
    PID_power_integral = 0.0;

  float output = proportional + PID_power_integral;

  if (output > 100.0)
    output = 100.0;

  if (output < 0.0)
    output = 0.0;

  control_power = output;
}

void timer1_callback(void)
{
  speed_calculator();
  power_calculator();
  triac_controller();
}

void timer2_callback(void)
{
  PID_speed();
  PID_power();
  if (control_speed<=control_power)
  {
    PID_speed_lock = false;
    control_voltage = control_speed;
  }
  else
  {
    PID_speed_lock = true;
    control_voltage = control_power;
  }
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
