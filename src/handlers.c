#include <stm32f1xx_hal.h>
#include "handlers.h"

/*This function is executed in case of error occurrence.*/
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
