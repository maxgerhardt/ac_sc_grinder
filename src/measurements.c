#include <stddef.h>
#include "measurements.h"

st_measurements measurements;

void measurements_init()
{
  measurements.voltage = 0.0f;
  measurements.current = 0.0f;
  measurements.potentiometer = 0.0f;
}

  /*Function is called when ADC conversion complete*/
void measurements_complete(void)
{

}
