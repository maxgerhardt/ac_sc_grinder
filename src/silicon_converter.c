#include "silicon_converter.h"

st_silicon_converter silicon_converter;

void silicon_converter_init(void)
{
  silicon_converter.voltage_percent = 0.0f;
  silicon_converter.tick_counter = 0;
}

  /*One tick of phase angle counter*/
void silicon_converter_tick(void)
{
  silicon_converter.tick_counter++;
}

  /*Restart phase angle counter. Should be called when the voltage crosses zero*/
void silicon_converter_sync(void)
{
  silicon_converter.tick_counter = 0;
}
