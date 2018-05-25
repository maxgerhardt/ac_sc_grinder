
#include "basic_hardware.h"
#include "measurements.h"
#include "silicon_converter.h"
#include "sync.h"

int main(void)
{
  basic_hardware_init();
  measurements_init();
  silicon_converter_init();
  sync_init();

  while (1)
  {
  }
}
