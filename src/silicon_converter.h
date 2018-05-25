#ifndef SILICON_CONVERTER_H
#define SILICON_CONVERTER_H

#include <stdint.h>

typedef struct{
  float voltage_percent;
  uint32_t tick_counter;
}st_silicon_converter;

void silicon_converter_init(void);
void silicon_converter_tick(void);
void silicon_converter_sync(void);

#endif
