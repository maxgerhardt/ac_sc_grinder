#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

typedef struct
{
  float voltage;
  float current;
  float potentiometer;
}st_measurements;

void measurements_init(void);
void measurements_complete(void);

#endif
