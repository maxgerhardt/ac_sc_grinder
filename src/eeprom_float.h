#ifndef __EEPROM_FLOAT__
#define __EEPROM_FLOAT__

/*
  Wrapper to load float numbers from emulated eeprom.

  1. If value not exists - return default param.
  2. Address value is logical, should be converted to physycal, according to
     data size.
*/

float eeprom_float_read(int addr, float default_value) {
  // TODO: add real eeprom read
  return default_value;
}

#endif
