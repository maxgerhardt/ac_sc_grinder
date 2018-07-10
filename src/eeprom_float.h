#ifndef __EEPROM_FLOAT__
#define __EEPROM_FLOAT__

/*
  Wrapper to load float numbers from emulated eeprom.

  1. If value not exists - return default param.
  2. Address value is logical, should be converted to physycal, according to
     data size.

  Current stub always return default value.
*/

/*
  !!! NOTE: this feature is reserved, to unify params loading style.
  Probably will never be implemented.
*/

float eeprom_float_read(int addr, float default_value)
{
  return default_value;
}

#endif
