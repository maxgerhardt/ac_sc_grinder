STM32CubeMX generated files
===========================

DON'T change content manually.

How to update:

1. Install and run STM32CubeMX
   (https://www.st.com/en/development-tools/stm32cubemx.html#getsoftware-scroll)
2. Press `Load Project`, select file `ac_sc_grinder.ioc`.
3. Make nesessary changes.
4. Menu: `File` -> `Save Project`
4. Menu: `Project` -> `Generate Code`.
5. DELETE `Drivers` folder
6. Make sure `Src/main.c` contains this lines: `#include "app.h"` &
   `app_start();`
7. Run build and check result.
