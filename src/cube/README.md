Config for STM32CubeMX
======================

You probably never need files from this folder.

If you wish to adjust MCU pinouts and update `/src/hw_init.h`, follow
instructions below:

1. Install and run STM32CubeMX
   (https://www.st.com/en/development-tools/stm32cubemx.html#getsoftware-scroll)
2. Press `Load Project`, select file `ac_sc_grinder.ioc`.
3. Make nesessary changes.
4. Select menu `Project` -> `Generate Code`.
5. Copy all `MX_*` functions from `/src/cube/Src/main.c` to `/src/hw_init.h`.
6. Copy all functions from `/src/cube/Src/stm32f1xx_hal_msp.c` to `/src/hw_init.h`.
7. Copy all functions from `/src/cube/Src/stm32f1xx_it.c` to `/src/hw_init.h`.
