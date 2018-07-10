Grinder speed control with stable RPM
=====================================

__WORK IN PROGRESS !!!__

> Advanced speed control for grinder AC brushed motor. With RPM stabilization
> via Back EMF measure. Replacement for default board.

Notes! Due size restrictions, it's impossible to create universal PCB.
We prepeared PCB for `Hilda 180W` - it's very popular and cheap. Borads for
other grinder models are left to volunteers.


### Required components

1. [Hilda 180W](https://www.aliexpress.com/wholesale?SearchText=hilda+180w).
   Regulator is not restricted for this tool, but existing PCB layout is for
   this grinder model. Also, firmware config defaults are for it's motor's power.
2. Go to [EasyEda project page](https://easyeda.com/speed/AC-speed-control-for-grinder)
    - Order PCB
    - Order details from BOM. We already prepeared links to LCSC for your
      convenience. It's not the cheapest in the wold, but good enougth. You
      may find nice to buy all details in one place and join delivery to
      single package.
3. Get additional details, not included into BOM:
    - [Cheap stlink programmer](https://www.aliexpress.com/af/stlink-stm32.html?jump=afs)
      for stm32 devices (only 2$).
    - [Plastik 70 CRC](https://www.google.com/search?q=Plastik+70+CRC) or any
      other acrylic protective coating. We strongly recommend to protect PCB
      from aspirated dust
4. Extract some components from native grinder board:
    - Speed potentiometer with wheel.
    - Terminal pins for motor contacts.


## Building hardware

1. Assemble PCB.
2. Flash firmware.
3. Test that it works.
4. Cover PCB with protective coating.


## How to flash firmware

1. Install PlatformIO IDE. Follow instructions [here](http://docs.platformio.org/en/latest/ide/pioide.html).
   We use PIO for Atom, but PIO for VSCode should be ok too.
   - Make sure you've [installed](http://docs.platformio.org/en/latest/installation.html#troubleshooting)
   udev rules (linux) or device drivers (windows).
2. Clone this repo or download via zip archive.
3. Optionally, edit defaults in `/src/config_map.h`, but defaults should be ok.
   We recommend to skip this step.
4. Open this project in installed IDE.
5. Open `PlatformIO` -> `Terminal` -> `New Terminal`

Now type this commands in terminat window:

```bash
pio run --target upload
```


## Development

Everything is done in `PlatformIO`. If you wish to update conig and regenerate
headers, you may need `node.js` and run `npm run build`.


## License

MIT.
