Grinder speed control with stable RPM
=====================================

__WORK IN PROGRESS !!!__

> Advanced speed control for grinder AC brushed motor. With RPM stabilization
> via Back EMF measure. Replacement for default board.

Notes:

1. Schematic is simplified due very heavy size limits.
2. Due the same restrictions, it's impossible to create universal PCB.
   We prepeared PCB for `Hilda-180` - it's very popular and cheap. Borads for
   other grinder models are left to volunteers.
3. Default schematic is for 220v & 200W motor. If you need device for 110v or
   400w - update shunt resistor according to comments and select a proper
   firmware file.


How to build
------------


### Components

1. Go to [EasyEda project page](https://easyeda.com/speed/AC_speed_control_for_grinder-55eba57594ea46788b39b1fe7634fd0b)
    - Order PCB
    - Order details from BOM. We already prepeared links to LCSC for your
      convenience. It's not the cheapest in the wold, but good enougth and you
      may find useful to buy all details in one place.
2. Get additional details, not included into BOM:
    - [Cheap stlink programmer](https://www.aliexpress.com/af/stlink-stm32.html?jump=afs)
      for stm32 devices (only 2$).
3. Extract some components from native grinder board:
    - Speed potentiometer with wheel.
    - Terminal pins for motor contacts.


### Assembly

TBD
