# M200Diagnostics
This is a low-level hardware diagnostic firmware for the M200 class printers such as the Malyan M200 or Monoprice Mini.
It allows access to the hardware at a low level for testing of fans, heaters, thermistors, steppers, endstops, LCDs, 
and SD cards.

USE AT YOUR OWN RISK.

BE CAREFUL.
If you turn on the heater, it will stay on unless you turn it off or restart the printer. The extruder fan is normally on
auto, but in this firmware, there is no auto-fan, so don't turn on the heater and leave it on without the fan. 

DO NOT USE THIS IF YOU DON'T HAVE A FIRMWARE TO GO BACK TO.

While I have tested it on V1 and early V2s, I haven't tried it on later run V2s or deltas.

To use this: Flash onto the printer and connect via your favorite terminal.
The menus should be largely self-explanatory.
