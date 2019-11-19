# nogasm
Software and hardware for Arduino-based orgasm prediction / detection

### Case
The case can be 3D printed without support material. Print at least the top half of the case in white to see the LED display beneath. 

### PCB
All the Gerber files are included to order a set of PCBs. The [BOM is available on Octopart](https://octopart.com/bom-tool/FGy3gDRv). The PCB has mostly surface-mount components, and is doable with either a soldering iron or solder paste.

### Vibrator
The vibrator is made from a Mabuchi RS-555PH motor, with an eccentric mass pressed onto the shaft. These can be purchased pre-assembled, or built from the motor and some .88" to 1" diameter cylindrical brass stock.
The vibrator case is compatible with attachments designed for the Hitachi Magic Wand. The motor is press-fit or glued into the two halves of the case.

### Dependencies
This project uses a [Teensy LC](https://www.pjrc.com/store/teensylc.html), which will require the [teensyduino](https://www.pjrc.com/teensy/teensyduino.html) addon for the Arduino IDE, and its included [encoder library](https://github.com/PaulStoffregen/Encoder).
The detection algorithm uses a circular buffer for calculating a running average, [available here](https://github.com/RobTillaart/Arduino/tree/master/libraries/RunningAverage).


This work is licensed under a Creative Commons Attribution-NonCommercial 4.0 International Licence, available at http://creativecommons.org/licenses/by-nc/4.0/.
