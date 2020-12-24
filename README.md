# Arduino Pulsed Motor Driver
Turn your Arduino into a controller and monitor for pulsed electric motors, like the Adams Motor or the Bedini SG.

Using two potentiometers, you can control the duty cycle and the timing of the drive pulses.

The serial monitor also logs RPM, duty cycle, pulse duration, and the timing of the pulse in degrees from the drive core center.

Inputs:
* D2: A3144 Hall effect sensor
* A0: 10K potentiometer to control duty cycle
* A1: 10K potentiometer to control pulse timing

Outputs:
* D52: Drive coil(s)

Created 22/12/2020
By Nick Kraakman
Modified 23/12/2020
By Nick Kraakman

https://waveguide.blog

TODO: 
- implement acceleration logging   
- implement input current sensing
- implement input voltage sensing
- implement power logging
- implement core temperature sensing
- implement generator coil switching
- implement multi-pulse option
