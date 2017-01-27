# Pitt RAS Micromouse

This is an Arduino project for Teensy 3.1/3.2.

## Dependencies

 - Teensyduino <https://www.pjrc.com/teensy/td_download.html>

 - Encoder (patched) <https://github.com/pitt-ras/Encoder>

 - I2Cdev and MPU9150 (patched) <https://github.com/pitt-ras/i2cdevlib>

## Overclocking

In the code, we enforce 144 MHz. By default, this is disabled in
Teensyduino. To enable this clock speed:

 1. In your Arduino installation, find this file:

     `hardware/teensy/avr/boards.txt`

 2. Uncomment this line:

     ```teensy31.menu.speed.144=144 MHz (overclock)```
