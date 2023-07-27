# LoggingPM2.5wCircuitPython
Mobile Air Quality Monitoring and Logging Project


Parts used:

Adafruit ESP32-S2 TFT Feather - 4MB Flash, 2MB PSRAM, STEMMA QT (https://www.adafruit.com/product/5300)

Adafruit PMSA003I Air Quality Breakout - STEMMA QT / Qwiic (https://www.adafruit.com/product/4632)

USB Battery Pack (Anker) and USB-C cable



To enable logging to flash, jumper GND - A2. (Or hookup a switch)

Install jumper before power up, this will setup the Flash RW for Circuit Python.

IMPORTANT NOTE: To be safe, please remove the jumper before powering OFF and wait a second!
Else you may just cut power while writing to flash and chances are to corrupt the file system! 
