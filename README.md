# SMORRT_Software
Embedded Software for SMORRT Avionics

PCB available here: https://github.com/lucaspallez/SMORRT

**:warning: WARNING :warning:This software is not yet complete and thus not ready for flight.**

## Features
- Dual Deploy Flight algorithm with sensor data and safety timers
- Bi-directional LoRa telemetry syste
- Sensor Data acquisition (IMU/Barometer)
- GPS Data acquisition (with NMEA Message parsing)
- Arming/Disarming capability for power outputs
- Power output status acquisition
- High Speed FRAM Data logging with regular dumps on SD Card
- Battery Voltage reading

## Changes planned:
- (TBC) Implement Ring Buffer for direct SD Card logging
- Add LoRa uplink from GRUND
- Finish the full v1 software

## Documentation
### Development Environment
- VSCode + PlatformIO

Board configuration: Teensy 4.1

### Libraries used
- Standard Arduino Libraries
- [Adafruit BMP280 Library](https://github.com/adafruit/Adafruit_BMP280_Library)
- [Adafruit MPU6050 Library](https://github.com/adafruit/Adafruit_MPU6050)
- [RadioHead Library](http://www.airspayce.com/mikem/arduino/RadioHead/)
- [TinyGPS++ Library](http://arduiniana.org/libraries/tinygpsplus/)

TO COMPLETE
