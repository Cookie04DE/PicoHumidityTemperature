# Pico Temperature Humidity

This C program turns the Pico W into a wireless air temperature and humidity sensor.

It measures these two parameters every minute and records this information along with the current time (using the onboard RTC).

Up to 32 (one flash page worth) of these measurements are kept in RAM and flushed to flash if necessary.

It uses the unoccupied flash sectors as a ring buffer for these measurements.

Wear leveling is implemented by picking a random flash sector inside this ring to start out on.

At the time of writing only 85 of the 512 flash sectors are used by the program.

This leaves roughly 1.67 MiB of the 2 MiB flash, or almost 152 days (!) worth of continuous measurement time, before the oldest measurements are overwritten.

Note that the flash is only used if the time between reads from the Pico is more than 32 minutes since the flash is only touched if the 32 ram entries are not enough.

# Reading measurements from the Pico

Use the Rust sister project [PicoHumidityTemperatureRead](https://github.com/Cookie04DE/PicoHumidityTemperatureRead) to read out the measurements and insert them into a PostgreSQL Database.

# RTC Initialization

To start measuring the Pico first needs to aquire the current time and date.

This is done by the first attempt to read from read, since every read access provides the Pico with the current time and date.

The RTC only needs to get initialized once but still get's readjusted every time the Pico get's read.

# Dependencies

## Hardware
- A [Raspberry Pi Pico W microcontroller board](https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html)
- A DHT11/DHT12/DHT22 air temperature and humidity sensor.

## Software
- A working [C/C++ SDK setup](https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html).
- [picocom](https://github.com/npat-efault/picocom) or [minicom](https://github.com/Distrotech/minicom) to monitor the Pico's output.
