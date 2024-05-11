#!/bin/bash
source pico.env

until [ -e "/dev/serial/by-id/$SERIAL_CONSOLE" ]
do
  sleep 1
done

picocom -b 9600 "/dev/serial/by-id/$SERIAL_CONSOLE" || exit 1
