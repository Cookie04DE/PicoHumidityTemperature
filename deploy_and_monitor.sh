#!/bin/bash
source pico.env

sudo mount "/dev/disk/by-id/$FLASH_MOUNT" "$MOUNT_PATH" || exit 1

sudo cp build/pico_temperature_humidity.uf2 "$MOUNT_PATH" || exit 1

./monitor.sh
