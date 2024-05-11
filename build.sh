#!/bin/bash
source pico.env

echo $PICO_SDK_PATH

rm -dR build/*
cd build
cmake ..
cp compile_commands.json ../
make
