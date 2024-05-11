#!/bin/bash
source pico.env

rm -dR build/*
cd build
env CMAKE_EXPORT_COMPILE_COMMANDS=1 cmake ..
cp compile_commands.json ../
make
