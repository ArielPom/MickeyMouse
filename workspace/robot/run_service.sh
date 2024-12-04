#!/bin/bash

cd $HOME/workspace/tof_reader/VL53L5CX_Linux_driver_1.3.0/user/test
OUTPUT_FILE="$HOME/workspace/tof_reader/info.txt"
./menu | tee "$OUTPUT_FILE"
