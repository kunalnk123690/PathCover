#!/bin/bash
clear
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch corridor_planning corridor_generation.launch.py