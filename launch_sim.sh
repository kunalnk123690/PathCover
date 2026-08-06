#!/bin/bash
clear
# mkdir -p src/trajectory_server/bag
# rm -r build devel .catkin_workspace src/CMakeLists.txt
catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch corridor_planning corridor_generation.launch