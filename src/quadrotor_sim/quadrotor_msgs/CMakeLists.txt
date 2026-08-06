cmake_minimum_required(VERSION 3.16)
project(quadrotor_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  TrajectoryCommand.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## catkin specific configuration ##
catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
)