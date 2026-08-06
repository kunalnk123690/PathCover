cmake_minimum_required(VERSION 3.16)
project(polytope_msgs)


find_package(catkin REQUIRED COMPONENTS
  std_msgs  
  message_generation
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Polytope.msg
  Polytopes.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## catkin specific configuration ##
catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)