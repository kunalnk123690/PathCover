cmake_minimum_required(VERSION 3.16)
project(quadrotor_msgs)

set(PATHCOVER_EXAMPLE "quadrotor" CACHE STRING "PathCover example to build (quadrotor or jackal)")

if(PATHCOVER_EXAMPLE STREQUAL "quadrotor")
  find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    message_generation
  )

  add_message_files(
    FILES
    TrajectoryCommand.msg
  )

  generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
  )

  catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs message_runtime)
else()
  find_package(catkin REQUIRED)
  catkin_package()
endif()
