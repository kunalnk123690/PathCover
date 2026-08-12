cmake_minimum_required(VERSION 3.16)
project(trajectory_server)

set(PATHCOVER_EXAMPLE "quadrotor" CACHE STRING "PathCover example to build (quadrotor or jackal)")
set_property(CACHE PATHCOVER_EXAMPLE PROPERTY STRINGS quadrotor jackal)
if(NOT PATHCOVER_EXAMPLE STREQUAL "quadrotor" AND NOT PATHCOVER_EXAMPLE STREQUAL "jackal")
  message(FATAL_ERROR "PATHCOVER_EXAMPLE must be 'quadrotor' or 'jackal' (got '${PATHCOVER_EXAMPLE}')")
endif()

set(TRAJECTORY_ROBOT_DEPENDENCIES)
if(PATHCOVER_EXAMPLE STREQUAL "jackal")
  list(APPEND TRAJECTORY_ROBOT_DEPENDENCIES tf)
else()
  list(APPEND TRAJECTORY_ROBOT_DEPENDENCIES quadrotor_msgs)
endif()
message(STATUS "trajectory_server: building ${PATHCOVER_EXAMPLE} example")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-march=native -O3 -w)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  polytope_msgs
  roscpp
  visualization_msgs
  ${TRAJECTORY_ROBOT_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs polytope_msgs roscpp visualization_msgs ${TRAJECTORY_ROBOT_DEPENDENCIES}
)

if(PATHCOVER_EXAMPLE STREQUAL "jackal")
  set(PATHCOVER_DIM 2)
else()
  set(PATHCOVER_DIM 3)
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node
  src/trajectory_server.cpp
  src/trajectory_server_node.cpp
)
target_compile_definitions(${PROJECT_NAME}_node PRIVATE PATHCOVER_DIM=${PATHCOVER_DIM})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
