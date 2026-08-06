cmake_minimum_required(VERSION 3.16)
project(trajectory_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-march=native -O3 -w)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  polytope_msgs
  quadrotor_msgs
  roscpp
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs polytope_msgs quadrotor_msgs roscpp visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/trajectory_server_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
