cmake_minimum_required(VERSION 3.16)
project(quadrotor_description)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-march=native -O3 -w)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  gazebo_msgs
  tf
)


catkin_package(CATKIN_DEPENDS 
  geometry_msgs 
  nav_msgs 
  roscpp 
  sensor_msgs 
  gazebo_msgs
  tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB SRC_FILE src/*.cpp)
add_executable(quadrotor_ground_truth_node ${SRC_FILE})
target_link_libraries(quadrotor_ground_truth_node ${catkin_LIBRARIES})



install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})