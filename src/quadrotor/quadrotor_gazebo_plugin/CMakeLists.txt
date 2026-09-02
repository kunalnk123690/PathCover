cmake_minimum_required(VERSION 3.22)
project(quadrotor_gazebo_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-march=native -O3 -w)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(quadrotor_msgs REQUIRED)

set(DEPENDENCIES "rclcpp" "quadrotor_msgs")

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)


gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(IGN_GAZEBO_VER ${gz-sim8_VERSION_MAJOR})


##### Plugin #####
file(GLOB_RECURSE LIB_FILES src/*.cpp)
add_library(QuadrotorGazeboPlugin SHARED ${LIB_FILES})
target_link_libraries(QuadrotorGazeboPlugin
  gz-plugin${IGN_PLUGIN_VER}::gz-plugin${IGN_PLUGIN_VER}
  gz-sim${IGN_GAZEBO_VER}::gz-sim${IGN_GAZEBO_VER}
)

ament_target_dependencies(QuadrotorGazeboPlugin
  ${DEPENDENCIES}
)

install(
  TARGETS QuadrotorGazeboPlugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_package()
