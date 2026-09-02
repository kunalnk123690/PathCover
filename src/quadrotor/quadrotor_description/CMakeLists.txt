cmake_minimum_required(VERSION 3.22)
project(quadrotor_description)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-march=native -O3 -w)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

set(dependencies 
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "tf2_ros"
)


# Add the ground_truth_node for rviz
file(GLOB SRC_FILES src/*.cpp)
add_executable(ground_truth_node ${SRC_FILES})
ament_target_dependencies(ground_truth_node ${dependencies})


install(TARGETS ground_truth_node DESTINATION lib/${PROJECT_NAME} )
install(DIRECTORY
  launch 
  config
  urdf
  meshes
  worlds
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
