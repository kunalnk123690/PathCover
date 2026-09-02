cmake_minimum_required(VERSION 3.22)
project(corridor_planning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-march=native -O3 -w)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(polytope_msgs REQUIRED)
find_package(jps_lib REQUIRED)
find_package(qhull_lib REQUIRED)

set(dependencies
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
  "std_msgs"
  "visualization_msgs"
  "polytope_msgs"
  "jps_lib"
  "qhull_lib"
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

file(GLOB SRC_FILES src/*.cpp)
add_executable(${PROJECT_NAME}_node ${SRC_FILES})
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})
target_link_libraries(${PROJECT_NAME}_node
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
