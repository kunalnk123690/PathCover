cmake_minimum_required(VERSION 3.16)
project(corridor_planning)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-march=native -O3 -w)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  roscpp
  sensor_msgs
  jps_lib
  visualization_msgs
  qhull_lib
  polytope_msgs
)


catkin_package(CATKIN_DEPENDS 
  roscpp 
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  jps_lib
  qhull_lib
  polytope_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS} 
)

file(GLOB SRC_FILES src/*.cpp)
add_executable(${PROJECT_NAME}_node ${SRC_FILES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES} 
  Eigen3::Eigen 
  ${OpenCV_LIBS}
)