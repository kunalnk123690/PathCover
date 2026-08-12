cmake_minimum_required(VERSION 3.16)
project(quadrotor_gazebo_plugin)

set(PATHCOVER_EXAMPLE "quadrotor" CACHE STRING "PathCover example to build (quadrotor or jackal)")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-march=native -O3 -w)

if(PATHCOVER_EXAMPLE STREQUAL "quadrotor")
  find_package(Boost REQUIRED)
  find_package(gazebo REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(catkin REQUIRED COMPONENTS gazebo_ros geometry_msgs roscpp quadrotor_msgs nav_msgs tf)
  catkin_package(CATKIN_DEPENDS gazebo_ros geometry_msgs roscpp quadrotor_msgs nav_msgs tf)
  include_directories(include ${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

  add_library(${PROJECT_NAME} SHARED src/quadrotor_gazebo_plugin.cpp)
  add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  target_link_libraries(${PROJECT_NAME}
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    Eigen3::Eigen
  )

  install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
else()
  find_package(catkin REQUIRED)
  catkin_package()
endif()
