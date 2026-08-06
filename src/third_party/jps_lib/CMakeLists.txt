cmake_minimum_required(VERSION 3.16)
project(jps_lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-march=native -O3 -w)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jps_lib
  CATKIN_DEPENDS roscpp
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


file(GLOB JPS_FILES "src/jps_planner/*.cpp")
file(GLOB DMP_FILES "src/distance_map_planner/*.cpp")
add_library(${PROJECT_NAME} ${JPS_FILES} ${DMP_FILES})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen ${catkin_LIBRARIES})


install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)