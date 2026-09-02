cmake_minimum_required(VERSION 3.22)
project(jps_lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-march=native -O3 -w)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Source files
file(GLOB_RECURSE LIB_FILES "src/*.cpp")

# Create library
file(GLOB JPS_FILES "src/jps_planner/*.cpp")
file(GLOB DMP_FILES "src/distance_map_planner/*.cpp")
add_library(${PROJECT_NAME} ${JPS_FILES} ${DMP_FILES})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# Dependencies for linking and export
ament_target_dependencies(${PROJECT_NAME} rclcpp)

# Export include dirs
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/
)

# Export package
ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(rclcpp Eigen3)

ament_package()
