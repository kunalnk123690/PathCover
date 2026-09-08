cmake_minimum_required(VERSION 3.18)
project(nanovoxmap_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  nav_msgs
  roscpp
  sensor_msgs
)

set(NANOVOXMAP_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/nanovoxmap")
if(NOT EXISTS "${NANOVOXMAP_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "NanoVoxMap submodule is missing. Run: git submodule update --init --recursive")
endif()

# Build only the core library when NanoVoxMap is embedded in this ROS package.
set(BUILD_EXAMPLES OFF CACHE BOOL "Build NanoVoxMap examples" FORCE)
set(BUILD_DOCS OFF CACHE BOOL "Build NanoVoxMap documentation" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Build NanoVoxMap tests" FORCE)
add_subdirectory("${NANOVOXMAP_SOURCE_DIR}"
                 "${CMAKE_CURRENT_BINARY_DIR}/nanovoxmap_core")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs message_filters nav_msgs roscpp sensor_msgs
)

add_executable(${PROJECT_NAME}_node src/main.cpp src/NanoVoxMapNode.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node nanovoxmap::nanovoxmap ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
