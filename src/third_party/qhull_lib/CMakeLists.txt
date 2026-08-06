cmake_minimum_required(VERSION 3.16)
project(qhull_lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-march=native -O3 -w)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB LIB_FILES src/libqhull_r/*.c src/libqhullcpp/*.cpp)
add_library(${PROJECT_NAME} ${LIB_FILES})

target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  cdd
)

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
