cmake_minimum_required(VERSION 3.10)
project(occupancy_inflation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
# -march=native is scoped to C++ only: it is a host-compiler flag nvcc does not
# understand, so applying it to the CUDA (.cu) source would fail the GPU build.
add_compile_options(-O3 -w $<$<COMPILE_LANGUAGE:CXX>:-march=native>)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_srvs
)

find_package(Eigen3 REQUIRED)

# The dilation loops are embarrassingly parallel over rows and planes. Without
# OpenMP everything still builds and runs, single threaded.
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  message(STATUS "occupancy_inflation: OpenMP enabled")
else()
  message(STATUS "occupancy_inflation: OpenMP not found, building single threaded")
endif()

# --- Optional CUDA-accelerated dilation --------------------------------------
# If a CUDA compiler is available, inflation_cuda.cu is built in and the
# dilation runs on the GPU; otherwise the same shifted ORs run on the CPU, with
# a bit-identical result. The node also falls back at runtime when no device
# answers, so a GPU build still runs on a machine without one.
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 61 75 86)
  endif()
  set(CMAKE_CUDA_STANDARD 17)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  # Full nvcc and PTX assembler optimization, and -w to match the C++ build.
  # No --use_fast_math: the kernels are pure integer bit arithmetic, which
  # gains nothing from it and must stay exact for the GPU and CPU grids to
  # agree bit for bit.
  set(INFLATION_CUDA_OPT_FLAGS -O3 -Xptxas=-O3 -Xcompiler=-O3 -w)
  set(INFLATION_CUDA_FOUND TRUE)
  message(STATUS "occupancy_inflation: CUDA compiler found, building GPU-accelerated dilation")
else()
  set(INFLATION_CUDA_FOUND FALSE)
  message(STATUS "occupancy_inflation: no CUDA compiler found, building CPU-only")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES point_cloud_inflater
  CATKIN_DEPENDS roscpp sensor_msgs std_srvs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Deliberately free of ROS and PCL so the morphology can be tested on its own.
set(INFLATER_SRC
  src/voxel_bit_grid.cpp
  src/point_cloud_inflater.cpp
)
if(INFLATION_CUDA_FOUND)
  list(APPEND INFLATER_SRC src/inflation_cuda.cu)
endif()

add_library(point_cloud_inflater ${INFLATER_SRC})
target_link_libraries(point_cloud_inflater ${catkin_LIBRARIES})
if(OpenMP_CXX_FOUND)
  target_link_libraries(point_cloud_inflater OpenMP::OpenMP_CXX)
endif()

# PUBLIC, not PRIVATE: the define changes what inflation_cuda.h declares, so
# anything including point_cloud_inflater.h has to see the same header the
# library was built against.
if(INFLATION_CUDA_FOUND)
  target_compile_definitions(point_cloud_inflater PUBLIC OCCUPANCY_INFLATION_WITH_CUDA)
  target_compile_options(point_cloud_inflater PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:${INFLATION_CUDA_OPT_FLAGS}>)
  target_link_libraries(point_cloud_inflater CUDA::cudart)
  set_target_properties(point_cloud_inflater PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
endif()

add_executable(occupancy_inflation_node src/occupancy_inflation_node.cpp)
target_link_libraries(occupancy_inflation_node
  point_cloud_inflater
  ${catkin_LIBRARIES}
)
add_dependencies(occupancy_inflation_node ${catkin_EXPORTED_TARGETS})

# Standalone timing harness: rosrun occupancy_inflation inflation_benchmark
add_executable(inflation_benchmark tools/inflation_benchmark.cpp)
target_link_libraries(inflation_benchmark point_cloud_inflater)

install(TARGETS point_cloud_inflater occupancy_inflation_node inflation_benchmark
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  # Compares the word-parallel inflation against a brute-force reference.
  # Plain main(), no gtest, so it also builds outside catkin.
  add_executable(inflation_test EXCLUDE_FROM_ALL test/inflation_test.cpp)
  target_link_libraries(inflation_test point_cloud_inflater)
  add_test(NAME inflation_test COMMAND inflation_test)
  add_dependencies(tests inflation_test)
endif()
