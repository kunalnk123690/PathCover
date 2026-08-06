cmake_minimum_required(VERSION 3.22)
project(nanovoxmap)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
# -march=native is scoped to C++ only: it is a host-compiler flag nvcc does not
# understand, so applying it to the CUDA (.cu) source would fail the GPU build.
add_compile_options(-O3 -w $<$<COMPILE_LANGUAGE:CXX>:-march=native>)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  message_filters
)

catkin_package(CATKIN_DEPENDS
  roscpp
  geometry_msgs
  sensor_msgs
  message_filters
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# --- Optional CUDA-accelerated point-cloud raycasting ----------------------
# If a CUDA compiler is available, nanovoxmap_cuda.cu is built in and
# insertPointCloud() runs on the GPU; otherwise the node falls back to the
# pure-CPU path with no functional difference.
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
  # Device-code optimization: full nvcc optimization and aggressive PTX
  # assembler optimization. Also suppress nvcc warnings to match the C++ (-w)
  # build.
  #
  # Deliberately NOT --use_fast_math: the raycast kernels are pure integer
  # arithmetic and gain nothing from it, while the ESDF solver's exactness
  # depends on IEEE division and sqrt -- approximate intrinsics would perturb
  # the distance transform's envelope breakpoints and let the GPU field drift
  # from the CPU one, which is precisely the equivalence the fallback relies on.
  set(NANOVOXMAP_CUDA_OPT_FLAGS
    -O3
    -Xptxas=-O3
    -Xcompiler=-O3
    -w
  )
  set(NANOVOXMAP_CUDA_FOUND TRUE)
  message(STATUS "nanovoxmap: CUDA compiler found, building GPU-accelerated raycast")
else()
  set(NANOVOXMAP_CUDA_FOUND FALSE)
  message(STATUS "nanovoxmap: no CUDA compiler found, building CPU-only")
endif()

file(GLOB SRC_FILES src/*.cpp)
if(NANOVOXMAP_CUDA_FOUND)
  list(APPEND SRC_FILES src/nanovoxmap_cuda.cu)
endif()

add_executable(${PROJECT_NAME}_node ${SRC_FILES})
target_link_libraries( ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

if(NANOVOXMAP_CUDA_FOUND)
  target_compile_definitions(${PROJECT_NAME}_node PRIVATE NANOVOXMAP_WITH_CUDA)
  target_compile_options(${PROJECT_NAME}_node PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:${NANOVOXMAP_CUDA_OPT_FLAGS}>)
  target_link_libraries(${PROJECT_NAME}_node CUDA::cudart)
  set_target_properties(${PROJECT_NAME}_node PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
endif()

# --- Signed-ESDF correctness / benchmark checks -----------------------------
# Built unconditionally rather than behind catkin's test machinery: the map is
# a ROS-independent header, and the three ESDF paths that have to agree (CPU
# transform, CUDA transform, incremental scheduler) are worth being able to
# check with a single binary on any machine. Run ./nanovoxmap_esdf_test; a
# non-zero exit means a check failed. Picks up the CUDA backend automatically
# when one was found, and reports it as skipped otherwise.
set(ESDF_TEST_SRC test/esdf_test.cpp)
if(NANOVOXMAP_CUDA_FOUND)
  list(APPEND ESDF_TEST_SRC src/nanovoxmap_cuda.cu)
endif()

add_executable(${PROJECT_NAME}_esdf_test ${ESDF_TEST_SRC})
target_link_libraries(${PROJECT_NAME}_esdf_test Eigen3::Eigen)

if(NANOVOXMAP_CUDA_FOUND)
  target_compile_definitions(${PROJECT_NAME}_esdf_test PRIVATE NANOVOXMAP_WITH_CUDA)
  target_compile_options(${PROJECT_NAME}_esdf_test PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:${NANOVOXMAP_CUDA_OPT_FLAGS}>)
  target_link_libraries(${PROJECT_NAME}_esdf_test CUDA::cudart)
  set_target_properties(${PROJECT_NAME}_esdf_test PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
endif()