cmake_minimum_required(VERSION 3.18)
# C is enabled because PCL pulls in VTK, whose MPI target needs the C language.
project(nanovoxmap VERSION 1.0.0 LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ------------------------------------------------------------
find_package(Eigen3 REQUIRED NO_MODULE)

# --- Build options -----------------------------------------------------------
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  set(NANOVOXMAP_CUDA_DEFAULT ON)
else()
  set(NANOVOXMAP_CUDA_DEFAULT OFF)
endif()
option(NANOVOXMAP_WITH_CUDA "Build NanoVoxMap with CUDA acceleration" ${NANOVOXMAP_CUDA_DEFAULT})
option(ENABLE_NATIVE_BUILD_OPTIMIZATIONS "Enable native build optimization flags" ON)
include(CTest)

# --- The library -------------------------------------------------------------
# NanoVoxMap is header-only on the CPU path; it only gains a compiled
# translation unit (nanovoxmap_cuda.cu) when CUDA is enabled. So it is built as
# an INTERFACE library by default and promoted to a real static/shared library
# carrying the .cu when NANOVOXMAP_WITH_CUDA is on.
if(NANOVOXMAP_WITH_CUDA)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  find_package(Threads REQUIRED)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES AND CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
    set(CMAKE_CUDA_ARCHITECTURES "native")
  endif()

  add_library(${PROJECT_NAME} src/nanovoxmap_cuda.cu)
  set_target_properties(${PROJECT_NAME} PROPERTIES
    CUDA_STANDARD 17
    CUDA_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON)
  target_compile_options(${PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)
  target_compile_definitions(${PROJECT_NAME} PUBLIC NANOVOXMAP_WITH_CUDA)
  target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen CUDA::cudart Threads::Threads)
  set(NANOVOXMAP_SCOPE PUBLIC)
else()
  add_library(${PROJECT_NAME} INTERFACE)
  target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
  set(NANOVOXMAP_SCOPE INTERFACE)
endif()

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} ${NANOVOXMAP_SCOPE}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} ${NANOVOXMAP_SCOPE} cxx_std_17)

if(BUILD_TESTING)
  add_executable(nanovoxmap_core_test test/core_test.cpp)
  target_link_libraries(nanovoxmap_core_test PRIVATE nanovoxmap::nanovoxmap)
  add_test(NAME nanovoxmap_core_test COMMAND nanovoxmap_core_test)
endif()

if(NOT NANOVOXMAP_WITH_CUDA)
  # INTERFACE targets cannot carry private compile options; only relevant for
  # the compiled CUDA path otherwise.
elseif(ENABLE_NATIVE_BUILD_OPTIMIZATIONS)
  # The library's only compiled TU is the .cu, which builds under
  # COMPILE_LANGUAGE:CUDA. nvcc's -O3 optimizes the host portion (device/ptxas
  # code already defaults to -O3); native tuning flags are forwarded to the host
  # compiler via -Xcompiler. The CXX branch is kept for any future CXX sources.
  target_compile_options(${PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-O3 -march=native -mtune=native>
    $<$<COMPILE_LANGUAGE:CUDA>:-O3 -Xcompiler=-O3,-march=native,-mtune=native -Xptxas=-O3>
    -w)
endif()

# --- Install + package export ------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKECONFIG_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/nanovoxmap/nanovoxmap.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/nanovoxmap)

# Export the build tree (lets other projects find_package() this without install).
export(TARGETS ${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")
export(PACKAGE ${PROJECT_NAME})

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKECONFIG_INSTALL_DIR})
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

install(EXPORT ${PROJECT_NAME}Targets
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKECONFIG_INSTALL_DIR})
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CMAKECONFIG_INSTALL_DIR})

# --- Uninstall target --------------------------------------------------------
if(NOT TARGET uninstall)
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)
  add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()


# --- Example / test app ------------------------------------------------------
option(BUILD_EXAMPLES "Build the NanoVoxMap example apps" ON)
if(BUILD_EXAMPLES)
# Example app for NanoVoxMap. PCL is used only here (point-cloud loading and
# occupied-voxel visualization); the core library does not depend on it.
find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization)

add_executable(nanovoxmap_example test/nanovoxmap_example.cpp)
  target_link_libraries(nanovoxmap_example
    PRIVATE nanovoxmap::nanovoxmap ${PCL_LIBRARIES})
  target_include_directories(nanovoxmap_example SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_directories(nanovoxmap_example PRIVATE ${PCL_LIBRARY_DIRS})
  target_compile_definitions(nanovoxmap_example PRIVATE
    ${PCL_DEFINITIONS}
    NANOVOXMAP_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
endif()

# --- API documentation -------------------------------------------------------
# `cmake --build build --target docs` runs Doxygen over the committed Doxyfile,
# emitting HTML into docs/html. Off by default so a plain build needs no Doxygen.
option(BUILD_DOCS "Generate API documentation with Doxygen" OFF)
if(BUILD_DOCS)
  find_package(Doxygen REQUIRED)
  add_custom_target(docs
    COMMAND Doxygen::doxygen ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM)
endif()
