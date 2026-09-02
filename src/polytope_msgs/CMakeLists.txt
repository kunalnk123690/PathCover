cmake_minimum_required(VERSION 3.22)
project(polytope_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(MSG_FILES
"msg/Polytope.msg"
"msg/PolytopeArray.msg"
)


rosidl_generate_interfaces(${PROJECT_NAME} ${MSG_FILES} DEPENDENCIES builtin_interfaces std_msgs)


ament_export_include_directories(custom_structs) 
ament_export_dependencies(rosidl_default_runtime)


ament_package()