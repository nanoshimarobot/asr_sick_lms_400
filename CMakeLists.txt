cmake_minimum_required(VERSION 3.8)
project(lms400)

add_definitions(-std=c++17)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(
  ${PROJECT_NAME}_component SHARED
  src/${PROJECT_NAME}_component.cpp
  src/sick_lms400.cpp
)
rclcpp_components_register_nodes(${PROJECT_NAME}_component "LMS400")
target_compile_options(${PROJECT_NAME}_component PUBLIC -Wall)

ament_auto_add_executable(${PROJECT_NAME} src/main.cpp)

ament_auto_package()