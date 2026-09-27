cmake_minimum_required(VERSION 3.8)
project(husky_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


add_executable(path_recorder src/path_recorder.cpp)
ament_target_dependencies(path_recorder
  rclcpp
  geometry_msgs
  nav_msgs
)


install(TARGETS
  path_recorder
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()