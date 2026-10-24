cmake_minimum_required(VERSION 3.8)
project(trail_object_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(ground_filter src/ground_filter.cpp)
ament_target_dependencies(ground_filter rclcpp sensor_msgs std_msgs)

install(TARGETS ground_filter DESTINATION lib/${PROJECT_NAME})

# ROS-free tests (plain asserts): `colcon test --packages-select trail_object_detection`
if(BUILD_TESTING)
  add_executable(test_obstacle_logic test/test_obstacle_logic.cpp)
  add_test(NAME test_obstacle_logic COMMAND test_obstacle_logic)
  add_executable(test_scenarios test/test_scenarios.cpp)
  add_test(NAME test_scenarios COMMAND test_scenarios)
endif()

ament_package()
