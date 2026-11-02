cmake_minimum_required(VERSION 3.8)
project(trail_object_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_executable(ground_filter src/ground_filter.cpp)
ament_target_dependencies(ground_filter rclcpp sensor_msgs std_msgs tf2_ros tf2)
target_link_libraries(ground_filter yaml-cpp)

add_executable(water_guard src/water_guard.cpp)
ament_target_dependencies(water_guard rclcpp geometry_msgs sensor_msgs std_msgs)

install(TARGETS ground_filter water_guard DESTINATION lib/${PROJECT_NAME})

# ROS-free tests (plain asserts): `colcon test --packages-select trail_object_detection`
if(BUILD_TESTING)
  add_executable(test_obstacle_logic test/test_obstacle_logic.cpp)
  add_test(NAME test_obstacle_logic COMMAND test_obstacle_logic)
  add_executable(test_scenarios test/test_scenarios.cpp)
  add_test(NAME test_scenarios COMMAND test_scenarios)
  add_executable(test_water_logic test/test_water_logic.cpp)
  add_test(NAME test_water_logic COMMAND test_water_logic)
  add_executable(test_trail_logic test/test_trail_logic.cpp)
  add_test(NAME test_trail_logic COMMAND test_trail_logic)
  add_executable(test_trail_keeper_logic test/test_trail_keeper_logic.cpp)
  add_test(NAME test_trail_keeper_logic COMMAND test_trail_keeper_logic)
  add_executable(test_scenery_map test/test_scenery_map.cpp)
  target_link_libraries(test_scenery_map yaml-cpp)
  target_compile_definitions(test_scenery_map PRIVATE
    TRAIL_MAP_YAML="${CMAKE_CURRENT_SOURCE_DIR}/../41068_ignition_bringup/maps/trail_map.yaml")
  add_test(NAME test_scenery_map COMMAND test_scenery_map)
endif()

ament_package()
