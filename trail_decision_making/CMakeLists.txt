cmake_minimum_required(VERSION 3.8)
project(trail_decision_making)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)


# ROS-independent core: graph + planner (easy to unit test)
add_library(${PROJECT_NAME}_core STATIC
  src/trail_graph.cpp
  src/route_planner.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_core PUBLIC yaml-cpp)

# ROS node
add_executable(mission_manager_node
  src/mission_manager.cpp
  src/main.cpp)
target_link_libraries(mission_manager_node ${PROJECT_NAME}_core)
ament_target_dependencies(mission_manager_node
  rclcpp rclcpp_action nav2_msgs geometry_msgs std_msgs std_srvs tf2_ros tf2_geometry_msgs tf2)

install(TARGETS mission_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(test_route_planner test/test_route_planner.cpp)
#   target_link_libraries(test_route_planner ${PROJECT_NAME}_core)
# endif()

ament_package()