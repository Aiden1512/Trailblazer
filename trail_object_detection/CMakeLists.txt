cmake_minimum_required(VERSION 3.8)
project(trail_object_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# --- Include headers from include/trail_object_detection ---
include_directories(include)

# --- Ground filter node ---
add_executable(ground_filter src/ground_filter.cpp)
ament_target_dependencies(ground_filter rclcpp sensor_msgs std_msgs)

# --- Install the compiled executable so `ros2 run` can find it ---
install(TARGETS
  ground_filter
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()