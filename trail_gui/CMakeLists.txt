cmake_minimum_required(VERSION 3.8)
project(trailblazer_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Every GUI module is installed next to main.py in lib/<pkg>, so main.py's
# plain "from state import TrailState" style imports keep resolving after
# install (Python adds the script's own directory to sys.path).
install(PROGRAMS
  main.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  camera_feed.py
  camera_panel.py
  camera_window.py
  legend_window.py
  map_canvas.py
  mission_graph.py
  mission_status.py
  phase1_teleop.py
  phase2_inspection.py
  robot_pose.py
  ros_context.py
  sim_obstacles.py
  start_screen.py
  state.py
  Floor_Texture.png
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright/license linting until headers are added; remove once they are.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()