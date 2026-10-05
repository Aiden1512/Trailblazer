cmake_minimum_required(VERSION 3.8)
project(trailblazer_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# All five GUI files are installed into the same lib/<pkg> directory so
# that main.py's plain "from state import TrailState" style imports keep
# resolving after install (Python adds a script's own directory to
# sys.path, and the sibling modules live right next to it here).
install(PROGRAMS
  main.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  state.py
  map_canvas.py
  phase1_teleop.py
  phase2_inspection.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright/license linting until headers are added; remove once they are.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
