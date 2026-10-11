cmake_minimum_required(VERSION 3.20)
project(lunabotics_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  launch
  meshes
  rviz
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python helper scripts (belt animator, URDF fixer/patcher) as
# executables under lib/${PROJECT_NAME} so `ros2 run` / launch files
# can find them on the path.
install(PROGRAMS
  scripts/belt_animator.py
  scripts/fix_kinematics.py
  scripts/add_ros2_control.py
  scripts/generate_arena_world.py
  scripts/bekker_terramechanics.py
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
