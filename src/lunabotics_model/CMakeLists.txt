cmake_minimum_required(VERSION 3.20)
project(lunabotics_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Create the Gazebo model from the CAD URDF at configure time.  The generator
# flattens the fixed CAD assembly into visuals and leaves the four wheels as
# physics joints for the AWD controller.
set(GAZEBO_MODEL_SDF "${CMAKE_CURRENT_BINARY_DIR}/lunabotics_rover.sdf")
execute_process(
  COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_gazebo_model.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/urdf/assembly_1.urdf"
  OUTPUT_FILE "${GAZEBO_MODEL_SDF}"
  RESULT_VARIABLE GAZEBO_MODEL_RESULT
)
if(NOT GAZEBO_MODEL_RESULT EQUAL 0)
  message(FATAL_ERROR "Could not generate Gazebo model from assembly_1.urdf")
endif()

install(DIRECTORY
  urdf
  launch
  meshes
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Gazebo resolves model://lunabotics_rover/meshes/... relative to the model.
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/models/lunabotics_rover/meshes
)

install(FILES ${GAZEBO_MODEL_SDF}
  DESTINATION share/${PROJECT_NAME}/models/lunabotics_rover
  RENAME model.sdf
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
