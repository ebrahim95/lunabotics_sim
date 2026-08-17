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
set(MOON_DEM_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/../../moon_base_map/ezrassor_sim_gazebo/models/dem_moon/materials/textures/dem_moon.jpg")
set(MOON_VISUAL_MESH "${CMAKE_CURRENT_BINARY_DIR}/lunar_heightmap_visual.obj")
set(MOON_VISUAL_MTL "${CMAKE_CURRENT_BINARY_DIR}/lunar_heightmap_visual.mtl")
set(MOON_VISUAL_MESH_GENERATOR "${CMAKE_CURRENT_BINARY_DIR}/generate_lunar_heightmap_mesh.py")
# Make CMake reconfigure whenever the generator changes.
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_gazebo_model.py"
  "${CMAKE_CURRENT_BINARY_DIR}/generate_gazebo_model.py"
  COPYONLY
)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_lunar_heightmap_mesh.py"
  "${MOON_VISUAL_MESH_GENERATOR}"
  COPYONLY
)
execute_process(
  COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_BINARY_DIR}/generate_gazebo_model.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/urdf/assembly_1.urdf"
  OUTPUT_FILE "${GAZEBO_MODEL_SDF}"
  RESULT_VARIABLE GAZEBO_MODEL_RESULT
)
if(NOT GAZEBO_MODEL_RESULT EQUAL 0)
  message(FATAL_ERROR "Could not generate Gazebo model from assembly_1.urdf")
endif()

# Harmonic renders visual heightmaps at a different offset from their physics
# heightfields.  Generate a small OBJ from the same DEM for an aligned visual.
execute_process(
  COMMAND ${Python3_EXECUTABLE}
    "${MOON_VISUAL_MESH_GENERATOR}"
    "${MOON_DEM_SOURCE}"
    "${MOON_VISUAL_MESH}"
  RESULT_VARIABLE MOON_VISUAL_MESH_RESULT
)
if(NOT MOON_VISUAL_MESH_RESULT EQUAL 0)
  message(FATAL_ERROR "Could not generate lunar heightmap visual mesh")
endif()

install(DIRECTORY
  urdf
  launch
  meshes
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Reuse the lightweight public-domain lunar DEM and its 512px surface image
# from the locally checked-out moon_base_map repository.  They are copied into
# the installed Gazebo model, so runtime does not depend on that repository.
install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/../../moon_base_map/ezrassor_sim_gazebo/models/dem_moon/materials/textures/dem_moon.jpg"
  DESTINATION share/${PROJECT_NAME}/models/lunar_heightmap/textures
)
install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/../../moon_base_map/ezrassor_sim_gazebo/models/dem_moon/materials/textures/AS16-110-18026HR-512x512.jpg"
  DESTINATION share/${PROJECT_NAME}/models/lunar_heightmap/textures
  RENAME lunar_surface.jpg
)
install(FILES "${MOON_VISUAL_MESH}" "${MOON_VISUAL_MTL}"
  DESTINATION share/${PROJECT_NAME}/models/lunar_heightmap/meshes
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
