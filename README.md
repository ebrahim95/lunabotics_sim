# lunabotics_sim

Build and source the workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lunabotics_model --symlink-install
source install/setup.bash
```

Launch the standard flat driving world:

```bash
ros2 launch lunabotics_model gazebo.launch.py
```

Launch the lightweight Moon world (native 513 × 513 heightmap, lunar gravity,
sunlight, and the Lunabotics rover):

```bash
ros2 launch lunabotics_model moon.launch.py
```
Controls
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```