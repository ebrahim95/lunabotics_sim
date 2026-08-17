# lunabotics_sim

## Build and source the workspace:

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
## RVIZ
<img width="1872" height="1048" alt="image" src="https://github.com/user-attachments/assets/6d366b0e-7fa7-4492-a5d4-8e7d840402d0" />

## Gazebo
<img width="1872" height="1048" alt="Full_CAD" src="https://github.com/user-attachments/assets/24301015-f4ee-4e71-af4b-d10934a81097" />
