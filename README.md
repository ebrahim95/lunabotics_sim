# lunabotics_sim

## Install dependencies

```bash
sudo apt install ros-lyrical-gz-ros2-control ros-lyrical-teleop-twist-keyboard
sudo apt update && sudo apt install --only-upgrade 'ros-lyrical-*'
```

## Build

```bash
cd ~/Desktop/lunabotics_sim
source /opt/ros/lyrical/setup.bash
colcon build --packages-select lunabotics_model
source install/setup.bash
```

## Run

Gazebo simulation:

```bash
ros2 launch lunabotics_model gazebo.launch.py
```

Gazebo simulation rendered on the NVIDIA GPU:

```bash
ros2 launch lunabotics_model gazebo.launch.py gpu:=true
```

RViz model viewer (no physics):

```bash
ros2 launch lunabotics_model display.launch.py
```

In RViz, set **Fixed Frame** to `root` and add a **RobotModel** display.

## Control

Open a second terminal and run:

```bash
cd ~/Desktop/lunabotics_sim && source install/setup.bash
```

Drive with the keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p use_sim_time:=true
```

| Key | Action |
|---|---|
| `i` | Forward |
| `,` | Reverse |
| `j` / `l` | Turn left / right |
| `u` / `o` | Forward, curving left / right |
| `m` / `.` | Reverse, curving left / right |
| `k` | Stop |
| `q` / `z` | Increase / decrease speed |

Run the excavator belt (`0.0` to stop):

```bash
ros2 topic pub -w 1 --times 3 /belt_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [3.0]}"
```

Lower the ladder (range `-0.15` to `0.15`), then raise it back:

```bash
ros2 topic pub -w 1 --times 3 /excavator_lift_controller/commands std_msgs/msg/Float64MultiArray "{data: [-0.15]}"
ros2 topic pub -w 1 --times 3 /excavator_lift_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0]}"
```

## Update the model from Onshape

Export the URDF to `src/lunabotics_model/urdf/assembly_1.urdf` and the meshes to `src/lunabotics_model/meshes/`, then run:

```bash
cd ~/Desktop/lunabotics_sim/src/lunabotics_model
python3 scripts/fix_kinematics.py --input urdf/assembly_1.urdf --output urdf/assembly_1_fixed.urdf
python3 scripts/add_ros2_control.py --input urdf/assembly_1_fixed.urdf --output urdf/assembly_1_sim.urdf
cd ~/Desktop/lunabotics_sim && colcon build --packages-select lunabotics_model
```
