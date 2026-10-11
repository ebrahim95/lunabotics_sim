# lunabotics_sim

For how everything works, see [docs/GUIDE.md](docs/GUIDE.md).

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

Gazebo simulation rendered on the NVIDIA GPU (hybrid-graphics laptops):

```bash
ros2 launch lunabotics_model gazebo.launch.py gpu:=true
```

Gazebo simulation in the Lunabotics Artemis Arena (BP-1 regolith simulant,
craters, rocks, column; the robot starts in the starting zone):

```bash
ros2 launch lunabotics_model gazebo.launch.py world:=artemis_arena
```

Start somewhere else in the arena (metres from the starting-zone corner, heading in radians):

```bash
ros2 launch lunabotics_model gazebo.launch.py world:=artemis_arena x:=1.5 y:=0.5 yaw:=0.3
```

RViz model viewer (no physics):

```bash
ros2 launch lunabotics_model display.launch.py
```

In RViz, set **Fixed Frame** to `base_link` and add a **RobotModel** display.

## Control

Open a second terminal and run:

```bash
cd ~/Desktop/lunabotics_sim && source install/setup.bash
```

Drive with the keyboard (the robot keeps moving until you press `k`):

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

## Change the robot's mass, motor limits or wheel slip

Edit the constants at the top of `src/lunabotics_model/scripts/fix_kinematics.py`
(`TOTAL_MASS`, `ELECTRONICS_MASS`, `BIN_MASS`, `DRIVE_MOTOR_MASS`,
`WHEEL_EFFORT`, `WHEEL_VELOCITY`), then regenerate as in
"Update the model from Onshape" above.

Wheel slip on the regolith: edit `SLIP_COMPLIANCE_LONGITUDINAL` and
`SLIP_COMPLIANCE_LATERAL` at the top of
`src/lunabotics_model/scripts/add_ros2_control.py`, then regenerate the same way.

## Change the Artemis Arena layout

Random rocks and craters, as the judges place them (any seed number):

```bash
cd ~/Desktop/lunabotics_sim/src/lunabotics_model
python3 scripts/generate_arena_world.py --output-dir worlds --seed 7
cd ~/Desktop/lunabotics_sim && colcon build --packages-select lunabotics_model
```

Back to the guidebook layout: run the same command without `--seed`.

## MuJoCo bucket digging test

Set up once:

```bash
cd ~/Desktop/lunabotics_sim
uv venv .venv-mujoco
uv pip install --python .venv-mujoco/bin/python mujoco numpy pyyaml
```

Watch the buckets dig:

```bash
.venv-mujoco/bin/python mujoco_dig/dig_test.py --viewer
```

The sim runs at about 0.17× real time. Keep the window open: buckets carry
regolith up the ladder after ~30 s and it reaches the bin after ~1 min; the
full run takes ~3 min, then the window stays open until you close it. Do not
use the NVIDIA offload variables with this viewer.

Viewer controls: left-drag to rotate, right-drag to pan, scroll to zoom,
space to pause.

Run without a window and print the counts:

```bash
.venv-mujoco/bin/python mujoco_dig/dig_test.py
```

Options:

| Option | Default | Meaning |
|---|---|---|
| `--duration` | `30` | seconds of digging |
| `--speed` | `0.15` | belt speed (m/s) |
| `--lift` | `-0.12` | dig depth: ladder position on its rails (m) |
| `--advance` | `0.005` | robot creep speed into the regolith (m/s) |
| `--start-lift` | `0.0` | ladder position before digging starts (m) |
| `--plunge` | `0.06` | speed the ladder is lowered into the regolith (m/s) |
| `--radius` | `0.012` | regolith particle radius (m) |
| `--layers` | `4` | layers of particles in the pit |
