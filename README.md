# lunabotics_sim

ROS 2 (Lyrical) description and Gazebo simulation of our Lunabotics rover: a
four-wheel skid-steer chassis with a bucket-ladder excavator. The model comes
from an Onshape export and is repaired by scripts so it drives and digs the
way the real robot does.

## Requirements

- ROS 2 Lyrical with Gazebo (`ros_gz_sim`, `ros_gz_bridge`)
- `ros2_control`, `ros2_controllers` and `gz_ros2_control` at matching versions:

```bash
sudo apt install ros-lyrical-gz-ros2-control
sudo apt update && sudo apt install --only-upgrade 'ros-lyrical-*'
```

> If Gazebo segfaults inside `libgz_ros2_control-system.so` on startup,
> `gz_ros2_control` was built against a newer `ros2_control` than the one
> installed. Run the upgrade command above.

## Build

```bash
cd ~/Desktop/lunabotics_sim
source /opt/ros/lyrical/setup.bash
colcon build --packages-select lunabotics_model
source install/setup.bash
```

## Run

### Gazebo simulation

```bash
ros2 launch lunabotics_model gazebo.launch.py
```

Opens Gazebo and RViz, spawns the robot on the ground and starts all
controllers (ready after ~7 s, when `Configured and activated
excavator_lift_controller` is logged).

### Model viewer only (no physics)

```bash
ros2 launch lunabotics_model display.launch.py
```

Opens RViz with a joint slider window. Drag `drive_roller_joint` to run the
buckets round the belt. In RViz set **Fixed Frame** to `root` and add a
**RobotModel** display.

## Controlling the robot

All controllers take `std_msgs/msg/Float64MultiArray` on
`/<controller>/commands`. Run these in a second terminal after
`source install/setup.bash`.

| Controller | Joints | Units |
|---|---|---|
| `wheel_velocity_controller` | `revolute_1`, `revolute_1_1`, `revolute_1_2`, `revolute_1_3` | rad/s, + drives toward the excavator |
| `belt_velocity_controller` | `drive_roller_joint` | rad/s |
| `excavator_lift_controller` | `excavator_lift` | m along the rails, -0.15 (down) to +0.15 (up) |
| `bucket_position_controller` | 30 bucket track joints | driven by `belt_animator.py`, don't command directly |

Wheel order is **not** left/left/right/right: `revolute_1` + `revolute_1_3`
are one side, `revolute_1_1` + `revolute_1_2` the other.

```bash
# Drive forward (negative = reverse)
ros2 topic pub -w 1 --times 3 /wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [2.0, 2.0, 2.0, 2.0]}"

# Turn in place
ros2 topic pub -w 1 --times 3 /wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0, -1.0, -1.0, 1.0]}"

# Stop
ros2 topic pub -w 1 --times 3 /wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}"

# Run the excavator belt (0.0 to stop)
ros2 topic pub -w 1 --times 3 /belt_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [3.0]}"

# Lower the ladder to dig, then raise it back
ros2 topic pub -w 1 --times 3 /excavator_lift_controller/commands std_msgs/msg/Float64MultiArray "{data: [-0.15]}"
ros2 topic pub -w 1 --times 3 /excavator_lift_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0]}"
```

Keep `-w 1 --times 3`: without it `ros2 topic pub` can exit before the
message is delivered, and the controllers keep executing their last command.

## Updating the model from Onshape

Export the URDF from Onshape into `src/lunabotics_model/urdf/assembly_1.urdf`
(meshes into `meshes/`), then regenerate everything derived from it:

```bash
cd src/lunabotics_model
python3 scripts/fix_kinematics.py --input urdf/assembly_1.urdf --output urdf/assembly_1_fixed.urdf
python3 scripts/add_ros2_control.py --input urdf/assembly_1_fixed.urdf --output urdf/assembly_1_sim.urdf
cd ../.. && colcon build --packages-select lunabotics_model
```

Never hand-edit the generated files; change the scripts instead.

## How it works

### Files

| Path | What it is |
|---|---|
| `urdf/assembly_1.urdf` | Raw Onshape export |
| `urdf/assembly_1_fixed.urdf` | Repaired model, *generated* by `fix_kinematics.py` (used by RViz) |
| `urdf/assembly_1_sim.urdf` | Repaired model + `<ros2_control>`/Gazebo plugin, *generated* by `add_ros2_control.py` (used by Gazebo) |
| `config/controllers.yaml` | Controller types and joints |
| `config/belt_animator.yaml`, `config/bucket_controller.yaml` | Belt geometry and bucket joints, *generated* by `fix_kinematics.py` |
| `scripts/fix_kinematics.py` | Repairs the Onshape export (see below) |
| `scripts/add_ros2_control.py` | Adds ros2_control hardware blocks and bucket start positions |
| `scripts/belt_animator.py` | Moves the buckets round the belt, following the drive roller |
| `launch/gazebo.launch.py`, `launch/display.launch.py` | Simulation and viewer |

### What `fix_kinematics.py` repairs

Every part keeps its exact CAD pose (the script refuses to write if anything
moves by more than 1 µm).

- **Wheels.** The export fastens all four wheels to one flange, so only one
  joint turned anything. Each wheel is now mounted on its own flange and all
  axes point the same way. Onshape's `<mimic>` couplings are removed.
- **Chassis and excavator.** The export hangs the excavator off the base and
  dangles the whole frame from the belt. The frame is now fixed to `root`,
  and the ladder rides on a prismatic `excavator_lift` joint along its rails,
  with a driven top roller (`drive_roller_joint`) and a free bottom roller
  (`idler_roller_joint`). Massless `*_loop_closure` dummy links are dropped.
- **Belt.** A URDF can't express a belt, so each bucket gets three joints on
  the ladder (`<bucket>_belt_u` along it, `_belt_v` across it, `_belt_rot`
  about the shafts). `belt_animator.py` reads the drive roller angle and sets
  them so the buckets run round the loop, as real bodies in Gazebo.
- **Collisions.** The export has none. Boxes are fitted to the extrusions and
  motor parts, cylinders to the wheels, rollers and shafts, and the low-poly
  bucket and bin meshes are used directly. Edit `COLLISION_SHAPES` in the
  script to change a part's shape.

Tunable values (lift travel and effort, roller and bucket limits) are at the
top of `fix_kinematics.py`.

```
root
 |- motors -> revolute_1* -> flange -> wheel        (x4)
 '- chassis frame ... slider_bracket
      '- excavator_lift (prismatic) -> ladder (side plates, shafts, belt)
           |- drive_roller_joint -> top roller
           |- idler_roller_joint -> bottom roller
           '- <bucket>_belt_u -> _belt_v -> _belt_rot -> bucket   (x10)
```

## Known limitations

- The ground is rigid, so buckets scrape it rather than dig; a regolith model
  is still to do.
- Buckets lag their target by ~10 mm while the belt runs (they settle exactly
  when it stops). Belt speed is computed at the 48.5 mm bucket mount radius,
  about 3% faster than the 47 mm roller surface.
- The top roller is assumed to be the driven one and the lift travel (±0.15 m)
  is an estimate; both are set in `fix_kinematics.py`.
- Many CAD parts have unrealistic masses (motors ~3 mg) because materials
  weren't assigned in Onshape.
- Without a working NVIDIA driver setup the Gazebo GUI may fall back to
  software rendering (`libEGL warning ... driver (null)`); physics is
  unaffected.
