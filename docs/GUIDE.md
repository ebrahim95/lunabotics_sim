# lunabotics_sim — How Everything Works

This guide explains the whole project: the ROS 2 and Gazebo concepts it is
built on, every file in the repo, and what actually happens when you launch
the simulation and press a key. Read it top to bottom once; after that, use
the section headings to jump around.

For the commands to build and run, see the [README](../README.md).

---

## Contents

1. [The big picture](#1-the-big-picture)
2. [ROS 2 in 10 minutes](#2-ros-2-in-10-minutes)
3. [Describing a robot: URDF](#3-describing-a-robot-urdf)
4. [Gazebo](#4-gazebo)
5. [ros2_control](#5-ros2_control)
6. [The repo, file by file](#6-the-repo-file-by-file)
7. [fix_kinematics.py: repairing the CAD export](#7-fix_kinematicspy-repairing-the-cad-export)
8. [The belt: buckets on joints](#8-the-belt-buckets-on-joints)
9. [add_ros2_control.py](#9-add_ros2_controlpy)
10. [What happens when you launch](#10-what-happens-when-you-launch)
11. [What happens when you press a key](#11-what-happens-when-you-press-a-key)
12. [Debugging toolbox](#12-debugging-toolbox)
13. [Gotchas we hit (and why)](#13-gotchas-we-hit-and-why)
14. [Glossary](#14-glossary)

---

## 1. The big picture

The robot is designed in **Onshape** (CAD). Onshape can export it as a
**URDF** — a text file listing every part and how parts connect. That export
is wrong in several ways for simulation, so two Python scripts repair it.
The repaired model is then loaded into **Gazebo** (the physics simulator),
and **ros2_control** connects ROS commands to the robot's joints inside
Gazebo.

```
 Onshape CAD
     │ export
     ▼
 urdf/assembly_1.urdf ──► fix_kinematics.py ──► urdf/assembly_1_fixed.urdf ──► RViz viewer
 (raw, broken)                 │                 (correct joints, collisions)
                               ├─► config/belt_animator.yaml
                               └─► config/bucket_controller.yaml
                                                    │
                                                    ▼
                                         add_ros2_control.py
                                                    │
                                                    ▼
                                      urdf/assembly_1_sim.urdf ──► Gazebo
                                      (+ ros2_control hardware)
```

At runtime:

```
 keyboard ─► teleop_twist_keyboard ─► /cmd_vel ─► diff_drive_controller ─┐
                                                                         │
 ros2 topic pub ─► /belt_velocity_controller/commands ─────────────────┐ │
 ros2 topic pub ─► /excavator_lift_controller/commands ──────────────┐ │ │
                                                                     ▼ ▼ ▼
                                                     controller_manager (inside Gazebo)
                                                                     │
                                                                     ▼
                                                          joints in the physics engine
                                                                     │
                                                                     ▼
                                     joint_state_broadcaster ─► /joint_states
                                                                     │
               ┌─────────────────────────────────────────────────────┤
               ▼                                                     ▼
       belt_animator.py                                   robot_state_publisher
   (roller angle → bucket positions)                       (joint angles → TF)
               │                                                     │
               ▼                                                     ▼
   /bucket_position_controller/commands                         RViz display
```

You don't need to understand all of that yet — each piece is explained below.

---

## 2. ROS 2 in 10 minutes

ROS 2 (Robot Operating System 2) is not an operating system. It is a set of
libraries and tools that let many small programs work together as one robot
system. This project uses the **Lyrical** release.

### Nodes

A **node** is one running program with one job. In this project:

| Node | Job |
|---|---|
| `robot_state_publisher` | Turns joint angles into 3D positions of every part (TF) |
| `belt_animator` | Moves the buckets round the belt |
| `controller_manager` | Runs the controllers (lives inside Gazebo) |
| `teleop_twist_keyboard` | Turns key presses into velocity commands |
| `rviz2` | 3D visualisation |
| `parameter_bridge` | Copies the simulation clock from Gazebo into ROS |

List the running nodes with `ros2 node list`.

### Topics and messages

Nodes talk by **publishing messages on topics**. A topic is a named channel
(like `/cmd_vel`); a message is a typed piece of data (like
`geometry_msgs/msg/TwistStamped`, which holds a linear and an angular
velocity). Any number of nodes can publish or subscribe to a topic; they
don't need to know about each other.

Topics used here:

| Topic | Message type | From → To |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | teleop → diff_drive_controller |
| `/joint_states` | `sensor_msgs/JointState` | joint_state_broadcaster → everyone |
| `/belt_velocity_controller/commands` | `std_msgs/Float64MultiArray` | you → belt controller |
| `/excavator_lift_controller/commands` | `std_msgs/Float64MultiArray` | you → lift controller |
| `/bucket_position_controller/commands` | `std_msgs/Float64MultiArray` | belt_animator → bucket controller |
| `/bucket_joint_states` | `sensor_msgs/JointState` | belt_animator → joint_state_publisher (RViz mode) |
| `/diff_drive_controller/odom` | `nav_msgs/Odometry` | diff_drive_controller → anyone |
| `/robot_description` | `std_msgs/String` | robot_state_publisher → Gazebo spawner, RViz |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | robot_state_publisher, diff_drive_controller → RViz |
| `/clock` | `rosgraph_msgs/Clock` | parameter_bridge → everyone using sim time |

Try: `ros2 topic list`, `ros2 topic echo /joint_states`, `ros2 topic info /cmd_vel`.

### Parameters

Nodes have **parameters** — settings you can pass in at start-up, usually
from a YAML file. Example: `belt_animator` reads `shaft_distance`,
`pitch_radius` and `start_positions` from `config/belt_animator.yaml`.
The format is:

```yaml
node_name:
  ros__parameters:
    some_setting: 1.5
```

Try: `ros2 param list /belt_animator`, `ros2 param get /belt_animator pitch_radius`.

### Launch files

Starting a dozen nodes by hand would be painful. A **launch file**
(`launch/*.launch.py`) is a Python script that describes which nodes to
start, with which parameters, in which order. `ros2 launch lunabotics_model
gazebo.launch.py` runs it.

Launch files can take **arguments**: `gpu:=true` is one. See them with
`ros2 launch lunabotics_model gazebo.launch.py --show-args`.

### Packages, workspaces and colcon

- A **package** is a folder with a `package.xml` (name, dependencies) and a
  build file (`CMakeLists.txt`). This repo has one: `src/lunabotics_model`.
- A **workspace** is a folder with a `src/` of packages. This repo is one.
- **colcon** builds the workspace. `colcon build` copies (installs) the
  package's files into `install/`, which is where ROS actually runs them
  from. That's why you must **rebuild after every change** to launch files,
  configs, URDFs or scripts.
- `source install/setup.bash` tells your terminal where the installed
  package is. Every new terminal needs it (after `source
  /opt/ros/lyrical/setup.bash`).

`build/`, `install/` and `log/` are generated by colcon and are ignored by git.

### TF (transforms)

A robot is a tree of coordinate frames: one per part. **TF** is the ROS
system that tracks where every frame is relative to every other, over time.
`robot_state_publisher` reads the URDF and the current joint angles from
`/joint_states`, and publishes a transform for every part. RViz uses TF to
draw the robot. `diff_drive_controller` adds one more: `odom → root`, the
robot's estimated position in the world.

Try: `ros2 run tf2_ros tf2_echo odom root`.

### Sim time

Gazebo can run slower or faster than real time. Nodes that care about time
should use Gazebo's clock, not the wall clock. They do this with the
parameter `use_sim_time: true` and by listening to `/clock`. All controllers
run on sim time automatically. This matters for teleop — see
[Gotchas](#13-gotchas-we-hit-and-why).

---

## 3. Describing a robot: URDF

**URDF** (Unified Robot Description Format) is an XML file that describes a
robot as **links** (rigid parts) connected by **joints**.

### Links

```xml
<link name="wheel">
  <inertial>   <!-- mass and inertia: how it moves under force -->
    <mass value="9.12"/>
    <origin xyz="..." rpy="..."/>
    <inertia ixx="..." .../>
  </inertial>
  <visual>     <!-- what it looks like -->
    <origin xyz="..." rpy="..."/>
    <geometry><mesh filename="package://lunabotics_model/meshes/wheel.stl"/></geometry>
  </visual>
  <collision>  <!-- what shape physics uses for contact -->
    <origin xyz="..." rpy="..."/>
    <geometry><cylinder radius="0.1805" length="0.1"/></geometry>
  </collision>
</link>
```

- **visual**: purely for display. Can be a detailed mesh.
- **collision**: what the physics engine uses. Should be simple (boxes,
  cylinders, spheres) because collision checking with detailed meshes is
  slow and unreliable.
- **inertial**: mass and how that mass is distributed. Without it, Gazebo
  ignores the link.

### Joints

```xml
<joint name="revolute_1" type="continuous">
  <parent link="57blr70_24_02_hg50_p6_3"/>
  <child link="flange_3"/>
  <origin xyz="0.03 0.09 0" rpy="0 0 0"/>   <!-- child frame, relative to parent -->
  <axis xyz="0 -1 0"/>                       <!-- axis of motion, in the child frame -->
</joint>
```

| Type | Motion | Used for here |
|---|---|---|
| `fixed` | none | bolted-together parts |
| `continuous` | unlimited rotation | wheels, rollers, bucket rotation |
| `revolute` | rotation with limits | — |
| `prismatic` | sliding along an axis | excavator lift, bucket track |

**Every URDF must be a tree**: each link has exactly one parent (except the
root). This is why closed loops (like a belt, or a four-bar linkage) can't be
described directly.

### Origins and frames — the most important idea

Every link has its own coordinate frame. A joint's `<origin>` says where the
child's frame is, relative to the parent's frame: `xyz` is a translation in
metres, `rpy` is roll-pitch-yaw rotation in radians. The child frame *is* the
joint frame, and the `<axis>` is expressed in it.

Internally each origin is a 4×4 matrix `T`. To get a part's position in the
world, you multiply down the tree from the root:

```
W[child] = W[parent] @ T_joint @ (joint motion)
```

This one formula is the heart of `fix_kinematics.py` (section 7).

### `package://` paths

`package://lunabotics_model/meshes/wheel.stl` means "the `meshes/wheel.stl`
file inside the installed `lunabotics_model` package". ROS tools resolve it
via `source install/setup.bash`; Gazebo resolves it via the
`GZ_SIM_RESOURCE_PATH` environment variable (set in the launch file).

---

## 4. Gazebo

**Gazebo** (the modern one, formerly "Ignition"; command `gz sim`) is a
physics simulator. It computes gravity, contacts, friction and joint motion
many times per second (here every 1 ms) and renders the result.

### Worlds and models

- A **world** is the environment: ground plane, lighting, gravity, physics
  settings. We use Gazebo's built-in `empty.sdf`.
- A **model** is an object in the world. Our robot is spawned as the model
  `lunabotics` by `ros_gz_sim create`, which reads `/robot_description`.

### SDF vs URDF

Gazebo's native format is **SDF**, not URDF. When the robot is spawned,
Gazebo converts URDF → SDF. Two things happen during conversion that matter:

1. **Fixed joints are merged ("lumped").** All parts connected by fixed
   joints become one rigid body. Our 60+ CAD parts become ~38 bodies. This
   is good for speed, but it means you can't move a part that's fixed to
   another — that's why the buckets need real joints (section 8).
2. **`<gazebo>` tags in the URDF become SDF plugins.** That's how the
   ros2_control plugin gets loaded.

You can see the converted model with:

```bash
gz sdf -p install/lunabotics_model/share/lunabotics_model/urdf/assembly_1_sim.urdf
```

### Physics engine

Gazebo can use different physics engines. The default, **DART**, is used
here. Physics runs on the **CPU only**. The GPU only renders the window —
that's what `gpu:=true` is for.

### Server and GUI

`gz sim` runs two processes: the **server** (`gz-sim-main`, does physics)
and the **GUI** (`gz-sim-gui-client`, draws the window). `gz sim -s` runs
only the server (headless), which is useful for automated tests.

### Talking to Gazebo from the terminal

```bash
gz topic -l                          # Gazebo's own topics (separate from ROS topics)
gz model -m lunabotics -p            # robot's position and orientation
gz topic -e -t /stats -n 1           # real-time factor (sim speed vs real time)
```

Gazebo and ROS have **separate** messaging systems. `ros_gz_bridge`
(`parameter_bridge`) copies specific topics between them; we only bridge
`/clock`. Everything else crosses over through ros2_control.

---

## 5. ros2_control

**ros2_control** is the standard ROS framework for controlling joints. It
separates *what* a joint can do from *how* it is controlled, so the same
controllers work on a real robot and in simulation.

### What is a controller?

A **controller** is a small program that turns a command *you* understand
into what the *joints* need, and keeps applying it every cycle.

You think "drive forward at 0.5 m/s". The motors only understand "spin this
joint at 2.77 rad/s". The controller translates between the two. For
example, when you press `i` in teleop, `diff_drive_controller`:

1. works out the wheel speeds: 0.5 m/s ÷ 0.1805 m wheel radius = 2.77 rad/s
2. sends 2.77 rad/s to all four wheel joints
3. repeats that 100 times a second until a new command arrives
4. reads back how far the wheels actually turned, and estimates where the
   robot is (odometry)

Controllers come in three kinds:

- **Smart** ones do real maths, like `diff_drive_controller` turning a speed
  and turn rate into four wheel speeds.
- **Pass-through** ones (`ForwardCommandController`) forward your number
  straight to a joint, like `belt_velocity_controller`.
- **Read-only** ones never move anything. `joint_state_broadcaster` just
  reads every joint and publishes `/joint_states`.

Controllers don't talk to motors directly. They talk to a **hardware
interface** underneath: in simulation `GazeboSimSystem`, which moves
Gazebo's joints. On the real robot you would swap only that layer for your
motor drivers, and the same controllers and teleop commands would work
unchanged.

### The pieces

```
  ROS topics                  controllers                    hardware
 ─────────────      ──────────────────────────────      ─────────────────────
 /cmd_vel ────────► diff_drive_controller ──┐
 /belt_.../commands ► belt_velocity_controller ─┤ command   ┌──────────────────┐
 /excavator_.../...► excavator_lift_controller ─┼─interfaces─►│ GazeboSimSystem  │──► Gazebo joints
 /bucket_.../... ──► bucket_position_controller ┘          │ (gz_ros2_control)│
                                                           └──────────────────┘
 /joint_states ◄──── joint_state_broadcaster ◄─── state interfaces ──┘
```

- **Hardware interface**: the bottom layer that actually moves joints. On a
  real robot this would talk to motor drivers. In simulation it's
  `gz_ros2_control/GazeboSimSystem`, which moves Gazebo's joints.
- **Command interfaces**: what a controller can set on a joint —
  `velocity` or `position` here.
- **State interfaces**: what a controller can read — `position` and
  `velocity` here.
- **Controllers**: plugins that turn ROS messages into joint commands.
- **controller_manager**: loads, configures and runs the controllers in a
  loop (100 Hz here). In Gazebo it runs inside the Gazebo process.
- **spawner**: a small program that asks controller_manager to load and
  activate a controller.

### Where each piece is declared

| What | File |
|---|---|
| Which joints have which interfaces | `<ros2_control>` blocks in `urdf/assembly_1_sim.urdf` |
| The Gazebo plugin that runs it all | `<gazebo><plugin ...GazeboSimROS2ControlPlugin>` in the same URDF |
| Which controllers exist and their settings | `config/controllers.yaml`, `config/bucket_controller.yaml` |
| Starting the controllers | spawner nodes in `launch/gazebo.launch.py` |

### Our controllers

| Controller | Type | Joints | You send |
|---|---|---|---|
| `diff_drive_controller` | `diff_drive_controller/DiffDriveController` | 4 wheels | linear + angular velocity on `/cmd_vel` |
| `belt_velocity_controller` | `forward_command_controller/ForwardCommandController` | `drive_roller_joint` | rad/s |
| `excavator_lift_controller` | `ForwardCommandController` | `excavator_lift` | position in m |
| `bucket_position_controller` | `ForwardCommandController` | 30 bucket joints | (sent by belt_animator) |
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | all | nothing — it publishes `/joint_states` |

**ForwardCommandController** passes your numbers straight to the joints and
keeps them until you send new ones.

**DiffDriveController** does the maths for a wheeled robot. Given forward
speed `v` and turn rate `w`:

```
left wheel  = (v − w · separation / 2) / wheel_radius
right wheel = (v + w · separation / 2) / wheel_radius
```

It also integrates wheel motion into an odometry estimate
(`/diff_drive_controller/odom`). Because a four-wheel robot **skids**
sideways when turning, `wheel_separation_multiplier: 2.1` makes it act as if
the wheels were further apart, so turns come out close to what you asked for.
Odometry rotation is still only approximate (~20% off).

---

## 6. The repo, file by file

```
lunabotics_sim/
├── README.md                     how to install, build, run, control
├── docs/GUIDE.md                 this file
├── old_urdf/                     first Onshape export (chassis only, ROS 1) — unused
└── src/lunabotics_model/         the ROS 2 package
    ├── package.xml               package name and dependencies
    ├── CMakeLists.txt            what gets installed by colcon
    ├── meshes/*.stl              3D shapes of every CAD part
    ├── urdf/
    │   ├── assembly_1.urdf       raw Onshape export (input — don't edit)
    │   ├── assembly_1_fixed.urdf GENERATED: repaired model (RViz uses this)
    │   └── assembly_1_sim.urdf   GENERATED: + ros2_control (Gazebo uses this)
    ├── config/
    │   ├── controllers.yaml          controller types and settings (hand-written)
    │   ├── belt_animator.yaml        GENERATED: belt geometry, bucket start positions
    │   └── bucket_controller.yaml    GENERATED: the 30 bucket joint names
    ├── scripts/
    │   ├── fix_kinematics.py     repairs the raw export (section 7, 8)
    │   ├── add_ros2_control.py   adds hardware blocks (section 9)
    │   └── belt_animator.py      runs the buckets round the belt (section 8)
    ├── launch/
    │   ├── gazebo.launch.py      full simulation
    │   └── display.launch.py     RViz-only viewer
    └── rviz/                     (empty)
```

**GENERATED** files are rewritten by the scripts. Never edit them by hand —
change the script and re-run it.

---

## 7. fix_kinematics.py: repairing the CAD export

### What Onshape got wrong

| Problem | Effect |
|---|---|
| All 4 wheels fastened to one flange (`flange_1`) | Only one wheel joint did anything, and it spun all four wheels around one axle |
| Wheel axes point opposite ways on each side | The same command drove one side forward and the other backward |
| The chassis frame wasn't attached to `root`; it hung off the excavator belt | Spinning the "belt" joint spun the whole chassis around the excavator |
| Closed loops exported as massless `*_loop_closure` duplicate parts | Junk links nothing can use |
| `<mimic>` tags coupling the wheels | Conflict with independent control |
| Buckets fixed rigidly to the belt | Gazebo merges them, so they can never move |
| No `<collision>` anywhere | The robot falls through the floor |

### How it fixes them

**Step 1 — measure the CAD.** `world_poses()` walks the original tree and
computes every part's world position `W[part]` (section 3). Everything else
is computed from these, so no part moves.

**Step 2 — remove the bad connections.** `DROP_JOINTS` lists the wrong
joints; loop-closure dummies and `<mimic>` tags are deleted.

**Step 3 — add the right ones.** `new_joints()` adds:

| New joint | Type | Connects |
|---|---|---|
| `chassis_mount` | fixed | root → chassis frame |
| `wheel*_mount` | fixed | each wheel → the flange nearest to it |
| `belt_mount`, `ladder_bottom_shaft` | fixed | belt and bottom shaft → top shaft (`rod`) |
| `drive_roller_joint` | continuous | top shaft → top roller |
| `idler_roller_joint` | continuous | bottom shaft → bottom roller |
| `slider_bracket_1_mount` | fixed | second slider bracket → frame |

`slider_1` is renamed `excavator_lift`.

**Step 4 — rebuild the tree.** The script now has a set of connections but
not which way each points. It does a **breadth-first search** from `root`:
every part it reaches becomes the child of the part it came from. For each
joint it recomputes the origin as

```python
origin = inv(W[parent]) @ W[child]
```

which places the child exactly where the CAD had it, whatever its new
parent is. Moving joints that got reversed (the lift) also have their axis
re-expressed in the new child frame, and the lift's direction is chosen so
**+ always means up**.

**Step 5 — check.** It recomputes all world positions from the new tree and
refuses to write the file if anything moved more than 1 µm.

**Step 6 — the belt** (section 8).

**Step 7 — collisions.** `add_collisions()` reads each STL mesh and adds a
simple collision shape, chosen by the `COLLISION_SHAPES` table:

| Part | Shape |
|---|---|
| Frame extrusions, brackets, motors, electronics | box fitted to the mesh |
| Wheels, rollers, shafts | cylinder |
| Buckets, regolith bin | the mesh itself (they're low-poly) |
| Belt band | none |

### The result

```
root
 ├─ motors ─► revolute_1* (continuous) ─► flange ─► wheel           (×4)
 └─ chassis frame ... slider_bracket
      └─ excavator_lift (prismatic) ─► ladder (side plates, shafts, belt)
           ├─ drive_roller_joint ─► top roller
           ├─ idler_roller_joint ─► bottom roller
           └─ <bucket>_belt_u ─► _belt_v ─► _belt_rot ─► bucket      (×10)
```

Tunable values (lift travel, efforts, speeds) are constants at the top of
the script.

---

## 8. The belt: buckets on joints

A conveyor belt is a closed loop, which a URDF tree can't express. So each
bucket gets three joints that together can place it anywhere on the loop:

```
ladder ─► bucket_N_belt_u   (prismatic: slide along the ladder)
         ─► bucket_N_belt_v   (prismatic: slide across it)
              ─► bucket_N_belt_rot (continuous: rotate about the shafts) ─► bucket
```

The two links between them (`*_belt_u_link`, `*_belt_v_link`) are invisible
and tiny; they only exist to chain the joints.

### The loop shape

Measured from the CAD: shafts **0.650 m** apart, buckets mounted
**0.0485 m** from the shaft line. The loop is two straights joined by two
half-circles (a racetrack), total length `2 × 0.65 + 2π × 0.0485 = 1.605 m`.

Positions are measured in a **ladder frame**: origin on the bottom shaft,
`u` toward the top shaft, `w` along the shafts, `v` across the ladder.

`loop_pose(s)` (in both `fix_kinematics.py` and `belt_animator.py`) turns a
distance `s` along the belt into `(u, v, θ)`:

| `s` | Segment | Position | Bucket angle θ |
|---|---|---|---|
| 0 → 0.650 | up the +v side | `(s, +r)` | 0 |
| → 0.802 | round the top shaft | arc | 0 → −π |
| → 1.452 | down the −v side | `(L − distance, −r)` | −π |
| → 1.605 | round the bottom shaft | arc | −π → −2π |

θ keeps counting down lap after lap (−2π, −4π, …) instead of jumping back to
0, so the rotation joint never gets a sudden full-turn command.

### Where each bucket starts

`fix_kinematics.py` measures each bucket's CAD position, works out its `s`
on the loop, and writes all ten to `config/belt_animator.yaml`
(`start_positions`). It also re-frames each bucket so that, at those joint
values, the bucket sits exactly where the CAD had it.

### belt_animator.py

A small node that runs 50 times a second:

1. Read the drive roller angle `θ_roller` from `/joint_states`.
2. For each bucket: `s = start + belt_direction × radius × θ_roller`.
3. `(u, v, θ) = loop_pose(s)`.
4. Publish all 30 values to `/bucket_position_controller/commands` (Gazebo)
   and `/bucket_joint_states` (RViz viewer).

Because it follows the roller's *measured* angle, the buckets always stay in
step with the roller, whatever is driving it. In Gazebo they lag ~10 mm
behind while moving and settle exactly when the belt stops.

---

## 9. add_ros2_control.py

Takes `assembly_1_fixed.urdf` and appends:

- **`lunabotics_wheel_drive`**: the 4 wheel joints, velocity command.
- **`lunabotics_excavator`**: `drive_roller_joint` (velocity),
  `excavator_lift` (position), `idler_roller_joint` (state only — it just
  spins freely).
- **`lunabotics_belt`**: the 30 bucket joints, position command, each with
  an `initial_value` so Gazebo spawns the buckets on the belt rather than
  piled at zero.
- **The `<gazebo>` plugin** `gz_ros2_control::GazeboSimROS2ControlPlugin`,
  pointing at `controllers.yaml`.

Output: `assembly_1_sim.urdf`.

---

## 10. What happens when you launch

`ros2 launch lunabotics_model gazebo.launch.py gpu:=true`, step by step:

| Time | What happens |
|---|---|
| 0 s | If `gpu:=true`: set the three NVIDIA PRIME environment variables, so the Gazebo window renders on the NVIDIA card |
| 0 s | Add the package's install folder to `GZ_SIM_RESOURCE_PATH` so Gazebo finds the meshes |
| 0 s | Read `assembly_1_sim.urdf` and replace `$(find lunabotics_model)` with the real path (nothing else would expand it) |
| 0 s | Start Gazebo with `empty.sdf` |
| 0 s | Start `robot_state_publisher` with the URDF → publishes `/robot_description` and TF |
| 0 s | Start the `/clock` bridge so ROS nodes can use sim time |
| 0 s | `ros_gz_sim create` reads `/robot_description` and spawns the robot at z = 0.32 m (wheels are 0.308 m below its origin) |
| ~1 s | Gazebo converts URDF→SDF, merges fixed joints, loads the ros2_control plugin, which starts `controller_manager` and the 3 hardware groups |
| 5 s | Spawner 1 activates `joint_state_broadcaster`, `diff_drive_controller`, `belt_velocity_controller`, `excavator_lift_controller` (passing `controllers.yaml`, and remapping the drive controller's input to `/cmd_vel`) |
| 5 s | Spawner 2 activates `bucket_position_controller` (passing `bucket_controller.yaml`) |
| 7 s | Start `belt_animator` |
| 0 s | Start RViz |

The timers give Gazebo time to load before the spawners ask for the
controller manager.

`display.launch.py` is much simpler: `robot_state_publisher` +
`joint_state_publisher_gui` (sliders for every joint) + `belt_animator` +
RViz. No physics. The slider GUI takes the bucket joint values from
`belt_animator` (its `source_list` parameter), so dragging the
`drive_roller_joint` slider runs the belt.

---

## 11. What happens when you press a key

You press `i` in the teleop terminal:

1. `teleop_twist_keyboard` publishes **one** `TwistStamped` on `/cmd_vel`:
   `linear.x = 0.5`, stamped with sim time (because of `use_sim_time:=true`).
2. `diff_drive_controller` receives it (we remapped its input topic to
   `/cmd_vel`), computes left and right wheel speeds
   (`0.5 / 0.1805 = 2.77 rad/s`), and writes them to the wheels' velocity
   command interfaces.
3. `GazeboSimSystem` sets those velocities on the joints in Gazebo.
4. The physics engine turns the wheels; friction with the ground moves the
   robot.
5. Every cycle, `GazeboSimSystem` reads joint positions/velocities back;
   `joint_state_broadcaster` publishes them on `/joint_states`;
   `diff_drive_controller` updates odometry and the `odom → root` transform.
6. The robot keeps moving with the last command until you press `k`
   (`cmd_vel_timeout` is set very long, because teleop sends only one
   message per key press).

For the belt (`ros2 topic pub ... /belt_velocity_controller/commands`): the
controller sets the drive roller's velocity → the roller turns →
`/joint_states` reports its angle → `belt_animator` computes bucket positions
→ `bucket_position_controller` moves the 30 bucket joints.

---

## 12. Debugging toolbox

| Question | Command |
|---|---|
| Which nodes are running? | `ros2 node list` |
| Which topics exist? | `ros2 topic list` |
| What's on a topic? | `ros2 topic echo /joint_states` |
| Who publishes / subscribes? | `ros2 topic info /cmd_vel` |
| How fast is it published? | `ros2 topic hz /joint_states` |
| Are the controllers running? | `ros2 control list_controllers` |
| Which joint interfaces exist? | `ros2 control list_hardware_interfaces` |
| A node's parameters | `ros2 param list /diff_drive_controller` |
| Where is the robot (estimate)? | `ros2 run tf2_ros tf2_echo odom root` |
| Where is the robot (truth)? | `gz model -m lunabotics -p` |
| Is the sim running at full speed? | `gz topic -e -t /stats -n 1` → `real_time_factor` |
| Is Gazebo using the GPU? | `nvidia-smi` → look for `gz-sim-gui-client` |
| What does Gazebo see? | `gz sdf -p <urdf file>` |
| Send a one-off command reliably | `ros2 topic pub -w 1 --times 3 <topic> <type> "<data>"` |

---

## 13. Gotchas we hit (and why)

**Gazebo segfaulted on start-up in `libgz_ros2_control-system.so`.**
`gz_ros2_control` had been built against a newer `ros2_control` than the one
installed. Fix: `sudo apt install --only-upgrade 'ros-lyrical-*'`.

**Controllers failed with "parameter 'joints' is not initialized".**
Recent `controller_manager` versions don't pass the plugin's
`controllers.yaml` to each controller, so the spawner passes it with
`--param-file`.

**`$(find lunabotics_model)` wasn't expanded.** That syntax belongs to xacro,
which we don't use. The launch file substitutes it.

**The robot didn't stop / didn't move with teleop.** `diff_drive_controller`
checks how old each command is using the message timestamp, on sim time.
Without `use_sim_time:=true`, teleop stamps messages with wall-clock time,
which looks like the far future to the controller. Always run teleop with
`-p stamped:=true -p use_sim_time:=true`.

**`ros2 topic pub --once` sometimes does nothing.** It can exit before ROS
discovery connects it to the subscriber, so the message is lost. Use
`-w 1 --times 3` (wait for a subscriber, send three times).

**velocity_controllers / position_controllers don't exist.** They were
removed from ros2_controllers in Lyrical; `ForwardCommandController` with
`interface_name` replaces them.

**The Gazebo window was slow / `libEGL warning ... driver (null)`.** The
laptop has an AMD iGPU and an NVIDIA MX350 in PRIME on-demand mode; apps use
the AMD chip unless told otherwise. `gpu:=true` tells Gazebo to use NVIDIA.

**Particles for digging were too slow.** A test with a pile of a few hundred
particles slowed DART to under 1% real time, and spheres rolled away (Gazebo
has no rolling resistance). Digging simulation is not part of the repo yet.

---

## 14. Glossary

| Term | Meaning |
|---|---|
| **colcon** | ROS 2 build tool |
| **controller** | ros2_control plugin that turns commands into joint actions |
| **controller_manager** | runs controllers in a loop |
| **DART** | Gazebo's default physics engine |
| **frame** | a coordinate system attached to a part |
| **Gazebo / gz sim** | the physics simulator |
| **hardware interface** | ros2_control's link to real or simulated joints |
| **launch file** | Python script that starts a set of nodes |
| **link** | a rigid part in a URDF |
| **joint** | a connection between two links (fixed or moving) |
| **lumping** | Gazebo merging links joined by fixed joints |
| **node** | one ROS program |
| **odometry** | position estimate from wheel motion |
| **package** | a unit of ROS code with `package.xml` |
| **parameter** | a node setting, usually from YAML |
| **PRIME offload** | running an app on the NVIDIA GPU of a hybrid laptop |
| **real-time factor (RTF)** | sim speed ÷ real speed (1.0 = real time) |
| **RViz** | 3D visualisation of ROS data |
| **SDF** | Gazebo's native model format |
| **sim time** | the simulation's clock, published on `/clock` |
| **skid steer** | turning by driving the two sides at different speeds, with sideways wheel slip |
| **spawner** | program that loads and activates a controller |
| **teleop** | driving a robot remotely (here, by keyboard) |
| **TF** | ROS system tracking all coordinate frames over time |
| **topic** | named channel for messages |
| **URDF** | XML robot description: links + joints |
| **workspace** | folder of packages built together |
| **xacro** | URDF macro language (not used here) |
