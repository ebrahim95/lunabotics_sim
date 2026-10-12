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
13. [The MuJoCo digging test](#13-the-mujoco-digging-test)
14. [How realistic is this simulation?](#14-how-realistic-is-this-simulation)
15. [Gotchas we hit (and why)](#15-gotchas-we-hit-and-why)
16. [The robot's parts](#16-the-robots-parts)
17. [Glossary](#17-glossary)

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

Separately from ROS, `mujoco_dig/dig_test.py` uses the **MuJoCo** physics
engine to test how well the buckets dig regolith (section 13).

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
draw the robot. `diff_drive_controller` adds one more: `odom → base_link`, the
robot's estimated position in the world.

Try: `ros2 run tf2_ros tf2_echo odom base_link`.

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

### The Artemis Arena world (BP-1)

`world:=artemis_arena` loads a model of the competition arena at Kennedy
Space Center, generated by `scripts/generate_arena_world.py` from the
Lunabotics guidebook layout:

| Feature | In the world |
|---|---|
| Arena | 6.88 × 5 m, walls around it; origin at the starting-zone corner |
| Starting zone | 2 × 2 m corner (green outline); no rocks or craters; the robot spawns at (1, 1) |
| Obstacle zone | first 3.88 m: rocks 30–40 cm, craters ≤ 45 cm wide and ~20 cm deep |
| Excavation zone | far 3 × 3 m (blue outline), with a crater and rocks |
| Construction zone | far 3 × 2 m (orange); berm target 2.2 × 0.9 m (red) |
| Column | ~0.45 m square near the middle |

**BP-1** (Black Point-1) is crushed basalt from Arizona used as a lunar
regolith simulant: rough angular grains, grain specific gravity 2.81, bulk
density 1.27–1.75 g/cm³, internal friction angle 39–51°, almost no
cohesion; the arena holds ~45 cm of it. Gazebo can't simulate it as a
granular soil (that's what the MuJoCo test is for), so the floor is a
rigid terrain mesh with BP-1's properties as a surface: friction 0.8
(≈ tan 39°), dark basalt colour, ~5 mm gravel roughness, and crater
depressions. Wheels don't sink into it the way they would in real BP-1, but
they do slip (see "Wheel slip" below).

### What is wheel slip?

**Wheel slip** is when a wheel turns faster than the ground moving past
it: some of the spin goes into scuffing the ground instead of moving the
robot. At full speed the wheel surface moves at 1.0 m/s (5.54 rad/s ×
0.1805 m radius). With no slip the robot moves at 1.0 m/s; with 20% slip it
moves at 0.8 m/s, and the other 0.2 m/s is the tyre sliding over the ground.

```
slip = 1 − (ground speed ÷ wheel surface speed)
```

0% is perfect grip; 100% is spinning in place.

**Why it happens.** A wheel pushes the robot forward by pushing back on the
ground. Concrete holds firm, so there's little slip. In loose sand like BP-1
the grains shear and get thrown backwards, so the wheel has to turn further
to go the same distance — and the harder it pushes (accelerating, climbing
out of a crater, carrying a load), the more it slips.

**Turning.** A skid-steer robot turns by spinning one side faster than the
other, which forces the wheels to slide sideways — sideways slip. Without
it, a high-friction floor would lock the wheels in place.

**Why it matters on the real robot:** odometry drifts (the encoders count
wheel turns the robot didn't travel), energy is wasted spinning wheels, and
slip near 100% means the robot is digging itself in.

### Wheel slip

`add_ros2_control.py` adds Gazebo's **WheelSlip** system to the four drive
wheels. It lets a wheel turn faster than the ground passes under it when
it pushes hard, as on loose sand. Roughly:

```
slip ≈ compliance × (traction force ÷ wheel load)
```

The compliance is **0.6** forward and sideways — an **estimate** for loose
sand (~20% slip while pushing at ~0.3× the wheel load); there's no measured
value for these wheels on BP-1. The wheel load is 50 kg × 9.81 ÷ 4 = 123 N.
In the arena this gives:

| Situation | Slip |
|---|---|
| Starting from rest at full speed | ~80% at first, falling to ~4% after 1 s |
| Cruising at 1 m/s | ~2.5% |
| Spinning in place | the wheels scrub sideways, so the robot can pivot |

Without slip, the 20 N·m motors couldn't drag the wheels sideways on the
high-friction floor and the robot barely turned on the spot. WheelSlip
needs each wheel link to have exactly one collision, which is why the
flange (hidden inside the hub) has none. Change the values with
`SLIP_COMPLIANCE_LONGITUDINAL` / `SLIP_COMPLIANCE_LATERAL` at the top of
`add_ros2_control.py`.

The default layout is the guidebook drawing's example; `--seed N` places
rocks and craters randomly in their zones, like the judges do each run.

### Soft soil: Bekker–Wong terramechanics (`soil:=bekker`)

`soil:=rigid` (the default) drives on a hard floor with WheelSlip.
`soil:=bekker` makes the wheels sink into the BP-1 instead. The wheels stop
colliding with the arena floor (collision masks: wheels `0x01`, floor
`0x02`; rocks and the chassis still collide normally), and a Gazebo
**Python system**, `scripts/bekker_terramechanics.py`, applies soil forces
to each wheel every physics step:

| Force | Model | Meaning |
|---|---|---|
| Support | Bekker pressure–sinkage, W = (3−n)/3 · (k_c + b·k_φ) · √D · z^((2n+1)/2) | the deeper the wheel sinks, the harder the soil pushes back |
| Traction | Janosi–Hanamoto shear, H = (A·c + W·tan φ)(1 − K/(sL)(1 − e^(−sL/K))) | the more the wheel slips (s), the more it can push |
| Rolling resistance | compaction, R = b(k_c/b + k_φ) z^(n+1)/(n+1) | the wheel keeps climbing out of its own rut |
| Sideways | same shear curve with sideways slip | skid-steer wheels plough sideways when turning |
| Torque | shear (traction, sideways) acts at the contact patch; soil pressure (support, rolling resistance) acts on the rim towards the axle | the motors must supply traction × radius, ~4.3 N·m per wheel when cruising |

(z sinkage, b wheel width, D wheel diameter, L = √(Dz) contact length,
A = bL contact area.) The soil surface comes from the arena's height grid
(`worlds/artemis_arena_height.npz`), so craters work; the wheel geometry
from `config/terramechanics.yaml` (written by `add_ros2_control.py`).

**Soil constants:** φ = 39° and near-zero cohesion are BP-1's; the Bekker
constants k_c = 1400, k_φ = 820 000, n = 1 and K = 18 mm are standard
**lunar regolith** values — no BP-1-specific ones were found.

**What it does (arena tests, 50 kg robot):**

| | rigid + WheelSlip | bekker |
|---|---|---|
| Sinkage | 0 | 24 mm |
| Slip while cruising | ~2.5% | ~12% |
| Forward 1.2 m commanded | ~1.2 m | 1.06 m |
| Turn 2.0 rad commanded | ~1.95 rad | 1.30 rad |
| Sim speed | ~0.66× | ~0.42× (the Python plugin) |

**Driving it** (teleop or autonomy): the robot sits ~24 mm down in the
BP-1, so in the Gazebo window its wheels look slightly sunk below the floor
— that's the sinkage, not a display bug. It accelerates more slowly than
on the rigid floor, cruises ~12% slower than commanded (and odometry
overestimates distance by about that much), and turns about two-thirds as
far as commanded. The 20 N·m wheel motors work harder: cruising takes
~4 N·m per wheel just to overcome the rolling resistance of the rut.

Set `LUNA_TERRA_DEBUG=1` to print sinkage, slip and wheel forces every 2 s.
The model treats each wheel as a smooth cylinder (no grousers), doesn't
leave ruts in the terrain, and isn't for the buckets (they still hit the
rigid floor). On the empty world the soil is flat at z = 0 but the wheels
still touch the default ground plane, so use it in the arena.

### Physics engine

Gazebo can use different physics engines. The default, **DART**, is used
here. Physics runs on the **CPU only**. The GPU only renders the window —
that's what `gpu:=true` is for.

### Simulation speed (real-time factor)

The **real-time factor (RTF)** is simulated time ÷ real time: 1.0 means one
simulated second takes one real second; 0.4 means it takes 2.5 s. Physics
runs on the CPU only (one core per simulation), so a faster GPU only makes
the window smoother. Measured on this laptop:

| Simulation | RTF |
|---|---|
| Gazebo, empty world | ~0.8 |
| Gazebo, Artemis Arena, `soil:=rigid` | ~0.66 |
| Gazebo, Artemis Arena, `soil:=bekker` | ~0.4 (the Python soil plugin) |
| MuJoCo digging test, 484 particles, no window | ~0.15–0.2 |
| MuJoCo digging test with the viewer | ~0.12–0.14 |

What costs the most: the 1 ms physics step, the 30 bucket joints held in
position, the arena's terrain mesh, the soil plugin running in Python, and
(in MuJoCo) every particle-to-particle contact. Controllers and odometry
run on sim time, so everything still behaves correctly — just slower.

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
Odometry rotation is still only approximate (see below).

### What is odometry?

**Odometry** is the robot working out where it is by counting how much its
wheels have turned — like walking with your eyes closed and counting steps.
`diff_drive_controller` does it from the wheel encoders:

- **Distance:** wheel turns × circumference; one turn = 2π × 0.1805 m ≈ 1.13 m.
- **Forward movement:** the average of the left and right wheels.
- **Turning:** the difference between them — if the right side travels
  further than the left, the robot turned left.

It adds up these small movements 100 times a second into a running
estimate of position (x, y) and heading, published on
`/diff_drive_controller/odom` and as the `odom → base_link` transform (where
the robot *thinks* it is relative to where it started; RViz uses it). Try
`ros2 run tf2_ros tf2_echo odom base_link`.

**Its weakness is drift.** It measures wheel turns, not real movement, so
anything that turns the wheels without moving the robot fools it: wheel
slip in sand, sideways sliding when a skid-steer robot turns, bumps and
craters. The errors add up and never correct themselves. From the sim tests:

| Test | Real (Gazebo) | Odometry said |
|---|---|---|
| Drive forward | 1.488 m | 1.437 m (3% off) |
| Turn in place, before wheel slip | 0.01 rad (stuck) | 1.11 rad (very wrong) |
| Turn in place, with wheel slip | 1.95 rad | 1.83 rad (6% off) |

Straight driving is fairly accurate; turning is the weak point, as is
typical for skid-steer robots. That's why autonomous Lunabotics robots
combine odometry with sensors that see the world — cameras with fiducial
markers, lidar, an IMU for heading — to correct the drift (the rules allow
beacons and fiducials on the arena frame for this).

---

## 6. The repo, file by file

```
lunabotics_sim/
├── README.md                     how to install, build, run, control
├── docs/GUIDE.md                 this file
├── .gitignore                    ignores build/, install/, log/, .venv-mujoco/
├── .venv-mujoco/                 Python environment for the MuJoCo test (not in git)
├── mujoco_dig/
│   └── dig_test.py               MuJoCo bucket digging test (section 13)
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
    │   ├── belt_animator.py      runs the buckets round the belt (section 8)
    │   └── generate_arena_world.py  writes the Artemis Arena world (section 4)
    ├── worlds/
    │   ├── artemis_arena.world.sdf   GENERATED: Lunabotics arena with BP-1
    │   └── meshes/artemis_arena_floor.stl  GENERATED: arena floor with craters
    ├── launch/
    │   ├── gazebo.launch.py      full simulation
    │   └── display.launch.py     RViz-only viewer
    └── rviz/                     (empty)
```

**GENERATED** files are rewritten by the scripts. Never edit them by hand —
change the script and re-run it.

### What reads and writes what

| File | Kind | Reads | Writes / used by |
|---|---|---|---|
| `scripts/fix_kinematics.py` | source | `urdf/assembly_1.urdf`, `meshes/*.stl` | writes `assembly_1_fixed.urdf`, `belt_animator.yaml`, `bucket_controller.yaml` |
| `scripts/add_ros2_control.py` | source | `assembly_1_fixed.urdf`, `belt_animator.yaml` | writes `assembly_1_sim.urdf` |
| `scripts/belt_animator.py` | source (ROS node) | `belt_animator.yaml`, `/joint_states` | publishes `/bucket_position_controller/commands`, `/bucket_joint_states` |
| `urdf/assembly_1.urdf` | CAD input | — | read by `fix_kinematics.py` |
| `urdf/assembly_1_fixed.urdf` | generated | — | used by `display.launch.py` (RViz) |
| `urdf/assembly_1_sim.urdf` | generated | — | used by `gazebo.launch.py` |
| `config/controllers.yaml` | source | — | used by `gazebo.launch.py` (controller manager, spawner) |
| `config/belt_animator.yaml` | generated | — | used by both launch files, `add_ros2_control.py` |
| `config/bucket_controller.yaml` | generated | — | used by `gazebo.launch.py` (bucket spawner) |
| `launch/gazebo.launch.py` | source | `assembly_1_sim.urdf`, all three configs | starts Gazebo, controllers, `belt_animator`, RViz |
| `launch/display.launch.py` | source | `assembly_1_fixed.urdf`, `belt_animator.yaml` | starts RViz, joint sliders, `belt_animator` |
| `meshes/*.stl` | CAD input | — | read by `fix_kinematics.py`; drawn by RViz, Gazebo and MuJoCo |
| `scripts/generate_arena_world.py` | source | — | writes `worlds/artemis_arena.world.sdf`, `worlds/meshes/artemis_arena_floor.stl` |
| `worlds/artemis_arena.world.sdf` | generated | `worlds/meshes/artemis_arena_floor.stl` | Gazebo world for `world:=artemis_arena` |
| `scripts/bekker_terramechanics.py` | source (Gazebo Python system) | `config/terramechanics.yaml`, `worlds/artemis_arena_height.npz` | applies soil forces to the wheels (`soil:=bekker`) |
| `config/terramechanics.yaml` | generated | — | wheel geometry for the soil model, written by `add_ros2_control.py` |
| `urdf/assembly_1_sim_bekker.urdf` | generated | — | robot for `soil:=bekker`, written by `add_ros2_control.py` |
| `worlds/artemis_arena_height.npz` | generated | — | arena surface as a height grid, written by `generate_arena_world.py` |
| `mujoco_dig/dig_test.py` | source | `assembly_1_fixed.urdf`, `belt_animator.yaml`, `meshes/*.stl` | builds and runs the MuJoCo digging scene |

Only edit the **source** files. Re-run the two scripts after changing them
or after a new Onshape export (see the README).

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
| The tree is rooted at a motor part sitting 5° off the chassis | The robot's frame isn't square to the robot, so "forward" drifts sideways |

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
| Wheel flanges | none (hidden in the hub; WheelSlip needs one collision per wheel) |
| Buckets, regolith bin | the mesh itself (they're low-poly) |
| Belt band | none |

**Step 8 — base frame.** `add_base_frame()` adds `base_link` as the new root
of the tree: x forward (toward the excavator, perpendicular to the wheel
axles), y left, z up, origin on the ground centred between the wheels — the
usual ROS convention. `root` hangs off it on a fixed joint, so no part moves
relative to any other; the whole robot is just re-squared by 5°.

**Step 9 — real masses and motor limits.** The CAD export has no
materials on most parts (motors weighed micrograms, each wheel 9 kg), so
`set_masses()` rebuilds them to the measured **50 kg** total:

| Part | Mass | Source |
|---|---|---|
| 4 drive gearmotors | 1.49 kg each, just inboard of each wheel | 57BLR70-24-02-HG50 datasheet |
| Frame, rollers, buckets, belt | 17.4 kg | CAD |
| Battery + electronics | 5 kg | **assumed** (`ELECTRONICS_MASS`) |
| Regolith bin | 1.5 kg | **assumed** (`BIN_MASS`) |
| Wheels | 5.19 kg each | whatever is left of 50 kg |

The wheel joints get the gearmotor's limits: **20 N·m** (gearbox continuous
limit; 40 N·m momentary) and **7.33 rad/s** (70 rpm rated). With the 50 kg
mass, that's realistic for a skid-steer robot: the wheels stall against a
wall instead of spinning forever, and pivoting on the spot needs wheel slip
(below) to work at all. Edit the
constants at the top of `fix_kinematics.py` if you know better masses.

**All motor and actuator limits** set by `fix_kinematics.py` (edit the
named constants at its top; regenerate as in the README):

| Actuator | Torque / force limit | Speed limit | Source |
|---|---|---|---|
| Wheel motors (×4), `revolute_1*` | **20 N·m** (`WHEEL_EFFORT`) | **7.33 rad/s = 70 rpm** (`WHEEL_VELOCITY`), ≈ 1.3 m/s | 57BLR70-24-02-HG50 datasheet: rated 17.63 N·m at 70 rpm, gearbox 20 N·m continuous / 40 N·m momentary |
| Drive roller (belt), `drive_roller_joint` | **6 N·m** (`ROLLER_EFFORT`) | **5.24 rad/s = 50 rpm** (`ROLLER_VELOCITY`), ≈ 0.25 m/s belt | TSINY TS-30GZ6287-R-LS 12 V worm-gear motor, 50 rpm version (~6 N·m rated, seller listings); assumed direct on the roller |
| Excavator lift, `excavator_lift` | **311 N = 2 × 35 lb** (`LIFT_EFFORT`) | **0.0508 m/s = 2"/s** (`LIFT_VELOCITY`); travel **−0.152 to 0 m** (6" stroke down from the CAD pose) | **two** Progressive Automations PA-14-6-35, one each side, driven together (12 V; 35 lb each; 1.38"/s at full load); assumed pushing straight along the rails, CAD pose = retracted |
| Bucket track joints (×30) | 200 N / N·m (`BUCKET_EFFORT`) | 5 m/s / rad/s (`BUCKET_VELOCITY`) | simulation only (moves the buckets round the belt) |
| Drive controller | — | 1.0 m/s forward, 2.0 rad/s turning | `linear.x.max_velocity`, `angular.z.max_velocity` in `config/controllers.yaml` |

**Live torque.** The wheels, the belt roller and the lift also report their
*effort* (torque in N·m, or force in N for the lift) on `/joint_states`
(`ros2 topic echo /joint_states --field effort`). Measured in the arena:

| Situation | Effort |
|---|---|
| Standing still | ~0.15 N·m per wheel |
| Cruising 0.3 m/s, `soil:=bekker` | ~4.3 N·m per wheel |
| Cruising 1.0 m/s, `soil:=bekker` | ~5.1 N·m per wheel |
| Accelerating to 1.0 m/s | 20 N·m per wheel (the motor limit) |
| Cruising, `soil:=rigid` | close to 0 |
| Lift holding the ladder | ~65 N (of 311 N available) |
| Belt running | ~0 N·m — the buckets don't load the belt in Gazebo |

What the wheel limit means for a 50 kg robot: four motors at 20 N·m on
0.1805 m wheels push up to 4 × 20 ÷ 0.1805 ≈ **443 N**, close to the robot's
weight (490 N). That's plenty to drive and climb on a rigid floor, but
pivoting on the spot needs the wheels to slip sideways (WheelSlip or the
soft-soil model) or the motors stall. The wheels never exceed 70 rpm, so
the robot tops out near 1.3 m/s (the drive controller caps it at 1.0 m/s). The belt roller is held to 50 rpm
(5.24 rad/s, ≈ 0.25 m/s belt speed) whatever you command; in Gazebo the
buckets lag their target by ~17 mm at 3 rad/s and ~29 mm at full speed,
settling when the belt stops. The lift lowers or raises its full
6" stroke in ~3 s (50 mm/s) and easily holds the 8 kg ladder, whose weight
puts ~64 N along the rails against the two actuators' combined 311 N
(each alone, 156 N, could still hold it).

### The result

```
base_link   (x forward, y left, z up; on the ground between the wheels)
 └─ root
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
| 0 s | `ros_gz_sim create` reads `/robot_description` and spawns the robot 1 cm above the ground (`base_link` sits on the ground under the wheels) |
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
   `diff_drive_controller` updates odometry and the `odom → base_link` transform.
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
| Where is the robot (estimate)? | `ros2 run tf2_ros tf2_echo odom base_link` |
| Where is the robot (truth)? | `gz model -m lunabotics -p` |
| Is the sim running at full speed? | `gz topic -e -t /stats -n 1` → `real_time_factor` |
| Is Gazebo using the GPU? | `nvidia-smi` → look for `gz-sim-gui-client` |
| What does Gazebo see? | `gz sdf -p <urdf file>` |
| Send a one-off command reliably | `ros2 topic pub -w 1 --times 3 <topic> <type> "<data>"` |

---

## 13. The MuJoCo digging test

`mujoco_dig/dig_test.py` answers one question: **how much regolith do the
buckets scoop and deliver to the bin?** It runs outside ROS, in the
**MuJoCo** physics engine, which handles piles of touching particles far
faster than Gazebo (Gazebo slowed below 1% of real time with a few hundred
particles).

### What is in the scene

Everything is built from the repaired URDF, so it matches the Gazebo robot:

| Part | How it is modelled |
|---|---|
| Whole robot | every CAD mesh, display only (no collisions, no mass) |
| Buckets (10) | the CAD bucket, moving round the belt loop with `loop_pose()` |
| Rollers | cylinders on the shafts |
| Bin | the CAD V-shaped hopper |
| Regolith | ~480 spheres, 2.4 cm, in a 10 cm deep pit in front of the ladder |
| Ground | a surface at z = 0 with the pit cut out of it |

The robot's frame is `base_link` (x forward, z up, origin on the ground),
so the pit is defined relative to the ladder's bottom shaft and lines up
with the robot.

### Hollow CAD parts and convex hulls

MuJoCo collides with the **convex hull** of a mesh — imagine shrink-wrapping
it. A hollow bucket would become a solid lump that can hold nothing. So the
script splits each hollow CAD part into its walls: in the STL every wall has
an outer face and a parallel inner face 2.5 mm (bucket) or 10 mm (bin) away,
and each such pair becomes one convex slab. The bucket comes out as 6 walls
(base, back, sloped wall, lid, two end plates) with its **only opening on the
leading face**; the bin as 4 walls (a V with two end walls), open on top.
Change the bucket in Onshape, re-export, and the test uses the new shape.

### How things move

- **Mocap bodies.** The chassis, the ladder and every bucket are MuJoCo
  *mocap* bodies: the script sets their position each step and they push
  anything in their way, but nothing pushes them back. So the buckets never
  stall, and the robot never tips — this tests what the buckets *carry*, not
  digging forces.
- **Rolling friction.** The spheres have rolling resistance (`condim="6"`),
  which Gazebo doesn't support, so a pile stays a pile.
- **Sequence.** The pile settles for 1 s with the ladder raised
  (`--start-lift`), then the belt starts, the ladder lowers into the bed at
  `--plunge` m/s down to `--lift`, and the robot creeps forward at
  `--advance` m/s to keep feeding the buckets.
- **Dig direction.** Buckets go down the underside of the ladder, scoop
  forward round the bottom shaft, climb the top face, and tip over the top
  shaft into the bin.

### What it reports

Every 2 s it prints how many particles are **in the bin** (inside the bin's
box, moving with the robot) and **lifted** (above the bed, not in the bin —
i.e. riding the buckets).

### What we found

| Change | Effect |
|---|---|
| Lowering the ladder from −0.03 to −0.12 m | yield went up several times |
| Creeping forward faster than ~5 mm/s | worse: the robot overruns material |
| Open scoop → your CAD bucket (with a lid) | ~93–110 in the bin vs 87; up to ~110 riding at once |
| Starting the ladder overlapping the pile | particles explode outward — always start clear and lower in |

Particle piles are chaotic, so expect about ±15% between runs; compare
designs over a few runs. It runs at about 0.15–0.2× real time.

### Limits

Buckets and robot follow scripted paths (no stalling, no tipping, wheels
don't spin); 2.4 cm spheres are far coarser than real regolith; this is a
capacity test, not a soil-mechanics model.

---

## 14. How realistic is this simulation?

Realistic enough to develop software against and to compare designs, but
not a faithful copy of the robot in BP-1.

### Reasonably realistic

| Aspect | Why it's believable |
|---|---|
| Robot geometry | straight from the CAD, every part in its true place |
| Arena | official dimensions, zones, crater and rock sizes, column |
| Drive motors | datasheet limits (20 N·m, 70 rpm): wheels stall against walls, pivoting needs slip |
| Total mass | the measured 50 kg |
| Driving | gradual spin-up with wheel slip, skid-steer turning, pitching over crater rims |

### Estimated

| Aspect | What's uncertain |
|---|---|
| Mass distribution | battery, electronics and bin masses are assumed; wheels take the remainder (affects tipping, braking, wheel loads) |
| Wheel slip (0.6) | estimated for loose sand, not measured on these wheels in BP-1 |
| Ground friction (0.8) | from BP-1's internal friction angle; real wheel traction can be lower |
| Excavator actuators | belt motor is the TSINY TS-30GZ6287 (6 N·m, 50 rpm) from seller listings, assumed direct on the roller; the lift is two PA-14-6-35 actuators, one each side (311 N combined, 50.8 mm/s, 6" stroke), assumed to stay in sync and to push straight along the rails from the CAD pose. The buckets don't load the belt motor, so it never stalls |

### Not modelled

1. **Soil.** With `soil:=rigid` the ground is hard. `soil:=bekker` adds
   sinkage, slip and rolling resistance from standard terramechanics, but
   with lunar-regolith constants (not measured BP-1 ones), smooth wheels,
   no ruts and no bulldozing.
2. **No digging in Gazebo.** Buckets scrape a hard floor. The MuJoCo test
   digs, but with 2.4 cm pebbles (real grains are fractions of a mm) and
   buckets that never stall.
3. **Ideal electronics:** no battery drain or voltage sag, no motor heating,
   no control lag, no sensor noise, perfect wheel encoders.
4. **No dust.**

### What to trust it for

- **Good for:** teleop and autonomy software, navigation and path planning
  around the arena's rocks and craters, controller and ROS integration,
  and *relative* design comparisons (bucket A vs bucket B).
- **Not good for:** absolute numbers — how fast the robot crosses BP-1,
  whether it climbs out of a particular crater, regolith collected per
  minute, or whether a motor is strong enough to dig.

### Making it more realistic

1. Measure the real masses (battery, electronics, bin, wheels) and the
   excavator motor specs — easy, and removes most of the guesses.
2. Drive the real robot on BP-1 (or similar sand), measure slip and pulling
   force, and tune the slip and friction values to match.
3. For sinkage and digging forces, use a terrain model built for loose soil
   (e.g. Project Chrono's soil models or a terramechanics plugin) — a much
   bigger step.

---

## 15. Gotchas we hit (and why)

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

**Particles for digging were too slow in Gazebo.** A pile of a few hundred
particles slowed DART to under 1% real time, and spheres rolled away
(Gazebo has no rolling resistance). That's why the digging test uses MuJoCo.

**A MuJoCo pile of particles exploded.** Particles that start overlapping a
bucket get shoved apart violently. The ladder now starts clear of the pile
and is lowered in.

**The robot was 5° off square.** Onshape rooted the tree at a motor part
sitting 5° off the chassis, so "forward" drifted sideways and the MuJoCo pit
looked crooked. `fix_kinematics.py` now adds a proper `base_link`.

**The MuJoCo viewer breaks with the NVIDIA offload variables** (`OpenGL
error 0x502`) on this Wayland desktop. Run it without them; physics is on
the CPU anyway.

**Where a force acts matters.** The first soft-soil version applied rolling
resistance at the bottom of the wheel, where it cancelled the traction
torque: the robot drove the same, but the motors read ~0.1 N·m while
cruising instead of ~4.3. Soil pressure pushes on the rim towards the axle,
so rolling resistance must be applied through the axle; only shear twists
the wheel.

**Measure speeds in sim time.** The arena sim runs at ~0.6× real time; a
test that timed the robot with the wall clock saw a fake "45% wheel slip".
Use the timestamps Gazebo and ROS (`use_sim_time`) provide.

**`diff_drive_controller` publishes no odometry until its first command.**
A test that waited for `/diff_drive_controller/odom` before sending
anything waited forever. Send a stop command first.

**Old Gazebo servers kept running in the background** after tests and
answered queries meant for the new one. If results look stale, check
`ps aux | grep gz-sim` and stop leftovers.

---

## 16. The robot's parts

Every part of the rover, what it is, and how the simulation models it. The
CAD link names are what you'll see in the URDF, RViz and Gazebo.

### Bought-in components

| Component | Qty | Specs | Where in the sim |
|---|---|---|---|
| **Drive gearmotor** — StepperOnline 57BLR70-24-02-HG50 | 4 | 24 V 172 W brushless DC + 50:1 planetary gearbox; rated 17.63 N·m at 70 rpm; gearbox 20 N·m continuous, 40 N·m momentary; 1.49 kg | in the CAD (parts P1–P6, merged into `root`); wheel joints `revolute_1*` limited to 20 N·m / 7.33 rad/s; 1.49 kg each on the chassis |
| **Belt motor** — TSINY TS-30GZ6287-R-LS | 1 | 12 V worm-gear DC motor, 50 rpm version, ~6 N·m rated, 100 W (seller listings) | not in the CAD; `drive_roller_joint` limited to 6 N·m / 5.24 rad/s, assumed direct on the top roller |
| **Lift actuator** — Progressive Automations PA-14-6-35 | 2 | 12 V linear actuator, 6" (152 mm) stroke, 35 lb (156 N), 2.00"/s no load, 1.38"/s full load | not in the CAD; one each side of the ladder, modelled as one joint `excavator_lift`: 311 N combined, 50.8 mm/s, −0.152 to 0 m |

### Structure and mechanisms (from the CAD)

| Part | CAD links | Qty | Size (mm) | Mass in sim | What it is |
|---|---|---|---|---|---|
| 30×30 extrusion, 467 mm | `30___30___1`, `30___30___2` | 2 | 467×30×30 | 0.35 kg each | chassis frame |
| 30×30 extrusion, 403 mm | `30___30___403` … `_3` | 4 | 403×30×30 | 0.30 kg each | chassis frame |
| 30×30 extrusion, 700 mm | `30___30___700`, `_700_1` | 2 | 700×30×30 | 0.52 kg each | chassis frame |
| 30×60 extrusion, 400 mm | `30___60___400`, `_400_1` | 2 | 400×60×30 | 0.58 kg each | chassis frame |
| 30×60 extrusion, 600 mm | `30___60___400_2`, `_3` | 2 | 600×60×30 | 0.88 kg each | inclined rails the ladder slides on |
| 40×40 extrusion, 400 mm | `40___40___1`, `_1_1` | 2 | 400×40×40 | 0.70 kg each | chassis frame |
| 40×40 extrusion, 407 mm | `40___40___407`, `_407_1` | 2 | 407×40×40 | 0.71 kg each | chassis frame |
| Ladder side plates (30×30, 700 mm) | `30___30___403_4`, `_5` | 2 | 700×30×30 | 0.52 kg each | the excavator ladder's sides; move with the lift |
| Slider brackets | `slider_bracket`, `_1` | 2 | 46×45×40 | 0.06 kg each | fixed to the rails; the ladder slides through them |
| Drive motor parts | `root` + `57blr70_24_02_hg50_*` | 4 sets | Ø57 | 1.49 kg each (in `root`) | the drive gearmotors above |
| Wheel flanges | `flange` … `flange_3` | 4 | 57×57×20 | — (inside the wheel) | hub adaptor; driven by `revolute_1*` |
| Wheels | `wheel`, `wheel_1`, `wheel_mirrored`, `wheel_mirrored_1` | 4 | Ø361 × 100 | 5.19 kg each | drive wheels (two are mirror images) |
| Rollers | `roller_1` (top), `roller` (bottom) | 2 | Ø94 × 260 | 2.29 kg each | belt rollers: top driven, bottom idler |
| Roller shafts | `rod` (top), `rod_1` (bottom) | 2 | Ø15 × 265 | 0.13 kg each | axles the rollers turn on |
| Belt | `belt` | 1 | 713×305×250 | 0.64 kg | the bucket belt (shape only; doesn't move) |
| Excavator buckets | `bucket_1` … `bucket_10` | 10 | 205×89×79 | 0.15 kg each | hollow, 2.5 mm walls, open on the leading face |
| Regolith bin | `bucket` | 1 | 400×300×192 | 1.5 kg (**assumed**) | V-shaped hopper, open top |
| Electronics box | `electronics` | 1 | 180×157×135 | 5 kg (**assumed**, incl. battery) | electronics and battery |

### Mass by subsystem (50 kg total, measured)

| Subsystem | Mass | Source |
|---|---|---|
| Wheels | 20.7 kg | remainder of 50 kg |
| Chassis frame (extrusions, brackets) | 8.8 kg | CAD |
| Excavator ladder (side plates, shafts, rollers, belt, buckets) | 8.0 kg | CAD |
| Drive gearmotors | 6.0 kg | datasheet |
| Battery + electronics | 5.0 kg | **assumed** |
| Regolith bin | 1.5 kg | **assumed** |

The belt motor and lift actuators aren't separate parts in the model; their
mass is inside the 50 kg total. Change the assumed masses with the
constants at the top of `fix_kinematics.py`.

---

## 17. Glossary

| Term | Meaning |
|---|---|
| **Artemis Arena** | the Lunabotics competition arena at Kennedy Space Center |
| **Bekker–Wong** | standard terramechanics model of wheels in soft soil |
| **base_link** | the robot's main frame: x forward, y left, z up |
| **BP-1** | Black Point-1, crushed-basalt lunar regolith simulant in the arena |
| **colcon** | ROS 2 build tool |
| **convex hull** | the shape you get by shrink-wrapping a mesh; what MuJoCo collides with |
| **controller** | ros2_control plugin that turns commands into joint actions |
| **controller_manager** | runs controllers in a loop |
| **DART** | Gazebo's default physics engine |
| **frame** | a coordinate system attached to a part |
| **Gazebo / gz sim** | the physics simulator |
| **hardware interface** | ros2_control's link to real or simulated joints |
| **launch file** | Python script that starts a set of nodes |
| **link** | a rigid part in a URDF |
| **mocap body** | MuJoCo body moved by script; pushes things but can't be pushed |
| **MuJoCo** | fast physics engine, used here for the digging test |
| **MJCF** | MuJoCo's model format |
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
