"""
Settings for the MuJoCo digging test: where the robot model lives, and every
tunable number (pit size, servo gains, belt drive model, contact groups).

The motor limits themselves (lift force and speed, belt torque and speed)
aren't here: they're imported from fix_kinematics.py, so the MuJoCo test
and the Gazebo model always use the same ones.
"""
import os
import sys

# Paths: the ROS package with the repaired URDF, meshes and belt config.
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(REPO, "src", "lunabotics_model")
# Make fix_kinematics.py importable from the other files here.
sys.path.insert(0, os.path.join(PKG, "scripts"))

# Robot geometry not in the URDF's collision shapes.
ROLLER_RADIUS = 0.047
ROLLER_LENGTH = 0.26
MAX_WALL = 0.015  # thickest wall to look for when splitting a hollow mesh
# Pit footprint relative to the ladder's bottom shaft, in base_link axes
# (x forward, y left; m): the strip the 0.205 m wide buckets sweep. Every
# particle costs sim speed, so keep it to where the buckets reach.
PIT_FORWARD = (-0.06, 0.24)
PIT_LEFT = (-0.13, 0.17)
# The regolith fills a pit this deep, level with the surface the robot
# stands on, so lowering the ladder digs into a bed rather than a heap.
PIT_DEPTH = 0.10
GROUND_HALF_SIZE = 3.0  # m, extent of the surface around the pit
# Servo gains. The lift servo stands in for the actuators' lead screws; the
# bucket servos for the belt holding each bucket at its place on the loop.
LIFT_KP, LIFT_KV = 50000.0, 3000.0      # N/m, N*s/m
BUCKET_KP, BUCKET_KV = 20000.0, 150.0   # N/m, N*s/m along / across the ladder
BUCKET_ROT_KP, BUCKET_ROT_KV = 30.0, 0.3  # N*m/rad, N*m*s/rad about the shafts
# Belt drive model: speed loop gain, and the moving mass of belt + buckets
# as seen along the belt (sets how fast the belt speeds up and slows down).
BELT_GAIN = 50.0    # N*m per (m/s) of belt speed error
BELT_MASS = 3.0     # kg
LOAD_FILTER = 0.02  # s, smoothing of the belt load the motor sees
# Contact groups: buckets and rollers touch only the regolith (they'd
# otherwise snag on the pit edges and the bin, which they clear on the robot).
REGOLITH_BITS = 'contype="1" conaffinity="3"'
DIGGER_BITS = 'contype="2" conaffinity="0"'
