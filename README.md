# reBot Arm ROS2 SDK

<p align="center">
  <img src="./media/rebot_arm_b601.png" alt="reBot Arm" width="720">
</p>

<p align="center">
  <strong>ROS2 · Arm Control · Gripper Control · Trajectory Interfaces · RViz · Open Source</strong>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble | Jazzy-blue.svg" alt="ROS2 Humble">
  <img src="https://img.shields.io/badge/Python-3.10+-blue.svg" alt="Python 3.10">
  <img src="https://img.shields.io/badge/Version-v0.3.0-brightgreen.svg" alt="Version v0.3.0">
  <img src="https://img.shields.io/badge/Platform-Ubuntu%2022.04+-orange.svg" alt="Ubuntu 22.04+">
  <img src="https://img.shields.io/badge/Hardware-DM%20%7C%20RS-lightgrey.svg" alt="DM and RS">
</p>

<p align="center">
  <strong>
    <a href="./README_zh.md">简体中文</a> &nbsp;|&nbsp;
    <a href="./README.md">English</a> &nbsp;|&nbsp;
    <a href="./API_zh.md">API Reference</a>
  </strong>
</p>

---

## Overview

Current version: `v0.3.0`

`rebotarm_ros2` is the ROS2 SDK workspace for reBot Arm B601 DM and RS models. It
wraps the current `reBotArm_control_py` Python control library into ROS2 topics,
services and actions, providing a unified entry point for development, planning,
RViz visualization, gravity compensation and per-motor debugging.

The current RS URDF uses measured masses and centers of mass. The model totals 6.5503 kg. Each gripper finger weighs 55.9 g and has 0–50 mm of travel. [URDF source](src/rebotarm_bringup/description/RS/urdf/ReBot_Arm_RS.urdf)

The workspace contains five ROS2 packages:

| Package | Purpose |
|---|---|
| `rebotarm_msgs` | Custom msg / srv / action interfaces |
| `rebotarmcontroller` | Driver node package, including `reBotArmController` |
| `rebotarm_bringup` | Launch files, configs, URDF and RViz resources |
| `rebotarm_moveit_config` | MoveIt 2 config, SRDF, ros2_control and RViz config |
| `rebotarm_moveit_demos` | MoveIt 2 demos, including pick-place and rectangle drawing |

---

## Features

- Publishes arm status: `/rebotarm/joint_states`, `/rebotarm/arm_status`
- Provides core services: `enable`, `disable`, `set_zero`, `safe_home`
- Supports Cartesian targets: `MoveToPoseIK` service, `MoveToPose` action
- Supports the standard `control_msgs/action/FollowJointTrajectory` interface
- Supports gripper control: `SetGripper` service, `GripperCommand` action
- Supports single-joint commands: `JointMitCmd`, `JointPosVelCmd`

---

## Requirements

| Item | Requirement |
|---|---|
| Arm | reBot Arm B601 DM or RS |
| Communication | DM: USB serial bridge; RS: SocketCAN interface |
| Host | Ubuntu 22.04+, ROS2, Python 3.10+ |

Wiring:

1. Connect the USB2CAN serial bridge to the arm CAN bus.
2. Connect the gripper motor to the same CAN bus.
3. Plug the USB2CAN bridge into the host and check the device name:

```bash
ls /dev/ttyACM*
```

For temporary serial access:

```bash
sudo chmod 666 /dev/ttyACM0
```

RS uses SocketCAN by default. Bring up the CAN interface before starting the ROS2 driver:

```bash
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

---

## Development Setup

### Step 1. Install ROS2

Choose and install the appropriate ROS2 distribution from the
[official ROS getting started guide](https://www.ros.org/blog/getting-started/).

### Step 2. Clone the ROS2 workspace

Preferred upstream repository:

```bash
git clone https://github.com/Seeed-Projects/reBotArmController_ROS2.git rebotarm_ros2
cd rebotarm_ros2
```

Current development repository:

```bash
git clone https://github.com/EclipseaHime017/reBotArmController_ROS2.git rebotarm_ros2
cd rebotarm_ros2
```

### Step 3. Install motorbridge

Install `motorbridge` from the official PyPI index:

```bash
python3 -m pip install --user --index-url https://pypi.org/simple motorbridge
```

### Step 4. Clone the low-level SDK

```bash
mkdir -p third_party
git clone https://github.com/vectorBH6/reBotArm_control_py.git third_party/reBotArm_control_py
```

## Build

```bash
colcon build --symlink-install
source install/setup.bash
```

Verify installed executables:

```bash
ros2 pkg executables rebotarmcontroller
```

Expected output:

```text
rebotarmcontroller reBotArmController
rebotarmcontroller GravityCompensation
rebotarmcontroller GripperControl
rebotarmcontroller MoveTo
rebotarmcontroller MoveToPose
```

---

## Directory Layout

```text
rebotarm_ros2/
├── README.md
├── README_zh.md
├── API_zh.md
├── PLAN.md
├── instruction.md
└── src/
    ├── rebotarm_msgs/
    │   ├── msg/
    │   ├── srv/
    │   └── action/
    ├── rebotarmcontroller/
    │   ├── rebotarmcontroller/
    │   │   ├── rebotarm_controller.py
    │   │   ├── hardware_manager.py
    │   │   ├── ros_publishers.py
    │   │   ├── ros_services.py
    │   │   ├── ros_actions.py
    │   │   ├── motor_passthrough.py
    │   │   ├── conversions.py
    │   │   └── examples/
    ├── rebotarm_bringup/
    │   ├── launch/
    │   ├── config/
    │   ├── description/
    │   └── rviz/
    ├── rebotarm_moveit_config/
    │   ├── config/
    │   └── launch/
    └── rebotarm_moveit_demos/
        ├── config/
        ├── launch/
        └── rebotarm_moveit_demos/
```

---

## Quick Start

Before using the robot, note the following: **The arm controller has a high
degree of freedom. Before enabling the controller or powering the arm, make
sure the workspace is clear of people and obstacles. Review every motion
command carefully to avoid accidents. Dangerous operation is strictly
prohibited; you are responsible for any consequences.**

### Launch the full system

Start the controller node, `robot_state_publisher`, and optionally RViz:

```bash
ros2 launch rebotarm_bringup bringup.launch.py
```

`reBotArmController` connects to real hardware on startup. The default model is
defined by `rebotarm_bringup/config/rebotarm_hardware.yaml` and is currently
`dm`.

If you use the RS arm and have not changed `default_model`, pass `model:=rs`
explicitly and set `channel:=can0`.

```bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM1
ros2 launch rebotarm_bringup bringup.launch.py model:=rs channel:=can0
```

Enable RViz to visualize the arm motion:

```bash
ros2 launch rebotarm_bringup bringup.launch.py use_rviz:=true
```

### Launch only the driver

Unlike `bringup.launch.py`, this starts only the controller node. Use it when
you want the minimal hardware control process without `robot_state_publisher`
or RViz visualization.

```bash
ros2 launch rebotarm_bringup driver.launch.py
```

### Run the controller directly

```bash
ros2 run rebotarmcontroller reBotArmController
```

Unlike `driver.launch.py`, which passes configuration files from
`rebotarm_bringup/config`, running the controller directly falls back to the
default SDK arm configuration. For normal use, launching through ROS is
recommended.

---

## Direct Pose Motion

Without running an example script, you can call ROS services and actions directly.
Start the controller in one terminal:

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM0
```

In another terminal:

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
```

1. Enable the arm:

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger
```

2. Move the TCP to a target pose:

```bash
ros2 action send_goal /rebotarm/move_to_pose rebotarm_msgs/action/MoveToPose \
  "{target_pose: {position: {x: 0.30, y: 0.0, z: 0.30}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, duration: 2.0}"
```

`move_to_pose` executes through the SDK end-pose controller. The arm control
mode is selected by `rebotarm_hardware.yaml`: DM defaults to `posvel`, RS
defaults to `mit`.

3. Close the gripper and return to safe home:

```bash
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger
```

4. Disable the arm:

```bash
ros2 service call /rebotarm/disable std_srvs/srv/Trigger
```

---

## Example Scripts

All examples assume `reBotArmController` is already running:

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM0
```

Examples are installed as ROS2 executables and can be launched with `ros2 run`.

Source files:

```text
src/rebotarmcontroller/rebotarmcontroller/examples/move_to.py
src/rebotarmcontroller/rebotarmcontroller/examples/move_to_pose.py
src/rebotarmcontroller/rebotarmcontroller/examples/gravity_compensation.py
src/rebotarmcontroller/rebotarmcontroller/examples/gripper_control.py
```

### move_to.py

Move all 6 joints to absolute joint angles in radians:

```bash
ros2 run rebotarmcontroller MoveTo -- \
  0.20 -0.20 -0.20 -0.20 0.10 -0.10 \
  --duration 8.0
```

Move a single joint:

```bash
ros2 run rebotarmcontroller MoveTo -- --joint joint3 --position -0.20 --duration 5.0
```

### move_to_pose.py

Move the TCP to a pose:

```bash
ros2 run rebotarmcontroller MoveToPose -- --x 0.30 --y 0.0 --z 0.30 --qw 1.0 --duration 2.0
```

### gravity_compensation.py

```bash
ros2 run rebotarmcontroller GravityCompensation
```

The script calls `/rebotarm/enable`, starts gravity compensation, then on
`Ctrl+C` calls `/rebotarm/gravity_compensation/stop`, `/rebotarm/safe_home` and
`/rebotarm/disable`.

### gripper_control.py

```bash
ros2 run rebotarmcontroller GripperControl
```

Interactive commands:

```text
o / open    open gripper
c / close   close gripper
q / quit    quit
```

---

## API Reference

The full ROS2 API is documented in [API_zh.md](API_zh.md).

It includes:

- Topics, services and actions with their message types
- `/rebotarm` namespace, QoS, units and state machine conventions
- Custom interfaces such as `JointMitCmd`, `JointPosVelCmd`, `ArmStatus` and `MoveToPose`
- Examples for TCP motion, gripper control, gravity compensation and raw commands
- Integration notes for higher-level applications and multi-arm setups

---

## Configuration

`rebotarm_bringup/config/` provides default driver configuration:

| File | Description |
|---|---|
| `rebotarm_hardware.yaml` | ROS2 hardware selection and overrides for DM / RS SDK configs |
| `driver_params.yaml` | ROS parameter example |

Common launch parameters:

| Parameter | Default | Description |
|---|---|---|
| `hardware_config` | built-in `rebotarm_hardware.yaml` | ROS2 hardware config path |
| `model` | empty string | Use `default_model` when empty; set `dm` or `rs` explicitly |
| `channel` | empty string | Use YAML value when empty; override communication channel when set |
| `joint_state_rate` | `100.0` | Publish rate for `/rebotarm/joint_states` |
| `cmd_arbitration` | `reject` | Low-level command arbitration during arm trajectory execution |
| `arm_namespace` | `rebotarm` | ROS namespace prefix |
| `frame_id` | `base_link` | Base frame ID reserved for TF, perception and planning |
| `ee_frame_id` | `end_link` | End-effector frame ID reserved for TF, perception and planning |
| `use_rviz` | `false` | Start bringup RViz |
| `disable_after_safe_home` | `true` | Controls whether motors are disabled after safe home completes |

Model defaults in `rebotarm_hardware.yaml`:

| Model | Default channel | Arm mode | Gripper limits |
|---|---|---|---|
| `dm` | `/dev/ttyACM0` | `posvel` | open `-5.0`, close `0.0` |
| `rs` | `can0` | `mit` | open `5.0`, close `0.0` |

---

## MoveIt 2

MoveIt 2 is the motion planning framework used here for inverse kinematics,
collision checking, trajectory planning and execution. The demos are separated
into their own package so application flows stay isolated from the base driver.
For more details, see the official [MoveIt 2 Documentation](https://moveit.picknik.ai/main/index.html).

MoveIt-related content is split into two packages:

| Package | Purpose |
|---|---|
| `rebotarm_moveit_config` | Robot model, SRDF, kinematics, joint limits, controller and RViz config |
| `rebotarm_moveit_demos` | Application demos based on MoveIt 2 |

The MoveIt environment uses simulated hardware through `ros2_control` and
`move_group` for planning and execution. It is intended for validating the
model, IK, trajectory planning and demo flow in RViz.

This repository also supports real hardware. Before connecting real hardware,
make sure the arm zero configuration, joint directions, joint limits, velocity
limits and gripper range are all correct, or keep the default repository
configuration.

### MoveIt Environment Setup

Make sure the ROS2 environment is available first. You can install packages for
the currently sourced ROS distribution through `ROS_DISTRO`:

```bash
sudo apt update
sudo apt install -y \
  ros-${ROS_DISTRO}-moveit \
  ros-${ROS_DISTRO}-moveit-configs-utils \
  ros-${ROS_DISTRO}-moveit-kinematics \
  ros-${ROS_DISTRO}-moveit-planners-ompl \
  ros-${ROS_DISTRO}-moveit-simple-controller-manager \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-ros2-controllers \
  ros-${ROS_DISTRO}-xacro
```

The MoveIt config and demos are included in this workspace. After installing
dependencies, rebuild the workspace:

```bash
cd your/path/to/rebotarm_ros2
colcon build --symlink-install
source install/setup.bash
```

Verify the MoveIt packages and demo entry points:

```bash
ros2 pkg list | grep rebotarm_moveit
ros2 pkg executables rebotarm_moveit_demos
```

Expected entries include:

```text
rebotarm_moveit_demos draw_square
rebotarm_moveit_demos pick_place
```

### Use MoveIt

MoveIt planning can be used through the RViz GUI or through ROS nodes, in both
simulation and real scenes.

#### Use MoveIt in simulation

MoveIt uses the ros2_control virtual hardware interface for RViz simulation:

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_moveit_config demo.launch.py
```

For the RS arm, pass `model:=rs` when the default model has not been changed:

```bash
ros2 launch rebotarm_moveit_config demo.launch.py model:=rs
```

By default this starts:

- `move_group`
- `robot_state_publisher`
- `ros2_control_node`
- `joint_state_broadcaster`
- `rebotarm_controller`
- `gripper_controller`
- RViz with the MoveIt MotionPlanning plugin

RViz opens automatically and loads the robot URDF model. You can control motion
through the panel on the left side of the GUI.

#### Use MoveIt with reBotArm hardware

For the real robot, first start the controller with the hardware interface
instead of the virtual controller, then start the hardware MoveIt environment:

```bash
ros2 launch rebotarm_bringup driver.launch.py
```

In another terminal:

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_moveit_config hardware.launch.py
```

To repeat: before running any demo on real hardware, make sure the workspace is
clear of people and obstacles, verify the planned path in RViz, and be ready to
stop the controller at any time.

### Run the draw-square demo

Start the MoveIt environment first, then run in another terminal:

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_moveit_demos draw_square.launch.py
```

`draw_square` moves `gripper_tcp` through the four corners of a coplanar rectangle.
Default parameters:

```text
src/rebotarm_moveit_demos/config/draw_square.yaml
src/rebotarm_moveit_demos/config/draw_square_rs.yaml
```

Common parameters:

| Parameter | Description |
|---|---|
| `start_point` | Joint reset position before the demo starts |
| `rectangle_center` | Rectangle center in `base_link` |
| `rectangle_width` / `rectangle_height` | Rectangle dimensions in meters |
| `tcp_rpy` | TCP orientation, defaulting to a downward-facing gripper |
| `tcp_yaw_offsets` | Alternative IK yaw values used to avoid large joint6 wraps |

### Run the pick-place demo

Start the MoveIt environment first, then run in another terminal:

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_moveit_demos pick_place.launch.py
```

Default parameters:

```text
src/rebotarm_moveit_demos/config/pick_place.yaml
src/rebotarm_moveit_demos/config/pick_place_rs.yaml
```

Common parameters:

| Parameter | Description |
|---|---|
| `ready_point` | Ready joint position used before and after pick/place |
| `pick_position` | Object bottom-center position in `base_link` |
| `pick_tcp_rpy` / `place_tcp_rpy` | TCP orientation for pick and place |
| `object_dimensions` | Planning-scene object dimensions in meters |
| `max_gripper_width` | Maximum total gripper travel, default `0.10 m` for RS |
| `open_gripper_position` / `grasp_gripper_position` / `closed_gripper_position` | Simulated single-side gripper joint positions |
| `hardware_open_gripper_position` / `hardware_closed_gripper_position` | Hardware gripper motor open/close positions |

### MoveIt configuration files

| File | Description |
|---|---|
| `rebotarm_moveit_config/config/rebotarm.urdf.xacro` | Robot model used by MoveIt |
| `rebotarm_moveit_config/config/rebotarm.srdf` | MoveIt groups, end effector and default states |
| `rebotarm_moveit_config/config/rebotarm_rs.urdf.xacro` | RS robot model used by MoveIt |
| `rebotarm_moveit_config/config/rebotarm_rs.srdf` | RS MoveIt groups and collision semantics |
| `rebotarm_moveit_config/config/kinematics.yaml` | IK solver configuration |
| `rebotarm_moveit_config/config/joint_limits.yaml` | Joint limits used by MoveIt planning |
| `rebotarm_moveit_config/config/moveit_controllers.yaml` | Shared DM/RS MoveIt trajectory execution controller config |
| `rebotarm_moveit_config/config/ros2_controllers.yaml` | Shared DM/RS ros2_control controller config |
| `rebotarm_moveit_config/config/initial_positions.yaml` | Initial joint positions for simulated hardware |
| `rebotarm_moveit_demos/config/draw_square.yaml` | Draw-square demo parameters |
| `rebotarm_moveit_demos/config/draw_square_rs.yaml` | RS draw-square demo parameters |
| `rebotarm_moveit_demos/config/pick_place.yaml` | Pick-place demo parameters |
| `rebotarm_moveit_demos/config/pick_place_rs.yaml` | RS pick-place demo parameters |

---

## FAQ / Troubleshooting

### `ros2: command not found`

The ROS2 environment is not available in the current shell, or ROS2 is not
installed. Install ROS2 first, then source the matching distribution once per
terminal session:

```bash
source /opt/ros/humble/setup.bash
```

If you use Jazzy, replace `humble` with `jazzy`.

To load ROS2 automatically in new terminals, add the source command to
`~/.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### ROS2 executable not found

If `ros2 run` or `ros2 launch` cannot find a package or executable, the
workspace overlay is usually not sourced in the current terminal. Rebuild if
needed, then source the workspace:

```bash
cd your/path/to/rebotarm_ros2
colcon build --symlink-install
source install/setup.bash
```

You can verify installed entry points with:

```bash
ros2 pkg executables rebotarmcontroller
ros2 pkg executables rebotarm_moveit_demos
```

### Serial device not found

If startup reports:

```text
open serial port /dev/ttyACM0 failed: No such file or directory
```

Check the actual device:

```bash
ls /dev/ttyACM*
```

Then override `channel`:

```bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM1
```

For RS, check that the SocketCAN interface is up:

```bash
ip -details link show can0
sudo ip link set can0 up type can bitrate 1000000
ros2 launch rebotarm_bringup bringup.launch.py model:=rs channel:=can0
```

### Permission denied

If the serial device exists but cannot be opened:

```bash
sudo usermod -a -G dialout $USER
```

Log out and log back in for the group change to take effect.

### RViz model is not displayed

Check that URDF mesh paths use:

```text
package://rebotarm_bringup/description/...
```

### FastDDS SHM port warning

If the terminal shows:

```text
[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7002: open_and_lock_file failed
```

This is usually caused by stale FastDDS shared-memory lock files after ROS2
processes exited unexpectedly. If services and actions still respond, it usually
does not affect control. To clean it up, stop ROS2 processes first, then run:

```bash
pkill -f ros2
pkill -f reBotArmController
rm -f /dev/shm/fastrtps_port*
```

To temporarily avoid shared-memory transport before launching ROS2:

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```
