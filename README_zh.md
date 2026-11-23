# reBot Arm ROS2 SDK

<p align="center">
  <img src="./media/rebot_arm_b601.png" alt="reBot Arm" width="720">
</p>

<p align="center">
  <strong>ROS2 · 机械臂控制 · 夹爪控制 · 轨迹接口 · RViz 可视化 · 全开源</strong>
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
    <a href="./API_zh.md">API 文档</a>
  </strong>
</p>

---

## 项目介绍

当前版本：`v0.3.0`

`rebotarm_ros2` 是 reBot Arm B601 DM 和 RS 机械臂的 ROS2 SDK 工作空间。它将新版
`reBotArm_control_py` Python 控制库封装为 ROS2 topic、service 和 action，
作为二次开发、上层规划、可视化、重力补偿和单电机调试的统一入口。

当前 RS URDF 使用实测质量和质心。模型总质量为 6.5503 kg，每侧夹爪手指为 55.9 g，滑动行程各为 0–50 mm。 [URDF 源文件](src/rebotarm_bringup/description/RS/urdf/ReBot_Arm_RS.urdf)

当前工作空间包含五个 ROS2 包：

| 包 | 作用 |
|---|---|
| `rebotarm_msgs` | 自定义 msg / srv / action 接口 |
| `rebotarmcontroller` | 控制节点包，提供 `reBotArmController` 节点 |
| `rebotarm_bringup` | launch、配置、URDF、RViz 等启动资源 |
| `rebotarm_moveit_config` | MoveIt 2 配置、SRDF、ros2_control、RViz 配置 |
| `rebotarm_moveit_demos` | MoveIt 2 应用示例，如抓取放置和矩形轨迹 |

---

## 核心功能

- 发布机械臂状态：`/rebotarm/joint_states`、`/rebotarm/arm_status`
- 提供基础服务：`enable`、`disable`、`set_zero`、`safe_home`
- 支持笛卡尔目标：`MoveToPoseIK` service、`MoveToPose` action
- 支持标准轨迹接口：`control_msgs/action/FollowJointTrajectory`
- 支持夹爪控制：`SetGripper` service、`GripperCommand` action
- 支持单个关节指令：`JointMitCmd`、`JointPosVelCmd`

---

## 环境配置

| 组件 | 型号 / 要求 |
|---|---|
| 机械臂 | reBot Arm B601 DM 或 RS |
| 通信接口 | DM：USB 串口桥；RS：SocketCAN |
| 主机 | Ubuntu 22.04+，ROS2，Python 3.10+ |

接线说明：

1. 将 USB2CAN 串口桥接器连接到机械臂 CAN 总线。
2. 将夹爪电机接入同一条 CAN 总线。
3. 将 USB2CAN 插入主机，并确认设备名：

```bash
ls /dev/ttyACM*
```

如果需要临时开放串口权限：

```bash
sudo chmod 666 /dev/ttyACM0
```

RS 默认使用 SocketCAN。启动 ROS2 驱动前先拉起 CAN 接口：

```bash
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

---

## 配置开发环境

### Step 1. 安装 ROS2 依赖

请参考[ROS官方下载文档](https://www.ros.org/blog/getting-started/)选择适合的版本进行安装。

### Step 2. 获取 ROS2 源码

优先使用 Seeed-Projects 官方仓库：

```bash
git clone https://github.com/Seeed-Projects/reBotArmController_ROS2.git rebotarm_ros2
cd rebotarm_ros2
```

也可以使用当前开发仓库：

```bash
git clone https://github.com/EclipseaHime017/reBotArmController_ROS2.git rebotarm_ros2
cd rebotarm_ros2
```

### Step 3. 安装 motorbridge

`motorbridge` 从 PyPI 官方源安装：

```bash
python3 -m pip install --user --index-url https://pypi.org/simple motorbridge
```

### Step 4. 获取底层 SDK


```bash
mkdir -p third_party
git clone https://github.com/vectorBH6/reBotArm_control_py.git third_party/reBotArm_control_py
```

## 构建工作空间

```bash
colcon build --symlink-install
source install/setup.bash
```

验证包和入口：

```bash
ros2 pkg executables rebotarmcontroller
```

期望输出：

```text
rebotarmcontroller reBotArmController
rebotarmcontroller GravityCompensation
rebotarmcontroller GripperControl
rebotarmcontroller MoveTo
rebotarmcontroller MoveToPose
```

---

## 目录结构

```text
rebotarm_ros2/
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

## 快速启动

在正式开始使用机械臂前请注意： **机械臂的控制器具有较高自由度，启用控制器或者给机械臂上电前务必注意械臂工作空间内无人和障碍物。同时，请严格审查每一次对机械臂的运动控制，避免出现意外。严禁危险操作，造成后果自负。**

### 启动完整系统

启动控制节点、`robot_state_publisher`，可选 RViz：

```bash
ros2 launch rebotarm_bringup bringup.launch.py
```

`reBotArmController` 启动时会直接连接真实硬件。默认型号由
`rebotarm_bringup/config/rebotarm_hardware.yaml` 中的 `default_model` 决定，
当前为 `dm`。

注意，如果使用 RS 版本的机械臂且未修改`default_model`时，需要显式传入 `model:=rs`，并指定 `channel:=can0`。
```bash
ros2 launch rebotarm_bringup bringup.launch.py channel:={/dev/实际的串口名称}
ros2 launch rebotarm_bringup bringup.launch.py model:=rs channel:=can0
```

如果需要在 RViz 中可视化机械臂运动，可以启用 RViz：

```bash
ros2 launch rebotarm_bringup bringup.launch.py use_rviz:=true
```

### 只启动控制节点

和上述```bringup.launch.py```不同，如果希望最小化机械臂控制操作而不启用多余的`robot_state_publisher`
或者 RViz 可视化，那么可以直接启动：

```bash
ros2 launch rebotarm_bringup driver.launch.py
```

### 直接运行控制节点

```bash
ros2 run rebotarmcontroller reBotArmController
```

需要注意的是，和```driver.launch.py```从```rebotarm_bringup/config```不同，直接运行控制节点会从 SDK 加载
默认机械臂参数，因此更推荐通过 ros launch 的方式去启动节点。

---

## 直接移动到 Pose

不运行 demo 时，可以直接调用 ROS service 和 action 完成一次末端位姿移动。
先在一个终端启动控制节点：

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM0
```

然后在另一个终端执行控制命令：

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
```

1. 使能机械臂：

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger
```

2. 移动末端到目标 pose：

```bash
ros2 action send_goal /rebotarm/move_to_pose rebotarm_msgs/action/MoveToPose \
  "{target_pose: {position: {x: 0.30, y: 0.0, z: 0.30}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, duration: 2.0}"
```

`move_to_pose` action 通过 SDK 末端控制器执行。机械臂控制模式由
`rebotarm_hardware.yaml` 决定：DM 默认 `posvel`，RS 默认 `mit`。

3. 闭合夹爪并回到安全零位：

```bash
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger
```

4. 失能并退出：

```bash
ros2 service call /rebotarm/disable std_srvs/srv/Trigger
```

---

## 示例脚本

所有示例都假设已经启动 `reBotArmController`：

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM0
```

示例已注册为 ROS2 可执行入口，可以直接通过 `ros2 run` 调用。

源文件位于：

```text
src/rebotarmcontroller/rebotarmcontroller/examples/move_to.py
src/rebotarmcontroller/rebotarmcontroller/examples/move_to_pose.py
src/rebotarmcontroller/rebotarmcontroller/examples/gravity_compensation.py
src/rebotarmcontroller/rebotarmcontroller/examples/gripper_control.py
```

### move_to.py

关节空间绝对角移动示例。一次性控制 6 个电机，参数为 6 个绝对关节角，单位 rad：

```bash
ros2 run rebotarmcontroller MoveTo -- \
  0.20 -0.20 -0.20 -0.20 0.10 -0.10 \
  --duration 8.0
```

一次性控制 1 个电机，参数为目标关节名和绝对关节角，单位 rad：

```bash
ros2 run rebotarmcontroller MoveTo -- --joint joint3 --position -0.20 --duration 5.0
```

### move_to_pose.py

末端位姿移动示例。

```bash
ros2 run rebotarmcontroller MoveToPose -- --x 0.30 --y 0.0 --z 0.30 --qw 1.0 --duration 2.0
```

### gravity_compensation.py

重力补偿示例。

```bash
ros2 run rebotarmcontroller GravityCompensation
```

脚本启动时会先调用 `/rebotarm/enable`，再启动重力补偿。按 `Ctrl+C` 退出时，
脚本会依次调用 `/rebotarm/gravity_compensation/stop`、`/rebotarm/safe_home`
和 `/rebotarm/disable`，让机械臂回到安全零位后失能。

对应底层服务：

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger
ros2 service call /rebotarm/gravity_compensation/start std_srvs/srv/Trigger
ros2 service call /rebotarm/gravity_compensation/stop std_srvs/srv/Trigger
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger
ros2 service call /rebotarm/disable std_srvs/srv/Trigger
```

### gripper_control.py

交互式夹爪开闭示例。

```bash
ros2 run rebotarmcontroller GripperControl
```

运行后输入：

```text
o / open    打开夹爪
c / close   闭合夹爪
q / quit    退出
```
---

## API 文档

完整 ROS2 API 已整理到独立文档：[API_zh.md](API_zh.md)。

其中包含：

- topic、service、action 的完整列表和类型
- `/rebotarm` 命名空间、QoS、单位和状态机约定
- `JointMitCmd`、`JointPosVelCmd`、`ArmStatus`、`MoveToPose` 等自定义接口说明
- 末端位姿移动、夹爪控制、重力补偿、低层 command 的调用示例
- 上层集成和多机械臂命名注意事项

---

## 配置说明

`rebotarm_bringup/config/` 提供默认配置：

| 文件 | 说明 |
|---|---|
| `rebotarm_hardware.yaml` | ROS2 上层硬件选择和覆盖配置，可选择 DM / RS 并覆盖 SDK 参数 |
| `driver_params.yaml` | ROS 参数示例 |

常用 launch 参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `hardware_config` | bringup 内置 `rebotarm_hardware.yaml` | ROS2 上层硬件配置路径 |
| `model` | 空字符串 | 留空使用 `default_model`，可设为 `dm` 或 `rs` |
| `channel` | 空字符串 | 留空使用 YAML，非空时覆盖通信通道 |
| `joint_state_rate` | `100.0` | `/rebotarm/joint_states` 发布频率 |
| `cmd_arbitration` | `reject` | 轨迹运行中 arm joint 低层 cmd 仲裁，`reject` 或 `preempt`；gripper 低层 cmd 不抢占 arm 轨迹 |
| `arm_namespace` | `rebotarm` | ROS 命名空间前缀 |
| `frame_id` | `base_link` | 机械臂基座坐标系，预留给 TF、视觉和规划集成 |
| `ee_frame_id` | `end_link` | 末端坐标系，预留给 TF、视觉和规划集成 |
| `use_rviz` | `false` | 是否启动 bringup RViz |
| `disable_after_safe_home` | `true` | 该参数控制 safe home 完成后是否失能电机 |

`rebotarm_hardware.yaml` 默认型号配置：

| 型号 | 默认通道 | arm 控制模式 | 夹爪限位 |
|---|---|---|---|
| `dm` | `/dev/ttyACM0` | `posvel` | open `-5.0`，close `0.0` |
| `rs` | `can0` | `mit` | open `5.0`，close `0.0` |

---

## MoveIt 2

MoveIt 2 是用于机械臂运动规划的框架，这里主要负责逆解、碰撞检测、轨迹规划和轨迹执行，
并通过独立的 demo 包将应用流程与底层驱动隔离开。
更多内容可参考官方 [MoveIt 2 文档](https://moveit.picknik.ai/main/index.html)。

MoveIt 相关内容集中在两个包：

| 包 | 作用 |
|---|---|
| `rebotarm_moveit_config` | 机械臂模型、SRDF、运动学、joint limits、controller 和 RViz 配置 |
| `rebotarm_moveit_demos` | 基于 MoveIt 2 的应用 demo |

MoveIt 环境使用 `ros2_control` 的模拟硬件和 `move_group` 进行规划执行，适合在 RViz
中验证模型、IK、轨迹规划和 demo 流程。

本仓库同样提供了硬件接口的支持。接入真实硬件前，请先确认机械臂零点配置、关节方向、限位、
速度和夹爪开闭范围相关配置准确或者保持仓库默认配置。

### MoveIt 环境配置

先确认已经加载 ROS2 环境。下面的命令会使用当前 `ROS_DISTRO` 安装对应版本依赖：

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

MoveIt 相关包和 demo 已包含在本工作空间中，安装依赖后重新构建：

```bash
cd your/path/to/rebotarm_ros2
colcon build --symlink-install
source install/setup.bash
```

验证 MoveIt 包和 demo 入口：

```bash
ros2 pkg list | grep rebotarm_moveit
ros2 pkg executables rebotarm_moveit_demos
```

期望至少能看到如下两个可执行 Demo：

```text
rebotarm_moveit_demos draw_square
rebotarm_moveit_demos pick_place
```

### 使用 MoveIt

MoveIt 的规划功能需要基于 RViz GUI 或者通过节点调用，可以适用于仿真或真实场景。

#### 在仿真环境使用 MoveIt

MoveIt 通过 ros2_control 虚拟硬件接口实现 RViz 中的仿真，首先启用

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_moveit_config demo.launch.py
```
注意 RS 版本机械臂在没有修改默认类型机械臂的情况下需要指定：

```bash
ros2 launch rebotarm_moveit_config demo.launch.py model:=rs
```

默认会启动：

- `move_group`
- `robot_state_publisher`
- `ros2_control_node`
- `joint_state_broadcaster`
- `rebotarm_controller`
- `gripper_controller`
- RViz MoveIt MotionPlanning 插件

RViz 界面会自动弹出并加载机械臂的urdf模型，可以通过左侧的 GUI 控制面板对机械臂的运动进行控制。

#### 使用 MoveIt 控制 reBotArm

在实际场景中使用 MoveIt 控制 reBotArm 需要先启动带有硬件接口的控制器而不再是虚拟控制器，
再启动针对实际场景的 MoveIt 环境：

```bash
ros2 launch rebotarm_bringup driver.launch.py
```

另开终端：

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_moveit_config hardware.launch.py
```

再次重申，在真实硬件上运行任何 demo 前，请确保机械臂工作空间内无人和障碍物，先在 RViz 中确认规划路径，并随时准备停止控制器。

### 运行画矩形 demo

先启动 MoveIt 环境，再另开一个终端运行：

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_moveit_demos draw_square.launch.py
```

`draw_square` 会控制 `gripper_tcp` 遍历同一平面矩形的四个角点。默认参数在：

```text
src/rebotarm_moveit_demos/config/draw_square.yaml
src/rebotarm_moveit_demos/config/draw_square_rs.yaml
```

常用参数：

| 参数 | 说明 |
|---|---|
| `start_point` | demo 开始前复位到的关节位置 |
| `rectangle_center` | 矩形中心点，坐标系为 `base_link` |
| `rectangle_width` / `rectangle_height` | 矩形宽高，单位 m |
| `tcp_rpy` | 末端姿态，默认让夹爪竖直朝下 |
| `tcp_yaw_offsets` | IK 备选 yaw，用于避免 joint6 大幅绕转 |

### 运行抓取放置 demo

先启动 MoveIt 环境，再另开一个终端运行：

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_moveit_demos pick_place.launch.py
```

默认参数在：

```text
src/rebotarm_moveit_demos/config/pick_place.yaml
src/rebotarm_moveit_demos/config/pick_place_rs.yaml
```

常用参数：

| 参数 | 说明 |
|---|---|
| `ready_point` | 抓取前后使用的预备关节位置 |
| `pick_position` | 物体底面中心位置，坐标系为 `base_link` |
| `pick_tcp_rpy` / `place_tcp_rpy` | 抓取和放置时的末端姿态 |
| `object_dimensions` | MoveIt 场景中物体尺寸，单位 m |
| `max_gripper_width` | 夹爪最大总开口，默认 `0.09m` |
| `open_gripper_position` / `grasp_gripper_position` / `closed_gripper_position` | 仿真夹爪单侧关节位置 |
| `hardware_open_gripper_position` / `hardware_closed_gripper_position` | 硬件夹爪电机开闭位置 |

### MoveIt 配置文件

| 文件 | 说明 |
|---|---|
| `rebotarm_moveit_config/config/rebotarm.urdf.xacro` | MoveIt 使用的机器人模型 |
| `rebotarm_moveit_config/config/rebotarm.srdf` | MoveIt group、end effector、默认状态等语义配置 |
| `rebotarm_moveit_config/config/rebotarm_rs.urdf.xacro` | MoveIt 使用的 RS 机器人模型 |
| `rebotarm_moveit_config/config/rebotarm_rs.srdf` | RS MoveIt group 和碰撞语义配置 |
| `rebotarm_moveit_config/config/kinematics.yaml` | IK solver 配置 |
| `rebotarm_moveit_config/config/joint_limits.yaml` | MoveIt 规划使用的关节限位 |
| `rebotarm_moveit_config/config/moveit_controllers.yaml` | DM/RS 共用的 MoveIt trajectory execution controller 配置 |
| `rebotarm_moveit_config/config/ros2_controllers.yaml` | DM/RS 共用的 ros2_control controller 配置 |
| `rebotarm_moveit_config/config/initial_positions.yaml` | ros2_control 模拟硬件初始关节位置 |
| `rebotarm_moveit_demos/config/draw_square.yaml` | 画矩形 demo 参数 |
| `rebotarm_moveit_demos/config/draw_square_rs.yaml` | RS 画矩形 demo 参数 |
| `rebotarm_moveit_demos/config/pick_place.yaml` | 抓取放置 demo 参数 |
| `rebotarm_moveit_demos/config/pick_place_rs.yaml` | RS 抓取放置 demo 参数 |

---

## FAQ / 排障

### `ros2: command not found`

当前终端没有 ROS2 环境，或者尚未安装 ROS2。先安装 ROS2，然后在每个新终端中加载对应发行版环境：

```bash
source /opt/ros/humble/setup.bash
```

如果使用 Jazzy，将 `humble` 替换为 `jazzy`。

如果希望新终端自动加载 ROS2 环境，可以把 source 命令写入 `~/.bashrc`：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 找不到 ROS2 可执行文件

如果 `ros2 run` 或 `ros2 launch` 找不到 package 或 executable，通常是当前终端没有加载本工作空间。
需要先构建，再 source 工作空间：

```bash
cd your/path/to/rebotarm_ros2
colcon build --symlink-install
source install/setup.bash
```

可以用下面的命令检查入口是否已经注册：

```bash
ros2 pkg executables rebotarmcontroller
ros2 pkg executables rebotarm_moveit_demos
```

### 找不到串口

如果启动时报：

```text
open serial port /dev/ttyACM0 failed: No such file or directory
```

说明默认串口不存在。先查看实际设备：

```bash
ls /dev/ttyACM*
```

然后用 `channel:=...` 覆盖：

```bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM1
```

RS 需要确认 SocketCAN 接口已经启动：

```bash
ip -details link show can0
sudo ip link set can0 up type can bitrate 1000000
ros2 launch rebotarm_bringup bringup.launch.py model:=rs channel:=can0
```

### 权限不足

如果串口存在但无权限：

```bash
sudo usermod -a -G dialout $USER
```

重新登录后生效。

### RViz 模型不显示

确认 URDF mesh 路径已经是：

```text
package://rebotarm_bringup/description/...
```

### FastDDS SHM 端口提示

如果终端出现类似：

```text
[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7002: open_and_lock_file failed
```

通常是之前的 ROS2 进程异常退出后，FastDDS shared memory 锁文件残留。服务和 action
能正常响应时，这个提示一般不影响控制。需要清理时，先停掉相关 ROS2 进程，再执行：

```bash
pkill -f ros2
pkill -f reBotArmController
rm -f /dev/shm/fastrtps_port*
```

如果希望临时绕开 shared memory transport，可在启动 ROS2 前设置：

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```
