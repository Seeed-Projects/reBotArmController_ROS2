from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rebotarm_msgs.msg import (
    ArmStatus,
    JointMitCmd,
    JointMotorState,
    JointPosVelCmd,
)
from rebotarm_msgs.srv import GripperCommand, SetGripper
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


_JOINT_LIMITS = (
    (-2.8, 2.8),
    (0.0, 3.14),
    (0.0, 3.14),
    (-1.57, 1.57),
    (-1.57, 1.57),
    (-3.14, 3.14),
)
_GRIPPER_VISUAL_HALF_OPEN_M = 0.05


class FakeRsDriver(Node):
    """RS ROS interface simulator with no hardware or MuJoCo dependency."""

    def __init__(self) -> None:
        super().__init__("fake_rebotarm_rs_driver")

        self.declare_parameter("arm_namespace", "rebotarm_rs")
        self.declare_parameter("joint_state_rate", 100.0)
        self.declare_parameter("max_joint_speed", 1.0)
        self.declare_parameter("max_gripper_speed", 2.0)
        self.declare_parameter("gripper_open_position", 5.0)
        self.declare_parameter("start_enabled", True)

        self.namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self.joint_names = [f"joint{index}" for index in range(1, 7)]
        self.max_joint_speed = max(
            0.01, float(self.get_parameter("max_joint_speed").value)
        )
        self.max_gripper_speed = max(
            0.01, float(self.get_parameter("max_gripper_speed").value)
        )
        self.gripper_open_position = max(
            0.01, float(self.get_parameter("gripper_open_position").value)
        )
        self.enabled = bool(self.get_parameter("start_enabled").value)

        self.positions = [0.0] * 6
        self.targets = [0.0] * 6
        self.velocities = [0.0] * 6
        self.gripper_position = 0.0
        self.gripper_target = 0.0
        self.gripper_velocity = 0.0
        self.state_machine = "IDLE"
        self.last_time = self.get_clock().now()

        self.joint_state_pub = self.create_publisher(
            JointState,
            f"/{self.namespace}/joint_states",
            qos_profile_sensor_data,
        )
        self.joint_motor_pubs = [
            self.create_publisher(
                JointMotorState,
                f"/{self.namespace}/joints/{name}/state",
                qos_profile_sensor_data,
            )
            for name in self.joint_names
        ]
        self.gripper_state_pub = self.create_publisher(
            JointMotorState,
            f"/{self.namespace}/gripper/state",
            qos_profile_sensor_data,
        )
        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.status_pub = self.create_publisher(
            ArmStatus,
            f"/{self.namespace}/arm_status",
            status_qos,
        )

        command_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.command_subscriptions = []
        for index, name in enumerate(self.joint_names):
            self.command_subscriptions.append(
                self.create_subscription(
                    JointPosVelCmd,
                    f"/{self.namespace}/joints/{name}/cmd/pos_vel",
                    self._joint_pos_vel_callback(index),
                    command_qos,
                )
            )
            self.command_subscriptions.append(
                self.create_subscription(
                    JointMitCmd,
                    f"/{self.namespace}/joints/{name}/cmd/mit",
                    self._joint_mit_callback(index),
                    command_qos,
                )
            )
        self.command_subscriptions.extend(
            [
                self.create_subscription(
                    JointPosVelCmd,
                    f"/{self.namespace}/gripper/cmd/pos_vel",
                    self._gripper_pos_vel_callback,
                    command_qos,
                ),
                self.create_subscription(
                    JointMitCmd,
                    f"/{self.namespace}/gripper/cmd/mit",
                    self._gripper_mit_callback,
                    command_qos,
                ),
            ]
        )

        self._service_handles = [
            self.create_service(Trigger, f"/{self.namespace}/enable", self._enable),
            self.create_service(Trigger, f"/{self.namespace}/disable", self._disable),
            self.create_service(Trigger, f"/{self.namespace}/safe_home", self._safe_home),
            self.create_service(
                Trigger,
                f"/{self.namespace}/gravity_compensation/start",
                self._start_gravity_compensation,
            ),
            self.create_service(
                Trigger,
                f"/{self.namespace}/gravity_compensation/stop",
                self._stop_gravity_compensation,
            ),
            self.create_service(
                Trigger,
                f"/{self.namespace}/gravity_compensation/status",
                self._gravity_compensation_status,
            ),
            self.create_service(
                SetGripper,
                f"/{self.namespace}/gripper/set",
                self._set_gripper,
            ),
            self.create_service(
                GripperCommand,
                f"/{self.namespace}/gripper/open",
                self._open_gripper,
            ),
            self.create_service(
                GripperCommand,
                f"/{self.namespace}/gripper/close",
                self._close_gripper,
            ),
        ]

        rate = max(float(self.get_parameter("joint_state_rate").value), 1.0)
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.status_timer = self.create_timer(0.5, self.publish_status)
        self.publish_status()
        self.get_logger().info(
            f"RS fake driver ready: namespace=/{self.namespace}, "
            "interface=JointPosVelCmd/JointMitCmd"
        )

    def _joint_pos_vel_callback(self, index: int):
        def callback(msg: JointPosVelCmd) -> None:
            self._set_joint_target(index, msg.pos)

        return callback

    def _joint_mit_callback(self, index: int):
        def callback(msg: JointMitCmd) -> None:
            self._set_joint_target(index, msg.pos)

        return callback

    def _set_joint_target(self, index: int, target: float) -> None:
        if not self.enabled or self.state_machine == "GRAVITY_COMP":
            return
        lower, upper = _JOINT_LIMITS[index]
        self.targets[index] = self._clamp(float(target), lower, upper)
        self.state_machine = "LOWLEVEL_STREAMING"
        self.publish_status()

    def _gripper_pos_vel_callback(self, msg: JointPosVelCmd) -> None:
        self._set_gripper_target(msg.pos)

    def _gripper_mit_callback(self, msg: JointMitCmd) -> None:
        self._set_gripper_target(msg.pos)

    def _set_gripper_target(self, target: float) -> None:
        if not self.enabled or self.state_machine == "GRAVITY_COMP":
            return
        self.gripper_target = self._clamp(
            float(target), 0.0, self.gripper_open_position
        )
        self.state_machine = "LOWLEVEL_STREAMING"
        self.publish_status()

    def _enable(self, _request, response):
        self.enabled = True
        self.state_machine = "IDLE"
        response.success = True
        response.message = "RS fake driver enabled"
        self.publish_status()
        return response

    def _disable(self, _request, response):
        self.enabled = False
        self.state_machine = "IDLE"
        response.success = True
        response.message = "RS fake driver disabled"
        self.publish_status()
        return response

    def _safe_home(self, _request, response):
        self.targets = [0.0] * 6
        self.gripper_target = 0.0
        self.state_machine = "LOWLEVEL_STREAMING"
        response.success = True
        response.message = "RS fake safe-home accepted"
        self.publish_status()
        return response

    def _start_gravity_compensation(self, _request, response):
        self.targets = list(self.positions)
        self.gripper_target = self.gripper_position
        self.state_machine = "GRAVITY_COMP"
        response.success = True
        response.message = "RS fake gravity compensation active"
        self.publish_status()
        return response

    def _stop_gravity_compensation(self, _request, response):
        self.state_machine = "IDLE"
        response.success = True
        response.message = "RS fake gravity compensation stopped"
        self.publish_status()
        return response

    def _gravity_compensation_status(self, _request, response):
        response.success = self.state_machine == "GRAVITY_COMP"
        response.message = self.state_machine
        return response

    def _set_gripper(self, request, response):
        self._set_gripper_target(request.position)
        response.success = self.enabled
        response.reached_position = float(self.gripper_target)
        return response

    def _open_gripper(self, request, response):
        target = request.position if request.position != 0.0 else self.gripper_open_position
        self._set_gripper_target(target)
        response.success = self.enabled
        response.reached_position = float(self.gripper_target)
        response.message = "RS fake gripper open accepted"
        return response

    def _close_gripper(self, request, response):
        self._set_gripper_target(request.position)
        response.success = self.enabled
        response.reached_position = float(self.gripper_target)
        response.message = "RS fake gripper close accepted"
        return response

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = max((now - self.last_time).nanoseconds / 1e9, 0.001)
        self.last_time = now
        moving = False

        for index in range(6):
            before = self.positions[index]
            self.positions[index] = self._step_towards(
                before, self.targets[index], self.max_joint_speed * dt
            )
            self.velocities[index] = (self.positions[index] - before) / dt
            moving = moving or abs(self.targets[index] - self.positions[index]) > 0.002

        before_gripper = self.gripper_position
        self.gripper_position = self._step_towards(
            before_gripper,
            self.gripper_target,
            self.max_gripper_speed * dt,
        )
        self.gripper_velocity = (self.gripper_position - before_gripper) / dt
        moving = moving or abs(self.gripper_target - self.gripper_position) > 0.01

        if not moving and self.state_machine == "LOWLEVEL_STREAMING":
            self.state_machine = "IDLE"
            self.publish_status()
        self._publish_states(now)

    def _publish_states(self, now) -> None:
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = list(self.joint_names)
        msg.position = list(self.positions)
        msg.velocity = list(self.velocities)
        msg.effort = [0.0] * 6

        ratio = self.gripper_position / self.gripper_open_position
        visual_position = ratio * _GRIPPER_VISUAL_HALF_OPEN_M
        visual_velocity = (
            self.gripper_velocity
            / self.gripper_open_position
            * _GRIPPER_VISUAL_HALF_OPEN_M
        )
        msg.name.extend(["gripper_joint1", "gripper_joint2"])
        msg.position.extend([visual_position, visual_position])
        msg.velocity.extend([visual_velocity, visual_velocity])
        msg.effort.extend([0.0, 0.0])
        self.joint_state_pub.publish(msg)

        for index, name in enumerate(self.joint_names):
            state = JointMotorState()
            state.header = msg.header
            state.joint_name = name
            state.position = float(self.positions[index])
            state.velocity = float(self.velocities[index])
            state.torque = 0.0
            state.status_code = 0
            self.joint_motor_pubs[index].publish(state)

        gripper_state = JointMotorState()
        gripper_state.header = msg.header
        gripper_state.joint_name = "gripper"
        gripper_state.position = float(self.gripper_position)
        gripper_state.velocity = float(self.gripper_velocity)
        gripper_state.torque = 0.0
        gripper_state.status_code = 0
        self.gripper_state_pub.publish(gripper_state)

    def publish_status(self) -> None:
        msg = ArmStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = "fake_rs_pos_vel"
        msg.enabled = self.enabled
        msg.control_loop_active = self.enabled
        msg.state_machine = self.state_machine
        msg.joint_names = list(self.joint_names)
        msg.per_joint_status_code = [0] * 6
        msg.error_codes = []
        self.status_pub.publish(msg)

    @staticmethod
    def _step_towards(current: float, target: float, max_step: float) -> float:
        delta = target - current
        if math.isclose(delta, 0.0, abs_tol=1e-9):
            return target
        return current + FakeRsDriver._clamp(delta, -max_step, max_step)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeRsDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
