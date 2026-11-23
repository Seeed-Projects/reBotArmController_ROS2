from __future__ import annotations

from contextlib import nullcontext
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import mujoco
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


_ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 7))
_LEGACY_VISUAL_OPEN_M = 0.0285
_MUJOCO_GRIPPER_OPEN_M = 0.05
_GRASP_SCENE_NAME = "rs_grasp_scene.xml"
_SOURCE_SCENE = Path(__file__).resolve().parents[1] / "models" / _GRASP_SCENE_NAME
_DEFAULT_OBJECTS = ("red_cube", "blue_block", "yellow_cylinder")


def find_default_model() -> Path:
    try:
        installed = Path(get_package_share_directory("rebotarm_mujoco_rs")) / "models" / _GRASP_SCENE_NAME
        if installed.is_file():
            return installed
    except PackageNotFoundError:
        pass
    if _SOURCE_SCENE.is_file():
        return _SOURCE_SCENE
    raise FileNotFoundError("Bundled B601-RS MuJoCo scene not found. Build rebotarm_mujoco_rs or set model_path.")


def load_model(path: Path) -> mujoco.MjModel:
    # The bundled scene and arm XML live in rebotarm_mujoco_rs. The meshes
    # already installed by rebotarm_bringup are shared with its RS URDF.
    path = path.resolve()
    if path.name != _GRASP_SCENE_NAME or path not in (
        _SOURCE_SCENE.resolve(), find_default_model().resolve()
    ):
        return mujoco.MjModel.from_xml_path(str(path))
    mesh_dir = path.parents[2] / "rebotarm_bringup" / "description" / "RS" / "meshes"
    if not mesh_dir.is_dir():
        mesh_dir = Path(get_package_share_directory("rebotarm_bringup")) / "description" / "RS" / "meshes"
    arm = ET.parse(path.with_name("rs_arm.xml"))
    files = {mesh.attrib["file"] for mesh in arm.findall("./asset/mesh")}
    assets = {f"meshes/{name}": (mesh_dir / name).read_bytes() for name in files}
    return mujoco.MjModel.from_xml_path(str(path), assets)


class RsMujocoSync(Node):
    """Run RS MuJoCo dynamics and synchronize them with ROS joint targets."""

    def __init__(self) -> None:
        super().__init__("rebotarm_rs_mujoco")

        self.declare_parameter("arm_namespace", "rebotarm_rs")
        self.declare_parameter("input_topic", "")
        self.declare_parameter("output_topic", "")
        self.declare_parameter("model_path", "")
        self.declare_parameter("simulation_mode", "kinematic")
        self.declare_parameter("update_rate", 250.0)
        self.declare_parameter("smoothing_alpha", 1.0)
        self.declare_parameter("stale_timeout", 1.0)
        self.declare_parameter("use_viewer", False)
        self.declare_parameter("object_names", list(_DEFAULT_OBJECTS))
        self.declare_parameter("object_publish_rate", 30.0)
        self.declare_parameter("arm_kp", [80.0, 100.0, 100.0, 35.0, 25.0, 18.0])
        self.declare_parameter("arm_kd", [8.0, 10.0, 10.0, 4.0, 3.0, 2.5])
        self.declare_parameter("gripper_kp", 300.0)
        self.declare_parameter("gripper_kd", 120.0)
        self.declare_parameter("gripper_tau_limit", 150.0)

        namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        input_topic = str(self.get_parameter("input_topic").value).strip()
        output_topic = str(self.get_parameter("output_topic").value).strip()
        input_topic = input_topic or f"/{namespace}/joint_states"
        output_topic = output_topic or f"/{namespace}/mujoco/joint_states"

        requested_path = str(self.get_parameter("model_path").value).strip()
        self.model_path = (
            Path(requested_path).expanduser().resolve()
            if requested_path
            else find_default_model()
        )
        if not self.model_path.is_file():
            raise FileNotFoundError(f"MuJoCo model not found: {self.model_path}")

        self.simulation_mode = str(
            self.get_parameter("simulation_mode").value
        ).strip().lower()
        if self.simulation_mode not in ("kinematic", "physics"):
            raise ValueError("simulation_mode must be 'kinematic' or 'physics'")

        self.update_rate = max(float(self.get_parameter("update_rate").value), 1.0)
        self.smoothing_alpha = float(
            np.clip(self.get_parameter("smoothing_alpha").value, 0.01, 1.0)
        )
        self.stale_timeout = max(
            float(self.get_parameter("stale_timeout").value), 0.0
        )
        self.arm_kp = self._vector_parameter("arm_kp")
        self.arm_kd = self._vector_parameter("arm_kd")
        self.arm_tau_limit = np.array([36.0, 36.0, 36.0, 14.0, 14.0, 14.0])
        self.gripper_kp = float(self.get_parameter("gripper_kp").value)
        self.gripper_kd = float(self.get_parameter("gripper_kd").value)
        self.gripper_tau_limit = float(self.get_parameter("gripper_tau_limit").value)

        self.model = load_model(self.model_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.arm_joint_ids = np.array(
            [self._required_id(mujoco.mjtObj.mjOBJ_JOINT, name) for name in _ARM_JOINTS]
        )
        self.arm_qpos_addrs = self.model.jnt_qposadr[self.arm_joint_ids]
        self.arm_dof_addrs = self.model.jnt_dofadr[self.arm_joint_ids]
        self.arm_actuator_ids = np.array(
            [
                self._required_id(mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_motor")
                for name in _ARM_JOINTS
            ]
        )
        self.gripper_joint_id = self._required_id(
            mujoco.mjtObj.mjOBJ_JOINT, "joint7"
        )
        self.gripper_qpos_addr = int(self.model.jnt_qposadr[self.gripper_joint_id])
        self.gripper_dof_addr = int(self.model.jnt_dofadr[self.gripper_joint_id])
        self.gripper_actuator_id = self._required_id(
            mujoco.mjtObj.mjOBJ_ACTUATOR, "joint7_motor"
        )
        self.left_joint_id = self._required_id(
            mujoco.mjtObj.mjOBJ_JOINT, "joint_left"
        )
        self.right_joint_id = self._required_id(
            mujoco.mjtObj.mjOBJ_JOINT, "joint_right"
        )
        self.left_qpos_addr = int(self.model.jnt_qposadr[self.left_joint_id])
        self.right_qpos_addr = int(self.model.jnt_qposadr[self.right_joint_id])
        self.left_dof_addr = int(self.model.jnt_dofadr[self.left_joint_id])
        self.right_dof_addr = int(self.model.jnt_dofadr[self.right_joint_id])
        self.object_names = [
            str(name) for name in self.get_parameter("object_names").value
        ]
        self.object_body_ids = {
            name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            for name in self.object_names
        }
        self.object_body_ids = {
            name: body_id
            for name, body_id in self.object_body_ids.items()
            if body_id >= 0
        }
        self.object_publish_period = 1.0 / max(
            float(self.get_parameter("object_publish_rate").value), 1.0
        )

        self.target_arm = self.data.qpos[self.arm_qpos_addrs].copy()
        self.target_gripper = 0.0
        self.last_input_time = 0.0
        self.last_publish_time = 0.0
        self.last_object_publish_time = 0.0

        self.publisher = self.create_publisher(
            JointState,
            output_topic,
            qos_profile_sensor_data,
        )
        self.object_publisher = self.create_publisher(
            String,
            f"/{namespace}/mujoco/object_states",
            10,
        )
        self.subscription = self.create_subscription(
            JointState,
            input_topic,
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        self.reset_service = self.create_service(
            Trigger,
            f"/{namespace}/mujoco/reset",
            self._reset,
        )

        self.viewer = None
        if bool(self.get_parameter("use_viewer").value):
            from mujoco import viewer as mujoco_viewer

            self.viewer = mujoco_viewer.launch_passive(self.model, self.data)

        self.timer = self.create_timer(1.0 / self.update_rate, self._update)
        self.get_logger().info(
            f"RS MuJoCo ready: mode={self.simulation_mode}, input={input_topic}, "
            f"output={output_topic}, model={self.model_path}"
        )

    def _vector_parameter(self, name: str) -> np.ndarray:
        values = np.asarray(self.get_parameter(name).value, dtype=np.float64)
        if values.shape != (6,):
            raise ValueError(f"{name} must contain 6 values")
        return values

    def _required_id(self, object_type, name: str) -> int:
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise ValueError(f"required MuJoCo object not found: {name}")
        return int(object_id)

    def _joint_state_callback(self, msg: JointState) -> None:
        values = dict(zip(msg.name, msg.position))
        for index, name in enumerate(_ARM_JOINTS):
            if name in values and np.isfinite(values[name]):
                self.target_arm[index] = float(values[name])

        if "gripper_joint1" in values:
            self.target_gripper = self._visual_to_mujoco_gripper(
                values["gripper_joint1"], _MUJOCO_GRIPPER_OPEN_M
            )
        elif "finger_left" in values:
            self.target_gripper = self._visual_to_mujoco_gripper(
                values["finger_left"], _LEGACY_VISUAL_OPEN_M
            )
        self.last_input_time = time.monotonic()

    @staticmethod
    def _visual_to_mujoco_gripper(position: float, visual_open: float) -> float:
        ratio = np.clip(float(position) / visual_open, 0.0, 1.0)
        return float(ratio * _MUJOCO_GRIPPER_OPEN_M)

    def _update(self) -> None:
        if self.last_input_time == 0.0:
            return
        if (
            self.stale_timeout > 0.0
            and time.monotonic() - self.last_input_time > self.stale_timeout
        ):
            return

        viewer_lock = self.viewer.lock() if self.viewer is not None else nullcontext()
        with viewer_lock:
            if self.simulation_mode == "kinematic":
                self._update_kinematic()
            else:
                self._update_physics()
            self._publish_state()
            self._publish_object_states()
        if self.viewer is not None:
            if self.viewer.is_running():
                self.viewer.sync()
            else:
                self.viewer.close()
                self.viewer = None

    def _update_kinematic(self) -> None:
        current = self.data.qpos[self.arm_qpos_addrs]
        next_arm = current + self.smoothing_alpha * (self.target_arm - current)
        self.data.qpos[self.arm_qpos_addrs] = next_arm
        self.data.qvel[self.arm_dof_addrs] = 0.0

        current_gripper = float(self.data.qpos[self.gripper_qpos_addr])
        next_gripper = current_gripper + self.smoothing_alpha * (
            self.target_gripper - current_gripper
        )
        self.data.qpos[self.gripper_qpos_addr] = next_gripper
        self.data.qpos[self.left_qpos_addr] = next_gripper
        self.data.qpos[self.right_qpos_addr] = next_gripper
        self.data.qvel[
            [self.gripper_dof_addr, self.left_dof_addr, self.right_dof_addr]
        ] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _update_physics(self) -> None:
        timestep = float(self.model.opt.timestep)
        steps = max(1, int(round((1.0 / self.update_rate) / timestep)))
        for _ in range(steps):
            q = self.data.qpos[self.arm_qpos_addrs]
            qd = self.data.qvel[self.arm_dof_addrs]
            gravity_bias = self.data.qfrc_bias[self.arm_dof_addrs]
            tau = gravity_bias + self.arm_kp * (self.target_arm - q) - self.arm_kd * qd
            self.data.ctrl[self.arm_actuator_ids] = np.clip(
                tau, -self.arm_tau_limit, self.arm_tau_limit
            )

            gripper_q = float(self.data.qpos[self.gripper_qpos_addr])
            gripper_qd = float(self.data.qvel[self.gripper_dof_addr])
            gripper_tau = self.gripper_kp * (
                self.target_gripper - gripper_q
            ) - self.gripper_kd * gripper_qd
            self.data.ctrl[self.gripper_actuator_id] = float(
                np.clip(gripper_tau, -self.gripper_tau_limit, self.gripper_tau_limit)
            )
            mujoco.mj_step(self.model, self.data)

    def _publish_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [*_ARM_JOINTS, "gripper_joint1", "gripper_joint2"]
        msg.position = [
            *[float(value) for value in self.data.qpos[self.arm_qpos_addrs]],
            float(self.data.qpos[self.left_qpos_addr]),
            float(self.data.qpos[self.right_qpos_addr]),
        ]
        msg.velocity = [
            *[float(value) for value in self.data.qvel[self.arm_dof_addrs]],
            float(self.data.qvel[self.left_dof_addr]),
            float(self.data.qvel[self.right_dof_addr]),
        ]
        msg.effort = [
            *[float(value) for value in self.data.qfrc_actuator[self.arm_dof_addrs]],
            0.0,
            0.0,
        ]
        self.publisher.publish(msg)
        self.last_publish_time = time.monotonic()

    def _publish_object_states(self) -> None:
        now = time.monotonic()
        if now - self.last_object_publish_time < self.object_publish_period:
            return
        objects = []
        for name, body_id in self.object_body_ids.items():
            objects.append(
                {
                    "name": name,
                    "position": [float(value) for value in self.data.xpos[body_id]],
                    "quaternion": [float(value) for value in self.data.xquat[body_id]],
                }
            )
        msg = String()
        msg.data = json.dumps(
            {"objects": objects, "simulation_mode": self.simulation_mode},
            separators=(",", ":"),
        )
        self.object_publisher.publish(msg)
        self.last_object_publish_time = now

    def _reset(self, _request, response):
        mujoco.mj_resetData(self.model, self.data)
        self.target_arm.fill(0.0)
        self.target_gripper = 0.0
        mujoco.mj_forward(self.model, self.data)
        response.success = True
        response.message = "RS MuJoCo reset"
        return response

    def destroy_node(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RsMujocoSync()
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
