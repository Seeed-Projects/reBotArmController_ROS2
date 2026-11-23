from __future__ import annotations

import json
import math
from pathlib import Path
import threading

import mujoco
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String

from .mujoco_sync import find_default_model, load_model


_ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 7))
_OBJECTS = ("red_cube", "blue_block", "yellow_cylinder")


class RsSceneCamera(Node):
    """Render the synchronized RS grasp scene through its overhead camera."""

    def __init__(self) -> None:
        super().__init__("rebotarm_rs_scene_camera")
        self.declare_parameter("arm_namespace", "rebotarm_rs")
        self.declare_parameter("model_path", "")
        self.declare_parameter("joint_state_topic", "")
        self.declare_parameter("object_states_topic", "")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("camera_name", "overhead_rgb")
        self.declare_parameter("frame_id", "overhead_rgb_frame")
        self.declare_parameter("width", 320)
        self.declare_parameter("height", 240)
        self.declare_parameter("publish_hz", 8.0)

        namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        requested_model = str(self.get_parameter("model_path").value).strip()
        self.model_path = (
            Path(requested_model).expanduser().resolve()
            if requested_model
            else find_default_model()
        )
        self.joint_state_topic = str(
            self.get_parameter("joint_state_topic").value
            or f"/{namespace}/mujoco/joint_states"
        )
        self.object_states_topic = str(
            self.get_parameter("object_states_topic").value
            or f"/{namespace}/mujoco/object_states"
        )
        self.image_topic = str(
            self.get_parameter("image_topic").value
            or f"/{namespace}/mujoco/overhead_rgb/image_raw"
        )
        self.camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
            or f"/{namespace}/mujoco/overhead_rgb/camera_info"
        )
        self.camera_name = str(self.get_parameter("camera_name").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.width = max(int(self.get_parameter("width").value), 64)
        self.height = max(int(self.get_parameter("height").value), 64)
        publish_hz = max(float(self.get_parameter("publish_hz").value), 0.5)

        self.model = load_model(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.joint_qpos = {
            name: self._joint_qpos_addr(name)
            for name in (*_ARM_JOINTS, "joint7", "joint_left", "joint_right")
        }
        self.object_qpos = {
            name: self._freejoint_qpos_addr(name) for name in _OBJECTS
        }
        self.camera_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name
        )
        if self.camera_id < 0:
            raise RuntimeError(f"MuJoCo camera {self.camera_name!r} was not found")

        self._lock = threading.RLock()
        self._renderer: mujoco.Renderer | None = None
        self._renderer_error_reported = False
        self.image_pub = self.create_publisher(
            Image, self.image_topic, qos_profile_sensor_data
        )
        self.camera_info_pub = self.create_publisher(
            CameraInfo, self.camera_info_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._joint_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String, self.object_states_topic, self._objects_callback, 10
        )
        self.create_timer(1.0 / publish_hz, self._render)
        self.get_logger().info(
            "RS overhead camera ready: "
            f"camera={self.camera_name}, image={self.image_topic}, "
            f"size={self.width}x{self.height}@{publish_hz:g}Hz"
        )

    def _joint_qpos_addr(self, name: str) -> int:
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise RuntimeError(f"MuJoCo joint {name!r} was not found")
        return int(self.model.jnt_qposadr[joint_id])

    def _freejoint_qpos_addr(self, body_name: str) -> int:
        body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, body_name
        )
        if body_id < 0:
            raise RuntimeError(f"MuJoCo body {body_name!r} was not found")
        first = int(self.model.body_jntadr[body_id])
        count = int(self.model.body_jntnum[body_id])
        for joint_id in range(first, first + count):
            if int(self.model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_FREE):
                return int(self.model.jnt_qposadr[joint_id])
        raise RuntimeError(f"MuJoCo body {body_name!r} has no freejoint")

    def _joint_callback(self, msg: JointState) -> None:
        values = dict(zip(msg.name, msg.position))
        with self._lock:
            for name in _ARM_JOINTS:
                value = values.get(name)
                if value is not None:
                    self.data.qpos[self.joint_qpos[name]] = float(value)
            left = values.get("gripper_joint1")
            right = values.get("gripper_joint2")
            if left is not None:
                self.data.qpos[self.joint_qpos["joint_left"]] = float(left)
            if right is not None:
                self.data.qpos[self.joint_qpos["joint_right"]] = float(right)
            if left is not None or right is not None:
                positions = [float(value) for value in (left, right) if value is not None]
                self.data.qpos[self.joint_qpos["joint7"]] = sum(positions) / len(positions)

    def _objects_callback(self, msg: String) -> None:
        try:
            objects = json.loads(msg.data or "{}").get("objects", [])
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        with self._lock:
            for item in objects:
                if not isinstance(item, dict):
                    continue
                name = str(item.get("name", ""))
                position = item.get("position")
                quaternion = item.get("quaternion")
                if name not in self.object_qpos:
                    continue
                if not isinstance(position, list) or len(position) < 3:
                    continue
                if not isinstance(quaternion, list) or len(quaternion) < 4:
                    continue
                address = self.object_qpos[name]
                self.data.qpos[address : address + 3] = np.asarray(
                    position[:3], dtype=np.float64
                )
                quat = np.asarray(quaternion[:4], dtype=np.float64)
                norm = float(np.linalg.norm(quat))
                self.data.qpos[address + 3 : address + 7] = (
                    quat / norm if norm > 1e-8 else np.array([1.0, 0.0, 0.0, 0.0])
                )

    def _render(self) -> None:
        try:
            if self._renderer is None:
                self._renderer = mujoco.Renderer(
                    self.model, height=self.height, width=self.width
                )
            with self._lock:
                mujoco.mj_forward(self.model, self.data)
                self._renderer.update_scene(self.data, camera=self.camera_name)
                rgb = self._renderer.render().copy()
        except Exception as exc:
            if not self._renderer_error_reported:
                self.get_logger().error(f"RS overhead camera rendering failed: {exc}")
                self._renderer_error_reported = True
            return

        stamp = self.get_clock().now().to_msg()
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = self.frame_id
        image.height = self.height
        image.width = self.width
        image.encoding = "rgb8"
        image.is_bigendian = False
        image.step = self.width * 3
        image.data = rgb.astype(np.uint8, copy=False).tobytes()
        self.image_pub.publish(image)
        self.camera_info_pub.publish(self._camera_info(stamp))

    def _camera_info(self, stamp) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height
        fovy = math.radians(float(self.model.cam_fovy[self.camera_id]))
        focal = 0.5 * self.height / math.tan(0.5 * fovy)
        cx = 0.5 * (self.width - 1)
        cy = 0.5 * (self.height - 1)
        info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def destroy_node(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RsSceneCamera()
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
