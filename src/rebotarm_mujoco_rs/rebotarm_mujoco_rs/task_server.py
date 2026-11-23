from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
import math
import threading
import time

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
import mujoco
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rebotarm_msgs.action import MoveToPose
from rebotarm_msgs.msg import JointPosVelCmd
from rebotarm_msgs.srv import MoveToPoseIK
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from .mujoco_sync import find_default_model, load_model


_ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 7))


@dataclass(frozen=True)
class ArmJoint:
    name: str
    qpos_addr: int
    dof_addr: int
    lower: float
    upper: float

    def clamp(self, value: float) -> float:
        return float(np.clip(value, self.lower, self.upper))


class RsTaskServer(Node):
    """RS Cartesian IK, smooth joint trajectories, and task recording."""

    def __init__(self) -> None:
        super().__init__("rebotarm_rs_task_server")
        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter("arm_namespace", "rebotarm_rs")
        self.declare_parameter("model_path", "")
        self.declare_parameter("joint_state_topic", "")
        self.declare_parameter("target_pose_topic", "")
        self.declare_parameter("tcp_body", "gripper_end")
        self.declare_parameter("tcp_offset", [-0.04, 0.0, 0.0])
        self.declare_parameter("command_hz", 60.0)
        self.declare_parameter("record_hz", 30.0)
        self.declare_parameter("max_joint_speed", 1.0)
        self.declare_parameter("ik_tolerance", 0.006)
        self.declare_parameter("ik_orientation_tolerance", 0.06)
        self.declare_parameter("ik_orientation_weight", 0.6)
        self.declare_parameter("ik_max_nfev", 600)
        self.declare_parameter("records_dir", "")

        self.namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self.joint_state_topic = str(
            self.get_parameter("joint_state_topic").value
            or f"/{self.namespace}/joint_states"
        )
        self.target_pose_topic = str(
            self.get_parameter("target_pose_topic").value
            or f"/{self.namespace}/mujoco/target_pose"
        )
        self.command_hz = max(float(self.get_parameter("command_hz").value), 10.0)
        self.record_hz = max(float(self.get_parameter("record_hz").value), 1.0)
        self.max_joint_speed = max(
            float(self.get_parameter("max_joint_speed").value), 0.05
        )
        self.ik_tolerance = max(float(self.get_parameter("ik_tolerance").value), 0.001)
        self.ik_orientation_tolerance = max(
            float(self.get_parameter("ik_orientation_tolerance").value), 0.01
        )
        self.ik_orientation_weight = max(
            float(self.get_parameter("ik_orientation_weight").value), 0.0
        )
        self.ik_max_nfev = max(int(self.get_parameter("ik_max_nfev").value), 100)
        records_dir = str(self.get_parameter("records_dir").value).strip()
        self.records_dir = (
            Path(records_dir).expanduser()
            if records_dir
            else Path.home() / ".ros" / "rebotarm_rs_records"
        )

        requested_model = str(self.get_parameter("model_path").value).strip()
        self.model_path = (
            Path(requested_model).expanduser().resolve()
            if requested_model
            else find_default_model()
        )
        self.model = load_model(self.model_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.base_qpos = self.data.qpos.copy()
        self.joints = self._build_joints()
        self.lower = np.array([joint.lower for joint in self.joints], dtype=np.float64)
        self.upper = np.array([joint.upper for joint in self.joints], dtype=np.float64)

        tcp_body_name = str(self.get_parameter("tcp_body").value)
        self.tcp_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, tcp_body_name
        )
        if self.tcp_body_id < 0:
            raise RuntimeError(f"MuJoCo body {tcp_body_name!r} was not found")
        self.tcp_offset = np.asarray(
            self.get_parameter("tcp_offset").value, dtype=np.float64
        )
        if self.tcp_offset.shape != (3,):
            raise ValueError("tcp_offset must contain three values")

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.command_pubs = {
            name: self.create_publisher(
                JointPosVelCmd,
                f"/{self.namespace}/joints/{name}/cmd/pos_vel",
                reliable,
            )
            for name in _ARM_JOINTS
        }
        self.target_pose_pub = self.create_publisher(
            PoseStamped, self.target_pose_topic, reliable
        )

        self._state_lock = threading.RLock()
        self._ik_lock = threading.Lock()
        self._current_q = np.zeros(6, dtype=np.float64)
        self._last_target = np.zeros(3, dtype=np.float64)
        self._cancel = threading.Event()
        self._recording = False
        self._record_started = 0.0
        self._record_samples: list[dict[str, float]] = []

        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._joint_state_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.create_timer(
            1.0 / self.record_hz,
            self._record_timer,
            callback_group=self.callback_group,
        )
        self.create_service(
            MoveToPoseIK,
            f"/{self.namespace}/move_to_pose_ik",
            self._handle_ik,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            f"/{self.namespace}/mujoco/record/start",
            self._record_start,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            f"/{self.namespace}/mujoco/record/stop",
            self._record_stop,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            f"/{self.namespace}/mujoco/record/replay",
            self._record_replay,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            f"/{self.namespace}/mujoco/record/clear",
            self._record_clear,
            callback_group=self.callback_group,
        )

        self.move_server = ActionServer(
            self,
            MoveToPose,
            f"/{self.namespace}/move_to_pose",
            execute_callback=self._execute_move,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self.callback_group,
        )
        self.trajectory_server = ActionServer(
            self,
            FollowJointTrajectory,
            f"/{self.namespace}/follow_joint_trajectory",
            execute_callback=self._execute_trajectory,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self.callback_group,
        )
        self.get_logger().info(
            "RS task server ready: "
            f"namespace=/{self.namespace}, model={self.model_path}, "
            f"tcp_offset={self.tcp_offset.tolist()}"
        )

    def _build_joints(self) -> list[ArmJoint]:
        result = []
        for name in _ARM_JOINTS:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise RuntimeError(f"MuJoCo joint {name!r} was not found")
            result.append(
                ArmJoint(
                    name=name,
                    qpos_addr=int(self.model.jnt_qposadr[joint_id]),
                    dof_addr=int(self.model.jnt_dofadr[joint_id]),
                    lower=float(self.model.jnt_range[joint_id, 0]),
                    upper=float(self.model.jnt_range[joint_id, 1]),
                )
            )
        return result

    def _joint_state_callback(self, msg: JointState) -> None:
        values = dict(zip(msg.name, msg.position))
        with self._state_lock:
            for index, name in enumerate(_ARM_JOINTS):
                value = values.get(name)
                if value is not None and math.isfinite(float(value)):
                    self._current_q[index] = self.joints[index].clamp(float(value))

    @staticmethod
    def _accept_goal(_request):
        return GoalResponse.ACCEPT

    def _accept_cancel(self, _goal_handle):
        self._cancel.set()
        return CancelResponse.ACCEPT

    def _handle_ik(self, request, response):
        target = self._pose_xyz(request.target_pose)
        target_mat = self._pose_matrix(request.target_pose)
        solution, pos_error, orientation_error = self._solve_ik(
            target, self._current_positions(), target_mat
        )
        response.q_solution = solution.tolist()
        response.success = bool(
            pos_error <= self.ik_tolerance
            and (
                target_mat is None
                or orientation_error <= self.ik_orientation_tolerance
            )
        )
        response.message = (
            f"RS IK {'success' if response.success else 'best effort'}, "
            f"position={pos_error * 1000.0:.1f} mm, orientation={orientation_error:.4f}"
        )
        return response

    def _execute_move(self, goal_handle):
        request = goal_handle.request
        target = self._pose_xyz(request.target_pose)
        target_mat = self._pose_matrix(request.target_pose)
        start = self._current_positions()
        solution, pos_error, orientation_error = self._solve_ik(target, start, target_mat)
        result = MoveToPose.Result()
        if pos_error > self.ik_tolerance or (
            target_mat is not None
            and orientation_error > self.ik_orientation_tolerance
        ):
            goal_handle.abort()
            result.success = False
            result.message = (
                f"RS IK failed: position={pos_error * 1000.0:.1f} mm, "
                f"orientation={orientation_error:.4f}"
            )
            result.final_pose = self._fk_pose_msg(start)
            return result

        duration = self._safe_duration(start, solution, float(request.duration))
        ok = self._run_path(
            [start, solution],
            [0.0, duration],
            goal_handle=goal_handle,
            move_feedback=True,
        )
        result.final_pose = self._fk_pose_msg(self._current_positions())
        if not ok:
            goal_handle.canceled()
            result.success = False
            result.message = "RS Cartesian motion canceled"
            return result
        goal_handle.succeed()
        result.success = True
        result.message = f"RS Cartesian motion complete, IK error={pos_error * 1000.0:.1f} mm"
        return result

    def _execute_trajectory(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        result = FollowJointTrajectory.Result()
        try:
            positions, times = self._expand_trajectory(
                list(trajectory.joint_names), list(trajectory.points)
            )
        except ValueError as exc:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            return result

        ok = self._run_path(positions, times, goal_handle=goal_handle)
        if not ok:
            goal_handle.canceled()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "trajectory canceled"
            return result
        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "RS trajectory complete"
        return result

    def _expand_trajectory(
        self, names: list[str], points: list[JointTrajectoryPoint]
    ) -> tuple[list[np.ndarray], list[float]]:
        if not names or not points:
            raise ValueError("trajectory requires joint_names and points")
        if len(names) != len(set(names)):
            raise ValueError("trajectory joint_names contain duplicates")
        if not set(names).issubset(_ARM_JOINTS):
            raise ValueError("trajectory contains unknown RS joints")

        path = [self._current_positions()]
        times = [0.0]
        for point in points:
            if len(point.positions) != len(names):
                raise ValueError("trajectory positions length does not match joint_names")
            next_q = path[-1].copy()
            for name, value in zip(names, point.positions):
                index = _ARM_JOINTS.index(name)
                next_q[index] = self.joints[index].clamp(float(value))
            requested = float(point.time_from_start.sec) + float(
                point.time_from_start.nanosec
            ) / 1e9
            minimum = times[-1] + self._safe_duration(path[-1], next_q, 0.0)
            path.append(next_q)
            times.append(max(requested, minimum))
        return path, times

    def _run_path(
        self,
        path: list[np.ndarray],
        times: list[float],
        *,
        goal_handle=None,
        move_feedback: bool = False,
    ) -> bool:
        if len(path) < 2 or len(path) != len(times):
            return True
        self._cancel.clear()
        period = 1.0 / self.command_hz
        started = time.monotonic()
        total = max(times[-1], period)
        for segment in range(1, len(path)):
            q0, q1 = path[segment - 1], path[segment]
            t0, t1 = times[segment - 1], max(times[segment], times[segment - 1] + period)
            while True:
                if self._cancel.is_set() or (
                    goal_handle is not None and goal_handle.is_cancel_requested
                ):
                    return False
                elapsed = time.monotonic() - started
                ratio = float(np.clip((elapsed - t0) / (t1 - t0), 0.0, 1.0))
                smooth = ratio * ratio * (3.0 - 2.0 * ratio)
                q = q0 + (q1 - q0) * smooth
                self._publish_targets(q)
                if move_feedback and goal_handle is not None:
                    feedback = MoveToPose.Feedback()
                    feedback.current_pose = self._fk_pose_msg(q)
                    feedback.progress = float(np.clip(elapsed / total, 0.0, 1.0))
                    feedback.time_elapsed = elapsed
                    goal_handle.publish_feedback(feedback)
                if ratio >= 1.0:
                    break
                time.sleep(period)
        with self._state_lock:
            self._current_q = path[-1].copy()
        return True

    def _publish_targets(self, q: np.ndarray) -> None:
        stamp = self.get_clock().now().to_msg()
        for index, name in enumerate(_ARM_JOINTS):
            msg = JointPosVelCmd()
            msg.pos = self.joints[index].clamp(float(q[index]))
            msg.vlim = self.max_joint_speed
            msg.stamp = stamp
            self.command_pubs[name].publish(msg)

    def _solve_ik(
        self,
        target: np.ndarray,
        start: np.ndarray,
        target_mat: np.ndarray | None,
    ) -> tuple[np.ndarray, float, float]:
        with self._ik_lock:
            seeds = self._ik_seeds(start, target)
            best: tuple[float, np.ndarray, float, float] | None = None
            for seed in seeds:
                try:
                    solved = least_squares(
                        self._ik_residual,
                        seed,
                        args=(target, target_mat),
                        bounds=(self.lower, self.upper),
                        max_nfev=self.ik_max_nfev,
                        xtol=1e-9,
                        ftol=1e-9,
                        gtol=1e-9,
                    )
                except (ValueError, np.linalg.LinAlgError):
                    continue
                q = np.clip(solved.x, self.lower, self.upper)
                position, rotation = self._fk_pose(q)
                pos_error = float(np.linalg.norm(target - position))
                orientation_error = (
                    0.0
                    if target_mat is None
                    else float(np.linalg.norm(self._orientation_error(rotation, target_mat)))
                )
                distance = float(np.linalg.norm((q - start) / np.maximum(self.upper - self.lower, 0.1)))
                score = pos_error + 0.25 * orientation_error + 0.002 * distance
                candidate = (score, q.copy(), pos_error, orientation_error)
                if best is None or candidate[0] < best[0]:
                    best = candidate
                if pos_error <= self.ik_tolerance and (
                    target_mat is None
                    or orientation_error <= self.ik_orientation_tolerance
                ) and distance < 0.8:
                    break
            if best is None:
                return start.copy(), float("inf"), float("inf")
            return best[1], best[2], best[3]

    def _ik_seeds(self, start: np.ndarray, target: np.ndarray) -> list[np.ndarray]:
        seeds = [
            start.copy(),
            np.array([0.0, 1.6, 1.2, -0.8, 0.0, 0.0]),
            np.array([0.2, 1.2, 0.5, 0.0, 0.5, -2.5]),
            np.array([-0.2, 1.2, 0.5, 0.0, -0.5, 2.5]),
            np.array([0.0, 2.0, 1.7, -1.3, 0.0, 0.0]),
        ]
        seed_value = int(abs(float(target[0] * 997 + target[1] * 577 + target[2] * 313)) * 1e6)
        rng = np.random.default_rng(seed_value)
        for _ in range(3):
            seeds.append(self.lower + rng.random(6) * (self.upper - self.lower))
        epsilon = 1e-5
        return [np.clip(seed, self.lower + epsilon, self.upper - epsilon) for seed in seeds]

    def _ik_residual(
        self, q: np.ndarray, target: np.ndarray, target_mat: np.ndarray | None
    ) -> np.ndarray:
        position, rotation = self._fk_pose(q)
        position_error = target - position
        if target_mat is None or self.ik_orientation_weight <= 0.0:
            return position_error
        return np.concatenate(
            [
                position_error,
                self.ik_orientation_weight
                * self._orientation_error(rotation, target_mat),
            ]
        )

    def _fk_pose(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        self.data.qpos[:] = self.base_qpos
        self.data.qvel[:] = 0.0
        for index, joint in enumerate(self.joints):
            self.data.qpos[joint.qpos_addr] = joint.clamp(float(q[index]))
        mujoco.mj_forward(self.model, self.data)
        rotation = self.data.xmat[self.tcp_body_id].reshape(3, 3).copy()
        position = self.data.xpos[self.tcp_body_id].copy() + rotation @ self.tcp_offset
        return position, rotation

    def _fk_pose_msg(self, q: np.ndarray) -> Pose:
        position, rotation = self._fk_pose(q)
        quat = np.zeros(4, dtype=np.float64)
        mujoco.mju_mat2Quat(quat, rotation.reshape(9))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position.tolist()
        pose.orientation.w = float(quat[0])
        pose.orientation.x = float(quat[1])
        pose.orientation.y = float(quat[2])
        pose.orientation.z = float(quat[3])
        return pose

    def _record_timer(self) -> None:
        with self._state_lock:
            if not self._recording:
                return
            sample = {"t": time.monotonic() - self._record_started}
            sample.update({name: float(value) for name, value in zip(_ARM_JOINTS, self._current_q)})
            self._record_samples.append(sample)

    def _record_start(self, _request, response):
        with self._state_lock:
            self._record_samples = []
            self._record_started = time.monotonic()
            self._recording = True
        response.success = True
        response.message = "RS task recording started"
        return response

    def _record_stop(self, _request, response):
        with self._state_lock:
            self._recording = False
            samples = list(self._record_samples)
        if not samples:
            response.success = False
            response.message = "No RS task samples recorded"
            return response
        self.records_dir.mkdir(parents=True, exist_ok=True)
        path = self.records_dir / f"rs_record_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=["t", *_ARM_JOINTS])
            writer.writeheader()
            writer.writerows(samples)
        response.success = True
        response.message = f"Saved {len(samples)} RS samples to {path}"
        return response

    def _record_replay(self, _request, response):
        with self._state_lock:
            samples = list(self._record_samples)
            self._recording = False
        if len(samples) < 2:
            response.success = False
            response.message = "Need at least two RS samples to replay"
            return response
        path = [np.array([sample[name] for name in _ARM_JOINTS]) for sample in samples]
        times = [float(sample["t"]) for sample in samples]
        threading.Thread(target=self._run_path, args=(path, times), daemon=True).start()
        response.success = True
        response.message = f"Replaying {len(samples)} RS samples"
        return response

    def _record_clear(self, _request, response):
        self._cancel.set()
        with self._state_lock:
            self._recording = False
            self._record_samples = []
        response.success = True
        response.message = "RS task recording cleared"
        return response

    def _current_positions(self) -> np.ndarray:
        with self._state_lock:
            return self._current_q.copy()

    def _safe_duration(self, start: np.ndarray, target: np.ndarray, requested: float) -> float:
        required = float(np.max(np.abs(target - start))) / self.max_joint_speed
        return max(float(requested), required, 0.2)

    @staticmethod
    def _pose_xyz(pose: Pose) -> np.ndarray:
        return np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)

    @staticmethod
    def _pose_matrix(pose: Pose) -> np.ndarray | None:
        quat = np.array(
            [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z],
            dtype=np.float64,
        )
        norm = float(np.linalg.norm(quat))
        if norm < 1e-8:
            return None
        quat /= norm
        if np.allclose(quat, [1.0, 0.0, 0.0, 0.0], atol=1e-5):
            return None
        matrix = np.zeros(9, dtype=np.float64)
        mujoco.mju_quat2Mat(matrix, quat)
        return matrix.reshape(3, 3)

    @staticmethod
    def _orientation_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
        # The former cross-product approximation becomes exactly zero for a
        # 180-degree error.  That made an upside-down gripper look perfect to
        # the solver.  A rotation vector preserves both the error axis and the
        # full angle all the way to pi radians.
        relative = target @ current.T
        return Rotation.from_matrix(relative).as_rotvec()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RsTaskServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
