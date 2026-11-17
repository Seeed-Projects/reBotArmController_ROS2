from __future__ import annotations

import functools
import math
import threading
import time

import numpy as np

from .conversions import fk_to_pose
from .hardware_config import resolve_hardware_config

_GRIPPER_GOAL_TOLERANCE_RAD = 0.12


def _locked(method):
    @functools.wraps(method)
    def wrapper(self, *args, **kwargs):
        with self._cmd_lock:
            return method(self, *args, **kwargs)

    return wrapper


class HardwareManager:
    """ROS-facing adapter for the new grouped reBotArm SDK."""

    def __init__(
        self,
        hardware_config: str | None = None,
        model: str = "",
        channel: str = "",
    ) -> None:
        self._cmd_lock = threading.RLock()
        hardware_config_path, hardware_data = resolve_hardware_config(
            hardware_config,
            model,
            channel,
        )

        from reBotArm_control_py.actuator import RebotArm
        from reBotArm_control_py.controllers import RebotArmEndPose
        from reBotArm_control_py.dynamics import compute_generalized_gravity
        from reBotArm_control_py.kinematics import (
            compute_fk,
            load_robot_model,
            pad_q_for_model,
        )

        self._compute_fk = compute_fk
        self._pad_q_for_model = pad_q_for_model

        runtime_config = hardware_data["_runtime"]
        self.model_name = str(runtime_config.get("model", "dm")).lower()
        control_runtime = runtime_config["control"]
        self._arm_control_mode = control_runtime["arm_control_mode"]

        self._robot = RebotArm(hw_yaml=str(hardware_config_path))
        self._arm_group = self._robot.groups.get("arm")
        if self._arm_group is None:
            raise ValueError("hardware config must define groups.arm")
        self._gripper_group = self._robot.groups.get("gripper")
        self._robot_get_state = self._robot.get_state
        self._robot.get_state = self._get_arm_state
        self._arm_mit_kp = np.array(control_runtime["mit_kp"], dtype=np.float64)
        self._arm_mit_kd = np.array(control_runtime["mit_kd"], dtype=np.float64)
        self._endpos_ctrl = RebotArmEndPose(
            self._robot,
            arm_control_mode=self._arm_control_mode,
        )

        self._gripper_name = (
            self._gripper_group.joint_names[0]
            if self.has_gripper and self._gripper_group.joint_names
            else ""
        )
        gripper_limits = hardware_data.get("gripper", {}).get("position_limits", {})
        self.gripper_open_position = float(gripper_limits.get("open", 0.0))
        self.gripper_close_position = float(gripper_limits.get("close", 0.0))
        self._configure_gripper_position_limits()
        self._gripper_target_position: float | None = None

        self._gc_model = load_robot_model()
        self._gc_data = self._gc_model.createData()
        self._gc_compute_generalized_gravity = compute_generalized_gravity
        gc_runtime = runtime_config["gravity_compensation"]
        self._gravity_comp_kp = np.array(gc_runtime["kp"], dtype=np.float64)
        self._gravity_comp_kd = np.array(gc_runtime["kd"], dtype=np.float64)
        self._gravity_comp_joint_direction = np.array(
            gc_runtime["joint_direction"],
            dtype=np.float64,
        )
        self._gravity_comp_tau_scale = np.array(
            gc_runtime["tau_scale"],
            dtype=np.float64,
        )

        self._connected = False
        self._enabled = False
        self._control_output_enabled = False
        self._state_machine = "IDLE"
        self._error_codes: list[str] = []
        self._gravity_comp_active = False
        self._gravity_comp_q_last: np.ndarray | None = None
        self._homing_thread: int | None = None

    # ------------------------------------------------------------------
    # state
    # ------------------------------------------------------------------

    @property
    def joint_names(self) -> list[str]:
        return list(self._arm_group.joint_names)

    @property
    def gripper_joint_names(self) -> list[str]:
        if self.model_name == "dm":
            return ["finger_left", "finger_right"]
        return ["gripper_joint1", "gripper_joint2"]

    def _configure_gripper_position_limits(self) -> None:
        open_position = self.gripper_open_position
        close_position = self.gripper_close_position
        if not all(math.isfinite(v) for v in (open_position, close_position)):
            raise ValueError("gripper position_limits open/close must be finite")

        if open_position == close_position:
            raise ValueError("gripper position_limits open and close must differ")

        self._gripper_position_lower = min(open_position, close_position)
        self._gripper_position_upper = max(open_position, close_position)

    def validate_gripper_position(self, position: float) -> float:
        target = float(position)
        if not math.isfinite(target):
            raise ValueError("gripper position must be finite")
        if not self._gripper_position_lower <= target <= self._gripper_position_upper:
            raise ValueError(
                f"gripper position {target:.3f} rad is outside safe range "
                f"[{self._gripper_position_lower:.3f}, "
                f"{self._gripper_position_upper:.3f}] rad"
            )
        return target

    @property
    def mode(self) -> str:
        return str(self._arm_group.mode)

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def control_loop_active(self) -> bool:
        return bool(self._robot.control_loop_active)

    @property
    def has_gripper(self) -> bool:
        return bool(self._robot.has_gripper)

    @property
    def state_machine(self) -> str:
        return self._state_machine

    @property
    def error_codes(self) -> list[str]:
        return list(self._error_codes)

    @_locked
    def set_state_machine(self, state: str) -> None:
        if state not in (
            "IDLE",
            "TRAJ_RUNNING",
            "LOWLEVEL_STREAMING",
            "GRAVITY_COMP",
            "SAFE_HOMING",
        ):
            raise ValueError(f"unsupported state machine value: {state}")
        self._state_machine = state

    # ------------------------------------------------------------------
    # arm
    # ------------------------------------------------------------------

    @_locked
    def connect(self) -> None:
        if self._connected:
            return
        try:
            self._robot.connect()
            if self.has_gripper:
                self._gripper_target_position = self.get_gripper_state()[0]
            self._start_endpos_loop()
            self._connected = True
            self._enabled = True
        except Exception:
            self._control_output_enabled = False
            self._endpos_ctrl._running = False
            try:
                self._robot.stop_control_loop()
                self._robot.disconnect()
            finally:
                self._connected = False
                self._enabled = False
            raise

    def shutdown(self, disable_after_safe_home: bool = True) -> None:
        if not self._connected:
            return
        try:
            # Do not re-enable motors that were already disabled by a client.
            if self._enabled:
                self.safe_home()
            with self._cmd_lock:
                self._robot.stop_control_loop()

            if disable_after_safe_home:
                self.disable()

            with self._cmd_lock:
                self._robot.disconnect()
        finally:
            self._connected = False
            self._enabled = False
            self._control_output_enabled = False
            self.set_state_machine("IDLE")

    def get_joint_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self._get_arm_state()

    def _get_arm_state(
        self,
        request_feedback: bool = True,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        pos, vel, torq = self._robot_get_state(request_feedback=request_feedback)
        n = len(self.joint_names)
        return pos[:n], vel[:n], torq[:n]

    def get_joint_positions(self, request: bool = False) -> np.ndarray:
        return self._arm_group.get_positions(request_feedback=request)

    def get_joint_velocities(self, request: bool = False) -> np.ndarray:
        return self._arm_group.get_velocities(request_feedback=request)

    @_locked
    def hold_current_position(self) -> np.ndarray:
        current = self.get_joint_positions(request=True).copy()
        if self._state_machine != "SAFE_HOMING":
            self._endpos_ctrl._q_target[:] = current
            self._endpos_ctrl._qd_target[:] = 0.0
        return current

    @_locked
    def set_joint_position_target(self, positions) -> None:
        if self._state_machine == "SAFE_HOMING":
            raise RuntimeError("rejecting joint target during safe home")
        target = np.asarray(positions, dtype=np.float64).reshape(-1)
        if len(target) != len(self.joint_names):
            raise ValueError(
                f"expected {len(self.joint_names)} joint targets, got {len(target)}"
            )
        self.stop_motion()
        self._endpos_ctrl._q_target[:] = target
        self._endpos_ctrl._qd_target[:] = 0.0

    @_locked
    def start_endpos_control(self) -> None:
        if self._gravity_comp_active:
            raise RuntimeError("stop gravity compensation before starting endpos control")
        if self._state_machine in ("SAFE_HOMING", "TRAJ_RUNNING"):
            raise RuntimeError(f"rejecting endpos control in state {self._state_machine}")

        if self.control_loop_active:
            self.set_state_machine("IDLE")
            return

        self._robot.stop_control_loop()
        self._start_endpos_loop()
        self._enabled = True
        self.set_state_machine("IDLE")

    def enable(self) -> None:
        self.start_endpos_control()

    @_locked
    def disable(self) -> None:
        if self._gravity_comp_active:
            raise RuntimeError("stop gravity compensation before disable")
        self._control_output_enabled = False
        self.stop_motion()
        self._endpos_ctrl._running = False
        self._robot.stop_control_loop()
        self._robot.disable_all()
        self._enabled = False
        self.set_state_machine("IDLE")

    def safe_home(self) -> None:
        with self._cmd_lock:
            self.stop_motion()
            self.set_state_machine("IDLE")
            if self._gravity_comp_active:
                if self._arm_control_mode == "mit" and self.control_loop_active:
                    hold_target = (
                        self._gravity_comp_q_last.copy()
                        if self._gravity_comp_q_last is not None
                        else self._arm_group.get_positions(request_feedback=False).copy()
                    )
                    self._arm_group._mit_kp = self._arm_mit_kp.copy()
                    self._arm_group._mit_kd = self._arm_mit_kd.copy()
                    self._endpos_ctrl._q_target[:] = hold_target
                    self._endpos_ctrl._qd_target[:] = 0.0
                    self._endpos_ctrl._running = True
                    self._control_output_enabled = True
                    self._send_endpos_hold_once()
                    self._robot._ctrl_fn = self._endpos_loop_cb
                    self._gravity_comp_active = False
                    self._gravity_comp_q_last = None
                else:
                    self.stop_gravity_compensation()
            else:
                self.start_endpos_control()
            self.set_state_machine("SAFE_HOMING")
            self._homing_thread = threading.get_ident()
        try:
            if self.has_gripper:
                self.set_gripper_position(self.gripper_close_position)
            self._endpos_ctrl.safe_home()
        finally:
            self._homing_thread = None
            self.set_state_machine("IDLE")

    @_locked
    def set_zero(self, joint_name: str = "") -> bool:
        self.stop_motion()
        self._robot.stop_control_loop()
        self._endpos_ctrl._running = False
        if joint_name:
            if joint_name not in self._robot._motor_map:
                raise KeyError(f"unknown joint: {joint_name}")
            self._set_zero_single(joint_name)
        else:
            self._robot.set_zero()
        self._enabled = False
        self.set_state_machine("IDLE")
        return True

    @_locked
    def send_joint_mit_cmd(
        self,
        joint_name: str,
        pos: float,
        vel: float,
        kp: float,
        kd: float,
        tau: float,
    ) -> None:
        index = self._joint_index(joint_name)
        self._begin_lowlevel_streaming("mit")
        q = self._arm_group.get_positions(request_feedback=True)
        target_pos = np.array(q, dtype=np.float64, copy=True)
        target_vel = np.zeros(len(self.joint_names), dtype=np.float64)
        target_tau = np.zeros(len(self.joint_names), dtype=np.float64)
        target_kp = np.array(self._arm_mit_kp, dtype=np.float64, copy=True)
        target_kd = np.array(self._arm_mit_kd, dtype=np.float64, copy=True)
        target_pos[index] = float(pos)
        target_vel[index] = float(vel)
        # When the web sends kp=0 or kd=0, keep the hardware's default gains
        # so the motor maintains PD control without the web needing to know
        # the per-joint MIT parameters.
        if kp != 0:
            target_kp[index] = float(kp)
        if kd != 0:
            target_kd[index] = float(kd)
        target_tau[index] = float(tau)
        self._arm_group.send_mit(
            target_pos,
            vel=target_vel,
            kp=target_kp,
            kd=target_kd,
            tau=target_tau,
        )
        self.set_state_machine("LOWLEVEL_STREAMING")

    @_locked
    def send_joint_pos_vel_cmd(
        self,
        joint_name: str,
        pos: float,
        vlim: float,
    ) -> None:
        index = self._joint_index(joint_name)
        self._begin_lowlevel_streaming("pos_vel")
        q = self._arm_group.get_positions(request_feedback=True)
        target_pos = np.array(q, dtype=np.float64, copy=True)
        target_vlim = np.array(
            getattr(self._arm_group, "_pv_vlim"), dtype=np.float64, copy=True
        )
        target_pos[index] = float(pos)
        target_vlim[index] = float(vlim)
        self._arm_group.send_pos_vel(target_pos, vlim=target_vlim)
        self.set_state_machine("LOWLEVEL_STREAMING")

    def current_pose(self):
        q, _, _ = self.get_joint_state()
        q_padded = self._pad_q_for_model(self._gc_model, q, len(self.joint_names))
        position, rotation, _ = self._compute_fk(self._gc_model, q_padded)
        return fk_to_pose(position, rotation)

    def _require_idle(self, what: str) -> None:
        state = self._state_machine
        if state in ("TRAJ_RUNNING", "GRAVITY_COMP", "SAFE_HOMING"):
            raise RuntimeError(f"rejecting {what} in state {state}")

    @_locked
    def begin_trajectory_stream(self) -> None:
        self._require_idle("trajectory stream")
        self.start_endpos_control()
        self.set_state_machine("TRAJ_RUNNING")

    def move_to_pose_traj(
        self, x, y, z, roll, pitch, yaw, duration: float
    ) -> bool:
        self.begin_trajectory_stream()
        ok = False
        try:
            ok = bool(
                self._endpos_ctrl.move_to_traj(x, y, z, roll, pitch, yaw, duration)
            )
        finally:
            if not ok and self._state_machine == "TRAJ_RUNNING":
                self.set_state_machine("IDLE")
        return ok

    @_locked
    def move_to_pose_ik(self, x, y, z, roll, pitch, yaw) -> tuple[bool, list[float]]:
        self._require_idle("IK target")
        self.start_endpos_control()
        ok = self._endpos_ctrl.move_to_ik(x, y, z, roll, pitch, yaw)
        return bool(ok), [float(v) for v in self._endpos_ctrl._q_target]

    def get_joint_status_codes(self) -> list[int]:
        codes: list[int] = []
        for name in self.joint_names:
            try:
                st = self._robot._motor_map[name].get_state()
                codes.append(int(st.status_code if st is not None else 0))
            except Exception:
                codes.append(0)
        return codes

    # ------------------------------------------------------------------
    # gravity compensation
    # ------------------------------------------------------------------

    @_locked
    def start_gravity_compensation(self) -> None:
        if self._state_machine in ("TRAJ_RUNNING", "SAFE_HOMING"):
            raise RuntimeError(
                f"rejecting gravity compensation in state {self._state_machine}"
            )
        if self._gravity_comp_active:
            return
        if not self._enabled:
            self._arm_group.enable()
            if self.has_gripper:
                self._gripper_group.enable()
            self._enabled = True
        self.stop_motion()
        self._robot.stop_control_loop()
        self._endpos_ctrl._running = False

        self._gravity_comp_q_last = self._arm_group.get_positions(
            request_feedback=True,
        ).copy()
        # MIT gains are supplied by every gravity-compensation command. When
        # the groups are already in MIT mode, keep the motors enabled instead
        # of reconfiguring the same mode and cycling their power state.
        mode_changed = False
        if self._arm_group.mode != "mit":
            self._arm_group.mode_mit(
                kp=self._gravity_comp_kp,
                kd=self._gravity_comp_kd,
            )
            mode_changed = True
        if self.has_gripper and self._gripper_group.mode != "mit":
            self._gripper_group.mode_mit()
            mode_changed = True
        if mode_changed:
            # Preserve the existing transition for position/velocity hardware.
            self._robot.disable_all()
            time.sleep(0.1)
            self._robot.enable_all()
        self._enabled = True
        self._gravity_comp_active = True
        arm_rate = float(getattr(self._robot, "_rate", 500.0))
        self._gravity_comp_tick(self._robot, 1.0 / arm_rate)
        self._robot.start_control_loop(self._gravity_comp_tick, rate=arm_rate)
        self.set_state_machine("GRAVITY_COMP")

    @_locked
    def stop_gravity_compensation(self) -> None:
        if not self._gravity_comp_active:
            return
        hold_target = (
            self._gravity_comp_q_last.copy()
            if self._gravity_comp_q_last is not None
            else None
        )
        self._robot.stop_control_loop()
        self._gravity_comp_active = False
        self._gravity_comp_q_last = None
        if self._enabled:
            self._start_endpos_hold(target=hold_target)
        self.set_state_machine("IDLE")

    def gravity_compensation_active(self) -> bool:
        return self._gravity_comp_active

    @staticmethod
    def _angles_near_reference(values: np.ndarray, reference: np.ndarray) -> np.ndarray:
        delta = values - reference
        delta = (delta + np.pi) % (2.0 * np.pi) - np.pi
        return reference + delta

    def _read_gravity_comp_positions(
        self,
        *,
        request: bool = False,
        reference: np.ndarray | None = None,
    ) -> np.ndarray:
        q = self._arm_group.get_positions(request_feedback=request)
        ref = reference if reference is not None else self._gravity_comp_q_last
        if ref is not None:
            q = self._angles_near_reference(q, ref)
        self._gravity_comp_q_last = np.array(q, dtype=np.float64, copy=True)
        return self._gravity_comp_q_last.copy()

    def _gravity_comp_tick(self, _robot, dt: float) -> None:
        del dt
        if not self._gravity_comp_active:
            return

        q = self._read_gravity_comp_positions(request=False)
        q_for_model = q * self._gravity_comp_joint_direction
        q_model = self._pad_q_for_model(
            self._gc_model, q_for_model, len(self.joint_names)
        )
        tau_model = self._gc_compute_generalized_gravity(
            self._gc_model,
            q_model,
            self._gc_data,
        )[: len(self.joint_names)]
        tau_motor = (
            tau_model
            * self._gravity_comp_joint_direction
            * self._gravity_comp_tau_scale
        )

        self._arm_group.send_mit(
            q,
            vel=np.zeros(len(self.joint_names)),
            kp=self._gravity_comp_kp,
            kd=self._gravity_comp_kd,
            tau=tau_motor,
        )
        if self.has_gripper:
            self._gripper_group.send_mit(
                self._gripper_group.get_positions()
            )

    # ------------------------------------------------------------------
    # gripper
    # ------------------------------------------------------------------

    @_locked
    def set_gripper_target(self, position: float) -> None:
        target = self.validate_gripper_position(position)
        self._begin_gripper_command(allow_endpos=True)
        self._endpos_ctrl.set_gripper_target(target)
        self._gripper_group.send_mit(
            np.array([target], dtype=np.float64),
            kp=getattr(self._gripper_group, "_mit_kp"),
            kd=getattr(self._gripper_group, "_mit_kd"),
        )
        self._gripper_target_position = target

    def wait_gripper_target(self, timeout: float = 3.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.gripper_reached_target():
                return True
            time.sleep(0.02)
        return False

    def set_gripper_position(
        self,
        position: float,
        timeout: float = 3.0,
    ) -> tuple[bool, float]:
        self.set_gripper_target(position)
        reached = self.wait_gripper_target(timeout)
        return reached, self.get_gripper_state()[0]

    def get_gripper_state(self) -> tuple[float, float, float, int]:
        if not self.has_gripper or not self._gripper_name:
            return 0.0, 0.0, 0.0, 0
        pos = float(self._gripper_group.get_positions()[0])
        vel = float(self._gripper_group.get_velocities(request_feedback=False)[0])
        status = 0
        torque = 0.0
        try:
            st = self._robot._motor_map[self._gripper_name].get_state()
            if st is not None:
                torque = float(st.torq)
                status = int(st.status_code)
        except Exception:
            status = 0
        return float(pos), float(vel), float(torque), status

    def gripper_reached_target(self) -> bool:
        if self._gripper_target_position is None:
            return True
        pos = self.get_gripper_state()[0]
        return abs(pos - self._gripper_target_position) < _GRIPPER_GOAL_TOLERANCE_RAD

    @_locked
    def send_gripper_mit_cmd(
        self,
        pos: float,
        vel: float,
        kp: float,
        kd: float,
        tau: float,
    ) -> None:
        pos = self.validate_gripper_position(pos)
        self._begin_gripper_command()
        self._begin_gripper_lowlevel("mit")
        # When the web sends kp=0 or kd=0, pass None so the JointGroup
        # falls back to its configured MIT gains (from the SDK config).
        gripper_kp = np.array([float(kp)], dtype=np.float64) if kp != 0 else None
        gripper_kd = np.array([float(kd)], dtype=np.float64) if kd != 0 else None
        self._gripper_group.send_mit(
            np.array([float(pos)], dtype=np.float64),
            vel=np.array([float(vel)], dtype=np.float64),
            kp=gripper_kp,
            kd=gripper_kd,
            tau=np.array([float(tau)], dtype=np.float64),
        )
        self._gripper_target_position = None

    @_locked
    def send_gripper_pos_vel_cmd(self, pos: float, vlim: float) -> None:
        pos = self.validate_gripper_position(pos)
        self._begin_gripper_command()
        self._begin_gripper_lowlevel("pos_vel")
        self._gripper_group.send_pos_vel(
            np.array([float(pos)], dtype=np.float64),
            vlim=np.array([float(vlim)], dtype=np.float64),
        )
        self._gripper_target_position = None

    def _begin_gripper_command(self, *, allow_endpos: bool = False) -> None:
        if not self._enabled:
            raise RuntimeError("rejecting gripper command while arm is disabled")
        if self._gravity_comp_active or self.state_machine == "GRAVITY_COMP":
            raise RuntimeError("rejecting gripper command during gravity compensation")
        if (
            self.state_machine == "SAFE_HOMING"
            and threading.get_ident() != self._homing_thread
        ):
            raise RuntimeError("rejecting gripper command during safe home")
        if self.state_machine == "TRAJ_RUNNING":
            raise RuntimeError("rejecting gripper command while trajectory is running")
        if not self.has_gripper or not self._gripper_name:
            raise RuntimeError("gripper is not initialized")
        if not allow_endpos and self.control_loop_active:
            self.stop_motion()
            self._robot.stop_control_loop()
            self._endpos_ctrl._running = False

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    def _begin_lowlevel_streaming(self, required_mode: str) -> None:
        if not self._enabled:
            raise RuntimeError("rejecting low-level command while arm is disabled")
        if self._gravity_comp_active or self.state_machine == "GRAVITY_COMP":
            raise RuntimeError("rejecting low-level command during gravity compensation")
        if self.state_machine == "SAFE_HOMING":
            raise RuntimeError("rejecting low-level command during safe home")
        if self.state_machine == "TRAJ_RUNNING":
            self.stop_motion()
        self._robot.stop_control_loop()
        self._endpos_ctrl._running = False

        if required_mode != self.mode:
            self._enter_mode(
                self._arm_group,
                required_mode,
                "arm",
                kp=self._arm_mit_kp,
                kd=self._arm_mit_kd,
            )
        self.set_state_machine("LOWLEVEL_STREAMING")

    def _begin_gripper_lowlevel(self, required_mode: str) -> None:
        self._enter_mode(self._gripper_group, required_mode, "gripper")
        self.set_state_machine("LOWLEVEL_STREAMING")

    @staticmethod
    def _enter_mode(group, required_mode: str, label: str, **mit_gains) -> None:
        if required_mode == "mit":
            ok = group.mode_mit(**mit_gains)
        elif required_mode == "pos_vel":
            ok = group.mode_pos_vel()
        else:
            raise ValueError(f"unsupported low-level mode: {required_mode}")
        if not ok:
            raise RuntimeError(f"{label} did not enter {required_mode} mode")

    def _start_endpos_hold(self, target: np.ndarray | None = None) -> None:
        if self.control_loop_active:
            return
        self._start_endpos_loop(target)

    def _start_endpos_loop(self, target: np.ndarray | None = None) -> None:
        self._configure_groups_for_endpos()
        if target is None:
            self.hold_current_position()
        else:
            self._endpos_ctrl._q_target[:] = np.asarray(target, dtype=np.float64)
            self._endpos_ctrl._qd_target[:] = 0.0
        self._control_output_enabled = True
        self._robot.start_control_loop(self._endpos_loop_cb)
        self._endpos_ctrl._running = True

    def _configure_groups_for_endpos(self) -> None:
        if self._arm_control_mode == "mit":
            self._arm_group.mode_mit(
                kp=self._arm_mit_kp,
                kd=self._arm_mit_kd,
            )
        else:
            self._arm_group.mode_pos_vel()
        self._arm_group.enable()
        if self.has_gripper:
            if not self._gripper_group.mode_mit():
                raise RuntimeError("gripper did not enter mit mode")
            self._gripper_group.enable()

    @_locked
    def stop_motion(self) -> None:
        self._endpos_ctrl._stop_send.set()
        if self._endpos_ctrl._send_thread is not None:
            self._endpos_ctrl._send_thread.join(timeout=5.0)
        self._endpos_ctrl._moving = False
        self._endpos_ctrl._stop_send.clear()

    def motion_active(self) -> bool:
        return bool(self._endpos_ctrl._moving)

    def _endpos_loop_cb(self, robot, dt: float) -> None:
        del dt
        if not self._cmd_lock.acquire(blocking=False):
            return
        try:
            if not self._control_output_enabled:
                return
            self._endpos_ctrl._loop_cb(robot, 0.0)
        finally:
            self._cmd_lock.release()

    def _send_endpos_hold_once(self) -> None:
        if self._arm_control_mode == "mit":
            self._arm_group.send_mit(
                self._endpos_ctrl._q_target,
                vel=self._endpos_ctrl._qd_target,
                kp=getattr(self._arm_group, "_mit_kp"),
                kd=getattr(self._arm_group, "_mit_kd"),
            )
        else:
            self._arm_group.send_pos_vel(
                self._endpos_ctrl._q_target,
                vlim=getattr(self._arm_group, "_pv_vlim"),
            )

    def _joint_index(self, joint_name: str) -> int:
        try:
            return self.joint_names.index(joint_name)
        except ValueError as exc:
            raise KeyError(f"unknown joint: {joint_name}") from exc

    def _set_zero_single(self, joint_name: str) -> None:
        self._robot.disable_all()
        time.sleep(0.3)
        motor = self._robot._motor_map[joint_name]
        ctrl = None
        for joint in self._robot._all_joints:
            if joint.name == joint_name:
                ctrl = self._robot._ctrl_map[str(joint.vendor)]
                break
        if ctrl is None:
            raise KeyError(f"unknown joint: {joint_name}")
        for _ in range(200):
            try:
                motor.request_feedback()
                ctrl.poll_feedback_once()
            except Exception:
                pass
            st = motor.get_state()
            if st is not None and st.status_code == 0:
                break
            time.sleep(0.05)
        motor.set_zero_position()
