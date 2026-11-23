"""Gravity compensation transitions using mocked motor/transport boundaries."""
import threading
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest

from rebotarmcontroller.hardware_manager import HardwareManager


def hardware(mode='mit', enabled=True, gripper=True):
    obj=HardwareManager.__new__(HardwareManager)
    obj._cmd_lock=threading.RLock()
    obj._state_machine='IDLE'
    obj._enabled=enabled
    obj._gravity_comp_active=False
    obj._gravity_comp_q_last=None
    obj._robot=Mock(has_gripper=gripper, _rate=500.)
    obj._endpos_ctrl=SimpleNamespace(_running=True)
    obj._arm_group=Mock(mode=mode)
    obj._arm_group.get_positions.return_value=np.zeros(6)
    obj._gripper_group=Mock(mode='mit') if gripper else None
    obj._gravity_comp_kp=np.ones(6)*2.
    obj._gravity_comp_kd=np.ones(6)
    obj.stop_motion=Mock()
    obj._gravity_comp_tick=Mock()
    return obj


@pytest.mark.parametrize('gripper',[False,True])
def test_enabled_mit_enters_gravity_without_enable_disable_cycle(gripper):
    obj=hardware(gripper=gripper)
    obj.start_gravity_compensation()
    obj._robot.disable_all.assert_not_called()
    obj._robot.enable_all.assert_not_called()
    obj._arm_group.enable.assert_not_called()
    obj._arm_group.mode_mit.assert_not_called()
    if gripper:
        obj._gripper_group.enable.assert_not_called()
        obj._gripper_group.mode_mit.assert_not_called()
    assert obj._state_machine=='GRAVITY_COMP'
    assert obj._gravity_comp_active
    obj._gravity_comp_tick.assert_called_once()
    obj._robot.start_control_loop.assert_called_once()


def test_repeated_gravity_start_keeps_existing_loop():
    obj=hardware()
    obj.start_gravity_compensation()
    obj._robot.reset_mock()
    obj._arm_group.reset_mock()
    obj.start_gravity_compensation()
    obj._robot.stop_control_loop.assert_not_called()
    obj._robot.start_control_loop.assert_not_called()
    obj._robot.disable_all.assert_not_called()
    obj._arm_group.enable.assert_not_called()


def test_disabled_mit_is_enabled_without_disable_cycle():
    obj=hardware(enabled=False)
    obj.start_gravity_compensation()
    obj._arm_group.enable.assert_called_once()
    obj._gripper_group.enable.assert_called_once()
    obj._robot.disable_all.assert_not_called()
    obj._robot.enable_all.assert_not_called()
    assert obj.enabled


def test_position_velocity_mode_keeps_existing_mode_switch_sequence():
    obj=hardware(mode='pos_vel')
    with patch('rebotarmcontroller.hardware_manager.time.sleep'):
        obj.start_gravity_compensation()
    obj._arm_group.mode_mit.assert_called_once()
    obj._robot.disable_all.assert_called_once()
    obj._robot.enable_all.assert_called_once()
    obj._robot.start_control_loop.assert_called_once()


@pytest.mark.parametrize('state',['TRAJ_RUNNING','SAFE_HOMING'])
def test_busy_state_is_rejected_before_motor_operations(state):
    obj=hardware()
    obj._state_machine=state
    with pytest.raises(RuntimeError):
        obj.start_gravity_compensation()
    obj._robot.stop_control_loop.assert_not_called()
    obj._arm_group.enable.assert_not_called()
