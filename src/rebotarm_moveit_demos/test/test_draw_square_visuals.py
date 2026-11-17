from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

from rebotarm_moveit_demos.draw_square import DrawSquare


def demo():
    obj = DrawSquare.__new__(DrawSquare)
    obj.node = Mock()
    obj.result_timeout = 30.0
    params = {
        'gripper_action_name': '/gripper_controller/follow_joint_trajectory',
        'hardware_gripper_action_name': '/rebotarm/gripper/command',
        'gripper_joint_names': ['gripper_joint1', 'gripper_joint2'],
        'closed_gripper_position': 0.0,
        'hardware_closed_gripper_position': 0.0,
        'gripper_motion_duration': 1.0,
        'gripper_max_effort': 10.0,
    }
    obj._param = params.__getitem__
    obj.wait = Mock(return_value=True)
    return obj


def action_client(result, status=GoalStatus.STATUS_SUCCEEDED, accepted=True):
    handle = Mock(accepted=accepted)
    handle.get_result_async.return_value.result.return_value = NS(result=result, status=status)
    client = Mock()
    client.send_goal_async.return_value.result.return_value = handle
    return client, handle


@pytest.mark.parametrize('kind,status,reached,error,expected', [
    ('simulation', GoalStatus.STATUS_SUCCEEDED, True, 0, True),
    ('simulation', GoalStatus.STATUS_SUCCEEDED, True, -1, False),
    ('simulation', GoalStatus.STATUS_ABORTED, True, 0, False),
    ('hardware', GoalStatus.STATUS_SUCCEEDED, True, 0, True),
    ('hardware', GoalStatus.STATUS_SUCCEEDED, False, 0, False),
    ('hardware', GoalStatus.STATUS_ABORTED, True, 0, False),
])
def test_close_gripper_checks_result(kind, status, reached, error, expected):
    obj = demo()
    trajectory, _ = action_client(FollowJointTrajectory.Result(error_code=error), status)
    command, _ = action_client(GripperCommand.Result(reached_goal=reached), status)
    trajectory.wait_for_server.return_value = kind == 'simulation'
    with patch('rebotarm_moveit_demos.draw_square.ActionClient', side_effect=[trajectory, command]):
        assert obj._close_gripper() is expected
    selected = trajectory if kind == 'simulation' else command
    goal = selected.send_goal_async.call_args.args[0]
    if kind == 'simulation':
        assert list(goal.trajectory.points[0].positions) == [0.0, 0.0]
    else:
        assert goal.command.position == 0.0
    trajectory.destroy.assert_called_once()
    command.destroy.assert_called_once()


def test_close_timeout_cancels_goal():
    obj = demo()
    trajectory, handle = action_client(FollowJointTrajectory.Result())
    obj.wait.side_effect = [True, False, True]
    with patch('rebotarm_moveit_demos.draw_square.ActionClient', side_effect=[trajectory, Mock()]):
        assert obj._close_gripper() is False
    handle.cancel_goal_async.assert_called_once()


def test_gripper_unavailable_does_not_send_goal():
    obj = demo()
    clients = [Mock(), Mock()]
    for client in clients:
        client.wait_for_server.return_value = False
    with patch('rebotarm_moveit_demos.draw_square.ActionClient', side_effect=clients):
        assert obj._close_gripper() is False
    for client in clients:
        client.send_goal_async.assert_not_called()


def test_trace_records_tf_positions_only_during_rectangle():
    obj = demo()
    obj.frame_id = 'base_link'
    obj.tcp_link_name = 'gripper_tcp'
    obj._tracing = False
    obj._trace_points = []
    obj._tf_buffer = Mock()
    obj._publish_markers = Mock()
    obj._record_tcp()
    obj._tf_buffer.lookup_transform.assert_not_called()
    obj._tracing = True
    transform = TransformStamped()
    transform.transform.translation.x = .28
    transform.transform.translation.z = .12
    obj._tf_buffer.lookup_transform.return_value = transform
    obj._record_tcp()
    obj._record_tcp()
    assert len(obj._trace_points) == 1
    assert obj._trace_points[0].x == .28
    transform.transform.translation.y = .04
    obj._record_tcp()
    assert len(obj._trace_points) == 2
    assert obj._trace_points[1].y == .04


def test_markers_close_reference_and_clear_previous_trace():
    from builtin_interfaces.msg import Time
    obj = demo()
    obj.frame_id = 'base_link'
    obj.rectangle_center = [.28, 0., .12]
    obj.rectangle_width = .04
    obj.rectangle_height = .08
    obj._trace_points = []
    obj._markers = Mock()
    obj.node.get_clock = lambda: NS(now=lambda: NS(to_msg=Time))
    obj._publish_markers()
    reference, trace = obj._markers.publish.call_args.args[0].markers
    assert len(reference.points) == 5
    assert reference.points[0] == reference.points[-1]
    assert reference.header.frame_id == 'base_link'
    assert reference.color.a > 0
    assert trace.action == Marker.DELETE
