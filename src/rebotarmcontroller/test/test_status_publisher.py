"""Status heartbeat regression tests; no motor or transport is created."""
from types import SimpleNamespace
from unittest.mock import Mock

from builtin_interfaces.msg import Time
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from rebotarmcontroller.ros_publishers import JointStatePublisher


def make_publisher():
    node = Mock()
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))
    hardware = SimpleNamespace(
        joint_names=["joint1"], has_gripper=False,
        mode="mit", enabled=True, control_loop_active=True,
        state_machine="IDLE", error_codes=[],
        get_joint_status_codes=Mock(return_value=[0]),
    )
    publisher = JointStatePublisher(node, hardware, "rebotarm", 100.0)
    return publisher, node, hardware


def test_status_keeps_latched_reliable_qos_and_adds_one_hz_heartbeat():
    publisher, node, _ = make_publisher()
    status_call = next(call for call in node.create_publisher.call_args_list
                       if call.args[1] == "/rebotarm/arm_status")
    qos = status_call.args[2]
    assert qos.depth == 1
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert any(call.args[0] == 1.0 and call.args[1] == publisher.publish_status
               for call in node.create_timer.call_args_list)


def test_heartbeat_reads_current_status_instead_of_replaying_startup_value():
    publisher, _, hardware = make_publisher()
    status_pub = publisher._status_publisher
    assert status_pub.publish.call_args.args[0].enabled is True
    hardware.enabled = False
    hardware.control_loop_active = False
    publisher.publish_status()
    message = status_pub.publish.call_args.args[0]
    assert message.enabled is False
    assert message.control_loop_active is False
    assert list(message.per_joint_status_code) == [0]
