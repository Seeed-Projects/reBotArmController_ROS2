#!/usr/bin/env python3
"""Start internal gravity compensation on the ROS controller."""

from __future__ import annotations

import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rebotarm_msgs.msg import ArmStatus
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger

_NAMESPACE = "rebotarm"
_CONTROLLER_TIMEOUT_SEC = 3.0


def _call_trigger(
    node: Node,
    client,
    label: str,
    timeout_sec: float = 5.0,
) -> bool:
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f"{label} service not available")
        return False
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        node.get_logger().error(f"{label} timed out")
        return False
    result = future.result()
    if result is None or not result.success:
        message = result.message if result is not None else "no response"
        node.get_logger().error(f"{label} failed: {message}")
        return False
    node.get_logger().info(message if (message := result.message) else f"{label} OK")
    return True


def main() -> None:
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("gravity_compensation")
    stop_requested = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stop_requested
        if not stop_requested:
            node.get_logger().info("stop requested, shutting down gravity compensation")
        stop_requested = True

    old_sigint = signal.getsignal(signal.SIGINT)
    old_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    enable_client = node.create_client(
        Trigger,
        f"/{_NAMESPACE}/enable",
    )
    start_client = node.create_client(
        Trigger,
        f"/{_NAMESPACE}/gravity_compensation/start",
    )
    safe_home_client = node.create_client(
        Trigger,
        f"/{_NAMESPACE}/safe_home",
    )
    disable_client = node.create_client(
        Trigger,
        f"/{_NAMESPACE}/disable",
    )

    last_status_received = time.monotonic()

    def receive_status(_message: ArmStatus) -> None:
        nonlocal last_status_received
        last_status_received = time.monotonic()

    node.create_subscription(
        ArmStatus,
        f"/{_NAMESPACE}/arm_status",
        receive_status,
        qos_profile_sensor_data,
    )
    controller_lost = False
    gc_started = False
    try:
        if not _call_trigger(node, enable_client, "enable"):
            raise SystemExit(1)
        if not _call_trigger(
            node,
            start_client,
            "start gravity compensation",
        ):
            raise SystemExit(1)
        gc_started = True
        last_status_received = time.monotonic()
        node.get_logger().info("press Ctrl+C to stop gravity compensation")
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.2)
            if (
                not start_client.service_is_ready()
                or time.monotonic() - last_status_received >= _CONTROLLER_TIMEOUT_SEC
            ):
                controller_lost = True
                node.get_logger().warn(
                    "Controller is unavailable or its status heartbeat stopped; "
                    "exiting gravity compensation client"
                )
                break
    finally:
        # The controller owns shutdown homing/disable. Do not issue duplicate
        # cleanup requests after it has disappeared or begun shutting down.
        if gc_started and not controller_lost:
            try:
                _call_trigger(node, safe_home_client, "safe_home", timeout_sec=35.0)
            except Exception as exc:
                node.get_logger().warn(f"safe_home cleanup failed: {exc}")
            try:
                _call_trigger(node, disable_client, "disable")
            except Exception as exc:
                node.get_logger().warn(f"disable cleanup failed: {exc}")
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
