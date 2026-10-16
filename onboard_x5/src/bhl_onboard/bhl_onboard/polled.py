"""Subscriptions read by the control thread instead of the executor.

On the RDK X5 (Cortex-A55) every rclpy executor wake-up costs ~5-7 ms of CPU
(the wait set is rebuilt in Python, with every service, action and QoS-event
handler of the node). A 200 Hz topic through the executor therefore needs more
than one core. Instead, high-rate topics are subscribed on a helper node that
is never added to an executor, and the control loop takes the newest message
itself -- ~20 us when nothing arrived.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node


class PolledSubscription:
    def __init__(self, node: Node, msg_type, topic: str, qos) -> None:
        self.sub = node.create_subscription(msg_type, topic, lambda _msg: None, qos)

    def take_latest(self):
        """Newest queued message, or None. Older queued ones are dropped."""
        latest = None
        while True:
            with self.sub.handle:
                taken = self.sub.handle.take_message(self.sub.msg_type, self.sub.raw)
            if taken is None:
                return latest
            latest = taken[0]


def io_node(name: str) -> Node:
    """A node for polled subscriptions: no parameter services, never spun."""
    return rclpy.create_node(name, start_parameter_services=False)
