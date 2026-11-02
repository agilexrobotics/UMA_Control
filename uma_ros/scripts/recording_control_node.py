#!/usr/bin/env python3
"""One global recording request; the web collector owns the collection launch."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Empty
from std_srvs.srv import SetBool


class RecordingControlNode(Node):
    def __init__(self):
        super().__init__("recording_control")
        self.recording_requested = False
        self.publisher = self.create_publisher(
            Bool, "/recording/requested",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(Empty, "/recording/toggle", self.toggle, 10)
        self.create_service(SetBool, "/recording/set_requested", self.set_requested)
        self.publisher.publish(Bool(data=False))

    def _update(self, requested):
        if requested == self.recording_requested:
            return
        self.recording_requested = requested
        self.publisher.publish(Bool(data=requested))
        self.get_logger().info(f"recording_requested={str(requested).lower()}")

    def toggle(self, _message):
        # The default mutually exclusive callback group serializes both handles.
        self._update(not self.recording_requested)

    def set_requested(self, request, response):
        self._update(request.data)
        response.success = True
        response.message = "Recording request updated; launch is controlled by the web collector."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RecordingControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._update(False)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
