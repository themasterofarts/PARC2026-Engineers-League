#!/usr/bin/env python3
"""Forward Nav2 velocity commands to the simulator's command topic."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelRelay(Node):
    """Bridge Nav2's final `/cmd_vel` output to the supplied Gazebo bridge."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_relay")
        self._publisher = self.create_publisher(
            Twist, "/robot_base_controller/cmd_vel_unstamped", 10
        )
        self._subscription = self.create_subscription(
            Twist, "/cmd_vel", self._forward_command, 10
        )
        self.get_logger().info(
            "Relaying Nav2 /cmd_vel to "
            "/robot_base_controller/cmd_vel_unstamped"
        )

    def _forward_command(self, command: Twist) -> None:
        self._publisher.publish(command)

    def stop(self) -> None:
        """Publish a final zero command when this relay is stopped."""
        # A ros2 launch Ctrl+C may already have invalidated rclpy's context
        # before this process reaches its cleanup block. Avoid converting an
        # otherwise normal shutdown into a traceback in that situation.
        try:
            if rclpy.ok():
                self._publisher.publish(Twist())
        # RCLError comes from rclpy's C extension in Jazzy and is not exported
        # by rclpy.exceptions. This cleanup path must never turn Ctrl+C into
        # an application error, regardless of shutdown ordering.
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
