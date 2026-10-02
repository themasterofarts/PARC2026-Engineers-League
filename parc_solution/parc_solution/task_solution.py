#!/usr/bin/env python3
"""Entry point for the PARC autonomous navigation task.

This initial milestone deliberately performs only a package-health check.
Navigation startup, localization, and goal dispatch will be added after the
simulator interfaces and static map have been validated.
"""

import rclpy
from rclpy.node import Node


class TaskSolution(Node):
    """Minimal node used to verify the solution package is runnable."""

    def __init__(self) -> None:
        super().__init__("task_solution")
        self.get_logger().info(
            "parc_solution is installed; navigation setup is the next milestone."
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskSolution()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
