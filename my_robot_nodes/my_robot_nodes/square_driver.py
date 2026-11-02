#!/usr/bin/env python3
"""Action client that drives the robot in a square trajectory."""

import math

from action_msgs.msg import GoalStatus
from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class SquareDriver(Node):
    """Action client executing a 4-side square using drive_distance and rotate."""

    def __init__(self):
        """Initialize the SquareDriver node and action clients."""
        super().__init__('square_driver')
        self.declare_parameter('side_length', 0.5)

        self.drive_client = ActionClient(self, DriveDistance, 'drive_distance')
        self.rotate_client = ActionClient(self, Rotate, 'rotate')
        self.get_logger().info('square_driver node initialized')

    def wait_for_servers(self, timeout_sec: float = 30.0) -> bool:
        """Wait for drive_distance and rotate action servers to become available."""
        self.get_logger().info('Waiting for drive_distance action server...')
        if not self.drive_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('drive_distance action server not available')
            return False

        self.get_logger().info('Waiting for rotate action server...')
        if not self.rotate_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('rotate action server not available')
            return False

        self.get_logger().info('Both action servers are available')
        return True

    def on_drive_feedback(self, feedback_msg) -> None:
        """Log drive_distance progress feedback."""
        travelled = feedback_msg.feedback.travelled
        self.get_logger().info(f'Drive feedback: travelled {travelled:.2f} m')

    def on_rotate_feedback(self, feedback_msg) -> None:
        """Log rotate progress feedback."""
        turned = feedback_msg.feedback.turned
        self.get_logger().info(f'Rotate feedback: turned {turned:.3f} rad')

    def drive_distance(self, distance: float) -> bool:
        """Send drive_distance goal and wait for its completion."""
        goal = DriveDistance.Goal()
        goal.distance = distance
        self.get_logger().info(f'Sending drive goal: {distance:.2f} m')

        send_future = self.drive_client.send_goal_async(
            goal, feedback_callback=self.on_drive_feedback
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Drive goal was rejected by action server')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f'Drive goal failed with status code {wrapped_result.status}'
            )
            return False

        self.get_logger().info(
            f'Drive completed: travelled {wrapped_result.result.travelled:.2f} m'
        )
        return True

    def rotate(self, angle: float) -> bool:
        """Send rotate goal and wait for its completion."""
        goal = Rotate.Goal()
        goal.angle = angle
        self.get_logger().info(f'Sending rotate goal: {angle:.3f} rad')

        send_future = self.rotate_client.send_goal_async(
            goal, feedback_callback=self.on_rotate_feedback
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Rotate goal was rejected by action server')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f'Rotate goal failed with status code {wrapped_result.status}'
            )
            return False

        self.get_logger().info(
            f'Rotate completed: turned {wrapped_result.result.turned:.3f} rad'
        )
        return True

    def execute_square(self) -> bool:
        """Execute a 4-side square trajectory with four 90-degree left turns."""
        side_length = float(self.get_parameter('side_length').value)
        if side_length <= 0.0:
            self.get_logger().error(
                f'Invalid side_length: {side_length}. Must be greater than 0.'
            )
            return False

        self.get_logger().info(
            f'Starting square trajectory with side_length={side_length:.2f} m'
        )

        turn_angle = math.pi / 2.0

        for side in range(1, 5):
            self.get_logger().info(
                f'=== Side {side}/4: driving {side_length:.2f} m ==='
            )
            if not self.drive_distance(side_length):
                self.get_logger().error(f'Square trajectory failed on side {side}')
                return False

            self.get_logger().info(
                f'=== Corner {side}/4: turning left {turn_angle:.3f} rad ==='
            )
            if not self.rotate(turn_angle):
                self.get_logger().error(f'Square trajectory failed on corner {side}')
                return False

        self.get_logger().info('Square trajectory completed successfully!')
        return True


def main(args=None):
    """Run the square_driver node."""
    rclpy.init(args=args)
    node = SquareDriver()
    try:
        if node.wait_for_servers():
            node.execute_square()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
