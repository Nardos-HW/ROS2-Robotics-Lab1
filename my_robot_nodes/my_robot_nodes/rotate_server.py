#!/usr/bin/env python3
"""Action server that rotates the robot by a specified angle using odometry."""

import math
import time

from geometry_msgs.msg import Twist
from my_robot_interfaces.action import Rotate
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node


def yaw_from_quaternion(q):
    """Rotation about the vertical axis, in rad, of an orientation quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_difference(target, current):
    """Calculate target - current, wrapped to [-pi, pi]."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


class RotateServer(Node):
    """Action server for rotating the robot by a specified angle in radians."""

    def __init__(self):
        """Initialize the RotateServer node, subscriptions, and action server."""
        super().__init__('rotate_server')
        self.declare_parameter('turn_speed', 0.5)

        self.yaw = None

        group = ReentrantCallbackGroup()
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_odom, 10, callback_group=group
        )
        self.action_server = ActionServer(
            self,
            Rotate,
            'rotate',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=group,
        )
        self.get_logger().info('rotate_server node initialized')

    def on_odom(self, msg: Odometry) -> None:
        """Update current yaw heading from odometry."""
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def on_goal(self, goal_request: Rotate.Goal) -> GoalResponse:
        """Validate rotation goal angle and odometry readiness."""
        angle = goal_request.angle
        if math.isnan(angle) or math.isinf(angle) or abs(angle) < 1e-6:
            self.get_logger().warning(f'Rejecting goal with invalid angle: {angle}')
            return GoalResponse.REJECT
        if self.yaw is None:
            start_time = time.time()
            while self.yaw is None and (time.time() - start_time) < 5.0:
                time.sleep(0.05)
        if self.yaw is None:
            self.get_logger().warning('Rejecting goal: odometry not yet received')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle) -> CancelResponse:
        """Accept goal cancellation requests."""
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    def execute(self, goal_handle) -> Rotate.Result:
        """Execute the rotation goal while accumulating yaw differences."""
        goal = float(goal_handle.request.angle)
        self.get_logger().info(f'Starting rotation: goal={goal:.3f} rad')

        feedback = Rotate.Feedback()
        result = Rotate.Result()

        last_yaw = self.yaw
        accumulated = 0.0
        twist = Twist()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop()
                goal_handle.canceled()
                result.turned = float(accumulated)
                self.get_logger().info(f'Rotation canceled at {accumulated:.3f} rad')
                return result

            current_yaw = self.yaw
            if current_yaw is not None and last_yaw is not None:
                delta = angle_difference(current_yaw, last_yaw)
                accumulated += delta
                last_yaw = current_yaw

            feedback.turned = float(accumulated)
            goal_handle.publish_feedback(feedback)
            result.turned = float(accumulated)

            if (goal > 0.0 and accumulated >= goal) or (goal < 0.0 and accumulated <= goal):
                break

            turn_speed = abs(float(self.get_parameter('turn_speed').value))
            twist.angular.z = turn_speed if goal > 0.0 else -turn_speed
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.02)

        self.stop()
        goal_handle.succeed()
        self.get_logger().info(
            f'Rotation completed: turned {result.turned:.3f} rad (goal: {goal:.3f} rad)'
        )
        return result

    def stop(self) -> None:
        """Publish zero velocity to stop robot rotation."""
        if rclpy.ok():
            self.cmd_vel_pub.publish(Twist())


def main(args=None):
    """Run the RotateServer node."""
    rclpy.init(args=args)
    node = RotateServer()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
