#!/usr/bin/env python3
"""Drive the robot forward until an obstacle is detected, then turn away."""

from geometry_msgs.msg import Twist
from lab_interfaces.msg import ObstacleInfo
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class TurnAway(Node):
    """Subscribes to /obstacle and publishes velocity commands to /cmd_vel."""

    def __init__(self):
        super().__init__('turn_away')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obstacle_sub = self.create_subscription(
            ObstacleInfo, '/obstacle', self.on_obstacle, 10
        )
        self.get_logger().info('turn_away node started')

    def on_obstacle(self, msg: ObstacleInfo) -> None:
        twist = Twist()

        if not msg.too_close:
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            self.get_logger().info('Driving forward at 0.2 m/s')
        else:
            twist.linear.x = 0.0
            if msg.angle > 0.0:
                twist.angular.z = -0.5
                self.get_logger().info(
                    f'Obstacle too close ({msg.distance:.2f} m, angle {msg.angle:.2f} rad): '
                    'turning right at -0.5 rad/s'
                )
            else:
                twist.angular.z = 0.5
                self.get_logger().info(
                    f'Obstacle too close ({msg.distance:.2f} m, angle {msg.angle:.2f} rad): '
                    'turning left at 0.5 rad/s'
                )

        self.cmd_vel_pub.publish(twist)

    def stop(self) -> None:
        """Publish zero velocity to stop the robot safely."""
        if rclpy.ok():
            self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = TurnAway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
