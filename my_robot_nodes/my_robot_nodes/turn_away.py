#!/usr/bin/env python3
"""Drive the robot forward until an obstacle is detected, then turn away."""

from geometry_msgs.msg import Twist
from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class TurnAway(Node):
    """Subscribes to /obstacle and publishes velocity commands to /cmd_vel."""

    def __init__(self):
        super().__init__('turn_away')
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.5)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(RobotStatus, '/robot_status', 10)
        self.obstacle_sub = self.create_subscription(
            ObstacleInfo, '/obstacle', self.on_obstacle, 10
        )
        self.get_logger().info('turn_away node started')

    def on_obstacle(self, msg: ObstacleInfo) -> None:
        forward_speed = float(self.get_parameter('forward_speed').value)
        turn_speed = float(self.get_parameter('turn_speed').value)

        twist = Twist()
        status = RobotStatus()
        status.obstacle_distance = float(msg.distance)

        if not msg.too_close:
            twist.linear.x = forward_speed
            twist.angular.z = 0.0
            status.state = 'forward'
            self.get_logger().info(f'Driving forward at {forward_speed:.2f} m/s')
        else:
            twist.linear.x = 0.0
            status.state = 'turning'
            if msg.angle > 0.0:
                twist.angular.z = -turn_speed
                self.get_logger().info(
                    f'Obstacle too close ({msg.distance:.2f} m, angle {msg.angle:.2f} rad): '
                    f'turning right at {-turn_speed:.2f} rad/s'
                )
            else:
                twist.angular.z = turn_speed
                self.get_logger().info(
                    f'Obstacle too close ({msg.distance:.2f} m, angle {msg.angle:.2f} rad): '
                    f'turning left at {turn_speed:.2f} rad/s'
                )

        self.cmd_vel_pub.publish(twist)
        self.status_pub.publish(status)

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
