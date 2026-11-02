#!/usr/bin/env python3
"""Drive the robot forward until an obstacle is detected, then turn away."""

from geometry_msgs.msg import Twist
from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus
from my_robot_interfaces.srv import SetTurnSpeed
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool


class TurnAway(Node):
    """Subscribes to /obstacle and publishes velocity commands to /cmd_vel."""

    def __init__(self):
        super().__init__('turn_away')
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('max_turn_speed', 1.5)

        self.enabled = False

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(RobotStatus, '/robot_status', 10)
        self.obstacle_sub = self.create_subscription(
            ObstacleInfo, '/obstacle', self.on_obstacle, 10
        )
        self.enable_srv = self.create_service(
            SetBool, '/turn_away_enable', self.on_enable
        )
        self.turn_speed_srv = self.create_service(
            SetTurnSpeed, '/set_turn_speed', self.on_set_turn_speed
        )
        self.get_logger().info('turn_away node started (disabled)')

    def on_enable(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """Enable or disable autonomous obstacle avoidance."""
        if request.data:
            self.enabled = True
            response.success = True
            response.message = 'Node enabled'
        else:
            self.enabled = False
            self.stop()
            response.success = True
            response.message = 'Node disabled'

        self.get_logger().info(response.message)
        return response

    def on_set_turn_speed(
        self, request: SetTurnSpeed.Request, response: SetTurnSpeed.Response
    ) -> SetTurnSpeed.Response:
        """Set the turn_speed parameter within (0, max_turn_speed]."""
        max_turn_speed = float(self.get_parameter('max_turn_speed').value)
        speed = float(request.turn_speed)

        if speed <= 0.0:
            response.success = False
            response.message = f'turn_speed must be positive, got {speed:.2f}'
        elif speed > max_turn_speed:
            response.success = False
            response.message = (
                f'turn_speed must not exceed max_turn_speed ({max_turn_speed:.2f}), '
                f'got {speed:.2f}'
            )
        else:
            self.set_parameters([Parameter('turn_speed', value=speed)])
            response.success = True
            response.message = f'turn_speed set to {speed:.2f} rad/s'

        self.get_logger().info(response.message)
        return response

    def on_obstacle(self, msg: ObstacleInfo) -> None:
        """Process incoming obstacle info and update movement/status."""
        status = RobotStatus()
        status.obstacle_distance = float(msg.distance)

        if not self.enabled:
            status.state = 'stopped'
            self.status_pub.publish(status)
            return

        forward_speed = float(self.get_parameter('forward_speed').value)
        turn_speed = float(self.get_parameter('turn_speed').value)

        twist = Twist()

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
