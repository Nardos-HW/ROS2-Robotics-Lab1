"""Launch the square trajectory demo with diffdrive_bot control and action nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate launch description for square driving."""
    pkg_diffdrive = get_package_share_directory('diffdrive_bot')

    side_length = ParameterValue(LaunchConfiguration('side_length'), value_type=float)

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_diffdrive, 'launch', 'control.launch.py')
        ),
    )

    rotate_server = Node(
        package='my_robot_nodes',
        executable='rotate_server',
        name='rotate_server',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    square_driver = Node(
        package='my_robot_nodes',
        executable='square_driver',
        name='square_driver',
        output='screen',
        parameters=[{
            'side_length': side_length,
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'side_length',
            default_value='0.5',
            description='Side length of the square in meters',
        ),
        control_launch,
        rotate_server,
        square_driver,
    ])
