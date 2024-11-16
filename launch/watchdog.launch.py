from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    watchdog_config_filepath = PathJoinSubstitution([
                                    FindPackageShare('go2_control_interface'),
                                    'config',
                                    'default_limits.yaml'
                                ])

    return LaunchDescription([
        Node(
            package='go2_control_interface',
            executable='watchdog.py',
            name='watchdog',
            output='screen',
            parameters=[watchdog_config_filepath],
           ),
])