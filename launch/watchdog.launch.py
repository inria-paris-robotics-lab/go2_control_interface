from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ignore_joint_limits_arg = DeclareLaunchArgument(
        'ignore_joint_limits',
        default_value='False',
        description="If set to true, watchdog won't enforce joint position limits"
    )
    freq_arg = DeclareLaunchArgument(
        'freq',
        default_value='100',
        description="Polling frequency of the safety (freq * n_fails = timeout limit)"
    )
    n_fails_arg = DeclareLaunchArgument(
        'n_fails',
        default_value='2',
        description="Number of polling step without receiving new joint command allowed (freq * n_fails = timeout limit)"
    )

    watchdog_config_filepath = PathJoinSubstitution([
                                    FindPackageShare('go2_control_interface'),
                                    'config',
                                    'default_limits.yaml'
                                ])

    return LaunchDescription([
        ignore_joint_limits_arg,
        freq_arg,
        n_fails_arg,
        Node(
            package='go2_control_interface',
            executable='watchdog_node.py',
            name='watchdog',
            output='screen',
            parameters=[watchdog_config_filepath,
            {
             "ignore_joint_limits": LaunchConfiguration("ignore_joint_limits"),
             "freq": LaunchConfiguration("freq"),
             "n_fails": LaunchConfiguration("n_fails"),
             }],
           ),
])
