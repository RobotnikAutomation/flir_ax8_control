from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace', default_value='robot'),
        DeclareLaunchArgument('desired_freq', default_value='10'),
        DeclareLaunchArgument('ip_address', default_value='192.168.0.104'),
        DeclareLaunchArgument('request_timeout_sec', default_value='1.0'),
        Node(
            package='flir_ax8_control',
            executable='flir_ax8_control',
            name='flir_ax8_control',
            namespace=LaunchConfiguration('robot_namespace'),
            output='screen',
            parameters=[
                {'desired_freq': LaunchConfiguration('desired_freq')},
                {'ip_address': LaunchConfiguration('ip_address')},
                {'request_timeout_sec': LaunchConfiguration('request_timeout_sec')}
            ]
        )
    ])
