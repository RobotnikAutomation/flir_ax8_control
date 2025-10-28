from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('desired_freq', default_value='10'),
        DeclareLaunchArgument('ip_address', default_value='192.168.0.185'),
        Node(
            package='flir_ax8_control',
            executable='flir_ax8_control_node.py',
            name='flir_ax8_control',
            output='screen',
            parameters=[
                {'desired_freq': LaunchConfiguration('desired_freq')},
                {'ip_address': LaunchConfiguration('ip_address')}
            ]
        )
    ])
