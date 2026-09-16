from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('frontscan_ros2')
    default_config = os.path.join(pkg_share, 'config', 'omniscan450_fs_params.yaml')
    sensor_number = LaunchConfiguration('sensor_number')

    return LaunchDescription([
        DeclareLaunchArgument('sensor_number', default_value='450'),
        DeclareLaunchArgument('ip_address', default_value='192.168.194.90'),
        DeclareLaunchArgument('sonar_port', default_value='51200'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        Node(
            package='frontscan_ros2',
            executable='frontscan_node',
            name=PythonExpression(["'frontscan' + '", sensor_number, "'"]),
            output='screen',
            respawn=True,
            respawn_delay=30.0,
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'sensor_number': sensor_number,
                    'ip_address': LaunchConfiguration('ip_address'),
                    'sonar_port': PythonExpression(
                        ["int('", LaunchConfiguration('sonar_port'), "')"]),
                },
            ],
        ),
    ])
