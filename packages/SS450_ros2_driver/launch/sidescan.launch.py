from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('sidescan_ros2')
    default_config = os.path.join(pkg_share, 'config', 'omniscan450_params.yaml')
    sensor_number = LaunchConfiguration('sensor_number')

    return LaunchDescription([
        DeclareLaunchArgument('sensor_number', default_value='450'),
        # Lab vehicle LAN defaults (override package 192.168.2.x examples).
        DeclareLaunchArgument('port_ip_address', default_value='192.168.194.92'),
        DeclareLaunchArgument('port_port', default_value='51200'),
        DeclareLaunchArgument('starboard_ip_address', default_value='192.168.194.93'),
        DeclareLaunchArgument('starboard_port', default_value='51200'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        # Sonar tune (override config yaml when set from GUI / CLI).
        DeclareLaunchArgument('speed_of_sound_mm', default_value='1482000'),
        DeclareLaunchArgument('start_mm', default_value='0'),
        DeclareLaunchArgument('length_mm', default_value='5000'),
        DeclareLaunchArgument('msec_per_ping', default_value='0'),
        DeclareLaunchArgument('pulse_len_percent', default_value='0.002'),
        DeclareLaunchArgument('filter_duration_percent', default_value='0.0015'),
        DeclareLaunchArgument('gain_index', default_value='-1'),
        DeclareLaunchArgument('num_results', default_value='600'),
        Node(
            package='sidescan_ros2',
            executable='sidescan_node',
            name=PythonExpression(["'omniscan' + '", sensor_number, "'"]),
            output='screen',
            respawn=True,
            respawn_delay=30.0,
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'sensor_number': sensor_number,
                    'port_ip_address': LaunchConfiguration('port_ip_address'),
                    'port_port': PythonExpression(["int('", LaunchConfiguration('port_port'), "')"]),
                    'starboard_ip_address': LaunchConfiguration('starboard_ip_address'),
                    'starboard_port': PythonExpression(
                        ["int('", LaunchConfiguration('starboard_port'), "')"]),
                    'speed_of_sound_mm': PythonExpression(
                        ["int('", LaunchConfiguration('speed_of_sound_mm'), "')"]),
                    'start_mm': PythonExpression(["int('", LaunchConfiguration('start_mm'), "')"]),
                    'length_mm': PythonExpression(["int('", LaunchConfiguration('length_mm'), "')"]),
                    'msec_per_ping': PythonExpression(
                        ["int('", LaunchConfiguration('msec_per_ping'), "')"]),
                    'pulse_len_percent': PythonExpression(
                        ["float('", LaunchConfiguration('pulse_len_percent'), "')"]),
                    'filter_duration_percent': PythonExpression(
                        ["float('", LaunchConfiguration('filter_duration_percent'), "')"]),
                    'gain_index': PythonExpression(
                        ["int('", LaunchConfiguration('gain_index'), "')"]),
                    'num_results': PythonExpression(
                        ["int('", LaunchConfiguration('num_results'), "')"]),
                },
            ],
        ),
    ])
