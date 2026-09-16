from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('frontscan_ros2')
    default_config = os.path.join(pkg_share, 'config', 'omniscan450_fs_params.yaml')

    sensor_number = LaunchConfiguration('sensor_number')
    web_port = LaunchConfiguration('web_port')
    rosbridge_port = LaunchConfiguration('rosbridge_port')

    rosbridge_globs = {
        'topics_glob': '/frontscan450/*',
        'topics_sub_glob': '/frontscan450/*',
        'topics_pub_glob': '/frontscan450/*',
    }

    return LaunchDescription([
        DeclareLaunchArgument('sensor_number', default_value='450'),
        DeclareLaunchArgument('ip_address', default_value='192.168.194.90'),
        DeclareLaunchArgument('sonar_port', default_value='51200'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('web_port', default_value='9002'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),

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

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': ParameterValue(rosbridge_port, value_type=int),
                'max_message_size': 10000000,
                **rosbridge_globs,
            }],
        ),

        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
            parameters=[rosbridge_globs],
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'frontscan_ros2', 'viewer_server',
                '--port', web_port,
            ],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
