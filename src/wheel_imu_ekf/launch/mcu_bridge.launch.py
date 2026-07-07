"""
MCU Odometry Bridge Launch
启动MCU里程计桥接节点，解析UART数据并发布到ROS2
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('wheel_imu_ekf')
    config_file = os.path.join(pkg_dir, 'config', 'mcu_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='MCU串口设备'
        ),

        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='串口波特率'
        ),

        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Odometry输出话题'
        ),

        Node(
            package='wheel_imu_ekf',
            executable='mcu_odom_bridge_node',
            name='mcu_odom_bridge_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'serial_port': LaunchConfiguration('serial_port'),
                    'baud_rate': LaunchConfiguration('baud_rate'),
                    'odom_topic': LaunchConfiguration('odom_topic'),
                }
            ],
        ),
    ])
