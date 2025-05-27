from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    port = LaunchConfiguration('port')
    baud_rate = LaunchConfiguration('baud_rate')
    frame_id = LaunchConfiguration('frame_id')
    version_num = LaunchConfiguration('version_num')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for the LiDAR device'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for the LiDAR serial communication'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='base_laser_link',
            description='Frame ID for the LaserScan messages'
        ),
        DeclareLaunchArgument(
            'version_num',
            default_value='1',
            description='Version number: 0 for n301 TOF, 1 for line laser 1 deg, 2 for line laser 0.5 deg'
        ),

        # Node for the LiDAR driver
        Node(
            package='line_laser_ros',
            executable='n301n_serial_publisher',
            name='line_laser',
            output='screen',
            parameters=[
                {'port': port},
                {'baud_rate': baud_rate},
                {'frame_id': frame_id},
                {'version_num': version_num}
            ]
        ),

        # Node for Lidar Processor
        Node(
            package='line_laser_ros',
            executable='lidar_processor',
            name='lidar_processor',
            output='screen',
            parameters=[
                {'uart_port': '/dev/ttyUSB1'},
                {'uart_baud_rate': 115200}
            ]
        )
    ])
