from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 包路径查找
    pkg_robot_desc = FindPackageShare('robot_description')
    pkg_slam_toolbox = FindPackageShare('slam_toolbox')

    # URDF 加载 launch 文件路径
    urdf_launch_path = PathJoinSubstitution([pkg_robot_desc, 'launch', 'display.launch.py'])

    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),

        # 启动 IMU 节点
        Node(
            package='wit_ros2_imu',
            executable='wit_ros2_imu',
            name='imu',
            remappings=[('/wit/imu', '/imu/data')],
            parameters=[
                {'port': '/dev/imu_usb'},
                {'baud': 9600}
            ],
            output='screen'
        ),

        # 启动 RPLIDAR 节点（可替换为你自己的 launch 文件）
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB1',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }],
            output='screen'
        ),

        # 加载 URDF 模型（确保它发布 /tf）
        IncludeLaunchDescription(urdf_launch_path),

        # 启动 IMU 滤波器（如果 IMU 只提供 /imu/data_raw）
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            parameters=[{'use_mag': False}],
            remappings=[
                ('imu/data_raw', '/imu/data'),
                ('imu/mag', '/imu/mag'),  # 可选
                ('imu/data', '/imu/data_filtered')
            ]
        ),

        # 启动 SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[{
                'use_sim_time': use_sim_time,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'imu_topic': '/imu/data_filtered',  # 使用滤波后的数据
                'enable_interactive_mode': False,
                'publish_period': 5.0,
                'minimum_travel_distance': 0.1,
                'minimum_turn_angle': 0.1,
                'max_laser_range': 6.0
            }],
            output='screen'
        ),

        # 启动 RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([pkg_robot_desc, 'rviz', 'mapping.rviz'])],
            output='screen'
        )
    ])