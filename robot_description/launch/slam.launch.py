from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 获取 robot_description 包目录
    robot_description_dir = get_package_share_directory('robot_description')

    # 获取 URDF 路径
    urdf_path = os.path.join(robot_description_dir, 'urdf', 'robot.urdf')

    # RViz 配置文件
    rviz_config_path = os.path.join("/home/sunrise/nav_ws/install/robot_description/share/robot_description/rviz.rviz")
 
    # 读取 URDF 内容
    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),

        # 雷达节点
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB1',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True
            }],
            output='screen'
        ),

        # IMU 节点
        Node(
            package='wit_ros2_imu',
            executable='wit_ros2_imu',
            name='imu_node',
            parameters=[{
                'port': '/dev/imu_usb',
                'baud': 9600
            }],
            remappings=[('/wit/imu', '/imu/data_raw')],
            output='screen'
        ),

        # joint_state 发布器
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),

        # 发布 robot_state（TF）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time
            }]
        ),



        # 补全cartgrapher
          Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', '/home/sunrise/nav_ws/install/robot_description/share/robot_description/config',
                '-configuration_basename', 'ld_2d.lua'
            ],
            remappings=[
                ('scan', '/scan'),          # LiDAR话题
                ('imu', '/imu/data_raw'),       # IMU话题
            ]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            arguments=['-resolution', '0.05',
                       '-publish_period_sec', '0.5']  # 地图分辨率 //地图更新频率
            
        ),
        # 启动 map_saver 服务（地图保存服务端）
        Node(
            package='nav2_map_server',
            executable='map_saver_server',
            name='map_saver_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'save_map_timeout': 5.0,
                'free_thresh': 0.25,
                'occupied_thresh': 0.65,
                'map_subscribe_transient_local': True,
                'autostart': True  # ✅ 加上这个，服务才能自动激活
            }]
        ),
        # RViz 可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
        # 自动激活生命周期节点
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_saver',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_saver_server']  # 要管理的节点名列表
            }]
        ),
        Node(
            package='robot_description',
            executable='auto_save_map_node.py',
            name='auto_save_map_node',
            output='screen'
        )
    ])
