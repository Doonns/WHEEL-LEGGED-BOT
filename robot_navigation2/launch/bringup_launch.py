
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明启动参数
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    map_file = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_navigation2'),
            'maps',
            'your_map.yaml'
        ]),
        description='Full path to map yaml file to load'
    )
    
    params_file = DeclareLaunchArgument(
        'params_file',
    default_value='/home/sunrise/nav_ws/src/robot_navigation2/param/robot_nav2.yaml',
    description='Full path to the ROS2 parameters file to use'
    )
    
    # 启动地图服务器
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': LaunchConfiguration('map')
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )
    
    # 启动AMCL
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )
    
    # 启动导航2
    navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file')
        }.items()
    )


    # 启动生命周期管理器
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'recoveries_server',
                'bt_navigator',
                'waypoint_follower',
            ]
        }]
    )
    
    # 启动RViz
#     rviz = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         arguments=['-d', PathJoinSubstitution([
#             FindPackageShare('nav2_bringup'),
#             'rviz',
#             'nav2_default_view.rviz'
#         ])],
#         parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
#         output='screen'
#     )
    
    return LaunchDescription([
        use_sim_time,
        map_file,
        params_file,
        map_server,
        amcl,      
        navigation2,
        lifecycle_manager,
    ])