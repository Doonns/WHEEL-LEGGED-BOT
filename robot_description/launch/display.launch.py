from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # 查找 robot_description 包路径
    robot_description_package = FindPackageShare('robot_description')

    # 查找 urdf_launch 工具包路径（用于加载 URDF）
    urdf_launch_package = FindPackageShare('urdf_launch')

    # ================== 启动参数 ==================

    # 是否启用 joint_state_publisher_gui
    jsp_gui_arg = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )
    ld.add_action(jsp_gui_arg)

    # RViz 配置文件路径
    default_rviz_config_path = PathJoinSubstitution([
        robot_description_package,
        'rviz',
        'config.rviz'
    ])
    rviz_config_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_config_path,
        description='Absolute path to RViz config file'
    )
    ld.add_action(rviz_config_arg)

    # ================== 包含 URDF 描述启动文件 ==================

    # 使用 urdf_launch 的 description.launch.py 来加载 URDF 文件
    urdf_launch_include = IncludeLaunchDescription(
        PathJoinSubstitution([urdf_launch_package, 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': 'robot_description',
            'urdf_package_path': PathJoinSubstitution(['urdf', 'robot.urdf'])
        }.items()
    )
    ld.add_action(urdf_launch_include)

    # ================== 启动 Joint State Publisher ==================

    # 根据参数决定使用哪个 joint_state_publisher
    #jsp_node = Node(
    #    package='joint_state_publisher',
    #    executable='joint_state_publisher',
    #    condition=UnlessCondition(LaunchConfiguration('jsp_gui'))
   # )
    #ld.add_action(jsp_node)

   # jsp_gui_node = Node(
     #   package='joint_state_publisher_gui',
    #    executable='joint_state_publisher_gui',
    #    condition=IfCondition(LaunchConfiguration('jsp_gui'))
   # )
  #  ld.add_action(jsp_gui_node)

    # ================== 启动 RViz2 ==================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )
    ld.add_action(rviz_node)

    return ld
