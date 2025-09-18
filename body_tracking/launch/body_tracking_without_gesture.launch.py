# Copyright (c) 2024，D-Robotics.
# ... (版权和许可信息)
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():

    # 1. 设置摄像头类型为 'usb'
    set_camera_type_env = SetEnvironmentVariable(
        name='CAM_TYPE',
        value='usb'
    )

    # 2. 修改人体跟随节点的速度参数
    body_tracking_node = Node(
        package='body_tracking',
        executable='body_tracking',
        output='screen',
        parameters=[
            # 删除这个旧的，错误的参数名
            # {"ai_msg_sub_topic_name": "/hobot_mono2d_body_detection"},
            {"activate_wakeup_gesture": 0},
            {"img_width": 640},
            {"img_height": 480},
            {"track_serial_lost_num_thr": 30},
            {"linear_velocity": 0.2},
            {"angular_velocity": 0.2},
            {"activate_robot_move_thr": 5},
            # 确保这里只留下正确的参数名
            {"ai_msg_topic": "/hobot_mono2d_body_detection"}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # ==================== 解决模型文件问题的关键修改 ====================
    # 我们不再手动拷贝模型，而是直接告诉 mono2d_body_detection 节点
    # 模型文件在系统里的绝对路径是什么。

    # 首先，定义模型文件的完整路径
    # ******** 关键修改：将 model_file_path 更新为正确的绝对路径 ********
    model_file_path = '/opt/tros/humble/lib/mono2d_body_detection/config/multitask_body_head_face_hand_kps_960x544.hbm'
    # *******************************************************************

    # 然后，在启动人体检测节点时，把这个路径作为参数传进去
    mono2d_body_det_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mono2d_body_detection'),
                'launch/mono2d_body_detection.launch.py')),
        # 使用 launch_arguments 传递参数
        launch_arguments={
            'smart_topic': '/hobot_mono2d_body_detection',
            'mono2d_body_pub_topic': '/hobot_mono2d_body_detection',
            # 将模型文件的绝对路径通过 'model_file_name' 参数传给节点
            'model_file_name': model_file_path
        }.items()
    )
    # =====================================================================

    # 返回所有需要启动的组件
    return LaunchDescription([
        set_camera_type_env,
        mono2d_body_det_node,
        body_tracking_node
    ])