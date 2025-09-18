# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'usb_video_device',
            default_value='/dev/video0',
            description='USB camera device'),
        DeclareLaunchArgument(
            'websocket_image_topic',
            default_value='/image',
            description='Image topic to display in websocket'),

        # 启动 USB 摄像头
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_usb_cam'),
                    'launch/hobot_usb_cam.launch.py')),
            launch_arguments={
                'usb_video_device': LaunchConfiguration('usb_video_device'),
                'usb_pixel_format': 'mjpeg',
                'usb_image_width': '1280',     # 添加宽度
                'usb_image_height': '720'     # 添加高度
            }.items()
        ),

        # 启动 websocket + nginx
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('websocket'),
                    'launch/websocket.launch.py')),
            launch_arguments={
                'websocket_image_topic': LaunchConfiguration('websocket_image_topic'),
                'websocket_only_show_image': 'true'
            }.items()
        )
    ])
