# -*- coding: utf-8 -*-
from setuptools import setup, find_packages # 确保 find_packages 已导入
import os
from glob import glob

package_name = 'voice_assistant'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']), 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=[
        'setuptools',
        'baidu-aip',
        'zhipuai',
        'requests',
        'pygame',
        'smbus2', 
    ],
    zip_safe=True,
    maintainer='wang',
    maintainer_email='1693367490@qq.com',
    description='A single-node ROS2 voice assistant package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_assistant_node = voice_assistant.voice_assistant_node:main',
            'sensor_publisher_node = voice_assistant.sensor_publisher_node:main', 
            'status_aggregator_node = voice_assistant.status_aggregator_node:main',
        ],
    },
)