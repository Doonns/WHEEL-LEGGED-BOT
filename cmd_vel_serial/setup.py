from setuptools import setup

package_name = 'cmd_vel_serial'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cmd_vel_to_serial.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='don',
    maintainer_email='wangyd@mail.dlut.edu.com',
    description='Serial velocity sender from /cmd_vel topic',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_serial = cmd_vel_serial.cmd_vel_to_serial_node:main',
        ],
    },
)
