# Don - Wheeled-Legged Robot 🤖 ROS2 Workspace

This is a comprehensive ROS2-based project for a wheeled-legged robot, featuring complete modules for robot control, navigation, perception, and human-robot interaction.

## Project Structure 📁

```
.
├── body_tracking          # Human body tracking module
├── cam_with_web           # Camera streaming with web interface
├── cmd_vel_serial         # Velocity command to serial communication
├── hand_detector          # Hand gesture detection module
├── robot_description      # Robot description files and configurations
├── robot_navigation2      # Robot navigation system
├── rplidar_ros            # RPLIDAR driver for ROS2
├── voice_assistant        # Voice assistant with sensor integration
└── wit_ros2_imu           # IMU sensor module
```

## Module Descriptions 🧩

### body_tracking 🎯
- Implements human pose recognition and tracking functionality
- Contains robot control nodes and parameter management nodes
- Provides multiple launch configuration options

### cam_with_web 📷
- Streams camera images through a web interface
- Enables remote monitoring of robot's visual information

### cmd_vel_serial 🔄
- Converts ROS2 `cmd_vel` messages to serial commands
- Facilitates communication with low-level hardware

### hand_detector ✋
- Vision-based hand gesture recognition
- Provides non-contact interaction methods

### robot_description 📐
- URDF model files
- SLAM and EKF configuration parameters
- Automatic map saving functionality

### robot_navigation2 🧭
- Complete navigation functionality based on Navigation2
- Includes map files and navigation parameter configurations

### rplidar_ros 📡
- ROS2 driver for RPLIDAR series LiDAR sensors
- Supports various RPLIDAR device models

### voice_assistant 🗣️
- Environmental monitoring with multiple sensors
- Voice interaction capabilities
- Status information aggregation and publishing

### wit_ros2_imu 🧭
- ROS2 driver for Wit IMU sensors
- Provides attitude and motion information

## System Requirements 💻

- Ubuntu 20.04 or higher 🐧
- ROS2 Foxy/Focal or higher 🤖
- Python 3.8+ 🐍
- Related hardware (wheeled-legged robot platform, sensors, etc.) ⚙️

## Installation 🔧

```bash
# Clone the project
cd ~/ros2_ws/src
git clone <repository-url>

# Install dependencies
rosdep install --from-paths . --ignore-src -r -y

# Build the workspace
cd ..
colcon build --symlink-install
```

## Usage ▶️

1. Launch core robot functionalities:
```bash
source install/setup.bash
ros2 launch robot_description display.launch.py
```

2. Start navigation system:
```bash
ros2 launch robot_navigation2 bringup_launch.py
```

3. Launch human body tracking:
```bash
ros2 launch body_tracking body_tracking.launch.py
```

## Hardware Connections 🔌

- Ensure all serial devices are properly connected and have correct permissions
- IMU, LiDAR, and other sensors connected via USB or serial interfaces
- Camera devices with proper driver installation

## Important Notes ⚠️

- Configure udev rules for hardware access permissions on first use
- Some modules may require parameter adjustments based on actual hardware
- Testing recommended in safe environments

## Contributing 🤝

Feel free to submit Issues and Pull Requests to improve project functionality.

## License 📄

Please refer to the LICENSE files in each module directory.
