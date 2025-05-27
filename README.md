**Motor Driver ROS2 Package**

A ROS2 package for interfacing with motor controllers via **serial communication**. This package handles velocity commands, encoder feedback, and publishes odometry data.
Features
- Serial communication with encoder and STM32 microcontroller
- Velocity command handling via ROS2 topics
- Odometry calculation and publishing
- TF2 transform broadcasting
- Configurable parameters via ROS2 parameter system
Dependencies
- ROS2 Humble
- CppLinuxSerial https://github.com/gbmhunter/CppLinuxSerial
- Standard ROS2 packages (rclcpp, std_msgs, nav_msgs, tf2, etc.)

**Installation**:
# Clone the repository into your ROS2 workspace
cd ~/your_ros2_workspace/src
git clone https://github.com/Phat-Hust/motor_driver.git

# Install essential dependencies
sudo apt install ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs

# Build the package
cd ~/your_ros2_workspace
colcon build --packages-select motor_driver

**Usage**:
ros2 launch motor_driver motor_driver.launch.py

**Parameters**:
Configure the node using the parameters in  config/params.yaml:
- portname_encoder: Serial port for encoder (default: /dev/ttyUSB0)
- portname_stm32: Serial port for STM32 (default: /dev/ttyUSB1)
- baudrate: Serial communication baudrate (default: 115200)
- wheel_radius: Wheel radius in meters (default: 0.05)
- wheel_seperation: Distance between wheels in meters (default: 0.325)
- ticks_per_revolution: Encoder ticks per wheel revolution (default: 1600)
- publish_tf: Whether to publish TF transforms (default: true)

**Topics**
Subscribed Topics
- cmd_vel_nav (geometry_msgs/Twist): Velocity commands
Published Topics
- odom (nav_msgs/Odometry): Odometry information

Maintainer:
- Phat Hoang phattphatt2811@gmail.com
