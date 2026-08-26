# Geni-Rover: Autonomous Navigation & Obstacle-Avoidance Differential-Drive Mobile Robot (ROS2)
A four-wheel differential-drive mobile robot for autonomous SLAM mapping,
localization, path planning and real-time obstacle avoidance. Perception uses
a 2D LiDAR for mapping and obstacle detection, an IMU for attitude estimation,
and CAN-controlled hub motors for motion. Wheel odometry is fused with IMU via
an EKF for robust state estimation. Developed on a laptop and deployed on an
NVIDIA Jetson AGX Orin.

## Hardware
| Component | Details |
|---|---|
| Robot Platform | Geni-Rover four-wheel differential-drive robot |
| Hub Motors | CAN-controlled in-wheel hub motors, gear ratio 5.2:1 |
| Motor Controller | USB-CAN II (dual-channel: front & rear axles) |
| 2D LiDAR | Wheeltec LSM10P (UART) |
| RGB-D Camera | Orbbec Gemini 336L (reserved; driver not yet integrated) |
| IMU | YaboSmart 9-axis IMU (UART) |
| Embedded Host | NVIDIA Jetson AGX Orin (Ubuntu 22.04, ROS2 Humble) |

## System Pipeline
2D LiDAR (/scan)          IMU (/imu/data_raw)
│                          │
▼                          ▼
SLAM Toolbox            Madgwick Filter
(online mapping)        (orientation)
│                          │
▼                          ▼
/map                     /imu/data
│                          │
└──────────┬───────────────┘
▼
robot_localization EKF
(fuse wheel odom + IMU yaw)
│
▼
/odometry/filtered
│
▼
Nav2 Stack
(AMCL + Navfn planner + DWB controller

- costmap obstacle/inflation layers)
│
▼
/cmd_vel (Twist)
│
▼
diff_odom_node
(dual-channel CAN motor control
  - wheel odometry from RPM feedback)
  │
  ▼
  Four hub motors → motion


## Repository Structure
geni_rover/
├── car_description/        # URDF robot model (4-wheel diff-drive, sensors) & RViz display
├── car_motor_control/      # Motor driver node (USB-CAN II, diff kinematics, wheel odometry),
│                           # EKF config (robot_localization), combined bringup launch
├── car_navigation/         # SLAM Toolbox config & launch, Nav2 params & bringup, pre-built maps
├── imu_ros2_device/        # YaboSmart IMU serial driver (accel/gyro/mag/baro/euler publisher)
├── lslidar_driver/         # Wheeltec LiDAR driver (C++), supports LSM10/LSM10P/LSN10 (net & UART)
├── lslidar_msgs/           # Custom LiDAR ROS2 messages (packet, point, scan, sweep)
└── wheeltec_udev.sh        # udev rules for stable USB device symlinks


## Environment
- Ubuntu 22.04 LTS
- ROS2 Humble
- Nav2, slam_toolbox, robot_localization, imu_filter_madgwick
- Python 3.10 (system Python)
- libcontrolcan.so (USB-CAN II driver)
- YbImuLib (IMU serial library)

## Notes
- **Motor control**: Four hub motors driven via dual-channel USB-CAN II (CH0 front, CH1 rear).
  Wheel odometry uses averaged RPM feedback from both axles, not open-loop commands.
- **EKF fusion**: Raw wheel odometry (`/odom`) is fused with IMU yaw rate by `robot_localization`,
  outputting `/odometry/filtered` and broadcasting `odom → base_footprint` TF.
- **Workflow**: Run `slam.launch.py` to map, save map with `map_saver_cli`, then run
  `combined.launch.py` + `nav2_bringup.launch.py` for autonomous navigation.

## Build
```bash
mkdir -p ~/geni_rover_ws/src && cd ~/geni_rover_ws/src
git clone https://gitee.com/nanj-robotics/geni_rover.git
cd ~/geni_rover_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

## References
- Nav2: [https://docs.ros.org/en/humble/Tutorials/Navigation2](https://docs.ros.org/en/humble/Tutorials/Navigation2)
- slam_toolbox: [https://github.com/SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- robot_localization: [https://github.com/cra-ros-pkg/robot_localization](https://github.com/cra-ros-pkg/robot_localization)
- OrbbecSDK_ROS2: [https://github.com/orbbec/OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2)
- Wheeltec LiDAR: [https://github.com/wheeltec/lslidar_ros](https://github.com/wheeltec/lslidar_ros)