# GeniRover: Autonomous Navigation & Obstacle-Avoidance Differential-Drive Mobile Robot

<div align="center">
  <img src="./assets/geni_rover_photo.png" height="300" style="margin-right:16px;" />
  <img src="./assets/GeniRover.PNG" height="300" />
  <p>GeniRover</p>
</div>

A four-wheel differential-drive mobile robot for autonomous SLAM mapping, localization, path planning and real-time obstacle avoidance. Perception uses a 2D LiDAR and an RGB-D camera for environment sensing, an IMU for attitude estimation, and CAN-controlled hub motors for motion. Wheel odometry is fused with IMU via an EKF for robust state estimation. Developed on a laptop and deployed on an NVIDIA Jetson AGX Orin.

## Hardware

| Component | Details |
|---|---|
| Robot Platform | GeniRover four-wheel differential-drive robot |
| Hub Motors | CAN-controlled in-wheel hub motors, gear ratio 5.2:1 |
| Motor Controller | USB-CAN II (dual-channel: front & rear axles) |
| 2D LiDAR | Wheeltec LiDAR |
| RGB-D Camera | Orbbec Gemini 336L |
| IMU | Yahboom 9-axis IMU |
| Embedded Host | NVIDIA Jetson AGX Orin (Ubuntu 22.04, ROS2 Humble) |

## System Pipeline

```
2D LiDAR (/scan)              IMU (/imu/data_raw)
     │                                │
     ▼                                ▼
SLAM Toolbox                  Madgwick Filter
(online mapping)              (orientation)
     │                                │
     ▼                                ▼
  /map                           /imu/data
     │                                │
     └────────────┬───────────────────┘
                  ▼
         robot_localization EKF
      (fuse wheel odom vx + IMU yaw)
                  │
                  ▼
         /odometry/filtered
          (odom → base_footprint TF)
                  │
                  ▼
              Nav2 Stack
   (AMCL + Global Planner + DWB controller
    + costmap obstacle/inflation layers)
                  │
                  ▼
             /cmd_vel (Twist)
                  │
                  ▼
            diff_odom_node
     (dual‑channel CAN motor control
      + wheel odometry from RPM feedback)
                  │
                  ▼
           Four hub motors → motion
```

## Repository Structure

```
geni_rover/
├── car_description/        # URDF robot model (4-wheel diff-drive, sensors) & RViz display
├── car_motor_control/      # Motor driver node (USB-CAN II, diff kinematics, wheel odometry),
│                           # EKF config, combined bringup launch
├── car_navigation/         # SLAM Toolbox config & launch, Nav2 params & bringup, pre-built maps
├── imu_ros2_device/        # Yahboom IMU driver
├── lslidar_driver/         # Wheeltec LiDAR driver
├── lslidar_msgs/           # Custom LiDAR ROS2 messages
└── wheeltec_udev.sh        # udev rules for stable USB device symlinks
```

## Environment

- Ubuntu 22.04 LTS
- ROS2 Humble (running in Docker on Jetson AGX Orin)
- Python 3.10
- NVIDIA Container Toolkit (GPU passthrough on Jetson)

## Notes

- **Motor control**: Four hub motors driven via dual-channel USB-CAN II (CH0 front, CH1 rear). Wheel odometry uses averaged RPM feedback from both axles.
- **EKF fusion**: Raw wheel odometry (`/odom`) is fused with IMU yaw rate by `robot_localization`, outputting `/odometry/filtered` and broadcasting `odom → base_footprint` TF.
- **Docker deployment on Jetson**: The entire ROS2 stack runs inside a Docker container on the Jetson AGX Orin. USB devices (CAN adapter, IMU, LiDAR) are passed through via `--device` flags, NVIDIA Container Toolkit enables GPU access.
- **Workflow**: Run `slam.launch.py` to map, save map, then run `combined.launch.py` + `nav2_bringup.launch.py` for autonomous navigation.

## Build

```bash
mkdir -p ~/geni_rover_ws/src && cd ~/geni_rover_ws/src
git clone https://gitee.com/nanj-robotics/geni_rover.git
cd ~/geni_rover_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## References

- Nav2: https://github.com/ros-navigation/navigation2
- slam_toolbox: https://github.com/SteveMacenski/slam_toolbox
- robot_localization: https://github.com/cra-ros-pkg/robot_localization
- OrbbecSDK_ROS2: https://github.com/orbbec/OrbbecSDK_ROS2
