#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # ==================== 获取各包路径 ====================
    imu_pkg_share = get_package_share_directory('imu_ros2_device')
    lidar_pkg_share = get_package_share_directory('lslidar_driver')
    urdf_pkg_share = get_package_share_directory('car_description')
    motor_pkg_share = get_package_share_directory('car_motor_control')
    
    # ==================== 参数文件路径 ====================
    param_file = os.path.join(motor_pkg_share, 'config', 'diff_control_params.yaml')
    ekf_param_file = os.path.join(motor_pkg_share, 'config', 'ekf_params.yaml')
    imu_filter_config = os.path.join(imu_pkg_share, 'config', 'imu_filter_param.yaml')
    
    # ==================== URDF 文件 ====================
    urdf_file = os.path.join(urdf_pkg_share, 'urdf', 'car.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # ==================== RViz 配置文件 ====================
    # 使用您自己的 RViz 配置文件
    default_rviz_config = os.path.join(motor_pkg_share, 'rviz', 'combined.rviz')
    
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config,
        description='Absolute path to rviz config file'
    )

    return LaunchDescription([
        rviz_arg,
        
        # ==================== 0. Robot State Publisher ====================
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # ==================== 0.5. Joint State Publisher ====================
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        
        # ==================== 1. 电机驱动节点 ====================
        Node(
            package='car_motor_control',
            executable='diff_odom_node',
            name='four_wheels_controller',
            output='screen',
            parameters=[param_file]
        ),
        
        # ==================== 2. EKF 融合节点 ====================
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_param_file],
        ),

        # ==================== 3. IMU 驱动节点 ====================
        Node(
            package='imu_ros2_device',
            executable='ybimu_driver',
            name='ybimu_driver',
            output='screen'
        ),
        
        # ==================== 4. IMU 滤波节点 ====================
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[imu_filter_config]
        ),
        
        # ==================== 5. 激光雷达驱动 ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lidar_pkg_share, 'launch', 'lsm10p_uart_launch.py')
            )
        ),
        
        # ==================== 6. RViz2 ====================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')]
        ),
    ])