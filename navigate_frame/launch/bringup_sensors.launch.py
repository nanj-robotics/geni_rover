"""
传感器启动文件
启动激光雷达、IMU、电机控制器、EKF传感器融合
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigate_frame_dir = get_package_share_directory('navigate_frame')

    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'
    )
    declare_publish_map_tf = DeclareLaunchArgument(
        'publish_map_tf', default_value='true',
        description='Whether to publish static map->odom transform. '
                    'Set to false when SLAM or AMCL is active to avoid TF conflict.'
    )
    declare_enable_depth_camera = DeclareLaunchArgument(
        'enable_depth_camera', default_value='true',
        description='Whether to launch the depth camera driver. '
                    'Set to false when camera disconnected or nvblox not needed.'
    )

    # EKF 配置文件
    ekf_params_file = os.path.join(navigate_frame_dir, 'config', 'ekf.yaml')

    # IMU filter 配置
    imu_ros2_dir = get_package_share_directory('imu_ros2_device')
    imu_filter_config = os.path.join(imu_ros2_dir, 'config', 'imu_filter_param.yaml')

    # ==================== 静态 TF 发布 ====================
    # map -> odom: 单位变换（仅调试用，SLAM/AMCL启动时必须关闭避免 TF 冲突）
    tf_map_to_odom = Node(
        condition=IfCondition(LaunchConfiguration('publish_map_tf')),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
    )

    # base_link -> laser: 雷达在车头位置 x=0.5m, z=0.35m
    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['--x', '0.8', '--y', '0', '--z', '0.5',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser'],
    )

    # base_link -> imu_link: IMU在车尾位置 x=-0.25m, z=0.3m
    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=['--x', '-0.8', '--y', '0', '--z', '0.5',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
    )

    # ==================== 激光雷达 (LSLIDAR M10P, 串口) ====================
    lslidar_dir = get_package_share_directory('lslidar_driver')
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lslidar_dir, 'launch', 'lsm10p_uart_launch.py')
        ),
    )

    # ==================== IMU (亚博 YbImu) ====================
    imu_driver = Node(
        package='imu_ros2_device',
        executable='ybimu_driver',
        name='ybimu_driver',
        output='screen',
        emulate_tty=True,
    )

    # IMU Madgwick 滤波器
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        parameters=[imu_filter_config],
        emulate_tty=True,
    )

    # ==================== 电机驱动（ros2_control + diff_drive_controller）====================
    # 已从 motor_controller 节点迁移到标准 ros2_control 架构：
    #   diff_drive.launch.py 启动 controller_manager + can_diff_drive_hw 插件 + diff_drive_controller
    #   cmd_vel 链路：Nav2 controller_server(/cmd_vel_nav) → diff_drive_controller → CAN
    #   odom：diff_drive_controller 发 /diff_drive_controller/odom，由 EKF 融合 IMU 后发 odom→base_link
    # 注意：diff_drive_controller.yaml 已设 enable_odom_tf:=false，TF 仍由 EKF 发布
    # can_diff_drive_dir = get_package_share_directory('can_diff_drive_hw')
    can_diff_drive_dir = get_package_share_directory('can_diff_drive_hw_trackervehicle')
    diff_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(can_diff_drive_dir, 'launch', 'diff_drive.launch.py')
        ),
    )

    # ==================== 深度相机 (Orbbec Astra Mini Pro) ====================
    # 走彩色路线（enable_color=true, enable_ir=false, enable_aligned_depth=true）
    # 输出 color/image_raw + aligned_depth_to_color/image_raw，供 nvblox 3D 避障
    # 注意：IR 与 Color 互斥（OpenNI 设备限制），不能同时开启
    # 通过 enable_depth_camera 开关控制（相机断开时设 false，避免驱动反复报错）
    orbbec_camera_dir = get_package_share_directory('orbbec_camera')
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_camera_dir, 'launch', 'gemini_336l_isaac_nav.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('enable_depth_camera')),
    )

    # 相机安装位置：车头 x=0.5m（与激光雷达同位），高度 z=0.51m
    # 正前方朝向（rpy=0），相机内部帧（camera_link → camera_depth_frame 等）由驱动发布
    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_link',
        arguments=['--x', '0.82', '--y', '0', '--z', '0.57',
                   '--roll', '3.14159', '--pitch', '0.0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
    )

    # ==================== EKF 传感器融合 ====================
    ekf_filter = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        emulate_tty=True,
        parameters=[ekf_params_file, {'use_sim_time': False}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_publish_map_tf,
        declare_enable_depth_camera,
        # 静态TF
        tf_map_to_odom,
        tf_base_to_laser,
        tf_base_to_imu,
        tf_base_to_camera,
        # 传感器
        lidar_launch,
        imu_driver,
        imu_filter,
        # 深度相机
        camera_launch,
        # 电机驱动（ros2_control）+ EKF
        diff_drive_launch,
        ekf_filter,
    ])
