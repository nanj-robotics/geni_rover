"""
nvblox 3D 避障启动文件

数据流：
  Orbbec color + aligned_depth ─→ nvblox ─→ ESDF 2D 切片 ─→ Nav2 costmap
  EKF (odom→base_link TF) ──────────────→ (位姿来源)

nvblox 订阅 Astra Mini Pro 彩色路线话题，用 EKF 的 TF 做位姿，
构建 3D 体素地图并输出 2D ESDF 切片，供 Nav2 的 nvblox_layer 代价层使用。

使用方法：
  ros2 launch navigate_frame nvblox.launch.py
（正常由 bringup.launch.py 自动编排，无需单独运行）
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigate_frame_dir = get_package_share_directory('navigate_frame')
    default_params_file = os.path.join(navigate_frame_dir, 'config', 'nvblox_params.yaml')

    declare_params_file = DeclareLaunchArgument(
        'nvblox_params_file', default_value=default_params_file,
        description='nvblox 参数文件路径'
    )

    # nvblox 节点
    # 话题映射：nvblox 内部 camera_0/* → Astra 彩色路线
    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('nvblox_params_file')],
        remappings=[
            # 深度：用与彩色像素对齐的深度图
            ('camera_0/depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('camera_0/depth/camera_info', '/camera/aligned_depth_to_color/camera_info'),
            # 彩色
            ('camera_0/color/image', '/camera/color/image_raw'),
            ('camera_0/color/camera_info', '/camera/color/camera_info'),
        ],
    )

    return LaunchDescription([
        declare_params_file,
        nvblox_node,
    ])
