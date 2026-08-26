import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 引用底层驱动启动文件
    combined_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('car_motor_control'),
                'launch',
                'combined.launch.py'
            )
        )
    )

    # SLAM Toolbox 节点
    slam_params = os.path.join(
        get_package_share_directory('car_navigation'),
        'config',
        'mapper_params_online_async.yaml'
    )
    
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params]
    )

    return LaunchDescription([
        combined_launch,
        slam_node
    ])
