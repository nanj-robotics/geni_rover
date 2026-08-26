#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取 car_navigation 包路径
    pkg_dir = get_package_share_directory('car_navigation')
    
    # 地图文件
    map_file = os.path.join(pkg_dir, 'maps', 'my_map_2.yaml')
    
    # Nav2 参数文件
    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    # Nav2 bringup 路径
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'use_sim_time': 'false',
                'autostart': 'true'
            }.items()
        )
    ])
