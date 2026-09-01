"""
导航系统总启动入口
启动传感器 + EKF + Nav2 导航栈

使用方法:
  # SLAM 建图模式（默认）
  ros2 launch navigate_frame bringup.launch.py

  # AMCL 定位模式（需要已有地图）
  ros2 launch navigate_frame bringup.launch.py slam:=false map:=/path/to/map.yaml

  # 室外 RTK GPS 定位模式（无地图, 动态代价, 启动点即 map 原点）
  ros2 launch navigate_frame bringup.launch.py outdoor:=true

  # 仅传感器
  ros2 launch navigate_frame bringup_sensors.launch.py

  # 仅导航栈
  ros2 launch navigate_frame navigation.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    navigate_frame_dir = get_package_share_directory('navigate_frame')

    # ==================== Launch Arguments ====================
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'
    )
    declare_slam = DeclareLaunchArgument(
        'slam', default_value='true',
        description='Whether to run SLAM (true) or AMCL localization (false)'
    )
    declare_map = DeclareLaunchArgument(
        'map', default_value='',
        description='Full path to map yaml file for AMCL localization mode'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(navigate_frame_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack'
    )
    declare_use_respawn = DeclareLaunchArgument(
        'use_respawn', default_value='false',
        description='Whether to respawn if a node crashes'
    )
    declare_enable_depth_camera = DeclareLaunchArgument(
        'enable_depth_camera', default_value='true',
        description='Whether to launch depth camera driver + nvblox. '
                    'Set to false when camera disconnected or nvblox not needed.'
    )
    declare_outdoor = DeclareLaunchArgument(
        'outdoor', default_value='false',
        description='true=RTK GPS 室外定位层(无地图, rolling costmap); '
                    'false=室内 AMCL/SLAM(默认). 与 slam 互斥: outdoor:=true 时忽略 slam.'
    )

    # ==================== 传感器 + EKF ====================
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigate_frame_dir, 'launch', 'bringup_sensors.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_map_tf': 'false',
            'enable_depth_camera': LaunchConfiguration('enable_depth_camera'),
        }.items(),
    )

    # ==================== Nav2 导航栈 ====================
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigate_frame_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'use_respawn': LaunchConfiguration('use_respawn'),
            'enable_depth_camera': LaunchConfiguration('enable_depth_camera'),
            'outdoor': LaunchConfiguration('outdoor'),
        }.items(),
    )

    # ==================== nvblox 3D 避障（由 AMCL 收敛守卫节点按需启动）====================
    # 不再固定 t=6s 启动 nvblox。改为启动 amcl_convergence_gate 节点，
    # 该节点监听 /amcl_pose 协方差，AMCL 收敛后用 subprocess 拉起 nvblox.launch.py。
    # 这样避免 AMCL 未收敛时 nvblox 用错误位姿积分深度点 → 幽灵障碍永久记忆。
    # 定位收敛守卫节点（室内 AMCL / 室外 RTK 双源通用）启动条件：
    #   深度相机启用 + （室外 OR 室内AMCL）。SLAM 模式不启动（不用 nvblox）。
    # 室内 AMCL：slam==false and outdoor==false → gate 走 /amcl_pose 协方差判据
    # 室外 GPS：outdoor==true → gate 走 /gps/fix_rtk 连续帧判据
    gate_condition = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration('enable_depth_camera'), "' == 'true' and ",
            "(", "'", LaunchConfiguration('outdoor'), "' == 'true' or ",
            "(", "'", LaunchConfiguration('slam'), "' == 'false' and ",
            "'", LaunchConfiguration('outdoor'), "' == 'false'))"
        ])
    )
    amcl_gate_node = Node(
        package='navigate_frame',
        executable='amcl_convergence_gate.py',
        name='amcl_convergence_gate',
        condition=gate_condition,
        parameters=[{
            'position_variance_threshold': 0.1,
            'yaw_variance_threshold': 0.05,
            'convergence_count': 5,
        }],
        output='screen',
        emulate_tty=True,
    )

    # ==================== outdoor 参数与三路互斥定位层 ====================
    outdoor = LaunchConfiguration('outdoor')
    slam = LaunchConfiguration('slam')

    # 三路互斥定位层（任一时刻 map→odom 只能有一个发布者）：
    #   outdoor==true                  → GPS 室外定位层（全局 EKF 发 map→odom）
    #   outdoor==false and slam==true  → SLAM 建图（SLAM Toolbox 发 map→odom）
    #   outdoor==false and slam==false → AMCL 定位（AMCL 发 map→odom）
    slam_condition = IfCondition(PythonExpression([
        "'", slam, "' == 'true' and '", outdoor, "' == 'false'"]))
    amcl_condition = IfCondition(PythonExpression([
        "'", slam, "' == 'false' and '", outdoor, "' == 'false'"]))
    gps_condition = IfCondition(outdoor)

    # GPS 室外定位层：gps_ntrip + navsat_transform + 全局 EKF（发 map→odom）+ gps_link TF
    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigate_frame_dir, 'launch', 'gps.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=gps_condition,
    )

    # ==================== SLAM 模式 ====================
    slam_launch = GroupAction(
        condition=slam_condition,
        actions=[
            # SLAM Toolbox 在线建图
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
        emulate_tty=True,
                parameters=[
                    LaunchConfiguration('params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                ],
            ),
            # Map Saver (保存建好的地图)
            Node(
                package='nav2_map_server',
                executable='map_saver_server',
                name='map_saver',
                output='screen',
        emulate_tty=True,
                parameters=[
                    LaunchConfiguration('params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'save_map_timeout': 5.0},
                ],
            ),
            # Lifecycle manager for map_saver
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
                output='screen',
        emulate_tty=True,
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'autostart': True},
                    {'node_names': ['map_saver']},
                ],
            ),
        ],
    )

    # ==================== AMCL 定位模式 ====================
    localization_launch = GroupAction(
        condition=amcl_condition,
        actions=[
            # Map Server
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
        emulate_tty=True,
                parameters=[
                    LaunchConfiguration('params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'yaml_filename': LaunchConfiguration('map')},
                ],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                ],
            ),
            # AMCL
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
        emulate_tty=True,
                parameters=[
                    LaunchConfiguration('params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                ],
            ),
            # Lifecycle manager for localization
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
        emulate_tty=True,
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']},
                    # 放宽超时：启动期 CPU 峰值时 map_server 加载 PGM 偶发变慢，
                    # 默认超时偏紧会触发 "failed to send response... timeout"，
                    # 连锁导致 AMCL 不激活 → map→odom 不发 → map 帧不存在 → RViz 无法设初始位姿。
                    {'bond_timeout': 8.0},        # 默认 4.0s，放宽到 8s
                    {'response_timeout': 5.0},    # 服务响应超时，默认偏短
                    {'attempt_configure_time': 2.0},
                ],
            ),
        ],
    )

    # 启动时序：
    #   t=0s:  传感器 + 相机 + 局部EKF + 静态TF
    #   t=3s:  定位层三选一（互斥）：SLAM / AMCL / GPS室外层（均发布 map→odom）
    #   t=6s:  Nav2 导航栈 + AMCL收敛守卫节点（gate 监听协方差，收敛后自行 subprocess 启 nvblox）
    delayed_slam = TimerAction(
        period=3.0,
        actions=[
            slam_launch,
            localization_launch,
            gps_launch,
        ],
    )
    delayed_navigation = TimerAction(
        period=6.0,
        actions=[
            navigation_launch,
            amcl_gate_node,
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam,
        declare_outdoor,
        declare_map,
        declare_params_file,
        declare_autostart,
        declare_use_respawn,
        declare_enable_depth_camera,
        # 传感器立即启动
        sensors_launch,
        # 定位层（SLAM/AMCL/GPS 三选一）延迟 3 秒启动
        delayed_slam,
        # Nav2 导航栈 + AMCL 守卫节点延迟 6 秒启动（等定位层就绪）
        delayed_navigation,
    ])

