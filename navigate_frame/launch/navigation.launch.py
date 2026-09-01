"""
Nav2 导航栈启动文件
启动 controller_server, planner_server, behavior_server, bt_navigator 等生命周期节点
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    navigate_frame_dir = get_package_share_directory('navigate_frame')

    # 默认参数文件
    default_params_file = os.path.join(navigate_frame_dir, 'config', 'nav2_params.yaml')

    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack'
    )
    declare_use_respawn = DeclareLaunchArgument(
        'use_respawn', default_value='false',
        description='Whether to respawn if a node crashes'
    )
    declare_log_level = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level'
    )
    declare_enable_depth_camera = DeclareLaunchArgument(
        'enable_depth_camera', default_value='true',
        description='Whether to enable nvblox costmap layers (sync with camera/nvblox node).'
    )
    declare_outdoor = DeclareLaunchArgument(
        'outdoor', default_value='false',
        description='true=室外 GPS 模式: global_costmap 切 rolling_window、关 static_layer.'
    )

    # 参数替换：将 use_sim_time 替换到 yaml 中
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')  # 默认 info；曾因 none 导致节点崩溃，已移除 --log-level 注入
    enable_depth_camera = LaunchConfiguration('enable_depth_camera')
    outdoor = LaunchConfiguration('outdoor')

    # 创建替换后的参数字典
    # 通过完整路径改写两处 nvblox_layer.enabled，使其与 enable_depth_camera 开关同步
    # 室外模式改写 global_costmap（与 enable_depth_camera 改写 nvblox 同模式，全点分路径）：
    #   - rolling_window = outdoor（室外 true→滚动窗口, 室内 false→非滚动用静态地图）
    #   - static_layer.enabled = (outdoor==false)（室外无 /map，关掉避免订阅告警）
    #   width/height 已在 nav2_params.yaml 设为 60，室内非 rolling 时被忽略，两边兼容。
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'local_costmap.local_costmap.ros__parameters.nvblox_layer.enabled': enable_depth_camera,
            'global_costmap.global_costmap.ros__parameters.nvblox_layer.enabled': enable_depth_camera,
            'global_costmap.global_costmap.ros__parameters.rolling_window': outdoor,
            'global_costmap.global_costmap.ros__parameters.static_layer.enabled': PythonExpression(
                ["'false' if '", outdoor, "' == 'true' else 'true'"]),
            # 室外纯深度避障：关闭激光 obstacle_layer（避障仅靠 nvblox 深度层）
            'local_costmap.local_costmap.ros__parameters.obstacle_layer.enabled': PythonExpression(
                ["'false' if '", outdoor, "' == 'true' else 'true'"]),
            'global_costmap.global_costmap.ros__parameters.obstacle_layer.enabled': PythonExpression(
                ["'false' if '", outdoor, "' == 'true' else 'true'"]),
        },
        convert_types=True
    )

    # Lifecycle nodes
    # velocity_smoother 已移除：diff_drive_controller 自带限速限加速，接管 cmd_vel
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        emulate_tty=True,
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            # 速度命令直发 /cmd_vel（can_diff_drive_hw_trackervehicle 的
            # diff_drive.launch.py remap cmd_vel_unstamped→/cmd_vel，两边对齐）
        ],
    )

    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        emulate_tty=True,
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    # Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        emulate_tty=True,
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            # 恢复行为(spin/backup/drive_on_heading)的速度命令直发 /cmd_vel
            # （can_diff_drive_hw_trackervehicle 的 diff_drive remap 已改为 /cmd_vel）
        ],
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        emulate_tty=True,
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    # Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        emulate_tty=True,
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    # Velocity Smoother 已移除：速度链路改为
    #   controller_server(/cmd_vel_nav) → diff_drive_controller → can_diff_drive_hw → CAN
    # diff_drive_controller 自带限速限加速，无需 velocity_smoother

    # Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_autostart,
        declare_use_respawn,
        declare_log_level,
        declare_enable_depth_camera,
        declare_outdoor,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
    ])

