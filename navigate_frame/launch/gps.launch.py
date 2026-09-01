"""
GPS 室外定位层启动文件（RTK + navsat_transform + 全局 EKF）

仅在 outdoor:=true 时由 bringup.launch.py 拉起（t=3s，与 SLAM/AMCL 并列互斥）。
室外定位分工：
  - GPS 全局 EKF 发 map→odom（替代室内 AMCL/SLAM 的全局定位角色）
  - 局部 EKF（bringup_sensors）仍发 odom→base_link（不变，控制用）
  - 激光/相机继续做 costmap 避障（不变，从"定位"转纯"避障"）

启动节点：
  1. gps_ntrip_node        RTK/NTRIP 桥接，发 /gps/fix, /gps/heading_imu, /gps/vel
  2. navsat_transform_node NavSatFix → /odometry/gps（map 系）
  3. ekf_global_node       全局 EKF，融合轮速+IMU+GPS+双天线航向，发 map→odom
  4. base_link→gps_link    RTK 天线安装位置静态 TF（需实测标定）

依赖链（线性，无环）：
  局部 EKF → /odometry/filtered → navsat → /odometry/gps → 全局 EKF → map→odom
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigate_frame_dir = get_package_share_directory('navigate_frame')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    ekf_global_params = os.path.join(navigate_frame_dir, 'config', 'ekf_global.yaml')
    navsat_params = os.path.join(navigate_frame_dir, 'config', 'navsat_transform.yaml')

    # ==================== 1. RTK/NTRIP 桥接节点 ====================
    # 串口(/dev/ttyUSB0)、NTRIP 账号硬编码在 gps_ntrip_node.py 源码内（包未参数化）。
    # respawn：USB 枚举晚或短暂断连时自动重试。
    gps_ntrip = Node(
        package='gps_ntrip_py',
        executable='gps_ntrip_node',
        name='gps_ntrip_node',
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=3.0,
    )

    # ==================== 1.2 GPS fix RTK 门控 ====================
    # 仅当定位精度=RTK fixed(STATUS_GBAS_FIX) 时才转发 /gps/fix → /gps/fix_rtk。
    # 保护 navsat_transform 的 datum：wait_for_datum=false 时用首帧 fix 锚定 datum，
    # 若首帧是 SPS（米级误差），datum 带偏差且不再更新 → 后续 RTK 位置全部偏移。
    gps_fix_gate = Node(
        package='navigate_frame',
        executable='gps_fix_rtk_gate.py',
        name='gps_fix_rtk_gate',
        output='screen',
        emulate_tty=True,
    )

    # ==================== 1.5 航向质量过滤器 ====================
    # 仅当双天线航向精度达到 RTK fixed(status=4) 时才转发，防止低精度航向
    # （自洽/浮点解）污染 navsat_transform 和全局 EKF 的航向估计。
    heading_filter = Node(
        package='navigate_frame',
        executable='heading_quality_filter.py',
        name='heading_quality_filter',
        output='screen',
        emulate_tty=True,
        parameters=[{'min_heading_quality': 4}],  # 0=无效 1=自洽 2=DGPS 4=RTKfixed 5=RTKfloat
    )

    # ==================== 2. navsat_transform_node ====================
    # 订阅局部 EKF 的 /odometry/filtered（不是全局 EKF）→ 依赖链线性无环。
    # imu 输入用过滤后的航向（仅 RTK fixed），不用相对陀螺 /imu/data。
    #   这是整套方案的精髓：用绝对航向把 GPS 的 ENU 位移正确旋进 odom/map 系。
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            navsat_params,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('imu', '/gps/heading_imu_filtered'),
            ('imu/data', '/gps/heading_imu_filtered'),
            ('gps/fix', '/gps/fix_rtk'),  # 用 RTK 门控后的 fix，保护 datum 锚定
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    # ==================== 2.5 GPS frame_id 中继 ====================
    # navsat_transform 输出 /odometry/gps(frame_id=odom)，全局 EKF 用 map→odom 变换
    # → 形成反馈环。本节点把 frame_id 改为 map，打破反馈环。
    gps_relay = Node(
        package='navigate_frame',
        executable='gps_frame_relay.py',
        name='gps_frame_relay',
        output='screen',
        emulate_tty=True,
    )

    # ==================== 3. 全局 EKF ====================
    # 输出 remap 到 /odometry/filtered_global，避让局部 EKF 的 /odometry/filtered。
    # 真正被消费的是它发的 map→odom TF；话题输出无人订阅，改名无害。
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            ekf_global_params,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odometry/filtered', 'odometry/filtered_global'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    # ==================== 4. base_link → gps_link 静态 TF ====================
    # 修改前（前后安装）：主天线在车中心、副在车尾 → 基线=-X（反方向）→ yaw=π
    # 修改后（左右侧装）：主天线在车尾左侧、副在车尾右侧 → 基线=-Y（垂直于车头）
    #   ROS REP-103: X=前 Y=左 Z=上 → 基线从左到右 = -Y → yaw=-π/2=-1.5708
    #   主天线在车尾左侧(x=-0.8,y=0.5) 需实测校准
    # ⚠️ xyz 需实测主天线相位中心相对 base_link 的位置（当前为估算值）
    tf_base_to_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_gps_link',
        arguments=['--x', '0', '--y', '0', '--z', '0.5',   # 主天线在车尾左侧(x=-0.8,y=0.5) 需实测校准
                   '--roll', '0', '--pitch', '0',
                   # yaw=-1.5708(-π/2)：gps_link X轴=基线=-Y → 旋转-90°。补偿后航向=车头方向
                   '--yaw', '3.14159',
                   '--frame-id', 'base_link', '--child-frame-id', 'gps_link'],
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        gps_ntrip,
        gps_fix_gate,
        heading_filter,
        navsat_transform,
        gps_relay,
        ekf_global,
        tf_base_to_gps,
    ])
