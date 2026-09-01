cmake_minimum_required(VERSION 3.8)
project(navigate_frame)

find_package(ament_cmake REQUIRED)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install AMCL 收敛守卫节点脚本（AMCL 收敛后再启动 nvblox）
install(PROGRAMS scripts/amcl_convergence_gate.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install GPS frame relay 节点脚本（把 /odometry/gps 的 frame_id 从 odom 改为 map + 杠杆臂补偿）
install(PROGRAMS scripts/gps_frame_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install GPS 航向质量过滤节点（仅 RTK fixed heading 才转发）
install(PROGRAMS scripts/heading_quality_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install GPS fix RTK 门控节点（仅 RTK fixed fix 才转发，保护 navsat datum）
install(PROGRAMS scripts/gps_fix_rtk_gate.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
