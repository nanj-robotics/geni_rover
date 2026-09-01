#!/usr/bin/env python3
"""
GPS fix RTK 质量门控

功能：仅当 GPS 定位精度达到 RTK fixed（status=STATUS_GBAS_FIX）时，
      才转发 /gps/fix 到 /gps/fix_rtk。

作用：保护 navsat_transform 的 datum 锚定。
      navsat 用首帧 /gps/fix 计算 datum（wait_for_datum=false），
      若首帧是 SPS（米级误差），datum 带米级偏差且不再更新。
      本节点在 navsat 上游过滤，确保 datum 只用 RTK 数据锚定。

NavSatFix.status 取值：
  -1 = STATUS_NO_FIX（无定位）
   0 = STATUS_FIX（SPS 单点，米级）
   1 = STATUS_SBAS_FIX（DGPS/SBAS，亚米级）
   2 = STATUS_GBAS_FIX（RTK fixed，厘米级）← 仅转发此等级

输入：/gps/fix（NavSatFix）
输出：/gps/fix_rtk（NavSatFix，仅 RTK fixed）
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GpsFixRtkGate(Node):
    def __init__(self):
        super().__init__('gps_fix_rtk_gate')

        self.sub = self.create_subscription(NavSatFix, '/gps/fix', self.cb, 10)
        self.pub = self.create_publisher(NavSatFix, '/gps/fix_rtk', 10)

        self.get_logger().info('GPS fix RTK 门控：仅 status=GBAS_FIX(2) 时转发 /gps/fix → /gps/fix_rtk')

    def cb(self, msg: NavSatFix):
        # 注意：STATUS_GBAS_FIX 定义在 NavSatStatus（msg.status 的类型）上，
        # 不是 NavSatFix —— 写成 NavSatFix.STATUS_GBAS_FIX 会在首帧回调抛
        # AttributeError 导致进程崩溃（曾导致 /gps/fix_rtk 无发布者、
        # nvblox 永不启动、RViz 话题显示灰色）
        if msg.status.status == NavSatStatus.STATUS_GBAS_FIX:
            self.pub.publish(msg)
        else:
            self.get_logger().info(
                f'GPS 非 RTK(status={msg.status.status})，丢弃不转发',
                throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = GpsFixRtkGate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
