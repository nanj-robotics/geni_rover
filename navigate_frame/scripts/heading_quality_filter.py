#!/usr/bin/env python3
"""
GPS 双天线航向质量过滤器 + 航向协方差放大

功能：
  1. 仅当双天线航向精度达到 RTK fixed（heading_status=4）时才转发 /gps/heading_imu
  2. 【关键】放大航向协方差，防止 GPS 航向微小抖动导致全局 EKF 的 map→odom yaw 缓慢漂移。
     RTK 双天线航向精度 ~0.1°（协方差 ~3e-5），但实际有 NTRIP 延迟 + 1Hz 低频更新，
     EKF 对每次微小航向变化都做修正 → yaw 缓慢漂 → 远距离导航目标偏移。
     放大协方差后 EKF 对 GPS 航向修正变温和，yaw 稳定性大幅提升。

UM982 HPR 定向状态：
  0=无效  1=自洽(单点)  2=DGPS  4=RTK fixed  5=RTK float

输入：/gps/heading_imu（Imu）+ /gps/heading_status（Int16）
输出：/gps/heading_imu_filtered（Imu，仅 RTK fixed 航向 + 放大协方差）
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16


class HeadingQualityFilter(Node):
    def __init__(self):
        super().__init__('heading_quality_filter')

        # 参数：最低航向质量等级（默认 4=RTK fixed）
        self.declare_parameter('min_heading_quality', 4)
        self.min_quality = self.get_parameter('min_heading_quality').value

        # 参数：航向协方差缩放因子（默认 1.0 = 不放大）
        # 方案 B（gps_ntrip 已用 GPS 测量时刻打戳）后，航向延迟由 robot_localization
        # 延迟测量更新消化，不再需要放大协方差降权。默认 1.0 让双天线用真实精度
        # （~1°）绝对锚定 yaw，压制 IMU 陀螺积分的长期漂移。
        # 若实测发现航向偶发跳变（多径/卫星切换），可适当放大（如 10~100）温和降权。
        self.declare_parameter('heading_cov_scale', 1.0)
        self.cov_scale = self.get_parameter('heading_cov_scale').value

        # 缓存
        self._heading_quality = -1
        self._latest_imu = None

        # 订阅
        self._sub_imu = self.create_subscription(
            Imu, '/gps/heading_imu', self._imu_cb, 10)
        self._sub_status = self.create_subscription(
            Int16, '/gps/heading_status', self._status_cb, 10)

        # 发布
        self._pub = self.create_publisher(
            Imu, '/gps/heading_imu_filtered', 10)

        self.get_logger().info(
            f'航向质量过滤：仅 status>={self.min_quality} 转发'
            f' + 协方差放大×{self.cov_scale}')

    def _imu_cb(self, msg: Imu):
        """缓存航向，检查质量，按条件发布"""
        self._latest_imu = msg
        if self._heading_quality >= self.min_quality:
            self._publish(msg)
        else:
            self.get_logger().info(
                f'航向质量={self._heading_quality}(<{self.min_quality})，滤掉不发布',
                throttle_duration_sec=10.0)

    def _status_cb(self, msg: Int16):
        """缓存质量，如果状态变为合格，立即发布已有航向"""
        prev = self._heading_quality
        self._heading_quality = msg.data
        if prev < self.min_quality and self._heading_quality >= self.min_quality and self._latest_imu:
            self.get_logger().warn(
                f'航向质量 {prev}→{self._heading_quality}，开始发布')
            self._publish(self._latest_imu)

    def _publish(self, msg: Imu):
        """放大 yaw 协方差后发布"""
        # orientation_covariance[8] = yaw×yaw（3x3 矩阵最后一个对角元素）
        msg.orientation_covariance[0] *= self.cov_scale  # roll（一般为0，放大无害）
        msg.orientation_covariance[4] *= self.cov_scale  # pitch
        msg.orientation_covariance[8] *= self.cov_scale  # yaw（关键）
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingQualityFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
