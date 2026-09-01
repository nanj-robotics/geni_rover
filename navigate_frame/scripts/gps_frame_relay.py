#!/usr/bin/env python3
"""
GPS frame_id 中继（打破 EKF 反馈环）+ 可选杠杆臂补偿

背景（方案 B：gps_ntrip 已改用 GPS 测量时刻打戳）：
  /gps/fix、/gps/heading_imu 的时间戳已是 GPS 测量时刻（不再是接收时刻）。
  robot_localization 的 EKF 据此做"延迟测量更新"：buffer 回溯到测量时刻
  做更新 → 前向平滑补回运动 → 运动中不再回跳。延迟不再靠"降权"躲避，
  而是靠时间戳 + EKF 延迟更新正确消化。

  本节点职责因此简化为：
    0. RTK status 过滤：仅转发 RTK(GBAS_FIX) 解的 GPS，避免 SPS 首帧锚定错误 datum。
    1. 把 /odometry/gps 的 frame_id 从 "odom" 改为 "map"（打破全局 EKF 的
       map→odom 反馈环：navsat 输出 odom 帧，全局 EKF 若用自己发的 map→odom
       变换去融合会自洽循环 → 永远收敛不了；改 frame_id 到 map 打破）。
    2. 时间戳透传（不打戳、不偏移）——gps_ntrip 源头已正确，动了反而错。
    3. 协方差不再放大（cov_scale 默认 1.0），恢复 GPS 真实 RTK 精度，
       让 EKF 正确信任 GPS（在测量时刻）。
    4. 可选杠杆臂补偿（GPS 天线 → base_link 中心），用【GPS 测量时刻】的
       航向查 TF，保证位置与航向时间一致（避免转向期间画弧）。

杠杆臂配置（与 gps.launch.py 的 base_link→gps_link TF 保持一致）：
  lever_x / lever_y：天线相位中心相对 base_link 的偏移（默认 0，即关闭效果）。
  enable_lever_compensation：默认 False（天线接近 base_link 正上方时无需补偿）。

协方差缩放（cov_scale，默认 1.0 = 不放大）：
  方案 B 后默认 1.0，让 GPS 用真实协方差（RTK fixed ~1cm，协方差 ~8e-5）。
  仅作微调保留：若实测发现 GPS 偶发跳变（多径/卫星几何切换），可适当放大
  （如 10~100）做温和降权，但不要回到之前的大值（会废掉 GPS 长时校正能力）。
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
import tf2_ros


class GpsFrameRelay(Node):
    def __init__(self):
        super().__init__('gps_frame_relay')

        # ---- 杠杆臂参数（与 gps.launch.py base_link→gps_link TF --x --y 一致）
        # 修改前（前后安装，主天线在中心）：lever_x=0, lever_y=0
        # 修改后（左右侧装，主天线在车尾左侧）：实测值，需校准
        # 改为侧装后主天线偏离 base_link 中心，enable=True 可补偿旋转时的位置画弧。
        # 先在航向验证正确后再开，不开也功能正常（cov_scale=1.0 下协方差可吸收小杠杆臂误差）。
        #self.declare_parameter('lever_x', -0.8)   # 主天线在 base_link 后方(-X)
        #self.declare_parameter('lever_y', 0.5)    # 主天线在 base_link 左侧(+Y)
        self.declare_parameter('lever_x', 0)   # 主天线在 base_link 后方(-X)
        self.declare_parameter('lever_y', 0)    # 主天线在 base_link 左侧(+Y)
        self.declare_parameter('enable_lever_compensation', False)
        self.lever_x = self.get_parameter('lever_x').value
        self.lever_y = self.get_parameter('lever_y').value
        self.enable_lever = self.get_parameter('enable_lever_compensation').value

        # ---- 协方差缩放（默认 1.0 = 不放大，方案 B 后 GPS 用真实协方差）----
        self.declare_parameter('cov_scale', 1.0)
        self.cov_scale = self.get_parameter('cov_scale').value

        # ---- RTK status 过滤：仅转发 RTK(GBAS_FIX) 解，避免 SPS 首帧锚定错误 datum ----
        self.declare_parameter('require_rtk', True)
        self.require_rtk = self.get_parameter('require_rtk').value
        self._latest_fix_status = NavSatStatus.STATUS_NO_FIX  # 缓存最新 fix 定位质量
        self._sub_fix = self.create_subscription(NavSatFix, '/gps/fix', self._fix_cb, 10)

        # ---- TF2：杠杆臂补偿用，查 GPS 测量时刻的 odom→base_link 航向 ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- GPS 订阅 + 改写后发布 ----
        self.sub = self.create_subscription(Odometry, '/odometry/gps', self.cb, 10)
        self.pub = self.create_publisher(Odometry, '/odometry/gps_mapframe', 10)

        self.get_logger().info(
            f'/odometry/gps(odom) → /odometry/gps_mapframe(map)'
            f' + RTK过滤(require={self.require_rtk})'
            f' + 杠杆臂[{self.lever_x},{self.lever_y}](enable={self.enable_lever})'
            f' + cov_scale×{self.cov_scale}')

    def _fix_cb(self, msg: NavSatFix):
        """缓存最新 GPS fix 的定位质量 status。"""
        self._latest_fix_status = msg.status.status

    def cb(self, msg: Odometry):
        # 0. RTK status 过滤：非 RTK(GBAS_FIX) 不转发。
        #    navsat wait_for_datum=false 在首帧 GPS 锚定 map 原点，若首帧是 SPS(米级)，
        #    datum 带米级误差且不再更新。过滤掉非 RTK 解，确保 datum 锚在 RTK 时刻。
        if self.require_rtk and self._latest_fix_status != NavSatStatus.STATUS_GBAS_FIX:
            self.get_logger().warn(
                f'GPS 非 RTK(status={self._latest_fix_status})，丢弃不转发',
                throttle_duration_sec=5.0)
            return

        # 1. 改 frame_id：odom → map（打破全局 EKF 的 map→odom 反馈环）
        msg.header.frame_id = 'map'
        if not msg.child_frame_id:
            msg.child_frame_id = 'base_link'

        # 2. 时间戳透传（gps_ntrip 已打 GPS 测量时刻，此处不动）。
        #    EKF 据此做延迟测量更新：回溯到该时刻更新 → 前向平滑 → 不回跳。

        # 3. 杠杆臂补偿（可选，默认关闭）：用 GPS 测量时刻的航向
        if self.enable_lever:
            self._compensate_lever(msg)

        # 4. 协方差缩放（默认 1.0 = 不放大，仅 cov_scale != 1.0 时生效）
        if self.cov_scale != 1.0:
            self._scale_covariance(msg)

        self.pub.publish(msg)

    def _compensate_lever(self, msg: Odometry):
        """用【GPS 测量时刻】的航向旋转杠杆臂，从 GPS 位置扣除。

        关键：时间戳已是 GPS 测量时刻（gps_ntrip 已改），用 msg.header.stamp
        查该时刻的 odom→base_link 航向，保证"位置"与"航向"在同一时刻，
        避免车转向期间用当前航向去补偿过去位置导致画弧。

        航向用 odom→base_link（局部 EKF，50Hz 实时），不用 map→base_link
        （全局 EKF 靠 GPS 1Hz 修正，旋转时滞后）。base_link 自身朝向在 odom
        系与 map 系里相同（two_d_mode 下 map→odom 只在平面内平移+旋转），
        故 lever 补偿用 odom 帧航向正确。
        """
        try:
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            t = self.tf_buffer.lookup_transform('odom', 'base_link', stamp)
            q = t.transform.rotation
            yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)

            rx = self.lever_x * math.cos(yaw) - self.lever_y * math.sin(yaw)
            ry = self.lever_x * math.sin(yaw) + self.lever_y * math.cos(yaw)

            msg.pose.pose.position.x -= rx
            msg.pose.pose.position.y -= ry
        except (tf2_ros.TransformException, Exception) as e:
            self.get_logger().warn(
                f'杠杆臂补偿跳过（TF 未就绪）: {e}', throttle_duration_sec=5.0)

    def _scale_covariance(self, msg: Odometry):
        """缩放 X/Y 位置协方差（仅 cov_scale != 1.0 时调用）。"""
        cov = msg.pose.covariance  # 6x6 = 36 元素，对角线 [0]=X, [7]=Y
        cov[0] *= self.cov_scale   # X 位置方差
        cov[7] *= self.cov_scale   # Y 位置方差

    @staticmethod
    def _quat_to_yaw(x, y, z, w):
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main(args=None):
    rclpy.init(args=args)
    node = GpsFrameRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
