#!/usr/bin/env python3
"""
定位收敛守卫节点（室内/室外通用）

功能：等定位稳定后再启动 nvblox，避免 nvblox 在定位未收敛（map→odom 跳变）
      时用错误位姿积分深度点，导致"幽灵障碍"被永久记忆进 TSDF。

双源自动适配（无需传模式参数，哪个来数据走哪个）：
  - 室内 AMCL：监听 /amcl_pose 协方差，位置+航向方差连续 convergence_count 次达标
  - 室外 RTK ：监听 /gps/fix_rtk（gps_fix_rtk_gate 仅定位 fixed/float 才转发）+
              /gps/heading_status（定向解状态），要求【定位有数据 且 定向 status≥
              min_heading_quality】同时达标，连续 rtk_convergence_count 帧才启动。
              定向判据 >=4：fixed(4) 或 float(5) 都算达标（现场定向常在 4/5 间跳动，
              float ~1° 精度对 nvblox 够用）；掉到 DGPS(2) 以下才重置计数。

任一判据达标即启动 nvblox。两个判据独立计数，互不干扰。

启动 nvblox：subprocess 调 ros2 launch navigate_frame nvblox.launch.py，
            start_new_session=True 独立进程组，退出时 killpg 清理。

兜底：/amcl_gate/trigger_nvblox 服务（Empty），判据降不下来时可手动触发。

由 bringup.launch.py 在「深度相机启用 + (室外 OR 室内AMCL)」时拉起。
"""

import os
import signal
import subprocess
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int16
from std_srvs.srv import Empty


# covariance 数组下标（6x6 行优先）
X_IDX = 0    # x 位置方差
Y_IDX = 7    # y 位置方差
YAW_IDX = 35  # yaw 航向方差


class AMCLConvergenceGate(Node):
    def __init__(self):
        super().__init__('amcl_convergence_gate')

        # ---- 参数 ----
        self.declare_parameter('position_variance_threshold', 0.05)
        self.declare_parameter('yaw_variance_threshold', 0.05)
        self.declare_parameter('convergence_count', 5)
        self.declare_parameter('rtk_convergence_count', 5)  # 室外：连续多少帧"定位+定向同时达标"算稳定
        self.declare_parameter('min_heading_quality', 4)    # 室外：定向最低质量（4=fixed,5=float 都≥4 即达标）
        self.declare_parameter('nvblox_launch_file', 'nvblox.launch.py')

        self.pos_thresh = self.get_parameter('position_variance_threshold').value
        self.yaw_thresh = self.get_parameter('yaw_variance_threshold').value
        self.need_count = int(self.get_parameter('convergence_count').value)
        self.rtk_need_count = int(self.get_parameter('rtk_convergence_count').value)
        self.min_heading = int(self.get_parameter('min_heading_quality').value)
        self.nvblox_launch_file = self.get_parameter('nvblox_launch_file').value

        # ---- 状态 ----
        self.amcl_hit_count = 0     # AMCL 连续达标计数
        self.rtk_hit_count = 0      # RTK 定位+定向同时达标 连续计数
        self.heading_quality = -1   # 最新定向质量（缓存 /gps/heading_status）
        self.converged = False      # 是否已启动 nvblox（防重复）
        self.nvblox_proc = None     # subprocess.Popen 句柄
        self.lock = threading.Lock()

        # ---- 订阅（双源，哪个来数据走哪个）----
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, 10)
        self.create_subscription(
            NavSatFix, '/gps/fix_rtk', self.rtk_cb, 10)
        self.create_subscription(
            Int16, '/gps/heading_status', self.heading_status_cb, 10)

        # ---- 手动触发服务（兜底）----
        self.create_service(Empty, '/amcl_gate/trigger_nvblox', self.trigger_cb)

        self.get_logger().info(
            f'等待定位收敛（双源）：AMCL 协方差连续 {self.need_count} 次，'
            f'或 RTK 定位(fix_rtk)+定向(status>={self.min_heading}) 同时达标 '
            f'{self.rtk_need_count} 帧。兜底服务 /amcl_gate/trigger_nvblox')

    # ============================================================
    # AMCL 协方差回调（室内判据）
    # ============================================================
    def pose_cb(self, msg: PoseWithCovarianceStamped):
        with self.lock:
            if self.converged:
                return

            x_var = msg.pose.covariance[X_IDX]
            y_var = msg.pose.covariance[Y_IDX]
            yaw_var = msg.pose.covariance[YAW_IDX]

            self.get_logger().info(
                f'AMCL 协方差 x²={x_var:.5f} y²={y_var:.5f} yaw²={yaw_var:.5f}',
                throttle_duration_sec=1.0)

            ok = (x_var < self.pos_thresh and y_var < self.pos_thresh and
                  yaw_var < self.yaw_thresh)
            if ok:
                self.amcl_hit_count += 1
                if self.amcl_hit_count >= self.need_count:
                    self.get_logger().warn(
                        f'AMCL 已收敛（连续 {self.amcl_hit_count} 次达标），启动 nvblox ...')
                    self._start_nvblox()
            else:
                if self.amcl_hit_count > 0:
                    self.amcl_hit_count = 0

    # ============================================================
    # 定向质量回调（缓存定向 status，供 RTK 判据使用）
    # ============================================================
    def heading_status_cb(self, msg: Int16):
        """缓存最新定向解状态（0=无效 1=自洽 2=DGPS 4=RTKfixed 5=RTKfloat）。"""
        with self.lock:
            self.heading_quality = msg.data

    # ============================================================
    # RTK fix 回调（室外判据：定位 fixed/float + 定向 ≥min_heading 同时达标）
    # ============================================================
    def rtk_cb(self, msg: NavSatFix):
        """收到 /gps/fix_rtk（gps_fix_rtk_gate 已确保定位是 fixed/float）时，
        同时检查定向 status。两者都达标才计数，定向不达标则重置计数。

        定向判据 >=4（fixed=4 或 float=5 都算达标）：现场定向常在 4/5 间跳动，
        float（~1°）精度对 nvblox 够用；只有掉到 DGPS(2) 以下才视为不可用重置。
        """
        with self.lock:
            if self.converged:
                return

            heading_ok = self.heading_quality >= self.min_heading
            if heading_ok:
                self.rtk_hit_count += 1
                self.get_logger().info(
                    f'RTK 达标累计 {self.rtk_hit_count}/{self.rtk_need_count} '
                    f'(定向 status={self.heading_quality})',
                    throttle_duration_sec=1.0)
                if self.rtk_hit_count >= self.rtk_need_count:
                    self.get_logger().warn(
                        f'RTK 稳定（定位+定向连续 {self.rtk_hit_count} 帧达标），启动 nvblox ...')
                    self._start_nvblox()
            else:
                # 定向未达标（掉到 DGPS 以下）：中断连续达标，等定向恢复
                if self.rtk_hit_count > 0:
                    self.get_logger().warn(
                        f'定向 status={self.heading_quality}(<{self.min_heading})，'
                        f'RTK 计数重置（{self.rtk_hit_count}→0）')
                    self.rtk_hit_count = 0

    # ============================================================
    # 手动触发服务
    # ============================================================
    def trigger_cb(self, request, response):
        with self.lock:
            if not self.converged:
                self.get_logger().warn('手动触发启动 nvblox')
                self._start_nvblox()
            else:
                self.get_logger().info('nvblox 已启动，忽略手动触发')
        return response

    # ============================================================
    # 启动 nvblox（subprocess，独立进程组）
    # ============================================================
    def _start_nvblox(self):
        if self.converged:
            return
        self.converged = True

        cmd = ['ros2', 'launch', 'navigate_frame', self.nvblox_launch_file]
        self.get_logger().info(f'执行: {" ".join(cmd)}')
        try:
            # start_new_session=True → nvblox 独立进程组，退出时用 killpg 整组清理
            self.nvblox_proc = subprocess.Popen(
                cmd, start_new_session=True,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(
                f'nvblox 已启动 (PID={self.nvblox_proc.pid})，'
                f'日志见 nvblox.launch.py 对应进程')
        except Exception as e:
            self.get_logger().error(f'启动 nvblox 失败: {e}')

    # ============================================================
    # 清理 nvblox 进程（关闭节点时调用）
    # ============================================================
    def shutdown_nvblox(self):
        if self.nvblox_proc is None:
            return
        try:
            pgid = os.getpgid(self.nvblox_proc.pid)
            os.killpg(pgid, signal.SIGTERM)
            self.nvblox_proc.wait(timeout=5.0)
            self.get_logger().info('nvblox 进程已退出')
        except subprocess.TimeoutExpired:
            self.get_logger().warn('nvblox 未响应 SIGTERM，强制 SIGKILL')
            try:
                os.killpg(os.getpgid(self.nvblox_proc.pid), signal.SIGKILL)
            except Exception:
                pass
        except ProcessLookupError:
            # 进程已退出
            pass
        except Exception as e:
            self.get_logger().error(f'清理 nvblox 时出错: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AMCLConvergenceGate()

    def _on_shutdown():
        node.shutdown_nvblox()

    rclpy.get_default_context().on_shutdown(_on_shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_nvblox()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

