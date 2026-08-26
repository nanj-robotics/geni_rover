#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion  # 修正：Quaternion移到顶部导入
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from ctypes import *
import os
import math

# ---------- CAN 底层结构 ----------
VCI_USBCAN2 = 4
STATUS_OK = 1

class VCI_INIT_CONFIG(Structure):
    _fields_ = [("AccCode", c_uint),
                ("AccMask", c_uint),
                ("Reserved", c_uint),
                ("Filter", c_ubyte),
                ("Timing0", c_ubyte),
                ("Timing1", c_ubyte),
                ("Mode", c_ubyte)]

class VCI_CAN_OBJ(Structure):
    _fields_ = [("ID", c_uint),
                ("TimeStamp", c_uint),
                ("TimeFlag", c_ubyte),
                ("SendType", c_ubyte),
                ("RemoteFlag", c_ubyte),
                ("ExternFlag", c_ubyte),
                ("DataLen", c_ubyte),
                ("Data", c_ubyte*8),
                ("Reserved", c_ubyte*3)]

class FourWheelsController(Node):
    def __init__(self):
        super().__init__('four_wheels_controller')

        # ---------- 声明参数 ----------
        self.declare_parameter('gear_ratio', 5.2)
        self.declare_parameter('left_dir', -1)
        self.declare_parameter('right_dir', 1)
        self.declare_parameter('acceleration', 255)
        self.declare_parameter('can_id', 0x1801E600)
        self.declare_parameter('wheel_base', 0.512)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('can_device_type', 4)
        self.declare_parameter('can_device_index', 0)
        self.declare_parameter('can_channel_front', 0)
        self.declare_parameter('can_channel_rear', 1)
        self.declare_parameter('can_baudrate_timing0', 0x00)
        self.declare_parameter('can_baudrate_timing1', 0x1C)
        self.declare_parameter('can_acc_code', 0x80000008)
        self.declare_parameter('can_acc_mask', 0xFFFFFFFF)

        # ---------- 读取参数 ----------
        self.GEAR_RATIO = self.get_parameter('gear_ratio').value
        self.LEFT_DIR = self.get_parameter('left_dir').value
        self.RIGHT_DIR = self.get_parameter('right_dir').value
        self.ACCELERATION = self.get_parameter('acceleration').value
        self.CAN_ID = self.get_parameter('can_id').value
        self.WHEEL_BASE = self.get_parameter('wheel_base').value
        self.WHEEL_RADIUS = self.get_parameter('wheel_radius').value

        device_type = self.get_parameter('can_device_type').value
        device_idx = self.get_parameter('can_device_index').value
        ch_front = self.get_parameter('can_channel_front').value
        ch_rear = self.get_parameter('can_channel_rear').value
        timing0 = self.get_parameter('can_baudrate_timing0').value
        timing1 = self.get_parameter('can_baudrate_timing1').value
        acc_code = self.get_parameter('can_acc_code').value
        acc_mask = self.get_parameter('can_acc_mask').value

        # ---------- CAN 初始化 ----------
        lib_path = os.path.join(os.path.dirname(__file__), '../../vendor/libcontrolcan.so')
        if os.path.exists(lib_path):
            self.can_dll = cdll.LoadLibrary(lib_path)
        else:
            self.can_dll = cdll.LoadLibrary('libcontrolcan.so')

        self.device_type = device_type
        self.device_idx = device_idx

        ret = self.can_dll.VCI_OpenDevice(self.device_type, self.device_idx, 0)
        if ret != STATUS_OK:
            self.get_logger().error('打开CAN设备失败，请检查USB连接')
            rclpy.shutdown()
            return

        vci_init = VCI_INIT_CONFIG(acc_code, acc_mask, 0, 0, timing0, timing1, 0)

        ret = self.can_dll.VCI_InitCAN(self.device_type, self.device_idx, ch_front, byref(vci_init))
        if ret != STATUS_OK:
            self.get_logger().error(f'初始化CH{ch_front}失败')
            rclpy.shutdown()
            return
        ret = self.can_dll.VCI_StartCAN(self.device_type, self.device_idx, ch_front)
        if ret != STATUS_OK:
            self.get_logger().error(f'启动CH{ch_front}失败')
            rclpy.shutdown()
            return
        self.get_logger().info(f'CH{ch_front}（前轮）初始化成功')

        ret = self.can_dll.VCI_InitCAN(self.device_type, self.device_idx, ch_rear, byref(vci_init))
        if ret != STATUS_OK:
            self.get_logger().error(f'初始化CH{ch_rear}失败')
            rclpy.shutdown()
            return
        ret = self.can_dll.VCI_StartCAN(self.device_type, self.device_idx, ch_rear)
        if ret != STATUS_OK:
            self.get_logger().error(f'启动CH{ch_rear}失败')
            rclpy.shutdown()
            return
        self.get_logger().info(f'CH{ch_rear}（后轮）初始化成功')

        self.get_logger().info('双通道初始化完成')
        self.get_logger().info(f'减速比: {self.GEAR_RATIO}, 左轮方向: {self.LEFT_DIR}, 右轮方向: {self.RIGHT_DIR}')
        self.get_logger().info(f'轮距: {self.WHEEL_BASE}m, 轮半径: {self.WHEEL_RADIUS}m')

        # 准备CAN数据结构
        self.ubyte_array = c_ubyte*8
        self.ubyte_3array = c_ubyte*3
        self.reserved = self.ubyte_3array(0, 0, 0)

        self.left_target = 0
        self.right_target = 0
        
        # ---------- 轮速反馈（分别存储左前、左后、右前、右后） ----------
        self.front_left_rpm = 0.0    # 左前轮
        self.front_right_rpm = 0.0   # 右前轮
        self.rear_left_rpm = 0.0     # 左后轮
        self.rear_right_rpm = 0.0    # 右后轮
        
        # 平均后的轮速（左侧平均、右侧平均，用于里程计计算）
        self.current_left_rpm = 0.0   # (左前 + 左后) / 2
        self.current_right_rpm = 0.0  # (右前 + 右后) / 2
        
        # 前后轮计数（用于统计收到多少轮反馈）
        self.front_read_count = 0
        self.rear_read_count = 0
        self.read_count = 0

        # ---------- 里程计相关 ----------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # 当前速度指令（仅用于发送，不用作里程计积分）
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        
        # 实际速度（从电机反馈计算得出，用于里程计发布）
        self.actual_v = 0.0
        self.actual_w = 0.0
        
        # 上次积分时间
        self.last_odom_time = self.get_clock().now()

        # 发布器
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 定时器
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.read_timer = self.create_timer(0.05, self.read_rpm_callback)
        self.odom_timer = self.create_timer(0.05, self.publish_odometry)  # 20Hz

        # 订阅 /cmd_vel
        self.sub_cmd = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )
        self.get_logger().info('差速控制节点已启动，订阅 /cmd_vel (Twist)')

    # ---------- 速度回调 ----------
    def cmd_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        # 存储指令速度（仅用于发送，不用作里程计积分）
        self.cmd_v = v
        self.cmd_w = w

        # 差速计算（发送给电机）
        half_base = self.WHEEL_BASE / 2.0
        left_linear = v - w * half_base
        right_linear = v + w * half_base
        rpm_factor = 60.0 / (2 * math.pi * self.WHEEL_RADIUS)
        left_rpm = left_linear * rpm_factor
        right_rpm = right_linear * rpm_factor
        self.left_target = int(left_rpm)
        self.right_target = int(right_rpm)

    # ---------- CAN 帧构建 ----------
    def _build_can_frame(self, left_speed, right_speed):
        motor_left = int(left_speed * self.GEAR_RATIO)
        motor_right = int(right_speed * self.GEAR_RATIO)
        motor_left = motor_left * self.LEFT_DIR
        motor_right = motor_right * self.RIGHT_DIR
        motor_left = max(-30000, min(30000, motor_left))
        motor_right = max(-30000, min(30000, motor_right))

        def to_bytes(val):
            if val < 0:
                val = 0x10000 + val
            return [(val >> 8) & 0xFF, val & 0xFF]

        left_h, left_l = to_bytes(motor_left)
        right_h, right_l = to_bytes(motor_right)

        data = self.ubyte_array(
            0xC3,
            left_h, left_l,
            self.ACCELERATION,
            right_h, right_l,
            self.ACCELERATION,
            0
        )
        return VCI_CAN_OBJ(self.CAN_ID, 0, 0, 1, 0, 1, 8, data, self.reserved)

    def timer_callback(self):
        left_speed = self.left_target
        right_speed = self.right_target
        frame_front = self._build_can_frame(left_speed, right_speed)
        frame_rear = self._build_can_frame(left_speed, right_speed)

        ret0 = self.can_dll.VCI_Transmit(self.device_type, self.device_idx, 0, byref(frame_front), 1)
        ret1 = self.can_dll.VCI_Transmit(self.device_type, self.device_idx, 1, byref(frame_rear), 1)

        if ret0 != 1:
            self.get_logger().warn('CH0发送失败')
        if ret1 != 1:
            self.get_logger().warn('CH1发送失败')

    # ---------- 读取转速反馈（前后轮分别读取，然后左右平均） ----------
    def read_rpm_callback(self):
        # 读取前轮反馈（CH0）
        recv_obj_front = VCI_CAN_OBJ()
        while True:
            ret = self.can_dll.VCI_Receive(self.device_type, self.device_idx, 0, byref(recv_obj_front), 1, 0)
            if ret <= 0:
                break
            if recv_obj_front.ID == 0x1801E001:
                data = recv_obj_front.Data
                motor_left_raw = (data[2] << 8) | data[3]
                if motor_left_raw >= 0x8000:
                    motor_left_raw -= 0x10000
                motor_right_raw = (data[4] << 8) | data[5]
                if motor_right_raw >= 0x8000:
                    motor_right_raw -= 0x10000
                self.front_left_rpm = motor_left_raw / self.GEAR_RATIO * self.LEFT_DIR
                self.front_right_rpm = motor_right_raw / self.GEAR_RATIO * self.RIGHT_DIR
                self.front_read_count += 1
        
        # 读取后轮反馈（CH1）
        recv_obj_rear = VCI_CAN_OBJ()
        while True:
            ret = self.can_dll.VCI_Receive(self.device_type, self.device_idx, 1, byref(recv_obj_rear), 1, 0)
            if ret <= 0:
                break
            if recv_obj_rear.ID == 0x1801E001:
                data = recv_obj_rear.Data
                motor_left_raw = (data[2] << 8) | data[3]
                if motor_left_raw >= 0x8000:
                    motor_left_raw -= 0x10000
                motor_right_raw = (data[4] << 8) | data[5]
                if motor_right_raw >= 0x8000:
                    motor_right_raw -= 0x10000
                self.rear_left_rpm = motor_left_raw / self.GEAR_RATIO * self.LEFT_DIR
                self.rear_right_rpm = motor_right_raw / self.GEAR_RATIO * self.RIGHT_DIR
                self.rear_read_count += 1
        
        # 左右分别平均：左轮 = (左前 + 左后) / 2，右轮 = (右前 + 右后) / 2
        # 只有前后轮都有数据时才平均，否则使用已收到的数据
        if self.front_read_count > 0 and self.rear_read_count > 0:
            # 左侧平均（左前 + 左后）
            self.current_left_rpm = (self.front_left_rpm + self.rear_left_rpm) / 2.0
            # 右侧平均（右前 + 右后）
            self.current_right_rpm = (self.front_right_rpm + self.rear_right_rpm) / 2.0
        elif self.front_read_count > 0:
            # 只有前轮数据
            self.current_left_rpm = self.front_left_rpm
            self.current_right_rpm = self.front_right_rpm
        elif self.rear_read_count > 0:
            # 只有后轮数据
            self.current_left_rpm = self.rear_left_rpm
            self.current_right_rpm = self.rear_right_rpm
        
        self.read_count += 1
        
        # 修正：日志移到循环外，避免每帧重复打印
        if self.read_count % 5 == 0:
            current_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(
                f'[{current_time:.3f}s] 左轮(平均): {self.current_left_rpm:7.2f} rpm | 右轮(平均): {self.current_right_rpm:7.2f} rpm'
            )

    # ---------- 发布里程计和 TF（使用电机反馈转速） ----------
    def publish_odometry(self):
        # 计算时间步长
        current_time = self.get_clock().now()
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = current_time
        
        # 修正：dt饱和限制，既防止跳变又不丢步
        if dt > 0 and dt < 0.1:
            # 获取当前实际轮速（从CAN反馈中读取）
            left_rpm = self.current_left_rpm
            right_rpm = self.current_right_rpm
            
            # RPM 转 m/s (线速度)
            left_vel = left_rpm * 2 * math.pi / 60.0 * self.WHEEL_RADIUS
            right_vel = right_rpm * 2 * math.pi / 60.0 * self.WHEEL_RADIUS
            
            # 计算实际线速度和角速度（差速模型）
            actual_v = (left_vel + right_vel) / 2.0
            actual_w = (right_vel - left_vel) / self.WHEEL_BASE
            
            # 保存实际速度用于发布
            self.actual_v = actual_v
            self.actual_w = actual_w
            
            # 使用圆弧运动模型更新位置（更准确）
            if abs(actual_w) > 0.001:
                radius = actual_v / actual_w
                self.x += radius * (math.sin(self.theta + actual_w * dt) - math.sin(self.theta))
                self.y += radius * (-math.cos(self.theta + actual_w * dt) + math.cos(self.theta))
            else:
                # 直线运动
                self.x += actual_v * math.cos(self.theta) * dt
                self.y += actual_v * math.sin(self.theta) * dt
            
            # 更新角度
            self.theta += actual_w * dt
            # 修正：航向角归一化到 [-pi, pi]
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 1. 发布 Odometry 消息
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        # 修正：符合ROS导航标准，子坐标系改为base_footprint
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        q = self.euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation = q

        # 添加速度信息（使用实际速度）
        odom_msg.twist.twist.linear.x = self.actual_v
        odom_msg.twist.twist.angular.z = self.actual_w

        # ========== 修正：补全协方差矩阵（EKF/导航必需） ==========
        # 位姿协方差 (6x6 行优先: x, y, z, roll, pitch, yaw)
        odom_msg.pose.covariance = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.01, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.1
        ]
        # 速度协方差 (6x6 行优先: vx, vy, vz, vroll, vpitch, vyaw)
        odom_msg.twist.covariance = [
            0.005, 0.0,   0.0,   0.0,   0.0,   0.0,
            0.0,   0.05,  0.0,   0.0,   0.0,   0.0,
            0.0,   0.0,   0.01,  0.0,   0.0,   0.0,
            0.0,   0.0,   0.0,   0.01,  0.0,   0.0,
            0.0,   0.0,   0.0,   0.0,   0.01,  0.0,
            0.0,   0.0,   0.0,   0.0,   0.0,   0.005
        ]
        # ========================================================

        self.odom_pub.publish(odom_msg)

        # 2. 发布 TF: odom → base_footprint
        #t = TransformStamped()
        #t.header.stamp = current_time.to_msg()
        #t.header.frame_id = 'odom'
        #t.child_frame_id = 'base_footprint'
        #t.transform.translation.x = self.x
        #t.transform.translation.y = self.y
        #t.transform.translation.z = 0.0
        #t.transform.rotation = q
        #self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        q = Quaternion()
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def __del__(self):
        try:
            self.can_dll.VCI_CloseDevice(self.device_type, self.device_idx)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = FourWheelsController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()