#!/usr/bin/env python3
# coding=utf-8
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
import threading
import serial
import socket
import time
import base64
import math
from datetime import datetime, timezone, timedelta

from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from std_msgs.msg import Float64, Int16
from geometry_msgs.msg import TwistStamped


# 串口通信相关
port = "/dev/ttyUSB0"
baudrate = 115200
AskGGA = "GPGGA 1\r\n"
# 启动时请求输出的 NMEA 语句（和芯星通 UM982 命令格式：<消息名> <频率Hz>）
# GGA=定位, HDT=双天线真航向(无状态), HPR=航向+俯仰+横滚+定向解状态, VTG=地速/航迹向, GST=位置误差统计
STARTUP_CMDS = ["GPGGA 1", "GPHDT 1", "GPHPR 1", "GPVTG 1", "GPGST 1"]

# NTRIP客户端相关
ntrip_server = "103.143.19.54"
ntrip_port = 8002
ntrip_username = "rtk2406"
ntrip_password = "55589"
ntrip_mountpoint = "RTCM33GRCEJpro"  # 注意：无下划线（厂家常写成 RTCM33_GRCEJpro 仅为可读）

sp = None
ntrip_socket = None
ntrip_is_connect = False
gga_rx_flag = False
gga_rx_data = ""

# 来自 GST 的最新位置误差（米），用于 NavSatFix 协方差；无数据时为 None
gst_lat_std = None
gst_lon_std = None
gst_alt_std = None

# 航向先验标准差（弧度）。双天线定向精度与基线长度相关，约 0.1~1°，按实际调整。
HEADING_STD_RAD = math.radians(1.0)

node = None  # rclpy Node，在 main() 中创建，供各函数 get_logger()/get_clock() 使用

# 发布者（在 main() 中创建）
pub_fix = None
pub_heading = None
pub_heading_imu = None
pub_vel = None
pub_heading_status = None

# 最近一次 $GNHPR 收到的时间（time.time()），用于让 HDT 在 HPR 在用时不重复发布航向
last_hpr_time = 0.0
# 最近一次定向解状态（0/1/2/4/5），仅供心跳日志展示
last_heading_status = -1


def send_gpgga_to_gps():
    global sp
    global gga_rx_flag
    if gga_rx_flag is True:
        gga_rx_flag = False
        return
    try:
        _serial_write(AskGGA.encode('utf-8'))
    except Exception as e:
        node.get_logger().error(f"Failed to send GPGGA to GPS: {e}")


def send_gpgga_to_ntrip():
    global ntrip_socket
    if ntrip_socket is None:
        return
    try:
        ntrip_socket.send(gga_rx_data.encode('utf-8') + b"\r\n")
    except Exception:
        return


def _fields(data):
    """剥掉 NMEA 行尾 *XX 校验和后按逗号切分。"""
    return data.split('*')[0].split(',')


def nmea_to_deg(value, hemi):
    """ddmm.mmmm + N/S/E/W -> 十进制度；非法返回 None。"""
    if not value or not hemi:
        return None
    try:
        v = float(value)
    except ValueError:
        return None
    deg = int(v / 100.0)
    minutes = v - deg * 100.0
    decimal = deg + minutes / 60.0
    if hemi in ('S', 'W'):
        decimal = -decimal
    return decimal


def heading_to_enu_quat(heading_deg):
    """NMEA 航向(0=北,顺时针) -> ROS ENU yaw(0=东,逆时针) 的四元数 (x,y,z,w)。"""
    yaw = math.radians(90.0 - heading_deg)
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


# 最近一次由 GPS UTC 解出的测量时间戳；供无时间字段的语句（如 VTG）复用，保持时间基准一致
last_gps_stamp = None


def _gps_stamp(hhmmss):
    """NMEA 的 hhmmss.ss（如 '073821.00'）+ 主机当前 UTC 日期 -> ROS 时间戳。
    处理 UTC 午夜翻转；解析失败返回 None。"""
    if not hhmmss:
        return None
    try:
        main, _, frac = hhmmss.partition('.')
        hh = int(main[0:2])
        mm = int(main[2:4])
        ss = int(main[4:6])
        ns = int(round(float('0.' + frac) * 1e9)) if frac else 0
        if ns >= 1000000000:  # 小数舍入进位到秒
            ns -= 1000000000
            ss += 1
    except (ValueError, IndexError):
        return None
    now = datetime.now(timezone.utc)
    try:
        dt = now.replace(hour=hh, minute=mm, second=ss, microsecond=0)
    except ValueError:
        return None
    # GPS 时分秒与主机 UTC 相差超过 12h，说明跨过午夜，日期 ±1 天
    diff = (dt - now).total_seconds()
    if diff > 43200:
        dt -= timedelta(days=1)
    elif diff < -43200:
        dt += timedelta(days=1)
    return Time(seconds=int(dt.timestamp()), nanoseconds=ns).to_msg()


def _now_stamp(gps_stamp=None):
    """取消息时间戳：优先 GPS 测量时刻，其次最近一次 GPS 时刻，最后主机当前时刻。"""
    if gps_stamp is not None:
        return gps_stamp
    if last_gps_stamp is not None:
        return last_gps_stamp
    return node.get_clock().now().to_msg()


def publish_fix(lat, lon, alt, quality, stamp=None):
    msg = NavSatFix()
    msg.header.stamp = _now_stamp(stamp)
    msg.header.frame_id = 'gps_link'
    msg.latitude = lat
    msg.longitude = lon
    msg.altitude = alt

    # 定位质量映射：0=无定位,1=单点,2=差分SBAS,3/4/5=RTK(固定/浮点)
    if quality == 0:
        msg.status.status = NavSatStatus.STATUS_NO_FIX
    elif quality == 1:
        msg.status.status = NavSatStatus.STATUS_FIX
    elif quality == 2:
        msg.status.status = NavSatStatus.STATUS_SBAS_FIX
    else:
        msg.status.status = NavSatStatus.STATUS_GBAS_FIX
    msg.status.service = (NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS |
                          NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO)

    # NavSatFix 协方差为 ENU 顺序：[0]=东(经度), [4]=北(纬度), [8]=天(高度)
    if None not in (gst_lat_std, gst_lon_std, gst_alt_std):
        c = msg.position_covariance
        c[0] = gst_lon_std ** 2
        c[4] = gst_lat_std ** 2
        c[8] = gst_alt_std ** 2
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    else:
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

    pub_fix.publish(msg)


def publish_heading(heading_deg, stamp=None):
    # 1) Float64：原始航向角（度，0=北，顺时针）。Float64 无 header，不带时间戳。
    h = Float64()
    h.data = heading_deg
    pub_heading.publish(h)

    # 2) Imu：ENU 四元数（喂 robot_localization）
    qx, qy, qz, qw = heading_to_enu_quat(heading_deg)
    imu = Imu()
    imu.header.stamp = _now_stamp(stamp)
    imu.header.frame_id = 'gps_link'
    imu.orientation.x = qx
    imu.orientation.y = qy
    imu.orientation.z = qz
    imu.orientation.w = qw
    imu.orientation_covariance[8] = HEADING_STD_RAD ** 2  # 仅 yaw 方差；roll/pitch 不提供
    pub_heading_imu.publish(imu)


def publish_vel(speed_kph):
    if speed_kph is None:
        return
    twist = TwistStamped()
    twist.header.stamp = _now_stamp()  # VTG 无 UTC，复用最近一次 GPS 测量时刻
    twist.header.frame_id = 'gps_link'
    twist.twist.linear.x = speed_kph / 3.6  # km/h -> m/s，沿车头方向
    pub_vel.publish(twist)


def parse_gga(data):
    global gga_rx_flag, gga_rx_data, last_gps_stamp
    fields = _fields(data)
    if len(fields) >= 14:
        try:
            quality = int(fields[6])
            lat = nmea_to_deg(fields[2], fields[3])
            lon = nmea_to_deg(fields[4], fields[5])
            alt = float(fields[9])
            if lat is None or lon is None:
                return
            # 原 GGA 握手逻辑（供 NTRIP 上行与首帧等待）
            gga_rx_flag = True
            gga_rx_data = data
            node.get_logger().info(gga_rx_data)
            stamp = _gps_stamp(fields[1])
            if stamp is not None:
                last_gps_stamp = stamp
            publish_fix(lat, lon, alt, quality, stamp)
        except (ValueError, IndexError):
            return


def parse_hdt(data):
    # 若 HPR（带状态的航向语句）正在输出，则跳过 HDT，避免重复发布航向
    if time.time() - last_hpr_time < 2.0:
        return
    fields = _fields(data)
    try:
        heading = float(fields[1])
    except (ValueError, IndexError):
        return
    publish_heading(heading)


def parse_hpr(data):
    """$GNHPR: UTC,航向,俯仰,横滚,解状态,... —— 航向与定向解状态的主来源。
    解状态与定位一致：0=无解,1=单点,2=差分,4=RTK固定,5=RTK浮点。"""
    global last_hpr_time, last_heading_status, last_gps_stamp
    fields = _fields(data)
    try:
        heading = float(fields[2])
        status = int(fields[5]) if len(fields) > 5 and fields[5] else -1
    except (ValueError, IndexError):
        return
    last_hpr_time = time.time()
    last_heading_status = status
    stamp = _gps_stamp(fields[1])
    if stamp is not None:
        last_gps_stamp = stamp
    publish_heading(heading, stamp)
    publish_heading_status(status)


def publish_heading_status(status):
    if pub_heading_status is None:
        return
    m = Int16()
    m.data = status
    pub_heading_status.publish(m)


def parse_vtg(data):
    fields = _fields(data)
    try:
        speed_kph = float(fields[7]) if len(fields) > 7 and fields[7] else None
    except (ValueError, IndexError):
        return
    publish_vel(speed_kph)


def parse_gst(data):
    global gst_lat_std, gst_lon_std, gst_alt_std
    fields = _fields(data)
    if len(fields) >= 9:
        try:
            gst_lat_std = float(fields[6]) if fields[6] else None
            gst_lon_std = float(fields[7]) if fields[7] else None
            gst_alt_std = float(fields[8]) if fields[8] else None
        except ValueError:
            return


def parse_gps(data):
    """NMEA 语句分发器。"""
    if "$GPGGA" in data or "$GNGGA" in data:
        parse_gga(data)
    elif "$GPHPR" in data or "$GNHPR" in data:
        parse_hpr(data)
    elif "$GPHDT" in data or "$GNHDT" in data:
        parse_hdt(data)
    elif "$GPVTG" in data or "$GNVTG" in data:
        parse_vtg(data)
    elif "$GPGST" in data or "$GNGST" in data:
        parse_gst(data)


_serial_tx_lock = threading.Lock()  # 串口写互斥：NTRIP 线程与 GGA 请求定时器都会写串口


def _serial_write(data):
    """线程安全地向串口写数据。"""
    if sp is not None and sp.is_open:
        with _serial_tx_lock:
            sp.write(data)


def _read_sentences(buffer):
    """读取串口并解析缓冲区中所有完整 NMEA 语句（$...\\r\\n）。
    UM982 会周期性发 `#VERSION,...` 等以 `#` 开头的私有消息（含 \\r\\n 但无 `$`），
    必须从 `$` 之后找行尾、并丢弃 `$` 之前的非 NMEA 字节，否则缓冲区只增不减、卡死。"""
    if sp.in_waiting > 0:
        data = sp.read(sp.in_waiting)
        if isinstance(data, str):
            data = data.encode('utf-8')
        buffer.extend(data)
    while True:
        start = buffer.find(b'$')
        if start == -1:
            buffer = bytearray()  # 无 NMEA 帧头，丢弃整段（如 #VERSION 等私有数据）
            break
        end = buffer.find(b'\r\n', start)  # 从 $ 之后找行尾
        if end == -1:
            buffer = buffer[start:]  # 保留半句等下次，丢弃之前的垃圾
            break
        parse_gps(buffer[start:end + 2].decode('utf-8', errors='ignore'))
        buffer = buffer[end + 2:]
    return buffer


def ntrip_forward_loop():
    """后台线程：连接 NTRIP caster，把 RTCM 差分转发到串口；断开自动重连。
    放在独立线程，避免阻塞主循环的串口读取与话题发布。"""
    global ntrip_socket, ntrip_is_connect
    while rclpy.ok():
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)  # 连接超时
            sock.connect((ntrip_server, ntrip_port))
            sock.settimeout(1.0)  # 读取超时

            auth_str = ntrip_username + ":" + ntrip_password
            auth_base64 = base64.b64encode(auth_str.encode('utf-8')).decode('utf-8')
            request = (
                "GET /%s HTTP/1.0\r\n"
                "User-Agent: NTRIP ntrip_client\r\n"
                "Accept:*/*\r\n"
                "Connection:close\r\n"
                "Authorization: Basic %s\r\n"
                "\r\n"
            ) % (ntrip_mountpoint, auth_base64)
            sock.send(request.encode('utf-8'))
            ntrip_socket = sock
            node.get_logger().info("NTRIP socket connected; waiting for data stream...")

            while rclpy.ok():
                try:
                    rtcm_data = sock.recv(4096)
                except socket.timeout:
                    continue
                if not rtcm_data:
                    break  # 对端关闭连接，退出内层循环去重连
                if not ntrip_is_connect and b"ICY 200 OK" in rtcm_data:
                    ntrip_is_connect = True
                    send_gpgga_to_ntrip()
                    node.get_logger().info("Connected to NTRIP server (ICY 200 OK).")
                # 仅在进入差分数据流后才写串口，避免把 HTTP 错误/sourcetable 灌进接收机
                if ntrip_is_connect:
                    _serial_write(rtcm_data)
        except Exception as e:
            node.get_logger().warn(f"NTRIP forward error (will retry): {e}")
        finally:
            ntrip_is_connect = False
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
            if ntrip_socket is sock:
                ntrip_socket = None
        # 重连前等待 ~3 秒（可被关闭打断）
        for _ in range(30):
            if not rclpy.ok():
                break
            time.sleep(0.1)


def gps_ntrip_node():
    global sp, ntrip_socket, ntrip_is_connect, gga_rx_flag, gga_rx_data
    # 节点由 main() 创建并自旋（后台执行器），这里直接使用全局 node

    # 配置串口
    sp = serial.Serial(port, baudrate, timeout=1)
    time.sleep(1)
    # 启动时请求所需 NMEA 语句输出（单条命令即可持续 1Hz 输出）
    for c in STARTUP_CMDS:
        sp.write((c + "\r\n").encode('utf-8'))
        time.sleep(0.1)

    # 第一阶段：等待首帧有效 GGA（作为打开 NTRIP 数据流的初始位置上报）
    buffer = bytearray()
    while rclpy.ok():
        buffer = _read_sentences(buffer)
        if gga_rx_flag is True:
            break
        time.sleep(1.0 / 100)

    # 启动 NTRIP 转发线程（独立线程，连接/收发 RTCM，断开自动重连）
    threading.Thread(target=ntrip_forward_loop, daemon=True).start()
    # GGA 上行定时器（保持「先有 GGA、再周期上行」时序）
    node.create_timer(3.0, send_gpgga_to_gps)
    node.create_timer(1.0, send_gpgga_to_ntrip)

    # 第二阶段：主循环只读串口 + 解析 + 发布（绝不阻塞在 socket 上）
    hb = 0
    buffer = bytearray()
    while rclpy.ok():
        buffer = _read_sentences(buffer)
        hb += 1
        if hb % 500 == 0:  # 约 5s 一次心跳，便于确认节点在运行
            node.get_logger().info(
                f"heartbeat: serial_ok={sp is not None and sp.is_open} "
                f"ntrip={'on' if ntrip_is_connect else 'off'} "
                f"heading_status={last_heading_status}"
                f"{'(HPR)' if last_hpr_time else '(HDT)'}")
        time.sleep(1.0 / 100)

    if sp is not None and sp.is_open:
        sp.close()
    if ntrip_socket is not None:
        ntrip_socket.close()


def main(args=None):
    global node, pub_fix, pub_heading, pub_heading_imu, pub_vel, pub_heading_status
    rclpy.init(args=args)
    node = Node('gps_ntrip_node')
    # 发布者
    pub_fix = node.create_publisher(NavSatFix, '/gps/fix', 10)
    pub_heading = node.create_publisher(Float64, '/gps/heading', 10)
    pub_heading_imu = node.create_publisher(Imu, '/gps/heading_imu', 10)
    pub_vel = node.create_publisher(TwistStamped, '/gps/vel', 10)
    pub_heading_status = node.create_publisher(Int16, '/gps/heading_status', 10)
    # 后台执行器：派发两个 GGA 上行定时器回调，主线程负责两段阻塞循环
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        gps_ntrip_node()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
