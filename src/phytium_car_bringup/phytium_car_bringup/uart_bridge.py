#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
uart_bridge.py

飞腾派 ROS2 <-> STM32 串口桥接节点

协议：
下行：飞腾派 -> STM32，11 字节
  [0]    0xAA
  [1]    0x55
  [2:6]  float32 linear.x, m/s, little-endian
  [6:10] float32 angular.z, rad/s, little-endian
  [10]   XOR checksum of bytes 0~9

上行：STM32 -> 飞腾派，16 字节
  [0]     0xBB
  [1]     0x66
  [2:6]   int32 left_total_ticks, little-endian
  [6:10]  int32 right_total_ticks, little-endian
  [10:14] float32 gyro_z_deg_s, little-endian
  [14]    uint8 safety_flags/status
  [15]    XOR checksum of bytes 0~14

功能：
  1. 订阅 /cmd_vel，按固定频率发送到 STM32
  2. 读取 STM32 上行编码器累计
  3. 用左右轮编码器差分积分发布 /odom
  4. 发布 odom -> base_footprint TF
  5. 发布 /imu/data 与 /collision

注意：
  - /odom 只根据 STM32 上行编码器帧积分，不根据 /cmd_vel 自己推算
  - 这样可以避免“实车转弯但 RViz 车模不转、地图旋转”的问题
"""

import json
import math
import struct
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import serial

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster


class UartBridge(Node):
    DOWN_STX1 = 0xAA
    DOWN_STX2 = 0x55
    UP_STX1 = 0xBB
    UP_STX2 = 0x66
    DOWNLINK_LEN = 11
    UPLINK_LEN = 16

    # STM32 上行 status/safety_flags bit definition, protocol v1.1
    STATUS_ESTOP = 0x01
    STATUS_ULTRA_SLOW = 0x02
    STATUS_ULTRA_STOP = 0x04
    STATUS_PI_TIMEOUT = 0x08
    STATUS_MPU_ERROR = 0x10
    STATUS_ENCODER_ERR = 0x20
    STATUS_BT_ACTIVE = 0x40

    def __init__(self):
        super().__init__('uart_bridge')

        # -------- robot / encoder parameters --------
        self.declare_parameter('wheel_base', 0.128)          # m
        self.declare_parameter('wheel_radius', 0.0192)        # m
        self.declare_parameter('encoder_ppr', 11)
        self.declare_parameter('gear_ratio', 20.0)
        self.declare_parameter('encoder_quadrature', 4.0)

        # 如果以后发现方向不对，只改这些参数，不要改积分公式
        self.declare_parameter('swap_encoders', False)
        self.declare_parameter('left_encoder_sign', 1)
        self.declare_parameter('right_encoder_sign', 1)
        self.declare_parameter('invert_odom_yaw', False)

        # -------- serial / ROS parameters --------
        self.declare_parameter('port', '/dev/phytium_stm32')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('read_period_sec', 0.01)
        self.declare_parameter('write_period_sec', 0.05)     # 20 Hz
        self.declare_parameter('cmd_watchdog_sec', 0.5)

        self.declare_parameter('max_speed', 0.8)             # m/s
        self.declare_parameter('max_angular', 13.0)          # rad/s

        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('debug_odom', False)

        self.declare_parameter('odom_max_dt_sec', 0.35)
        self.declare_parameter('max_tick_delta', 12000)

        # covariance
        self.declare_parameter('pose_cov_xx', 0.05)
        self.declare_parameter('pose_cov_yy', 0.05)
        self.declare_parameter('pose_cov_yaw', 0.2)
        self.declare_parameter('twist_cov_vx', 0.02)
        self.declare_parameter('twist_cov_wz', 0.05)

        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.encoder_ppr = int(self.get_parameter('encoder_ppr').value)
        self.gear_ratio = float(self.get_parameter('gear_ratio').value)
        self.encoder_quadrature = float(self.get_parameter('encoder_quadrature').value)

        self.swap_encoders = bool(self.get_parameter('swap_encoders').value)
        self.left_encoder_sign = 1 if int(self.get_parameter('left_encoder_sign').value) >= 0 else -1
        self.right_encoder_sign = 1 if int(self.get_parameter('right_encoder_sign').value) >= 0 else -1
        self.invert_odom_yaw = bool(self.get_parameter('invert_odom_yaw').value)

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.read_period = float(self.get_parameter('read_period_sec').value)
        self.write_period = float(self.get_parameter('write_period_sec').value)
        self.cmd_watchdog = float(self.get_parameter('cmd_watchdog_sec').value)

        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_angular = float(self.get_parameter('max_angular').value)

        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.imu_frame_id = str(self.get_parameter('imu_frame_id').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.debug_odom = bool(self.get_parameter('debug_odom').value)

        self.odom_max_dt = float(self.get_parameter('odom_max_dt_sec').value)
        self.max_tick_delta = int(self.get_parameter('max_tick_delta').value)

        self.pose_cov_xx = float(self.get_parameter('pose_cov_xx').value)
        self.pose_cov_yy = float(self.get_parameter('pose_cov_yy').value)
        self.pose_cov_yaw = float(self.get_parameter('pose_cov_yaw').value)
        self.twist_cov_vx = float(self.get_parameter('twist_cov_vx').value)
        self.twist_cov_wz = float(self.get_parameter('twist_cov_wz').value)

        if self.read_period <= 0.0:
            self.read_period = 0.01
        if self.write_period <= 0.0:
            self.write_period = 0.05

        ticks_per_rev = self.encoder_ppr * self.gear_ratio * self.encoder_quadrature
        if ticks_per_rev <= 0:
            raise RuntimeError('encoder_ppr * gear_ratio * encoder_quadrature must be > 0')
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / ticks_per_rev

        # -------- state --------
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_left_ticks: Optional[int] = None
        self.last_right_ticks: Optional[int] = None
        self.last_odom_time = self.get_clock().now()

        self.rx_buf = bytearray()
        self.last_collision: Optional[bool] = None
        self.last_status: Optional[int] = None

        # -------- serial --------
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.0,
                write_timeout=0.1,
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except serial.SerialException as exc:
            self.get_logger().fatal(f'串口打开失败: {self.port}, {exc}')
            raise SystemExit(1)

        # -------- ROS pubs/subs --------
        odom_qos = QoSProfile(
            depth=30,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', odom_qos)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.collision_pub = self.create_publisher(Bool, '/collision', 10)
        self.stm32_status_pub = self.create_publisher(String, '/stm32/status', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        self.write_timer = self.create_timer(self.write_period, self.write_serial)
        self.read_timer = self.create_timer(self.read_period, self.read_serial)

        self.get_logger().info(
            f'UART bridge started: port={self.port}, baud={self.baudrate}, '
            f'write={self.write_period}s, read={self.read_period}s, '
            f'meters_per_tick={self.meters_per_tick:.9f}, '
            f'swap={self.swap_encoders}, Lsign={self.left_encoder_sign}, Rsign={self.right_encoder_sign}'
        )

    # ---------------- command downlink ----------------

    def cmd_callback(self, msg: Twist):
        linear = max(min(float(msg.linear.x), self.max_speed), -self.max_speed)
        angular = max(min(float(msg.angular.z), self.max_angular), -self.max_angular)

        if abs(linear) < 0.005:
            linear = 0.0
        if abs(angular) < 0.005:
            angular = 0.0

        self.target_linear = linear
        self.target_angular = angular
        self.last_cmd_time = self.get_clock().now()

    @staticmethod
    def xor_checksum(data: bytes) -> int:
        checksum = 0
        for b in data:
            checksum ^= b
        return checksum

    def make_downlink_frame(self, linear: float, angular: float) -> bytes:
        frame = bytearray([self.DOWN_STX1, self.DOWN_STX2])
        frame.extend(struct.pack('<f', float(linear)))
        frame.extend(struct.pack('<f', float(angular)))
        frame.append(self.xor_checksum(frame))
        return bytes(frame)

    def write_serial(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_watchdog:
            linear = 0.0
            angular = 0.0
        else:
            linear = self.target_linear
            angular = self.target_angular

        try:
            self.ser.write(self.make_downlink_frame(linear, angular))
        except Exception as exc:
            self.get_logger().error(f'串口发送失败: {exc}', throttle_duration_sec=2.0)

    # ---------------- uplink parsing ----------------

    def parse_uplink_frame(self, frame: bytes) -> Optional[Tuple[int, int, float, int]]:
        if len(frame) != self.UPLINK_LEN:
            return None
        if frame[0] != self.UP_STX1 or frame[1] != self.UP_STX2:
            return None
        if self.xor_checksum(frame[:-1]) != frame[-1]:
            return None

        raw_left = struct.unpack('<i', frame[2:6])[0]
        raw_right = struct.unpack('<i', frame[6:10])[0]
        gyro_z_deg_s = struct.unpack('<f', frame[10:14])[0]
        status = int(frame[14])

        if self.swap_encoders:
            raw_left, raw_right = raw_right, raw_left

        left_ticks = raw_left * self.left_encoder_sign
        right_ticks = raw_right * self.right_encoder_sign

        return left_ticks, right_ticks, gyro_z_deg_s, status

    def read_serial(self):
        try:
            waiting = self.ser.in_waiting
            if waiting:
                self.rx_buf.extend(self.ser.read(waiting))

            latest = None

            while len(self.rx_buf) >= self.UPLINK_LEN:
                # 对齐帧头
                if self.rx_buf[0] != self.UP_STX1 or self.rx_buf[1] != self.UP_STX2:
                    self.rx_buf.pop(0)
                    continue

                frame = bytes(self.rx_buf[:self.UPLINK_LEN])
                parsed = self.parse_uplink_frame(frame)
                if parsed is None:
                    # 帧头对了但校验错，丢 1 字节重新找
                    self.rx_buf.pop(0)
                    continue

                latest = parsed
                del self.rx_buf[:self.UPLINK_LEN]

            # 每次 read callback 只积分最新一帧，避免同一 stamp 积分多帧导致速度尖峰
            if latest is not None:
                left_ticks, right_ticks, gyro_z_deg_s, status = latest
                now = self.get_clock().now()
                self.publish_imu(now, gyro_z_deg_s)
                self.publish_stm32_status(now, status, gyro_z_deg_s)
                self.integrate_and_publish_odom(now, left_ticks, right_ticks)

        except Exception as exc:
            self.get_logger().error(f'串口读取/解析失败: {exc}', throttle_duration_sec=2.0)

    # ---------------- odometry ----------------

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    def integrate_and_publish_odom(self, now, left_ticks: int, right_ticks: int):
        if self.last_left_ticks is None or self.last_right_ticks is None:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.last_odom_time = now
            self.publish_odom_tf(now, 0.0, 0.0)
            return

        dt = (now - self.last_odom_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        if dt > self.odom_max_dt:
            self.get_logger().warn(
                f'odom dt too large: {dt:.3f}s, realign encoder ticks',
                throttle_duration_sec=5.0,
            )
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.last_odom_time = now
            return

        d_left_ticks = left_ticks - self.last_left_ticks
        d_right_ticks = right_ticks - self.last_right_ticks

        if abs(d_left_ticks) > self.max_tick_delta or abs(d_right_ticks) > self.max_tick_delta:
            self.get_logger().warn(
                f'encoder jump ignored: dL={d_left_ticks}, dR={d_right_ticks}',
                throttle_duration_sec=3.0,
            )
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.last_odom_time = now
            return

        left_dist = d_left_ticks * self.meters_per_tick
        right_dist = d_right_ticks * self.meters_per_tick

        delta_s = 0.5 * (left_dist + right_dist)
        delta_yaw = (right_dist - left_dist) / self.wheel_base
        if self.invert_odom_yaw:
            delta_yaw = -delta_yaw

        self.x += delta_s * math.cos(self.yaw + 0.5 * delta_yaw)
        self.y += delta_s * math.sin(self.yaw + 0.5 * delta_yaw)
        self.yaw += delta_yaw
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        vx = delta_s / dt
        wz = delta_yaw / dt

        if self.debug_odom:
            self.get_logger().info(
                f'dL={d_left_ticks} dR={d_right_ticks} '
                f'ds={delta_s:.4f} dyaw={delta_yaw:.4f} '
                f'vx={vx:.3f} wz={wz:.3f}',
                throttle_duration_sec=0.5,
            )

        self.publish_odom_tf(now, vx, wz)

        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_odom_time = now

    def publish_odom_tf(self, now, vx: float, wz: float):
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.yaw_to_quaternion(self.yaw)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz

        odom.pose.covariance[0] = self.pose_cov_xx
        odom.pose.covariance[7] = self.pose_cov_yy
        odom.pose.covariance[35] = self.pose_cov_yaw
        odom.twist.covariance[0] = self.twist_cov_vx
        odom.twist.covariance[35] = self.twist_cov_wz

        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now.to_msg()
            tf_msg.header.frame_id = self.odom_frame_id
            tf_msg.child_frame_id = self.base_frame_id
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

    # ---------------- IMU / collision ----------------

    def publish_imu(self, now, gyro_z_deg_s: float):
        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = self.imu_frame_id
        imu.angular_velocity.z = math.radians(float(gyro_z_deg_s))

        imu.angular_velocity_covariance[0] = 1e6
        imu.angular_velocity_covariance[4] = 1e6
        imu.angular_velocity_covariance[8] = 0.03

        self.imu_pub.publish(imu)

    def decode_status(self, status: int) -> dict:
        return {
            'raw': int(status) & 0xFF,
            'estop': bool(status & self.STATUS_ESTOP),
            'ultra_slow': bool(status & self.STATUS_ULTRA_SLOW),
            'ultra_stop': bool(status & self.STATUS_ULTRA_STOP),
            'pi_timeout': bool(status & self.STATUS_PI_TIMEOUT),
            'mpu_error': bool(status & self.STATUS_MPU_ERROR),
            'encoder_error': bool(status & self.STATUS_ENCODER_ERR),
            'bt_active': bool(status & self.STATUS_BT_ACTIVE),
        }

    def publish_collision_state(self, collision: bool):
        """Publish /collision on every valid STM32 uplink frame.

        Reason:
        - Late subscribers such as `ros2 topic echo /collision` should still see data.
        - safety_guard / Web status can receive periodic heartbeat-style collision state.
        - Logs are still printed only when the state changes, to avoid spam.
        """
        msg = Bool()
        msg.data = collision
        self.collision_pub.publish(msg)

        if self.last_collision is None or collision != self.last_collision:
            self.last_collision = collision

            if collision:
                self.get_logger().warn('stm32 collision/stop flag = true', throttle_duration_sec=1.0)
            else:
                self.get_logger().info('stm32 collision/stop flag = false', throttle_duration_sec=2.0)

    def publish_stm32_status(self, now, status: int, gyro_z_deg_s: float):
        decoded = self.decode_status(status)

        # /collision 保持旧接口，但语义收窄为“必须停车/急停”的安全状态。
        # 超声波限速 ultra_slow 只进入 /stm32/status，不把 /collision 拉高。
        collision = decoded['estop'] or decoded['ultra_stop']
        self.publish_collision_state(collision)

        payload = dict(decoded)
        payload['stamp_sec'] = float(now.nanoseconds) / 1e9
        payload['gyro_z_deg_s'] = float(gyro_z_deg_s)

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        self.stm32_status_pub.publish(msg)

        if self.last_status is None or int(status) != self.last_status:
            self.last_status = int(status)
            self.get_logger().info(
                'stm32 status=0x%02X estop=%s ultra_slow=%s ultra_stop=%s pi_timeout=%s mpu_error=%s encoder_error=%s'
                % (
                    int(status) & 0xFF,
                    decoded['estop'],
                    decoded['ultra_slow'],
                    decoded['ultra_stop'],
                    decoded['pi_timeout'],
                    decoded['mpu_error'],
                    decoded['encoder_error'],
                ),
                throttle_duration_sec=1.0,
            )

    # ---------------- shutdown ----------------

    def send_stop(self):
        try:
            if hasattr(self, 'ser') and self.ser.is_open:
                stop = self.make_downlink_frame(0.0, 0.0)
                for _ in range(3):
                    self.ser.write(stop)
                    time.sleep(0.02)
        except Exception:
            pass

    def close(self):
        self.send_stop()
        try:
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = UartBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
