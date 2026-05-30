#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
d6_final.py - optimized D6 lidar driver for Phytium ROS2 robot.

Main optimizations compared with the old version:
1. No sort on every scan publish.
2. No 360 x cached_points nested search.
3. Directly bins each incoming point into 360 LaserScan slots while parsing.
4. Publishes at fixed publish_hz.
5. Lower timer frequency and bounded packets per callback.
6. Much lower logging overhead.

Expected CPU drop:
- Old version: 60% ~ 80% on Flyt-Pi
- Optimized target: 10% ~ 30%, depending on system load

Output:
- /scan sensor_msgs/msg/LaserScan
"""

import math
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class D6Final(Node):
    SYNC = b"\xAA\x55"

    def __init__(self):
        super().__init__("d6_final")

        self.declare_parameter("port", "/dev/phytium_d6_lidar")
        self.declare_parameter("baudrate", 230400)
        self.declare_parameter("frame_id", "laser_link")

        # Processing / publishing
        self.declare_parameter("read_period_sec", 0.005)       # old: 0.002
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("max_packets_per_process", 80)
        self.declare_parameter("min_points_per_scan", 80)

        # Scan config
        self.declare_parameter("angle_bins", 360)
        self.declare_parameter("range_min", 0.02)
        self.declare_parameter("range_max", 15.0)
        self.declare_parameter("angle_offset_deg", 0.0)
        self.declare_parameter("invert_angle", False)

        # Debug
        self.declare_parameter("debug", False)
        self.declare_parameter("log_period_sec", 5.0)

        port = str(self.get_parameter("port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.read_period_sec = float(self.get_parameter("read_period_sec").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.publish_period_sec = 1.0 / max(1.0, self.publish_hz)
        self.max_packets_per_process = int(self.get_parameter("max_packets_per_process").value)
        self.min_points_per_scan = int(self.get_parameter("min_points_per_scan").value)

        self.angle_bins = int(self.get_parameter("angle_bins").value)
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.angle_offset_deg = float(self.get_parameter("angle_offset_deg").value)
        self.invert_angle = bool(self.get_parameter("invert_angle").value)

        self.debug = bool(self.get_parameter("debug").value)
        self.log_period_sec = float(self.get_parameter("log_period_sec").value)

        self.angle_increment = 2.0 * math.pi / float(self.angle_bins)
        self.bin_scale = float(self.angle_bins) / 360.0

        self.empty_ranges = [self.range_max] * self.angle_bins
        self.empty_intensities = [0.0] * self.angle_bins
        self.ranges = self.empty_ranges.copy()
        self.intensities = self.empty_intensities.copy()
        self.points_in_scan = 0

        self.buf = bytearray()
        self.msg_count = 0
        self.packet_count = 0
        self.drop_count = 0

        self.pub = self.create_publisher(LaserScan, "/scan", 10)

        # Non-blocking serial. Timer controls read frequency.
        self.ser = serial.Serial(port, baudrate, timeout=0)

        self.last_publish_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now()

        self.timer = self.create_timer(self.read_period_sec, self.process)

        self.get_logger().info(
            "D6 optimized driver started: "
            f"port={port}, baud={baudrate}, publish_hz={self.publish_hz}, "
            f"bins={self.angle_bins}, read_period={self.read_period_sec}s"
        )

    def reset_scan_buffers(self):
        self.ranges = self.empty_ranges.copy()
        self.intensities = self.empty_intensities.copy()
        self.points_in_scan = 0

    def normalize_angle_deg(self, angle_deg: float) -> float:
        angle_deg = angle_deg + self.angle_offset_deg
        if self.invert_angle:
            angle_deg = -angle_deg
        angle_deg = angle_deg % 360.0
        return angle_deg

    def bin_point(self, angle_deg: float, rng: float, intensity: float):
        if not (self.range_min <= rng <= self.range_max):
            return

        angle_deg = self.normalize_angle_deg(angle_deg)

        # Nearest-degree bin.
        bin_idx = int(angle_deg * self.bin_scale + 0.5) % self.angle_bins

        # If multiple returns fall into one bin in one publish window,
        # keep the nearest obstacle. This is safer for obstacle display/SLAM.
        if self.ranges[bin_idx] >= self.range_max or rng < self.ranges[bin_idx]:
            self.ranges[bin_idx] = rng
            self.intensities[bin_idx] = intensity

        self.points_in_scan += 1

    def parse_packet(self, frame: bytes):
        n = frame[3]
        if n <= 0:
            return

        # D6 angle unit follows previous driver logic.
        first_angle = ((frame[4] | (frame[5] << 8)) >> 1) / 64.0
        last_angle = ((frame[6] | (frame[7] << 8)) >> 1) / 64.0

        span = 360.0 + last_angle - first_angle if last_angle < first_angle else last_angle - first_angle
        step = span / (n - 1) if n > 1 else 0.0

        idx = 10
        for i in range(n):
            intensity = float(frame[idx] & 0x3F)
            raw = (frame[idx + 2] << 8) | frame[idx + 1]
            rng = (raw >> 2) * 0.001

            # Skip invalid zero distance.
            if rng > 0.0:
                angle = first_angle + step * i
                self.bin_point(angle, rng, intensity)

            idx += 3

        self.packet_count += 1

    def process(self):
        try:
            waiting = self.ser.in_waiting
            if waiting > 0:
                self.buf.extend(self.ser.read(waiting))

            packets = 0

            while len(self.buf) >= 10 and packets < self.max_packets_per_process:
                start = self.buf.find(self.SYNC)

                if start < 0:
                    # Keep at most the last byte in case it is 0xAA.
                    if len(self.buf) > 1:
                        del self.buf[:-1]
                    break

                if start > 0:
                    del self.buf[:start]

                if len(self.buf) < 10:
                    break

                n = self.buf[3]

                # Basic sanity check, avoids being trapped by bad bytes.
                # D6 packet point count is normally small; keep a generous upper limit.
                if n <= 0 or n > 100:
                    del self.buf[0]
                    self.drop_count += 1
                    continue

                total = 10 + n * 3
                if len(self.buf) < total:
                    break

                frame = bytes(self.buf[:total])
                del self.buf[:total]

                self.parse_packet(frame)
                packets += 1

            self.maybe_publish()

        except Exception as e:
            self.get_logger().error(f"D6 driver error: {e}", throttle_duration_sec=2.0)

    def maybe_publish(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_publish_time).nanoseconds / 1e9

        if elapsed < self.publish_period_sec:
            return

        if self.points_in_scan < self.min_points_per_scan:
            # Do not publish very sparse scans unless debugging.
            if self.debug:
                self.get_logger().warn(
                    f"skip sparse scan: points={self.points_in_scan}",
                    throttle_duration_sec=2.0
                )
            return

        self.publish_scan(now, elapsed)
        self.last_publish_time = now
        self.reset_scan_buffers()

    def publish_scan(self, now, elapsed_sec: float):
        msg = LaserScan()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id

        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi - self.angle_increment
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = max(elapsed_sec, self.publish_period_sec)

        msg.range_min = self.range_min
        msg.range_max = self.range_max

        # Only one list copy each. No append loop, no sort, no nested matching.
        msg.ranges = self.ranges.copy()
        msg.intensities = self.intensities.copy()

        self.pub.publish(msg)
        self.msg_count += 1

        if self.debug:
            now_log = self.get_clock().now()
            log_elapsed = (now_log - self.last_log_time).nanoseconds / 1e9
            if log_elapsed >= self.log_period_sec:
                self.get_logger().info(
                    f"scan#{self.msg_count}: points={self.points_in_scan}, "
                    f"packets={self.packet_count}, drops={self.drop_count}, "
                    f"buf={len(self.buf)}"
                )
                self.last_log_time = now_log

    def destroy_node(self):
        try:
            if hasattr(self, "ser") and self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = D6Final()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
