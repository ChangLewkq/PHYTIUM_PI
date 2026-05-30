#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
safety_guard.py

ROS2 system-level safety guard for Phytium robot.

This version uses the same *physical front clearance* rule as the STM32 ultrasonic layer.

Important:
- D6 LiDAR is mounted behind the front ultrasonic module.
- lidar_to_front_offset means the distance from LiDAR origin to the robot's front edge / ultrasonic plane.
- front_clearance = lidar_front_min - lidar_to_front_offset

Physical front clearance rule:
- front_clearance > 0.25 m:
    normal
- 0.10 m < front_clearance <= 0.25 m:
    limit forward speed to 0.08 m/s
- front_clearance <= 0.10 m:
    block forward motion
    allow reverse and in-place turning for escape

Input:
- /cmd_vel_raw                geometry_msgs/Twist
- /scan                       sensor_msgs/LaserScan
- /cmd_vel_mux/active_source  std_msgs/String
- /perception/target          std_msgs/String, JSON from target_depth_follower
- /safety/estop               std_msgs/Bool

Output:
- /cmd_vel                    geometry_msgs/Twist
- /safety/status              std_msgs/String, JSON status
"""

import json
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class SafetyGuard(Node):
    def __init__(self):
        super().__init__("safety_guard")

        # Topics
        self.declare_parameter("input_cmd_topic", "/cmd_vel_raw")
        self.declare_parameter("output_cmd_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("active_source_topic", "/cmd_vel_mux/active_source")
        self.declare_parameter("target_topic", "/perception/target")
        self.declare_parameter("estop_topic", "/safety/estop")
        self.declare_parameter("status_topic", "/safety/status")

        # Timing
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("cmd_timeout_sec", 0.50)
        self.declare_parameter("scan_timeout_sec", 0.80)
        self.declare_parameter("follow_target_timeout_sec", 0.60)

        # Global speed limits
        self.declare_parameter("max_forward", 0.10)
        self.declare_parameter("max_reverse", 0.05)
        self.declare_parameter("max_angular", 0.35)

        # LiDAR safety, aligned with STM32 ultrasonic rule.
        self.declare_parameter("enable_scan_guard", True)
        self.declare_parameter("front_angle_deg", 25.0)

        # LiDAR is 15cm behind the ultrasonic/front edge by default.
        # Compare obstacle distance using physical clearance:
        # front_clearance = front_min - lidar_to_front_offset
        self.declare_parameter("lidar_to_front_offset", 0.15)

        # Ignore impossible near returns in front sector.
        # Since the LiDAR is mounted behind the robot front edge,
        # any front-sector range smaller than this is usually robot body / wire / bracket self-hit.
        self.declare_parameter("front_min_valid_range", 0.16)

        # These thresholds are physical distances from the robot's front edge,
        # aligned with STM32 ultrasonic rules.
        self.declare_parameter("slow_distance", 0.25)
        self.declare_parameter("stop_distance", 0.10)
        self.declare_parameter("slow_max_forward", 0.08)

        # Vision follow safety
        self.declare_parameter("enable_follow_guard", True)
        self.declare_parameter("follow_source_name", "follow")
        self.declare_parameter("stop_follow_when_target_lost", True)

        # Debug / status
        self.declare_parameter("debug", False)
        self.declare_parameter("status_publish_period_sec", 0.20)

        self.input_cmd_topic = str(self.get_parameter("input_cmd_topic").value)
        self.output_cmd_topic = str(self.get_parameter("output_cmd_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.active_source_topic = str(self.get_parameter("active_source_topic").value)
        self.target_topic = str(self.get_parameter("target_topic").value)
        self.estop_topic = str(self.get_parameter("estop_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.cmd_timeout_sec = float(self.get_parameter("cmd_timeout_sec").value)
        self.scan_timeout_sec = float(self.get_parameter("scan_timeout_sec").value)
        self.follow_target_timeout_sec = float(self.get_parameter("follow_target_timeout_sec").value)

        self.max_forward = float(self.get_parameter("max_forward").value)
        self.max_reverse = float(self.get_parameter("max_reverse").value)
        self.max_angular = float(self.get_parameter("max_angular").value)

        self.enable_scan_guard = bool(self.get_parameter("enable_scan_guard").value)
        self.front_angle_deg = float(self.get_parameter("front_angle_deg").value)
        self.lidar_to_front_offset = float(self.get_parameter("lidar_to_front_offset").value)
        self.front_min_valid_range = float(self.get_parameter("front_min_valid_range").value)
        self.slow_distance = float(self.get_parameter("slow_distance").value)
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.slow_max_forward = float(self.get_parameter("slow_max_forward").value)

        self.enable_follow_guard = bool(self.get_parameter("enable_follow_guard").value)
        self.follow_source_name = str(self.get_parameter("follow_source_name").value)
        self.stop_follow_when_target_lost = bool(self.get_parameter("stop_follow_when_target_lost").value)

        self.debug = bool(self.get_parameter("debug").value)
        self.status_period = float(self.get_parameter("status_publish_period_sec").value)

        self.last_cmd: Optional[Twist] = None
        self.last_cmd_time = None

        self.last_scan_time = None
        self.front_min = None
        self.front_clearance = None
        self.left_min = None
        self.right_min = None

        self.estop = False
        self.active_source = "stop"

        self.target_has = False
        self.target_distance = None
        self.target_reason = ""
        self.last_target_time = None

        self.last_status_time = self.get_clock().now()

        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.create_subscription(Twist, self.input_cmd_topic, self.on_cmd, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.create_subscription(String, self.active_source_topic, self.on_active_source, 10)
        self.create_subscription(String, self.target_topic, self.on_target, 10)
        self.create_subscription(Bool, self.estop_topic, self.on_estop, 10)

        self.timer = self.create_timer(1.0 / max(1.0, self.rate_hz), self.on_timer)

        self.get_logger().info(
            "safety_guard started with STM32-style physical front clearance rule: "
            f"{self.input_cmd_topic} -> {self.output_cmd_topic}, "
            f"front=±{self.front_angle_deg}deg, "
            f"lidar_to_front_offset={self.lidar_to_front_offset}m, "
            f"slow<= {self.slow_distance}m to {self.slow_max_forward}m/s, "
            f"block forward<= {self.stop_distance}m"
        )

    def age_sec(self, t) -> Optional[float]:
        if t is None:
            return None
        return (self.get_clock().now() - t).nanoseconds / 1e9

    def on_cmd(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def on_estop(self, msg: Bool):
        self.estop = bool(msg.data)

    def on_active_source(self, msg: String):
        self.active_source = str(msg.data).strip()

    def on_target(self, msg: String):
        self.last_target_time = self.get_clock().now()

        try:
            data = json.loads(msg.data)
            self.target_has = bool(data.get("has_target", False))
            self.target_distance = data.get("distance_m", None)
            self.target_reason = str(data.get("reason", ""))
        except Exception as e:
            self.target_has = False
            self.target_distance = None
            self.target_reason = f"bad_target_json:{e}"

    @staticmethod
    def normalize_angle_rad(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    def on_scan(self, msg: LaserScan):
        front_min = None
        left_min = None
        right_min = None

        front_rad = math.radians(self.front_angle_deg)

        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                a = self.normalize_angle_rad(angle)
                abs_a = abs(a)

                if abs_a <= front_rad:
                    # Filter impossible self-hit points caused by robot body, wires or brackets.
                    # A real obstacle in front of the robot should be at least around
                    # lidar_to_front_offset away from the LiDAR origin.
                    if r >= self.front_min_valid_range:
                        front_min = r if front_min is None else min(front_min, r)

                if math.radians(30.0) <= a <= math.radians(120.0):
                    left_min = r if left_min is None else min(left_min, r)

                if math.radians(-120.0) <= a <= math.radians(-30.0):
                    right_min = r if right_min is None else min(right_min, r)

            angle += msg.angle_increment

        self.front_min = front_min
        self.left_min = left_min
        self.right_min = right_min
        self.last_scan_time = self.get_clock().now()

    def zero_cmd(self) -> Twist:
        return Twist()

    def copy_and_limit_speed(self, cmd: Twist, reasons: list, limited_flags: dict) -> Twist:
        out = Twist()

        original_linear = float(cmd.linear.x)
        original_angular = float(cmd.angular.z)

        out.linear.x = clamp(original_linear, -self.max_reverse, self.max_forward)
        out.linear.y = 0.0
        out.linear.z = 0.0

        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = clamp(original_angular, -self.max_angular, self.max_angular)

        if abs(out.linear.x - original_linear) > 1e-6 or abs(out.angular.z - original_angular) > 1e-6:
            reasons.append("speed_limited")
            limited_flags["speed_limited"] = True

        return out

    def is_follow_target_stale_or_lost(self) -> bool:
        if not self.stop_follow_when_target_lost:
            return False

        if self.active_source != self.follow_source_name:
            return False

        age = self.age_sec(self.last_target_time)
        if age is None or age > self.follow_target_timeout_sec:
            return True

        return not self.target_has

    def apply_scan_guard(self, cmd: Twist, reasons: list, limited_flags: dict) -> Twist:
        if not self.enable_scan_guard:
            return cmd

        scan_age = self.age_sec(self.last_scan_time)
        if scan_age is None or scan_age > self.scan_timeout_sec:
            limited_flags["scan_stale"] = True
            return cmd

        if self.front_min is None:
            self.front_clearance = None
            return cmd

        # Convert LiDAR-origin distance to actual robot-front clearance.
        # Example: LiDAR is 0.15m behind the ultrasonic/front edge:
        # obstacle 0.40m from LiDAR = 0.25m from robot front.
        self.front_clearance = max(0.0, self.front_min - self.lidar_to_front_offset)

        # STM32-style physical clearance rule:
        # Only intercept forward motion. Reverse / turning are allowed for escape.
        if cmd.linear.x > 0.0:
            if self.front_clearance <= self.stop_distance:
                cmd.linear.x = 0.0
                reasons.append("front_block_forward")
                limited_flags["front_block"] = True

            elif self.front_clearance <= self.slow_distance:
                if cmd.linear.x > self.slow_max_forward:
                    cmd.linear.x = self.slow_max_forward
                    reasons.append("front_slow_forward")
                    limited_flags["front_slow"] = True

        return cmd

    def publish_status(self, safe: bool, reasons: list, output_cmd: Twist, limited_flags: dict):
        now = self.get_clock().now()
        if (now - self.last_status_time).nanoseconds / 1e9 < self.status_period:
            return

        self.last_status_time = now

        status = {
            "safe": bool(safe),
            "reasons": reasons,
            "estop": self.estop,
            "active_source": self.active_source,

            "front_min": self.front_min,
            "front_clearance": self.front_clearance,
            "lidar_to_front_offset": self.lidar_to_front_offset,
            "front_min_valid_range": self.front_min_valid_range,
            "left_min": self.left_min,
            "right_min": self.right_min,

            "front_rule": {
                "distance_basis": "robot_front_clearance",
                "normal_above_m": self.slow_distance,
                "slow_between_m": [self.stop_distance, self.slow_distance],
                "block_forward_below_m": self.stop_distance,
                "equivalent_lidar_slow_threshold_m": self.slow_distance + self.lidar_to_front_offset,
                "equivalent_lidar_block_threshold_m": self.stop_distance + self.lidar_to_front_offset,
                "slow_max_forward_mps": self.slow_max_forward,
                "escape_reverse_turn_allowed": True,
            },

            "cmd_age_sec": self.age_sec(self.last_cmd_time),
            "scan_age_sec": self.age_sec(self.last_scan_time),
            "target_age_sec": self.age_sec(self.last_target_time),

            "target_has": self.target_has,
            "target_distance": self.target_distance,
            "target_reason": self.target_reason,

            "limited": limited_flags,
            "out_linear": output_cmd.linear.x,
            "out_angular": output_cmd.angular.z,
        }

        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(msg)

    def on_timer(self):
        reasons = []
        limited_flags = {
            "speed_limited": False,
            "front_slow": False,
            "front_block": False,
            "scan_stale": False,
            "cmd_timeout": False,
            "follow_target_lost": False,
            "estop": False,
        }

        safe = True

        if self.estop:
            out = self.zero_cmd()
            reasons.append("estop")
            limited_flags["estop"] = True
            safe = False
            self.cmd_pub.publish(out)
            self.publish_status(safe, reasons, out, limited_flags)
            return

        cmd_age = self.age_sec(self.last_cmd_time)
        if self.last_cmd is None or cmd_age is None or cmd_age > self.cmd_timeout_sec:
            out = self.zero_cmd()
            reasons.append("cmd_timeout")
            limited_flags["cmd_timeout"] = True
            safe = False
            self.cmd_pub.publish(out)
            self.publish_status(safe, reasons, out, limited_flags)
            return

        if self.enable_follow_guard and self.is_follow_target_stale_or_lost():
            out = self.zero_cmd()
            reasons.append("follow_target_lost")
            limited_flags["follow_target_lost"] = True
            safe = False
            self.cmd_pub.publish(out)
            self.publish_status(safe, reasons, out, limited_flags)
            return

        out = self.copy_and_limit_speed(self.last_cmd, reasons, limited_flags)
        out = self.apply_scan_guard(out, reasons, limited_flags)

        # This is not "system unsafe"; it means forward command is being blocked.
        # Keep safe=false only when the robot is actively blocked from forward motion.
        if limited_flags["front_block"]:
            safe = False

        self.cmd_pub.publish(out)
        self.publish_status(safe, reasons, out, limited_flags)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
