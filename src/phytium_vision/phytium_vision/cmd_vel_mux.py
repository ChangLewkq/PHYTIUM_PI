#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cmd_vel_mux.py

Robust velocity multiplexer for Phytium robot.

Main feature of this version:
- Supports normal ROS parameters, but no longer depends on parameter services for mode switching.
- Web/backend can switch mode by publishing std_msgs/String to /cmd_vel_mux/mode_cmd.

Inputs:
- /cmd_vel_web
- /cmd_vel_keyboard
- /cmd_vel_follow
- /cmd_vel_nav
- /cmd_vel_mux/estop
- /cmd_vel_mux/mode_cmd

Output:
- /cmd_vel_raw

Active source:
- /cmd_vel_mux/active_source

Modes:
- stop
- web
- keyboard
- follow
- nav
- auto
"""

from typing import Dict, Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class CmdVelMux(Node):
    VALID_MODES = {"stop", "web", "keyboard", "follow", "nav", "auto"}

    def __init__(self):
        super().__init__("cmd_vel_mux")

        self.declare_parameter("mode", "stop")

        self.declare_parameter("web_topic", "/cmd_vel_web")
        self.declare_parameter("keyboard_topic", "/cmd_vel_keyboard")
        self.declare_parameter("follow_topic", "/cmd_vel_follow")
        self.declare_parameter("nav_topic", "/cmd_vel_nav")

        self.declare_parameter("estop_topic", "/cmd_vel_mux/estop")
        self.declare_parameter("mode_cmd_topic", "/cmd_vel_mux/mode_cmd")

        self.declare_parameter("output_topic", "/cmd_vel_raw")
        self.declare_parameter("active_source_topic", "/cmd_vel_mux/active_source")

        self.declare_parameter("web_timeout_sec", 0.40)
        self.declare_parameter("keyboard_timeout_sec", 0.40)
        self.declare_parameter("follow_timeout_sec", 0.60)
        self.declare_parameter("nav_timeout_sec", 0.60)

        self.declare_parameter("rate", 20.0)

        self.declare_parameter("max_linear", 0.20)
        self.declare_parameter("max_reverse", 0.08)
        self.declare_parameter("max_angular", 0.70)

        self.declare_parameter("auto_priority", "keyboard,web,follow,nav")

        self.mode = self._normalize_mode(str(self.get_parameter("mode").value))

        self.timeouts = {
            "web": float(self.get_parameter("web_timeout_sec").value),
            "keyboard": float(self.get_parameter("keyboard_timeout_sec").value),
            "follow": float(self.get_parameter("follow_timeout_sec").value),
            "nav": float(self.get_parameter("nav_timeout_sec").value),
        }

        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_reverse = float(self.get_parameter("max_reverse").value)
        self.max_angular = float(self.get_parameter("max_angular").value)

        self.auto_priority = [
            x.strip()
            for x in str(self.get_parameter("auto_priority").value).split(",")
            if x.strip() in ("keyboard", "web", "follow", "nav")
        ]
        if not self.auto_priority:
            self.auto_priority = ["keyboard", "web", "follow", "nav"]

        self.last_cmd: Dict[str, Optional[Twist]] = {
            "web": None,
            "keyboard": None,
            "follow": None,
            "nav": None,
        }
        self.last_time = {
            "web": None,
            "keyboard": None,
            "follow": None,
            "nav": None,
        }

        self.estop = False
        self.last_active_source = ""

        self.output_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self.active_pub = self.create_publisher(
            String,
            str(self.get_parameter("active_source_topic").value),
            10,
        )

        self.create_subscription(
            Twist,
            str(self.get_parameter("web_topic").value),
            lambda msg: self._on_cmd("web", msg),
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("keyboard_topic").value),
            lambda msg: self._on_cmd("keyboard", msg),
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("follow_topic").value),
            lambda msg: self._on_cmd("follow", msg),
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("nav_topic").value),
            lambda msg: self._on_cmd("nav", msg),
            10,
        )

        self.create_subscription(
            Bool,
            str(self.get_parameter("estop_topic").value),
            self._on_estop,
            10,
        )

        self.create_subscription(
            String,
            str(self.get_parameter("mode_cmd_topic").value),
            self._on_mode_cmd,
            10,
        )

        rate = float(self.get_parameter("rate").value)
        self.timer = self.create_timer(1.0 / max(1.0, rate), self._on_timer)

        self.get_logger().info(
            "cmd_vel_mux started: "
            f"mode={self.mode}, output={self.get_parameter('output_topic').value}, "
            f"mode_cmd={self.get_parameter('mode_cmd_topic').value}, "
            f"active={self.get_parameter('active_source_topic').value}"
        )

    def _normalize_mode(self, mode: str) -> str:
        mode = (mode or "stop").strip().lower()
        if mode == "manual":
            mode = "web"
        if mode == "mapping":
            mode = "web"
        if mode == "navigation":
            mode = "nav"
        if mode not in self.VALID_MODES:
            mode = "stop"
        return mode

    def _on_mode_cmd(self, msg: String):
        new_mode = self._normalize_mode(msg.data)
        old_mode = self.mode

        self.mode = new_mode

        if old_mode != new_mode:
            self.get_logger().info(f"mode changed by topic: {old_mode} -> {new_mode}")

        # Immediately publish active state for Web feedback.
        self._publish_active_source(f"mode:{new_mode}")

    def _on_estop(self, msg: Bool):
        self.estop = bool(msg.data)
        if self.estop:
            self.mode = "stop"
            self.get_logger().warn("estop active, mode forced to stop")
            self.output_pub.publish(Twist())
            self._publish_active_source("estop")

    def _on_cmd(self, source: str, msg: Twist):
        self.last_cmd[source] = msg
        self.last_time[source] = self.get_clock().now()

    def _age_sec(self, source: str):
        t = self.last_time.get(source)
        if t is None:
            return None
        return (self.get_clock().now() - t).nanoseconds / 1e9

    def _is_fresh(self, source: str) -> bool:
        age = self._age_sec(source)
        if age is None:
            return False
        return age <= self.timeouts.get(source, 0.5)

    def _zero(self) -> Twist:
        return Twist()

    def _limit_cmd(self, cmd: Twist) -> Twist:
        out = Twist()

        out.linear.x = clamp(float(cmd.linear.x), -self.max_reverse, self.max_linear)
        out.linear.y = 0.0
        out.linear.z = 0.0

        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = clamp(float(cmd.angular.z), -self.max_angular, self.max_angular)

        return out

    def _select(self):
        if self.estop:
            return "estop", self._zero()

        if self.mode == "stop":
            return "stop", self._zero()

        if self.mode == "auto":
            for src in self.auto_priority:
                if self._is_fresh(src) and self.last_cmd[src] is not None:
                    return src, self._limit_cmd(self.last_cmd[src])
            return "timeout", self._zero()

        src = self.mode

        if src not in self.last_cmd:
            return "stop", self._zero()

        if self._is_fresh(src) and self.last_cmd[src] is not None:
            return src, self._limit_cmd(self.last_cmd[src])

        return "timeout", self._zero()

    def _publish_active_source(self, source: str):
        msg = String()
        msg.data = source
        self.active_pub.publish(msg)
        self.last_active_source = source

    def _on_timer(self):
        source, out = self._select()

        self.output_pub.publish(out)
        self._publish_active_source(source)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
