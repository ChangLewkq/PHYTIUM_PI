#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanAmclDelay(Node):
    def __init__(self):
        super().__init__('scan_amcl_delay')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_amcl')
        self.declare_parameter('delay_sec', 0.08)
        self.declare_parameter('frame_id', 'laser_link')

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.delay_sec = float(self.get_parameter('delay_sec').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.delay_ns = int(self.delay_sec * 1e9)
        self.count = 0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(LaserScan, self.output_topic, qos)
        self.sub = self.create_subscription(LaserScan, self.input_topic, self.cb, qos)

        self.get_logger().info(
            f'scan_amcl_delay started: {self.input_topic} -> {self.output_topic}, '
            f'stamp = now - {self.delay_sec:.3f}s, frame={self.frame_id}'
        )

    def cb(self, msg: LaserScan):
        out = copy.deepcopy(msg)

        stamp = self.get_clock().now() - Duration(nanoseconds=self.delay_ns)
        out.header.stamp = stamp.to_msg()
        out.header.frame_id = self.frame_id

        self.pub.publish(out)

        self.count += 1
        if self.count <= 5 or self.count % 100 == 0:
            self.get_logger().info(
                f'publish {self.output_topic} #{self.count}, delay={self.delay_sec:.3f}s'
            )


def main():
    rclpy.init()
    node = ScanAmclDelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
