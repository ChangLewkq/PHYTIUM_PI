#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Flyt-Pi ROS2 node: receive cloud YOLO bboxes, compute local depth, publish follow cmd.

Input from RTX server, TCP JSON line:
{
  "rgb_width": 480,
  "rgb_height": 270,
  "detections": [
    {"class_id":0, "class_name":"person", "conf":0.82, "bbox":[x1,y1,x2,y2]}
  ]
}

ROS input:
  /camera/aligned_depth_to_color/image_raw, 16UC1 in millimeters

ROS output:
  /perception/detections      std_msgs/String JSON
  /perception/target          std_msgs/String JSON
  /cmd_vel_follow             geometry_msgs/Twist
"""

import json
import socket
import threading
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class TargetDepthFollower(Node):
    def __init__(self):
        super().__init__('target_depth_follower')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 9997)
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('detections_topic', '/perception/detections')
        self.declare_parameter('target_topic', '/perception/target')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_follow')
        self.declare_parameter('publish_cmd', True)

        self.declare_parameter('target_class', 'person')
        self.declare_parameter('min_conf', 0.45)
        self.declare_parameter('target_distance', 1.0)
        self.declare_parameter('distance_deadband', 0.12)
        self.declare_parameter('too_close_distance', 0.45)
        self.declare_parameter('depth_min_m', 0.15)
        self.declare_parameter('depth_max_m', 8.0)
        self.declare_parameter('roi_scale', 0.35)
        self.declare_parameter('lost_timeout_sec', 0.6)

        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('linear_kp', 0.22)
        self.declare_parameter('angular_kp', 0.35)
        self.declare_parameter('max_linear', 0.12)
        self.declare_parameter('max_reverse', 0.05)
        self.declare_parameter('max_angular', 0.35)
        self.declare_parameter('angular_deadband', 0.05)
        self.declare_parameter('stop_when_lost', True)

        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.detections_topic = str(self.get_parameter('detections_topic').value)
        self.target_topic = str(self.get_parameter('target_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.publish_cmd = bool(self.get_parameter('publish_cmd').value)

        self.target_class = str(self.get_parameter('target_class').value)
        self.min_conf = float(self.get_parameter('min_conf').value)
        self.target_distance = float(self.get_parameter('target_distance').value)
        self.distance_deadband = float(self.get_parameter('distance_deadband').value)
        self.too_close_distance = float(self.get_parameter('too_close_distance').value)
        self.depth_min_m = float(self.get_parameter('depth_min_m').value)
        self.depth_max_m = float(self.get_parameter('depth_max_m').value)
        self.roi_scale = float(self.get_parameter('roi_scale').value)
        self.lost_timeout_sec = float(self.get_parameter('lost_timeout_sec').value)

        self.linear_kp = float(self.get_parameter('linear_kp').value)
        self.angular_kp = float(self.get_parameter('angular_kp').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_reverse = float(self.get_parameter('max_reverse').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.angular_deadband = float(self.get_parameter('angular_deadband').value)
        self.stop_when_lost = bool(self.get_parameter('stop_when_lost').value)

        self.bridge = CvBridge()
        self.depth_frame: Optional[np.ndarray] = None
        self.depth_lock = threading.Lock()

        self.latest_packet: Optional[Dict] = None
        self.latest_packet_time = self.get_clock().now()
        self.det_lock = threading.Lock()

        self.detections_pub = self.create_publisher(String, self.detections_topic, 10)
        self.target_pub = self.create_publisher(String, self.target_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data)

        rate = float(self.get_parameter('control_rate').value)
        self.create_timer(1.0 / max(1.0, rate), self.control_loop)

        self.server_thread = threading.Thread(target=self.server_loop, daemon=True)
        self.server_thread.start()

        self.get_logger().info(
            f'target_depth_follower started: det={self.host}:{self.port}, depth={self.depth_topic}, '
            f'cmd={self.cmd_vel_topic}, target={self.target_class}, publish_cmd={self.publish_cmd}'
        )

    @staticmethod
    def clamp(x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, x))

    def server_loop(self):
        while rclpy.ok():
            server = None
            try:
                server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((self.host, self.port))
                server.listen(1)
                self.get_logger().info(f'waiting detection connection on {self.host}:{self.port}')
                while rclpy.ok():
                    conn, addr = server.accept()
                    self.get_logger().info(f'✅ detection connected: {addr}')
                    self.handle_detection_conn(conn)
            except Exception as exc:
                self.get_logger().error(f'detection server error: {exc}')
            finally:
                if server is not None:
                    try:
                        server.close()
                    except Exception:
                        pass

    def handle_detection_conn(self, conn: socket.socket):
        buf = b''
        with conn:
            while rclpy.ok():
                data = conn.recv(4096)
                if not data:
                    break
                buf += data
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    if not line.strip():
                        continue
                    try:
                        packet = json.loads(line.decode('utf-8'))
                    except Exception as exc:
                        self.get_logger().warn(f'bad detection json: {exc}')
                        continue
                    with self.det_lock:
                        self.latest_packet = packet
                        self.latest_packet_time = self.get_clock().now()
                    msg = String()
                    msg.data = json.dumps(packet, ensure_ascii=False)
                    self.detections_pub.publish(msg)
        self.get_logger().warn('detection connection closed')

    def depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            with self.depth_lock:
                self.depth_frame = depth
        except Exception as exc:
            self.get_logger().warn(f'depth convert failed: {exc}', throttle_duration_sec=2.0)

    def choose_detection(self, detections: List[Dict]) -> Optional[Dict]:
        candidates = []
        for det in detections:
            name = str(det.get('class_name', ''))
            conf = float(det.get('conf', 0.0))
            bbox = det.get('bbox', None)
            if name != self.target_class or conf < self.min_conf or not bbox or len(bbox) != 4:
                continue
            x1, y1, x2, y2 = map(float, bbox)
            area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
            candidates.append((conf, area, det))
        if not candidates:
            return None
        candidates.sort(key=lambda item: (item[0], item[1]), reverse=True)
        return candidates[0][2]

    def compute_depth_for_bbox(self, bbox: List[float], rgb_w: int, rgb_h: int, depth: np.ndarray) -> Tuple[Optional[float], Dict]:
        dh, dw = depth.shape[:2]
        x1, y1, x2, y2 = map(float, bbox)
        bw = max(1.0, x2 - x1)
        bh = max(1.0, y2 - y1)
        cx = (x1 + x2) * 0.5
        cy = (y1 + y2) * 0.5
        rw = bw * self.roi_scale
        rh = bh * self.roi_scale

        rx1 = cx - rw * 0.5
        rx2 = cx + rw * 0.5
        ry1 = cy - rh * 0.5
        ry2 = cy + rh * 0.5

        # Assumption: depth is aligned to color. Resize ratio maps low-res RGB bbox to high-res aligned depth.
        sx = dw / max(1.0, float(rgb_w))
        sy = dh / max(1.0, float(rgb_h))

        dx1 = int(self.clamp(rx1 * sx, 0, dw - 1))
        dx2 = int(self.clamp(rx2 * sx, 0, dw))
        dy1 = int(self.clamp(ry1 * sy, 0, dh - 1))
        dy2 = int(self.clamp(ry2 * sy, 0, dh))

        debug = {'depth_width': dw, 'depth_height': dh, 'depth_roi': [dx1, dy1, dx2, dy2]}
        if dx2 <= dx1 or dy2 <= dy1:
            debug['reason'] = 'empty_roi'
            return None, debug

        roi = depth[dy1:dy2, dx1:dx2].astype(np.float32) * 0.001
        valid = roi[(roi > self.depth_min_m) & (roi < self.depth_max_m)]
        debug['valid_depth_count'] = int(valid.size)

        if valid.size < 8:
            debug['reason'] = 'too_few_depth_points'
            return None, debug

        return float(np.median(valid)), debug

    def publish_zero(self):
        if self.publish_cmd:
            self.cmd_pub.publish(Twist())

    def publish_target_and_stop(self, target_msg: Dict):
        out = String()
        out.data = json.dumps(target_msg, ensure_ascii=False)
        self.target_pub.publish(out)
        if self.stop_when_lost:
            self.publish_zero()

    def control_loop(self):
        now = self.get_clock().now()
        with self.det_lock:
            packet = self.latest_packet
            packet_time = self.latest_packet_time
        with self.depth_lock:
            depth = self.depth_frame.copy() if self.depth_frame is not None else None

        target_msg = {'has_target': False, 'reason': '', 'stamp_sec': now.nanoseconds / 1e9}

        if packet is None:
            target_msg['reason'] = 'no_detection_packet'
            self.publish_target_and_stop(target_msg)
            return

        age = (now - packet_time).nanoseconds / 1e9
        if age > self.lost_timeout_sec:
            target_msg['reason'] = f'detection_timeout_{age:.2f}s'
            self.publish_target_and_stop(target_msg)
            return

        if depth is None:
            target_msg['reason'] = 'no_depth'
            self.publish_target_and_stop(target_msg)
            return

        rgb_w = int(packet.get('rgb_width', packet.get('width', 480)))
        rgb_h = int(packet.get('rgb_height', packet.get('height', 270)))
        detections = packet.get('detections', [])

        det = self.choose_detection(detections)
        if det is None:
            target_msg['reason'] = 'no_target_class'
            self.publish_target_and_stop(target_msg)
            return

        bbox = det['bbox']
        x1, y1, x2, y2 = map(float, bbox)
        cx = (x1 + x2) * 0.5
        cy = (y1 + y2) * 0.5

        distance, depth_debug = self.compute_depth_for_bbox(bbox, rgb_w, rgb_h, depth)
        if distance is None:
            target_msg['reason'] = 'invalid_depth'
            target_msg.update(depth_debug)
            self.publish_target_and_stop(target_msg)
            return

        offset = (cx - rgb_w * 0.5) / max(1.0, rgb_w * 0.5)

        linear = 0.0
        err_dist = distance - self.target_distance
        if distance < self.too_close_distance:
            linear = -min(self.max_reverse, abs(err_dist) * self.linear_kp)
        elif abs(err_dist) > self.distance_deadband:
            linear = self.clamp(err_dist * self.linear_kp, -self.max_reverse, self.max_linear)

        if abs(offset) < self.angular_deadband:
            angular = 0.0
        else:
            # target right => offset positive => turn right => angular negative
            angular = self.clamp(-offset * self.angular_kp, -self.max_angular, self.max_angular)

        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        if self.publish_cmd:
            self.cmd_pub.publish(cmd)

        target_msg.update({
            'has_target': True,
            'class_name': det.get('class_name', ''),
            'class_id': det.get('class_id', -1),
            'conf': float(det.get('conf', 0.0)),
            'bbox': [int(x1), int(y1), int(x2), int(y2)],
            'rgb_width': rgb_w,
            'rgb_height': rgb_h,
            'cx': float(cx),
            'cy': float(cy),
            'offset': float(offset),
            'distance_m': float(distance),
            'cmd_linear': float(linear),
            'cmd_angular': float(angular),
        })
        target_msg.update(depth_debug)

        out = String()
        out.data = json.dumps(target_msg, ensure_ascii=False)
        self.target_pub.publish(out)

        self.get_logger().info(
            f'target {self.target_class}: dist={distance:.2f}m offset={offset:.2f} cmd=({linear:.2f},{angular:.2f})',
            throttle_duration_sec=0.5
        )


def main():
    rclpy.init()
    node = TargetDepthFollower()
    try:
        rclpy.spin(node)
    finally:
        node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
