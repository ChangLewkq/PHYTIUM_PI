#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RTX3050/4060Ti side: receive RGB frames, run YOLO/TensorRT, return bboxes.

The server only returns bboxes. Depth is computed on Flyt-Pi with local aligned depth.
"""

import argparse
import json
import socket
import struct
import threading
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
from flask import Flask, Response, render_template_string
from ultralytics import YOLO


def recv_exact(conn: socket.socket, n: int) -> bytes:
    data = b''
    while len(data) < n:
        chunk = conn.recv(n - len(data))
        if not chunk:
            raise ConnectionError('socket closed')
        data += chunk
    return data


def recv_frame(conn: socket.socket) -> Tuple[Dict, bytes]:
    header_len = struct.unpack('>I', recv_exact(conn, 4))[0]
    header = json.loads(recv_exact(conn, header_len).decode('utf-8'))
    payload = recv_exact(conn, int(header['payload_len']))
    return header, payload


class YOLOBBoxServer:
    def __init__(self, args):
        self.args = args
        self.running = True
        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_header = None
        self.vis_frame = None
        self.bbox_sock: Optional[socket.socket] = None
        print(f'🔧 loading model: {args.model}')
        self.model = YOLO(args.model)
        print('✅ model loaded')

    def start(self):
        threading.Thread(target=self.bbox_connect_loop, daemon=True).start()
        threading.Thread(target=self.rgb_server_loop, daemon=True).start()
        threading.Thread(target=self.detect_loop, daemon=True).start()
        if self.args.web:
            self.web_loop()
        else:
            while self.running:
                time.sleep(1.0)

    def bbox_connect_loop(self):
        while self.running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(3.0)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.connect((self.args.flyt_pi_host, self.args.bbox_port))
                sock.settimeout(None)
                self.bbox_sock = sock
                print(f'✅ bbox connected to Flyt-Pi {self.args.flyt_pi_host}:{self.args.bbox_port}')
                while self.running and self.bbox_sock is not None:
                    time.sleep(1.0)
            except Exception as exc:
                self.bbox_sock = None
                print(f'⏳ waiting bbox receiver: {exc}')
                time.sleep(2.0)

    def send_result(self, packet: Dict):
        sock = self.bbox_sock
        if sock is None:
            return
        try:
            raw = json.dumps(packet, ensure_ascii=False, separators=(',', ':')).encode('utf-8') + b'\n'
            sock.sendall(raw)
        except Exception as exc:
            print(f'bbox send failed: {exc}')
            try:
                sock.close()
            except Exception:
                pass
            self.bbox_sock = None

    def rgb_server_loop(self):
        while self.running:
            server = None
            try:
                server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((self.args.host, self.args.rgb_port))
                server.listen(1)
                print(f'🖥️ waiting RGB stream on {self.args.host}:{self.args.rgb_port}')
                conn, addr = server.accept()
                print(f'✅ RGB connected: {addr}')
                self.handle_rgb_conn(conn)
            except Exception as exc:
                print(f'RGB server error: {exc}')
                time.sleep(1.0)
            finally:
                if server is not None:
                    try:
                        server.close()
                    except Exception:
                        pass

    def handle_rgb_conn(self, conn: socket.socket):
        with conn:
            while self.running:
                header, payload = recv_frame(conn)
                img = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    with self.lock:
                        self.latest_frame = img
                        self.latest_header = header

    def detect_loop(self):
        last_infer = 0.0
        while self.running:
            with self.lock:
                frame = self.latest_frame.copy() if self.latest_frame is not None else None
                header = dict(self.latest_header) if self.latest_header is not None else None
            if frame is None or header is None:
                time.sleep(0.01)
                continue
            now = time.time()
            if self.args.max_fps > 0 and (now - last_infer) < 1.0 / self.args.max_fps:
                time.sleep(0.005)
                continue
            last_infer = now

            results = self.model(frame, verbose=False, conf=self.args.conf, imgsz=self.args.imgsz)
            detections: List[Dict] = []
            vis = frame.copy()
            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    name = self.model.names.get(cls, str(cls)) if isinstance(self.model.names, dict) else self.model.names[cls]
                    if self.args.only_target and self.args.target_class and name != self.args.target_class:
                        continue
                    det = {'class_id': cls, 'class_name': name, 'conf': conf, 'bbox': [int(x1), int(y1), int(x2), int(y2)]}
                    detections.append(det)
                    color = (0, 255, 0) if name == self.args.target_class else (255, 0, 0)
                    cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                    cv2.putText(vis, f'{name} {conf:.2f}', (int(x1), max(20, int(y1) - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            packet = {
                'seq': header.get('seq', 0),
                'stamp_sec': header.get('stamp_sec', 0),
                'stamp_nanosec': header.get('stamp_nanosec', 0),
                'rgb_width': int(header.get('width', frame.shape[1])),
                'rgb_height': int(header.get('height', frame.shape[0])),
                'src_width': int(header.get('src_width', frame.shape[1])),
                'src_height': int(header.get('src_height', frame.shape[0])),
                'detections': detections,
            }
            self.send_result(packet)
            with self.lock:
                self.vis_frame = vis

    def web_loop(self):
        app = Flask(__name__)
        html = """<!DOCTYPE html><html><head><title>YOLO BBox Server</title>
        <style>body{margin:0;background:#111;color:white;text-align:center;font-family:Arial}
        img{max-width:95%;border:2px solid #2ecc71;margin-top:10px}</style></head>
        <body><h2>YOLO BBox Server</h2><img src='/video_feed'></body></html>"""
        @app.route('/')
        def index():
            return render_template_string(html)
        @app.route('/video_feed')
        def video_feed():
            return Response(self.gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
        print(f'🌐 web: http://0.0.0.0:{self.args.web_port}')
        app.run(host='0.0.0.0', port=self.args.web_port, threaded=True)

    def gen_frames(self):
        while self.running:
            with self.lock:
                img = self.vis_frame.copy() if self.vis_frame is not None else None
            if img is None:
                img = np.zeros((360, 640, 3), dtype=np.uint8)
                cv2.putText(img, 'waiting frames...', (30, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)
            ok, jpeg = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if ok:
                yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n'
            time.sleep(0.03)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--rgb-port', type=int, default=9999)
    parser.add_argument('--flyt-pi-host', required=True)
    parser.add_argument('--bbox-port', type=int, default=9997)
    parser.add_argument('--model', default='yolov8n.pt')
    parser.add_argument('--imgsz', type=int, default=480)
    parser.add_argument('--conf', type=float, default=0.45)
    parser.add_argument('--target-class', default='person')
    parser.add_argument('--only-target', action='store_true')
    parser.add_argument('--max-fps', type=float, default=30.0)
    parser.add_argument('--web', action='store_true', default=True)
    parser.add_argument('--web-port', type=int, default=8080)
    return parser.parse_args()


if __name__ == '__main__':
    server = YOLOBBoxServer(parse_args())
    try:
        server.start()
    except KeyboardInterrupt:
        server.running = False
