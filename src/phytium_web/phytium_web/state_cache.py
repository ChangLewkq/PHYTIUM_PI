import threading
import time
from copy import deepcopy


class StateCache:
    def __init__(self):
        self._lock = threading.Lock()
        self._state = {
            'mode': 'manual',
            'estop': False,
            'robot': {
                'speed': 0,
                'angular': 0,
                'odom_x': 0,
                'odom_y': 0,
                'heading_deg': 0,
                'gyro_z': 0,
                'collision': False,
                'odom_online': False,
                'imu_online': False,
            },
            'vision': {
                'has_target': False,
                'reason': 'no_data',
                'class_name': '',
                'conf': 0,
                'distance_m': None,
                'offset': 0,
                'cmd_linear': 0,
                'cmd_angular': 0,
                'target_online': False,
                'follow_cmd_online': False,
            },
            'radar': {
                'online': False,
                'points': [],
                'nearest_m': None,
                'front_nearest_m': None,
                'left_nearest_m': None,
                'right_nearest_m': None,
            },
            'safety': {
                'online': False,
                'safe': True,
                'reasons': [],
                'estop': False,
                'active_source': 'stop',
                'front_min': None,
                'front_clearance': None,
                'lidar_to_front_offset': 0.15,
                'out_linear': 0,
                'out_angular': 0,
                'limited': {},
            },
            'map': {
                'online': False,
                'width': 0,
                'height': 0,
                'resolution': 0.05,
                'display_width': 0,
                'display_height': 0,
                'display_resolution': 0.05,
                'origin_x': 0.0,
                'origin_y': 0.0,
                'origin_yaw': 0.0,
                'data': [],
                'robot_pose_online': False,
            },
            'system': {},
            'remote': {},
            'nodes': {},
            'timestamp': time.time(),
        }

    def merge(self, key, d):
        with self._lock:
            self._state.setdefault(key, {}).update(d)
            self._state['timestamp'] = time.time()

    def set_mode(self, m):
        if m not in ('manual', 'follow', 'mapping', 'navigation', 'stop'):
            m = 'manual'
        with self._lock:
            self._state['mode'] = m
            self._state['timestamp'] = time.time()

    def set_estop(self, e):
        with self._lock:
            self._state['estop'] = bool(e)
            self._state['timestamp'] = time.time()

    def snapshot(self):
        with self._lock:
            return deepcopy(self._state)

    def get_key(self, key, default=None):
        with self._lock:
            return deepcopy(self._state.get(key, default if default is not None else {}))
