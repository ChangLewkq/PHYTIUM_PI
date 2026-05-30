#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Feature task process manager for phytium_web.

This version adds stale-process cleanup.

Why:
- `ros2 launch` is a parent process.
- Sometimes SIGINT stops the launch process but a child node such as
  `async_slam_toolbox_node` may remain alive.
- Then TaskManager says "stopped", but `ros2 node list` still shows /slam_toolbox.

This file fixes that by applying task-specific cleanup patterns after stopping.
"""

import os
import signal
import subprocess
import time
from typing import Dict, List, Optional

from .remote_vision_controller import RemoteVisionController


def _sanitize_target_class(value):
    t = str(value or 'person').strip().lower()
    t = ''.join(ch for ch in t if ch.isalnum() or ch in ('_', '-'))
    return t or 'person'


class TaskManager:
    def __init__(self, log_dir: str = '/tmp/phytium_web_tasks'):
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)
        self.remote_vision = RemoteVisionController(log_dir=self.log_dir)

        self._tasks: Dict[str, Dict] = {
            'vision': {
                'label': '视觉跟随',
                'cmd': ['ros2', 'launch', 'phytium_vision', 'vision_client.launch.py'],
                'process': None,
                'started_at': None,
                'log_path': os.path.join(self.log_dir, 'vision.log'),
                'cleanup_patterns': [
                    'vision_client.launch.py',
                    'rgb_sender',
                    'target_depth_follower',
                    'realsense2_camera_node',
                ],
            },
            'mapping': {
                'label': 'SLAM 建图',
                'cmd': ['ros2', 'launch', 'phytium_car_bringup', 'slam_mapping.launch.py'],
                'process': None,
                'started_at': None,
                'log_path': os.path.join(self.log_dir, 'mapping.log'),
                'cleanup_patterns': [
                    'slam_mapping.launch.py',
                    'async_slam_toolbox_node',
                    'sync_slam_toolbox_node',
                    'slam_toolbox',
                ],
            },
            'navigation': {
                'label': 'Nav2 导航',
                'cmd': ['ros2', 'launch', 'phytium_navigation', 'nav_direct_cold.launch.py'],
                'process': None,
                'started_at': None,
                'log_path': os.path.join(self.log_dir, 'navigation.log'),
                'cleanup_patterns': [
                    'nav_direct_cold.launch.py',
                    'nav_direct_cold.launch.py',
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'bt_navigator',
                    'lifecycle_manager_localization',
                    'lifecycle_manager_navigation',
                    'nav2_lifecycle_manager/lifecycle_manager',
                    'scan_amcl_delay.py',
                ],
            },
        }

    def _get(self, name: str) -> Optional[Dict]:
        return self._tasks.get(str(name))

    @staticmethod
    def _is_running(proc: Optional[subprocess.Popen]) -> bool:
        return proc is not None and proc.poll() is None

    def is_running(self, name: str) -> bool:
        t = self._get(name)
        return bool(t and self._is_running(t.get('process')))

    def start(self, name: str, target_class: Optional[str] = None) -> Dict:
        t = self._get(name)
        if not t:
            return {'ok': False, 'task': name, 'reason': 'unknown_task'}

        # If stale nodes from the same task exist, clean them first.
        # This prevents duplicate /slam_toolbox, stale Nav2 lifecycle nodes, etc.
        self.cleanup_task(name, polite=True)

        proc = t.get('process')
        if self._is_running(proc):
            return {
                'ok': True,
                'task': name,
                'status': 'already_running',
                'pid': proc.pid,
                'log_path': t.get('log_path'),
            }

        remote_result = None
        cmd = list(t['cmd'])
        vision_target_class = _sanitize_target_class(target_class)

        if name == 'vision':
            remote_result = self.remote_vision.start(
                target_class=vision_target_class,
                force_restart=True,
            )
            cmd.append('target_class:={}'.format(vision_target_class))

            if self.remote_vision.required and not remote_result.get('ok', False):
                return {
                    'ok': False,
                    'task': name,
                    'status': 'remote_vision_failed',
                    'target_class': vision_target_class,
                    'remote': remote_result,
                }

        log_path = t['log_path']
        os.makedirs(os.path.dirname(log_path), exist_ok=True)

        log_file = open(log_path, 'a', buffering=1)
        if remote_result is not None:
            log_file.write('REMOTE_VISION_START: {}\n'.format(remote_result))
            log_file.write('VISION_TARGET_CLASS: {}\n'.format(vision_target_class))
        log_file.write('\n\n===== START {} at {} =====\n'.format(name, time.strftime('%F %T')))

        try:
            proc = subprocess.Popen(
                cmd,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                text=True,
                preexec_fn=os.setsid,
            )
            t['process'] = proc
            t['started_at'] = time.time()
            t['log_file'] = log_file

            result = {
                'ok': True,
                'task': name,
                'status': 'started',
                'pid': proc.pid,
                'cmd': ' '.join(cmd),
                'log_path': log_path,
            }
            if name == 'vision':
                result['target_class'] = vision_target_class
            if remote_result is not None:
                result['remote'] = remote_result
            return result

        except Exception as e:
            try:
                log_file.write('START FAILED: {}\n'.format(e))
                log_file.close()
            except Exception:
                pass

            t['process'] = None
            t['started_at'] = None

            return {
                'ok': False,
                'task': name,
                'reason': str(e),
                'cmd': ' '.join(cmd),
                'log_path': log_path,
            }

    def stop(self, name: str, timeout: float = 4.0) -> Dict:
        t = self._get(name)
        if not t:
            return {'ok': False, 'task': name, 'reason': 'unknown_task'}

        proc = t.get('process')
        pid = proc.pid if proc else None
        status = 'not_running'

        if self._is_running(proc):
            status = 'stopped'

            try:
                os.killpg(os.getpgid(pid), signal.SIGINT)
            except Exception:
                try:
                    proc.terminate()
                except Exception:
                    pass

            deadline = time.time() + timeout
            while time.time() < deadline:
                if proc.poll() is not None:
                    break
                time.sleep(0.1)

            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(pid), signal.SIGTERM)
                except Exception:
                    try:
                        proc.kill()
                    except Exception:
                        pass

            try:
                proc.wait(timeout=1.0)
            except Exception:
                pass

        self._close_log(t)

        t['process'] = None
        t['started_at'] = None

        cleanup = self.cleanup_task(name, polite=False)
        remote_result = None
        if name == 'vision':
            remote_result = self.remote_vision.stop()

        result = {
            'ok': True,
            'task': name,
            'status': status,
            'pid': pid,
            'cleanup': cleanup,
        }
        if remote_result is not None:
            result['remote'] = remote_result
        return result

    def stop_many(self, names: List[str]) -> Dict:
        results = {}
        for name in names:
            results[name] = self.stop(name)
        return results

    def stop_all_features(self) -> Dict:
        return self.stop_many(['vision', 'mapping', 'navigation'])

    def status(self) -> Dict:
        out = {}
        now = time.time()

        for name, t in self._tasks.items():
            proc = t.get('process')
            running = self._is_running(proc)

            exit_code = None
            if proc is not None and not running:
                exit_code = proc.poll()
                self._close_log(t)
                t['process'] = None
                t['started_at'] = None

            started_at = t.get('started_at')
            out[name] = {
                'label': t.get('label'),
                'running': running,
                'pid': proc.pid if running else None,
                'uptime_sec': round(now - started_at, 1) if running and started_at else 0.0,
                'cmd': ' '.join(t.get('cmd', [])),
                'log_path': t.get('log_path'),
                'exit_code': exit_code,
            }
            if name == 'vision':
                out[name]['remote'] = self.remote_vision.status()

        return out

    def cleanup_task(self, name: str, polite: bool = False) -> Dict:
        """Kill stale processes by command-line patterns.

        polite=True only sends SIGINT.
        polite=False sends SIGINT, then SIGTERM for stubborn leftovers.

        We use pkill -f patterns because ROS2 node children may survive after
        `ros2 launch` has exited.
        """
        t = self._get(name)
        if not t:
            return {'ok': False, 'task': name, 'reason': 'unknown_task'}

        patterns = t.get('cleanup_patterns', [])
        results = []

        # Do not let pkill match this web_server python command accidentally.
        # Patterns are task-specific ROS process names / launch names.
        for pattern in patterns:
            results.append(self._pkill(pattern, signal_name='INT'))

        time.sleep(0.4)

        if not polite:
            for pattern in patterns:
                results.append(self._pkill(pattern, signal_name='TERM'))

        return {
            'ok': True,
            'task': name,
            'patterns': patterns,
            'results': results,
        }

    def _pkill(self, pattern: str, signal_name: str = 'TERM') -> Dict:
        try:
            p = subprocess.run(
                ['pkill', '-' + signal_name, '-f', pattern],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=2.0,
                check=False,
            )

            # pkill return codes:
            # 0 = matched and signaled
            # 1 = no process matched
            # both are acceptable for cleanup.
            return {
                'pattern': pattern,
                'signal': signal_name,
                'returncode': p.returncode,
                'ok': p.returncode in (0, 1),
                'stderr': (p.stderr or '').strip(),
            }
        except Exception as e:
            return {
                'pattern': pattern,
                'signal': signal_name,
                'returncode': -1,
                'ok': False,
                'stderr': str(e),
            }


    # ---------- Transition readiness detection ----------
    def wait_mode_ready(self, mode: str, timeout_sec: float = 20.0) -> Dict:
        """Wait until selected mode is ready.

        Two-stage result:
        - ready: node/lifecycle is fully detected
        - started: launch process is alive, but node/lifecycle was not observed in time

        The frontend should not show "timeout failure" when a feature launch is alive.
        On small ARM boards ROS graph discovery may lag behind launch startup.
        """
        mode = str(mode or 'manual')
        start = time.time()
        timeout_sec = max(0.5, float(timeout_sec))
        deadline = start + timeout_sec
        samples = []

        def task_running(task_name: str) -> bool:
            return self.is_running(task_name)

        def finish(ok: bool, reason: str, stage: str = 'ready', detail: Optional[Dict] = None):
            now = time.time()
            return {
                'ok': bool(ok),
                'mode': mode,
                'reason': reason,
                'stage': stage,  # ready / started / timeout
                'duration_sec': round(now - start, 3),
                'timeout_sec': timeout_sec,
                'detail': detail or {},
                'samples': samples[-6:],
            }

        if mode == 'stop':
            return finish(True, 'stop_mode_no_task_wait', 'ready')

        # Manual must really clean feature nodes/processes.
        if mode == 'manual':
            while time.time() < deadline:
                nodes = self._ros2_nodes()
                samples.append({'t': round(time.time() - start, 2), 'nodes': nodes[:40]})
                leftovers = [
                    n for n in nodes
                    if n in (
                        '/slam_toolbox',
                        '/map_server',
                        '/amcl',
                        '/planner_server',
                        '/controller_server',
                        '/bt_navigator',
                        '/rgb_sender',
                        '/target_depth_follower',
                    )
                    or 'slam_toolbox' in n
                    or 'bt_navigator' in n
                    or 'controller_server' in n
                    or 'planner_server' in n
                ]
                process_leftovers = self._feature_process_lines()
                task_leftovers = [
                    t for t in ('vision', 'mapping', 'navigation')
                    if task_running(t)
                ]
                samples[-1]['process_leftovers'] = process_leftovers[:12]
                samples[-1]['task_leftovers'] = task_leftovers

                if not leftovers:
                    return finish(True, 'base_only_ready', 'ready', {
                        'leftovers': leftovers,
                        'process_leftovers': process_leftovers[:12],
                        'task_leftovers': task_leftovers,
                    })

                # ROS graph may keep stale lifecycle/node names for a few seconds.
                # If actual launch processes and task handles are gone, manual is safe.
                if not process_leftovers and not task_leftovers:
                    return finish(True, 'base_only_ready_ros_graph_stale', 'ready', {
                        'leftovers': leftovers,
                        'process_leftovers': [],
                        'task_leftovers': [],
                    })

                time.sleep(0.25)
            # Timeout reached. Do one final hard cleanup before reporting failure.
            try:
                final_cleanup = self.stop_feature_tasks()
                time.sleep(1.0)
                final_process_leftovers = self._feature_process_lines()
                final_task_leftovers = [
                    t for t in ('vision', 'mapping', 'navigation')
                    if task_running(t)
                ]
                if not final_process_leftovers and not final_task_leftovers:
                    return finish(True, 'base_only_ready_after_force_cleanup', 'ready', {
                        'final_cleanup': final_cleanup,
                        'process_leftovers': [],
                        'task_leftovers': [],
                    })
                return finish(False, 'timeout_waiting_manual_cleanup', 'timeout', {
                    'final_cleanup': final_cleanup,
                    'process_leftovers': final_process_leftovers[:12],
                    'task_leftovers': final_task_leftovers,
                })
            except Exception as e:
                return finish(False, 'timeout_waiting_manual_cleanup_exception', 'timeout', {
                    'error': str(e),
                })

        # Other modes: detect full readiness, but accept started process as a non-failing intermediate.
        task_name = {
            'mapping': 'mapping',
            'navigation': 'navigation',
            'follow': 'vision',
        }.get(mode)

        last_nodes = []
        while time.time() < deadline:
            nodes = self._ros2_nodes()
            last_nodes = nodes
            sample = {'t': round(time.time() - start, 2), 'nodes': nodes[:40]}
            samples.append(sample)

            if mode == 'mapping':
                if any('slam_toolbox' in n for n in nodes):
                    return finish(True, 'slam_toolbox_ready', 'ready', {'matched': '/slam_toolbox'})
                # The launch process itself being alive means the switch has at least started successfully.
                if task_running('mapping') and time.time() - start > 2.0:
                    # Keep waiting for full node ready, but remember this is not a hard failure.
                    sample['launch_process'] = 'running'

            elif mode == 'navigation':
                # Direct Nav2 mode does not use bt_navigator.
                # Functional readiness is: map published + compute_path action + follow_path action + cmd_vel_nav.
                nav_ready = self._nav_direct_ready_check()
                sample['nav_direct_ready'] = {
                    'ok': nav_ready.get('ok', False),
                    'reason': nav_ready.get('reason'),
                    'localization_ok': nav_ready.get('localization_ok'),
                    'planner_ok': nav_ready.get('planner_ok'),
                    'controller_ok': nav_ready.get('controller_ok'),
                    'map_ok': nav_ready.get('map_ok'),
                    'compute_action_servers': nav_ready.get('compute_action_servers'),
                    'follow_action_servers': nav_ready.get('follow_action_servers'),
                }

                if nav_ready.get('ok', False):
                    return finish(True, nav_ready.get('reason', 'nav_direct_ready'), 'ready', {
                        'nav_direct_ready': nav_ready,
                    })

                if task_running('navigation') and time.time() - start > 3.0:
                    sample['launch_process'] = 'running'

            elif mode == 'follow':
                has_rgb = any('rgb_sender' in n for n in nodes)
                has_target = any('target_depth_follower' in n for n in nodes)
                has_camera = any('camera' in n for n in nodes)
                remote_status = self.remote_vision.status()
                sample['remote_vision_online'] = remote_status.get('online', False)

                if (has_rgb or has_target or has_camera) and remote_status.get('online', False):
                    return finish(True, 'vision_local_and_remote_ready', 'ready', {
                        'rgb_sender': has_rgb,
                        'target_depth_follower': has_target,
                        'camera': has_camera,
                        'remote': remote_status,
                    })

                if has_rgb or has_target or has_camera:
                    return finish(True, 'vision_local_ready_remote_pending', 'started', {
                        'rgb_sender': has_rgb,
                        'target_depth_follower': has_target,
                        'camera': has_camera,
                        'remote': remote_status,
                    })

                if task_running('vision') and time.time() - start > 2.0:
                    sample['launch_process'] = 'running'

            time.sleep(0.35)

        # Timeout fallback: if launch is still alive, do not mark this as a failed switch.
        # This avoids a false "timeout" UI when ROS graph discovery is slow.
        if task_name and task_running(task_name):
            reason = {
                'mapping': 'mapping_launch_running_node_not_seen_yet',
                'navigation': 'navigation_launch_running_lifecycle_not_active_yet',
                'follow': 'vision_launch_running_node_not_seen_yet',
            }.get(mode, 'launch_running')
            detail = {'last_nodes': last_nodes[:40]}
            if mode == 'navigation':
                detail['bt_lifecycle'] = self._ros2_lifecycle('/bt_navigator')
            return finish(True, reason, 'started', detail)

        return finish(False, 'timeout_waiting_' + mode, 'timeout', {'last_nodes': last_nodes[:40]})



    def _nav_direct_ready_check(self) -> Dict:
        """Run functional direct-nav readiness check.

        It considers Nav2 ready when map is published and direct action servers exist,
        instead of relying only on lifecycle CLI state, which is unstable on Foxy.
        """
        try:
            proc = subprocess.run(
                [
                    'bash', '-lc',
                    'source /opt/ros/foxy/setup.bash && '
                    'source /home/user/phytium_ws/install/setup.bash && '
                    'python3 /home/user/phytium_ws/web_nav_direct_ready_check.py'
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=10.0,
                check=False,
            )
            out = (proc.stdout or '').strip()
            data = {}
            if out:
                try:
                    import json
                    data = json.loads(out)
                except Exception:
                    data = {'ok': False, 'reason': 'ready_check_output_not_json', 'raw': out[-1000:]}
            else:
                data = {'ok': False, 'reason': 'ready_check_empty_stdout'}

            data['returncode'] = proc.returncode
            data['stderr'] = (proc.stderr or '').strip()[-1000:]
            return data
        except Exception as e:
            return {'ok': False, 'reason': str(e)}


    def _feature_process_lines(self) -> List[str]:
        """Return real feature processes that should be gone in manual mode.

        Used to avoid false manual cleanup timeout caused by stale ROS graph discovery.
        """
        try:
            pattern = (
                "nav_direct_cold|bringup_nav|nav2.launch|map_server|amcl|"
                "planner_server|controller_server|bt_navigator|scan_amcl_delay|"
                "lifecycle_manager|bringup_slam|slam_toolbox|vision_client|"
                "rgb_sender|target_depth_follower|realsense2_camera"
            )
            cmd = (
                "ps -eo pid,ppid,stat,etime,cmd | "
                "egrep '" + pattern + "' | "
                "grep -v egrep || true"
            )
            proc = subprocess.run(
                ['bash', '-lc', cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=2.0,
                check=False,
            )
            lines = []
            for line in (proc.stdout or '').splitlines():
                line = line.strip()
                if not line:
                    continue
                # Exclude the diagnostic shell/grep command itself.
                if 'grep -E nav_direct_cold' in line:
                    continue
                if 'egrep ' in line and 'nav_direct_cold' in line:
                    continue
                if 'bash -lc ps -eo' in line:
                    continue
                lines.append(line)
            return lines
        except Exception:
            return []


    def _ros2_nodes(self) -> List[str]:
        try:
            p = subprocess.run(
                ['ros2', 'node', 'list'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=4.0,
                check=False,
            )
            if p.returncode != 0:
                return []
            return [line.strip() for line in p.stdout.splitlines() if line.strip()]
        except Exception:
            return []

    def _ros2_lifecycle(self, node_name: str) -> str:
        try:
            p = subprocess.run(
                ['ros2', 'lifecycle', 'get', node_name],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=4.0,
                check=False,
            )
            return (p.stdout or '').strip()
        except Exception as e:
            return str(e)

    def _close_log(self, t: Dict):
        f = t.get('log_file')
        if f:
            try:
                f.write('===== STOP at {} =====\n'.format(time.strftime('%F %T')))
                f.close()
            except Exception:
                pass
            t['log_file'] = None
