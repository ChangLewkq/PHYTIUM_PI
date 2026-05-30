#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TCP protocol helpers for phytium_vision.

RGB frame packet:
  uint32 big-endian header_length
  header JSON bytes
  JPEG payload bytes, length recorded in header["payload_len"]

Detection packet:
  one JSON object per line, UTF-8, ending with "\n"
"""

import json
import socket
import struct
from typing import Dict, Tuple


def recv_exact(conn: socket.socket, n: int) -> bytes:
    data = b""
    while len(data) < n:
        chunk = conn.recv(n - len(data))
        if not chunk:
            raise ConnectionError('socket closed')
        data += chunk
    return data


def send_frame(sock: socket.socket, header: Dict, payload: bytes) -> None:
    header = dict(header)
    header['payload_len'] = len(payload)
    raw_header = json.dumps(header, separators=(',', ':')).encode('utf-8')
    sock.sendall(struct.pack('>I', len(raw_header)) + raw_header + payload)


def recv_frame(conn: socket.socket) -> Tuple[Dict, bytes]:
    header_len = struct.unpack('>I', recv_exact(conn, 4))[0]
    if header_len <= 0 or header_len > 1024 * 1024:
        raise ValueError(f'invalid header_len={header_len}')
    header = json.loads(recv_exact(conn, header_len).decode('utf-8'))
    payload_len = int(header.get('payload_len', 0))
    if payload_len <= 0 or payload_len > 50 * 1024 * 1024:
        raise ValueError(f'invalid payload_len={payload_len}')
    payload = recv_exact(conn, payload_len)
    return header, payload


def json_line(obj: Dict) -> bytes:
    return json.dumps(obj, ensure_ascii=False, separators=(',', ':')).encode('utf-8') + b'\n'
