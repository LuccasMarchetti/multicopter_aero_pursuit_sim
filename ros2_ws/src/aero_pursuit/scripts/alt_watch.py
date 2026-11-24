#!/usr/bin/env python3
"""Follows the pursuer altitude, tilt and distance (TFs) for N s, 2 lines/s.

Uso: alt_watch.py [dur_s=15]
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
rclpy.init()
n = Node('alt_watch')
buf = Buffer()
TransformListener(buf, n)
t0 = time.monotonic()
last = -1.0
while time.monotonic() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)
    el = time.monotonic() - t0
    if el - last < 0.5:
        continue
    try:
        c = buf.lookup_transform('lawn', 'x500_depth_0', Time())
        x = buf.lookup_transform('lawn', 'X4', Time()).transform.translation
    except Exception:
        continue
    last = el
    p, q = c.transform.translation, c.transform.rotation
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x ** 2 + q.y ** 2)))))
    d = math.dist((p.x, p.y, p.z), (x.x, x.y, x.z))
    print(f't={el:5.1f}s z={p.z:6.2f} m  incl={tilt:5.1f}°  d={d:5.2f} m', flush=True)
