#!/usr/bin/env python3
"""APPROACH test (no depth) with ground truth from the TFs.

1. Vision with depth: measures the apparent target size D = mean ROI side · r / f.
2. Removes depth (vision_tracker sync_slop_sec = -1) with the target still: it must brake
   from the ROI expansion and stop near target_distance (+ stop_margin).
3. The target flees along the LOS (flee_speed): the pursuer must accelerate (boost).
Restores sync_slop_sec at the end. Usage: approach_test.py [still_s=25] [flee_s=15] [flee_speed=1.0]
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.srv import SetParameters
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo
from aero_pursuit.msg import PursuitStatus, TargetDetection, TargetTfCmd

args = [float(a) for a in sys.argv[1:]]
HOLD, FLEE, FV = (args + [25.0, 15.0, 1.0][len(args):])[:3]
WORLD, CHASER, TARGET = 'lawn', 'x500_depth_0', 'X4'


class T(Node):
    def __init__(self):
        super().__init__('approach_test')
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.tw = self.create_publisher(Twist, f'/{TARGET}/gazebo/command/twist', 10)
        self.cmd = self.create_publisher(TargetTfCmd, '/target_tf_cmd', 10)
        self.vis = self.create_client(SetParameters, '/vision_tracker_node/set_parameters')
        self.acq = self.create_client(SetParameters, '/target_acquisition/set_parameters')
        self.est = self.create_client(SetParameters, '/target_estimator_node/set_parameters')
        self.create_subscription(TargetDetection, '/target_detection', self.on_det, qos_profile_sensor_data)
        self.create_subscription(PursuitStatus, '/pursuit/status', self.on_status, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.fx = None
        self.det = None
        self.status = None
        self.sizes = []

    def on_info(self, m):
        self.fx = m.k[0]

    def on_det(self, m):
        self.det = m
        if m.status == TargetDetection.CONFIRMED and m.has_depth and self.fx:
            r = self.true_range()
            if r:
                side = math.sqrt(m.width * m.height)  # Same as vision_tracker's angular_size
                self.sizes.append(side * r / self.fx)

    def on_status(self, m):
        self.status = m

    def poses(self):
        a = self.buf.lookup_transform(WORLD, TARGET, Time())
        b = self.buf.lookup_transform(WORLD, CHASER, Time())
        return a, b

    def true_range(self):
        try:
            a, b = self.poses()
        except Exception:
            return None
        ta, tb = a.transform.translation, b.transform.translation
        return math.dist((ta.x, ta.y, ta.z), (tb.x, tb.y, tb.z))

    def set(self, cli, name, value):
        cli.wait_for_service(timeout_sec=5.0)
        r = SetParameters.Request(); r.parameters = [Parameter(name, value=value).to_parameter_msg()]
        f = cli.call_async(r); rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)

    def flee_twist(self):
        """Target velocity along the LOS (horizontal), in the X4 body frame."""
        a, b = self.poses()
        ta, tb = a.transform.translation, b.transform.translation
        d = np.array([ta.x - tb.x, ta.y - tb.y]); d /= max(np.linalg.norm(d), 1e-6)
        q = a.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        c, s = math.cos(yaw), math.sin(yaw)
        tw = Twist()
        tw.linear.x = FV * (c * d[0] + s * d[1])
        tw.linear.y = FV * (-s * d[0] + c * d[1])
        return tw


def spin_for(n, s, cb=None):
    t0 = time.monotonic()
    while time.monotonic() - t0 < s:
        rclpy.spin_once(n, timeout_sec=0.02)
        if cb:
            cb()


def log_phase(n, dur, flee):
    rows, t0 = [], time.monotonic()
    while time.monotonic() - t0 < dur:
        if flee:
            try:
                n.tw.publish(n.flee_twist())
            except Exception:
                pass
        rclpy.spin_once(n, timeout_sec=0.02)
        r = n.true_range()
        st = n.status
        if r and st:
            rows.append((time.monotonic() - t0, r, st.mode_name, st.command_speed))
    n.tw.publish(Twist())
    return rows


def summary(name, rows, d_ref):
    if not rows:
        print(f'[{name}] no data'); return
    r = np.array([x[1] for x in rows]); v = np.array([x[3] for x in rows])
    modes = {}
    for x in rows:
        modes[x[2]] = modes.get(x[2], 0) + 1
    print(f'[{name}] modos {modes}')
    for t in np.arange(0, rows[-1][0] + 0.01, 2.5):
        i = min(range(len(rows)), key=lambda k: abs(rows[k][0] - t))
        print(f'   t={rows[i][0]:5.1f}s  d={rows[i][1]:5.2f} m  modo={rows[i][2]:9s}  |v_cmd|={rows[i][3]:.2f}')
    print(f'   final d {r[-20:].mean():.2f} m (ref {d_ref:.2f}); min d {r.min():.2f}; max |v_cmd| {v.max():.2f}')


def main():
    rclpy.init()
    n = T()
    spin_for(n, 1.0)
    # 1. Vision with depth
    c = TargetTfCmd(); c.target_frame = TARGET; c.enable_tf_tracking = False
    for _ in range(3):
        n.cmd.publish(c); spin_for(n, 0.2)
    n.set(n.est, 'use_tf_tracking', False)
    n.set(n.acq, 'enabled', True)
    t0 = time.monotonic()
    while len(n.sizes) < 5 and time.monotonic() - t0 < 40.0:
        spin_for(n, 0.2)
    n.sizes.clear()
    spin_for(n, 8.0)
    if n.sizes:
        s = np.array(n.sizes)
        print(f'tamanho aparente D = sqrt(w h)·r/f: mediana {np.median(s):.3f} m '
              f'(p10 {np.percentile(s,10):.3f}, p90 {np.percentile(s,90):.3f}, n={len(s)})')
    else:
        print('no detections with depth; status', n.det.status if n.det else None)
    d_ref = 5.0
    # 2. No depth, stationary target
    n.set(n.vis, 'sync_slop_sec', -1.0)
    summary('no depth, target still', log_phase(n, HOLD, False), d_ref)
    # 3. Alvo fugindo
    if FLEE > 0:
        summary(f'no depth, target fleeing at {FV:g} m/s', log_phase(n, FLEE, True), d_ref)
    n.set(n.vis, 'sync_slop_sec', 0.05)
    n.destroy_node(); rclpy.shutdown()


main()
