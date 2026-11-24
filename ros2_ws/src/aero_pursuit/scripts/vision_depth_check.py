#!/usr/bin/env python3
"""Checks whether depth enters the estimate in vision mode.

Desliga o omnisciente, liga /target_acquisition e compara TargetDetection.depth e
|/target_3d_state_filtered| against the real distance (Gazebo TFs).
Usage: vision_depth_check.py [duration_s] [--zigzag]
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from aero_pursuit.msg import TargetDetection, TargetTfCmd

DUR = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith('-') else 30.0
ZIGZAG = '--zigzag' in sys.argv
WORLD, CHASER, TARGET = 'lawn', 'x500_depth_0', 'X4'


class Check(Node):
    def __init__(self):
        super().__init__('vision_depth_check')
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.tw = self.create_publisher(Twist, f'/{TARGET}/gazebo/command/twist', 10)
        self.cmd = self.create_publisher(TargetTfCmd, '/target_tf_cmd', 10)
        self.acq = self.create_client(SetParameters, '/target_acquisition/set_parameters')
        self.create_subscription(TargetDetection, '/target_detection', self.on_det, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/target_3d_state_filtered', self.on_est, 50)
        self.det, self.est = [], []
        self.status = {}

    def truth(self, stamp):
        try:
            st = Time.from_msg(stamp)
            a = self.buf.lookup_transform(WORLD, TARGET, st).transform.translation
            b = self.buf.lookup_transform(WORLD, CHASER, st).transform.translation
        except Exception:
            return None
        return float(np.linalg.norm([a.x - b.x, a.y - b.y, a.z - b.z]))

    def on_det(self, m):
        self.status[m.status] = self.status.get(m.status, 0) + 1
        if m.status != TargetDetection.CONFIRMED:
            return
        d = self.truth(m.header.stamp)
        if d is not None:
            self.det.append((m.has_depth, float(m.depth), d))

    def on_est(self, m):
        d = self.truth(m.header.stamp)
        if d is not None:
            p = m.pose.pose.position
            self.est.append((float(np.linalg.norm([p.x, p.y, p.z])), d))


def spin_for(n, s):
    t0 = time.monotonic()
    while time.monotonic() - t0 < s:
        rclpy.spin_once(n, timeout_sec=0.01)


def main():
    rclpy.init()
    n = Check()
    spin_for(n, 1.0)
    c = TargetTfCmd(); c.target_frame = TARGET; c.enable_tf_tracking = False
    for _ in range(3):
        n.cmd.publish(c); spin_for(n, 0.2)
    est = n.create_client(SetParameters, '/target_estimator_node/set_parameters')
    if est.wait_for_service(timeout_sec=5.0):
        r = SetParameters.Request()
        r.parameters = [Parameter('use_tf_tracking', value=False).to_parameter_msg()]
        rclpy.spin_until_future_complete(n, est.call_async(r), timeout_sec=5.0)
    if n.acq.wait_for_service(timeout_sec=5.0):
        req = SetParameters.Request()
        req.parameters = [Parameter('enabled', value=True).to_parameter_msg()]
        f = n.acq.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=5.0)
    else:
        print('no /target_acquisition'); return
    # Espera confirmar
    t0 = time.monotonic()
    while not n.det and time.monotonic() - t0 < 30.0:
        spin_for(n, 0.2)
    if not n.det:
        print('tracker did not confirm; statuses seen:', n.status); return
    n.det.clear(); n.est.clear()
    t0 = time.monotonic()
    while time.monotonic() - t0 < DUR:
        if ZIGZAG:
            t = Twist(); el = time.monotonic() - t0
            t.linear.y = 1.0 if (el % 6.0) < 3.0 else -1.0
            n.tw.publish(t)
        spin_for(n, 0.05)
    n.tw.publish(Twist())

    det = np.array(n.det) if n.det else np.zeros((0, 3))
    est = np.array(n.est) if n.est else np.zeros((0, 2))
    print(f'status: {n.status}')
    if len(det):
        hd = det[:, 0] > 0.5
        print(f'CONFIRMED detections={len(det)}  with depth={hd.mean()*100:.1f} %')
        if hd.any():
            err = det[hd, 1] - det[hd, 2]
            print(f'depth: {det[hd,1].min():.2f}–{det[hd,1].max():.2f} m; real {det[hd,2].min():.2f}–'
                  f'{det[hd,2].max():.2f} m; depth−true mean {err.mean():+.3f} σ {err.std():.3f}')
    if len(est):
        err = est[:, 0] - est[:, 1]
        print(f'|estimate|−true: mean {err.mean():+.3f} m  σ {err.std():.3f}  '
              f'|max| {np.abs(err).max():.3f}  (n={len(est)}, mean true {est[:,1].mean():.2f} m)')
    n.destroy_node(); rclpy.shutdown()


main()
