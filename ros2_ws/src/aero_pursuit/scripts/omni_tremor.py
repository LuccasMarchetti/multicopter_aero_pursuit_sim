#!/usr/bin/env python3
"""Measures the omniscient estimator error: /target_3d_state_filtered vs TF(lawn,X4)-TF(lawn,x500)."""
import math, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from aero_pursuit.msg import TargetTfCmd

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
LIFT = '--no-lift' not in sys.argv


class M(Node):
    def __init__(self):
        super().__init__('omni_tremor')
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.en = self.create_publisher(Bool, '/X4/enable', 10)
        self.tw = self.create_publisher(Twist, '/X4/gazebo/command/twist', 10)
        self.cmd = self.create_publisher(TargetTfCmd, '/target_tf_cmd', 10)
        self.create_subscription(Odometry, '/target_3d_state_filtered', self.cb, 50)
        self.rec = []
        self.frame = None
        self.nstamp = 0

    def cb(self, m):
        self.frame = m.header.frame_id
        p = m.pose.pose.position
        try:
            st = Time.from_msg(m.header.stamp)
            a = self.buf.lookup_transform('lawn', 'X4', st)
            b = self.buf.lookup_transform('lawn', 'x500_depth_0', st)
            self.nstamp += 1
        except Exception:
            return
        ta, tb = a.transform.translation, b.transform.translation
        tr = np.array([ta.x - tb.x, ta.y - tb.y, ta.z - tb.z])
        self.rec.append((time.monotonic(), np.array([p.x, p.y, p.z]), tr))


def spin_for(n, s):
    t0 = time.monotonic()
    while time.monotonic() - t0 < s:
        rclpy.spin_once(n, timeout_sec=0.01)


def main():
    rclpy.init()
    n = M()
    spin_for(n, 1.0)
    n.en.publish(Bool(data=True))
    if LIFT:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            t = Twist(); t.linear.z = 1.5; n.tw.publish(t); spin_for(n, 0.05)
        n.tw.publish(Twist())
    c = TargetTfCmd(); c.target_frame = 'X4'; c.enable_tf_tracking = True
    for _ in range(5):
        n.cmd.publish(c); spin_for(n, 0.2)
    spin_for(n, 5.0)  # assentar
    n.rec.clear()
    t0 = time.monotonic()
    while time.monotonic() - t0 < DUR:
        t = Twist(); el = time.monotonic() - t0
        t.linear.y = 2.0 if (el % 4.0) < 2.0 else -2.0
        n.tw.publish(t); spin_for(n, 0.05)
    n.tw.publish(Twist())
    r = n.rec
    if len(r) < 10:
        print('few samples', len(r)); return
    ts = np.array([x[0] for x in r]); est = np.array([x[1] for x in r]); tru = np.array([x[2] for x in r])
    e = np.linalg.norm(est - tru, axis=1)
    de = np.linalg.norm(np.diff(est, axis=0), axis=1); dt_ = np.linalg.norm(np.diff(tru, axis=0), axis=1)
    rate = len(r) / (ts[-1] - ts[0])
    print(f'frame={n.frame} samples={len(r)} rate={rate:.1f} Hz')
    print(f'mean error={e.mean():.3f} m  p95={np.percentile(e,95):.3f}  max={e.max():.3f}')
    print(f'error per axis (mean abs) = {np.abs(est-tru).mean(axis=0).round(3)}')
    print(f'step est/true = {de.sum()/max(dt_.sum(),1e-9):.3f}')
    print(f'stamp ok={n.nstamp}')
    for lag in range(0,6):
        ee=np.linalg.norm(est[lag:]-tru[:len(tru)-lag],axis=1); print(f'  delay {lag} samples: mean error {ee.mean():.3f}')
    print(f'mean |rel|={np.linalg.norm(tru,axis=1).mean():.2f} m')
    n.destroy_node(); rclpy.shutdown()


main()
