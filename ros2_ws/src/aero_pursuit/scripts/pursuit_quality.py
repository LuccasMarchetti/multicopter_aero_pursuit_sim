#!/usr/bin/env python3
"""Pursuit quality (ground truth from the Gazebo TFs).

Phases: target still (hold_s) and target in lateral zig-zag (zig_s, ±zig_speed m/s, period zig_T).
Reports the distance error (|rel| − target_distance), its oscillation and the pursuer acceleration.
Uso: pursuit_quality.py [hold_s=20] [zig_s=30] [zig_speed=1.0] [zig_T=6.0]
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rcl_interfaces.srv import GetParameters
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Twist

args = [float(a) for a in sys.argv[1:]]
HOLD, ZIG, ZV, ZT = (args + [20.0, 30.0, 1.0, 6.0][len(args):])[:4]
WORLD, CHASER, TARGET = 'lawn', 'x500_depth_0', 'X4'


class Q(Node):
    def __init__(self):
        super().__init__('pursuit_quality')
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.tw = self.create_publisher(Twist, f'/{TARGET}/gazebo/command/twist', 10)
        self.cli = self.create_client(GetParameters, '/pursuit_controller_node/get_parameters')

    def sample(self):
        try:
            a = self.buf.lookup_transform(WORLD, TARGET, Time()).transform.translation
            b = self.buf.lookup_transform(WORLD, CHASER, Time())
        except Exception:
            return None
        t = b.header.stamp.sec + 1e-9 * b.header.stamp.nanosec
        bt = b.transform.translation
        return t, np.array([a.x - bt.x, a.y - bt.y, a.z - bt.z]), np.array([bt.x, bt.y, bt.z])

    def params(self, names):
        self.cli.wait_for_service(timeout_sec=3.0)
        r = GetParameters.Request(); r.names = names
        f = self.cli.call_async(r); rclpy.spin_until_future_complete(self, f, timeout_sec=3.0)
        out = {}
        for n, v in zip(names, f.result().values):
            out[n] = v.double_value if v.type == 3 else v.string_value if v.type == 4 else v.bool_value
        return out


def run_phase(n, dur, zig):
    rec, t0, last = [], time.monotonic(), None
    while time.monotonic() - t0 < dur:
        if zig:
            tw = Twist(); el = time.monotonic() - t0
            tw.linear.y = ZV if (el % ZT) < ZT / 2 else -ZV
            n.tw.publish(tw)
        rclpy.spin_once(n, timeout_sec=0.02)
        s = n.sample()
        if s and (last is None or s[0] > last):
            rec.append(s); last = s[0]
    n.tw.publish(Twist())
    return rec


def report(name, rec, d_ref):
    t = np.array([r[0] for r in rec]); rel = np.array([r[1] for r in rec]); pc = np.array([r[2] for r in rec])
    d = np.linalg.norm(rel, axis=1); e = d - d_ref
    # Pursuer velocity/acceleration by differencing (resampled at 20 Hz)
    tu = np.arange(t[0], t[-1], 0.05)
    p = np.stack([np.interp(tu, t, pc[:, i]) for i in range(3)], 1)
    v = np.gradient(p, 0.05, axis=0); a = np.gradient(v, 0.05, axis=0)
    eu = np.interp(tu, t, e) - np.mean(e)
    zc = np.sum(np.diff(np.sign(eu)) != 0) / (tu[-1] - tu[0]) / 2.0  # error cycles/s
    print(f'[{name}] mean d {d.mean():.2f} m (ref {d_ref:.2f}); mean error {e.mean():+.3f} σ {e.std():.3f} '
          f'faixa {e.min():+.2f}..{e.max():+.2f}; z σ {pc[:,2].std():.3f} m; '
          f'|a| rms {np.sqrt((a**2).sum(1).mean()):.2f} m/s²; osc ~{zc:.2f} Hz')


def main():
    rclpy.init()
    n = Q()
    t0 = time.monotonic()
    while n.sample() is None and time.monotonic() - t0 < 5.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    p = n.params(['target_distance', 'max_vel', 'max_acc', 'vertical_mode', 'mpc.mode', 'controller_type'])
    print('params:', p)
    d_ref = p['target_distance']
    if HOLD > 0:
        report('still', run_phase(n, HOLD, False), d_ref)
    if ZIG > 0:
        report(f'zigue ±{ZV:g} m/s T={ZT:g}s', run_phase(n, ZIG, True), d_ref)
    n.destroy_node(); rclpy.shutdown()


main()
