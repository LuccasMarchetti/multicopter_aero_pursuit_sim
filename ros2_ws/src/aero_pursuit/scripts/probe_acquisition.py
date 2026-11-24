#!/usr/bin/env python3
"""Validation probe (simulation): automatic ROI accuracy, camera alignment during staging
and actual depth camera range.

Records, on every tracker frame: scenario state, true distance to the target, IoU and center
error (/acquisition/truth), measured depth and range classification
(/target_detection), pursuer heading and direction to the target (TF). Also keeps raw
statistics of the depth image (values beyond far: +inf? saturated?).

    ros2 run aero_pursuit probe_acquisition.py --out resultado.json --duration 120
"""
import argparse
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener

from aero_pursuit.msg import ScenarioStatus, TargetDetection, TrackingTruth

STATES = {0: 'IDLE', 1: 'LOADED', 2: 'STAGING', 3: 'RUNNING', 4: 'PAUSED', 5: 'FINISHED'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self, args):
        super().__init__('probe_acquisition')
        self.args = args
        self.rows, self.depth_stats = [], []
        self.truth = None
        self.status = None
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(TargetDetection, '/target_detection', self.on_det, qos_profile_sensor_data)
        self.create_subscription(TrackingTruth, '/acquisition/truth', self.on_truth, 10)
        self.create_subscription(ScenarioStatus, '/scenario/status', self.on_status,
                                 rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                                                      reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
        self.create_subscription(Image, '/depth_camera', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera', self.on_rgb, qos_profile_sensor_data)
        self.last_depth = 0.0
        self.last_rgb = 0.0
        self.frames = 0
        self.create_timer(0.1, self.on_timer)
        self.yaw_rows = []

    def on_status(self, m):
        self.status = m

    def on_truth(self, m):
        self.truth = m

    def pose(self, frame):
        try:
            t = self.tf.lookup_transform(self.args.world, frame, rclpy.time.Time())
            return t.transform
        except Exception:  # noqa: BLE001
            return None

    def on_timer(self):
        p, t = self.pose(self.args.pursuer), self.pose(self.args.target)
        if not p or not t:
            return
        d = (t.translation.x - p.translation.x, t.translation.y - p.translation.y, t.translation.z - p.translation.z)
        bearing = math.atan2(d[1], d[0])
        yaw = yaw_of(p.rotation)
        err = math.degrees(math.atan2(math.sin(yaw - bearing), math.cos(yaw - bearing)))
        self.yaw_rows.append({'t': time.time(), 'state': STATES.get(self.status.state, '?') if self.status else '?',
                              'yaw_error_deg': err, 'range': math.sqrt(sum(x * x for x in d)),
                              'elevation_deg': math.degrees(math.atan2(d[2], math.hypot(d[0], d[1])))})

    def on_det(self, m):
        tr = self.truth
        self.rows.append({
            't': time.time(), 'state': STATES.get(self.status.state, '?') if self.status else '?',
            'status': m.status, 'score': m.score, 'box': [m.x, m.y, m.width, m.height],
            'has_depth': m.has_depth, 'depth': None if math.isnan(m.depth) else m.depth,
            'beyond': m.depth_beyond_range, 'valid_fraction': m.depth_valid_fraction,
            'truth_distance': None if not tr or math.isnan(tr.target_distance) else tr.target_distance,
            'iou': tr.iou if tr else None, 'center_error_px': None if not tr or math.isnan(tr.center_error_px) else tr.center_error_px,
            'gt_box': [tr.gt_x, tr.gt_y, tr.gt_width, tr.gt_height] if tr else None,
            'acquisitions': tr.acquisitions if tr else 0, 'on_target': tr.on_target if tr else False})

    def on_rgb(self, m):
        """Frame with the projected (ground truth) and tracker boxes, for visual inspection."""
        now = time.time()
        if not self.args.frames or now - self.last_rgb < self.args.frame_period or self.frames >= self.args.frames:
            return
        tr = self.truth
        if not tr or tr.gt_width <= 0 or not self.status or self.status.state not in (2, 3):
            return
        self.last_rgb = now
        img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)[:, :, :3].copy()
        if m.encoding.lower().startswith('bgr'):
            img = img[:, :, ::-1].copy()

        def rect(x, y, w, h, color, t=3):
            x0, y0, x1, y1 = max(0, x), max(0, y), min(m.width - 1, x + w), min(m.height - 1, y + h)
            img[y0:y0 + t, x0:x1] = color
            img[y1 - t:y1, x0:x1] = color
            img[y0:y1, x0:x0 + t] = color
            img[y0:y1, x1 - t:x1] = color
        rect(tr.gt_x, tr.gt_y, tr.gt_width, tr.gt_height, (255, 60, 40))
        det = self.rows[-1] if self.rows else None
        if det and det['status'] >= 2:
            rect(*det['box'], (40, 200, 255), 2)
        # enlarged crop around the projected box
        cx, cy = tr.gt_x + tr.gt_width // 2, tr.gt_y + tr.gt_height // 2
        half = max(60, int(1.6 * max(tr.gt_width, tr.gt_height)))
        crop = img[max(0, cy - half):cy + half, max(0, cx - half):cx + half]
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        dist = tr.target_distance
        plt.imsave(f"{self.args.frames_dir}/frame_{self.frames:02d}_{dist:05.1f}m.png", crop)
        self.frames += 1

    def on_depth(self, m):
        now = time.time()
        if now - self.last_depth < 1.0 or m.encoding != '32FC1':
            return
        self.last_depth = now
        a = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
        finite = a[np.isfinite(a)]
        self.depth_stats.append({
            't': now, 'pos_inf': int(np.isposinf(a).sum()), 'neg_inf': int(np.isneginf(a).sum()),
            'nan': int(np.isnan(a).sum()), 'finite': int(finite.size),
            'max_finite': float(finite.max()) if finite.size else None,
            'min_finite': float(finite.min()) if finite.size else None,
            'at_far_19.1': int((np.abs(a - 19.1) < 1e-3).sum())})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--duration', type=float, default=120)
    ap.add_argument('--world', default='lawn')
    ap.add_argument('--pursuer', default='x500_depth_0')
    ap.add_argument('--target', default='X4')
    ap.add_argument('--frames', type=int, default=0, help='frames with the projected box to save')
    ap.add_argument('--frame-period', type=float, default=3.0)
    ap.add_argument('--frames-dir', default='.')
    args = ap.parse_args()
    rclpy.init()
    node = Probe(args)
    t0 = time.time()
    try:
        while rclpy.ok() and time.time() - t0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.status and node.status.state == 5 and time.time() - t0 > 10:
                break
    finally:
        with open(args.out, 'w') as f:
            json.dump({'detections': node.rows, 'yaw': node.yaw_rows, 'depth_stats': node.depth_stats}, f)
        node.get_logger().info(f'{len(node.rows)} frames, {len(node.yaw_rows)} heading samples -> {args.out}')
        rclpy.shutdown()


if __name__ == '__main__':
    main()
