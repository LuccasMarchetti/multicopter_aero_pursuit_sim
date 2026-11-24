#!/usr/bin/env python3
"""True image error of recorded sessions: where the target really was in the camera image.

Uses the Gazebo ground-truth TFs of the bag (world -> pursuer, world -> target) and the camera
mount (static TF base -> camera_link, or base -> gimbal_mount -> camera_link with the measured
gimbal joints), so it does not depend on the estimator or on PursuitMetrics.rmse_image.

    ros2 run aero_pursuit image_error.py datasets/20261115_*_s3_figure8_*          # one line per session
    ./ros2_ws/src/aero_pursuit/scripts/image_error.py --group datasets/2026*_r?    # mean ± std per variant

Columns: rms off-axis sqrt(az² + el²) over the whole engagement [rad]; azimuth/elevation RMS and
mean after --settle seconds [deg] (+ az: target to the left, + el: target above the center).
"""

import argparse
import bisect
import glob
import math
import re
import statistics
from collections import defaultdict

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def rotation(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def analyze(session, args):
    bags = glob.glob(session + '/*.mcap')
    if not bags:
        return None
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bags[0], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    dynamic, static, modes = {}, {}, []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in ('/tf', '/tf_static'):
            for tf in deserialize_message(data, get_message(types[topic])).transforms:
                key = (tf.header.frame_id, tf.child_frame_id)
                t = tf.transform.translation
                value = (np.array([t.x, t.y, t.z]), rotation(tf.transform.rotation))
                if topic == '/tf_static':
                    static[key] = value
                else:
                    stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                    dynamic.setdefault(key, []).append((stamp, value))
        elif topic == '/pursuit/status':
            m = deserialize_message(data, get_message(types[topic]))
            modes.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.mode_name))
    stamps = {}
    for key, seq in dynamic.items():
        seq.sort(key=lambda x: x[0])
        stamps[key] = [s for s, _ in seq]

    def at(key, stamp):
        if key in static:
            return static[key]
        i = min(bisect.bisect_left(stamps[key], stamp), len(stamps[key]) - 1)
        return dynamic[key][i][1]

    base, world = args.base_frame, args.world_frame
    if (world, base) not in dynamic or (world, args.target) not in dynamic:
        return None
    gimbal = (base, 'gimbal_mount') in static and ('gimbal_mount', 'camera_link') in dynamic
    engaged = [s for s, m in modes if m in ('TRACKING', 'COASTING', 'APPROACH')]
    if not engaged:
        return None
    t0, t1 = engaged[0], engaged[-1]
    center = np.array(args.target_center)
    all_sq, late = [], []
    for stamp in (s for s in stamps[(world, base)] if t0 <= s <= t1):
        p_b, R_b = at((world, base), stamp)
        if gimbal:
            p_m, R_m = static[(base, 'gimbal_mount')]
            p_c, R_c = at(('gimbal_mount', 'camera_link'), stamp)
            R_cam, p_cam = R_b @ R_m @ R_c, p_b + R_b @ (p_m + R_m @ p_c)
        elif (base, 'camera_link') in static:
            p_c, R_c = static[(base, 'camera_link')]
            R_cam, p_cam = R_b @ R_c, p_b + R_b @ p_c
        else:
            R_cam, p_cam = R_b, p_b
        p_t, R_t = at((world, args.target), stamp)
        c = R_cam.T @ (p_t + R_t @ center - p_cam)
        az, el = math.atan2(c[1], c[0]), math.atan2(c[2], math.hypot(c[0], c[1]))
        all_sq.append(az * az + el * el)
        if stamp - t0 > args.settle:
            late.append((az, el))
    if not all_sq or not late:
        return None
    late = np.degrees(np.array(late))
    return {'rms_image': math.sqrt(np.mean(all_sq)),
            'az_rms': math.sqrt(np.mean(late[:, 0] ** 2)), 'el_rms': math.sqrt(np.mean(late[:, 1] ** 2)),
            'az_mean': float(np.mean(late[:, 0])), 'el_mean': float(np.mean(late[:, 1])), 'gimbal': gimbal}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sessions', nargs='+', help='session directories (datasets/<stamp>_<run_id>)')
    ap.add_argument('--group', action='store_true', help='mean ± std per run_id without the _r<k> suffix')
    ap.add_argument('--settle', type=float, default=15.0, help='az/el statistics after this time [s]')
    ap.add_argument('--target', default='X4', help='target TF frame')
    ap.add_argument('--base-frame', default='x500_depth_0')
    ap.add_argument('--world-frame', default='lawn')
    ap.add_argument('--target-center', type=float, nargs=3, default=[0.0, 0.0, 0.09],
                    help='visible center of the target in its frame [m] (X4: 0 0 0.09, model_boxes)')
    args = ap.parse_args()

    groups = defaultdict(list)
    for session in sorted(args.sessions):
        res = analyze(session.rstrip('/'), args)
        name = re.sub(r'^\d{8}_\d{6}_', '', session.rstrip('/').split('/')[-1])
        if res is None:
            print(f'{name}: no data (bag, world TFs or engagement missing)')
            continue
        if args.group:
            groups[re.sub(r'_r\d+$', '', name)].append(res)
        else:
            print('%-42s rms_image %.3f rad | az %.2f° el %.2f° | mean az %+.2f° el %+.2f°%s' % (
                name, res['rms_image'], res['az_rms'], res['el_rms'], res['az_mean'], res['el_mean'],
                ' | gimbal' if res['gimbal'] else ''))
    for name, runs in groups.items():
        def stat(key):
            x = [r[key] for r in runs]
            return statistics.mean(x), statistics.stdev(x) if len(x) > 1 else 0.0
        print('%-38s n=%d rms_image %.3f ± %.3f rad | az %.2f° el %.2f° | mean el %+.2f°' % (
            name, len(runs), *stat('rms_image'), stat('az_rms')[0], stat('el_rms')[0], stat('el_mean')[0]))


if __name__ == '__main__':
    main()
