#!/usr/bin/env python3
"""Analysis of a study run by run_study.py: statistics per variant and plots from the rosbags.

    ros2 run aero_pursuit analyze_study.py datasets/studies/<study>_<date>
    ./ros2_ws/src/aero_pursuit/scripts/analyze_study.py <study folder> [--no-bags]

Reads <folder>/summary.csv (one row per run, written by run_study.py from each session's
session_info.yaml) and writes into the same folder:

    analysis.csv        per variant: runs, end reasons, mean and standard deviation of each metric
                        (over scenarios and repetitions)
    metric_<name>.png   bar chart per metric (mean +- std per variant)
    range_<scenario>.png  distance to the target along the run, one line per run, colored by
                          variant (from /pursuit/status in each session's MCAP rosbag)
    timeseries.csv      the same series as numbers: run_id, variant, t [s], range, range_error,
                        command_speed, image_azimuth [deg] (for your own analysis)

The rosbag of a run is the .mcap file in the session folder (column `session`). Any topic recorded
there can be read the same way (see read_series below): /pursuit/status, /pursuit/metrics,
/pursuit/predicted_path, /fmu/out/vehicle_odometry, /tf, /target_3d_state_filtered, ...
"""
import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path

METRICS = [  # (column of summary.csv, label, unit)
    ('rmse_position', 'Position RMSE (standoff point)', 'm'),
    ('rmse_velocity', 'Relative velocity RMSE', 'm/s'),
    ('rmse_image', 'Target-in-image RMSE', 'rad'),
    ('max_position_error', 'Max position error', 'm'),
    ('effort_l2', 'Effort ∫|a|² dt', 'm²/s³'),
    ('engaged_time', 'Engaged time', 's'),
    ('time_tracking', 'Time in TRACKING', 's'),
    ('time_lost', 'Time in LOST', 's'),
    ('visible_time', 'Target in the image (truth)', 's'),
    ('box_time', 'Tracker box (any object)', 's'),
    ('box_on_target_time', 'Tracker box on the target', 's'),
    ('position_time', 'Target 3D position available', 's'),
    ('position_on_target_time', '3D position of the right target', 's'),
    ('target_mean_speed', 'Target speed achieved (mean)', 'm/s'),
    ('target_mean_reference_error', 'Target off its reference (mean)', 'm'),
    ('target_mean_range', 'Pursuer-target distance (mean)', 'm'),
    ('solve_mean_us', 'MPC solve time (mean)', 'µs'),
    ('solve_p95_us', 'MPC solve time (p95)', 'µs'),
    ('solve_max_us', 'MPC solve time (max)', 'µs'),
]


def number(x):
    try:
        v = float(x)
        return v if math.isfinite(v) else None
    except (TypeError, ValueError):
        return None


def load_rows(folder: Path) -> list:
    with open(folder / 'summary.csv', newline='') as f:
        return list(csv.DictReader(f))


def per_variant(rows: list) -> dict:
    out = defaultdict(list)
    for r in rows:
        out[r.get('variant') or 'base'].append(r)
    return dict(sorted(out.items()))


def write_analysis(folder: Path, groups: dict):
    keys = [m[0] for m in METRICS]
    with open(folder / 'analysis.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['variant', 'runs', 'end_reasons'] + [f'{k}_{s}' for k in keys for s in ('mean', 'std')])
        for variant, rows in groups.items():
            reasons = defaultdict(int)
            for r in rows:
                reasons[r.get('end_reason') or '?'] += 1
            line = [variant, len(rows), '; '.join(f'{k} x{n}' for k, n in sorted(reasons.items()))]
            for k in keys:
                vals = [v for v in (number(r.get(k)) for r in rows) if v is not None]
                line += [statistics.fmean(vals) if vals else '',
                         statistics.stdev(vals) if len(vals) > 1 else (0.0 if vals else '')]
            w.writerow(line)
    print(f'  {folder / "analysis.csv"}')


def plot_metrics(folder: Path, groups: dict):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    names = list(groups)
    for key, label, unit in METRICS:
        means, stds = [], []
        for v in names:
            vals = [x for x in (number(r.get(key)) for r in groups[v]) if x is not None]
            means.append(statistics.fmean(vals) if vals else 0.0)
            stds.append(statistics.stdev(vals) if len(vals) > 1 else 0.0)
        if not any(means):
            continue
        fig, ax = plt.subplots(figsize=(max(5, 0.8 * len(names) + 2), 3.6))
        ax.bar(names, means, yerr=stds, capsize=4, color='#4c8bd6')
        ax.set_ylabel(f'{label} [{unit}]')
        ax.set_title(label)
        ax.tick_params(axis='x', rotation=45)
        fig.tight_layout()
        fig.savefig(folder / f'metric_{key}.png', dpi=120)
        plt.close(fig)
    print(f'  {folder}/metric_*.png')


def read_series(bag_dir: Path, topics: dict) -> dict:
    """{topic: [(t [s], message), ...]} from the MCAP rosbag of a session folder.
    `topics`: {topic: message type string}, e.g. {'/pursuit/status': 'aero_pursuit/msg/PursuitStatus'}."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    types = {t: get_message(ty) for t, ty in topics.items()}
    out = defaultdict(list)
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        out[topic].append((stamp * 1e-9, deserialize_message(data, types[topic])))
    return out


def plot_series(folder: Path, rows: list):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    by_scenario = defaultdict(list)
    table = []
    variants = sorted({r.get('variant') or 'base' for r in rows})
    colors = {v: plt.cm.tab10(i % 10) for i, v in enumerate(variants)}
    for r in rows:
        session = Path(r.get('session') or '')
        if not session.is_dir() or not any(session.glob('*.mcap')):
            continue
        try:
            s = read_series(session, {'/pursuit/status': 'aero_pursuit/msg/PursuitStatus',
                                      '/pursuit/metrics': 'aero_pursuit/msg/PursuitMetrics'})
        except Exception as e:  # noqa: BLE001 - one broken bag must not stop the analysis
            print(f'  {session.name}: unreadable bag ({e})')
            continue
        status = s.get('/pursuit/status', [])
        if not status:
            continue
        t0 = status[0][0]
        az = {round(t, 2): math.degrees(m.image_azimuth) for t, m in s.get('/pursuit/metrics', [])}
        ts = [t - t0 for t, _ in status]
        rng = [m.range for _, m in status]
        by_scenario[r.get('scenario') or '?'].append((r.get('variant') or 'base', ts, rng))
        for (t, m) in status:
            table.append([r.get('run_id'), r.get('variant'), round(t - t0, 3), m.range, m.range_error,
                          m.command_speed, az.get(round(t, 2), '')])
    with open(folder / 'timeseries.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['run_id', 'variant', 't', 'range', 'range_error', 'command_speed', 'image_azimuth_deg'])
        w.writerows(table)
    for scenario, runs in by_scenario.items():
        fig, ax = plt.subplots(figsize=(8, 4))
        seen = set()
        for variant, ts, rng in runs:
            ax.plot(ts, rng, color=colors[variant], lw=1.0, alpha=0.85,
                    label=None if variant in seen else variant)
            seen.add(variant)
        ax.set_xlabel('t [s]')
        ax.set_ylabel('distance to the target [m]')
        ax.set_title(scenario)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(folder / f'range_{scenario}.png', dpi=120)
        plt.close(fig)
    print(f'  {folder / "timeseries.csv"} and range_*.png ({sum(len(v) for v in by_scenario.values())} bags)')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('folder', help='datasets/studies/<study>_<date> (with summary.csv)')
    ap.add_argument('--no-bags', action='store_true', help='only the statistics (no rosbag reading)')
    a = ap.parse_args()
    folder = Path(a.folder).resolve()
    rows = load_rows(folder)
    groups = per_variant(rows)
    print(f'{folder.name}: {len(rows)} runs, {len(groups)} variants')
    write_analysis(folder, groups)
    plot_metrics(folder, groups)
    if not a.no_bags:
        plot_series(folder, rows)


if __name__ == '__main__':
    main()
