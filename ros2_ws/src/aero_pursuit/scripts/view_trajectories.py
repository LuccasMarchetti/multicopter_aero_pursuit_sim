#!/usr/bin/env python3
"""3D viewer of the pursuer and target trajectories of every run of one or more studies.

    # studies on this computer (study folders, session folders or a datasets/ folder)
    ./view_trajectories.py datasets/studies/yaw_camera_speed_isaac_20261122_042219
    ./view_trajectories.py datasets/studies/yaw_camera_speed_*

    # studies on another computer (ssh): reads summary.csv there and copies every session into
    # ~/.cache/aero_pursuit/sessions in the background, from the run on screen forward (the
    # title shows "copied n/m"; copies are kept, so the next start is instant)
    ./view_trajectories.py --remote 100.68.147.83 'yaw_camera_speed_*'

Radio buttons on the right pick the runs shown (simulator, scenario, target speed, distance,
MPC formulation, camera; 'all' = no filter); the arrows then move within that selection.
Keys:  → / ←  next / previous run      ↑ / ↓  10 runs forward / back
       e  estimated target on/off      p  planned target path on/off
       r  reload the run list (new runs of a study still running)        q  quit
The slider below the plot moves a time cursor: both drones and the line between them.

Data from the session rosbag (MCAP): /tf (world -> pursuer and world -> target, the simulator
truth), /target_3d_state_filtered (estimate, relative to the pursuer in ENU) and /scenario/paths
(planned target path). Needs ROS 2 sourced (rosbag2_py): source /opt/ros/jazzy/setup.bash
"""
import argparse
import csv
import glob
import subprocess
import shutil
import sys
import tempfile
import textwrap
import threading
import time
from pathlib import Path

import numpy as np

CACHE = Path.home() / '.cache' / 'aero_pursuit' / 'sessions'
RATE_HZ = 20.0      # trajectories resampled to this rate (enough for the plot, keeps it light)


# --------------------------------------------------------------------------------------------
# Run list
# --------------------------------------------------------------------------------------------

def rows_from_summary(text: str, study: str, datasets: str) -> list:
    """Rows of a summary.csv; `session` mapped to <datasets>/<session folder name> (the study
    may have been run in a container where the path was /home/aero/...)."""
    rows = []
    for r in csv.DictReader(text.splitlines()):
        name = Path(r.get('session') or '').name
        if not name:
            continue
        r['study'] = study
        r['session_dir'] = f'{datasets}/{name}'
        rows.append(r)
    return rows


def local_runs(paths: list) -> list:
    runs = []
    for arg in paths:
        for p in sorted(Path(x) for x in (glob.glob(arg) or [arg])):
            if (p / 'summary.csv').is_file():                      # a study folder
                runs += rows_from_summary((p / 'summary.csv').read_text(), p.name,
                                          str(p.resolve().parent.parent))
            elif any(p.glob('*.mcap')):                             # one session folder
                runs.append({'run_id': p.name, 'study': '', 'session_dir': str(p)})
            elif (p / 'studies').is_dir():                          # a datasets/ folder
                runs += local_runs([str(s) for s in sorted((p / 'studies').iterdir())])
            else:
                print(f'skipped {p}: no summary.csv, .mcap or studies/', file=sys.stderr)
    return runs


def ssh(host: str, command: str) -> str:
    return subprocess.run(['ssh', '-o', 'BatchMode=yes', host, command], capture_output=True,
                          text=True, timeout=60).stdout


def remote_runs(host: str, repo: str, patterns: list) -> list:
    datasets = f'{repo}/datasets'
    # the patterns are globs expanded by the remote shell (not quoted on purpose)
    names = ssh(host, f'cd {datasets}/studies && ls -d {" ".join(patterns)} 2>/dev/null').split()
    runs = []
    for name in names:
        text = ssh(host, f'cat {datasets}/studies/{name}/summary.csv 2>/dev/null')
        runs += rows_from_summary(text, name, datasets)
    return runs


def is_copied(session_dir: str) -> bool:
    local = CACHE / Path(session_dir).name
    return local.is_dir() and any(local.glob('*.mcap'))


_fetch_locks = {}
_fetch_locks_guard = threading.Lock()


def fetch(host: str, session_dir: str) -> Path:
    """Local copy of a remote session folder (copied once, then reused).

    One copy per session at a time (the background prefetch and the viewer may ask for the same
    one), each into its own temporary folder, moved into place only when complete."""
    name = Path(session_dir).name
    local = CACHE / name
    with _fetch_locks_guard:
        lock = _fetch_locks.setdefault(name, threading.Lock())
    with lock:
        if local.is_dir() and any(local.glob('*.mcap')):
            return local
        CACHE.mkdir(parents=True, exist_ok=True)
        tmp = Path(tempfile.mkdtemp(prefix=f'.{name}.', dir=CACHE))
        try:
            subprocess.run(['scp', '-q', '-r', '-o', 'BatchMode=yes', f'{host}:{session_dir}',
                            str(tmp)], check=True, timeout=600)
            shutil.rmtree(local, ignore_errors=True)   # an incomplete copy from an older version
            (tmp / name).rename(local)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
        return local


# --------------------------------------------------------------------------------------------
# Reading a session
# --------------------------------------------------------------------------------------------

def resample(ts: list, xs: list, t0: float):
    """[(t - t0), xyz] at RATE_HZ (last sample per bin)."""
    if not ts:
        return np.empty(0), np.empty((0, 3))
    t = np.asarray(ts) - t0
    x = np.asarray(xs)
    order = np.argsort(t, kind='stable')
    t, x = t[order], x[order]
    bins = np.floor(t * RATE_HZ)
    keep = np.r_[bins[1:] != bins[:-1], True]
    return t[keep], x[keep]


def load_session(folder: Path) -> dict:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import MarkerArray

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(folder), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    available = {t.name for t in reader.get_all_topics_and_types()}
    wanted = ['/tf', '/target_3d_state_filtered', '/scenario/paths']
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in wanted if t in available]))

    poses = {}            # child frame of the world -> ([t], [xyz])
    world = None
    est_t, est_rel = [], []
    planned, target_name = [], None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        t = stamp * 1e-9
        if topic == '/tf':
            for tf in deserialize_message(data, TFMessage).transforms:
                parent, child = tf.header.frame_id, tf.child_frame_id
                # World frame: the parent of the model poses (lawn, world, ...).
                # (not x500_*_enu, a child of the pursuer body: a bag that starts with it picked the body as the world)
                if world is None and not child.endswith('_enu') and (
                        child.startswith('x500') or child in ('X3', 'X4', 'X5')):
                    world = parent
                if parent != world:
                    continue
                p = tf.transform.translation
                ts, xs = poses.setdefault(child, ([], []))
                ts.append(t)
                xs.append((p.x, p.y, p.z))
        elif topic == '/target_3d_state_filtered':
            p = deserialize_message(data, Odometry).pose.pose.position
            est_t.append(t)
            est_rel.append((p.x, p.y, p.z))
        elif topic == '/scenario/paths':
            for m in deserialize_message(data, MarkerArray).markers:
                if m.ns.startswith('scenario/') and m.points:
                    target_name = m.ns.split('/', 1)[1]
                    planned = [(q.x, q.y, q.z) for q in m.points]

    pursuer = next((c for c in poses if c.startswith('x500') and not c.endswith('_enu')), None)
    if target_name not in poses:
        others = [c for c in poses if c != pursuer and 'camera' not in c and not c.endswith('_enu')]
        # the target is the model that moved the most
        target_name = max(others, key=lambda c: np.ptp(np.asarray(poses[c][1]), axis=0).sum(),
                          default=None)
    if pursuer is None or target_name is None:
        raise RuntimeError(f'no pursuer/target poses in /tf (frames: {sorted(poses)})')

    t0 = min(poses[pursuer][0][0], poses[target_name][0][0])
    tp, xp = resample(*poses[pursuer], t0)
    tt, xt = resample(*poses[target_name], t0)
    te, xe = resample(est_t, est_rel, t0)
    if len(te):  # relative (pursuer-centred ENU) -> world: add the pursuer position at that time
        xe = xe + np.column_stack([np.interp(te, tp, xp[:, i]) for i in range(3)])
    return {'pursuer': (tp, xp), 'target': (tt, xt), 'estimate': (te, xe),
            'planned': np.asarray(planned), 'names': (pursuer, target_name)}


# --------------------------------------------------------------------------------------------
# Viewer
# --------------------------------------------------------------------------------------------

METRICS = [('rmse_position', 'pos RMSE', 'm'), ('rmse_image', 'image RMSE', 'rad'),
           ('time_lost', 'lost', 's'), ('tracked_ratio', 'tracked', ''),
           ('target_mean_speed', 'target speed', 'm/s'), ('target_mean_range', 'mean range', 'm'),
           ('solve_mean_us', 'solve', 'µs')]


ALL = 'all'
# run attribute -> radio button group title
SELECTORS = [('simulator', 'simulator'), ('scenario', 'scenario'), ('speed', 'target speed'),
             ('distance', 'distance'), ('formulation', 'MPC'), ('camera', 'camera')]
SPEED_TOKENS = {'v3': '3 m/s', 'v6_5': '6.5 m/s', 'v10': '10 m/s'}


def run_attr(r: dict, key: str) -> str:
    """simulator/scenario from summary.csv; speed, distance, formulation and camera from the
    variant tokens (e.g. pampc-track-v6_5-d10)."""
    if key in ('simulator', 'scenario'):
        return r.get(key, '')
    tokens = r.get('variant', '').split('-')
    for t in tokens:
        if key == 'speed' and t in SPEED_TOKENS:
            return SPEED_TOKENS[t]
        if key == 'distance' and t[:1] == 'd' and t[1:].isdigit():
            return f'{t[1:]} m'
        if key == 'formulation' and t in ('noyaw', 'pampc'):
            return {'noyaw': 'translational', 'pampc': 'perception-aware'}[t]
        if key == 'camera' and t in ('fixed', 'track'):
            return {'fixed': 'fixed', 'track': 'gimbal'}[t]
    return ''


def sort_key(v: str):
    try:
        return (0, float(v.split()[0]), v)
    except ValueError:
        return (1, 0.0, v)


class Viewer:
    def __init__(self, args):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Slider
        self.plt = plt
        self.args = args
        self.cache = {}
        self.lock = threading.Lock()
        self.show_est = True
        self.show_plan = True
        self.i = 0
        self.all_runs = self.list_runs()
        if not self.all_runs:
            sys.exit('no runs found')
        self.sel = {key: ALL for key, _ in SELECTORS}
        self.runs = self.all_runs
        self.fig = plt.figure(figsize=(13, 8.5))
        self.ax = self.fig.add_axes([0.0, 0.1, 0.62, 0.85], projection='3d')
        self.info = self.fig.add_axes([0.63, 0.45, 0.36, 0.5])
        self.info.axis('off')
        self.build_selectors()
        sax = self.fig.add_axes([0.1, 0.03, 0.6, 0.03])
        self.slider = Slider(sax, 't [s]', 0.0, 1.0, valinit=0.0)
        self.slider.on_changed(lambda _: self.draw_cursor())
        self.cursor = []
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.copying = set()
        if args.remote and not args.no_copy_all:
            for _ in range(max(1, args.copies)):
                threading.Thread(target=self.copy_all, daemon=True).start()
            # refresh the "copied n/m" counter in the title
            self.timer = self.fig.canvas.new_timer(interval=2000)
            self.timer.add_callback(lambda: (self.fig.suptitle(self.title(), fontsize=11),
                                             self.fig.canvas.draw_idle()))
            self.timer.start()
        self.show()
        plt.show()

    def list_runs(self) -> list:
        a = self.args
        runs = remote_runs(a.remote, a.repo, a.paths) if a.remote else local_runs(a.paths)
        if a.filter:
            runs = [r for r in runs if all(f in (r.get('run_id', '') + ' ' + r.get('variant', ''))
                                           for f in a.filter)]
        return runs

    def build_selectors(self):
        """One radio button group per run attribute (simulator, scenario, speed, ...), each with
        'all'; only the values present in the runs are offered."""
        from matplotlib.widgets import RadioButtons
        self.radios = []
        groups = [(key, label, [ALL] + sorted({run_attr(r, key) for r in self.all_runs} - {''},
                                              key=sort_key))
                  for key, label in SELECTORS]
        groups = [g for g in groups if len(g[2]) > 2]   # a single value: nothing to choose
        width = 0.36 / max(1, len(groups))
        for n, (key, label, values) in enumerate(groups):
            rax = self.fig.add_axes([0.63 + n * width, 0.1, width - 0.005, 0.33])
            rax.set_title(label, fontsize=9)
            radio = RadioButtons(rax, values, active=0)
            for t in radio.labels:
                t.set_fontsize(8)
            radio.on_clicked(lambda value, key=key: self.select(key, value))
            self.radios.append(radio)

    def select(self, key: str, value: str):
        current = self.runs[self.i].get('session_dir') if self.runs else None
        self.sel[key] = value
        runs = [r for r in self.all_runs
                if all(v == ALL or run_attr(r, k) == v for k, v in self.sel.items())]
        if not runs:
            self.fig.suptitle('no run with this selection', fontsize=11)
            self.fig.canvas.draw_idle()
            self.runs = []
            return
        self.runs = runs
        self.i = next((k for k, r in enumerate(runs) if r.get('session_dir') == current), 0)
        self.show()

    def session(self, k: int):
        r = self.runs[k]
        key = r['session_dir']
        with self.lock:
            if key in self.cache:
                return self.cache[key]
        try:
            folder = fetch(self.args.remote, key) if self.args.remote else Path(key)
            data = load_session(folder)
        except Exception as e:  # noqa: BLE001 - show the error, keep browsing
            data = e
        with self.lock:
            self.cache[key] = data
        return data

    def prefetch(self, k: int):
        if self.args.remote and 0 <= k < len(self.runs):
            threading.Thread(target=self.session, args=(k,), daemon=True).start()

    def copied(self) -> int:
        return sum(is_copied(r['session_dir']) for r in self.runs)

    def copy_all(self):
        """Background copier: copies every session of the list, one at a time, always the first
        missing one from the run on screen forward (so the ones you are about to see come first).
        Runs added by 'r' are picked up too."""
        while True:
            runs, start = self.runs, self.i
            with self.lock:  # several copiers: each takes a session nobody is copying
                missing = next((s for s in (runs[(start + d) % len(runs)]['session_dir']
                                            for d in range(len(runs)))
                                if s not in self.copying and not is_copied(s)), None)
                if missing is not None:
                    self.copying.add(missing)
            if missing is None:
                time.sleep(5.0)
                continue
            try:
                fetch(self.args.remote, missing)
            except Exception as e:  # noqa: BLE001 - skip it, try again on the next round
                print(f'copy of {Path(missing).name} failed: {e}', file=sys.stderr)
                time.sleep(5.0)
            finally:
                with self.lock:
                    self.copying.discard(missing)

    def title(self) -> str:
        r = self.runs[self.i]
        text = f"[{self.i + 1}/{len(self.runs)}] {r.get('run_id', '')}"
        if self.args.remote:
            text += f'   (copied {self.copied()}/{len(self.runs)})'
        return text

    def show(self):
        ax, r = self.ax, self.runs[self.i]
        title = self.title()
        self.fig.suptitle(title + ('  (loading...)' if self.args.remote else ''), fontsize=11)
        self.fig.canvas.draw_idle()
        self.plt.pause(0.01)
        data = self.session(self.i)
        self.prefetch(self.i + 1)
        ax.clear()
        self.cursor = []
        self.fig.suptitle(title, fontsize=11)
        self.info.clear()
        self.info.axis('off')
        lines = [f"study: {r.get('study', '')}", f"simulator: {r.get('simulator', '')}",
                 f"scenario: {r.get('scenario', '')}", f"variant: {r.get('variant', '')}",
                 f"controller: {r.get('controller', '')}", f"end: {r.get('end_reason', '')}", '']
        for key, label, unit in METRICS:
            v = r.get(key)
            if v not in (None, ''):
                lines.append(f'{label}: {v} {unit}')
        if isinstance(data, Exception):
            lines += ['', f'ERROR: {data}']
            self.info.text(0, 1, '\n'.join(lines), va='top', fontsize=9, family='monospace',
                           wrap=True)
            self.fig.canvas.draw_idle()
            return
        lines += ['', f'pursuer: {data["names"][0]}', f'target: {data["names"][1]}']
        text = '\n'.join(textwrap.fill(line, 36, subsequent_indent='  ') if line else ''
                         for line in lines)
        self.info.text(0, 1, text, va='top', fontsize=9, family='monospace')
        tp, xp = data['pursuer']
        tt, xt = data['target']
        ax.plot(*xp.T, color='tab:blue', lw=1.4, label='pursuer')
        ax.plot(*xt.T, color='tab:orange', lw=1.4, label='target (truth)')
        te, xe = data['estimate']
        if self.show_est and len(te):
            ax.plot(*xe.T, '.', color='tab:green', ms=1.5, alpha=0.5, label='target (estimate)')
        if self.show_plan and len(data['planned']):
            ax.plot(*data['planned'].T, '--', color='0.6', lw=0.8, label='target plan')
        for x, c in ((xp, 'tab:blue'), (xt, 'tab:orange')):
            ax.scatter(*x[0], color=c, marker='o', s=30)
            ax.scatter(*x[-1], color=c, marker='x', s=40)
        pts = np.vstack([xp, xt])
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        mid, half = (lo + hi) / 2, max((hi - lo).max() / 2, 1.0)
        ax.set_xlim(mid[0] - half, mid[0] + half)
        ax.set_ylim(mid[1] - half, mid[1] + half)
        ax.set_zlim(max(0.0, mid[2] - half), mid[2] + half)
        ax.set_box_aspect((1, 1, 1))
        ax.set_xlabel('x (east) [m]')
        ax.set_ylabel('y (north) [m]')
        ax.set_zlabel('z [m]')
        ax.legend(loc='upper left', fontsize=8)
        t_end = float(max(tp[-1], tt[-1]))
        self.slider.valmax = t_end
        self.slider.ax.set_xlim(0, t_end)
        self.slider.set_val(min(self.slider.val, t_end))
        self.draw_cursor()

    def draw_cursor(self):
        data = self.cache.get(self.runs[self.i]['session_dir'])
        if not isinstance(data, dict):
            return
        for a in self.cursor:
            a.remove()
        t = self.slider.val
        tp, xp = data['pursuer']
        tt, xt = data['target']
        p = np.array([np.interp(t, tp, xp[:, i]) for i in range(3)])
        q = np.array([np.interp(t, tt, xt[:, i]) for i in range(3)])
        self.cursor = [self.ax.scatter(*p, color='tab:blue', s=60, edgecolor='k'),
                       self.ax.scatter(*q, color='tab:orange', s=60, edgecolor='k'),
                       self.ax.plot(*np.vstack([p, q]).T, color='k', lw=0.8)[0],
                       self.ax.text2D(0.02, 0.02, f't = {t:.1f} s   distance = {np.linalg.norm(q - p):.2f} m',
                                      transform=self.ax.transAxes, fontsize=9)]
        self.fig.canvas.draw_idle()

    def on_key(self, ev):
        step = {'right': 1, 'left': -1, 'up': 10, 'down': -10}.get(ev.key)
        if not self.runs:
            return
        if step:
            self.i = (self.i + step) % len(self.runs)
        elif ev.key == 'e':
            self.show_est = not self.show_est
        elif ev.key == 'p':
            self.show_plan = not self.show_plan
        elif ev.key == 'r':
            current = self.runs[self.i].get('session_dir')
            self.all_runs = self.list_runs() or self.all_runs
            self.runs = [r for r in self.all_runs
                         if all(v == ALL or run_attr(r, k) == v for k, v in self.sel.items())] or self.all_runs
            self.i = next((k for k, r in enumerate(self.runs) if r.get('session_dir') == current), 0)
            print(f'{len(self.runs)} runs')
        else:
            return
        self.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('paths', nargs='+', help='study folders / session folders / datasets folder '
                    '(with --remote: study folder names or globs under <repo>/datasets/studies)')
    ap.add_argument('--remote', help='ssh host holding the studies (e.g. 100.68.147.83)')
    ap.add_argument('--repo', default='~/aero_pursuit', help='repository on the remote host')
    ap.add_argument('--no-copy-all', action='store_true',
                    help='with --remote: copy only the run on screen and the next one '
                    '(default: every run of the list is copied in the background)')
    ap.add_argument('--copies', type=int, default=3,
                    help='with --remote: sessions copied in parallel in the background (default 3)')
    ap.add_argument('--filter', action='append', default=[],
                    help='only runs whose run_id/variant contain this text (repeatable: all must match)')
    args = ap.parse_args()
    try:
        import rosbag2_py  # noqa: F401
    except ImportError:
        sys.exit('rosbag2_py not found: source /opt/ros/jazzy/setup.bash first')
    Viewer(args)


if __name__ == '__main__':
    main()
