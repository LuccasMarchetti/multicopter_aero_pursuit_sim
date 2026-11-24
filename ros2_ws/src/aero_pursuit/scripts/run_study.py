#!/usr/bin/env python3
"""Runs a study: each scenario x each variant, `repetitions` times, and summarizes the
metrics of the recorded sessions in a CSV.

    ros2 run aero_pursuit run_study.py scenarios/studies/controllers_vs_trajectories.yaml
    ./ros2_ws/run_study.py <estudo.yaml> [<estudo2.yaml> ...] [--dry-run] [--keep-running] [--gui] [--external-sim]

Several files (or a cascade file with `studies: [a.yaml, b.yaml]`) run one after the other on the
same simulation, each with its own folder and summary.csv.

Study file:
    name: my_study
    repetitions: 3
    scenarios: [../figure8_basic.yaml, ...]      # relative to the study file
    common:                                      # merged into every run (optional)
      duration: 40
      pursuer: {detection: omniscient}
    variants:                                    # a list of overrides of scenario fields...
      - {name: hold, parameters: {/pursuit_controller_node: {mpc.params.yaw_mode: 0.0}}}
      - {name: omni, pursuer: {detection: omniscient}}
    axes:                                        # ...and/or axes: every combination of one item
      yaw:                                       #    per axis (cartesian product); the run name
        - {name: B, parameters: {/pursuit_controller_node: {mpc.params.yaw_mode: 1.0}}}
        - {name: opt, parameters: {/pursuit_controller_node: {mpc.params.yaw_mode: 3.0}}}
      target:                                    #    joins the item names: B-t1, B-t3, opt-t1...
        - {name: t1, target_speed: 1.0}
        - {name: t3, target_speed: 3.0}

The simulation is started (headless) by sim_manager.sh if it is not running. Each
run becomes a session in <repo>/datasets (run_id = <scenario>_<variant>_r<k>);
the summary goes to <repo>/datasets/studies/<study>_<date>/summary.csv.
"""

import argparse
import copy
import csv
import itertools
import json
import os
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import yaml

def hms(seconds: float) -> str:
    seconds = int(max(0, seconds))
    return f'{seconds // 3600}:{seconds // 60 % 60:02d}:{seconds % 60:02d}'


class Progress:
    """Runs done / left over the whole cascade, printed at each run and kept in
    <repo>/datasets/study_progress.json (read by `aero study status` and the GCS)."""

    def __init__(self, repo: Path, total: int, tag: str):
        self.path = repo / 'datasets' / 'study_progress.json'
        self.total = total
        self.tag = tag
        self.done = 0
        self.t0 = time.time()
        self.info = {}

    def eta(self) -> float:
        return (time.time() - self.t0) / self.done * (self.total - self.done) if self.done else 0.0

    def write(self, state='running', **info):
        self.info.update(info)
        elapsed = time.time() - self.t0
        doc = {'state': state, 'tag': self.tag, 'done': self.done, 'total': self.total,
               'percent': round(100.0 * self.done / max(1, self.total), 1),
               'started': datetime.fromtimestamp(self.t0).isoformat(timespec='seconds'),
               'elapsed_s': round(elapsed), 'eta_s': round(self.eta()) if self.done else None,
               'finish_estimate': (datetime.fromtimestamp(time.time() + self.eta()).isoformat(timespec='minutes')
                                   if self.done else None),
               'updated': datetime.now().isoformat(timespec='seconds'), **self.info}
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_suffix('.tmp')
            tmp.write_text(json.dumps(doc, indent=1))
            tmp.replace(self.path)
        except OSError:
            pass

    def line(self, what: str):
        eta = f', ETA {hms(self.eta())}' if self.done else ''
        print(f'  [{self.done}/{self.total} runs, {100.0 * self.done / max(1, self.total):.0f} %, '
              f'elapsed {hms(time.time() - self.t0)}{eta}] {what}', flush=True)


def find_repo() -> Path:
    """Repository root (the script runs from the source tree or from install/)."""
    for p in [Path.cwd(), *Path.cwd().parents, Path(__file__).resolve(), *Path(__file__).resolve().parents]:
        if (p / 'ros2_ws' / 'sim_manager.sh').exists():
            return p
    return Path.home() / 'aero_pursuit'


def deep_merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        out[k] = deep_merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else copy.deepcopy(v)
    return out


class Runner:
    def __init__(self, repo: Path, headless: bool):
        self.repo = repo
        self.ws = repo / 'ros2_ws'
        self.headless = headless
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from aero_pursuit.msg import ScenarioStatus
        from aero_pursuit.srv import ScenarioCommand
        rclpy.init()
        self.rclpy = rclpy
        self.node = Node('study_runner')
        self.status = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(ScenarioStatus, '/scenario/status', self._on_status, qos)
        self.client = self.node.create_client(ScenarioCommand, '/scenario_runner/command')
        self.Command = ScenarioCommand

    def _on_status(self, msg):
        self.status = msg

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def sim(self, *args, env=None):
        return subprocess.run([str(self.ws / 'sim_manager.sh'), *args], env=env,
                              capture_output=True, text=True)

    def running(self) -> bool:
        return 'aero_pursuit  running' in self.sim('status').stdout

    def ensure_sim(self, scenario_file: Path, variant: str) -> bool:
        """Starts the simulation if needed. True if it already started the scenario."""
        if self.running():
            return False
        env = dict(os.environ, SKIP_PX4_BUILD='1', FOXGLOVE='0', SCENARIO=str(scenario_file))
        if self.headless:
            env['GZ_HEADLESS'] = '1'
        out = self.sim('start', 'record:=true', 'joystick:=false', f'scenario_variant:={variant}', env=env)
        if out.returncode != 0:
            raise RuntimeError(out.stdout + out.stderr)
        print('  simulation started (the scenario starts when the PX4 takes off)')
        return True

    def command(self, command, path='', variant=''):
        if not self.client.wait_for_service(timeout_sec=120):
            raise RuntimeError('scenario_runner unavailable')
        req = self.Command.Request(command=command, path=path, variant=variant)
        fut = self.client.call_async(req)
        self.rclpy.spin_until_future_complete(self.node, fut, timeout_sec=10)
        res = fut.result()
        if res is None or not res.success:
            raise RuntimeError(res.message if res else 'no answer')
        return res.message

    def wait_finished(self, name: str, timeout: float, progress=None, base=0, reps=1) -> str:
        """Waits for the scenario to finish; `progress` counts each run as it ends (a new
        run_id) and the remaining ones of the combination when it finishes or fails."""
        end = time.monotonic() + timeout
        last = ''
        run_id = ''
        while time.monotonic() < end:
            self.spin(0.5)
            s = self.status
            if not s or s.scenario != name:
                continue
            if progress is not None and s.run_index >= 1 and s.run_id != run_id:  # r0 = loaded, not a run
                if run_id:
                    progress.done = min(base + reps, progress.done + 1)
                run_id = s.run_id
                progress.line(f'{s.run_id} (run {s.run_index}/{s.repeat})')
            if progress is not None:
                progress.write(run_id=s.run_id, run=f'{s.run_index}/{s.repeat}', phase=s.state_name,
                               message=s.message)
            line = f'{s.state_name} {s.run_id}: {s.message}'
            key = re.sub(r'\d+(\.\d+)?', '#', line)   # counters/distances alone: not a new line
            if key != last:
                print(f'    [{s.elapsed:5.0f} s] {line}')
                last = key
            if s.state_name in ('FINISHED', 'ERROR'):
                if progress is not None:
                    progress.done = base + reps
                return s.state_name
        if progress is not None:
            progress.done = base + reps   # skipped: counted, so the estimate stays right
        return 'TIMEOUT'

    def shutdown(self, stop_sim):
        if stop_sim:
            self.sim('stop')
        self.node.destroy_node()
        self.rclpy.shutdown()


def load_session_info(path: Path) -> dict:
    """session_info.yaml; older recorders wrote end_reason unquoted (may contain ': ')."""
    text = path.read_text()
    try:
        return yaml.safe_load(text) or {}
    except yaml.YAMLError:
        fixed = []
        for line in text.splitlines():
            key, sep, value = line.partition(': ')
            if sep and key in ('end_reason', 'scenario', 'run_id', 'detection', 'controller', 'yaw_mode') \
                    and not value.startswith('"'):
                line = f'{key}: "{value.replace(chr(34), chr(39))}"'
            fixed.append(line)
        return yaml.safe_load('\n'.join(fixed)) or {}


AVAILABILITY = ('visible_time', 'box_time', 'box_on_target_time', 'position_time',
                'position_on_target_time')


def availability_from_bag(session: Path) -> dict:
    """The session recorder's `availability` block computed from the rosbag, for sessions
    recorded before it existed: each /pursuit/status interval counts for every item seen less
    than 0.2 s before (target visible, tracker box, box on the target, 3D position)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(session), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(
        topics=['/pursuit/status', '/acquisition/truth', '/target_3d_position']))
    Truth = get_message('aero_pursuit/msg/TrackingTruth')
    confirmed = 3  # TargetDetection.CONFIRMED
    last = dict.fromkeys(('visible', 'box', 'box_on_target', 'position'), -1e9)
    out = dict.fromkeys(AVAILABILITY, 0.0)
    prev = None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        t = stamp * 1e-9
        if topic == '/acquisition/truth':
            m = deserialize_message(data, Truth)
            if m.target_visible:
                last['visible'] = t
            if m.tracker_status == confirmed:
                last['box'] = t
                if m.target_visible and m.on_target:
                    last['box_on_target'] = t
        elif topic == '/target_3d_position':
            last['position'] = t
        else:  # /pursuit/status tick
            if prev is not None and 0.0 <= t - prev <= 1.0:
                dt = t - prev
                fresh = {k: t - v < 0.2 for k, v in last.items()}
                out['visible_time'] += dt * fresh['visible']
                out['box_time'] += dt * fresh['box']
                out['box_on_target_time'] += dt * fresh['box_on_target']
                out['position_time'] += dt * fresh['position']
                out['position_on_target_time'] += dt * (fresh['position'] and fresh['box_on_target'])
            prev = t
    return {k: round(v, 3) for k, v in out.items()}


def summarize(datasets: Path, since: float, out_csv: Path, tag: str = ''):
    rows = []
    for info in sorted(datasets.glob('*/session_info.yaml')):
        if info.stat().st_mtime < since:
            continue
        d = load_session_info(info)
        if 'run_id' not in d:
            continue
        m, t, modes = d.get('metrics') or {}, d.get('tracking') or {}, d.get('time_in_mode') or {}
        st, tg = d.get('solve_time') or {}, d.get('target') or {}
        av = d.get('availability')
        if not av:  # recorded before the recorder wrote it: from the rosbag
            try:
                av = availability_from_bag(info.parent)
            except Exception as e:  # noqa: BLE001 - one unreadable bag must not stop the summary
                print(f'  {info.parent.name}: availability not computed ({e})')
                av = {}
        rows.append({
            **({'simulator': tag} if tag else {}),
            'scenario': d.get('scenario'), 'variant': d.get('variant'), 'run_id': d.get('run_id'),
            'controller': d.get('controller'), 'detection': d.get('detection'),
            'end_reason': d.get('end_reason'), 'duration': d.get('duration'),
            **{k: m.get(k) for k in ('rmse_position', 'rmse_velocity', 'rmse_los', 'rmse_image', 'max_position_error',
                                     'effort_l2', 'engaged_time')},
            **{k: t.get(k) for k in ('tracked_ratio', 'on_target_ratio', 'mean_iou', 'acquisitions',
                                     'identity_switches', 'false_track_ratio', 'max_target_distance')},
            **{f'target_{k}': tg.get(k) for k in ('mean_speed', 'max_speed', 'mean_reference_speed', 'mean_reference_error',
                                                   'max_reference_error', 'mean_range')},
            **{f'solve_{k}': st.get(k) for k in ('mean_us', 'p50_us', 'p95_us', 'p99_us', 'max_us')},
            **{f'time_{k.lower()}': v for k, v in modes.items()},
            **{k: av.get(k) for k in AVAILABILITY},
            'session': str(info.parent),
        })
    if not rows:
        print('No session found for the summary.')
        return
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(out_csv, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    print(f'Summary ({len(rows)} runs): {out_csv}')


def expand(paths) -> list:
    """Study files in order; a file with `studies: [...]` (a cascade) expands to its list,
    relative to it (nested cascades too)."""
    out = []
    for p in paths:
        p = Path(p).resolve()
        doc = yaml.safe_load(p.read_text()) or {}
        if 'studies' in doc:
            out += expand([p.parent / s for s in doc['studies']])
        else:
            out.append(p)
    return out


def combinations(study: dict) -> list:
    """Variants of a study: `variants` (a list) and `axes` (every combination, one item per axis),
    combined when both are given. Each variant: {name, <scenario overrides>}."""
    groups = []
    if study.get('variants'):
        groups.append(study['variants'])
    for items in (study.get('axes') or {}).values():
        groups.append(items)
    if not groups:
        return [{'name': 'base'}]
    out = []
    for combo in itertools.product(*groups):
        v = {'name': '-'.join(str(item['name']) for item in combo)}
        for item in combo:
            v = deep_merge(v, {k: val for k, val in item.items() if k != 'name'})
        out.append(v)
    return out


def count_runs(study_path: Path, args) -> int:
    study = yaml.safe_load(study_path.read_text())
    reps = args.repetitions or int(study.get('repetitions', 1))
    return len(study['scenarios']) * len(combinations(study)) * reps


def run_one(study_path: Path, args, repo: Path, runner, progress=None) -> Path:
    """One study on the running (or started) simulation; returns its output folder."""
    study = yaml.safe_load(study_path.read_text())
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    tag = f'_{args.tag}' if args.tag else ''
    out_dir = repo / 'datasets' / 'studies' / f"{study.get('name', study_path.stem)}{tag}_{stamp}"
    out_dir.mkdir(parents=True, exist_ok=True)
    reps = args.repetitions or int(study.get('repetitions', 1))
    variants = combinations(study)
    common = study.get('common') or {}

    # Combinations (scenario x variant), saved for later reproduction.
    plan = []
    for sc in study['scenarios']:
        base = yaml.safe_load((study_path.parent / sc).resolve().read_text())
        for v in variants:
            over = {k: val for k, val in v.items() if k != 'name'}
            doc = deep_merge(deep_merge(base, common), over)
            doc['repetitions'] = reps
            f = out_dir / f"{doc['name']}__{v['name']}.yaml"
            f.write_text(yaml.safe_dump(doc, sort_keys=False, allow_unicode=True))
            plan.append((f, doc, v['name']))
    (out_dir / 'study.yaml').write_text(study_path.read_text())
    print(f'\n#### {study.get("name", study_path.stem)}: {len(plan)} combinations x {reps} repetitions -> {out_dir}')
    if args.dry_run:
        return out_dir

    t0 = time.time()
    try:
        for i, (f, doc, variant) in enumerate(plan, 1):
            print(f"\n== {doc['name']} / {variant}  (combination {i}/{len(plan)})")
            base = progress.done if progress else 0
            if progress:
                progress.write(study=study.get('name', study_path.stem), out_dir=str(out_dir),
                               combination=f'{i}/{len(plan)}', scenario=doc['name'], variant=variant)
            started = False if args.external_sim else runner.ensure_sim(f, variant)
            if not started:
                print('  ', runner.command('start', path=str(f), variant=variant))
            per_run = (doc.get('duration') or 300) + (doc.get('staging') or {}).get('timeout', 45) + 30
            result = runner.wait_finished(doc['name'], reps * per_run + (180 if started else 30),
                                          progress, base, reps)
            print(f'  -> {result}')
            if result == 'TIMEOUT':
                runner.command('stop')
    finally:
        time.sleep(8)  # the last session_info.yaml is written after the post-roll
        summarize(repo / 'datasets', t0, out_dir / 'summary.csv', args.tag)
    return out_dir


def add_availability(study_dir: Path, repo: Path):
    """Adds/refreshes the availability columns of an existing summary.csv (studies run before
    they existed). The session paths may come from a container (/home/aero/...): they are looked
    up by folder name in <repo>/datasets."""
    path = study_dir / 'summary.csv'
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    for i, r in enumerate(rows, 1):
        session = Path(r.get('session') or '')
        if not session.is_dir():
            session = repo / 'datasets' / session.name
        info = session / 'session_info.yaml'
        av = (load_session_info(info).get('availability') if info.is_file() else None) or {}
        if not av:
            try:
                av = availability_from_bag(session)
            except Exception as e:  # noqa: BLE001
                print(f'  {session.name}: availability not computed ({e})')
        r.update({k: av.get(k, '') for k in AVAILABILITY})
        print(f'\r  {i}/{len(rows)}', end='', flush=True)
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    print(f'\n{path}: availability columns for {len(rows)} runs')


def main():
    if len(sys.argv) > 2 and sys.argv[1] == '--availability':
        # run_study.py --availability <study folder> [...]: only adds the availability columns
        repo = find_repo()
        for d in sys.argv[2:]:
            add_availability(Path(d), repo)
        return 0
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('study', nargs='+',
                    help='one or more studies, run in cascade (one after the other, on the same simulation); '
                         'a file with "studies: [...]" stands for its list')
    ap.add_argument('--dry-run', action='store_true', help='only writes the combined scenarios')
    ap.add_argument('--keep-running', action='store_true', help='does not stop the simulation at the end')
    ap.add_argument('--gui', action='store_true', help='Gazebo with a window (default: headless)')
    ap.add_argument('--external-sim', action='store_true',
                    help='simulator already running outside sim_manager.sh (Isaac Sim: aero study with the GCS on Isaac)')
    ap.add_argument('--repetitions', type=int, help='overrides the repetitions of the file')
    ap.add_argument('--tag', default='', help='label of this batch (e.g. the simulator): in the folder '
                    'name, a "simulator" column of summary.csv and the progress file')
    args = ap.parse_args()

    repo = find_repo()
    studies = expand(args.study)
    if len(studies) > 1:
        print(f'cascade: {len(studies)} studies -> ' + ', '.join(p.stem for p in studies))
    runner = None if args.dry_run else Runner(repo, headless=not args.gui)
    total = sum(count_runs(p, args) for p in studies)
    progress = None if args.dry_run else Progress(repo, total, args.tag)
    print(f'{total} runs in total')
    done = []
    state = 'finished'
    try:
        for i, path in enumerate(studies, 1):
            if len(studies) > 1:
                print(f'\n######## study {i}/{len(studies)}: {path.name}')
            if progress:
                progress.write(cascade=f'{i}/{len(studies)}')
            done.append(run_one(path, args, repo, runner, progress))
    except KeyboardInterrupt:
        print('\nInterrupted.')
        state = 'interrupted'
    except Exception:
        state = 'failed'
        raise
    finally:
        if progress:
            progress.write(state=state)
            progress.line(state)
        if runner is not None:
            runner.shutdown(stop_sim=not (args.keep_running or args.external_sim))
    if len(done) > 1:
        print('\nCascade summary:')
        for d in done:
            print(f'  {d / "summary.csv"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
