#!/usr/bin/env python3
"""Batch tuning in SITL: Nelder-Mead in log(theta) around the study runner.

Each cost evaluation runs every training scenario once (fixed seeds) with the
candidate gains and computes the common cost J from the recorded session:

    J = rmse_pos^2 + 0.2 rmse_vel^2 + 0.05 effort_l2 / T + 10 T_LOST / T

    ./ros2_ws/tune_study.py scenarios/tuning/pid_omniscient.yaml [--max-evals 15] [--gui]
    ./ros2_ws/tune_study.py <tuning.yaml> --dry-run        # print the plan only

Tuning file:
    name: pid_omniscient
    scenarios: [../figure8_basic.yaml]           # training scenarios (relative to this file)
    seeds: [100, 101]                            # one run per (scenario, seed)
    base:                                        # overrides merged into every scenario
      pursuer: {detection: omniscient}
      parameters: {/pursuit_controller_node: {controller_type: pbvs_pid}}
    px4_params: {MPC_XY_VEL_P_ACC: 1.8}          # PX4 variant (set via sim_manager.sh px4)
    theta:                                       # tuned parameters and starting point
      - {node: /pursuit_controller_node, name: pid.kp, start: 0.8, min: 0.05, max: 20}
      - {node: /pursuit_controller_node, name: pid.kd, start: 0.2, min: 0.0, max: 5}
    max_evals: 15
    initial_step: 0.5                            # simplex size in log units

Results go to <repo>/datasets/tuning/<name>_<date>/: evaluations.csv (one row per
evaluation) and best.yaml (best theta and J).
"""

import argparse
import copy
import csv
import math
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_study import Runner, deep_merge, find_repo, load_session_info  # noqa: E402

LOST_WEIGHT = 10.0


def session_cost(info: dict) -> float:
    """Common cost J of one recorded session (session_info.yaml)."""
    m = info.get('metrics') or {}
    modes = info.get('time_in_mode') or {}
    duration = max(1e-3, float(info.get('duration') or m.get('engaged_time') or 1.0))
    engaged = max(1e-3, float(m.get('engaged_time') or duration))
    rp = float(m.get('rmse_position') or 0.0)
    rv = float(m.get('rmse_velocity') or 0.0)
    effort = float(m.get('effort_l2') or 0.0) / engaged
    lost = float(modes.get('LOST', 0.0))
    return rp * rp + 0.2 * rv * rv + 0.05 * effort + LOST_WEIGHT * lost / duration


def nelder_mead(f, x0, step, max_evals, log=print):
    """Plain Nelder-Mead (standard coefficients); returns (x_best, f_best)."""
    n = len(x0)
    simplex = [list(x0)] + [[x0[j] + (step if j == i else 0.0) for j in range(n)] for i in range(n)]
    values = [f(x) for x in simplex]
    evals = len(values)
    while evals < max_evals:
        order = sorted(range(n + 1), key=lambda i: values[i])
        simplex = [simplex[i] for i in order]
        values = [values[i] for i in order]
        log(f'  simplex best J = {values[0]:.4f} after {evals} evaluations')
        centroid = [sum(p[j] for p in simplex[:-1]) / n for j in range(n)]
        worst = simplex[-1]
        reflected = [c + (c - w) for c, w in zip(centroid, worst)]
        fr = f(reflected)
        evals += 1
        if values[0] <= fr < values[-2]:
            simplex[-1], values[-1] = reflected, fr
            continue
        if fr < values[0] and evals < max_evals:
            expanded = [c + 2.0 * (c - w) for c, w in zip(centroid, worst)]
            fe = f(expanded)
            evals += 1
            simplex[-1], values[-1] = (expanded, fe) if fe < fr else (reflected, fr)
            continue
        if evals >= max_evals:
            break
        contracted = [c + 0.5 * (w - c) for c, w in zip(centroid, worst)]
        fc = f(contracted)
        evals += 1
        if fc < values[-1]:
            simplex[-1], values[-1] = contracted, fc
            continue
        for i in range(1, n + 1):  # shrink towards the best vertex
            if evals >= max_evals:
                break
            simplex[i] = [b + 0.5 * (p - b) for b, p in zip(simplex[0], simplex[i])]
            values[i] = f(simplex[i])
            evals += 1
    best = min(range(n + 1), key=lambda i: values[i])
    return simplex[best], values[best]


def bayesian_opt(f, x0, step, max_evals, bounds_halfwidth=2.0, seed=0, log=print):
    """Gaussian-process Bayesian optimization (RBF kernel, expected improvement).

    Search box: x0 +- bounds_halfwidth (log units). Starts with x0 plus a small Latin
    design, then maximizes EI over random candidates. Returns (x_best, f_best).
    """
    import numpy as np
    from scipy.stats import norm
    rng = np.random.default_rng(seed)
    x0 = np.asarray(x0, float)
    lo, hi = x0 - bounds_halfwidth, x0 + bounds_halfwidth
    n0 = min(max_evals, 2 * len(x0) + 1)
    X = [x0] + [lo + (hi - lo) * rng.permutation((np.arange(len(x0)) + i + rng.random(len(x0))) % n0) / n0
                for i in range(n0 - 1)]
    X = [np.clip(x, lo, hi) for x in X]
    Y = [f(list(x)) for x in X]
    while len(Y) < max_evals:
        Xa, Ya = np.array(X), np.array(Y, float)
        y = np.log1p(np.maximum(Ya, 0.0))      # compress outliers (failed runs)
        mu_y, sd_y = y.mean(), y.std() + 1e-9
        yn = (y - mu_y) / sd_y
        ell = 0.5 * (hi - lo)
        def k(A, B):
            d = (A[:, None, :] - B[None, :, :]) / ell
            return np.exp(-0.5 * (d ** 2).sum(-1))
        K = k(Xa, Xa) + 1e-4 * np.eye(len(Xa))
        L = np.linalg.cholesky(K)
        alpha = np.linalg.solve(L.T, np.linalg.solve(L, yn))
        C = lo + (hi - lo) * rng.random((4000, len(x0)))
        Ks = k(C, Xa)
        mu = Ks @ alpha
        v = np.linalg.solve(L, Ks.T)
        sd = np.sqrt(np.maximum(1.0 - (v ** 2).sum(0), 1e-12))
        best = yn.min()
        z = (best - mu) / sd
        ei = (best - mu) * norm.cdf(z) + sd * norm.pdf(z)
        x = C[int(np.argmax(ei))]
        X.append(x)
        Y.append(f(list(x)))
        log(f'  BO best J = {min(Y):.4f} after {len(Y)} evaluations')
    i = int(np.argmin(Y))
    return list(X[i]), Y[i]


class Tuner:
    def __init__(self, spec_path: Path, runner, out_dir: Path, dry_run: bool):
        self.spec_path = spec_path
        self.spec = yaml.safe_load(spec_path.read_text())
        self.theta = self.spec['theta']
        self.runner = runner
        self.out_dir = out_dir
        self.dry_run = dry_run
        self.repo = find_repo()
        self.evaluations = []
        self.cache = {}

    def to_theta(self, z):
        """log-space vector -> clipped physical parameters (zero-start entries use an offset)."""
        out = []
        for p, zi in zip(self.theta, z):
            offset = float(p.get('offset', 0.0 if p['start'] > 0 else 0.05))
            v = math.exp(zi) - offset
            out.append(min(float(p.get('max', math.inf)), max(float(p.get('min', -math.inf)), v)))
        return out

    def z0(self):
        return [math.log(p['start'] + float(p.get('offset', 0.0 if p['start'] > 0 else 0.05)))
                for p in self.theta]

    def scenario_docs(self, values):
        overrides = {}
        for p, v in zip(self.theta, values):
            overrides.setdefault(p['node'], {})[p['name']] = float(v)
        docs = []
        for sc in self.spec['scenarios']:
            base = yaml.safe_load((self.spec_path.parent / sc).resolve().read_text())
            doc = deep_merge(base, self.spec.get('base') or {})
            doc = deep_merge(doc, {'parameters': overrides})
            for seed in self.spec.get('seeds') or [doc.get('seed', 0)]:
                d = copy.deepcopy(doc)
                d['seed'] = int(seed)
                d['repeat'] = 1
                docs.append(d)
        return docs

    def evaluate(self, z):
        values = self.to_theta(z)
        key = tuple(round(v, 6) for v in values)
        if key in self.cache:
            return self.cache[key]
        k = len(self.evaluations)
        label = ', '.join(f"{p['name']}={v:.4g}" for p, v in zip(self.theta, values))
        print(f'\n[eval {k}] {label}')
        costs = []
        for i, doc in enumerate(self.scenario_docs(values)):
            variant = f'tune{k:03d}_{i}'
            f = self.out_dir / f"{doc['name']}__{variant}.yaml"
            f.write_text(yaml.safe_dump(doc, sort_keys=False, allow_unicode=True))
            if self.dry_run:
                continue
            costs.append(self.run_once(f, doc, variant))
        J = sum(costs) / len(costs) if costs else float('nan')
        row = {'eval': k, **{p['name']: v for p, v in zip(self.theta, values)}, 'J': J,
               'J_runs': ' '.join(f'{c:.4f}' for c in costs)}
        self.evaluations.append(row)
        self.write_csv()
        print(f'  J = {J:.4f}')
        self.cache[key] = J if math.isfinite(J) else 1e9
        return self.cache[key]

    def run_once(self, f: Path, doc: dict, variant: str) -> float:
        t0 = time.time()
        started = self.runner.ensure_sim(f, variant, 0)
        if started:
            self.apply_px4_params()
        else:
            self.runner.command('start', path=str(f), variant=variant)
        per_run = (doc.get('duration') or 60) + (doc.get('staging') or {}).get('timeout', 45) + 30
        result = self.runner.wait_finished(doc['name'], per_run + (180 if started else 30))
        if result == 'TIMEOUT':
            self.runner.command('stop')
        # session_info.yaml is written after the post-roll and the bag close: wait for it.
        infos = []
        deadline = time.monotonic() + 20.0
        while not infos and time.monotonic() < deadline:
            infos = [load_session_info(p)
                     for p in (self.repo / 'datasets').glob('*/session_info.yaml')
                     if p.stat().st_mtime >= t0]
            infos = [i for i in infos if i.get('variant') == variant]
            if not infos:
                self.runner.spin(1.0)
        if not infos:
            print('  no session recorded: penalised')
            return 1e3
        return session_cost(infos[-1])

    def apply_px4_params(self):
        for name, value in (self.spec.get('px4_params') or {}).items():
            subprocess.run([str(self.repo / 'ros2_ws' / 'sim_manager.sh'), 'px4', 'param', 'set',
                            name, str(value)], capture_output=True, text=True)

    def write_csv(self):
        keys = list(dict.fromkeys(k for r in self.evaluations for k in r))
        with open(self.out_dir / 'evaluations.csv', 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            w.writerows(self.evaluations)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('spec')
    ap.add_argument('--max-evals', type=int)
    ap.add_argument('--method', choices=['nm', 'bo'], default='nm',
                    help='nm: Nelder-Mead (local) | bo: Bayesian optimization (global, box x0 +- 2 log units)')
    ap.add_argument('--dry-run', action='store_true', help='write the scenarios of the first evaluation only')
    ap.add_argument('--keep-running', action='store_true')
    ap.add_argument('--gui', action='store_true')
    args = ap.parse_args()

    spec_path = Path(args.spec).resolve()
    spec = yaml.safe_load(spec_path.read_text())
    repo = find_repo()
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = repo / 'datasets' / 'tuning' / f"{spec.get('name', spec_path.stem)}_{stamp}"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / 'tuning.yaml').write_text(spec_path.read_text())

    runner = None if args.dry_run else Runner(repo, headless=not args.gui)
    tuner = Tuner(spec_path, runner, out_dir, args.dry_run)
    if args.dry_run:
        tuner.evaluate(tuner.z0())
        print(f'Plan written to {out_dir}')
        return 0
    max_evals = args.max_evals or int(spec.get('max_evals', 15))
    try:
        opt = bayesian_opt if args.method == 'bo' else nelder_mead
        z, J = opt(tuner.evaluate, tuner.z0(), float(spec.get('initial_step', 0.5)), max_evals)
        best = {p['name']: v for p, v in zip(tuner.theta, tuner.to_theta(z))}
        (out_dir / 'best.yaml').write_text(yaml.safe_dump({'theta': best, 'J': J}, sort_keys=False))
        print(f'\nBest: {best}  J = {J:.4f}  -> {out_dir}')
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        runner.shutdown(stop_sim=not args.keep_running)
    return 0


if __name__ == '__main__':
    sys.exit(main())
