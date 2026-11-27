#!/usr/bin/env python3
"""Figures of a study for the monograph (PDF + PNG), from its summary.csv and the recorded sessions.

    plot_study_figures.py maps <summary.csv> [--pool-distances] [--metrics position,lost,rmse]
        Colour maps scenario x target speed, one panel per metric and one row per standoff distance
        (mean of the repetitions of the same configuration, their standard deviation below; also one
        file per distance). --pool-distances: a single row with both distances together.
    plot_study_figures.py compare <A/summary.csv> --against <B/summary.csv> [--filter pampc,track]
        Cell by cell between two studies of the same cases (e.g. Isaac Sim and Gazebo): difference
        of the means (B - A) and the standard deviation of the repetitions in each; prints the mean
        |difference|, the mean std of each and the share of cells where the difference exceeds
        std A + std B.
    plot_study_figures.py snapshots <summary.csv> --run RUN_ID --times 26,end [--labels 'a|b']
        One run drawn up to each instant, side by side (paths so far, drones at that instant).
    plot_study_figures.py paths <summary.csv>
        The target path of each scenario in 3D (planned, from /scenario/paths), with the pursuer start.
    plot_study_figures.py examples <summary.csv> [--good RUN,RUN,RUN] [--bad RUN,RUN,RUN]
        Two figures of 3 runs each in 3D (target, pursuer, estimate): runs followed to the end and
        runs where the pursuer lost the target. Without --good/--bad they are picked by the time
        with a 3D position of the target (best / worst, one per scenario).

    --filter pampc,track keeps only the variants with those tokens (studies with more axes); --name
    adds a suffix to the file names. --out DIR (default: <study folder>/figures). The sessions are read with rosbag2_py (source ROS).
    Metrics: position (time with a 3D target position, % of the run), visible (target in the image,
    % of the run), lost (time lost, % of the run), rmse (RMS position error, m), image (RMS angle between
    the optical axis and the target, ground truth, deg), velocity (RMS, m/s),
    solve (MPC mean solve time, ms).
"""
import argparse
import csv
import statistics as st
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402

SCENARIOS = ['circle', 'helix', 'zigzag', 'figure8', 'figure8_tilted', 'lissajous3d']
SCENARIO_NAMES = {'circle': 'Círculo', 'helix': 'Hélice', 'zigzag': 'Zigue-zague', 'figure8': 'Oito',
                  'figure8_tilted': 'Oito inclinado', 'lissajous3d': 'Lissajous 3D'}
SPEEDS = [('v3', '3'), ('v6_5', '6.5'), ('v10', '10')]
DECIMAL = '.'   # decimal marker of the numbers in the figures (the monograph tables use '.')
DISTANCES = [('d7', '7 m'), ('d10', '10 m')]

# metric -> (title, unit format, value of a run, "better" hint)
def _pct(k):
    return lambda x: 100.0 * float(x[k]) / float(x['duration'])


METRICS = {
    'position': ('Tempo com posição 3D do alvo', '{:.0f}%', _pct('position_time'), 'maior é melhor'),
    'visible': ('Tempo com o alvo na imagem', '{:.0f}%', _pct('visible_time'), 'maior é melhor'),
    'lost': ('Tempo perdido', '{:.0f}%', _pct('time_lost'), 'menor é melhor'),
    'rmse': ('RMS do erro de posição', '{:.2f} m', lambda x: float(x['rmse_position']), 'menor é melhor'),
    'image': ('RMS do erro de imagem', '{:.0f}°', lambda x: np.degrees(float(x['rmse_image'])),
              'menor é melhor'),
    'velocity': ('RMS do erro de velocidade', '{:.2f} m/s', lambda x: float(x['rmse_velocity']),
                 'menor é melhor'),
    'solve': ('Tempo de solução do MPC (média)', '{:.1f} ms', lambda x: float(x['solve_mean_us']) / 1e3,
              'menor é melhor'),
}

# Reference palette (dataviz skill): one blue ramp for magnitude; categorical slots 1-3 for the 3D.
BLUES = LinearSegmentedColormap.from_list(
    'seq_blue', ['#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#184f95', '#0d366b'])
GREENS = LinearSegmentedColormap.from_list(
    'seq_green', ['#d4f0d4', '#a9dfa9', '#7ccb7c', '#4fb04f', '#2a8f2a', '#17701a', '#0b4f0e'])
YELLOWS = LinearSegmentedColormap.from_list(
    'seq_yellow', ['#fdf0c8', '#fbe29a', '#f6cf63', '#eeb72e', '#d69a0a', '#a87706', '#6e4d04'])
PANEL_MAPS = [BLUES, GREENS, YELLOWS]   # one hue per panel, in order
# Warm set (red, orange, purple): for the omniscient-detection references, so they are not mistaken
# for the vision results (blue, green, yellow) next to them in the monograph.
REDS = LinearSegmentedColormap.from_list(
    'seq_red', ['#fde0dc', '#f9b4ab', '#f28374', '#e5533f', '#c8321f', '#9b2114', '#68130b'])
ORANGES = LinearSegmentedColormap.from_list(
    'seq_orange', ['#fde8d4', '#fbcf9f', '#f8b066', '#f28f2e', '#d9700e', '#a85307', '#6f3704'])
PURPLES = LinearSegmentedColormap.from_list(
    'seq_purple', ['#ece2f6', '#d3bdec', '#b593de', '#9468cc', '#7545b0', '#56298a', '#3a165f'])
PALETTES = {'fria': [BLUES, GREENS, YELLOWS], 'quente': [REDS, ORANGES, PURPLES]}
C_TARGET, C_PURSUER, C_ESTIMATE = '#eb6834', '#2a78d6', '#1baf7a'
INK, INK_2, MUTED = '#0b0b0b', '#52514e', '#8a8984'

plt.rcParams.update({
    'font.size': 9, 'axes.titlesize': 10, 'axes.edgecolor': MUTED, 'axes.labelcolor': INK_2,
    'xtick.color': INK_2, 'ytick.color': INK_2, 'figure.dpi': 150, 'savefig.bbox': 'tight',
    'pdf.fonttype': 42,
})


def read_summary(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        sys.exit(f'{path}: empty summary')
    return rows


def value(row, metric):
    try:
        v = METRICS[metric][2](row)
    except (ValueError, ZeroDivisionError):
        return None
    return None if np.isnan(v) else v


def has(variant, key):
    return f'-{key}-' in f'-{variant}-'


def save(fig, out, name):
    out.mkdir(parents=True, exist_ok=True)
    for ext in ('pdf', 'png'):
        fig.savefig(out / f'{name}.{ext}')
    plt.close(fig)
    print(f'  {out / name}.pdf / .png')


# ----------------------------------------------------------------------------------------- maps
def cell_stats(rows, scenario, speed, distance, metric):
    vals = [value(x, metric) for x in rows if x['scenario'] == scenario and has(x['variant'], speed)
            and (distance is None or has(x['variant'], distance))]
    vals = [v for v in vals if v is not None]
    if not vals:
        return None, None, 0
    return st.mean(vals), (st.stdev(vals) if len(vals) > 1 else 0.0), len(vals)


def plot_maps(rows, metrics, distances, out, name, fig_title='', panel_maps=PANEL_MAPS):
    """One row of panels per distance (None: the distances pooled); the colour scale of a metric is
    the same in every row, so the rows compare."""
    scenarios = [s for s in SCENARIOS if any(x['scenario'] == s for x in rows)]
    stats = {}
    for metric in metrics:
        for distance in distances:
            mean = np.full((len(scenarios), len(SPEEDS)), np.nan)
            std = np.full_like(mean, np.nan)
            for i, s in enumerate(scenarios):
                for j, (sp, _) in enumerate(SPEEDS):
                    m, d, _n = cell_stats(rows, s, sp, distance, metric)
                    if m is not None:
                        mean[i, j], std[i, j] = m, d
            stats[metric, distance] = (mean, std)
    fig, axes = plt.subplots(len(distances), len(metrics),
                             figsize=(2.6 * len(metrics) + 1.2, (0.42 * len(scenarios) + 1.3) * len(distances)),
                             squeeze=False)
    for k, metric in enumerate(metrics):
        cmap = panel_maps[k % len(panel_maps)]
        title, fmt, _, hint = METRICS[metric]
        vmin, vmax = (0.0, 100.0) if fmt.endswith('%') else (
            0.0, max(np.nanmax(stats[metric, d][0]) for d in distances))
        for row, distance in enumerate(distances):
            ax = axes[row][k]
            mean, std = stats[metric, distance]
            # Vector cells (pcolormesh): an imshow image is resampled by PDF viewers (smoothed: the
            # colours bled across cells in diagonal triangles, some cells looked empty).
            im = ax.pcolormesh(np.arange(len(SPEEDS) + 1) - 0.5, np.arange(len(scenarios) + 1) - 0.5,
                               np.ma.masked_invalid(mean), cmap=cmap, vmin=vmin, vmax=vmax,
                               edgecolors='white', linewidth=2)
            ax.set_xlim(-0.5, len(SPEEDS) - 0.5)
            ax.set_ylim(len(scenarios) - 0.5, -0.5)
            for i in range(len(scenarios)):
                for j in range(len(SPEEDS)):
                    if np.isnan(mean[i, j]):
                        ax.text(j, i, '–', ha='center', va='center', color=MUTED, fontsize=9)
                        continue
                    r, g, b, _a = cmap((mean[i, j] - vmin) / max(vmax - vmin, 1e-9))
                    ink = INK if 0.2126 * r + 0.7152 * g + 0.0722 * b > 0.5 else 'white'
                    ax.text(j, i - 0.12, fmt.format(mean[i, j]).replace('.', DECIMAL), ha='center',
                            va='center', color=ink, fontsize=9, fontweight='bold')
                    ax.text(j, i + 0.22, '±' + fmt.format(std[i, j]).replace('.', DECIMAL), ha='center',
                            va='center', color=ink, fontsize=6.5)
            ax.set_xticks(range(len(SPEEDS)), [s for _, s in SPEEDS])
            ax.set_xlabel('velocidade do alvo [m/s]')
            ax.set_yticks(range(len(scenarios)), [SCENARIO_NAMES[s] for s in scenarios] if k == 0 else [])
            head = f'{title}\n({hint})' if row == 0 else ''
            if distance is not None:
                head = (head + '\n' if head else '') + f'$d$ = {dict(DISTANCES)[distance]}'
            ax.set_title(head, color=INK)
            ax.tick_params(length=0)
            for side in ax.spines.values():
                side.set_visible(False)
            cb = fig.colorbar(im, ax=ax, fraction=0.05, pad=0.03)
            cb.outline.set_visible(False)
            cb.ax.tick_params(labelsize=7, length=0)
    if fig_title:   # what the figure is (simulator, formulation, camera), above the panels
        fig.suptitle(fig_title, fontsize=12, fontweight='bold', color=INK)
    fig.tight_layout(h_pad=1.5)
    save(fig, out, name)


# -------------------------------------------------------------------------------------- compare
DIVERGING = LinearSegmentedColormap.from_list(
    'div_blue_orange', ['#184f95', '#6da7ec', '#cde2fb', '#f7f7f5', '#fbd9c8', '#f09a6c', '#b8461a'])


def compare_stats(rows_a, rows_b, metric, distance):
    """Per cell (scenario x speed) of one distance: mean B - mean A, std A, std B (NaN: no data)."""
    scenarios = [s for s in SCENARIOS if any(x['scenario'] == s for x in rows_a + rows_b)]
    shape = (len(scenarios), len(SPEEDS))
    delta, sa, sb = np.full(shape, np.nan), np.full(shape, np.nan), np.full(shape, np.nan)
    for i, s in enumerate(scenarios):
        for j, (sp, _) in enumerate(SPEEDS):
            ma, da, _ = cell_stats(rows_a, s, sp, distance, metric)
            mb, db, _ = cell_stats(rows_b, s, sp, distance, metric)
            if ma is not None and mb is not None:
                delta[i, j], sa[i, j], sb[i, j] = mb - ma, da, db
    return scenarios, delta, sa, sb


def plot_compare(rows_a, rows_b, names, metrics, out, name, fig_title=''):
    """Cell by cell: the difference of the means (B - A, diverging colours centred on 0) and, below,
    the standard deviation of the repetitions in each simulator (A | B)."""
    distances = [d for d, _ in DISTANCES]
    stats = {(m, d): compare_stats(rows_a, rows_b, m, d) for m in metrics for d in distances}
    scenarios = stats[metrics[0], distances[0]][0]
    fig, axes = plt.subplots(len(distances), len(metrics),
                             figsize=(2.6 * len(metrics) + 1.2, (0.42 * len(scenarios) + 1.3) * len(distances)),
                             squeeze=False)
    for k, metric in enumerate(metrics):
        title, fmt, _, _hint = METRICS[metric]
        lim = max(np.nanmax(np.abs(stats[metric, d][1])) for d in distances) or 1.0
        for row, distance in enumerate(distances):
            ax = axes[row][k]
            _, delta, sa, sb = stats[metric, distance]
            im = ax.pcolormesh(np.arange(len(SPEEDS) + 1) - 0.5, np.arange(len(scenarios) + 1) - 0.5,
                               np.ma.masked_invalid(delta), cmap=DIVERGING, vmin=-lim, vmax=lim,
                               edgecolors='white', linewidth=2)
            ax.set_xlim(-0.5, len(SPEEDS) - 0.5)
            ax.set_ylim(len(scenarios) - 0.5, -0.5)
            unit = fmt.format(0).lstrip('0.')                       # '%', ' m', '°', ...
            num = (lambda v: fmt.format(v).replace(unit, '')) if unit else fmt.format
            for i in range(len(scenarios)):
                for j in range(len(SPEEDS)):
                    if np.isnan(delta[i, j]):
                        ax.text(j, i, '–', ha='center', va='center', color=MUTED, fontsize=9)
                        continue
                    r, g, b, _a = DIVERGING(0.5 + delta[i, j] / (2 * lim))
                    ink = INK if 0.2126 * r + 0.7152 * g + 0.0722 * b > 0.5 else 'white'
                    sign = '+' if delta[i, j] >= 0 else '−'
                    ax.text(j, i - 0.12, sign + fmt.format(abs(delta[i, j])).replace('.', DECIMAL),
                            ha='center', va='center', color=ink, fontsize=9, fontweight='bold')
                    ax.text(j, i + 0.22, f'σ {num(sa[i, j])} | {num(sb[i, j])}'.replace('.', DECIMAL),
                            ha='center', va='center', color=ink, fontsize=6.5)
            ax.set_xticks(range(len(SPEEDS)), [s for _, s in SPEEDS])
            ax.set_xlabel('velocidade do alvo [m/s]')
            ax.set_yticks(range(len(scenarios)), [SCENARIO_NAMES[s] for s in scenarios] if k == 0 else [])
            head = f'{title}\n({names[1]} − {names[0]})' if row == 0 else ''
            head = (head + '\n' if head else '') + f'$d$ = {dict(DISTANCES)[distance]}'
            ax.set_title(head, color=INK)
            ax.tick_params(length=0)
            for side in ax.spines.values():
                side.set_visible(False)
            cb = fig.colorbar(im, ax=ax, fraction=0.05, pad=0.03)
            cb.outline.set_visible(False)
            cb.ax.tick_params(labelsize=7, length=0)
    if fig_title:   # what the figure is (simulator, formulation, camera), above the panels
        fig.suptitle(fig_title, fontsize=12, fontweight='bold', color=INK)
    fig.tight_layout(h_pad=1.5)
    save(fig, out, name)


def compare_table(rows_a, rows_b, metrics, speeds):
    """Mean over the cells (both distances, the given speeds) of |delta|, std A, std B, and the
    share of cells where |delta| > std A + std B (difference beyond the run-to-run spread)."""
    keep = [j for j, (sp, _) in enumerate(SPEEDS) if sp in speeds]
    res = {}
    for metric in metrics:
        d, a, b = [], [], []
        for distance, _ in DISTANCES:
            _, delta, sa, sb = compare_stats(rows_a, rows_b, metric, distance)
            d += list(delta[:, keep].ravel()); a += list(sa[:, keep].ravel()); b += list(sb[:, keep].ravel())
        d, a, b = np.abs(np.array(d)), np.array(a), np.array(b)
        ok = ~np.isnan(d)
        res[metric] = (d[ok].mean(), a[ok].mean(), b[ok].mean(), 100 * np.mean(d[ok] > a[ok] + b[ok]),
                       int(ok.sum()))
    return res


# ------------------------------------------------------------------------------------- sessions
def load(row):
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from view_trajectories import load_session
    return load_session(Path(row['session']))


def set_equal_3d(ax, pts):
    pts = np.vstack([p for p in pts if len(p)])
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    c, r = (lo + hi) / 2, max((hi - lo).max() / 2, 1.0)
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(max(c[2] - r, 0.0), c[2] + r)
    ax.set_box_aspect((1, 1, 1), zoom=0.82)


def style_3d(ax):
    ax.set_xlabel('x [m]', labelpad=-4)
    ax.set_ylabel('y [m]', labelpad=-4)
    ax.set_zlabel('z [m]', labelpad=-4)
    ax.tick_params(labelsize=6, pad=-2)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.set_facecolor((1, 1, 1, 0))
        axis.pane.set_edgecolor('#e4e3df')
        axis._axinfo['grid']['color'] = '#ecebe7'
    ax.view_init(elev=28, azim=-60)


def representative(rows, scenario):
    """A run of the scenario for its path: the slowest speed, first run."""
    cand = [x for x in rows if x['scenario'] == scenario]
    cand.sort(key=lambda x: (not has(x['variant'], 'v3'), x['run_id']))
    return cand[0] if cand else None


def plot_paths(rows, out):
    scenarios = [s for s in SCENARIOS if any(x['scenario'] == s for x in rows)]
    cols = 3
    nrows = (len(scenarios) + cols - 1) // cols
    fig = plt.figure(figsize=(9.5, 3.3 * nrows))
    for k, s in enumerate(scenarios):
        ax = fig.add_subplot(nrows, cols, k + 1, projection='3d')
        data = load(representative(rows, s))
        planned = data['planned']
        if not len(planned):
            planned = data['target'][1]
        xp = data['pursuer'][1]
        ax.plot(*planned.T, color=C_TARGET, linewidth=1.6)
        ax.scatter(*xp[0], color=C_PURSUER, s=24, marker='^', depthshade=False)
        set_equal_3d(ax, [planned, xp[:1]])
        style_3d(ax)
        ax.set_title(SCENARIO_NAMES[s], color=INK)
    handles = [plt.Line2D([], [], color=C_TARGET, linewidth=1.6, label='Trajetória do alvo'),
               plt.Line2D([], [], color=C_PURSUER, marker='^', linestyle='', label='Perseguidor')]
    fig.legend(handles=handles, loc='lower center', ncol=2, frameon=False, bbox_to_anchor=(0.5, -0.01))
    save(fig, out, 'trajetorias')


def run_score(x):
    try:
        return float(x['position_time']) / float(x['duration'])
    except (ValueError, ZeroDivisionError):
        return 0.0


def pick(rows, best):
    """One run per scenario, ranked by the time with a 3D position; three scenarios."""
    by_scen = {}
    for x in rows:
        if best or float(x['engaged_time'] or 0) > 0:      # a bad example still has a chase
            by_scen.setdefault(x['scenario'], []).append(x)
    picks = [max(v, key=run_score) if best else min(v, key=run_score) for v in by_scen.values()]
    picks.sort(key=run_score, reverse=best)
    return picks[:3]


def run_title(x):
    sp = next((n for k, n in SPEEDS if has(x['variant'], k)), '?')
    d = next((n for k, n in DISTANCES if has(x['variant'], k)), '?')
    return (f'{SCENARIO_NAMES.get(x["scenario"], x["scenario"])}, {sp} m/s, {d}\n'
            f'posição 3D em {100 * run_score(x):.0f}% do ensaio').replace('.', DECIMAL)


def plot_examples(rows, runs, out, name):
    fig = plt.figure(figsize=(10, 3.6))
    for k, x in enumerate(runs):
        ax = fig.add_subplot(1, len(runs), k + 1, projection='3d')
        ax.computed_zorder = False    # drawing order by zorder: the target line over the estimate
        data = load(x)
        xt, xp, xe = data['target'][1], data['pursuer'][1], data['estimate'][1]
        if len(xe):   # under the lines: the estimate lies on the target path when it is good
            ax.scatter(*xe[::4].T, color=C_ESTIMATE, s=6, alpha=0.35, linewidths=0, depthshade=False,
                       zorder=1)
        ax.plot(*xt.T, color=C_TARGET, linewidth=1.2, zorder=3)
        ax.plot(*xp.T, color=C_PURSUER, linewidth=1.4, zorder=3)
        set_equal_3d(ax, [xt, xp])
        style_3d(ax)
        ax.set_title(run_title(x), color=INK, fontsize=8.5)
    handles = [plt.Line2D([], [], color=C_TARGET, linewidth=1.4, label='Alvo'),
               plt.Line2D([], [], color=C_PURSUER, linewidth=1.4, label='Perseguidor'),
               plt.Line2D([], [], color=C_ESTIMATE, marker='o', markersize=3, linestyle='',
                          label='Posição estimada do alvo')]
    fig.legend(handles=handles, loc='lower center', ncol=3, frameon=False, bbox_to_anchor=(0.5, 0.04))
    save(fig, out, name)
    for x in runs:
        print(f'    {x["run_id"]}: {x["session"]}')


def plot_snapshots(row, times, labels, out, name, top=False):
    """One run drawn up to each time (side by side): paths so far, the drones at that instant
    (dots) and the line between them, the planned target path faint. 'end' = the whole run.
    With a list of runs (top view only): one panel per run, each at its end."""
    if isinstance(row, list):
        datas = [load(r) for r in row]
        ends = [float(min(d['target'][0][-1], d['pursuer'][0][-1])) for d in datas]
        return plot_snapshots_top(datas, ends, labels, out, name)
    data = load(row)
    (tt, xt), (tp, xp), (te, xe) = data['target'], data['pursuer'], data['estimate']
    plan = data['planned']
    t_end = float(min(tt[-1], tp[-1]))
    times = [t_end if t == 'end' else float(t) for t in times]
    if top:   # view from above (x, y): planar paths read better than in a flattened 3D box
        return plot_snapshots_top([data] * len(times), times, labels, out, name)
    fig = plt.figure(figsize=(4.6 * len(times), 4.2))
    for k, t in enumerate(times):
        ax = fig.add_subplot(1, len(times), k + 1, projection='3d')
        ax.computed_zorder = False
        if len(plan):
            ax.plot(*plan.T, color=MUTED, linewidth=0.8, linestyle='--', zorder=0)
        e = xe[te <= t]
        if len(e):
            ax.scatter(*e[::4].T, color=C_ESTIMATE, s=6, alpha=0.35, linewidths=0, depthshade=False, zorder=1)
        a, b = xt[tt <= t], xp[tp <= t]
        ax.plot(*a.T, color=C_TARGET, linewidth=1.3, zorder=3)
        ax.plot(*b.T, color=C_PURSUER, linewidth=1.5, zorder=3)
        q = np.array([np.interp(t, tt, xt[:, i]) for i in range(3)])
        r = np.array([np.interp(t, tp, xp[:, i]) for i in range(3)])
        ax.plot(*np.vstack([r, q]).T, color=INK_2, linewidth=0.8, linestyle=':', zorder=4)
        ax.scatter(*q, color=C_TARGET, s=36, edgecolors=INK, linewidths=0.6, depthshade=False, zorder=5)
        ax.scatter(*r, color=C_PURSUER, s=36, edgecolors=INK, linewidths=0.6, depthshade=False, zorder=5)
        set_equal_3d(ax, [xt, xp] + ([plan] if len(plan) else []))
        style_3d(ax)
        head = labels[k] if k < len(labels) and labels[k] else ''
        ax.set_title((f't = {t:.1f} s' + (f': {head}' if head else '')).replace('.', DECIMAL), color=INK,
                     fontsize=9)
    handles = [plt.Line2D([], [], color=C_TARGET, linewidth=1.4, label='Alvo'),
               plt.Line2D([], [], color=C_PURSUER, linewidth=1.4, label='Perseguidor'),
               plt.Line2D([], [], color=C_ESTIMATE, marker='o', markersize=3, linestyle='',
                          label='Posição estimada do alvo'),
               plt.Line2D([], [], color=MUTED, linewidth=0.8, linestyle='--', label='Trajetória prevista do alvo')]
    fig.legend(handles=handles, loc='lower center', ncol=4, frameon=False, bbox_to_anchor=(0.5, 0.0))
    save(fig, out, name)


def plot_snapshots_top(datas, times, labels, out, name):
    def bounds(ds):
        allp = np.vstack([p for d in ds for p in (d['target'][1], d['pursuer'][1], d['planned']) if len(p)])
        return allp[:, :2].min(axis=0) - 2, allp[:, :2].max(axis=0) + 2
    lo_all, hi_all = bounds(datas)
    # Shared x, y per run (equal aspect: panel height from its y span).
    spans = [bounds([d]) for d in datas]
    ratios = [max(1.0, h[1] - l[1]) for l, h in spans]
    width = 8.5
    h_in = sum(ratios) * width / (hi_all[0] - lo_all[0]) + 0.45 * len(times) + 0.6
    fig, axes = plt.subplots(len(times), 1, figsize=(width, h_in), squeeze=False,
                             gridspec_kw={'height_ratios': ratios})
    for k, t in enumerate(times):
        data = datas[k]
        lo = np.array([lo_all[0], spans[k][0][1]])
        hi = np.array([hi_all[0], spans[k][1][1]])
        (tt, xt), (tp, xp), (te, xe) = data['target'], data['pursuer'], data['estimate']
        plan = data['planned']
        ax = axes[k][0]
        if len(plan):
            ax.plot(plan[:, 0], plan[:, 1], color=MUTED, linewidth=0.8, linestyle='--', zorder=0)
        e = xe[te <= t]
        if len(e):
            ax.scatter(e[::4, 0], e[::4, 1], color=C_ESTIMATE, s=6, alpha=0.4, linewidths=0, zorder=1)
        a, b = xt[tt <= t], xp[tp <= t]
        ax.plot(a[:, 0], a[:, 1], color=C_TARGET, linewidth=1.3, zorder=3)
        ax.plot(b[:, 0], b[:, 1], color=C_PURSUER, linewidth=1.5, zorder=3)
        q = [np.interp(t, tt, xt[:, i]) for i in range(2)]
        r = [np.interp(t, tp, xp[:, i]) for i in range(2)]
        ax.plot([r[0], q[0]], [r[1], q[1]], color=INK_2, linewidth=0.8, linestyle=':', zorder=4)
        ax.scatter(*q, color=C_TARGET, s=40, edgecolors=INK, linewidths=0.6, zorder=5)
        ax.scatter(*r, color=C_PURSUER, s=40, edgecolors=INK, linewidths=0.6, zorder=5)
        ax.set_xlim(lo[0], hi[0])
        ax.set_ylim(lo[1], hi[1])
        ax.set_aspect('equal')
        ax.set_ylabel('y [m]')
        if k == len(times) - 1:
            ax.set_xlabel('x [m]')
        for side in ('top', 'right'):
            ax.spines[side].set_visible(False)
        head = labels[k] if k < len(labels) and labels[k] else ''
        ax.set_title((f't = {t:.1f} s' + (f': {head}' if head else '')).replace('.', DECIMAL), color=INK,
                     fontsize=9, loc='left')
    handles = [plt.Line2D([], [], color=C_TARGET, linewidth=1.4, label='Alvo'),
               plt.Line2D([], [], color=C_PURSUER, linewidth=1.4, label='Perseguidor'),
               plt.Line2D([], [], color=C_ESTIMATE, marker='o', markersize=3, linestyle='',
                          label='Posição estimada do alvo'),
               plt.Line2D([], [], color=MUTED, linewidth=0.8, linestyle='--', label='Trajetória prevista do alvo')]
    fig.legend(handles=handles, loc='lower center', ncol=4, frameon=False, bbox_to_anchor=(0.5, -0.02))
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    save(fig, out, name)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('figure', choices=['maps', 'paths', 'examples', 'compare', 'snapshots'])
    ap.add_argument('summary', type=Path)
    ap.add_argument('--out', type=Path)
    ap.add_argument('--metrics', default='position,lost,rmse')
    ap.add_argument('--pool-distances', action='store_true')
    ap.add_argument('--good', help='run ids, comma-separated (default: picked)')
    ap.add_argument('--bad', help='run ids, comma-separated (default: picked)')
    ap.add_argument('--name', default='', help='suffix of the file names')
    ap.add_argument('--run', help='snapshots: run_id of the run (several, comma-separated, with --top: one panel each, at its end)')
    ap.add_argument('--times', default='end', help="snapshots: instants [s], comma-separated ('end' = whole run)")
    ap.add_argument('--top', action='store_true', help='snapshots: view from above (x, y)')
    ap.add_argument('--labels', default='', help='snapshots: a caption per instant, separated by |')
    ap.add_argument('--title', default='', help='maps/compare: title above the panels')
    ap.add_argument('--against', type=Path, help='compare: summary.csv of the other study (B; the '
                    'first one is A, the maps show B - A)')
    ap.add_argument('--palette', choices=list(PALETTES), default='fria',
                    help="maps: colour ramps of the panels (fria: blue/green/yellow; quente: "
                         "red/orange/purple, used for the omniscient references)")
    ap.add_argument('--filter', default='',
                    help='variant tokens that must all be present, comma-separated (e.g. pampc,track)')
    a = ap.parse_args()
    rows = read_summary(a.summary)
    tokens = [t for t in a.filter.split(',') if t]
    rows = [x for x in rows if all(has(x['variant'], t) for t in tokens)]
    if not rows:
        sys.exit(f'no run with {a.filter}')
    out = a.out or a.summary.parent / 'figures'
    if a.figure == 'maps':
        metrics = a.metrics.split(',')
        for m in metrics:
            if m not in METRICS:
                sys.exit(f'unknown metric {m} (one of {", ".join(METRICS)})')
        if a.pool_distances:
            plot_maps(rows, metrics, [None], out, f'mapas_juntas{a.name}', a.title, PALETTES[a.palette])
        else:
            plot_maps(rows, metrics, [d for d, _ in DISTANCES], out, f'mapas{a.name}', a.title,
                      PALETTES[a.palette])
            for d, _ in DISTANCES:
                plot_maps(rows, metrics, [d], out, f'mapas_{d}{a.name}', '', PALETTES[a.palette])
    elif a.figure == 'compare':
        if not a.against:
            sys.exit('compare needs --against <summary.csv>')
        rows_b = [x for x in read_summary(a.against) if all(has(x['variant'], t) for t in tokens)]
        names = [r[0].get('simulator', 'A').capitalize().replace('Isaac', 'Isaac Sim')
                 for r in (rows, rows_b)]
        metrics = a.metrics.split(',')
        plot_compare(rows, rows_b, names, metrics, out, f'comparacao{a.name}', a.title)
        for label, speeds in (('all speeds', [sp for sp, _ in SPEEDS]), ('3 and 6.5 m/s', ['v3', 'v6_5'])):
            print(f'  {label}: metric |delta| stdA stdB %cells(|delta|>stdA+stdB) n')
            for m, v in compare_table(rows, rows_b, metrics, speeds).items():
                print(f'    {m:9s} {v[0]:7.2f} {v[1]:7.2f} {v[2]:7.2f} {v[3]:5.0f}% {v[4]}')
    elif a.figure == 'snapshots':
        ids = a.run.split(',')
        found = [next((x for x in rows if x['run_id'] == i), None) for i in ids]
        if None in found:
            sys.exit(f'no run {ids[found.index(None)]}')
        row = found[0] if len(found) == 1 else found
        plot_snapshots(row, a.times.split(','), a.labels.split('|'), out, f'instantes{a.name}', a.top)
    elif a.figure == 'paths':
        plot_paths(rows, out)
    else:
        ids = {x['run_id']: x for x in rows}
        for kind, arg, best in (('bons', a.good, True), ('ruins', a.bad, False)):
            runs = [ids[r] for r in arg.split(',')] if arg else pick(rows, best)
            print(f'  exemplos {kind}:')
            plot_examples(rows, runs, out, f'exemplos_{kind}{a.name}')


if __name__ == '__main__':
    main()
