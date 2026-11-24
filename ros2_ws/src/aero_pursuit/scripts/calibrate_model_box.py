#!/usr/bin/env python3
"""Calibrates the visible box of a model (target_acquisition.model_boxes) from the frames of the
sonda `probe_acquisition.py --frames N`.

In each saved crop: the current projected box is drawn in red; the drone is segmented
as dark, low-saturation pixels against the bright sky, near the box. The center difference and
the visible size, converted to meters by the true distance (in the file name), give the
correction. Only frames with the target against the sky are reliable (the script ignores the very small ones).

    calibrate_model_box.py <dir_quadros> --current "0 0 -0.02 0.78 0.86 0.28" [--hfov 1.204 --width 1920]
"""
import argparse
import glob
import math

import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('frames')
    ap.add_argument('--current', required=True, help='caixa atual: "cx cy cz sx sy sz"')
    ap.add_argument('--hfov', type=float, default=1.204, help='horizontal FOV of the RGB camera [rad]')
    ap.add_argument('--width', type=int, default=1920)
    ap.add_argument('--min-px', type=float, default=20.0, help='minimum width of the segmented drone [px]')
    args = ap.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fx = args.width / 2 / math.tan(args.hfov / 2)
    cur = [float(x) for x in args.current.split()]
    rows = []
    for f in sorted(glob.glob(f'{args.frames}/frame_*.png')):
        dist = float(f.rsplit('_', 1)[-1][:-5])
        im = plt.imread(f)[:, :, :3]
        r, g, b = im[..., 0], im[..., 1], im[..., 2]
        red = (r > 0.8) & (g < 0.4) & (b < 0.3)
        if not red.any():
            continue
        ys, xs = np.nonzero(red)
        gx0, gx1, gy0, gy1 = xs.min(), xs.max(), ys.min(), ys.max()
        dark = (im.mean(2) < 0.45) & (np.abs(r - g) < 0.12) & (np.abs(g - b) < 0.12) & ~red
        mask = np.zeros_like(dark)
        mask[max(0, gy0 - 40):gy1 + 40, max(0, gx0 - 40):gx1 + 40] = True
        ys, xs = np.nonzero(dark & mask)
        if len(xs) < 5:
            continue
        ox0, ox1 = np.percentile(xs, 1), np.percentile(xs, 99)
        oy0, oy1 = np.percentile(ys, 1), np.percentile(ys, 99)
        if ox1 - ox0 < args.min_px:
            continue
        rows.append({'dist': dist, 'w': (ox1 - ox0) * dist / fx, 'h': (oy1 - oy0) * dist / fx,
                     'dz': -((oy0 + oy1) - (gy0 + gy1)) / 2 * dist / fx,
                     'dy': ((ox0 + ox1) - (gx0 + gx1)) / 2 * dist / fx})
    if not rows:
        print('no usable frame (target too small or no sky behind it)')
        return 1
    print(f'{len(rows)} usable frames')
    for x in rows:
        print(f"  {x['dist']:5.1f} m: visible {x['w']:.3f} × {x['h']:.3f} m, center Δz {x['dz']:+.3f} m, Δlateral {x['dy']:+.3f} m")
    med = {k: float(np.median([x[k] for x in rows])) for k in ('w', 'h', 'dz')}
    cz = cur[2] + med['dz']
    print(f"\nmedian: width {med['w']:.3f} m, height {med['h']:.3f} m, Δz {med['dz']:+.3f} m")
    print('suggested model_boxes (seen width applied to both horizontal axes; review it if the')
    print('model is not symmetric):')
    print(f'  "{cur[0]:g} {cur[1]:g} {cz:.2f} {med["w"]:.2f} {med["w"]:.2f} {med["h"]:.2f}"')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
