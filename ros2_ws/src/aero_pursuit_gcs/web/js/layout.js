// Modular GCS layout: draggable and resizable windows (camera, scene, tab panel and
// detached tabs), with grid/edge snapping, stacking order, presets and persistence.
// "Classic" mode keeps the original fixed layout (and is used on narrow screens).

const KEY = 'gcs.layout.v1';
const GRID = 8;          // px
const SNAP = 10;         // px of attraction to edges
const MIN_W = 220, MIN_H = 140;
const TOL = 6;           // px: edges closer than this are shared (resized together)

export const PRESETS = {
  // Fractions of the container: x, y, w, h
  padrao: { video: [0, 0, 0.44, 1], scene: [0.44, 0, 0.28, 1], side: [0.72, 0, 0.28, 1] },
  perseguicao: { video: [0, 0, 0.62, 0.66], scene: [0, 0.66, 0.62, 0.34], side: [0.62, 0, 0.38, 1] },
  analise: { video: [0, 0, 0.36, 0.5], scene: [0, 0.5, 0.36, 0.5], side: [0.36, 0, 0.32, 1], popped: [0.68, 0, 0.32, 1] },
  video_grande: { video: [0, 0, 0.75, 1], scene: [0.75, 0.55, 0.25, 0.45], side: [0.75, 0, 0.25, 0.55] },
};

export class WindowManager {
  /**
   * @param {HTMLElement} root (positioned) container of the windows
   * @param {Function} onAnyResize called after moving/resizing (redraw of the views)
   */
  constructor(root, onAnyResize) {
    this.root = root;
    this.onAnyResize = onAnyResize;
    this.wins = new Map();   // id -> {el, title, opts}
    this.z = 10;
    const saved = JSON.parse(localStorage.getItem(KEY) || 'null');
    this.saved = saved && saved.wins ? saved : { mode: 'free', wins: {}, popped: [] };
    this.listeners = new Set();
    new ResizeObserver(() => this.applyAll()).observe(root);
  }

  get mode() {
    // Narrow screens: always classic (the CSS stacks the views).
    return window.matchMedia('(max-width: 900px)').matches ? 'classic' : this.saved.mode;
  }

  onChange(fn) { this.listeners.add(fn); }
  emit() { for (const fn of this.listeners) fn(); }

  save() { localStorage.setItem(KEY, JSON.stringify(this.saved)); this.emit(); }

  /** Registers a window. `handle` is the drag bar; `closable` creates the × button. */
  add(id, { el, title, handle, defaultRect, closable = false, onClose = null, onResize = null }) {
    el.classList.add('win');
    el.dataset.win = id;
    const w = { id, el, title, handle, defaultRect, closable, onClose, onResize };
    this.wins.set(id, w);
    for (const dir of ['e', 's', 'se', 'w', 'sw', 'n', 'ne', 'nw']) {
      const r = document.createElement('div');
      r.className = `rz rz-${dir}`;
      r.addEventListener('pointerdown', (e) => this.startResize(e, w, dir));
      el.append(r);
    }
    handle.classList.add('win-handle');
    handle.addEventListener('pointerdown', (e) => {
      if (e.target.closest('button,select,input,label,a,textarea')) return;
      this.startMove(e, w);
    });
    handle.addEventListener('dblclick', (e) => {
      if (e.target.closest('button,select,input,label')) return;
      this.toggleMax(w);
    });
    el.addEventListener('pointerdown', () => this.raise(w), true);
    if (closable) {
      const x = document.createElement('button');
      x.className = 'small win-close';
      x.title = 'Close window';
      x.textContent = '×';
      x.addEventListener('click', () => this.close(id));
      handle.append(x);
    }
    if (onResize) new ResizeObserver(() => onResize()).observe(el);
    this.apply(w);
    return w;
  }

  rectOf(w) {
    const s = this.saved.wins[w.id];
    return s && s.rect ? s.rect : w.defaultRect;
  }

  apply(w) {
    const el = w.el;
    if (this.mode !== 'free') {
      el.style.left = el.style.top = el.style.width = el.style.height = el.style.zIndex = '';
      return;
    }
    const s = this.saved.wins[w.id] || {};
    el.style.display = s.hidden ? 'none' : '';
    const [fx, fy, fw, fh] = s.max ? [0, 0, 1, 1] : this.rectOf(w);
    const W = this.root.clientWidth, H = this.root.clientHeight;
    el.style.left = `${Math.round(fx * W)}px`;
    el.style.top = `${Math.round(fy * H)}px`;
    el.style.width = `${Math.round(fw * W)}px`;
    el.style.height = `${Math.round(fh * H)}px`;
    el.style.zIndex = s.max ? 900 : (s.z || 10);
  }

  applyAll() {
    document.body.classList.toggle('free-layout', this.mode === 'free');
    for (const w of this.wins.values()) this.apply(w);
    this.onAnyResize?.();
  }

  setMode(mode) { this.saved.mode = mode; this.save(); this.applyAll(); }

  preset(name) {
    const p = PRESETS[name];
    if (!p) return;
    for (const w of this.wins.values()) {
      const r = p[w.id] || (w.id.startsWith('tab:') ? p.popped : null);
      const s = (this.saved.wins[w.id] ||= {});
      if (r) { s.rect = r; s.hidden = false; }
      s.max = false;
    }
    this.saved.mode = 'free';
    this.save();
    this.applyAll();
  }

  reset() {
    for (const id of [...this.wins.keys()]) if (id.startsWith('tab:')) this.close(id);
    this.saved = { mode: 'free', wins: {}, popped: [] };
    this.preset('padrao');
  }

  setHidden(id, hidden) {
    const w0 = this.wins.get(id);
    if (hidden && w0 && w0.el.style.display !== 'none') this.fillGap(w0);
    const s = (this.saved.wins[id] ||= {});
    s.hidden = hidden;
    this.save();
    const w = this.wins.get(id);
    if (w) this.apply(w);
    this.onAnyResize?.();
  }

  close(id) {
    const w = this.wins.get(id);
    if (!w) return;
    if (w.closable) {
      if (w.el.style.display !== 'none') this.fillGap(w);
      w.onClose?.();
      w.el.remove();
      this.wins.delete(id);
      delete this.saved.wins[id];
      this.saved.popped = (this.saved.popped || []).filter((x) => `tab:${x}` !== id);
      this.save();
    } else {
      this.setHidden(id, true);
    }
  }

  raise(w) {
    if (this.mode !== 'free') return;
    const s = (this.saved.wins[w.id] ||= {});
    if (s.z === this.z) return;
    s.z = ++this.z;
    w.el.style.zIndex = s.max ? 900 : s.z;
    localStorage.setItem(KEY, JSON.stringify(this.saved));
  }

  toggleMax(w) {
    if (this.mode !== 'free') return;
    const s = (this.saved.wins[w.id] ||= {});
    s.max = !s.max;
    this.save();
    this.apply(w);
    this.onAnyResize?.();
  }

  // --- arrasto e redimensionamento -------------------------------------------------------
  geometry(w) {
    return { x: w.el.offsetLeft, y: w.el.offsetTop, w: w.el.offsetWidth, h: w.el.offsetHeight };
  }

  snap(v, targets) {
    let best = Math.round(v / GRID) * GRID;
    let bestD = Math.abs(best - v);
    for (const t of targets) {
      const d = Math.abs(t - v);
      if (d < SNAP && d < bestD) { best = t; bestD = d; }
    }
    return best;
  }

  edges(except) {
    const xs = [0, this.root.clientWidth], ys = [0, this.root.clientHeight];
    for (const o of this.wins.values()) {
      if (o === except || o.el.style.display === 'none') continue;
      const g = this.geometry(o);
      xs.push(g.x, g.x + g.w);
      ys.push(g.y, g.y + g.h);
    }
    return { xs, ys };
  }

  commit(w, g) {
    const W = this.root.clientWidth, H = this.root.clientHeight;
    const s = (this.saved.wins[w.id] ||= {});
    s.rect = [g.x / W, g.y / H, g.w / W, g.h / H];
    s.max = false;
    this.save();
  }

  startMove(e, w) {
    if (this.mode !== 'free' || e.button !== 0) return;
    const s = this.saved.wins[w.id] || {};
    if (s.max) return;
    e.preventDefault();
    w.handle.setPointerCapture(e.pointerId);
    const g0 = this.geometry(w);
    const x0 = e.clientX, y0 = e.clientY;
    const { xs, ys } = this.edges(w);
    document.body.classList.add('win-dragging');
    const move = (ev) => {
      let x = g0.x + ev.clientX - x0, y = g0.y + ev.clientY - y0;
      // Snaps the left or right edge, the top or the bottom one (the closest).
      const sx = this.snap(x, xs), sx2 = this.snap(x + g0.w, xs) - g0.w;
      x = Math.abs(sx - x) <= Math.abs(sx2 - x) ? sx : sx2;
      const sy = this.snap(y, ys), sy2 = this.snap(y + g0.h, ys) - g0.h;
      y = Math.abs(sy - y) <= Math.abs(sy2 - y) ? sy : sy2;
      x = Math.max(-g0.w + 80, Math.min(this.root.clientWidth - 80, x));
      y = Math.max(0, Math.min(this.root.clientHeight - 32, y));
      w.el.style.left = `${x}px`;
      w.el.style.top = `${y}px`;
    };
    const up = () => {
      w.handle.removeEventListener('pointermove', move);
      w.handle.removeEventListener('pointerup', up);
      document.body.classList.remove('win-dragging');
      this.commit(w, this.geometry(w));
    };
    w.handle.addEventListener('pointermove', move);
    w.handle.addEventListener('pointerup', up);
  }

  // --- shared edges (tiling) ------------------------------------------------------------
  // Windows that touch (edges within TOL px, overlapping spans) are resized together: dragging
  // an edge moves the matching edge of every neighbour, so they neither overlap nor leave a
  // gap. Dragging an edge that lies on the container border shrinks/grows the window from the
  // opposite side instead (when it has neighbours there): the neighbours fill the space.
  visibleGeoms() {
    const out = new Map();
    for (const o of this.wins.values()) {
      if (o.el.style.display === 'none' || (this.saved.wins[o.id] || {}).max) continue;
      out.set(o, this.geometry(o));
    }
    return out;
  }

  /// Windows on each side of the line `pos` (axis 'x': vertical line; 'y': horizontal), grown
  /// from `seed` (which has its `seedSide` edge, 'lo' = left/top, 'hi' = right/bottom, on the line).
  edgeGroups(geoms, axis, pos, seed, seedSide) {
    const lo = axis === 'x' ? 'x' : 'y', len = axis === 'x' ? 'w' : 'h';
    const plo = axis === 'x' ? 'y' : 'x', plen = axis === 'x' ? 'h' : 'w';
    const before = new Set(), after = new Set();   // edge 'hi' on the line | edge 'lo' on the line
    (seedSide === 'hi' ? before : after).add(seed);
    const overlaps = (a, b) => Math.min(a[plo] + a[plen], b[plo] + b[plen]) - Math.max(a[plo], b[plo]) > TOL;
    let grew = true;
    while (grew) {
      grew = false;
      for (const [o, g] of geoms) {
        if (before.has(o) || after.has(o)) continue;
        const onHi = Math.abs(g[lo] + g[len] - pos) <= TOL, onLo = Math.abs(g[lo] - pos) <= TOL;
        if (onHi && [...after].some((a) => overlaps(g, geoms.get(a)))) { before.add(o); grew = true; }
        else if (onLo && [...before].some((b) => overlaps(g, geoms.get(b)))) { after.add(o); grew = true; }
      }
    }
    return { before, after };
  }

  /// Resize plan of one axis for the edge `side` ('lo' | 'hi') of w.
  edgePlan(geoms, w, axis, side) {
    const g = geoms.get(w);
    const lo = axis === 'x' ? 'x' : 'y', len = axis === 'x' ? 'w' : 'h';
    const size = axis === 'x' ? this.root.clientWidth : this.root.clientHeight;
    const min = axis === 'x' ? MIN_W : MIN_H;
    let edge = side === 'hi' ? g[lo] + g[len] : g[lo];
    let flip = 1;
    let groups = this.edgeGroups(geoms, axis, edge, w, side);
    const alone = (side === 'hi' ? groups.after : groups.before).size === 0;
    const onBorder = side === 'hi' ? Math.abs(edge - size) <= TOL : Math.abs(edge) <= TOL;
    if (alone && onBorder) {
      // Border edge: resize from the opposite edge if neighbours are attached there.
      const other = side === 'hi' ? 'lo' : 'hi';
      const oe = other === 'hi' ? g[lo] + g[len] : g[lo];
      const og = this.edgeGroups(geoms, axis, oe, w, other);
      if ((other === 'hi' ? og.after : og.before).size > 0) {
        edge = oe;
        groups = og;
        flip = -1;
      }
    }
    // The line can move while every window keeps its minimum size.
    let lower = -Infinity, upper = Infinity;
    for (const o of groups.before) lower = Math.max(lower, geoms.get(o)[lo] + min);
    for (const o of groups.after) upper = Math.min(upper, geoms.get(o)[lo] + geoms.get(o)[len] - min);
    if (!groups.before.size) lower = 0;
    if (!groups.after.size) upper = size;
    return { axis, edge, flip, ...groups, lower, upper };
  }

  applyPlan(geoms, plan, pos, out) {
    const lo = plan.axis === 'x' ? 'x' : 'y', len = plan.axis === 'x' ? 'w' : 'h';
    for (const o of plan.before) {
      const g = (out.get(o) || out.set(o, { ...geoms.get(o) }).get(o));
      g[len] = pos - g[lo];
    }
    for (const o of plan.after) {
      const g = (out.get(o) || out.set(o, { ...geoms.get(o) }).get(o));
      const end = g[lo] + g[len];
      g[lo] = pos;
      g[len] = end - pos;
    }
  }

  startResize(e, w, dir) {
    if (this.mode !== 'free' || e.button !== 0) return;
    e.preventDefault();
    e.stopPropagation();
    const target = e.currentTarget;
    target.setPointerCapture(e.pointerId);
    const geoms = this.visibleGeoms();
    if (!geoms.has(w)) geoms.set(w, this.geometry(w));
    const plans = [];
    if (dir.includes('e')) plans.push(this.edgePlan(geoms, w, 'x', 'hi'));
    if (dir.includes('w')) plans.push(this.edgePlan(geoms, w, 'x', 'lo'));
    if (dir.includes('s')) plans.push(this.edgePlan(geoms, w, 'y', 'hi'));
    if (dir.includes('n')) plans.push(this.edgePlan(geoms, w, 'y', 'lo'));
    // Snap targets: edges of the windows that do not move.
    const moving = new Set(plans.flatMap((p) => [...p.before, ...p.after]));
    const xs = [0, this.root.clientWidth], ys = [0, this.root.clientHeight];
    for (const [o, g] of geoms) {
      if (moving.has(o)) continue;
      xs.push(g.x, g.x + g.w);
      ys.push(g.y, g.y + g.h);
    }
    const x0 = e.clientX, y0 = e.clientY;
    document.body.classList.add('win-dragging');
    let last = new Map();
    const move = (ev) => {
      const out = new Map();
      for (const p of plans) {
        const d = (p.axis === 'x' ? ev.clientX - x0 : ev.clientY - y0) * p.flip;
        let pos = this.snap(p.edge + d, p.axis === 'x' ? xs : ys);
        pos = Math.max(p.lower, Math.min(p.upper, pos));
        this.applyPlan(geoms, p, pos, out);
      }
      for (const [o, g] of out) {
        Object.assign(o.el.style, { left: `${g.x}px`, top: `${Math.max(0, g.y)}px`, width: `${g.w}px`, height: `${g.h}px` });
      }
      last = out;
    };
    const up = () => {
      target.removeEventListener('pointermove', move);
      target.removeEventListener('pointerup', up);
      document.body.classList.remove('win-dragging');
      for (const o of (last.size ? last.keys() : [w])) this.commit(o, this.geometry(o));
      this.onAnyResize?.();
    };
    target.addEventListener('pointermove', move);
    target.addEventListener('pointerup', up);
  }

  /// A window leaves the layout (hidden/closed): its neighbours on one side take its area.
  fillGap(w) {
    if (this.mode !== 'free') return;
    const geoms = this.visibleGeoms();
    const g = geoms.get(w) || this.geometry(w);
    geoms.delete(w);
    // Neighbours whose edge touches the gap on a side and that, together, cover that whole side.
    const sides = [
      ['x', 'lo', g.x + g.w, (o) => Math.abs(o.x - (g.x + g.w)) <= TOL, (o) => [o.y, o.y + o.h], [g.y, g.y + g.h]],
      ['x', 'hi', g.x, (o) => Math.abs(o.x + o.w - g.x) <= TOL, (o) => [o.y, o.y + o.h], [g.y, g.y + g.h]],
      ['y', 'lo', g.y + g.h, (o) => Math.abs(o.y - (g.y + g.h)) <= TOL, (o) => [o.x, o.x + o.w], [g.x, g.x + g.w]],
      ['y', 'hi', g.y, (o) => Math.abs(o.y + o.h - g.y) <= TOL, (o) => [o.x, o.x + o.w], [g.x, g.x + g.w]],
    ];
    for (const [axis, side, , touches, span, [a, b]] of sides) {
      const ns = [...geoms].filter(([, og]) => touches(og) && span(og)[0] >= a - TOL && span(og)[1] <= b + TOL);
      const covered = ns.reduce((acc, [, og]) => acc + span(og)[1] - span(og)[0], 0);
      if (!ns.length || covered < b - a - 2 * TOL * ns.length) continue;
      for (const [o, og] of ns) {
        if (axis === 'x') {
          if (side === 'lo') { og.w += og.x - g.x; og.x = g.x; } else og.w = g.x + g.w - og.x;
        } else if (side === 'lo') { og.h += og.y - g.y; og.y = g.y; } else og.h = g.y + g.h - og.y;
        this.commit(o, og);
        this.apply(o);
      }
      return;
    }
  }
}
