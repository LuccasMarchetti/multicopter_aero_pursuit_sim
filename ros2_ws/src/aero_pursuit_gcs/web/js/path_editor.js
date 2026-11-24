// Path editor: a full-screen window with the map seen from above (x east, y north) and, below
// it, the height (z) and speed along time. Edits one trajectory of a scenario:
//   points   click to add, click on a segment to insert, drag to move, drag in the lower chart
//            to change the height; each point may have its own speed and stop time
//   timed    points with a time each (t, x, y, z), typically imported from a CSV/Excel table
//   shapes   circle, polygon, figure 8, lissajous, helix, random: a generator with parameters
//            (the center is dragged on the map); "Convert to points" makes it editable
// The drawn curve comes from scenario_tool (`scenario.sample`): the same C++ that flies it.
//
//   openPathEditor(ctl, { title, trajectory, context, onApply })
//   pickPoint(ctl, { title, value, context, onApply })      one [x, y, z] on the map

import { h, button, toast } from './ui.js';

export const SHAPES = {
  circle: { label: 'Circle', fields: ['center', 'radius', 'clockwise', 'phase_deg', 'roll_deg', 'pitch_deg'],
    defaults: { radius: 10 } },
  polygon: { label: 'Polygon', fields: ['center', 'radius', 'sides', 'heading_deg', 'phase_deg', 'clockwise', 'roll_deg', 'pitch_deg'],
    defaults: { radius: 10, sides: 4 } },
  figure8: { label: 'Figure 8', fields: ['center', 'size', 'heading_deg', 'roll_deg', 'pitch_deg'],
    defaults: { size: [12, 6, 0] } },
  lissajous: { label: 'Lissajous', fields: ['center', 'size', 'frequency', 'phase3_deg', 'heading_deg', 'roll_deg', 'pitch_deg'],
    defaults: { size: [12, 8, 1], frequency: [1, 2, 1] } },
  helix: { label: 'Helix', fields: ['center', 'radius', 'height', 'turns', 'clockwise', 'phase_deg', 'heading_deg', 'roll_deg', 'pitch_deg'],
    defaults: { radius: 8, height: 6, turns: 2 } },
  random: { label: 'Random loop', fields: ['center', 'size', 'count', 'seed', 'heading_deg', 'roll_deg', 'pitch_deg'],
    defaults: { size: [15, 15, 2], count: 6, seed: 1 } },
};

// [label, kind, help]
const FIELD = {
  center: ['center x, y, z [m]', 'vec', 'Middle of the shape (drag the ✛ handle on the map).'],
  radius: ['radius [m]', 'num', ''],
  sides: ['sides', 'int', ''],
  heading_deg: ['rotation [°]', 'num', 'Turns the whole shape around its center (0 = as drawn).'],
  roll_deg: ['tilt roll [°]', 'num', 'Tilts the shape about its own x axis (its length): 0 = flat.'],
  pitch_deg: ['tilt pitch [°]', 'num', 'Tilts the shape about its own y axis: one end up, the other down.'],
  phase_deg: ['start angle [°]', 'num', 'Where on the shape the drone starts (0 = east of the center).'],
  clockwise: ['clockwise', 'bool', ''],
  size: ['size x, y, z [m]', 'vec', 'Half width, half height and vertical oscillation.'],
  frequency: ['frequency x, y, z', 'vec', 'Oscillations per lap on each axis (1, 2 = figure 8 lying down).'],
  phase3_deg: ['phase x, y, z [°]', 'vec', ''],
  height: ['climb [m]', 'num', 'Climbs this much over the turns, then comes back down the same way.'],
  turns: ['turns', 'num', ''],
  count: ['random points', 'int', 'More points = more turns.'],
  seed: ['random seed', 'int', 'Same number = same route; change it for another route.'],
};

const COLOR = { path: '#ffb74d', point: '#ffb74d', first: '#66bb6a', sel: '#ffffff', target: '#e57373',
  pursuer: '#4fc3f7', context: 'rgba(176,190,197,0.45)', patrol: 'rgba(79,195,247,0.6)', speed: '#81c784',
  grid: 'rgba(255,255,255,0.05)', grid5: 'rgba(255,255,255,0.11)', axis: 'rgba(255,255,255,0.3)' };

const r2 = (v) => Math.round(v * 100) / 100;
const clone = (o) => JSON.parse(JSON.stringify(o ?? null));
const numOr = (v, d) => (v === '' || v === null || v === undefined || Number.isNaN(Number(v)) ? d : Number(v));

// -- trajectory <-> editable model --------------------------------------------------------
/// Editable model of a trajectory: { type, speed, interpolation, closed, relative, pts: [{p, t, speed, hold}], shape fields }
function toModel(traj) {
  const t = clone(traj) || {};
  let type = t.type || 'waypoints';
  if (type === 'line') {           // a line is two waypoints (closed = there and back)
    type = 'waypoints';
    t.interpolation = 'linear';
  }
  const m = { ...t, type, speed: numOr(t.speed, 3) };
  delete m.points;
  m.pts = (t.points || []).map((q) => {
    if (Array.isArray(q)) return q.length === 4 ? { t: q[0], p: q.slice(1) } : { p: q.slice(0, 3) };
    return { p: (q.p || [0, 0, 5]).slice(0, 3), speed: q.speed, hold: q.hold, t: q.t };
  });
  return m;
}

function fromModel(m) {
  const out = {};
  for (const [k, v] of Object.entries(m)) if (k !== 'pts' && v !== undefined && v !== '') out[k] = v;
  if (m.type === 'waypoints') {
    out.points = m.pts.map((q) => {
      const p = q.p.map(r2);
      return q.speed || q.hold ? { p, ...(q.speed ? { speed: q.speed } : {}), ...(q.hold ? { hold: q.hold } : {}) } : p;
    });
    for (const k of Object.keys(SHAPES).flatMap((s) => SHAPES[s].fields)) if (k !== 'center') delete out[k];
    delete out.center;
  } else if (m.type === 'timed') {
    out.points = m.pts.map((q) => [r2(q.t ?? 0), ...q.p.map(r2)]);
    delete out.closed;
    delete out.speed;
  }
  if (m.type !== 'waypoints' && m.type !== 'timed') delete out.interpolation;
  if (!out.relative) delete out.relative;
  if (!out.closed) delete out.closed;
  return out;
}

/// Field names of a shape -> the inputs; `onChange` after each edit.
function shapeInputs(m, type, onChange) {
  return SHAPES[type].fields.map((k) => {
    const [label, kind, help] = FIELD[k];
    let el;
    if (kind === 'bool') {
      el = h('input', { type: 'checkbox' });
      el.checked = !!m[k];
      el.addEventListener('change', () => { m[k] = el.checked; onChange(); });
      return h('label', { class: 'pe-check', title: help }, el, label);
    }
    if (kind === 'vec') {
      const v = Array.isArray(m[k]) ? m[k] : (k === 'center' ? [0, 0, 5] : [0, 0, 0]);
      m[k] = v;
      const parts = [0, 1, 2].map((i) => {
        const p = h('input', { type: 'number', step: 'any', value: v[i] ?? 0 });
        p.addEventListener('change', () => { v[i] = numOr(p.value, 0); onChange(); });
        return p;
      });
      el = h('div', { class: 'pe-vec' }, ...parts);
    } else {
      el = h('input', { type: 'number', step: kind === 'int' ? 1 : 'any', value: m[k] ?? '' });
      el.addEventListener('change', () => {
        if (el.value === '') delete m[k]; else m[k] = Number(el.value);
        onChange();
      });
    }
    return h('label', { class: 'pe-field' }, h('span', {}, label), el, help ? h('small', {}, help) : null);
  });
}

// -- the window -------------------------------------------------------------------------
class MapWindow {
  constructor(ctl, { title, context, mode }) {
    this.ctl = ctl;
    this.context = context || {};
    this.mode = mode;            // 'path' | 'point'
    this.view = { cx: 15, cy: 0, scale: 12 };
    this.snap = false;
    this.map = h('canvas', { class: 'pe-map', tabindex: 0 });
    this.prof = h('canvas', { class: 'pe-prof' });
    this.side = h('div', { class: 'pe-side' });
    this.status = h('div', { class: 'pe-status' });
    this.applyBtn = button('Apply', () => this.apply(), 'primary');
    const zoom = (f) => { this.view.scale = Math.min(200, Math.max(1, this.view.scale * f)); this.draw(); };
    this.snapBox = h('input', { type: 'checkbox' });
    this.snapBox.addEventListener('change', () => { this.snap = this.snapBox.checked; });
    this.el = h('div', { class: 'pe-overlay' },
      h('div', { class: 'pe-top' }, h('b', {}, title),
        h('span', { class: 'spacer' }),
        h('label', { class: 'pe-check', title: 'Round new and moved points to 0.5 m' }, this.snapBox, 'snap 0.5 m'),
        button('−', () => zoom(1 / 1.3), 'small', 'Zoom out'), button('+', () => zoom(1.3), 'small', 'Zoom in'),
        button('Fit', () => this.fit(), 'small', 'Show everything'),
        button('Cancel', () => this.close(), 'small'), this.applyBtn),
      h('div', { class: 'pe-body' },
        h('div', { class: 'pe-main' }, h('div', { class: 'pe-mapwrap' }, this.map, this.status),
          mode === 'path' ? h('div', { class: 'pe-profwrap' }, this.prof) : null),
        this.side));
    this.onKey = (e) => this.key(e);
    this.ro = new ResizeObserver(() => this.draw());
  }

  show() {
    document.body.append(this.el);
    document.addEventListener('keydown', this.onKey);
    this.ro.observe(this.map);
    if (this.mode === 'path') this.ro.observe(this.prof);
    this.bindMap();
    this.fit();
  }

  close() {
    document.removeEventListener('keydown', this.onKey);
    this.ro.disconnect();
    this.el.remove();
  }

  key(e) {
    if (e.key === 'Escape') this.close();
  }

  // -- coordinates ----------------------------------------------------------------------
  size(c) {
    const r = c.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    if (c.width !== Math.round(r.width * dpr) || c.height !== Math.round(r.height * dpr)) {
      c.width = Math.round(r.width * dpr);
      c.height = Math.round(r.height * dpr);
    }
    const g = c.getContext('2d');
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { g, w: r.width, h: r.height };
  }

  toScreen(x, y) {
    const r = this.map.getBoundingClientRect();
    return [r.width / 2 + (x - this.view.cx) * this.view.scale, r.height / 2 - (y - this.view.cy) * this.view.scale];
  }

  toWorld(sx, sy, snap = true) {
    const r = this.map.getBoundingClientRect();
    let x = this.view.cx + (sx - r.width / 2) / this.view.scale;
    let y = this.view.cy - (sy - r.height / 2) / this.view.scale;
    if (snap && this.snap) { x = Math.round(x * 2) / 2; y = Math.round(y * 2) / 2; }
    return [r2(x), r2(y)];
  }

  /// Everything worth showing: [[x, y], ...].
  extent() {
    const pts = [];
    const c = this.context;
    if (c.targetStart) pts.push(c.targetStart);
    if (c.pursuerStart) pts.push(c.pursuerStart);
    return pts.concat(this.ownExtent());
  }

  ownExtent() { return []; }

  fit() {
    const pts = this.extent();
    if (!pts.length) pts.push([0, 0], [30, 0]);
    const xs = pts.map((p) => p[0]);
    const ys = pts.map((p) => p[1]);
    const [x0, x1, y0, y1] = [Math.min(...xs), Math.max(...xs), Math.min(...ys), Math.max(...ys)];
    const r = this.map.getBoundingClientRect();
    this.view.cx = (x0 + x1) / 2;
    this.view.cy = (y0 + y1) / 2;
    const span = Math.max(x1 - x0, (y1 - y0) * (r.width / Math.max(1, r.height)), 10);
    this.view.scale = Math.min(60, Math.max(1, (r.width - 80) / span));
    this.draw();
  }

  // -- drawing --------------------------------------------------------------------------
  draw() {
    if (this._raf) return;
    this._raf = requestAnimationFrame(() => {
      this._raf = null;
      this.drawMap();
      if (this.mode === 'path') this.drawProfile();
    });
  }

  drawGrid(g, w, hgt) {
    const s = this.view.scale;
    const step = s > 25 ? 1 : s > 8 ? 5 : s > 3 ? 10 : 50;
    const [wx0, wy1] = this.toWorld(0, 0, false);
    const [wx1, wy0] = this.toWorld(w, hgt, false);
    g.lineWidth = 1;
    g.font = '10px system-ui';
    g.fillStyle = 'rgba(255,255,255,0.35)';
    for (let x = Math.floor(wx0 / step) * step; x <= wx1; x += step) {
      const [sx] = this.toScreen(x, 0);
      g.strokeStyle = x === 0 ? COLOR.axis : (x % (step * 5) === 0 ? COLOR.grid5 : COLOR.grid);
      g.beginPath(); g.moveTo(sx, 0); g.lineTo(sx, hgt); g.stroke();
      if (x % (step * 2) === 0) g.fillText(`${x}`, sx + 2, hgt - 4);
    }
    for (let y = Math.floor(wy0 / step) * step; y <= wy1; y += step) {
      const [, sy] = this.toScreen(0, y);
      g.strokeStyle = y === 0 ? COLOR.axis : (y % (step * 5) === 0 ? COLOR.grid5 : COLOR.grid);
      g.beginPath(); g.moveTo(0, sy); g.lineTo(w, sy); g.stroke();
      if (y % (step * 2) === 0) g.fillText(`${y}`, 3, sy - 2);
    }
    g.fillStyle = 'rgba(255,255,255,0.5)';
    g.fillText('x east [m] →', w - 80, hgt - 16);
    g.fillText('↑ y north [m]', 6, 14);
  }

  line(g, pts, color, width = 2, dash = null) {
    if (!pts || pts.length < 2) return;
    g.strokeStyle = color;
    g.lineWidth = width;
    g.setLineDash(dash || []);
    g.beginPath();
    pts.forEach((p, i) => { const [x, y] = this.toScreen(p[0], p[1]); if (i) g.lineTo(x, y); else g.moveTo(x, y); });
    g.stroke();
    g.setLineDash([]);
  }

  marker(g, p, color, label) {
    const [x, y] = this.toScreen(p[0], p[1]);
    g.fillStyle = color;
    g.beginPath(); g.moveTo(x, y - 8); g.lineTo(x + 7, y + 6); g.lineTo(x - 7, y + 6); g.closePath(); g.fill();
    g.font = 'bold 11px system-ui';
    g.fillText(label, x + 9, y + 4);
  }

  drawContext(g) {
    const c = this.context;
    for (const path of c.paths || []) this.line(g, path, COLOR.context, 1.5, [5, 4]);
    for (const path of c.patrol || []) this.line(g, path, COLOR.patrol, 1.5, [3, 4]);
    if (c.pursuerStart) this.marker(g, c.pursuerStart, COLOR.pursuer, 'pursuer start');
    if (c.targetStart) this.marker(g, c.targetStart, COLOR.target, 'target start');
  }

  drawMap() {
    const { g, w, h: hgt } = this.size(this.map);
    g.clearRect(0, 0, w, hgt);
    this.drawGrid(g, w, hgt);
    this.drawContext(g);
    this.drawOwn(g);
  }

  drawOwn() {}
  drawProfile() {}
  bindMap() {}
}

// -- path editing ---------------------------------------------------------------------------
class PathWindow extends MapWindow {
  constructor(ctl, { title, trajectory, context, onApply, startWith }) {
    super(ctl, { title, context, mode: 'path' });
    this.m = toModel(trajectory);
    this.onApply = onApply;
    this.sel = this.m.pts.length ? 0 : -1;
    this.history = [];
    this.sample = null;
    this.sampleSeq = 0;
    this.requestSample = (() => {
      let t = null;
      return () => { clearTimeout(t); t = setTimeout(() => this.fetchSample(), 120); };
    })();
    this.renderSide();
    this.requestSample();
    if (startWith === 'import') setTimeout(() => this.importDialog(), 50);
  }

  // -- model edits ------------------------------------------------------------------------
  isPoints() { return this.m.type === 'waypoints' || this.m.type === 'timed'; }

  remember() {
    this.history.push(clone(this.m));
    if (this.history.length > 100) this.history.shift();
  }

  undo() {
    const prev = this.history.pop();
    if (!prev) return;
    this.m = prev;
    this.sel = Math.min(this.sel, this.m.pts.length - 1);
    this.changed(true);
  }

  changed(side = false) {
    if (side) this.renderSide(); else this.renderSelected();
    this.requestSample();
    this.draw();
  }

  async fetchSample() {
    const seq = ++this.sampleSeq;
    const traj = fromModel(this.m);
    const enough = this.m.type === 'timed' ? this.m.pts.length >= 2 : (this.m.type !== 'waypoints' || this.m.pts.length >= 1);
    if (!enough) { this.sample = null; this.setStatus(''); this.draw(); return; }
    try {
      const r = await this.ctl.req('scenario.sample', { trajectory: traj, dt: 0.1 }, 10000);
      if (seq !== this.sampleSeq) return;
      if (r.ok) {
        this.sample = r;
        if (!this.fitted) { this.fitted = true; this.fit(); }
        const vmax = Math.max(0, ...r.samples.map((s) => s[4]));
        this.setStatus(`length ${r.length.toFixed(1)} m · one pass ${r.duration.toFixed(1)} s · top speed ${vmax.toFixed(1)} m/s`);
      } else {
        this.sample = null;
        this.setStatus(r.error, true);
      }
    } catch (e) { this.setStatus(e.message, true); }
    this.draw();
  }

  setStatus(text, bad = false) {
    this.status.textContent = text;
    this.status.classList.toggle('bad', bad);
  }

  ownExtent() {
    const pts = this.m.pts.map((q) => q.p);
    if (this.sample) for (const s of this.sample.samples) pts.push([s[1], s[2]]);
    return pts;
  }

  apply() {
    const traj = fromModel(this.m);
    if (this.m.type === 'waypoints' && !this.m.pts.length) { toast('Add at least one point (click on the map)', 'error'); return; }
    if (this.m.type === 'timed' && this.m.pts.length < 2) { toast('A timed path needs at least two points', 'error'); return; }
    this.onApply(traj);
    this.close();
  }

  key(e) {
    if (['INPUT', 'TEXTAREA', 'SELECT'].includes(document.activeElement?.tagName)) {
      if (e.key === 'Escape') document.activeElement.blur();
      return;
    }
    if (e.key === 'Escape') { if (!this.history.length || confirm('Discard the changes to this path?')) this.close(); }
    else if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'z') { e.preventDefault(); this.undo(); }
    else if ((e.key === 'Delete' || e.key === 'Backspace') && this.sel >= 0 && this.isPoints()) { e.preventDefault(); this.removeSelected(); }
  }

  removeSelected() {
    if (this.sel < 0) return;
    this.remember();
    this.m.pts.splice(this.sel, 1);
    this.sel = Math.min(this.sel, this.m.pts.length - 1);
    this.changed(true);
  }

  /// Time of a new point at `p` after point `i` (timed): at the "new point speed".
  timeAfter(i, p) {
    const prev = this.m.pts[i];
    if (!prev) return 0;
    const d = Math.hypot(p[0] - prev.p[0], p[1] - prev.p[1], p[2] - prev.p[2]);
    return r2((prev.t ?? 0) + d / Math.max(0.1, this.newSpeed || 3));
  }

  addPoint(xy, at = this.m.pts.length) {
    this.remember();
    const ref = this.m.pts[Math.min(at, this.m.pts.length - 1)] || this.m.pts[at - 1];
    const z = ref ? ref.p[2] : (this.context.defaultZ ?? 6);
    const q = { p: [xy[0], xy[1], z] };
    if (this.m.type === 'timed') {
      q.t = this.timeAfter(at - 1, q.p);
      if (at < this.m.pts.length) {   // inserted: push the later points by the same delay
        const next = this.m.pts[at];
        const shift = Math.max(0, r2(q.t + Math.hypot(next.p[0] - q.p[0], next.p[1] - q.p[1]) / Math.max(0.1, this.newSpeed || 3) - (next.t ?? 0)));
        for (let k = at; k < this.m.pts.length; k++) this.m.pts[k].t = r2((this.m.pts[k].t ?? 0) + shift);
      }
    }
    this.m.pts.splice(at, 0, q);
    this.sel = at;
    this.changed(true);
  }

  // -- side panel ---------------------------------------------------------------------------
  renderSide() {
    const m = this.m;
    this.side.innerHTML = '';
    const kind = h('select', {},
      h('option', { value: 'waypoints' }, 'Points: click on the map'),
      h('option', { value: 'timed' }, 'Timed points (t, x, y, z: CSV / Excel)'),
      ...Object.entries(SHAPES).map(([k, s]) => h('option', { value: k }, `Shape: ${s.label}`)));
    kind.value = m.type;
    kind.addEventListener('change', () => this.switchType(kind.value));
    this.side.append(h('label', { class: 'pe-field' }, h('span', {}, 'kind of path'), kind));

    if (m.type === 'waypoints') this.sideWaypoints();
    else if (m.type === 'timed') this.sideTimed();
    else this.sideShape();

    const rel = h('input', { type: 'checkbox' });
    rel.checked = !!m.relative;
    rel.addEventListener('change', () => { m.relative = rel.checked; this.changed(); });
    this.side.append(h('details', { class: 'pe-adv' }, h('summary', {}, 'Advanced'),
      h('label', { class: 'pe-check', title: '' }, rel, 'relative to where the drone is'),
      h('small', { class: 'muted' }, 'Relative: the coordinates are offsets from the drone position when this leg starts (the map shows them around 0, 0).')));
    this.side.append(this.helpBox());
  }

  speedField(label, help) {
    const m = this.m;
    const sp = h('input', { type: 'number', step: 'any', min: 0.1, value: m.speed ?? 3 });
    sp.addEventListener('change', () => { this.remember(); m.speed = Math.max(0.1, numOr(sp.value, 3)); this.changed(); });
    return h('label', { class: 'pe-field' }, h('span', {}, label), sp, help ? h('small', {}, help) : null);
  }

  sideWaypoints() {
    const m = this.m;
    const interp = h('select', {}, h('option', { value: 'linear' }, 'straight lines'), h('option', { value: 'spline' }, 'smooth curve'));
    interp.value = m.interpolation || 'linear';
    interp.addEventListener('change', () => { this.remember(); m.interpolation = interp.value; this.changed(); });
    const closed = h('input', { type: 'checkbox' });
    closed.checked = !!m.closed;
    closed.addEventListener('change', () => { this.remember(); m.closed = closed.checked; this.changed(); });
    this.side.append(
      this.speedField('speed [m/s]', 'Constant along the path; a point can set its own speed to the next one.'),
      h('label', { class: 'pe-field' }, h('span', {}, 'between points'), interp),
      h('label', { class: 'pe-check' }, closed, 'closed loop (back to the first point)'),
      h('div', { class: 'row wrap' },
        button('Reverse', () => { this.remember(); m.pts.reverse(); this.changed(true); }, 'small', 'Fly the other way'),
        button('Clear', () => { if (confirm('Remove every point?')) { this.remember(); m.pts = []; this.sel = -1; this.changed(true); } }, 'small danger'),
        button('Import CSV / Excel', () => this.importDialog(), 'small')),
      this.selectedBox = h('div', { class: 'pe-sel' }));
    this.renderSelected();
  }

  sideTimed() {
    const m = this.m;
    const interp = h('select', {}, h('option', { value: 'linear' }, 'straight lines'), h('option', { value: 'spline' }, 'smooth curve'));
    interp.value = m.interpolation || 'linear';
    interp.addEventListener('change', () => { this.remember(); m.interpolation = interp.value; this.changed(); });
    const ns = h('input', { type: 'number', step: 'any', value: this.newSpeed || 3 });
    ns.addEventListener('change', () => { this.newSpeed = Math.max(0.1, numOr(ns.value, 3)); });
    const last = m.pts.length ? m.pts[m.pts.length - 1].t ?? 0 : 0;
    this.side.append(
      h('p', { class: 'muted' }, `${m.pts.length} points over ${r2(last)} s. The drone is at each point at its time: the speed changes from segment to segment.`),
      h('div', { class: 'row wrap' },
        button('Import CSV / Excel', () => this.importDialog(), 'small primary'),
        button('To constant speed', () => {
          this.remember();
          m.type = 'waypoints';
          for (const q of m.pts) delete q.t;
          m.speed = m.speed || 3;
          this.changed(true);
        }, 'small', 'Keep the points, drop the times (one speed along the path)')),
      h('label', { class: 'pe-field' }, h('span', {}, 'between points'), interp),
      h('label', { class: 'pe-field' }, h('span', {}, 'speed for new points [m/s]'), ns,
        h('small', {}, 'A point added with the mouse gets the time to reach it at this speed.')),
      this.selectedBox = h('div', { class: 'pe-sel' }));
    this.renderSelected();
  }

  sideShape() {
    const m = this.m;
    this.side.append(this.speedField('speed [m/s]', 'Constant along the whole shape.'),
      ...shapeInputs(m, m.type, () => this.changed()),
      h('div', { class: 'row wrap' }, button('Convert to points', () => this.shapeToPoints(), 'small',
        'Replace the shape by about 16 editable points along it (smooth curve)')));
  }

  renderSelected() {
    const box = this.selectedBox;
    if (!box) return;
    box.innerHTML = '';
    const q = this.m.pts[this.sel];
    if (!q) {
      box.append(h('p', { class: 'muted' }, this.m.pts.length ? 'Click a point to edit it.' : 'Click on the map to place the first point.'));
      return;
    }
    const field = (label, get, set, help) => {
      const el = h('input', { type: 'number', step: 'any', value: get() ?? '' });
      el.addEventListener('change', () => { this.remember(); set(el.value === '' ? undefined : Number(el.value)); this.requestSample(); this.draw(); });
      return h('label', { class: 'pe-field' }, h('span', {}, label), el, help ? h('small', {}, help) : null);
    };
    const timed = this.m.type === 'timed';
    box.append(h('div', { class: 'pe-selhead' }, h('b', {}, `Point ${this.sel + 1} of ${this.m.pts.length}`),
      button('◀', () => { this.sel = Math.max(0, this.sel - 1); this.renderSelected(); this.draw(); }, 'small'),
      button('▶', () => { this.sel = Math.min(this.m.pts.length - 1, this.sel + 1); this.renderSelected(); this.draw(); }, 'small'),
      button('Remove', () => this.removeSelected(), 'small danger')),
    h('div', { class: 'pe-grid3' },
      field('x [m]', () => q.p[0], (v) => { q.p[0] = v ?? 0; }),
      field('y [m]', () => q.p[1], (v) => { q.p[1] = v ?? 0; }),
      field('z [m]', () => q.p[2], (v) => { q.p[2] = Math.max(0.3, v ?? 0); })),
    timed
      ? field('time [s]', () => q.t, (v) => { q.t = v ?? 0; }, 'Must be later than the previous point.')
      : h('div', { class: 'pe-grid3' },
        field('speed → next', () => q.speed, (v) => { if (v) q.speed = v; else delete q.speed; }, 'empty = path speed'),
        field('stop here [s]', () => q.hold, (v) => { if (v) q.hold = v; else delete q.hold; })));
  }

  helpBox() {
    return h('details', { class: 'pe-adv', open: true }, h('summary', {}, 'Mouse'),
      h('ul', { class: 'pe-help' },
        h('li', {}, 'click on the map: new point at the end'),
        h('li', {}, 'click on a line: insert a point there'),
        h('li', {}, 'drag a point: move it; drag the empty map: pan; wheel: zoom'),
        h('li', {}, 'lower chart: drag a point up or down to change its height'),
        h('li', {}, 'Delete: remove the selected point · Ctrl+Z: undo · Esc: cancel')));
  }

  switchType(type) {
    const m = this.m;
    this.remember();
    if (type === m.type) return;
    if (type === 'waypoints' || type === 'timed') {
      if (!this.isPoints() && this.sample) {
        this.shapeToPoints(type);
        return;
      }
      if (type === 'timed') {
        let t = 0;
        m.pts.forEach((q, i) => {
          if (i) t += Math.hypot(...q.p.map((v, k) => v - m.pts[i - 1].p[k])) / Math.max(0.1, q.speed || m.speed || 3);
          q.t = r2(t);
          delete q.speed; delete q.hold;
        });
      } else {
        for (const q of m.pts) delete q.t;
      }
      m.type = type;
    } else {
      const c = this.isPoints() && m.pts.length
        ? [r2(m.pts.reduce((a, q) => a + q.p[0], 0) / m.pts.length), r2(m.pts.reduce((a, q) => a + q.p[1], 0) / m.pts.length),
          m.pts[0].p[2]]
        : (m.center || [this.view.cx, this.view.cy, this.context.defaultZ ?? 6].map(r2));
      m.type = type;
      m.center = c;
      for (const [k, v] of Object.entries(SHAPES[type].defaults)) if (m[k] === undefined) m[k] = clone(v);
    }
    this.sel = m.pts.length ? 0 : -1;
    this.changed(true);
    this.fit();
  }

  shapeToPoints(type = 'waypoints') {
    const s = this.sample;
    if (!s || s.samples.length < 2) { toast('No shape to convert yet', 'error'); return; }
    const m = this.m;
    const n = Math.min(24, Math.max(6, Math.round(s.length / 6)));
    const total = s.samples.length - 1;
    const closed = m.type !== 'helix';
    const count = closed ? n : n + 1;
    const pts = [];
    for (let k = 0; k < count; k++) {
      const smp = s.samples[Math.round((k * total) / n)];
      pts.push({ p: [r2(smp[1]), r2(smp[2]), r2(smp[3])], ...(type === 'timed' ? { t: r2(smp[0]) } : {}) });
    }
    for (const k of Object.keys(m)) if (!['speed', 'relative'].includes(k)) delete m[k];
    Object.assign(m, { type, interpolation: 'spline', closed: type === 'waypoints' && closed, pts });
    this.sel = 0;
    this.changed(true);
  }

  // -- import -------------------------------------------------------------------------------
  importDialog() {
    const file = h('input', { type: 'file', accept: '.csv,.tsv,.txt,.xlsx,.xlsm' });
    const paste = h('textarea', { class: 'pe-paste', placeholder: 'or paste here (copied cells from Excel, or CSV text)\n\nt,x,y,z\n0,10,0,6\n2,14,3,6\n...' });
    const out = h('div', {});
    const box = h('div', { class: 'pe-dialog' },
      h('h4', {}, 'Import a path from a table'),
      h('p', { class: 'muted' }, 'One row per point. Columns x, y, z [m] (world ENU: x east, y north, z up) and, optionally, the time t [s]. With a time column the drone is at each point at its time; without it, it flies them at constant speed.'),
      file, paste, h('div', { class: 'row' }, button('Read', () => read(), 'small primary'), button('Close', () => box.remove(), 'small')), out);
    const read = async () => {
      let name = 'paste.txt';
      let bytes;
      if (file.files[0]) {
        name = file.files[0].name;
        bytes = new Uint8Array(await file.files[0].arrayBuffer());
      } else if (paste.value.trim()) {
        bytes = new TextEncoder().encode(paste.value);
      } else { toast('Choose a file or paste the table', 'error'); return; }
      let b64 = '';
      for (let i = 0; i < bytes.length; i += 0x8000) b64 += String.fromCharCode(...bytes.subarray(i, i + 0x8000));
      try {
        const table = await this.ctl.req('scenario.table', { name, data: btoa(b64) }, 20000);
        this.mapColumns(table, out, () => box.remove());
      } catch (e) { toast(e.message, 'error', 6000); }
    };
    file.addEventListener('change', read);
    this.side.prepend(box);
  }

  mapColumns(table, out, done) {
    const cols = table.columns;
    const guess = (names) => cols.findIndex((c) => names.includes(String(c).trim().toLowerCase()));
    const sel = (names, extra) => {
      const s = h('select', {}, ...(extra || []).map(([v, t]) => h('option', { value: v }, t)),
        ...cols.map((c, i) => h('option', { value: String(i) }, `${c}`)));
      const g = guess(names);
      s.value = g >= 0 ? String(g) : (extra ? extra[0][0] : '0');
      return s;
    };
    const cT = sel(['t', 'time', 'tempo', 'timestamp', 'time_s', 't_s', 'sec', 's'], [['none', '— none (constant speed) —']]);
    const cX = sel(['x', 'east', 'e', 'x_m', 'pos_x']);
    const cY = sel(['y', 'north', 'n', 'y_m', 'pos_y']);
    const cZ = sel(['z', 'up', 'alt', 'altitude', 'h', 'height', 'z_m', 'pos_z'], [['fixed', '— fixed height —']]);
    if (cX.value === cY.value) { cX.value = String(Math.min(1, cols.length - 1)); cY.value = String(Math.min(2, cols.length - 1)); }
    const unit = h('select', {}, h('option', { value: '1' }, 's'), h('option', { value: '0.001' }, 'ms'), h('option', { value: '1e-9' }, 'ns'));
    const fixedZ = h('input', { type: 'number', step: 'any', value: this.context.defaultZ ?? 6 });
    const off = [0, 0, 0].map(() => h('input', { type: 'number', step: 'any', value: 0 }));
    const every = h('input', { type: 'number', min: 1, step: 1, value: Math.max(1, Math.ceil(table.rows.length / 1500)) });
    const mode = h('select', {}, h('option', { value: 'replace' }, 'replace the points'), h('option', { value: 'append' }, 'add after the last point'));
    out.innerHTML = '';
    out.append(h('p', {}, `${table.rows.length} rows, columns: ${cols.join(', ')}`),
      h('div', { class: 'pe-grid2' },
        h('label', { class: 'pe-field' }, h('span', {}, 'time column'), cT),
        h('label', { class: 'pe-field' }, h('span', {}, 'time unit'), unit),
        h('label', { class: 'pe-field' }, h('span', {}, 'x column'), cX),
        h('label', { class: 'pe-field' }, h('span', {}, 'y column'), cY),
        h('label', { class: 'pe-field' }, h('span', {}, 'z column'), cZ),
        h('label', { class: 'pe-field' }, h('span', {}, 'fixed height [m]'), fixedZ),
        h('label', { class: 'pe-field' }, h('span', {}, 'use one row in'), every),
        h('label', { class: 'pe-field' }, h('span', {}, 'import'), mode)),
      h('label', { class: 'pe-field' }, h('span', {}, 'shift every point by x, y, z [m]'), h('div', { class: 'pe-vec' }, ...off)),
      h('small', { class: 'muted' }, 'The first time becomes 0. "One row in N" thins out dense logs.'),
      h('div', { class: 'row' }, button('Import', () => {
        const ix = Number(cX.value);
        const iy = Number(cY.value);
        const step = Math.max(1, Math.round(numOr(every.value, 1)));
        const scale = Number(unit.value);
        const o = off.map((e) => numOr(e.value, 0));
        const pts = [];
        let t0 = null;
        table.rows.forEach((row, k) => {
          if (k % step && k !== table.rows.length - 1) return;
          const x = Number(row[ix]); const y = Number(row[iy]);
          const z = cZ.value === 'fixed' ? numOr(fixedZ.value, 6) : Number(row[Number(cZ.value)]);
          if (![x, y, z].every(Number.isFinite)) return;
          const q = { p: [r2(x + o[0]), r2(y + o[1]), r2(Math.max(0.3, z + o[2]))] };
          if (cT.value !== 'none') {
            const t = Number(row[Number(cT.value)]) * scale;
            if (!Number.isFinite(t)) return;
            if (t0 === null) t0 = t;
            q.t = r2(t - t0);
            if (pts.length && q.t <= pts[pts.length - 1].t) return;   // repeated / out-of-order time
          }
          pts.push(q);
        });
        if (pts.length < 2) { toast('Fewer than two valid rows', 'error'); return; }
        this.remember();
        const timed = cT.value !== 'none';
        if (mode.value === 'append' && this.isPoints() && this.m.pts.length) {
          const lastT = this.m.pts[this.m.pts.length - 1].t ?? 0;
          if (timed && this.m.type === 'timed') for (const q of pts) q.t = r2(q.t + lastT + 1);
          if (!timed && this.m.type === 'timed') { toast('This path is timed: the table needs a time column', 'error'); return; }
          if (timed && this.m.type !== 'timed') for (const q of pts) delete q.t;
          this.m.pts.push(...pts);
        } else {
          for (const k of Object.keys(this.m)) if (!['speed', 'relative'].includes(k)) delete this.m[k];
          Object.assign(this.m, { type: timed ? 'timed' : 'waypoints', interpolation: 'linear', pts });
          if (!timed) this.m.speed = this.m.speed || 3;
        }
        this.sel = 0;
        done();
        this.changed(true);
        this.fit();
        toast(`${pts.length} points imported`, 'good');
      }, 'small primary')));
  }

  // -- drawing ------------------------------------------------------------------------------
  drawOwn(g) {
    const m = this.m;
    const curve = this.sample ? this.sample.samples.map((s) => [s[1], s[2]]) : null;
    if (this.isPoints()) {
      // Straight polyline while the server sample is on its way (and the control polygon of a spline).
      const poly = m.pts.map((q) => q.p);
      if (m.closed && poly.length > 2) poly.push(poly[0]);
      this.line(g, poly, m.interpolation === 'spline' ? 'rgba(255,183,77,0.3)' : COLOR.path, m.interpolation === 'spline' ? 1 : 2.5);
    }
    if (curve) {
      this.line(g, curve, COLOR.path, 2.5);
      this.arrows(g, curve);
    }
    if (this.isPoints()) {
      g.font = '10px system-ui';
      m.pts.forEach((q, i) => {
        const [x, y] = this.toScreen(q.p[0], q.p[1]);
        g.fillStyle = i === 0 ? COLOR.first : COLOR.point;
        g.beginPath(); g.arc(x, y, i === this.sel ? 7 : 5, 0, 2 * Math.PI); g.fill();
        if (i === this.sel) { g.strokeStyle = COLOR.sel; g.lineWidth = 2; g.stroke(); }
        if (m.pts.length < 80 || i === this.sel) {
          g.fillStyle = '#fff';
          g.fillText(`${i + 1}${q.hold ? ` ⏸${q.hold}s` : ''}`, x + 7, y - 6);
        }
      });
    } else if (m.center) {
      const [x, y] = this.toScreen(m.center[0], m.center[1]);
      g.strokeStyle = COLOR.sel; g.lineWidth = 2;
      g.beginPath(); g.moveTo(x - 9, y); g.lineTo(x + 9, y); g.moveTo(x, y - 9); g.lineTo(x, y + 9); g.stroke();
      g.beginPath(); g.arc(x, y, 6, 0, 2 * Math.PI); g.stroke();
    }
  }

  arrows(g, curve) {
    let acc = 0;
    const every = Math.max(4, 90 / this.view.scale);
    for (let i = 1; i < curve.length; i++) {
      const d = Math.hypot(curve[i][0] - curve[i - 1][0], curve[i][1] - curve[i - 1][1]);
      acc += d;
      if (acc < every || d < 1e-6) continue;
      acc = 0;
      const [x, y] = this.toScreen(curve[i][0], curve[i][1]);
      const a = Math.atan2(-(curve[i][1] - curve[i - 1][1]), curve[i][0] - curve[i - 1][0]);
      g.fillStyle = COLOR.path;
      g.beginPath();
      g.moveTo(x + 7 * Math.cos(a), y + 7 * Math.sin(a));
      g.lineTo(x + 5 * Math.cos(a + 2.5), y + 5 * Math.sin(a + 2.5));
      g.lineTo(x + 5 * Math.cos(a - 2.5), y + 5 * Math.sin(a - 2.5));
      g.fill();
    }
  }

  /// Height and speed over time; control points at their arrival times.
  drawProfile() {
    const { g, w, h: hgt } = this.size(this.prof);
    g.clearRect(0, 0, w, hgt);
    const pad = { l: 36, r: 44, t: 18, b: 18 };
    this.profBox = { ...pad, w, h: hgt };
    const s = this.sample;
    if (!s || !s.samples.length) {
      g.fillStyle = 'rgba(255,255,255,0.4)';
      g.fillText('height and speed along the path appear here', pad.l, hgt / 2);
      return;
    }
    const T = Math.max(1e-3, s.duration);
    const zs = s.samples.map((q) => q[3]).concat(this.m.pts.map((q) => q.p[2]));
    const zmax = Math.max(2, ...zs) * 1.15;
    const vmax = Math.max(1, ...s.samples.map((q) => q[4])) * 1.15;
    this.profScale = { T, zmax };
    const X = (t) => pad.l + (t / T) * (w - pad.l - pad.r);
    const Yz = (z) => hgt - pad.b - (z / zmax) * (hgt - pad.t - pad.b);
    const Yv = (v) => hgt - pad.b - (v / vmax) * (hgt - pad.t - pad.b);
    g.font = '10px system-ui';
    g.strokeStyle = COLOR.grid5; g.lineWidth = 1;
    for (let k = 0; k <= 4; k++) {
      const z = (zmax * k) / 4;
      g.beginPath(); g.moveTo(pad.l, Yz(z)); g.lineTo(w - pad.r, Yz(z)); g.stroke();
      g.fillStyle = COLOR.path; g.fillText(z.toFixed(0), 4, Yz(z) + 3);
      g.fillStyle = COLOR.speed; g.fillText(((vmax * k) / 4).toFixed(1), w - pad.r + 4, Yz(z) + 3);
    }
    g.fillStyle = 'rgba(255,255,255,0.5)';
    g.fillText(`t [s] → ${T.toFixed(1)}`, w - pad.r - 60, hgt - 4);
    g.fillStyle = COLOR.path; g.fillText('height z [m]', pad.l, 11);
    g.fillStyle = COLOR.speed; g.fillText('speed [m/s]', w - pad.r - 60, 11);
    const plot = (Y, idx, color) => {
      g.strokeStyle = color; g.lineWidth = 2; g.beginPath();
      s.samples.forEach((q, i) => { if (i) g.lineTo(X(q[0]), Y(q[idx])); else g.moveTo(X(q[0]), Y(q[idx])); });
      g.stroke();
    };
    plot(Yv, 4, COLOR.speed);
    plot(Yz, 3, COLOR.path);
    this.profPoints = [];
    if (this.isPoints()) {
      const kt = s.knot_times || [];
      this.m.pts.forEach((q, i) => {
        const t = this.m.type === 'timed' ? q.t ?? 0 : kt[i];
        if (t === undefined) return;
        const x = X(t); const y = Yz(q.p[2]);
        this.profPoints.push([x, y, i]);
        g.fillStyle = i === this.sel ? COLOR.sel : (i === 0 ? COLOR.first : COLOR.point);
        g.beginPath(); g.arc(x, y, i === this.sel ? 6 : 4, 0, 2 * Math.PI); g.fill();
      });
    }
  }

  // -- mouse -----------------------------------------------------------------------------------
  hitPoint(sx, sy) {
    if (!this.isPoints()) return -1;
    let best = -1; let bd = 11;
    this.m.pts.forEach((q, i) => {
      const [x, y] = this.toScreen(q.p[0], q.p[1]);
      const d = Math.hypot(x - sx, y - sy);
      if (d < bd) { bd = d; best = i; }
    });
    return best;
  }

  /// Index after which a click at (sx, sy) on the control polyline inserts a point, or -1.
  hitSegment(sx, sy) {
    const pts = this.m.pts;
    const n = pts.length;
    const segs = this.m.closed && n > 2 ? n : n - 1;
    for (let i = 0; i < segs; i++) {
      const a = this.toScreen(...pts[i].p.slice(0, 2));
      const b = this.toScreen(...pts[(i + 1) % n].p.slice(0, 2));
      const L = Math.hypot(b[0] - a[0], b[1] - a[1]);
      if (L < 1) continue;
      const u = ((sx - a[0]) * (b[0] - a[0]) + (sy - a[1]) * (b[1] - a[1])) / (L * L);
      if (u <= 0.05 || u >= 0.95) continue;
      const d = Math.hypot(sx - (a[0] + u * (b[0] - a[0])), sy - (a[1] + u * (b[1] - a[1])));
      if (d < 6) return i;
    }
    return -1;
  }

  bindMap() {
    const c = this.map;
    let drag = null;
    const local = (e) => { const r = c.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; };
    c.addEventListener('pointerdown', (e) => {
      const [sx, sy] = local(e);
      c.setPointerCapture(e.pointerId);
      const i = this.hitPoint(sx, sy);
      if (i >= 0) {
        this.sel = i;
        this.renderSelected();
        drag = { kind: 'point', i, sx, sy, moved: false };
      } else if (!this.isPoints() && this.m.center) {
        const [x, y] = this.toScreen(this.m.center[0], this.m.center[1]);
        drag = Math.hypot(x - sx, y - sy) < 12 ? { kind: 'center', sx, sy, moved: false }
          : { kind: 'pan', sx, sy, cx: this.view.cx, cy: this.view.cy, moved: false };
      } else {
        drag = { kind: 'pan', sx, sy, cx: this.view.cx, cy: this.view.cy, moved: false };
      }
      this.draw();
    });
    c.addEventListener('pointermove', (e) => {
      if (!drag) return;
      const [sx, sy] = local(e);
      if (!drag.moved && Math.hypot(sx - drag.sx, sy - drag.sy) < 4) return;
      if (!drag.moved && drag.kind !== 'pan') this.remember();
      drag.moved = true;
      if (drag.kind === 'pan') {
        this.view.cx = drag.cx - (sx - drag.sx) / this.view.scale;
        this.view.cy = drag.cy + (sy - drag.sy) / this.view.scale;
      } else if (drag.kind === 'point') {
        const [x, y] = this.toWorld(sx, sy);
        this.m.pts[drag.i].p[0] = x;
        this.m.pts[drag.i].p[1] = y;
        this.renderSelected();
      } else {
        const [x, y] = this.toWorld(sx, sy);
        this.m.center[0] = x;
        this.m.center[1] = y;
      }
      this.draw();
    });
    c.addEventListener('pointerup', (e) => {
      if (!drag) return;
      const d = drag;
      drag = null;
      if (d.moved) {
        if (d.kind !== 'pan') this.changed(d.kind === 'center');
        return;
      }
      if (d.kind === 'pan' && this.isPoints()) {
        const [sx, sy] = local(e);
        const seg = this.hitSegment(sx, sy);
        this.addPoint(this.toWorld(sx, sy), seg >= 0 ? seg + 1 : this.m.pts.length);
      }
    });
    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      const [sx, sy] = local(e);
      const before = this.toWorld(sx, sy, false);
      this.view.scale = Math.min(200, Math.max(1, this.view.scale * (e.deltaY < 0 ? 1.15 : 1 / 1.15)));
      const after = this.toWorld(sx, sy, false);
      this.view.cx += before[0] - after[0];
      this.view.cy += before[1] - after[1];
      this.draw();
    }, { passive: false });

    // Height chart: drag a point vertically.
    const p = this.prof;
    let zdrag = null;
    p.addEventListener('pointerdown', (e) => {
      const r = p.getBoundingClientRect();
      const sx = e.clientX - r.left; const sy = e.clientY - r.top;
      const hit = (this.profPoints || []).find(([x, y]) => Math.hypot(x - sx, y - sy) < 10);
      if (!hit) return;
      p.setPointerCapture(e.pointerId);
      this.remember();
      this.sel = hit[2];
      zdrag = { i: hit[2] };
      this.renderSelected();
      this.draw();
    });
    p.addEventListener('pointermove', (e) => {
      if (!zdrag || !this.profBox) return;
      const r = p.getBoundingClientRect();
      const { t, b, h: hgt } = this.profBox;
      const z = ((hgt - b - (e.clientY - r.top)) / (hgt - t - b)) * this.profScale.zmax;
      this.m.pts[zdrag.i].p[2] = Math.max(0.3, Math.round(z * 10) / 10);
      this.renderSelected();
      this.draw();
    });
    p.addEventListener('pointerup', () => { if (zdrag) { zdrag = null; this.changed(); } });
  }
}

// -- single point ---------------------------------------------------------------------------
class PointWindow extends MapWindow {
  constructor(ctl, { title, value, context, onApply }) {
    super(ctl, { title, context, mode: 'point' });
    this.p = Array.isArray(value) ? [...value] : null;
    this.onApply = onApply;
    this.inputs = [0, 1, 2].map((i) => {
      const el = h('input', { type: 'number', step: 'any' });
      el.addEventListener('change', () => {
        this.p = this.p || [0, 0, context.defaultZ ?? 6];
        this.p[i] = numOr(el.value, 0);
        this.draw();
      });
      return el;
    });
    this.side.append(h('p', { class: 'muted' }, 'Click on the map to place the point; set its height below.'),
      h('div', { class: 'pe-grid3' }, ...['x [m]', 'y [m]', 'z [m]'].map((l, i) => h('label', { class: 'pe-field' }, h('span', {}, l), this.inputs[i]))));
    this.sync();
  }

  sync() { this.inputs.forEach((el, i) => { el.value = this.p ? this.p[i] : ''; }); }
  ownExtent() { return this.p ? [this.p] : []; }

  apply() {
    if (!this.p) { toast('Click on the map first', 'error'); return; }
    this.onApply(this.p.map(r2));
    this.close();
  }

  drawOwn(g) {
    if (!this.p) return;
    const [x, y] = this.toScreen(this.p[0], this.p[1]);
    g.fillStyle = COLOR.point;
    g.beginPath(); g.arc(x, y, 7, 0, 2 * Math.PI); g.fill();
    g.fillStyle = '#fff';
    g.fillText(`${this.p.map((v) => v.toFixed(1)).join(', ')}`, x + 9, y - 6);
  }

  bindMap() {
    const c = this.map;
    let down = null;
    c.addEventListener('pointerdown', (e) => { down = { x: e.clientX, y: e.clientY, cx: this.view.cx, cy: this.view.cy }; c.setPointerCapture(e.pointerId); });
    c.addEventListener('pointermove', (e) => {
      if (!down) return;
      this.view.cx = down.cx - (e.clientX - down.x) / this.view.scale;
      this.view.cy = down.cy + (e.clientY - down.y) / this.view.scale;
      this.draw();
    });
    c.addEventListener('pointerup', (e) => {
      if (down && Math.hypot(e.clientX - down.x, e.clientY - down.y) < 4) {
        const r = c.getBoundingClientRect();
        const [x, y] = this.toWorld(e.clientX - r.left, e.clientY - r.top);
        this.p = [x, y, this.p ? this.p[2] : (this.context.defaultZ ?? 6)];
        this.sync();
        this.draw();
      }
      down = null;
    });
    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      this.view.scale = Math.min(200, Math.max(1, this.view.scale * (e.deltaY < 0 ? 1.15 : 1 / 1.15)));
      this.draw();
    }, { passive: false });
  }
}

export function openPathEditor(ctl, opts) {
  const w = new PathWindow(ctl, opts);
  w.show();
  return w;
}

export function pickPoint(ctl, opts) {
  const w = new PointWindow(ctl, opts);
  w.show();
  return w;
}
