// Time-series plots (own canvas, no dependencies).
// Sources: telemetry fields (status/metrics/detection) or any topic.

import { h, button, select, section, textInput, toast, PALETTE } from './ui.js';

const TEL_FIELDS = {
  metrics: ['position_error', 'velocity_error', 'range_error', 'los_error', 'camera_off_axis',
    'image_azimuth', 'image_elevation', 'rmse_image', 'acceleration', 'deadband_factor', 'thrust_ratio', 'tilt', 'solve_time_us', 'rmse_position',
    'rmse_velocity', 'rmse_los', 'max_position_error', 'effort_l2', 'effort_l1', 'engaged_time'],
  status: ['range', 'range_error', 'command_speed', 'target_age', 'bearing_age', 'mode'],
  detection: ['score', 'depth', 'azimuth', 'elevation', 'processing_time_ms', 'status'],
};

export function getPath(obj, path) {
  let v = obj;
  for (const k of path.split('.')) {
    if (v === null || v === undefined) return undefined;
    v = v[/^\d+$/.test(k) ? Number(k) : k];
  }
  return typeof v === 'boolean' ? Number(v) : v;
}

export class TimePlot {
  constructor(container) {
    this.container = container;
    this.canvas = h('canvas', { class: 'plot' });
    container.append(this.canvas);
    this.series = [];         // {key, label, color, data: [[t, v]]}
    this.window = 30;
    this.paused = false;
    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    const loop = () => { requestAnimationFrame(loop); if (container.offsetParent !== null) this.draw(); };
    requestAnimationFrame(loop);
  }

  resize() {
    const r = this.container.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = Math.max(10, r.width * dpr);
    this.canvas.height = Math.max(10, r.height * dpr);
  }

  add(key, label) {
    if (this.series.some((s) => s.key === key)) return;
    this.series.push({ key, label, color: PALETTE[this.series.length % PALETTE.length], data: [] });
  }

  remove(key) { this.series = this.series.filter((s) => s.key !== key); }

  push(key, t, v) {
    if (this.paused) return;
    const s = this.series.find((x) => x.key === key);
    if (!s || v === null || v === undefined || !Number.isFinite(Number(v))) return;
    s.data.push([t, Number(v)]);
    const cut = t - 600;   // keeps up to 10 min
    while (s.data.length && s.data[0][0] < cut) s.data.shift();
  }

  clear() { for (const s of this.series) s.data = []; }

  draw() {
    const c = this.canvas, ctx = c.getContext('2d');
    const dpr = window.devicePixelRatio || 1;
    const W = c.width / dpr, H = c.height / dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = '#111820'; ctx.fillRect(0, 0, W, H);
    const pad = { l: 56, r: 12, t: 26, b: 22 };
    const pw = W - pad.l - pad.r, ph = H - pad.t - pad.b;
    if (!this.series.length) {
      ctx.fillStyle = '#7c8a99'; ctx.font = '13px sans-serif';
      ctx.fillText(('Add series in the Plots tab'), pad.l, H / 2);
      return;
    }
    const tNow = this.paused ? Math.max(0, ...this.series.map((s) => (s.data.length ? s.data[s.data.length - 1][0] : 0)))
      : performance.now() / 1000;
    const t0 = tNow - this.window;
    let lo = Infinity, hi = -Infinity;
    for (const s of this.series) for (const [t, v] of s.data) if (t >= t0) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
    if (!Number.isFinite(lo)) { lo = 0; hi = 1; }
    if (hi - lo < 1e-9) { hi += 0.5; lo -= 0.5; }
    const m = (hi - lo) * 0.08; lo -= m; hi += m;
    const X = (t) => pad.l + ((t - t0) / this.window) * pw;
    const Y = (v) => pad.t + (1 - (v - lo) / (hi - lo)) * ph;
    ctx.strokeStyle = '#223344'; ctx.lineWidth = 1; ctx.fillStyle = '#7c8a99'; ctx.font = '11px sans-serif';
    for (let i = 0; i <= 4; i++) {
      const v = lo + ((hi - lo) * i) / 4, y = Y(v);
      ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(W - pad.r, y); ctx.stroke();
      ctx.fillText(Math.abs(v) >= 1000 || (Math.abs(v) < 0.01 && v !== 0) ? v.toExponential(1) : v.toFixed(2), 4, y + 4);
    }
    for (let i = 0; i <= 5; i++) {
      const t = t0 + (this.window * i) / 5;
      ctx.fillText(`-${(tNow - t).toFixed(0)}s`, X(t) - 10, H - 6);
    }
    let lx = pad.l;
    for (const s of this.series) {
      ctx.strokeStyle = s.color; ctx.lineWidth = 1.6; ctx.beginPath();
      let started = false;
      for (const [t, v] of s.data) {
        if (t < t0) continue;
        const x = X(t), y = Y(v);
        started ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
        started = true;
      }
      ctx.stroke();
      const last = s.data.length ? s.data[s.data.length - 1][1] : null;
      const text = `${s.label}${last !== null ? ' = ' + (Math.abs(last) < 1e4 ? last.toFixed(3) : last.toExponential(2)) : ''}`;
      ctx.fillStyle = s.color; ctx.font = '12px sans-serif';
      ctx.fillText(text, lx, 16);
      lx += ctx.measureText(text).width + 16;
    }
  }
}

/// Plots tab: chooses series; the plot itself lives in the main panel.
export function plotsPanel(ctl, state, plot, showPlot) {
  const list = h('div', { class: 'series-list' });
  const topicSubs = new Map();     // key -> {sid, topic, field}

  const refresh = () => {
    list.innerHTML = '';
    for (const s of plot.series) {
      list.append(h('div', { class: 'series' }, h('span', { class: 'dot', style: { background: s.color } }), s.label,
        button('×', () => {
          plot.remove(s.key);
          const ts = topicSubs.get(s.key);
          if (ts) { ctl.req('unsub', { sid: ts.sid }).catch(() => {}); topicSubs.delete(s.key); }
          refresh();
        }, 'small')));
    }
  };

  ctl.on('tel', (m) => {
    const t = performance.now() / 1000;
    for (const s of plot.series) {
      if (!s.key.startsWith('tel:')) continue;
      const [, group, field] = s.key.split(':');
      if (m[group]) plot.push(s.key, t, getPath(m[group], field));
      if (group === 'rtf' && m.rtf !== undefined) plot.push(s.key, t, m.rtf);
    }
  });
  ctl.on('msg', (m) => {
    const t = performance.now() / 1000;
    for (const [key, ts] of topicSubs) if (ts.sid === m.sid) plot.push(key, t, getPath(m.data, ts.field));
  });
  ctl.on('open', () => {
    // Re-subscribes the topic series after reconnecting.
    for (const ts of topicSubs.values()) ctl.req('sub', { sid: ts.sid, topic: ts.topic, rate: 30 }).catch(() => {});
  });

  const groupSel = select({ label: 'Group', options: Object.keys(TEL_FIELDS).concat(['rtf']), value: 'metrics',
    onChange: (g) => fieldSel.setOptions(TEL_FIELDS[g] || ['rtf']) });
  const fieldSel = select({ label: 'Field', options: TEL_FIELDS.metrics });
  const topicIn = textInput({ label: 'Topic', placeholder: '/target_3d_state_filtered' });
  const fieldIn = textInput({ label: 'Field', placeholder: 'pose.pose.position.x' });
  const winSel = select({ label: 'Window', options: [['10', '10 s'], ['30', '30 s'], ['60', '1 min'], ['120', '2 min'], ['300', '5 min']],
    value: '30', onChange: (v) => { plot.window = Number(v); } });
  const pauseBtn = button('Pause', () => { plot.paused = !plot.paused; pauseBtn.textContent = plot.paused ? 'Resume' : 'Pause'; });

  // Default series useful for the thesis.
  plot.add('tel:metrics:position_error', 'position error [m]');
  plot.add('tel:metrics:los_error', 'LOS error [rad]');
  plot.add('tel:status:range', 'distance [m]');
  refresh();

  return h('div', {},
    section('Display',
      h('div', { class: 'row wrap' }, button('Show plot', () => showPlot('plot'), 'primary'), button('Back to 3D', () => showPlot('3d'))),
      h('div', { class: 'row wrap' }, winSel, pauseBtn, button('Clear', () => plot.clear()))),
    section('Series', list),
    section('Add from telemetry', groupSel, fieldSel,
      button('Add', () => {
        const g = groupSel.get(), f = g === 'rtf' ? 'rtf' : fieldSel.get();
        plot.add(`tel:${g}:${f}`, g === 'rtf' ? 'RTF' : `${g}.${f}`);
        refresh(); showPlot('plot');
      }, 'primary')),
    section('Add from a topic', topicIn, fieldIn,
      button('Add', async () => {
        const topic = topicIn.get().trim(), field = fieldIn.get().trim();
        if (!topic || !field) return;
        const key = `topic:${topic}:${field}`;
        const sid = 'plot-' + Math.random().toString(36).slice(2);
        try {
          await ctl.req('sub', { sid, topic, rate: 30 });
          topicSubs.set(key, { sid, topic, field });
          plot.add(key, `${topic} ${field}`);
          refresh(); showPlot('plot');
        } catch (e) { toast(e.message, 'error'); }
      }, 'primary')));
}
