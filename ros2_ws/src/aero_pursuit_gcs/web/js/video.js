// Video view: canvas drawn by the worker + overlay (bounding box, crosshair, HUD) and
// target selection by dragging over the image.

import { fmt } from './ui.js';

const STATUS_NAMES = ['idle', 'lost', 'confirming', 'confirmed'];

export class VideoView {
  /// `channel`: server channel (main = drone camera, spectator = free camera).
  /// `interactive`: target selection by dragging and tracker overlays (main only).
  constructor(ctl, state, { wrap, canvas, overlay, message, channel = 'main', interactive = true }) {
    this.ctl = ctl;
    this.state = state;
    this.channel = channel;
    this.interactive = interactive;
    this.wrap = wrap;
    this.canvas = canvas;
    this.overlay = overlay;
    this.message = message;
    this.size = null;           // {w, h, srcW, srcH, codec}
    this.stats = null;
    this.vstats = null;
    this.connected = false;
    this.drag = null;
    this.hover = null;
    this.options = interactive ? { bbox: true, crosshair: true, hud: true } : { bbox: false, crosshair: false, hud: false };
    this.listeners = [];
    this.lastError = '';

    const off = canvas.transferControlToOffscreen();
    this.worker = new Worker(new URL('./video_worker.js', import.meta.url));
    const url = (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws/video?ch=' + channel;
    this.worker.postMessage({ type: 'init', canvas: off, url }, [off]);
    this.worker.onmessage = (ev) => this.onWorker(ev.data);
    ctl.on('rtt', ({ offset }) => this.worker.postMessage({ type: 'offset', value: offset }));
    ctl.on('stream.state', (s) => {
      if ((s.channel || 'main') !== channel) return;
      this.streamState = s;
      const codecs = {};
      if (s.info && s.info.codec) {
        const id = { h264: 1, h265: 2, mjpeg: 3 }[s.info.name];
        if (id) codecs[id] = s.info.codec;
      }
      this.worker.postMessage({ type: 'codecs', codecs });
    });
    if (interactive) this.bindPointer();
    new ResizeObserver(() => this.resizeOverlay()).observe(wrap);
    this.resizeOverlay();
    const loop = () => { this.drawOverlay(); requestAnimationFrame(loop); };
    requestAnimationFrame(loop);
  }

  onStats(fn) { this.listeners.push(fn); }

  onWorker(m) {
    if (m.type === 'stats') {
      this.stats = m;
      for (const fn of this.listeners) fn(m, this.vstats);
    } else if (m.type === 'vstats') {
      this.vstats = m.data;
    } else if (m.type === 'size') {
      this.size = m;
    } else if (m.type === 'conn') {
      this.connected = m.connected;
    } else if (m.type === 'error') {
      this.lastError = m.text;
      this.lastErrorT = performance.now();
    }
  }

  setEnabled(on) { this.worker.postMessage({ type: 'enable', on }); }
  requestKey() { this.worker.postMessage({ type: 'key' }); }

  resizeOverlay() {
    const r = this.wrap.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    this.overlay.width = Math.round(r.width * dpr);
    this.overlay.height = Math.round(r.height * dpr);
  }

  /// Rectangle (CSS px) where the image appears inside the wrap (object-fit: contain).
  imageRect() {
    const r = this.wrap.getBoundingClientRect();
    if (!this.size) return null;
    const s = Math.min(r.width / this.size.w, r.height / this.size.h);
    const w = this.size.w * s, hgt = this.size.h * s;
    return { x: (r.width - w) / 2, y: (r.height - hgt) / 2, w, h: hgt, scale: s };
  }

  /// Wrap point (CSS px) -> ORIGINAL image pixel (what the tracker uses).
  toSource(px, py) {
    const ir = this.imageRect();
    if (!ir) return null;
    const u = (px - ir.x) / ir.w, v = (py - ir.y) / ir.h;
    return { x: u * this.size.srcW, y: v * this.size.srcH, inside: u >= 0 && u <= 1 && v >= 0 && v <= 1 };
  }

  /// Original image pixel -> wrap (CSS px).
  fromSource(x, y, srcW, srcH) {
    const ir = this.imageRect();
    if (!ir) return null;
    return { x: ir.x + (x / srcW) * ir.w, y: ir.y + (y / srcH) * ir.h };
  }

  bindPointer() {
    const el = this.overlay;
    const pos = (e) => { const r = el.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; };
    el.addEventListener('pointerdown', (e) => {
      if (e.button !== 0 || !this.size) return;
      const [x, y] = pos(e);
      const p = this.toSource(x, y);
      if (!p || !p.inside) return;
      el.setPointerCapture(e.pointerId);
      this.drag = { a: p, b: p };
    });
    el.addEventListener('pointermove', (e) => {
      const [x, y] = pos(e);
      this.hover = this.toSource(x, y);
      if (this.drag && this.hover) this.drag.b = this.hover;
    });
    el.addEventListener('pointerleave', () => { this.hover = null; });
    el.addEventListener('pointerup', () => {
      if (!this.drag) return;
      const { a, b } = this.drag;
      this.drag = null;
      const clamp = (v, m) => Math.max(0, Math.min(m, v));
      const x0 = clamp(Math.min(a.x, b.x), this.size.srcW), x1 = clamp(Math.max(a.x, b.x), this.size.srcW);
      const y0 = clamp(Math.min(a.y, b.y), this.size.srcH), y1 = clamp(Math.max(a.y, b.y), this.size.srcH);
      if (x1 - x0 < 6 || y1 - y0 < 6) return;
      this.ctl.req('select', { x: Math.round(x0), y: Math.round(y0), w: Math.round(x1 - x0), h: Math.round(y1 - y0) })
        .catch((err) => this.flash(err.message));
      this.flash('Target selected');
    });
    el.addEventListener('contextmenu', (e) => { e.preventDefault(); this.stopTracking(); });
    window.addEventListener('keydown', (e) => {
      if (e.key === 'Escape' && !['INPUT', 'TEXTAREA', 'SELECT'].includes(document.activeElement.tagName)) {
        this.drag ? (this.drag = null) : this.stopTracking();
      }
    });
  }

  stopTracking() {
    this.ctl.req('select.stop').catch(() => {});
    this.flash('Tracking stopped');
  }

  flash(text) { this.flashText = text; this.flashT = performance.now(); }

  drawOverlay() {
    const c = this.overlay, ctx = c.getContext('2d');
    const dpr = window.devicePixelRatio || 1;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, c.width, c.height);
    const W = c.width / dpr, H = c.height / dpr;
    const ir = this.imageRect();

    let msg = '';
    if (!this.connected) msg = 'Connecting to video...';
    else if (!this.streamState || !this.streamState.connected) msg = 'Streamer offline: start the simulation (Simulation tab) with "GCS video" on';
    else if (!this.size) msg = 'Waiting for the first frame...';
    if (this.lastError && performance.now() - this.lastErrorT < 8000) msg = this.lastError;
    this.message.textContent = msg;
    this.message.style.display = msg ? 'block' : 'none';
    if (!ir) return;

    const det = this.state.detection;
    const detFresh = det && performance.now() - this.state.detectionT < 1000;
    if (this.options.crosshair) {
      ctx.strokeStyle = 'rgba(255,255,255,0.35)'; ctx.lineWidth = 1;
      const cx = ir.x + ir.w / 2, cy = ir.y + ir.h / 2;
      ctx.beginPath(); ctx.moveTo(cx - 14, cy); ctx.lineTo(cx + 14, cy); ctx.moveTo(cx, cy - 14); ctx.lineTo(cx, cy + 14); ctx.stroke();
    }
    if (this.options.bbox && detFresh && det.status >= 2 && det.width > 0) {
      const p0 = this.fromSource(det.x, det.y, det.image_width, det.image_height);
      const p1 = this.fromSource(det.x + det.width, det.y + det.height, det.image_width, det.image_height);
      const color = det.status === 3 ? (det.has_depth ? '#4caf50' : '#4fc3f7') : '#ffa726';
      ctx.strokeStyle = color; ctx.lineWidth = 2;
      ctx.strokeRect(p0.x, p0.y, p1.x - p0.x, p1.y - p0.y);
      const label = `${(STATUS_NAMES[det.status])} ${fmt(det.score, 2)}` +
        (det.has_depth ? ` | ${fmt(det.depth, 1)} m` : '') + ` | ${fmt(det.processing_time_ms, 0)} ms`;
      ctx.font = '12px sans-serif';
      const tw = ctx.measureText(label).width;
      ctx.fillStyle = 'rgba(0,0,0,0.6)'; ctx.fillRect(p0.x, p0.y - 18, tw + 8, 16);
      ctx.fillStyle = color; ctx.fillText(label, p0.x + 4, p0.y - 6);
    }
    if (this.drag) {
      const a = this.fromSource(this.drag.a.x, this.drag.a.y, this.size.srcW, this.size.srcH);
      const b = this.fromSource(this.drag.b.x, this.drag.b.y, this.size.srcW, this.size.srcH);
      ctx.setLineDash([6, 4]); ctx.strokeStyle = '#ffeb3b'; ctx.lineWidth = 1.5;
      ctx.strokeRect(a.x, a.y, b.x - a.x, b.y - a.y); ctx.setLineDash([]);
    }
    if (this.options.hud) {
      const st = this.state.status;
      const s = this.stats;
      const lines = [];
      if (st) lines.push(`${st.mode_name || '—'} | ${st.controller} | d=${fmt(st.range, 1)} m`);
      // ENU velocities [m/s] and rates [°/s]; the target's from ours + the relative estimate.
      const vec = (v) => (v ? v.map((x) => fmt(x, 1).padStart(5)).join(' ') : '—');
      const deg = (r) => (r === null || r === undefined ? null : r * 180 / Math.PI);
      if (st && st.chaser_velocity) lines.push(`pursuer v[${vec(st.chaser_velocity)}] m/s  r=${fmt(deg(st.chaser_yaw_rate), 0)}°/s`);
      if (st && st.target_velocity) lines.push(`target  v[${vec(st.target_velocity)}] m/s  ω=${fmt(deg(st.target_turn_rate), 0)}°/s`);
      if (s) lines.push(`${this.size.w}x${this.size.h} ${this.size.codec} | ${fmt(s.drawFps, 0)} fps | ${fmt(s.latency, 0)} ms`);
      ctx.font = '12px monospace';
      lines.forEach((t, i) => {
        const tw = ctx.measureText(t).width;
        ctx.fillStyle = 'rgba(0,0,0,0.55)'; ctx.fillRect(ir.x + 6, ir.y + 6 + i * 17, tw + 8, 16);
        ctx.fillStyle = '#e0e0e0'; ctx.fillText(t, ir.x + 10, ir.y + 18 + i * 17);
      });
    }
    if (this.flashText && performance.now() - this.flashT < 1500) {
      ctx.font = 'bold 14px sans-serif';
      const tw = ctx.measureText(this.flashText).width;
      ctx.fillStyle = 'rgba(0,0,0,0.7)'; ctx.fillRect(W / 2 - tw / 2 - 8, H - 40, tw + 16, 24);
      ctx.fillStyle = '#ffeb3b'; ctx.fillText(this.flashText, W / 2 - tw / 2, H - 23);
    }
  }
}
