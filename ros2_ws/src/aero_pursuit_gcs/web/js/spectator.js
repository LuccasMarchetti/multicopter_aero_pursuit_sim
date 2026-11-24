// "Gazebo" view: free-camera video (spectator channel) and its control.
//  - Free: drag = look, WASD = walk, Q/E = down/up, Shift = fast,
//    wheel = speed. The pose is integrated here and sent at ~30 Hz.
//  - Follow / Orbit / Look: the spectator_camera node computes the pose from the target;
//    dragging changes azimuth/elevation, the wheel changes the distance.

import { h, select, toast } from './ui.js';
import { VideoView } from './video.js';

const SEND_MS = 33;
const KEYS = new Set(['w', 'a', 's', 'd', 'q', 'e', 'shift']);

function yawPitchFromQuat([x, y, z, w]) {
  // The camera looks along the link's +x (yaw on Z, pitch on Y; + looks down).
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
  return [yaw, pitch];
}

export class SpectatorView {
  constructor(ctl, state, root) {
    this.ctl = ctl;
    this.state = state;
    this.cmd = { mode: 'free', target: '', position: [-18, -14, 12], yaw: 0.6, pitch: 0.35,
      distance: 10, azimuth: 0, elevation: 0.35, orbit_rate: 0.15 };
    this.speed = 6;               // m/s in free mode
    this.keys = new Set();
    this.dirty = true;
    this.visible = false;

    const wrap = h('div', { class: 'video-wrap' });
    const canvas = h('canvas', {});
    const overlay = h('canvas', { class: 'overlay', tabindex: 0 });
    const message = h('div', { class: 'video-msg' });
    wrap.append(canvas, overlay, message);
    this.mode = select({ options: [['free', 'Free'], ['follow', 'Follow'], ['orbit', 'Orbit'], ['look', 'Look at']],
      value: 'free', onChange: (v) => this.setMode(v) });
    this.target = select({ options: [['X4', 'X4']], value: 'X4', onChange: (v) => { this.cmd.target = v; this.dirty = true; } });
    this.hint = h('span', { class: 'muted mini' });
    // FPS of the free camera: limits the extracted video (encoder and bandwidth). 0 turns the
    // camera off: the streamer unsubscribes and Isaac Sim stops rendering it (that render costs
    // ~3 ms per camera frame whatever the FPS/resolution, ~7 % of the real-time factor).
    this.fps = 30;
    this.fpsLabel = h('span', { class: 'mini' }, '30 FPS');
    this.fpsSlider = h('input', { type: 'range', min: 0, max: 30, step: 1, value: 30, class: 'fps-slider',
      title: 'Free camera FPS (0 = off: in Isaac Sim the RTF goes back to 1)' });
    this.fpsSlider.addEventListener('input', () => this.showFps(Number(this.fpsSlider.value)));
    this.fpsSlider.addEventListener('change', () => this.setFps(Number(this.fpsSlider.value)));
    const bar = h('div', { class: 'spectator-bar' }, this.mode, this.target,
      h('label', { class: 'fps-ctl' }, this.fpsSlider, this.fpsLabel), this.hint);
    root.append(bar, wrap);
    this.overlay = overlay;

    this.video = new VideoView(ctl, state, { wrap, canvas, overlay, message, channel: 'spectator', interactive: false });
    this.video.setEnabled(false);
    this.bindInput(overlay);
    this.updateHint();
    this.last = performance.now();
    const loop = () => { requestAnimationFrame(loop); this.step(); };
    requestAnimationFrame(loop);
    setInterval(() => this.refreshTargets(), 2000);
  }

  showFps(v) {
    this.fpsLabel.textContent = v ? `${v} FPS` : 'off';
  }

  setFps(v) {
    this.fps = v;
    this.showFps(v);
    this.video.setEnabled(this.visible && v > 0);
    if (v > 0) {
      this.ctl.req('stream.set', { channel: 'spectator', cfg: { fps: v } })
        .catch((e) => toast(`free camera: ${e.message}`, 'error'));
    }
  }

  /// Resumes the last camera command (another browser may have moved it).
  onHello(hello) {
    const fps = hello && hello.streams && hello.streams.spectator && hello.streams.spectator.cfg &&
      hello.streams.spectator.cfg.fps;
    if (fps) { this.fps = Math.round(fps); this.fpsSlider.value = this.fps; this.showFps(this.fps); }
    if (hello && hello.spectator) Object.assign(this.cmd, hello.spectator);
    this.mode.set(this.cmd.mode);
    if (this.cmd.target) this.target.set(this.cmd.target);
    this.updateHint();
  }

  setVisible(on) {
    this.visible = on;
    this.video.setEnabled(on && this.fps > 0);
    if (on) { this.dirty = true; setTimeout(() => this.video.resizeOverlay(), 30); }
  }

  refreshTargets() {
    const names = Object.keys(this.state.tf || {}).filter((n) => n !== 'spectator_camera' &&
      !/(_enu|camera_link|camera_optical|target_estimate)$/.test(n));
    if (!names.length) return;
    const cur = this.target.get();
    this.target.setOptions(names.sort().map((n) => [n, n]));
    if (names.includes(cur)) this.target.set(cur);
    if (!this.cmd.target) this.cmd.target = this.target.get();
  }

  setMode(mode) {
    if (mode === 'free' || mode === 'look') {
      // Starts from the current camera pose (TF published by spectator_camera).
      const tf = this.state.tf && this.state.tf.spectator_camera;
      if (tf) {
        this.cmd.position = [tf[0], tf[1], tf[2]];
        [this.cmd.yaw, this.cmd.pitch] = yawPitchFromQuat(tf.slice(3));
      }
    }
    this.cmd.mode = mode;
    this.cmd.target = this.target.get();
    this.dirty = true;
    this.updateHint();
  }

  updateHint() {
    const free = this.cmd.mode === 'free';
    this.target.style.display = free ? 'none' : '';
    this.hint.textContent = free
      ? `drag: look | WASD/QE: move | Shift: fast | wheel: speed (${this.speed.toFixed(0)} m/s)`
      : `drag: orbit | wheel: distance (${this.cmd.distance.toFixed(0)} m)`;
  }

  bindInput(el) {
    let drag = null;
    el.addEventListener('pointerdown', (e) => { el.focus(); el.setPointerCapture(e.pointerId); drag = [e.clientX, e.clientY]; });
    el.addEventListener('pointerup', () => { drag = null; });
    el.addEventListener('pointermove', (e) => {
      if (!drag) return;
      const dx = e.clientX - drag[0], dy = e.clientY - drag[1];
      drag = [e.clientX, e.clientY];
      const c = this.cmd;
      if (c.mode === 'free' || c.mode === 'look') {
        c.yaw -= dx * 0.004;
        c.pitch = Math.max(-1.4, Math.min(1.4, c.pitch + dy * 0.004));
      } else {
        c.azimuth -= dx * 0.006;
        c.elevation = Math.max(-0.3, Math.min(1.45, c.elevation + dy * 0.004));
      }
      this.dirty = true;
    });
    el.addEventListener('wheel', (e) => {
      e.preventDefault();
      const f = e.deltaY > 0 ? 1.15 : 1 / 1.15;
      if (this.cmd.mode === 'free') this.speed = Math.max(0.5, Math.min(60, this.speed / f));
      else { this.cmd.distance = Math.max(2, Math.min(150, this.cmd.distance * f)); this.dirty = true; }
      this.updateHint();
    }, { passive: false });
    el.addEventListener('keydown', (e) => {
      const k = e.key.toLowerCase();
      if (KEYS.has(k)) { this.keys.add(k); e.preventDefault(); }
    });
    el.addEventListener('keyup', (e) => this.keys.delete(e.key.toLowerCase()));
    el.addEventListener('blur', () => this.keys.clear());
  }

  step() {
    const now = performance.now();
    const dt = Math.min(0.1, (now - this.last) / 1000);
    this.last = now;
    if (!this.visible) return;
    const c = this.cmd;
    if (c.mode === 'free' && this.keys.size) {
      const v = this.speed * (this.keys.has('shift') ? 3 : 1) * dt;
      const fx = Math.cos(c.yaw), fy = Math.sin(c.yaw);
      let [x, y, z] = c.position;
      if (this.keys.has('w')) { x += fx * v; y += fy * v; z -= Math.sin(c.pitch) * v; }
      if (this.keys.has('s')) { x -= fx * v; y -= fy * v; z += Math.sin(c.pitch) * v; }
      if (this.keys.has('a')) { x -= fy * v; y += fx * v; }
      if (this.keys.has('d')) { x += fy * v; y -= fx * v; }
      if (this.keys.has('e')) z += v;
      if (this.keys.has('q')) z -= v;
      c.position = [x, y, Math.max(0.3, z)];
      this.dirty = true;
    }
    if (this.dirty && now - (this.sentT || 0) >= SEND_MS) {
      this.dirty = false;
      this.sentT = now;
      this.ctl.req('spectator', { cmd: c }).catch((e) => { if (!this.warned) { this.warned = true; toast(e.message, 'error'); } });
    }
  }

  resize() { this.video.resizeOverlay(); }
}
