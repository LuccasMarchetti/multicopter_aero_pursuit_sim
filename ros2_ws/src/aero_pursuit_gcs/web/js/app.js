// Aero GCS: wires connection, state, video, 3D, plots and tabs.

import { controlPanel } from './control.js';
import { h, fmt, grade } from './ui.js';
import { Ctl } from './ctl.js';
import { VideoView } from './video.js';
import { Scene3D } from './scene3d.js';
import { TimePlot, plotsPanel } from './plots.js';
import { pursuitPanel } from './pursuit.js';
import { dronesPanel } from './drones.js';
import { helpPanel } from './help.js';
import { streamPanel } from './stream.js';
import { joystickPanel } from './joystick.js';
import { paramsPanel } from './params.js';
import { simPanel } from './sim.js';
import { topicsPanel } from './topics.js';
import { networkPanel } from './network.js';
import { scenariosPanel } from './scenarios.js';
import { ScenarioLayer } from './scene_editor.js';
import { SpectatorView } from './spectator.js';
import { recorderButton } from './recorder.js';
import { WindowManager, PRESETS } from './layout.js';

const ctl = new Ctl();
window.gcs = { ctl };   // debugging through the browser console

const state = {
  worldFrame: 'lawn', tf: {}, status: null, metrics: null, detection: null, detectionT: 0,
  estimate: null, estimateT: 0, path: null, pathT: 0, teleop: null, px4: null,
  sim: null, stream: null, tunnel: null, sys: null, hello: null, rtf: null, scenario: null, truth: null,
};

// Session recording (this tab, in the browser): button in the top bar.
document.getElementById('layout-menu').before(recorderButton(state));
window.gcs.state = state;

ctl.on('tel', (m) => {
  if (m.tf) state.tf = m.tf;
  const t = performance.now();
  for (const k of ['status', 'metrics', 'teleop', 'px4', 'scenario', 'truth']) if (m[k]) state[k] = m[k];
  if (m.detection) { state.detection = m.detection; state.detectionT = t; }
  if (m.estimate) { state.estimate = m.estimate; state.estimateT = t; }
  if (m.path) { state.path = m.path; state.pathT = t; }
  if (m.rtf !== undefined) state.rtf = m.rtf;
});
ctl.on('sim.status', (m) => {
  state.sim = m;
  // Free-camera view named after the simulator (Gazebo or Isaac Sim).
  const b = document.querySelector('#view-switch [data-view="gazebo"]');
  const isaac = m.backend === 'isaac';
  if (b && b.dataset.backend !== m.backend) {
    b.dataset.backend = m.backend;
    b.textContent = isaac ? 'Isaac' : 'Gazebo';
    b.title = isaac ? 'Isaac Sim free camera' : 'Gazebo free camera';
    document.title = isaac ? 'Aero GCS · Isaac Sim' : 'Aero GCS · Gazebo';
  }
});
ctl.on('stream.state', (m) => { if ((m.channel || 'main') === 'main') state.stream = m; });
ctl.on('tunnel.status', (m) => { state.tunnel = m; });
ctl.on('sys', (m) => { state.sys = m; });

// -- vistas principais --------------------------------------------------------------
const video = new VideoView(ctl, state, {
  wrap: document.getElementById('video-wrap'),
  canvas: document.getElementById('video'),
  overlay: document.getElementById('overlay'),
  message: document.getElementById('video-msg'),
});
const scene = new Scene3D(document.getElementById('scene'), state);
window.gcs.scene = scene;   // debugging / manual screenshots
const plot = new TimePlot(document.getElementById('plot'));
const scenarioLayer = new ScenarioLayer(scene);
const spectator = new SpectatorView(ctl, state, document.getElementById('spectator'));

function showView(which) {
  document.getElementById('scene').style.display = which === '3d' ? '' : 'none';
  document.getElementById('plot').style.display = which === 'plot' ? '' : 'none';
  document.getElementById('spectator').style.display = which === 'gazebo' ? '' : 'none';
  document.querySelectorAll('#view-switch button').forEach((b) => b.classList.toggle('active', b.dataset.view === which));
  spectator.setVisible(which === 'gazebo');   // free-camera video only with the view open
  if (which === '3d') scene.resize(); else if (which === 'plot') plot.resize();
  localStorage.setItem('gcs.view', which);
}
document.querySelectorAll('#view-switch button').forEach((b) => b.addEventListener('click', () => showView(b.dataset.view)));
showView(localStorage.getItem('gcs.view') || '3d');

const follow = document.getElementById('follow');
for (const [v, t] of scene.followOptions()) follow.append(h('option', { value: v }, t));
follow.addEventListener('change', () => { scene.follow = follow.value; });
document.getElementById('trails').addEventListener('change', (e) => { scene.showTrails = e.target.checked; });
document.getElementById('frames').addEventListener('change', (e) => { scene.showFrames = e.target.checked; });
document.getElementById('clear-trails').addEventListener('click', () => scene.clearTrails());
document.getElementById('video-full').addEventListener('click', () => {
  const pane = document.getElementById('video-pane');
  document.fullscreenElement ? document.exitFullscreen() : pane.requestFullscreen();
});
document.getElementById('video-stop').addEventListener('click', () => video.stopTracking());

// Resizable splitter between video and 3D.
(() => {
  const split = document.getElementById('splitter');
  const views = document.getElementById('views');
  const saved = localStorage.getItem('gcs.split');
  if (saved) views.style.setProperty('--split', saved);
  split.addEventListener('pointerdown', (e) => {
    split.setPointerCapture(e.pointerId);
    const vertical = getComputedStyle(views).flexDirection === 'column';
    const move = (ev) => {
      const r = views.getBoundingClientRect();
      const f = vertical ? (ev.clientY - r.top) / r.height : (ev.clientX - r.left) / r.width;
      const v = `${Math.max(15, Math.min(85, f * 100)).toFixed(1)}%`;
      views.style.setProperty('--split', v);
      localStorage.setItem('gcs.split', v);
    };
    const up = () => { split.removeEventListener('pointermove', move); split.removeEventListener('pointerup', up); };
    split.addEventListener('pointermove', move);
    split.addEventListener('pointerup', up);
  });
})();

// -- abas ---------------------------------------------------------------------------
const params = paramsPanel(ctl);
const topics = topicsPanel(ctl);
const network = networkPanel(ctl, state, video);
const scenarios = scenariosPanel(ctl, state, scenarioLayer, showView);
const control = controlPanel(ctl);
// Order of the usage flow: start the simulation -> pursue -> move drones -> scenarios
// -> analyze; then the fine-tuning and diagnostics tabs. Manual: docs/MANUAL_GCS.md.
const TABS = [
  ['sim', 'Simulation', simPanel(ctl, state)],
  ['pursuit', 'Pursuit', pursuitPanel(ctl, state, video)],
  ['control', 'Control', control.el, control.onShow],
  ['drones', 'Drones', dronesPanel(ctl, state)],
  ['joy', 'Joystick', joystickPanel(ctl)],
  ['scenarios', 'Scenarios', scenarios.el, scenarios.onShow],
  ['plots', 'Plots', plotsPanel(ctl, state, plot, showView)],
  ['params', 'Parameters', params.el, params.refresh],
  ['video', 'Video', streamPanel(ctl, state, video)],
  ['topics', 'Topics', topics.el, topics.refresh],
  ['net', 'Network', network.el],
  ['help', 'Help', helpPanel()],
];
const tabs = document.getElementById('tabs');
const panels = document.getElementById('panels');
const popped = new Map();   // tab -> own window
let currentTab = null;
function selectTab(id) {
  if (popped.has(id)) {  // detached tab: brings the window to the front
    wm.raise(popped.get(id));
    return;
  }
  currentTab = id;
  for (const [tid, , el, onShow] of TABS) {
    if (popped.has(tid)) continue;
    const on = tid === id;
    el.style.display = on ? '' : 'none';
    tabs.querySelector(`[data-tab="${tid}"]`).classList.toggle('active', on);
    if (on && onShow) onShow();
  }
  localStorage.setItem('gcs.tab', id);
}

// -- layout modular (janelas) ---------------------------------------------------------
const resizeAll = () => { scene.resize(); plot.resize(); video.resizeOverlay(); spectator.resize(); };
const wm = new WindowManager(document.getElementById('main'), resizeAll);
wm.add('video', { el: document.getElementById('video-pane'), title: 'Camera',
  handle: document.querySelector('#video-pane .pane-bar'), defaultRect: PRESETS.padrao.video,
  onResize: () => video.resizeOverlay() });
wm.add('scene', { el: document.getElementById('scene-pane'), title: '3D / Gazebo / Plots',
  handle: document.querySelector('#scene-pane .pane-bar'), defaultRect: PRESETS.padrao.scene,
  onResize: () => { scene.resize(); plot.resize(); spectator.resize(); } });
wm.add('side', { el: document.getElementById('side'), title: 'Tab panel',
  handle: document.getElementById('side-bar'), defaultRect: PRESETS.padrao.side });

function popTab(id) {
  if (popped.has(id) || wm.mode !== 'free') return;
  const tab = TABS.find(([tid]) => tid === id);
  if (!tab) return;
  const [, label, el, onShow] = tab;
  const bar = h('div', { class: 'pane-bar' }, h('span', {}, label), h('span', { class: 'spacer' }),
    h('button', { class: 'small', title: 'Return to the tab panel', onclick: () => wm.close(`tab:${id}`) }, '⇲'));
  const body = h('div', { class: 'win-body' });
  const win = h('div', { class: 'pane win-tab' }, bar, body);
  body.append(el);
  el.style.display = '';
  document.getElementById('main').append(win);
  const n = popped.size;
  const w = wm.add(`tab:${id}`, { el: win, title: label, handle: bar, closable: true,
    defaultRect: [Math.min(0.62, 0.30 + 0.04 * n), 0.04 + 0.04 * n, 0.3, 0.7],
    onClose: () => {
      popped.delete(id);
      panels.append(el);
      tabs.querySelector(`[data-tab="${id}"]`).classList.remove('popped');
      selectTab(currentTab && !popped.has(currentTab) ? currentTab : id);
    } });
  popped.set(id, w);
  tabs.querySelector(`[data-tab="${id}"]`).classList.add('popped');
  if (!(wm.saved.popped || []).includes(id)) { (wm.saved.popped ||= []).push(id); wm.save(); }
  wm.raise(w);
  if (onShow) onShow();
  if (currentTab === id) {  // the tab that was open in the panel left: show another
    const other = TABS.find(([tid]) => !popped.has(tid));
    if (other) selectTab(other[0]);
  }
}

for (const [id, label, el] of TABS) {
  const pop = h('span', { class: 'tab-pop', title: 'Open in its own window (free mode)',
    onclick: (e) => { e.stopPropagation(); popTab(id); } }, '⧉');
  tabs.append(h('button', { 'data-tab': id, onclick: () => selectTab(id) }, label, pop));
  panels.append(el);
}
const savedTab = localStorage.getItem('gcs.tab');
// /app#tab=<id> (home screen "Entrar"): opens on that tab.
const hashTab = (location.hash.match(/tab=([\w-]+)/) || [])[1];
if (hashTab) history.replaceState(null, '', location.pathname + location.search);
selectTab([hashTab, savedTab].find((id) => TABS.some(([t]) => t === id)) || 'pursuit');
for (const id of [...(wm.saved.popped || [])]) popTab(id);
wm.applyAll();

// Layout menu
const layoutPop = document.getElementById('layout-pop');
function renderLayoutMenu() {
  const free = wm.mode === 'free';
  const item = (label, fn, on) => h('button', { class: `menu-item${on ? ' on' : ''}`, onclick: () => { fn(); renderLayoutMenu(); } }, label);
  const vis = (id, label) => {
    const hidden = wm.saved.wins[id]?.hidden;
    return item(`${hidden ? '☐' : '☑'} ${label}`, () => wm.setHidden(id, !hidden), false);
  };
  layoutPop.replaceChildren(
    h('div', { class: 'menu-head' }, 'Mode'),
    item('Free (windows)', () => wm.setMode('free'), free),
    item('Classic (fixed)', () => {
      for (const id of [...popped.keys()]) wm.close(`tab:${id}`);
      wm.setMode('classic');
    }, !free),
    h('div', { class: 'menu-head' }, 'Presets'),
    item('Default', () => wm.preset('padrao')),
    item('Pursuit (video + 3D below)', () => wm.preset('perseguicao')),
    item('Large video', () => wm.preset('video_grande')),
    item('Analysis (tabs detached on the right)', () => wm.preset('analise')),
    h('div', { class: 'menu-head' }, 'Windows'),
    vis('video', 'Camera'), vis('scene', '3D / Gazebo / Plots'), vis('side', 'Tab panel'),
    h('div', { class: 'menu-head' }, ''),
    item('Restore default layout', () => wm.reset()),
    h('div', { class: 'menu-hint' }, free ? 'Drag by the title bar; edges and corners resize; double click maximizes; ⧉ on a tab opens it in a window.' :
      'In classic mode the views sit side by side with the splitter; narrow screens always use this mode.'));
}
document.getElementById('layout-btn').addEventListener('click', (e) => {
  e.stopPropagation();
  layoutPop.hidden = !layoutPop.hidden;
  if (!layoutPop.hidden) renderLayoutMenu();
});
document.addEventListener('click', (e) => { if (!e.target.closest('#layout-menu')) layoutPop.hidden = true; });
document.getElementById('toggle-side').addEventListener('click', () => {
  if (wm.mode === 'free') {
    wm.setHidden('side', !wm.saved.wins.side?.hidden);
  } else {
    document.body.classList.toggle('side-hidden');
  }
  setTimeout(resizeAll, 50);
});

// -- header indicators ----------------------------------------------------------
const badges = {};
for (const [id, label, title] of [
  ['conn', 'Connection', 'Control channel RTT (ping every 0.5 s)'],
  ['lat', 'Latency', 'Streamer capture → frame on screen (median; p95 in the tooltip)'],
  ['fps', 'FPS', 'Frames shown per second'],
  ['bw', 'Bandwidth', 'Video received'],
  ['drop', 'Losses', 'Frames dropped to avoid accumulating delay'],
  ['sim', 'Simulation', 'Simulator state and real-time factor'],
  ['mode', 'Pursuit', 'Supervisor mode'],
  ['cpu', 'CPU/GPU', 'Simulation machine usage'],
]) {
  const v = h('b', {}, '—');
  const el = h('div', { class: 'badge', title }, h('span', {}, label), v);
  el.set = (text, g, tip) => { v.textContent = text; el.dataset.grade = g || ''; if (tip) el.title = `${title}\n${tip}`; };
  badges[id] = el;
  document.getElementById('badges').append(el);
}
ctl.on('open', async () => {
  badges.conn.set('online', 'good');
  try {
    const hello = await ctl.req('hello');
    state.hello = hello;
    state.worldFrame = hello.world_frame;
    for (const [op, m] of [['stream.state', hello.stream], ['sim.status', hello.sim], ['tunnel.status', hello.tunnel]]) ctl.emit(op, m);
    for (const m of Object.values(hello.streams || {})) ctl.emit('stream.state', m);
    network.onHello(hello);
    spectator.onHello(hello);
    if (!hello.ros) badges.sim.set('no ROS', 'bad');
  } catch (e) { console.error(e); }
});
ctl.on('close', () => badges.conn.set('offline', 'bad'));
ctl.on('rtt', ({ rtt, jitter }) => badges.conn.set(`${fmt(rtt, 0)} ms`, grade(rtt, 60, 150), `jitter ${fmt(jitter, 1)} ms`));
video.onStats((s, vs) => {
  if (!s.drawFps) { badges.lat.set('—', 'na'); badges.fps.set('0', s.recvFps ? 'bad' : 'na'); }
  else {
    badges.lat.set(`${fmt(s.latency, 0)} ms`, grade(s.latency, 120, 250),
      `p95 ${fmt(s.latencyP95, 0)} ms | encode ${fmt(s.enc, 1)} | network ${fmt(s.net, 1)} | decode ${fmt(s.dec, 1)} ms`);
    badges.fps.set(fmt(s.drawFps, 0), grade(s.drawFps, 24, 12, true));
  }
  badges.bw.set(`${fmt(s.kbps / 1000, 1)} Mb/s`);
  if (vs) badges.drop.set(`${fmt(vs.drop_fps, 0)}/s`, vs.drop_fps > 0.5 ? 'bad' : vs.drop_fps > 0 ? 'ok' : 'good', `total ${vs.drops_total}`);
});
ctl.on('sim.status', (s) => {
  if (s.busy) badges.sim.set(s.busy + '...', 'ok');
  else if (!s.running) badges.sim.set('stopped', 'na');
  else badges.sim.set(state.rtf ? `RTF ${fmt(state.rtf, 2)}` : 'running', state.rtf ? grade(state.rtf, 0.95, 0.7, true) : 'good');
});
ctl.on('tel', (m) => {
  if (m.status) {
    const g = { TRACKING: 'good', APPROACH: 'ok', COASTING: 'ok', LOST: 'bad' }[m.status.mode_name] || 'na';
    badges.mode.set(m.status.mode_name || '—', g);
  }
});
ctl.on('sys', (m) => badges.cpu.set(`${fmt(m.cpu, 0)}% / ${m.gpu ?? '—'}%`, grade(Math.max(m.cpu || 0, m.gpu || 0), 70, 90)));
