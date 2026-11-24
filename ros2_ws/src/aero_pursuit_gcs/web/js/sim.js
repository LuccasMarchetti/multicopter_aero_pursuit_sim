// Simulation tab: start/stop/restart with every launch and script option,
// build, process state, live logs and PX4 console.

import { h, button, select, section, toggle, textInput, toast, fmt } from './ui.js';
import { replayPanel } from './replay_panel.js';
import { studiesPanel } from './studies_panel.js';


// [key, label, type] grouped by subject.
const GROUPS = [
  ['Pursuer control', [
  ]],
  ['Pursuer (where the on-board nodes run)', [
    ['pursuer_host', 'Pursuer execution', [['local', 'local (this computer)'], ['remote', 'remote (Raspberry Pi etc., docker/pursuer.sh)']]],
    ['pursuer_remote', 'Remote host (user@IP; empty = AERO_PI_HOST variable)', 'text'],
    ['pursuer_max_rtt_ms', 'Stop the simulation if the latency exceeds (ms; 0 = never)', 'number'],
  ]],
  ['Input and target', [
    ['joystick', 'Joystick', [['app', 'browser (this GCS)'], ['local', 'USB on the server'], ['none', 'none']]],
    ['joy_driver', 'USB driver', ['game_controller_node', 'joy_node']],
  ]],
  ['Visual scenery', [
    ['isaac_env', 'Isaac Sim environment (Isaac only)', [['Flat_Plane', 'Flat Plane (grey tiles, default)'],
      ['Rough_Plane', 'Rough Plane (terreno irregular)'], ['Slope_Plane', 'Slope Plane (rampa/morro)'],
      ['Stairs_Plane', 'Stairs Plane (degraus)'], ['Default_Environment', 'Default Environment (grade)'],
      ['Black_Gridroom', 'Black Gridroom'], ['Warehouse', 'Warehouse'], ['Full_Warehouse', 'Full Warehouse']]],
    ['scenery', 'Scenery (restarts the world; Gazebo and Isaac)', [['none', 'none (open field)'], ['forest_sparse', 'trees in the background, light (forest_sparse)'], ['forest_edge', 'trees around and behind (forest_edge)']]],
  ]],
  ['Perception and recording', [
    ['dnn_target', 'Inference', ['cpu', 'opencl', 'opencl_fp16']],
    ['tf_source', 'TFs', [['gazebo', 'PosePublisher (Gazebo)'], ['node', 'gazebo_tf_bridge']]],
    ['record', 'Record dataset (rosbag)', 'bool'],
    ['record_mode', 'Recording mode', ['session', 'continuous']],
    ['gcs_stream', 'GCS video (streamer)', 'bool'],
    ['foxglove', 'Foxglove bridge', 'bool'],
  ]],
  ['Gazebo, PX4 and advanced', [
    ['world', 'World', 'text'],
    ['gz_headless', 'Headless Gazebo', 'bool'],
    ['gz_render_api', 'Render', ['opengl', 'vulkan']],
    ['skip_px4_build', 'Skip PX4 build', 'bool'],
    ['px4_quiet', 'PX4 console only in the log', 'bool'],
    ['log_level', 'ROS log level', ['warn', 'info', 'debug', 'error']],
    ['debug', 'Container under gdbserver', 'bool'],
    ['extra_args', 'Extra launch args', 'text'],
  ]],
];

export function simPanel(ctl, state) {
  const controls = {};
  const cfg = {};
  const save = () => ctl.req('sim.save', { cfg }).catch((e) => toast(e.message, 'error'));
  const form = h('div', { class: 'form' });
  for (const [title, fields] of GROUPS) {
    form.append(h('h4', {}, title));
    for (const [key, label, kind] of fields) {
      const onChange = (v) => { cfg[key] = v; save(); };
      let el;
      if (kind === 'bool') el = toggle({ label, onChange });
      else if (kind === 'text') el = textInput({ label, onChange });
      else if (kind === 'number') el = textInput({ label, type: 'number', onChange: (v) => onChange(Number(v) || 0) });
      else el = select({ label, options: kind, onChange });
      controls[key] = el;
      form.append(el);
    }
  }

  // Simulator of this GCS (backends.py): switching restarts the GCS process.
  const SIM_NAMES = { gazebo: 'Gazebo', isaac: 'Isaac Sim' };
  let backendNow = '';
  const backendSel = select({ label: 'Simulator of this GCS (switching restarts the GCS)', options: [], onChange: async (b) => {
    if (b === backendNow) return;
    if (!confirm(`Switch the simulator to ${SIM_NAMES[b] || b}? The GCS restarts and the page reloads.`)) { backendSel.set(backendNow); return; }
    try {
      const r = await ctl.req('sim.backend', { backend: b });
      if (r && r.restart) { toast('restarting the GCS...'); setTimeout(() => location.reload(), 5000); }
    } catch (e) { toast(e.message, 'error', 6000); backendSel.set(backendNow); }
  } });
  const backendHint = h('div', { class: 'muted' });
  function renderBackends(s) {
    const ch = s.backends;
    backendNow = s.backend;
    backendSel.style.display = ch && !ch.fixed ? '' : 'none';
    if (!ch || ch.fixed) { backendHint.textContent = ''; return; }
    const opts = ch.options || {};
    backendSel.setOptions(Object.keys(SIM_NAMES).map((b) => [b,
      SIM_NAMES[b] + (opts[b] && !opts[b].ok ? ` — ${opts[b].reason}` : ''), !(opts[b] && opts[b].ok) && b !== s.backend]));
    if (document.activeElement !== backendSel.input) backendSel.set(s.backend);
    const other = Object.keys(SIM_NAMES).find((b) => b !== s.backend);
    backendHint.textContent = opts[other] && !opts[other].ok ? `${SIM_NAMES[other]} unavailable: ${opts[other].reason}` : '';
  }

  const procBox = h('div', { class: 'procs' });
  // Remote pursuer (pursuer_host: remote): container on the other computer, polled over SSH.
  const pursuerBox = h('div', { class: 'procs' });
  const pursuerRow = h('div', { class: 'row wrap' },
    button('▶ Pursuer', () => run('pursuer.start'), 'small', 'starts the aero-pursuer container on the remote host again'),
    button('■ Pursuer', () => run('pursuer.stop'), 'small danger', 'stops only the remote container'),
    button('↻', () => run('pursuer.poll'), 'small', 'query the host now'),
    button('Log', () => { logSel.set('pursuer'); subscribeLog('pursuer'); }, 'small'));
  const pursuerSec = h('div', {}, h('h4', {}, 'Remote pursuer'), pursuerBox, pursuerRow);
  function renderPursuer(p) {
    pursuerSec.style.display = p ? '' : 'none';
    if (!p) return;
    pursuerBox.innerHTML = '';
    const rtt = p.rtt_ms == null ? '—' : `${fmt(p.rtt_ms, 1)} ms (mediana ${fmt(p.rtt_med_ms, 1)} ms)`;
    const cls = p.reachable === false ? 'bad' : p.running ? 'good' : 'na';
    pursuerBox.append(
      h('div', { class: 'proc' }, h('span', { class: 'dot ' + cls }), p.host || 'remote host',
        h('span', { class: 'muted' }, p.reachable === false ? 'unreachable' : p.running ? 'aero-pursuer running' : 'container stopped')),
      h('div', { class: 'proc' }, 'Network latency (TCP)', h('span', { class: 'muted' }, rtt)),
      h('div', { class: 'proc' }, 'CPU | memory | temperature',
        h('span', { class: 'muted' }, `${p.cpu || '—'} | ${p.mem || '—'} | ${p.temp_c == null ? '—' : fmt(p.temp_c, 1) + ' °C'}`)));
    if (p.error) pursuerBox.append(h('div', { class: 'muted' }, p.error));
  }
  const busy = h('div', { class: 'muted' });
  const backendNote = h('div', { class: 'muted' });
  const startBtn = button('▶ Start', () => run('sim.start', { cfg }), 'primary');
  const stopBtn = button('■ Stop', () => run('sim.stop'), 'danger');
  const restartBtn = button('⟲ Restart', restart);
  const buildBtn = button('⚙ Build', () => { logSel.set('build'); subscribeLog('build'); run('sim.build'); },
    '', 'colcon build of the aero_pursuit and aero_pursuit_gcs packages');

  async function run(op, args = {}) {
    try { await ctl.req(op, args); } catch (e) { toast(e.message, 'error', 6000); }
  }

  function restart() { return run('sim.restart', { cfg }); }

  ctl.on('sim.status', (s) => {
    // Isaac Sim (isaac/host.sh): no Gazebo options; build = aero-isaac image on the host.
    const hidden = new Set(s.unsupported || []);
    for (const [k, el] of Object.entries(controls)) el.style.display = hidden.has(k) ? 'none' : '';
    const isaac = s.backend === 'isaac';
    buildBtn.title = isaac ? 'colcon build of the aero_pursuit and aero_pursuit_gcs packages (also the ROS side of Isaac)'
      : 'colcon build of the aero_pursuit and aero_pursuit_gcs packages';
    backendNote.textContent = isaac ? 'Isaac Sim + Pegasus (isaac/host.sh): one pursuer, physics targets.' : '';
    for (const [k, el] of Object.entries(controls)) {
      if (s.cfg[k] !== undefined && document.activeElement !== el.input) { el.set(s.cfg[k]); cfg[k] = s.cfg[k]; }
    }
    renderBackends(s);
    renderPursuer(s.pursuer);
    controls.pursuer_remote.style.display = controls.pursuer_max_rtt_ms.style.display =
      hidden.has('pursuer_host') || cfg.pursuer_host !== 'remote' ? 'none' : '';
    procBox.innerHTML = '';
    for (const [, p] of Object.entries(s.processes)) {
      const n = p.instances.length;
      const cpu = p.instances.reduce((a, i) => a + (i.cpu || 0), 0);
      const rss = p.instances.reduce((a, i) => a + i.rss_mb, 0);
      procBox.append(h('div', { class: 'proc' }, h('span', { class: 'dot ' + (n ? 'good' : 'na') }), p.label,
        h('span', { class: 'muted' }, n ? `${n > 1 ? n + '× ' : ''}${fmt(cpu, 0)}% CPU | ${fmt(rss, 0)} MB` : 'stopped')));
    }
    busy.textContent = s.busy ? `In progress: ${s.busy}...` : s.last_result || '';
    startBtn.disabled = !!s.busy || s.running;
    stopBtn.disabled = s.busy === 'stop';
    buildBtn.disabled = !!s.busy;
    const logs = s.logs || [];
    const cur = logSel.get();
    logSel.setOptions(logs.length ? logs : ['gcs']);
    if (cur && logs.includes(cur)) logSel.set(cur);
  });

  // Live logs.
  const logView = h('pre', { class: 'log' });
  const logFilter = textInput({ placeholder: 'filter (e.g. pursuit_controller_node, ERROR)' });
  let follow = true;
  let lines = [];
  const logSel = select({ label: 'Log', options: ['aero_pursuit'], onChange: (v) => subscribeLog(v) });
  function subscribeLog(name) { lines = []; logView.textContent = ''; ctl.req('log.sub', { name }).catch(() => {}); }
  const renderLog = () => {
    const q = logFilter.get().trim().toLowerCase();
    const shown = q ? lines.filter((l) => l.toLowerCase().includes(q)) : lines;
    logView.textContent = shown.slice(-1500).join('\n');
    if (follow) logView.scrollTop = logView.scrollHeight;
  };
  logFilter.input.addEventListener('input', renderLog);
  logView.addEventListener('scroll', () => { follow = logView.scrollTop + logView.clientHeight >= logView.scrollHeight - 30; });
  ctl.on('log', (m) => {
    if (m.name !== logSel.get()) return;
    if (m.reset) lines = [];
    lines.push(...m.lines);
    if (lines.length > 5000) lines = lines.slice(-4000);
    renderLog();
  });
  ctl.on('open', () => setTimeout(() => subscribeLog(logSel.get() || 'aero_pursuit'), 300));

  const showLog = (name) => { logSel.set(name); subscribeLog(name); };
  const studiesSec = studiesPanel(ctl, { run, showLog });
  const replaySec = replayPanel(ctl, { run, showLog });

  // PX4 console.
  const px4In = textInput({ placeholder: 'PX4 console command (e.g. commander status)' });
  const px4 = (cmd) => ctl.req('sim.px4', { cmd }).then(() => toast(`PX4: ${cmd}`)).catch((e) => toast(e.message, 'error'));
  px4In.input.addEventListener('keydown', (e) => { if (e.key === 'Enter' && px4In.get().trim()) { px4(px4In.get().trim()); px4In.input.value = ''; } });

  return h('div', {},
    section('Run', backendSel, backendHint, backendNote, h('div', { class: 'row wrap' }, startBtn, stopBtn, restartBtn, buildBtn), busy, procBox, pursuerSec),
    section('Configuration (saved automatically; applies on next start)',
      h('p', { class: 'muted' }, 'The MPC and gimbal settings are live in the Control tab.'), form),
    studiesSec,
    replaySec,
    section('Logs', h('div', { class: 'row' }, logSel, button('Clear', () => { lines = []; renderLog(); }, 'small')), logFilter, logView),
    section('PX4 console', px4In,
      h('div', { class: 'row wrap' }, ...['commander status', 'commander arm', 'commander takeoff', 'commander land', 'commander disarm', 'listener vehicle_status']
        .map((c) => button(c, () => px4(c), 'small')))));
}
