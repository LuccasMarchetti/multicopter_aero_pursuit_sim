// Simulation tab -> session replay from the rosbag of a recorded session (replay.py):
// "no simulador" rebuilds the world in Gazebo from the recorded poses (video, free camera, 3D);
// "só a gravação" plays the bag alone (3D, plots, status), also with Isaac Sim.

import { h, button, select, toggle, toast, fmt, section } from './ui.js';

const RATES = [['0.25', '0,25×'], ['0.5', '0,5×'], ['1', '1×'], ['2', '2×'], ['4', '4×']];

export function replayPanel(ctl, { run, showLog }) {
  const sessionSel = select({ label: 'Recorded session (datasets/)', options: [] });
  const rateSel = select({ label: 'Speed', options: RATES, value: '1',
    onChange: (v) => rp.active && ctl.req('replay.cmd', { cmd: 'rate', value: Number(v) }).catch((e) => toast(e.message, 'error')) });
  const loopToggle = toggle({ label: 'Loop at the end' });
  const seek = h('input', { type: 'range', min: 0, max: 1, step: 0.1, value: 0, style: 'width:100%' });
  const info = h('div', { class: 'muted' }, 'stopped');
  let rp = { active: false };
  let seeking = false;
  seek.addEventListener('input', () => { seeking = true; });
  seek.addEventListener('change', () => {
    seeking = false;
    ctl.req('replay.cmd', { cmd: 'seek', value: Number(seek.value) }).catch((e) => toast(e.message, 'error'));
  });
  const refresh = () => ctl.req('replay.list').then((l) => sessionSel.setOptions(
    l.map((x) => [x.path, `${x.path} (${fmt(x.duration, 0)} s${x.video ? ', with video' : ''})`]))).catch(() => {});
  const pauseBtn = button('⏸', () => ctl.req('replay.cmd', { cmd: rp.paused ? 'resume' : 'pause' }).catch((e) => toast(e.message, 'error')), 'small');
  const start = (world) => {
    showLog('replay');
    run('replay.start', { session: sessionSel.get(), rate: Number(rateSel.get()), loop: !!loopToggle.get(), world });
  };
  const worldBtn = button('▶ In the simulator', () => start(true), 'primary',
    'restarts Gazebo without pursuer or PX4 and rebuilds the world from the recorded poses');
  ctl.on('replay.status', (st) => {
    rp = st;
    pauseBtn.textContent = st.paused ? '▶' : '⏸';
    seek.max = st.duration || 1;
    if (!seeking) seek.value = st.position || 0;
    if (st.active && st.session && document.activeElement !== sessionSel.input) sessionSel.set(st.session);
    const mode = st.world ? 'in the simulator' : 'recording only';
    info.textContent = st.active
      ? `${st.session} | ${mode} | ${fmt(st.position, 1)} / ${fmt(st.duration, 1)} s | ${st.rate}×${st.paused ? ' | paused' : ''}${st.video ? '' : ' | no video (camera not recorded)'}`
      : (st.phase || 'stopped');
  });
  ctl.on('sim.status', (s) => {
    worldBtn.disabled = s.backend !== 'gazebo';
    worldBtn.title = s.backend !== 'gazebo' ? 'Gazebo only (with Isaac, use "recording only")'
      : 'restarts Gazebo without pursuer or PX4 and rebuilds the world from the recorded poses';
  });
  ctl.on('open', () => setTimeout(refresh, 700));

  return section('Session replay',
    h('p', { class: 'muted' }, 'In the simulator: Gazebo restarts without the pursuer nodes and without PX4, and every drone (and the gimbal) is moved to its recorded pose each frame. The scene is rendered again exactly as it happened: pursuer camera video, free camera and 3D; plots and status come from the recording.'),
    h('p', { class: 'muted' }, 'Recording only: plays the rosbag without a simulator (3D, plots and status); also with Isaac Sim. In both, commands to the vehicles are not replayed.'),
    sessionSel, h('div', { class: 'row wrap' }, rateSel, loopToggle),
    h('div', { class: 'row wrap' }, worldBtn, button('▶ Recording only', () => start(false)),
      pauseBtn, button('■ Stop', () => run('replay.stop'), 'danger'), button('↻', refresh, 'small')),
    seek, info);
}
