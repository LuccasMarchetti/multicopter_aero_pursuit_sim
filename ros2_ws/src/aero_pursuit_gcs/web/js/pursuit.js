// Pursuit tab: supervisor state, enable/disable, target selection,
// omniscient mode and quick controller and vision settings.
// (Target drones, world and training live in the Drones tab.)

import { h, button, select, section, stat, fmt, toast, hint, callService } from './ui.js';
import { ParamBinding } from './params.js';

const MODE_CLASS = { TRACKING: 'good', APPROACH: 'ok', COASTING: 'ok', LOST: 'bad', DISABLED: 'na', IDLE: 'na' };
const NAV_STATES = { 0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'AUTO_MISSION', 4: 'AUTO_LOITER', 5: 'AUTO_RTL',
  14: 'OFFBOARD', 17: 'AUTO_TAKEOFF', 18: 'AUTO_LAND' };

const CONTROLLER_PARAMS = ['target_distance', 'max_vel'];
const VISION_PARAMS = ['tracker_model', 'dnn_target', 'tracking_score_threshold', 'min_consecutive_frames', 'depth_fallback_enabled',
  'max_processing_rate_hz', 'verifier.enabled', 'verifier.depth_check', 'verifier.scale_check',
  'verifier.appearance_check', 'verifier.appearance_min', 'verifier.score_check', 'verifier.fail_frames',
  'verifier.box_check', 'verifier.max_box_fraction', 'verifier.hold_time', 'verifier.recover_frames'];

const call = callService;

/// `notes(values)` -> {group title: text}: explanation shown under a group (e.g. why its
/// controls are hidden for the current technique).
export function quickParams(ctl, node, groups, box, notes) {
  const binding = new ParamBinding(ctl, node);
  binding.conditional = true;
  const noteEls = {};
  if (notes) {
    binding.onChange((vals) => {
      const texts = notes(vals);
      for (const [title, el] of Object.entries(noteEls)) {
        el.textContent = texts[title] || '';
        el.style.display = texts[title] ? '' : 'none';
      }
    });
  }
  let loaded = false;
  const load = async () => {
    try {
      await binding.load();
      binding.controls.clear();
      box.innerHTML = '';
      for (const [title, names] of groups) {
        if (title) box.append(h('h4', {}, title));
        noteEls[title] = hint('');
        box.append(noteEls[title]);
        for (const n of names) if (binding.params.has(n)) box.append(binding.control(n));
      }
      binding.refreshOptions();
      loaded = true;
    } catch (e) {
      loaded = false;
      box.innerHTML = '';
      box.append(h('div', { class: 'muted' }, `${node}: ${e.message}`));
    }
  };
  return { load, get loaded() { return loaded; } };
}

export function pursuitPanel(ctl, state, video) {
  const mode = stat('Mode');
  const ctrl = stat('Controller');
  const range = stat('Distance');
  const rangeErr = stat('Distance error');
  const age = stat('Target age');
  const speed = stat('Commanded vel.');
  const track = stat('Tracker');
  const px4 = stat('PX4');
  const teleop = h('div', { class: 'muted mono' });

  ctl.on('tel', (m) => {
    if (m.status) {
      const s = m.status;
      mode.set(s.mode_name || '—', MODE_CLASS[s.mode_name] || '');
      ctrl.set(s.controller);
      const known = s.range !== null && s.range !== undefined && !Number.isNaN(s.range);
      if (s.mode_name === 'APPROACH') {
        range.set(known ? `~${fmt(s.range, 1, 'm')} (last measurement; no depth)` : 'unknown (no depth)', 'ok');
      } else if (!known) {
        range.set(s.mode_name === 'LOST' ? '— (target lost)' : '—', s.mode_name === 'LOST' ? 'bad' : '');
      } else {
        range.set(fmt(s.range, 2, 'm'));
      }
      rangeErr.set(fmt(s.range_error, 2, 'm'));
      age.set(s.target_age < 0 ? 'never' : fmt(s.target_age, 2, 's'), s.target_age >= 0 && s.target_age < 0.3 ? 'good' : 'bad');
      speed.set(fmt(s.command_speed, 2, 'm/s'));
    }
    if (m.detection) {
      const d = m.detection;
      const names = ['idle', 'lost', 'confirming', 'confirmed'];
      const ver = d.verifier ? ` | ${('verif.')} ${d.verifier}${Number.isFinite(d.appearance) ? ' ' + fmt(d.appearance, 2) : ''}` : '';
      track.set(`${(names[d.status]) || d.status} | score ${fmt(d.score, 2)} | ${d.has_depth ? fmt(d.depth, 1, 'm') : ('no depth')}${ver}`,
        d.status === 3 ? 'good' : d.status === 2 ? 'ok' : d.status === 1 ? 'bad' : '');
    }
    if (m.px4) {
      const armed = m.px4.arming_state === 2;
      px4.set(`${armed ? ('ARMED') : ('disarmed')} | ${NAV_STATES[m.px4.nav_state] || m.px4.nav_state}`, armed ? 'good' : '');
    }
    if (m.teleop) teleop.textContent = `Teleop: ${m.teleop.data}`;
  });

  const ctrlBox = h('div', {});
  const visionBox = h('div', {});
  // Full tuning per method: Control tab.
  const qc = quickParams(ctl, '/pursuit_controller_node', [
    ['', CONTROLLER_PARAMS]], ctrlBox);
  const qv = quickParams(ctl, '/vision_tracker_node', [['', VISION_PARAMS]], visionBox);
  const speedBox = h('div', {});
  const qs = quickParams(ctl, '/scenario_runner', [['', ['target_speed']]], speedBox);
  const reload = () => { qc.load(); qv.load(); qs.load(); };
  ctl.on('open', reload);
  // Reloads when the container comes up (simulation started by the GCS).
  let wasRunning = false;
  ctl.on('sim.status', (s) => {
    const running = !!(s.processes.container && s.processes.container.instances.length);
    if (running && (!wasRunning || !qc.loaded)) setTimeout(reload, wasRunning ? 0 : 4000);
    wasRunning = running;
  });

  const omniFrame = select({ label: 'Target frame', options: ['X4', 'X3', 'Tiltrotor', 'X5'], value: 'X4' });

  return h('div', {},
    section('State', mode, ctrl, range, rangeErr, age, speed, track, px4, teleop),
    section('Pursuit',
      hint('Enable/Disable switches the control loop (the PX4 must be armed in OFFBOARD). ' +
        'When disabled, offboard_control holds position.'),
      h('div', { class: 'row wrap' },
        button('Enable', () => call(ctl, '/pursuit_controller_node/enable', 'std_srvs/srv/SetBool', { data: true }, 'Pursuit enabled'), 'primary'),
        button('Disable', () => call(ctl, '/pursuit_controller_node/enable', 'std_srvs/srv/SetBool', { data: false }, 'Pursuit disabled'), 'danger'),
        button('↻ parameters', reload, 'small'))),
    section('Target',
      hint('Vision: drag a rectangle over the video to select the target; right click or Esc to stop.'),
      h('div', { class: 'row wrap' },
        button('Stop tracking', () => video.stopTracking(), 'danger')),
      h('h4', {}, 'Target speed (scenario, live)'),
      hint('Speed of the pursued drone along the scenario plan [m/s] (0 = the plan\'s). Same paths; the other targets keep pace.'),
      speedBox,
      h('h4', {}, 'Omniscient mode (position from TF, no vision)'),
      h('div', { class: 'row wrap' }, omniFrame,
        button('Activate', () => ctl.req('omni', { frame: omniFrame.get(), enable: true }).then(() => toast('Omnisciente: ' + omniFrame.get(), 'good')).catch((e) => toast(e.message, 'error'))),
        button('Deactivate', () => ctl.req('omni', { frame: omniFrame.get(), enable: false }).then(() => toast('Omniscient off')).catch((e) => toast(e.message, 'error'))))),
    section('Distance and limit (live)',
      hint('The MPC sends velocity + acceleration (feedforward) + heading to the PX4 velocity loop (TrajectorySetpoint). ' +
        'The MPC weights and limits are in the Control tab.'),
      ctrlBox),
    section('Vision (live)', visionBox));
}
