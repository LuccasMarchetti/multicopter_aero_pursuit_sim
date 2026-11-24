// Drones tab: which vehicle the joystick/keyboard commands, on/off of the target drones,
// Gazebo world control.

import { h, button, section, stat, hint, toggle, callService, publish } from './ui.js';
import { quickParams } from './pursuit.js';

// Same order as drone_joy_teleop's `target_vehicles` (+ pursuer at the end).
const TARGETS = ['X4', 'X3', 'Tiltrotor', 'X5'];

export function dronesPanel(ctl, state) {
  const active = stat('Commanded vehicle');
  const select = (name) => publish(ctl, '/drone_joy_teleop/select', 'std_msgs/msg/String', { data: name });
  const vehicleButtons = {};
  for (const name of [...TARGETS, 'chaser']) {
    vehicleButtons[name] = button(name === 'chaser' ? 'Pursuer' : name, () => select(name), 'small');
  }
  ctl.on('tel', (m) => {
    if (!m.teleop) return;
    const text = m.teleop.data || '';
    const name = text.split(' ')[0];
    active.set(text.replace('chaser', ('pursuer')), 'good');
    for (const [n, b] of Object.entries(vehicleButtons)) b.classList.toggle('active', n === name);
  });

  // Which base-world drones stay in the simulation (scenario_runner ~/set_drones).
  const KEY = 'gcs.drones.present';
  let wanted = TARGETS;
  try { wanted = JSON.parse(localStorage.getItem(KEY)) || TARGETS; } catch { /* default: all */ }
  const presence = {};
  const checks = TARGETS.map((name) => {
    const el = toggle({ label: name, value: wanted.includes(name), onChange: (v) => {
      wanted = v ? [...new Set([...wanted, name])] : wanted.filter((n) => n !== name);
      try { localStorage.setItem(KEY, JSON.stringify(wanted)); } catch { /* private mode */ }
    } });
    presence[name] = h('span', { class: 'muted' }, '');
    return h('div', { class: 'row' }, el, presence[name]);
  });
  ctl.on('tel', (m) => {
    if (!m.tf) return;
    for (const name of TARGETS) presence[name].textContent = m.tf[name] ? ('in the world') : ('absent');
  });
  const applyPresence = () => callService(ctl, '/scenario_runner/set_drones', 'aero_pursuit/srv/SetDrones',
    { present: wanted }, ('Drones updated'));

  // Live speed of the pursued drone along its scenario plan [m/s] (scenario_runner target_speed).
  const speedBox = h('div', {});
  const qs = quickParams(ctl, '/scenario_runner', [['', ['target_speed']]], speedBox);
  ctl.on('open', () => qs.load());
  let wasRunning = false;
  ctl.on('sim.status', (s) => {
    const running = !!(s.processes.container && s.processes.container.instances.length);
    if (running && (!wasRunning || !qs.loaded)) setTimeout(() => qs.load(), wasRunning ? 0 : 4000);
    wasRunning = running;
  });

  const world = () => (state.sim && state.sim.cfg && state.sim.cfg.world) || state.worldFrame || 'lawn';
  const worldCtl = (req, txt) => callService(ctl, `/world/${world()}/control`, 'ros_gz_interfaces/srv/ControlWorld',
    { world_control: req }, txt);

  return h('div', {},
    section('Teleoperation (joystick / keyboard)',
      hint('Chooses which drone receives the Joystick tab commands. Keyboard: [ ] , . or PageUp/PageDown cycle through the vehicles. ' +
        'When you take over the pursuer, automatic pursuit pauses and resumes when you switch vehicles.'),
      active,
      h('div', { class: 'row wrap' },
        button('◀', () => select('prev'), 'small', 'Previous vehicle (LB)'),
        ...Object.values(vehicleButtons),
        button('▶', () => select('next'), 'small', 'Next vehicle (RB)'))),
    section('Target drones (Gazebo velocity controller)',
      hint('Turned on automatically at start. When off, the drone ignores the joystick, scenarios and evader. ' +
        '"Stop" zeroes the velocity (the drone hovers).'),
      ...TARGETS.map((ns) => h('div', { class: 'row wrap' },
        h('span', { style: { width: '72px' } }, ns),
        button('Turn on', () => publish(ctl, `/${ns}/enable`, 'std_msgs/msg/Bool', { data: true }, `${ns} ${('on')}`), 'small'),
        button('Turn off', () => publish(ctl, `/${ns}/enable`, 'std_msgs/msg/Bool', { data: false }, `${ns} ${('off')}`), 'small'),
        button('Stop', () => publish(ctl, `/${ns}/gazebo/command/twist`, 'geometry_msgs/msg/Twist', {}, `${ns}: ${('zero velocity')}`), 'small')))),
    section('Target drone speed (live)',
      hint('Speed of the pursued drone along its scenario plan [m/s] (0 = the plan\'s own speeds); the other ' +
        'targets keep pace (same paths). Applies at once. Joystick drones: Z/X in the Joystick tab.'),
      speedBox),
    section('Drones in the simulation',
      hint('Check the target drones that take part. "Apply" removes the unchecked ones and inserts the ' +
        'checked missing ones (velocity controller on). Drones used by the loaded scenario are not removed.'),
      ...checks,
      h('div', { class: 'row wrap' }, button('Apply', applyPresence, 'primary'))),
    section('World (Gazebo)',
      hint('Reset returns all models to their initial pose; scenario-inserted drones stay in the world until "Unload" (Scenarios tab).'),
      h('div', { class: 'row wrap' },
        button('⏸ Pause', () => worldCtl({ pause: true }, 'World paused')),
        button('▶ Resume', () => worldCtl({ pause: false }, 'World running')),
        button('⏭ Step', () => worldCtl({ pause: true, multi_step: 1 }, 'Step')),
        button('⟲ Reset', () => worldCtl({ reset: { all: true } }, 'World reset'), 'danger'))));
}
