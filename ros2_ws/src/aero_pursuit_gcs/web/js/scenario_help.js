// Help window of the Scenarios tab: what each field does, the kinds of path legs, the shape
// generator, the CSV/Excel format and the commented YAML reference (scenarios/reference.yaml).

import { h, button } from './ui.js';

const TOPICS = [
  ['How a run goes', [
    ['1. Staging', 'The target flies to its start, the pursuer (PX4) to its own. When both are there and at rest, the run starts.'],
    ['2. Running', 'The target flies its path. After "start chase after", the pursuer is enabled and chases it.'],
    ['3. End', 'When the duration is over, the target path ends (duration "until the path ends") or the pursuer gets within the capture distance. With repetitions > 1 it goes back to 1.'],
    ['Recording', 'Each run is a session in datasets/ (rosbag + session_info.yaml), used by the studies.'],
  ]],
  ['Scenario', [
    ['duration', 'Seconds of chase per run. "Until the path ends": stops when the target finishes its path (a path that repeats forever needs a duration).'],
    ['repetitions', 'Runs in a row, each from the start positions (to average results).'],
    ['target speed', '"As written in the path": the speeds you set in each leg. "Fixed": the whole path is flown at this speed (the shape and stops stay; the clock is scaled). The GCS slider changes it live.'],
    ['end on capture', 'The run ends early when the pursuer is closer than this to the target.'],
  ]],
  ['Target', [
    ['drone', 'X4 is already in the world; other names are spawned. Model: x3, x4, x5, tiltrotor.'],
    ['start', 'Where the target waits before the run. Its path begins where the first leg begins (it flies there in a straight line if they differ).'],
    ['nose', '"Along the motion": points where it flies. "Hold": keeps its heading. A number: fixed heading in degrees (0 = east, 90 = north).'],
    ['repeat the path', 'How many times the whole list of legs is flown (forever = until the duration ends).'],
    ['max speed', 'Limit of the velocity command sent to the target.'],
  ]],
  ['Path legs', [
    ['Path', 'A trajectory: points you click on the map, a table imported from CSV/Excel (with or without time), or a shape (circle, figure 8...). "laps" repeats only this leg.'],
    ['Go to', 'Straight line to a point at the given speed.'],
    ['Hover', 'Stays still for some seconds (where it is, or at a given point).'],
    ['Reactive', 'The target reacts to the pursuer instead of following a fixed path: flee (runs away with lateral manoeuvres), random (random walk inside the arena), orbit (circles the pursuer). Its "random seed" makes the manoeuvres repeatable (same number = same run).'],
    ['speed of a path', 'The target follows the curve at constant speed (a point can change it until the next point; "stop here" waits). A timed path (t, x, y, z) has the speed implied by the times. In curves tighter than the drone can fly (acceleration ≈ 3 m/s²) it lags and cuts inside a little.'],
  ]],
  ['Shapes (generator)', [
    ['circle', 'center, radius, start angle, direction.'],
    ['polygon', 'Regular polygon: center, radius (to the corners), sides, rotation.'],
    ['figure 8', 'size x = half width, y = half height, z = height oscillation (twice per lap).'],
    ['lissajous', 'x = sx·sin(fx·u + φx), y = sy·sin(fy·u + φy), z likewise: frequencies 1, 2 = a lying 8; 3, 2 = a knot.'],
    ['helix', 'Circles while climbing "climb" meters in "turns" laps, then comes back down the same way.'],
    ['random loop', 'A smooth closed loop through "random points" inside the box center ± size; "random seed" picks the route.'],
    ['Convert to points', 'Turns any shape into ~16 editable points (smooth curve), to adjust it by hand.'],
  ]],
  ['CSV / Excel', [
    ['columns', 'One row per point: x, y, z in meters (x east, y north, z up, world frame) and, optionally, t in seconds. Headers like t/time, x, y, z/alt are recognized; you can also choose the columns.'],
    ['with time', 'The target is at each point at its time (variable speed). Times must increase; the first one becomes 0.'],
    ['without time', 'The points are flown at the path speed (constant).'],
    ['Excel', 'Open the .xlsx directly (first sheet), or copy the cells and paste them in the import box.'],
    ['dense logs', '"Use one row in N" keeps every N-th row (a few hundred points are plenty).'],
  ]],
  ['Pursuer', [
    ['start / heading', 'Where the PX4 waits before the run, and its heading (ignored with "turn to the target").'],
    ['turn to the target', 'Before starting, it turns until the target is in front of the camera (within the tolerance), and keeps facing it until the chase starts.'],
    ['detection', 'vision: the camera tracker finds the target. omniscient: the exact target position from the simulator (to test the controller alone).'],
    ['first box', 'auto: the tracker starts from a box computed from the simulator (repeatable). manual: you draw the box on the video.'],
    ['restart if lost', 'With auto: if the tracker loses the target, it is restarted from the simulator box.'],
    ['start chase after', 'Seconds after the run starts; optionally only once the tracker has the target.'],
    ['patrol', 'A path the pursuer flies before the chase starts (same legs as the target). Without it, it holds its start position.'],
  ]],
  ['Coordinates', [
    ['world', 'ENU, meters: x east, y north, z up; 0, 0 is the middle of the world. Angles in degrees, 0 = east, counter-clockwise.'],
    ['relative', 'In "Advanced" of a path: the points are offsets from where the target is when the leg starts.'],
  ]],
];

export function openScenarioHelp(ctl) {
  const body = h('div', { class: 'help-body' });
  const tabs = h('div', { class: 'row wrap help-tabs' });
  const show = (i) => {
    body.innerHTML = '';
    [...tabs.children].forEach((b, k) => b.classList.toggle('active', k === i));
    if (i < TOPICS.length) {
      body.append(h('dl', {}, ...TOPICS[i][1].flatMap(([k, v]) => [h('dt', {}, k), h('dd', {}, v)])));
      return;
    }
    const pre = h('pre', { class: 'help-yaml' }, 'loading scenarios/reference.yaml ...');
    body.append(h('p', { class: 'muted' }, 'Every field a scenario file can have, with its default (scenarios/reference.yaml; the YAML button of the editor shows the current scenario this way).'), pre);
    ctl.req('scenario.read', { file: 'reference.yaml' }).then((r) => { pre.textContent = r.yaml; })
      .catch((e) => { pre.textContent = e.message; });
  };
  TOPICS.forEach(([t], i) => tabs.append(button(t, () => show(i), 'small')));
  tabs.append(button('YAML reference', () => show(TOPICS.length), 'small'));
  const el = h('div', { class: 'pe-overlay help-overlay' },
    h('div', { class: 'pe-top' }, h('b', {}, 'Scenarios: help'), h('span', { class: 'spacer' }),
      button('Close', () => el.remove(), 'small')),
    tabs, body);
  document.body.append(el);
  show(0);
}
