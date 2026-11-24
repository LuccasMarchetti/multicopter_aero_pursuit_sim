// Help tab: quick reference. The full manual is in docs/MANUAL_GCS.md.

import { h, section } from './ui.js';

const li = (...items) => h('ul', {}, ...items.map((t) => h('li', {}, t)));

export function helpPanel() {
  return h('div', {},
    section('Basic workflow',
      h('ol', {},
        h('li', {}, 'Simulation → ▶ Start (wait for PX4 to arm and take off; "Pursuit" badge on top).'),
        h('li', {}, 'Pursuit → select the target by dragging over the video (or enable omniscient mode).'),
        h('li', {}, 'Move the target: Joystick (sending on) + Drones to choose the vehicle, or run a scenario.'),
        h('li', {}, 'Plots → follow errors and metrics; Scenarios for repeatable experiments.'))),
    section('Tabs',
      li('Simulation: start/stop/restart, launch options, logs and PX4 console.',
        'Pursuit: state, enable, target, distance and live vision.',
        'Drones: which drone the joystick commands, turn targets on/off, drones in the world.',
        'Joystick: browser gamepad or keyboard → /joy.',
        'Scenarios: the target path (map editor, shapes, CSV/Excel), run, stop and unload.',
        'Layout: drag windows by the title bar, resize by the edges (touching windows share the edge: the neighbour shrinks or fills the space), ⧉ on a tab opens it in its own window (Layout menu).',
        'Plots, Parameters, Video, Topics, Network: analysis and fine tuning.')),
    section('Keyboard (Joystick tab → "Keyboard")',
      li('W/S forward/back · A/D left/right', 'R/F up/down · Q/E rotate (yaw)',
        '[ ] , . PgUp/PgDn switch drone (X4 → X3 → Tiltrotor → X5 → pursuer)', 'Z/X decrease/increase speed',
        'Esc or right click on the video: stops tracking')),
    section('Phone / tablet (🎮 in the header)',
      li('Left stick: up/down and yaw · right stick: forward/back and sideways.',
        '◀ ▶ switch the commanded drone; speed −/+ (hold) change the speed; × closes.')),
    section('Common problems',
      li('Target is not following the scenario: release the sticks / turn off joystick sending for the selected drone.',
        'Evader policy only works with "Training arena" on.',
        'Scenario drones stay in the world: Scenarios → ⏏ Unload.')),
    h('p', { class: 'muted' }, 'Full manual: docs/MANUAL_GCS.md in the repository.'));
}
