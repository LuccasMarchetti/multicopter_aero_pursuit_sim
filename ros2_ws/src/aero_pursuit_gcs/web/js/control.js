// Control tab: the pursuit MPC (control/mpc.cpp, problem in acados/pursuit_ocp.py), one card per
// part of the controller, each with only its own parameters:
//   Prediction horizon            N, dt (a new pair compiles a solver in the background)
//   Translational MPC             standoff reference geometry, weights and acceleration bounds
//   Heading (yaw)                 the heading formulation and its weights (none when not controlled)
//   Visibility constraint         field of view (only with the formulations that use it)
//   Other MPC parameters          any extra mpc.params.<name> (free list)
// then the pursuit node, supervisor timing, PX4 inner loop and the camera gimbal.
// Every MPC number is the ROS parameter mpc.params.<name> -> Mpc::setParameter(name, value).

import { h, section, hint, button, toast } from './ui.js';
import { ParamBinding } from './params.js';

const NODE = '/pursuit_controller_node';
const OFFBOARD = '/offboard_control';
const GIMBAL = '/gimbal';
const PREFIX = 'mpc.params.';

// Heading formulations (mpc.params.yaw_mode), with the work each one comes from.
const YAW_MODES = [
  { v: 0, name: 'No heading control', ref: '',
    text: 'The MPC only plans the translation; the node keeps the heading it had when the chase started.' },
  { v: 1, name: 'LOS heading tracking', ref: '',
    text: 'The heading follows the line-of-sight bearing atan2(p_y, p_x) of the previous plan, given to the ' +
      'solver as a reference (the problem stays a QP). Translation and heading are decoupled.' },
  { v: 2, name: 'LOS heading tracking + visibility constraint', ref: 'Allibert et al., 2010',
    text: 'As LOS heading tracking, plus a (soft) constraint keeping the target inside the horizontal field of view.' },
  { v: 3, name: 'Perception-aware (PAMPC)', ref: 'Falanga et al., 2018',
    text: 'The relative heading is optimized inside the solver: cost on cos ψ − p_x/ρ and sin ψ − p_y/ρ ' +
      '(the 1 − cos δ metric of Lee et al., 2010). The MPC may also move sideways to frame the target; ' +
      'a larger q_psi weighs framing more.' },
  { v: 4, name: 'Perception-aware + visibility constraint', ref: 'Falanga et al., 2018; Allibert et al., 2010',
    text: 'Perception-aware cost plus the visibility constraint.' },
];
const REF_GEOMETRY = [
  { v: 1, name: 'Planar standoff (horizontal line of sight)',
    text: 'Standoff point at target_distance behind the target along the horizontal line of sight, at the target altitude (p_rel,z → 0).' },
  { v: 0, name: '3D line-of-sight standoff',
    text: 'Standoff along the 3D line of sight: the altitude difference is kept as it is.' },
];

// [name, label, unit, help, default, [min, max], integer]
const P = (name, label, unit, help, dflt, range, integer = false) => ({ name, label, unit, help, dflt, range, integer });
const GROUP = {
  horizon: [
    P('N', 'steps N', '', 'Prediction steps. A new (N, dt) pair compiles a solver in the background (~3-5 s); the current one keeps running.', 20, [5, 80], true),
    P('dt', 'step dt', 's', 'Time per step; horizon = N · dt.', 0.1, [0.01, 0.3]),
  ],
  translation: [
    P('q_px', 'q_px', '', 'Weight of the x error to the standoff point.', 10, [0, 50]),
    P('q_py', 'q_py', '', 'Weight of the y error to the standoff point.', 10, [0, 50]),
    P('q_pz', 'q_pz', '', 'Weight of the z error to the standoff point.', 10, [0, 50]),
    P('q_vx', 'q_vx', '', 'Weight of the relative velocity in x (match the target velocity).', 1, [0, 10]),
    P('q_vy', 'q_vy', '', 'Weight of the relative velocity in y.', 1, [0, 10]),
    P('q_vz', 'q_vz', '', 'Weight of the relative velocity in z.', 1, [0, 10]),
    P('r_ax', 'r_ax', '', 'Weight of the pursuer acceleration in x (effort).', 0.5, [0, 5]),
    P('r_ay', 'r_ay', '', 'Weight of the pursuer acceleration in y.', 0.5, [0, 5]),
    P('r_az', 'r_az', '', 'Weight of the pursuer acceleration in z.', 0.5, [0, 5]),
    P('q_f', 'q_f', '', 'Terminal cost factor: W_N = q_f · diag(q_p, q_v).', 5, [0, 20]),
    P('a_max_xy', 'a_max_xy', 'm/s²', 'Bound on |a_x| and |a_y|.', 3, [0.5, 8]),
    P('a_max_z', 'a_max_z', 'm/s²', 'Bound on |a_z|.', 2, [0.5, 6]),
  ],
  heading: [
    P('q_psi', 'q_psi', '', 'Weight of the heading error (to the LOS bearing, or the framing cost in PAMPC).', 5, [0, 100]),
    P('r_r', 'r_r', '', 'Weight of the yaw rate (effort).', 0.5, [0, 5]),
    P('r_max', 'r_max', 'rad/s', 'Bound on the yaw rate |r|.', 1.2, [0.1, 3]),
  ],
  visibility: [
    P('fov_deg', 'field of view', '°', 'Horizontal field of view of the camera.', 69, [30, 120]),
    P('fov_margin_deg', 'margin', '°', 'Kept free on each side of the image (usable half angle = fov/2 − margin).', 8, [0, 20]),
    P('w_fov', 'w_fov', '', 'Penalty on the slack of the soft constraint (how hard it is enforced).', 200, [0, 2000]),
  ],
};
const KNOWN = new Set(['yaw_mode', 'ref_horizontal', ...Object.values(GROUP).flat().map((p) => p.name)]);

const RANGE_KEY = 'aero.mpc.range.';
function loadRange(name, fallback) {
  try {
    const r = JSON.parse(localStorage.getItem(RANGE_KEY + name) || 'null');
    if (Array.isArray(r) && r.length === 2 && r.every(Number.isFinite) && r[0] < r[1]) return r;
  } catch { /* storage unavailable */ }
  return fallback;
}
function saveRange(name, r) {
  try { localStorage.setItem(RANGE_KEY + name, JSON.stringify(r)); } catch { /* ignore */ }
}

export function controlPanel(ctl) {
  const binding = new ParamBinding(ctl, NODE);
  const offboard = new ParamBinding(ctl, OFFBOARD);
  const gimbal = new ParamBinding(ctl, GIMBAL);
  const values = new Map();   // short name -> value the node has
  const mpcBox = h('div', {});
  const restBox = h('div', {});

  async function send(name, value) {
    try {
      await ctl.req('param.set', { node: NODE, name: PREFIX + name, type: 'double', value: Number(value) });
      values.set(name, Number(value));
      return true;
    } catch (e) {
      toast(`${name}: ${e.message}`, 'error', 6000);
      return false;
    }
  }

  async function unset(name) {
    try {
      await ctl.req('param.set', { node: NODE, name: PREFIX + name, type: 'unset', value: null });
      await ctl.req('param.forget', { node: NODE, name: PREFIX + name });
      values.delete(name);
      renderMpc();
    } catch (e) {
      toast(`${name}: ${e.message}`, 'error', 6000);
    }
  }

  /// One parameter: label | min | slider | max | value (Enter sends) | ↺ default. The slider sends
  /// when released (not while dragging: a new N/dt compiles a solver).
  function paramRow(p, { free = false } = {}) {
    const value = values.has(p.name) ? values.get(p.name) : p.dflt;
    let [lo, hi] = loadRange(p.name, p.range || (value > 0 ? [0, Math.max(1, value * 4)] : value < 0 ? [value * 4, 0] : [0, 1]));
    const minIn = h('input', { type: 'number', step: 'any', value: String(lo), class: 'num', title: 'slider minimum' });
    const maxIn = h('input', { type: 'number', step: 'any', value: String(hi), class: 'num', title: 'slider maximum' });
    const slider = h('input', { type: 'range' });
    const valueIn = h('input', { type: 'number', step: 'any', value: String(value), class: 'num', title: 'value: Enter sends' });
    const applyRange = () => {
      slider.min = String(lo);
      slider.max = String(hi);
      slider.step = p.integer ? '1' : String((hi - lo) / 200);
      slider.value = String(Math.min(hi, Math.max(lo, Number(valueIn.value))));
    };
    const setRange = () => {
      const a = Number(minIn.value), b = Number(maxIn.value);
      if (!Number.isFinite(a) || !Number.isFinite(b) || a >= b) {
        toast(`${p.name}: minimum < maximum`, 'error');
        minIn.value = String(lo); maxIn.value = String(hi);
        return;
      }
      [lo, hi] = [a, b];
      saveRange(p.name, [lo, hi]);
      applyRange();
    };
    minIn.addEventListener('change', setRange);
    maxIn.addEventListener('change', setRange);
    slider.addEventListener('input', () => { valueIn.value = slider.value; });
    slider.addEventListener('change', () => send(p.name, slider.value));
    valueIn.addEventListener('keydown', (e) => {
      if (e.key !== 'Enter') return;
      const v = Number(valueIn.value);
      if (valueIn.value === '' || !Number.isFinite(v)) { toast(`${p.name}: numeric value`, 'error'); return; }
      if (v < lo) { lo = v; minIn.value = String(v); saveRange(p.name, [lo, hi]); }
      if (v > hi) { hi = v; maxIn.value = String(v); saveRange(p.name, [lo, hi]); }
      applyRange();
      send(p.name, v);
    });
    applyRange();
    const reset = free
      ? button('remove', () => unset(p.name), 'small danger', 'Remove this parameter')
      : button('↺', async () => { if (await send(p.name, p.dflt)) renderMpc(); }, 'small', `Back to the default (${p.dflt})`);
    return h('div', { class: 'mpc-param' },
      h('div', { class: 'mpc-param-head' }, h('b', {}, p.label), p.unit ? h('span', { class: 'muted' }, ` [${p.unit}]`) : null,
        p.label !== p.name ? h('code', { class: 'muted' }, ` ${p.name}`) : null,
        p.help ? h('small', { class: 'muted' }, p.help) : null),
      h('div', { class: 'row wrap' }, minIn, slider, maxIn, valueIn, reset));
  }

  /// Radio-like choice of a numeric parameter (formulation / geometry), with the description.
  function choice(name, options, current) {
    const box = h('div', { class: 'mpc-choice' });
    for (const o of options) {
      const input = h('input', { type: 'radio', name: `mpc-${name}`, value: String(o.v) });
      input.checked = o.v === current;
      input.addEventListener('change', async () => { if (await send(name, o.v)) renderMpc(); });
      box.append(h('label', { class: 'mpc-option' + (o.v === current ? ' on' : '') }, input,
        h('div', {}, h('b', {}, o.name), o.ref ? h('span', { class: 'muted' }, ` (${o.ref})`) : null,
          h('small', {}, o.text))));
    }
    return box;
  }

  function renderMpc() {
    mpcBox.innerHTML = '';
    const yaw = Math.round(values.get('yaw_mode') ?? 0);
    const horizontal = (values.get('ref_horizontal') ?? 1) > 0.5 ? 1 : 0;
    const visibility = yaw === 2 || yaw === 4;
    const free = [...values.keys()].filter((n) => !KNOWN.has(n)).sort();
    const freeBox = h('div', {});
    const nameIn = h('input', { type: 'text', placeholder: 'name', size: 16 });
    const valueIn = h('input', { type: 'number', step: 'any', value: '1', size: 8 });
    const add = async () => {
      const name = nameIn.value.trim();
      if (!/^[A-Za-z_][A-Za-z0-9_.]*$/.test(name)) { toast('Invalid name (letters, digits, _ and .)', 'error'); return; }
      if (KNOWN.has(name)) { toast(`${name} already has its own row above`, 'error'); return; }
      if (valueIn.value === '' || Number.isNaN(Number(valueIn.value))) { toast('A numeric value is required', 'error'); return; }
      if (await send(name, valueIn.value)) renderMpc();
    };
    for (const n of free) freeBox.append(paramRow(P(n, n, '', '', values.get(n), null), { free: true }));
    mpcBox.append(
      section('MPC: prediction horizon',
        hint('Relative-state linear MPC (x = [p_rel, v_rel], u = pursuer acceleration), solved by acados (SQP-RTI + HPIPM) every cycle.'),
        ...GROUP.horizon.map((p) => paramRow(p))),
      section('MPC: translation (standoff tracking)',
        h('div', { class: 'mpc-sub' }, 'Reference geometry'), choice('ref_horizontal', REF_GEOMETRY, horizontal),
        h('div', { class: 'mpc-sub' }, 'Weights and bounds'),
        hint('Position error to the standoff point (q_p), relative velocity (q_v), acceleration effort (r_a). The standoff distance is the pursuit node\'s target_distance (below).'),
        ...GROUP.translation.map((p) => paramRow(p))),
      section('MPC: heading (yaw)',
        choice('yaw_mode', YAW_MODES, yaw),
        yaw === 0 ? hint('No heading parameters: the MPC does not command the heading.')
          : h('div', {}, h('div', { class: 'mpc-sub' }, 'Heading weights and bounds'), ...GROUP.heading.map((p) => paramRow(p)))),
      visibility ? section('MPC: visibility constraint (field of view)',
        hint('|lateral| ≤ tan(fov/2 − margin) · front for the target in the camera frame, at every stage; soft (slack penalized by w_fov), linearized by SQP-RTI around the previous plan.'),
        ...GROUP.visibility.map((p) => paramRow(p))) : null,
      section('MPC: other parameters',
        hint('Any other mpc.params.<name> the MPC reads (param("name", default) in src/control/mpc.cpp).'),
        free.length ? freeBox : hint('None.'),
        h('div', { class: 'row wrap' }, nameIn, valueIn, button('Add', add, 'small primary'), button('Reload', render, 'small'))));
  }

  async function render() {
    try {
      await binding.load();
    } catch (e) {
      mpcBox.innerHTML = '';
      mpcBox.append(h('div', { class: 'muted' }, `${NODE}: ${e.message}`));
      return;
    }
    values.clear();
    for (const [name, p] of binding.params) {
      if (name.startsWith(PREFIX)) values.set(name.slice(PREFIX.length), Number(p.value));
    }
    renderMpc();
    binding.controls.clear();
    restBox.innerHTML = '';
    const has = (list) => list.filter((n) => binding.params.has(n));
    restBox.append(
      section('Pursuit node',
        hint('target_distance: standoff to keep [m]. max_vel: safety saturation of the velocity sent to the PX4 [m/s].'),
        ...has(['target_distance', 'max_vel']).map((n) => binding.control(n))),
      section('Supervisor (timing)',
        hint('Without a fresh estimate for coast_timeout → COASTING; lost_timeout → LOST (chase off). bearing_timeout: a recent box without depth → APPROACH.'),
        ...has(['coast_timeout', 'lost_timeout', 'bearing_timeout', 'velocity_timeout']).map((n) => binding.control(n))));
    binding.refreshOptions();
    try {
      await offboard.load();
      if (offboard.params.has('acceleration_feedforward')) {
        offboard.controls.clear();
        restBox.append(section('PX4 inner loop (offboard_control)',
          hint('acceleration_feedforward off: acceleration = NaN in the TrajectorySetpoint (no feedforward).'),
          offboard.control('acceleration_feedforward')));
        offboard.refreshOptions();
      }
    } catch { /* offboard_control not running */ }
    try {
      await gimbal.load();
      if (!gimbal.params.has('auto_pitch')) throw new Error('no /gimbal');
      gimbal.controls.clear();
      restBox.append(section('Camera gimbal (pitch)',
        hint('auto_pitch off (default): fixed camera at pitch_deg (+ = up; 0 = straight ahead). ' +
          'On: geometric law, points the camera at the target compensating the body pitch.'),
        ...['auto_pitch', 'pitch_deg', 'max_rate_deg'].filter((n) => gimbal.params.has(n)).map((n) => gimbal.control(n))));
      gimbal.refreshOptions();
    } catch {
      restBox.append(section('Camera gimbal (pitch)', hint('The /gimbal node is not running (simulation stopped or remote pursuer).')));
    }
  }

  ctl.on('open', render);
  return { el: h('div', {}, mpcBox, restBox), onShow: render };
}
