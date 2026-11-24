// Scenarios tab: files in <repo>/scenarios and the editor of one scenario.
//   Scenario   duration, repetitions, target speed, end on capture
//   Target     the pursued drone: start, nose, and its path (a list of legs: a path drawn on the
//              map / imported from CSV-Excel / a shape, go to, hover, reactive behavior)
//   Pursuer    start, camera alignment, detection, when the chase starts, optional patrol
//   Controller the standoff distance and other ROS parameters applied at each run
// The paths are edited in a map window (path_editor.js); the 3D view shows the predicted paths
// (scenario_tool, the same code the scenario_runner flies). "YAML" switches to the text.

import { h, button, section, toast, debounce, fmt } from './ui.js';
import { openPathEditor, pickPoint, SHAPES } from './path_editor.js';
import { openScenarioHelp } from './scenario_help.js';

const TEMPLATE = () => ({
  name: 'new_scenario', description: '', duration: 90, repetitions: 1,
  target: { start: [5, -2, 1], path: [{ trajectory: { type: 'circle', center: [22, 0, 7], radius: 10, speed: 2.5 }, loops: 0 }] },
  pursuer: { start: [0, 0, 5], detection: 'vision', acquisition: 'auto', start_after: 2 },
  parameters: { '/pursuit_controller_node': { target_distance: 5 } },
});

const BEHAVIORS = {
  flee: ['Flee', 'Runs away from the pursuer with lateral manoeuvres.',
    ['speed', 'period', 'amplitude', 'arena_center', 'arena_radius', 'min_altitude', 'max_altitude', 'seed']],
  random: ['Random walk', 'Wanders smoothly inside the arena, ignoring the pursuer.',
    ['speed', 'arena_center', 'arena_radius', 'min_altitude', 'max_altitude', 'seed']],
  orbit: ['Orbit the pursuer', 'Circles around the pursuer.', ['speed', 'radius', 'phase_deg', 'height_offset']],
};
// [label, kind, help]
const BF = {
  speed: ['speed [m/s]', 'num', ''],
  period: ['manoeuvre period [s]', 'num', 'Time between direction changes.'],
  amplitude: ['lateral share', 'num', 'How much of the speed goes sideways (0..1).'],
  arena_center: ['arena center x, y', 'vec2', ''],
  arena_radius: ['arena radius [m]', 'num', 'It turns back before leaving this circle.'],
  min_altitude: ['min height [m]', 'num', ''],
  max_altitude: ['max height [m]', 'num', ''],
  seed: ['random seed', 'int', 'Same number = same manoeuvres in every run; 0 = different each run.'],
  radius: ['radius [m]', 'num', ''],
  phase_deg: ['start angle [°]', 'num', ''],
  height_offset: ['height above the pursuer [m]', 'num', ''],
};

const clone = (o) => JSON.parse(JSON.stringify(o));
const num = (v) => (v === '' || v === null || v === undefined || Number.isNaN(Number(v)) ? undefined : Number(v));
const r1 = (v) => Math.round(v * 10) / 10;

// -- small bound controls ----------------------------------------------------------------
/// Labeled control with an explanation under it.
function fld(label, control, help, cls = '') {
  return h('label', { class: 'sf ' + cls }, h('span', {}, label), control, help ? h('small', {}, help) : null);
}

function numIn(obj, key, onChange, { placeholder, int = false, min } = {}) {
  const el = h('input', { type: 'number', step: int ? 1 : 'any', value: obj[key] ?? '', placeholder: placeholder ?? '', min });
  el.addEventListener('change', () => {
    let v = num(el.value);
    if (v !== undefined && min !== undefined && v < min) {
      toast(`Minimum ${min}`, 'error');
      v = min;
      el.value = String(min);
    }
    if (v === undefined) delete obj[key]; else obj[key] = int ? Math.round(v) : v;
    onChange();
  });
  return el;
}

function textIn(obj, key, onChange, placeholder = '') {
  const el = h('input', { type: 'text', value: obj[key] ?? '', placeholder });
  el.addEventListener('change', () => { const v = el.value.trim(); if (v) obj[key] = v; else delete obj[key]; onChange(); });
  return el;
}

function selIn(options, value, onChange) {
  const el = h('select', {}, ...options.map(([v, t]) => h('option', { value: v }, t)));
  el.value = value;
  el.addEventListener('change', () => onChange(el.value));
  return el;
}

function check(label, value, onChange, help) {
  const el = h('input', { type: 'checkbox' });
  el.checked = !!value;
  el.addEventListener('change', () => onChange(el.checked));
  return h('label', { class: 'sf-check', title: help || '' }, el, h('span', {}, label));
}

/// Number with a "forever / until ..." check box that stores 0.
function numOrZero(obj, key, onChange, zeroLabel, dflt) {
  const box = h('div', { class: 'sf-inline' });
  const draw = () => {
    box.innerHTML = '';
    const zero = obj[key] === 0;
    const el = numIn(obj, key, onChange, { placeholder: String(dflt), min: 0 });
    el.disabled = zero;
    if (zero) el.value = '';
    box.append(el, check(zeroLabel, zero, (on) => { if (on) obj[key] = 0; else obj[key] = dflt; draw(); onChange(); }));
  };
  draw();
  return box;
}

export function scenariosPanel(ctl, state, layer) {
  let doc = null;              // document being edited
  let file = '';               // its file
  let dirty = false;
  let yamlMode = false;
  let pv = null;               // last preview

  const list = h('div', { class: 'scn-list' });
  const fileLabel = h('b', {}, '—');
  const errors = h('div', { class: 'scn-error' });
  const info = h('div', { class: 'muted' });
  const body = h('div', {});
  const yamlArea = h('textarea', { class: 'scn-yaml', spellcheck: 'false' });
  const statusBox = h('div', {});

  const preview = debounce(async () => {
    if (!doc) return;
    try {
      const r = await ctl.req('scenario.preview', yamlMode ? { yaml: yamlArea.value } : { doc }, 20000);
      layer.setPreview(r);
      errors.textContent = r.ok ? '' : r.error;
      if (r.ok) {
        pv = r;
        const t = r.drones[0];
        const pass = t ? (t.reactive ? 'reacts to the pursuer' : t.finite ? `path ≈ ${fmt(t.plan_duration, 0)} s` : 'path repeats forever') : '';
        info.textContent = `${pass} | run ${r.duration > 0 ? r.duration + ' s' : 'until the path ends'} × ${r.repeat}`;
      }
    } catch (e) { errors.textContent = e.message; }
  }, 450);

  const changed = () => { dirty = true; fileLabel.textContent = `${file || '(new)'} *`; preview(); };

  /// What the map window shows around the edited path.
  const context = () => {
    const d = pv && pv.ok ? pv : null;
    return {
      targetStart: doc.target && doc.target.start, pursuerStart: doc.pursuer && doc.pursuer.start,
      paths: d && d.drones[0] ? d.drones[0].paths : [], patrol: d && d.pursuer ? d.pursuer.paths : [],
      defaultZ: 6,
    };
  };

  // -- files ---------------------------------------------------------------------------
  async function refreshList() {
    try {
      const items = await ctl.req('scenario.list');
      list.innerHTML = '';
      for (const it of items) {
        list.append(h('div', { class: 'scn-item' + (it.file === file ? ' active' : ''), onclick: () => open(it.file) },
          it.name, h('small', {}, it.file + (it.description ? ' — ' + it.description : ''))));
      }
    } catch (e) { list.textContent = e.message; }
  }

  async function open(name) {
    if (dirty && !confirm('Discard unsaved changes?')) return;
    try {
      const r = await ctl.req('scenario.read', { file: name });
      load(r.doc, r.file, r.yaml);
    } catch (e) { toast(e.message, 'error'); }
  }

  function load(d, name, text) {
    doc = d;
    file = name || '';
    dirty = !name;
    fileLabel.textContent = file ? file : '(new) *';
    yamlArea.value = text || '';
    render();
    refreshList();
    preview();
  }

  async function save(asName) {
    if (!doc) return null;
    const target = asName || file || `${doc.name || 'scenario'}.yaml`;
    try {
      const r = await ctl.req('scenario.write', yamlMode ? { file: target, yaml: yamlArea.value } : { file: target, doc });
      doc = r.doc; file = r.file; dirty = false; yamlArea.value = r.yaml;
      fileLabel.textContent = file;
      toast(`Saved: ${file}`, 'good');
      refreshList();
      if (yamlMode) render();
      return file;
    } catch (e) { toast(e.message, 'error', 6000); return null; }
  }

  function download() {
    ctl.req(yamlMode ? 'scenario.parse' : 'scenario.yaml', yamlMode ? { yaml: yamlArea.value } : { doc }).then((r) => {
      const text = yamlMode ? yamlArea.value : r.yaml;
      const a = h('a', { href: URL.createObjectURL(new Blob([text], { type: 'text/yaml' })), download: file || `${doc.name}.yaml` });
      a.click();
    }).catch((e) => toast(e.message, 'error'));
  }

  const importer = h('input', { type: 'file', accept: '.yaml,.yml', style: { display: 'none' } });
  importer.addEventListener('change', async () => {
    const f = importer.files[0];
    if (!f) return;
    const text = await f.text();
    try { const r = await ctl.req('scenario.parse', { yaml: text }); load(r.doc, '', text); } catch (e) { toast(e.message, 'error'); }
    importer.value = '';
  });

  // -- form ------------------------------------------------------------------------------
  function render() {
    body.innerHTML = '';
    if (!doc) { body.append(h('p', { class: 'muted' }, 'Open a scenario from the list or create a new one.')); return; }
    if (yamlMode) { body.append(yamlArea); return; }
    doc.target = doc.target || {};
    doc.pursuer = doc.pursuer || {};
    body.append(general(), target(), pursuer(), controller(), advanced());
  }

  function vecIn(obj, key, title, onChange = changed) {
    const v = Array.isArray(obj[key]) ? obj[key] : null;
    const parts = [0, 1, 2].map((i) => {
      const p = h('input', { type: 'number', step: 'any', value: v ? v[i] ?? '' : '', placeholder: 'xyz'[i] });
      p.addEventListener('change', () => {
        const vals = parts.map((x) => num(x.value));
        if (vals.every((x) => x === undefined)) delete obj[key]; else obj[key] = vals.map((x) => x ?? 0);
        onChange();
      });
      return p;
    });
    const pick = button('📍', () => pickPoint(ctl, { title, value: obj[key], context: context(),
      onApply: (p) => { obj[key] = p; parts.forEach((el, i) => { el.value = p[i]; }); onChange(); } }), 'small', 'Pick on the map');
    return h('div', { class: 'sf-vec' }, ...parts, pick);
  }

  function general() {
    const ts = h('div', { class: 'sf-inline' });
    const drawTs = () => {
      ts.innerHTML = '';
      const fixed = (doc.target_speed || 0) > 0;
      ts.append(selIn([['path', 'as written in the path'], ['fixed', 'fixed']], fixed ? 'fixed' : 'path', (v) => {
        if (v === 'fixed') doc.target_speed = 2; else delete doc.target_speed;
        drawTs(); changed();
      }));
      if (fixed) ts.append(numIn(doc, 'target_speed', changed, { min: 0.1 }), h('span', { class: 'muted' }, 'm/s'));
    };
    drawTs();
    const cap = h('div', { class: 'sf-inline' });
    const drawCap = () => {
      cap.innerHTML = '';
      const on = (doc.capture_distance || 0) > 0;
      cap.append(check('end when the pursuer is within', on, (v) => { if (v) doc.capture_distance = 1.5; else delete doc.capture_distance; drawCap(); changed(); }));
      if (on) cap.append(numIn(doc, 'capture_distance', changed, { min: 0.1 }), h('span', { class: 'muted' }, 'm'));
    };
    drawCap();
    const desc = h('input', { type: 'text', value: doc.description || '' });
    desc.addEventListener('change', () => { if (desc.value.trim()) doc.description = desc.value; else delete doc.description; changed(); });
    return section('Scenario',
      h('div', { class: 'sf-grid' },
        fld('name', textIn(doc, 'name', changed), 'Letters, digits, _ or -.'),
        fld('repetitions', numIn(doc, 'repetitions', changed, { int: true, min: 1, placeholder: '1' }), 'Runs in a row.'),
        fld('duration [s]', numOrZero(doc, 'duration', changed, 'until the path ends', 60), 'Chase time per run.', 'wide'),
        fld('target speed', ts, 'Fixed: the whole path at this speed (also the live slider).', 'wide'),
        fld('', cap, '', 'wide'),
        fld('description', desc, '', 'wide')));
  }

  // -- target -----------------------------------------------------------------------------
  function target() {
    const t = doc.target;
    t.path = t.path || [];
    const yaw = h('div', { class: 'sf-inline' });
    const drawYaw = () => {
      yaw.innerHTML = '';
      const v = t.yaw ?? 'velocity';
      const mode = v === 'velocity' || v === 'hold' ? v : 'fixed';
      yaw.append(selIn([['velocity', 'along the motion'], ['hold', 'keep its heading'], ['fixed', 'fixed heading']], mode, (m) => {
        if (m === 'velocity') delete t.yaw; else if (m === 'hold') t.yaw = 'hold'; else t.yaw = 0;
        drawYaw(); changed();
      }));
      if (mode === 'fixed') yaw.append(numIn(t, 'yaw', changed, { placeholder: '0' }), h('span', { class: 'muted' }, '° (0 = east)'));
    };
    drawYaw();
    const pathBox = h('div', {});
    const drawPath = () => { pathBox.innerHTML = ''; pathBox.append(legsEditor(t.path, drawPath, 'target')); };
    drawPath();
    return section('Target (pursued drone)',
      h('div', { class: 'sf-grid' },
        fld('drone', h('div', { class: 'sf-inline' }, textIn(t, 'name', changed, 'X4'),
          selIn([['x4', 'x4'], ['x3', 'x3'], ['x5', 'x5'], ['tiltrotor', 'tiltrotor']], t.model || 'x4',
            (v) => { if (v === 'x4') delete t.model; else t.model = v; changed(); })), 'Name and model (X4 is already in the world).'),
        fld('start x, y, z [m]', vecIn(t, 'start', 'Target start'), 'Where it waits before the run.', 'wide'),
        fld('nose', yaw, '', 'wide'),
        fld('repeat the path', numOrZero(t, 'loops', changed, 'forever', 1), 'Times the whole list below is flown.'),
        fld('max speed [m/s]', numIn(t, 'max_speed', changed, { placeholder: '6' }))),
      h('h4', {}, 'Path: legs flown in order'), pathBox);
  }

  // -- legs ---------------------------------------------------------------------------------
  const legKind = (leg) => {
    for (const k of ['trajectory', 'goto', 'hover', 'behavior']) if (leg && k in leg) return k;
    return 'hover';
  };

  /// Trajectory object of a leg (from the library when it is a name).
  const trajOf = (leg) => (typeof leg.trajectory === 'string' ? (doc.trajectories || {})[leg.trajectory] : leg.trajectory) || {};

  function describe(tr, leg) {
    const type = tr.type || 'waypoints';
    const sp = leg.speed || tr.speed;
    const n = (tr.points || []).length;
    let s;
    if (type === 'waypoints' || type === 'line') {
      s = `${n} point${n === 1 ? '' : 's'}, ${tr.interpolation === 'spline' ? 'smooth' : 'straight'}${tr.closed ? ', closed loop' : ''}`;
    } else if (type === 'timed') {
      const last = n ? tr.points[n - 1] : null;
      return `timed: ${n} points over ${Array.isArray(last) ? r1(last[0]) : '?'} s`;
    } else if (type === 'hover') {
      return `hover ${tr.duration || 0} s`;
    } else {
      const label = SHAPES[type] ? SHAPES[type].label : type;
      s = `${label}${tr.radius ? `, radius ${tr.radius} m` : ''}${tr.size ? `, size ${tr.size.slice(0, 2).join(' × ')} m` : ''}`;
    }
    return `${s} · ${sp ?? 2} m/s`;
  }

  function editTrajectory(leg, owner, rerender) {
    const shared = typeof leg.trajectory === 'string';
    const tr = clone(trajOf(leg));
    if (!shared && leg.speed) { tr.speed = leg.speed; delete leg.speed; }
    openPathEditor(ctl, {
      title: shared ? `Shared trajectory "${leg.trajectory}" (every leg using it changes)` : `${owner === 'target' ? 'Target' : 'Patrol'} path`,
      trajectory: tr, context: context(),
      onApply: (t) => {
        if (shared) doc.trajectories[leg.trajectory] = t; else leg.trajectory = t;
        rerender(); changed();
      },
    });
  }

  function addLeg(legs, kind, rerender, owner) {
    const ctx = context();
    const last = legs.length ? legs[legs.length - 1] : null;
    const near = (last && last.goto) || (owner === 'target' ? ctx.targetStart : ctx.pursuerStart) || [10, 0, 6];
    const z = Math.max(3, near[2] ?? 6);
    const push = (leg) => { legs.push(leg); rerender(); changed(); };
    const draw = (trajectory, startWith) => openPathEditor(ctl, {
      title: `New ${owner === 'target' ? 'target' : 'patrol'} path`, trajectory, context: ctx, startWith,
      onApply: (t) => push({ trajectory: t }),
    });
    if (kind === 'draw') draw({ type: 'waypoints', speed: 3, points: [] });
    else if (kind === 'shape') draw({ type: 'circle', center: [r1(near[0] + 12), r1(near[1]), z], radius: 10, speed: 3 });
    else if (kind === 'import') draw({ type: 'waypoints', speed: 3, points: [] }, 'import');
    else if (kind === 'goto') push({ goto: [r1(near[0] + 10), r1(near[1]), z], speed: 3 });
    else if (kind === 'hover') push({ hover: 3 });
    else push({ behavior: 'flee', duration: 20, speed: 3 });
  }

  function legsEditor(legs, rerender, owner) {
    const box = h('div', { class: 'leg-list' });
    legs.forEach((raw, i) => {
      if (typeof raw === 'string') legs[i] = { trajectory: raw };
      const leg = legs[i];
      const kind = legKind(leg);
      const move = (d) => { const j = i + d; if (j < 0 || j >= legs.length) return; [legs[i], legs[j]] = [legs[j], legs[i]]; rerender(); changed(); };
      const title = { trajectory: 'Path', goto: 'Go to', hover: 'Hover', behavior: 'Reactive' }[kind];
      let summary = '';
      const fields = h('div', { class: 'sf-grid' });
      fields.append = (...els) => HTMLElement.prototype.append.apply(fields, els.filter((e) => e !== null && e !== undefined));
      if (kind === 'trajectory') {
        const tr = trajOf(leg);
        summary = describe(tr, leg);
        const shared = typeof leg.trajectory === 'string';
        fields.append(
          h('div', { class: 'sf wide' }, h('div', { class: 'row wrap' },
            button('✎ Edit on the map', () => editTrajectory(leg, owner, rerender), 'small primary'),
            shared ? h('span', { class: 'muted' }, `shared trajectory "${leg.trajectory}"`) : null,
            shared ? button('make it this leg\'s own', () => { leg.trajectory = clone(trajOf(leg)); rerender(); changed(); }, 'small',
              'Copy it into this leg, so editing it does not change the other legs') : null)),
          fld('laps of this leg', numOrZero(leg, 'loops', changed, 'forever', 1)),
          shared ? fld('speed [m/s]', numIn(leg, 'speed', changed, { placeholder: String(tr.speed ?? 2) }), 'Overrides the shared trajectory speed.') : null);
      } else if (kind === 'goto') {
        summary = `(${(leg.goto || []).join(', ')})`;
        fields.append(fld('point x, y, z [m]', vecIn(leg, 'goto', 'Go to'), '', 'wide'),
          fld('speed [m/s]', numIn(leg, 'speed', changed, { placeholder: '3' })));
      } else if (kind === 'hover') {
        summary = `${leg.hover ?? 0} s${leg.at ? ` at (${leg.at.join(', ')})` : ''}`;
        const at = h('div', {});
        const drawAt = () => {
          at.innerHTML = '';
          at.append(check('at another point (else where it is)', !!leg.at, (v) => { if (v) leg.at = clone(context().targetStart || [0, 0, 5]); else delete leg.at; drawAt(); changed(); }));
          if (leg.at) at.append(vecIn(leg, 'at', 'Hover at'));
        };
        drawAt();
        fields.append(fld('seconds', numIn(leg, 'hover', changed, { min: 0 })), h('div', { class: 'sf wide' }, at));
      } else {
        const b = BEHAVIORS[leg.behavior] ? leg.behavior : 'flee';
        summary = BEHAVIORS[b][0];
        fields.append(
          fld('behavior', selIn(Object.entries(BEHAVIORS).map(([k, v]) => [k, v[0]]), b, (v) => { leg.behavior = v; rerender(); changed(); }),
            BEHAVIORS[b][1], 'wide'),
          fld('duration [s]', numOrZero(leg, 'duration', changed, 'forever', 20)),
          ...BEHAVIORS[b][2].map((k) => {
            const [label, kindF, help] = BF[k];
            if (kindF === 'vec2') {
              const v = leg[k] || [];
              const parts = [0, 1].map((j) => {
                const p = h('input', { type: 'number', step: 'any', value: v[j] ?? '', placeholder: 'xy'[j] });
                p.addEventListener('change', () => {
                  const vals = parts.map((x) => num(x.value));
                  if (vals.every((x) => x === undefined)) delete leg[k]; else leg[k] = vals.map((x) => x ?? 0);
                  changed();
                });
                return p;
              });
              return fld(label, h('div', { class: 'sf-vec' }, ...parts), help);
            }
            return fld(label, numIn(leg, k, changed, { int: kindF === 'int' }), help);
          }));
      }
      box.append(h('div', { class: 'leg-card' },
        h('div', { class: 'leg-head' }, h('span', { class: 'leg-n' }, `${i + 1}`), h('b', {}, title),
          h('span', { class: 'leg-sum' }, summary), h('span', { class: 'spacer' }),
          button('↑', () => move(-1), 'small', 'Earlier'), button('↓', () => move(1), 'small', 'Later'),
          button('✕', () => { legs.splice(i, 1); rerender(); changed(); }, 'small danger', 'Remove this leg')),
        fields));
    });
    if (!legs.length) box.append(h('p', { class: 'muted' }, owner === 'target' ? 'No legs: the target hovers at its start.' : 'No legs yet.'));
    box.append(h('div', { class: 'leg-add' }, h('span', { class: 'muted' }, 'add:'),
      button('✏ Path on the map', () => addLeg(legs, 'draw', rerender, owner), 'small', 'Click the points on a map seen from above'),
      button('◯ Shape', () => addLeg(legs, 'shape', rerender, owner), 'small', 'Circle, figure 8, helix... from a few parameters'),
      button('📄 CSV / Excel', () => addLeg(legs, 'import', rerender, owner), 'small', 'Points (and times) from a table'),
      button('→ Go to', () => addLeg(legs, 'goto', rerender, owner), 'small', 'Straight to a point'),
      button('⏸ Hover', () => addLeg(legs, 'hover', rerender, owner), 'small', 'Stay still for some seconds'),
      owner === 'target' ? button('⚡ Reactive', () => addLeg(legs, 'behavior', rerender, owner), 'small', 'Flee, wander or orbit the pursuer') : null));
    return box;
  }

  // -- pursuer ------------------------------------------------------------------------------
  function pursuer() {
    const p = doc.pursuer;
    const box = h('div', {});
    const draw = () => {
      box.innerHTML = '';
      const face = p.face_target !== false;
      const vision = (p.detection || 'vision') === 'vision';
      const auto = (p.acquisition || 'auto') === 'auto';
      const patrolBox = h('div', {});
      const drawPatrol = () => { patrolBox.innerHTML = ''; patrolBox.append(legsEditor(p.patrol, drawPatrol, 'pursuer')); };
      if (p.patrol) drawPatrol();
      box.append(h('div', { class: 'sf-grid' },
        fld('start x, y, z [m]', vecIn(p, 'start', 'Pursuer start'), 'Where the PX4 waits before the run.', 'wide'),
        h('div', { class: 'sf wide' },
          check('turn the camera to the target before starting', face, (v) => { if (v) delete p.face_target; else p.face_target = false; draw(); changed(); },
            'It turns until the target is in front of the camera and keeps facing it until the chase starts.')),
        face ? fld('aligned within [°]', numIn(p, 'heading_tolerance_deg', changed, { placeholder: '3', min: 0.5 }), 'Staging waits until the heading error is below this (≥ 0.5°).')
          : fld('start heading [°]', numIn(p, 'heading_deg', changed, { placeholder: '0' }), '0 = east, 90 = north.'),
        fld('detection', selIn([['vision', 'vision (camera tracker)'], ['omniscient', 'omniscient (exact position)']], p.detection || 'vision',
          (v) => { if (v === 'vision') delete p.detection; else p.detection = v; draw(); changed(); }),
        vision ? 'The tracker finds the target in the camera image.' : 'Simulator ground truth: tests the controller alone.', 'wide'),
        vision ? fld('first box', selIn([['auto', 'automatic (from the simulator)'], ['manual', 'manual (draw it on the video)']], p.acquisition || 'auto',
          (v) => { if (v === 'auto') delete p.acquisition; else p.acquisition = v; draw(); changed(); }), 'How the tracker gets the target the first time.', 'wide') : null,
        vision && auto ? h('div', { class: 'sf wide' }, check('restart the tracker if it loses the target', p.reacquire !== false,
          (v) => { if (v) delete p.reacquire; else p.reacquire = false; changed(); })) : null,
        fld('start the chase after [s]', numIn(p, 'start_after', changed, { placeholder: '0', min: 0 }), 'Counted from the start of the run.'),
        vision ? h('div', { class: 'sf' }, check('…and only once the tracker has the target', !!p.wait_for_detection,
          (v) => { if (v) p.wait_for_detection = true; else delete p.wait_for_detection; changed(); })) : null,
        h('div', { class: 'sf wide' }, check('fly a patrol before the chase', !!p.patrol, (v) => {
          if (v) p.patrol = []; else delete p.patrol;
          draw(); changed();
        }, 'Without a patrol it holds its start position until the chase starts.'))),
      p.patrol ? h('h4', {}, 'Patrol: legs flown before the chase') : null, p.patrol ? patrolBox : null);
    };
    draw();
    return section('Pursuer', box);
  }

  // -- controller and advanced --------------------------------------------------------------
  function controller() {
    doc.parameters = doc.parameters || {};
    const pc = doc.parameters['/pursuit_controller_node'] = doc.parameters['/pursuit_controller_node'] || {};
    const params = h('textarea', { class: 'scn-yaml', style: { minHeight: '90px' }, spellcheck: 'false' });
    const sync = () => { params.value = JSON.stringify(doc.parameters, null, 1); };
    sync();
    params.addEventListener('change', () => {
      try { doc.parameters = JSON.parse(params.value || '{}'); changed(); render(); } catch (e) { toast('Parameters: invalid JSON', 'error'); }
    });
    return section('Controller',
      h('div', { class: 'sf-grid' },
        fld('distance to keep [m]', numIn(pc, 'target_distance', () => { sync(); changed(); }, { placeholder: 'current' }),
          'Standoff from the target; empty = keep the current value.')),
      h('details', {}, h('summary', {}, 'Other ROS parameters set at each run'),
        h('p', { class: 'muted' }, 'JSON: node → parameter → value, e.g. {"/pursuit_controller_node": {"mpc.params.yaw_mode": 3}}. Parameters not listed keep their current value (also what you changed in the Control tab!): a study should fix what it compares.'),
        params));
  }

  function advanced() {
    const st = doc.staging = doc.staging || {};
    const shared = Object.keys(doc.trajectories || {});
    const used = (name) => [...(doc.target.path || []), ...(doc.pursuer.patrol || [])]
      .filter((l) => l === name || (l && l.trajectory === name)).length;
    return h('details', { class: 'card sf-adv' }, h('summary', {}, 'Advanced'),
      h('h4', {}, 'Staging (before each run)'),
      h('div', { class: 'sf-grid' },
        fld('timeout [s]', numIn(st, 'timeout', changed, { placeholder: '45' }), 'Gives up if they are not in place by then.'),
        fld('target within [m]', numIn(st, 'tolerance', changed, { placeholder: '1.0' })),
        fld('pursuer within [m]', numIn(st, 'pursuer_tolerance', changed, { placeholder: '0.3' })),
        fld('then wait [s]', numIn(st, 'settle', changed, { placeholder: '1.0' })),
        fld('at rest below [m/s]', numIn(st, 'max_speed', changed, { placeholder: '0.15' }))),
      shared.length ? h('div', {}, h('h4', {}, 'Shared trajectories'),
        h('p', { class: 'muted' }, 'Trajectories with a name, used by more than one leg (from YAML files).'),
        ...shared.map((name) => h('div', { class: 'row wrap' }, h('b', {}, name),
          h('span', { class: 'muted' }, describe(doc.trajectories[name], {}) + ` · used by ${used(name)} leg(s)`),
          used(name) ? null : button('✕', () => { delete doc.trajectories[name]; render(); changed(); }, 'small danger')))) : null);
  }

  // -- execution ---------------------------------------------------------------------------
  async function run() {
    if (!doc) return;
    if (dirty || !file) { if (!(await save())) return; }
    const cfg = state.sim ? state.sim.cfg : {};
    try {
      const r = await ctl.req('scenario.run', { file, cfg }, 15000);
      toast(r.message || 'Scenario started', 'good');
    } catch (e) { toast(e.message, 'error', 8000); }
  }
  const cmd = (c) => ctl.req('scenario.cmd', { command: c }).then((r) => toast(r.message || c, r.success ? 'good' : 'error'))
    .catch((e) => toast(e.message, 'error'));

  ctl.on('tel', (m) => {
    const s = m.scenario;
    if (!s) return;
    const t = m.truth;
    statusBox.innerHTML = '';
    statusBox.append(h('div', {},
      h('div', {}, h('b', {}, s.state_name), ` ${s.scenario || ''} ${s.run_id ? '(' + s.run_id + ')' : ''}`),
      h('div', { class: 'muted' }, s.message || ''),
      s.state_name === 'RUNNING' || s.state_name === 'PAUSED'
        ? h('div', {}, `run ${s.run_index}/${s.repeat} | ${fmt(s.elapsed, 1)} s${s.duration > 0 ? ' / ' + s.duration + ' s' : ''} | chase ${s.engaged ? 'on' : 'waiting'}`) : null,
      t ? h('div', {}, `tracking: ${t.tracked_identity || '—'} ${t.on_target ? '✔ target' : ''} | IoU ${fmt(t.iou, 2)} | acquisitions ${t.acquisitions} | switches ${t.identity_switches}`) : null,
      h('table', { class: 'scn-status' }, ...(s.drones || []).filter((d) => d.role !== 'pursuer').map((d) =>
        h('tr', {}, h('td', {}, d.name), h('td', {}, d.in_transit ? `→ ${d.leg}` : d.leg || ''),
          h('td', { class: 'muted' }, d.finished ? 'path finished' : d.leg_iteration ? `lap ${d.leg_iteration + 1}` : ''))))));
  });

  const yamlToggle = button('YAML', async () => {
    try {
      if (!yamlMode) {
        const r = await ctl.req('scenario.yaml', { doc });
        yamlArea.value = r.yaml;
      } else {
        const r = await ctl.req('scenario.parse', { yaml: yamlArea.value });
        doc = r.doc;
      }
      yamlMode = !yamlMode;
      yamlToggle.classList.toggle('active', yamlMode);   // not e.currentTarget: null after an await
      render();
      preview();
    } catch (err) { toast(err.message, 'error'); }
  }, 'small', 'Switch between the form and the YAML text (comments are kept when saving in YAML mode)');
  yamlArea.addEventListener('input', () => { dirty = true; fileLabel.textContent = `${file || '(new)'} *`; preview(); });

  ctl.on('open', () => refreshList());
  render();

  return {
    el: h('div', {},
      section('Run', h('div', { class: 'row wrap' },
        button('▶ Run', run, 'primary', 'Saves and runs (starts the simulation if needed)'),
        button('■ Stop', () => cmd('stop'), 'danger', 'Stops: drones hover where they are and the chase is disabled'),
        button('⏸', () => cmd('pause'), 'small', 'Pause'), button('⏵', () => cmd('resume'), 'small', 'Resume'),
        button('⏏ Unload', () => cmd('unload'), 'small', 'Stops and removes the drones the scenario inserted (base-world X3/X4/Tiltrotor/X5 stay)')),
      h('p', { class: 'muted' }, 'Stop keeps the drones in the world; Unload removes them. ' +
        'Turn off joystick sending or leave the sticks released: manual commands override the scenario on the selected drone.'),
      statusBox),
      section('Files', list, h('div', { class: 'row wrap' },
        button('New', () => { if (!dirty || confirm('Discard changes?')) load(TEMPLATE(), '', ''); }, 'small'),
        button('Save', () => save(), 'small primary'),
        button('Save as', () => { const n = prompt('File name', file || `${doc.name}.yaml`); if (n) save(n); }, 'small'),
        button('Delete', async () => {
          if (!file || !confirm(`Delete ${file}?`)) return;
          await ctl.req('scenario.delete', { file }).catch((e) => toast(e.message, 'error'));
          doc = null; file = ''; dirty = false; layer.setPreview(null); render(); refreshList();
        }, 'small danger'),
        button('Export', download, 'small'), button('Import', () => importer.click(), 'small'), importer)),
      h('div', { class: 'card scn-editor-head' }, h('div', { class: 'row wrap' }, fileLabel, h('span', { class: 'spacer' }),
        button('? Help', () => openScenarioHelp(ctl), 'small', 'What each field does, the path kinds, CSV format, YAML reference'),
        yamlToggle, button('↻', preview, 'small', 'Recompute the preview')), info, errors),
      body),
    onShow: () => refreshList(),
  };
}
