// Parameters of any node: listing, live editing (sliders, switches, lists) and
// sync with /parameter_events (changes made by others also show up).

import { h, button, select, slider, toggle, textInput, toast, section } from './ui.js';

// Known options (string -> drop-down list).
export const ENUMS = {
  kf_model: ['cv', 'singer'],
  tracker_model: ['vit', 'nano', 'dasiamrpn'],
  record_mode: ['session', 'continuous'],
  dnn_target: ['cpu', 'opencl', 'opencl_fp16'],
  storage_id: ['mcap', 'sqlite3'],
};

export const VISIBLE_WHEN = {};

function niceStep(span) {
  const raw = span / 200;
  const p = 10 ** Math.floor(Math.log10(raw || 1e-3));
  return [1, 2, 5, 10].map((m) => m * p).find((s) => s >= raw) || raw;
}

/// Slider range: the one declared in the node or a heuristic from the current value.
function rangeFor(p) {
  if (p.range && p.range[0] !== p.range[1]) return [p.range[0], p.range[1], p.range[2] || null];
  const v = Number(p.value) || 0;
  const isInt = p.type === 'int';
  if (v === 0) return [0, isInt ? 10 : 1, null];
  if (v > 0) return [0, isInt ? Math.max(10, Math.ceil(v * 4)) : v * 4, null];
  return [v * 4, -v * 4, null];
}

/// Control of one parameter. `apply(value)` sends it to the node.
export function paramControl(p, apply, options) {
  const label = p.name;
  const title = p.description || p.name;
  let el;
  if (p.type === 'bool') {
    el = toggle({ label, value: p.value, onChange: apply, title });
  } else if (p.type === 'int' || p.type === 'double') {
    const [lo, hi, st] = rangeFor(p);
    const step = st || (p.type === 'int' ? 1 : niceStep(hi - lo));
    el = slider({ label, min: lo, max: hi, step, value: p.value, digits: p.type === 'int' ? 0 : 4,
      onChange: apply, live: false, title });
  } else if (p.type === 'string' && (options || ENUMS[p.name] || ENUMS[p.name.split('.').pop()])) {
    const opts = options || ENUMS[p.name] || ENUMS[p.name.split('.').pop()];
    el = select({ label, options: opts.includes(p.value) ? opts : [p.value, ...opts], value: p.value, onChange: apply, title });
  } else if (p.type === 'string') {
    el = textInput({ label, value: p.value, onChange: apply });
    el.title = title;
  } else {
    // Vectors: editable JSON.
    el = textInput({ label: `${label} (${p.type})`, value: JSON.stringify(p.value), onChange: (txt) => {
      try { apply(JSON.parse(txt)); } catch (e) { toast('Invalid JSON', 'error'); }
    } });
    el.set = (v) => { if (document.activeElement !== el.input) el.input.value = JSON.stringify(v); };
  }
  if (p.read_only) {
    el.classList.add('disabled');
    el.querySelectorAll('input,select').forEach((i) => { i.disabled = true; });
    el.title = (title || '') + ' (somente leitura)';
  }
  return el;
}

/// Binds a set of parameters of a node to controls (also used in the quick panels).
export class ParamBinding {
  constructor(ctl, node) {
    this.ctl = ctl;
    this.node = node;
    this.params = new Map();     // name -> description
    this.controls = new Map();   // name -> [controls]
    this.conditional = false;    // hide controls that do not apply (VISIBLE_WHEN)
    this.listeners = [];         // called after every change (values)
    ctl.on('param.event', (ev) => {
      if (ev.node !== this.node) return;
      for (const c of ev.changes) {
        const p = this.params.get(c.name);
        if (p) p.value = c.value;
        for (const el of this.controls.get(c.name) || []) el.set(c.value);
      }
      this.refreshOptions();
    });
  }

  async load() {
    const list = await this.ctl.req('param.list', { node: this.node }, 8000);
    this.params = new Map(list.map((p) => [p.name, p]));
    for (const [name, els] of this.controls) {
      const p = this.params.get(name);
      if (p) els.forEach((el) => el.set(p.value));
    }
    return list;
  }

  async apply(name, value) {
    const p = this.params.get(name);
    if (!p) { toast(`${name}: parameter not loaded`, 'error'); return; }
    try {
      await this.ctl.req('param.set', { node: this.node, name, type: p.type, value });
      p.value = value;
    } catch (e) {
      toast(`${name}: ${e.message}`, 'error', 6000);
      for (const el of this.controls.get(name) || []) el.set(p.value);   // desfaz
    }
  }

  values() {
    return Object.fromEntries([...this.params].map(([k, p]) => [k, p.value]));
  }

  /// Options valid now (DYNAMIC_ENUMS), e.g. only `trajectory` for laws without direct outputs.
  optionsFor() {
    return undefined;
  }

  onChange(fn) { this.listeners.push(fn); }

  refreshOptions() {
    const vals = this.values();
    if (this.conditional) {
      for (const [name, els] of this.controls) {
        const show = !VISIBLE_WHEN[name] || VISIBLE_WHEN[name](vals);
        for (const el of els) el.style.display = show ? '' : 'none';
      }
    }
    for (const fn of this.listeners) fn(vals);
  }

  control(name, fallback) {
    const p = this.params.get(name) || fallback;
    const el = paramControl(p, (v) => this.apply(name, v), this.optionsFor(name));
    if (!this.controls.has(name)) this.controls.set(name, []);
    this.controls.get(name).push(el);
    return el;
  }
}

export function paramsPanel(ctl) {
  const nodeSel = select({ label: 'Node', options: [['', '(carregando)']], onChange: () => load() });
  const filter = textInput({ label: 'Filter', placeholder: 'e.g. mpc, deadband' });
  const body = h('div', { class: 'params' });
  const status = h('div', { class: 'muted' });
  let binding = null;

  async function refreshNodes() {
    try {
      const nodes = await ctl.req('nodes');
      const cur = nodeSel.get();
      nodeSel.setOptions(nodes.length ? nodes : [['', '(no nodes: is the simulation stopped?)']]);
      const prefer = ['/pursuit_controller_node', '/vision_tracker_node'];
      if (!nodes.includes(cur)) nodeSel.set(prefer.find((n) => nodes.includes(n)) || nodes[0] || '');
      else nodeSel.set(cur);
      if (!binding || binding.node !== nodeSel.get()) load();
    } catch (e) { status.textContent = e.message; }
  }

  async function load() {
    const node = nodeSel.get();
    body.innerHTML = '';
    if (!node) return;
    status.textContent = 'carregando...';
    binding = new ParamBinding(ctl, node);
    try {
      const list = await binding.load();
      status.textContent = `${list.length} parameters`;
      render();
    } catch (e) {
      status.textContent = e.message;
    }
  }

  function render() {
    body.innerHTML = '';
    if (!binding) return;
    binding.controls.clear();
    const q = filter.get().trim().toLowerCase();
    const groups = new Map();
    for (const p of binding.params.values()) {
      if (q && !p.name.toLowerCase().includes(q)) continue;
      const g = p.name.includes('.') ? p.name.split('.').slice(0, -1).join('.') : '(geral)';
      if (!groups.has(g)) groups.set(g, []);
      groups.get(g).push(p);
    }
    for (const [g, ps] of [...groups].sort((a, b) => (a[0] === '(geral)' ? -1 : b[0] === '(geral)' ? 1 : a[0].localeCompare(b[0])))) {
      const details = h('details', { class: 'pgroup', open: true }, h('summary', {}, `${g} (${ps.length})`));
      for (const p of ps) details.append(binding.control(p.name));
      body.append(details);
    }
  }

  // Remembered settings: re-applied by the server when the node reappears.
  const ovBox = h('div', { class: 'muted' });
  const remember = toggle({ label: 'Remember changes (re-apply when the simulation restarts)', value: true,
    onChange: (v) => ctl.req('param.remember', { on: v }).catch(() => {}) });
  ctl.on('param.overrides', (m) => {
    remember.set(m.remember);
    ovBox.innerHTML = '';
    const entries = Object.entries(m.overrides).flatMap(([n, ps]) => Object.entries(ps).map(([k, p]) => [n, k, p]));
    if (!entries.length) { ovBox.textContent = 'no remembered settings'; return; }
    for (const [n, k, p] of entries) {
      ovBox.append(h('div', { class: 'series' }, h('span', { class: 'mono' }, `${n} ${k} = ${JSON.stringify(p.value)}`),
        button('×', () => ctl.req('param.forget', { node: n, name: k }), 'small', 'Forget')));
    }
    ovBox.append(button('Forget all', () => ctl.req('param.forget', {}), 'small danger'));
  });

  filter.input.addEventListener('input', render);
  ctl.on('open', () => { refreshNodes(); ctl.req('param.overrides').then((m) => ctl.emit('param.overrides', m)).catch(() => {}); });
  return {
    el: h('div', {},
      section(null, h('div', { class: 'row' }, nodeSel, button('↻', () => refreshNodes(), 'small', 'Refresh nodes')), filter, status),
      body,
      section('Remembered settings', remember, ovBox)),
    refresh: refreshNodes,
  };
}
