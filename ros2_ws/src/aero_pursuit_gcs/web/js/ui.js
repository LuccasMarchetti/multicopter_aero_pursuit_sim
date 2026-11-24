// Interface utilities: DOM builder, controls and lightweight charts.

export function h(tag, attrs = {}, ...children) {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) {
    if (v === undefined || v === null || v === false) continue;
    if (k === 'class') el.className = v;
    else if (k === 'style' && typeof v === 'object') Object.assign(el.style, v);
    else if (k.startsWith('on') && typeof v === 'function') el.addEventListener(k.slice(2), v);
    else if (k === 'html') el.innerHTML = v;
    else el.setAttribute(k, v === true ? '' : v);
  }
  for (const c of children.flat()) {
    if (c === null || c === undefined || c === false) continue;
    el.append(c instanceof Node ? c : document.createTextNode(String(c)));
  }
  return el;
}

export function debounce(fn, ms) {
  let t = null;
  return (...a) => { clearTimeout(t); t = setTimeout(() => fn(...a), ms); };
}

/// Limits the call rate, but always delivers the last value.
export function throttle(fn, ms) {
  let last = 0, timer = null, pending = null;
  return (...a) => {
    pending = a;
    const now = performance.now();
    if (now - last >= ms) {
      last = now; fn(...pending); pending = null;
    } else if (!timer) {
      timer = setTimeout(() => { timer = null; last = performance.now(); if (pending) fn(...pending); pending = null; }, ms - (now - last));
    }
  };
}

export function fmt(v, digits = 1, unit = '') {
  if (v === null || v === undefined || Number.isNaN(v)) return '—';
  if (typeof v !== 'number') return String(v);
  return v.toFixed(digits) + (unit ? ' ' + unit : '');
}

/// Quality class (good/ok/bad) for a value and limits [good, acceptable].
export function grade(v, good, ok, higherIsBetter = false) {
  if (v === null || v === undefined || Number.isNaN(v)) return 'na';
  if (higherIsBetter) return v >= good ? 'good' : v >= ok ? 'ok' : 'bad';
  return v <= good ? 'good' : v <= ok ? 'ok' : 'bad';
}

let toastBox = null;
export function toast(text, kind = 'info', ms = 3500) {
  if (!toastBox) { toastBox = h('div', { class: 'toasts' }); document.body.append(toastBox); }
  const t = h('div', { class: 'toast ' + kind }, text);
  toastBox.append(t);
  setTimeout(() => t.classList.add('hide'), ms);
  setTimeout(() => t.remove(), ms + 400);
}

/// Slider + numeric field. `onChange(value)` is called live (rate limited) and at the end.
export function slider({ label, min, max, step = 'any', value, unit = '', digits = 2, onChange,
  live = true, rate = 120, title, format }) {
  const range = h('input', { type: 'range', min, max, step, value });
  const num = h('input', { type: 'number', min, max, step, value, class: 'num' });
  const disp = format ? h('span', { class: 'fmt' }) : null;
  const send = throttle((v) => onChange && onChange(v), rate);
  const set = (v, emit) => {
    if (v === null || v === undefined || Number.isNaN(Number(v))) return;
    v = Number(v);
    if (v < Number(range.min)) range.min = v;
    if (v > Number(range.max)) range.max = v;
    range.value = v;
    if (document.activeElement !== num) num.value = +v.toFixed(Math.max(0, digits));
    if (disp) disp.textContent = format(v);
    if (emit) send(v);
  };
  range.addEventListener('input', () => set(range.value, live));
  range.addEventListener('change', () => set(range.value, true));
  num.addEventListener('change', () => set(num.value, true));
  num.addEventListener('keydown', (e) => { if (e.key === 'Enter') set(num.value, true); });
  set(value, false);
  const el = h('label', { class: 'ctl slider', title }, h('span', { class: 'lbl' }, label),
    h('div', { class: 'row' }, range, num, unit ? h('span', { class: 'unit' }, unit) : null, disp));
  el.set = (v) => set(v, false);
  el.setRange = (a, b) => { range.min = a; range.max = b; num.min = a; num.max = b; };
  el.setDisabled = (d) => { range.disabled = d; num.disabled = d; el.classList.toggle('disabled', d); };
  return el;
}

export function select({ label, options, value, onChange, title }) {
  const s = h('select', {});
  const fill = (opts) => {
    const cur = s.value;
    s.innerHTML = '';
    for (const o of opts) {
      const [v, text, disabled] = Array.isArray(o) ? o : [o, o];
      s.append(h('option', { value: v, disabled: !!disabled }, text));
    }
    if (cur && opts.some((o) => (Array.isArray(o) ? o[0] : o) === cur)) s.value = cur;
  };
  fill(options);
  if (value !== undefined) s.value = value;
  s.addEventListener('change', () => onChange && onChange(s.value));
  const el = h('label', { class: 'ctl select', title }, label ? h('span', { class: 'lbl' }, label) : null, s);
  el.set = (v) => { if (v !== undefined && v !== null) s.value = v; };
  el.get = () => s.value;
  el.setOptions = (opts) => fill(opts);
  el.input = s;
  return el;
}

export function toggle({ label, value, onChange, title }) {
  const i = h('input', { type: 'checkbox' });
  i.checked = !!value;
  i.addEventListener('change', () => onChange && onChange(i.checked));
  const el = h('label', { class: 'ctl toggle', title }, i, h('span', { class: 'sw' }), h('span', { class: 'lbl' }, label));
  el.set = (v) => { i.checked = !!v; };
  el.get = () => i.checked;
  el.input = i;
  return el;
}

export function textInput({ label, value = '', placeholder = '', onChange, type = 'text' }) {
  const i = h('input', { type, value, placeholder });
  i.addEventListener('change', () => onChange && onChange(i.value));
  const el = h('label', { class: 'ctl text' }, label ? h('span', { class: 'lbl' }, label) : null, i);
  el.set = (v) => { if (document.activeElement !== i) i.value = v ?? ''; };
  el.get = () => i.value;
  el.input = i;
  return el;
}

export function button(text, onClick, cls = '', title) {
  return h('button', { class: cls, onclick: onClick, title }, text);
}

export function section(title, ...children) {
  return h('section', { class: 'card' }, title ? h('h3', {}, title) : null, ...children);
}

/// Updatable "label: value" row.
export function stat(label, cls = '') {
  const v = h('span', { class: 'v' }, '—');
  const el = h('div', { class: 'stat ' + cls }, h('span', { class: 'k' }, label), v);
  el.set = (text, grade) => {
    v.textContent = text;
    el.dataset.grade = grade || '';
  };
  return el;
}

/// Mini line chart (last N values).
export class Sparkline {
  constructor(n = 120, { min = 0, max = null, color = '#4fc3f7' } = {}) {
    this.n = n; this.values = []; this.min = min; this.max = max; this.color = color;
    this.canvas = h('canvas', { class: 'spark', width: 240, height: 40 });
  }
  push(v) {
    this.values.push(v);
    if (this.values.length > this.n) this.values.shift();
    this.draw();
  }
  draw() {
    const c = this.canvas, ctx = c.getContext('2d');
    const w = c.width, hgt = c.height;
    ctx.clearRect(0, 0, w, hgt);
    const vals = this.values.filter((v) => v !== null && v !== undefined && !Number.isNaN(v));
    if (vals.length < 2) return;
    const lo = this.min ?? Math.min(...vals);
    const hi = this.max ?? Math.max(lo + 1e-6, ...vals) * 1.1;
    ctx.strokeStyle = this.color; ctx.lineWidth = 1.5; ctx.beginPath();
    this.values.forEach((v, i) => {
      if (v === null || v === undefined || Number.isNaN(v)) return;
      const x = (i / (this.n - 1)) * w;
      const y = hgt - ((Math.min(v, hi) - lo) / (hi - lo)) * (hgt - 2) - 1;
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.stroke();
    ctx.fillStyle = '#9aa7b4'; ctx.font = '10px sans-serif';
    ctx.fillText(hi.toFixed(hi < 10 ? 1 : 0), 2, 10);
  }
}

export const PALETTE = ['#4fc3f7', '#ffb74d', '#81c784', '#e57373', '#ba68c8', '#fff176', '#4db6ac', '#f06292', '#a1887f', '#90a4ae'];

/// Calls a ROS service through the GCS and shows the result in a toast.
export async function callService(ctl, service, type, request, okText) {
  try {
    const r = await ctl.req('srv', { service, type, request }, 8000);
    if (r && r.success === false) toast(`${service}: ${r.message || 'refused'}`, 'error');
    else toast(okText || `${service}: ok`, 'good');
    return r;
  } catch (e) {
    toast(`${service}: ${e.message}`, 'error', 6000);
    return null;
  }
}

/// Publishes a ROS message through the GCS and shows the result in a toast.
export function publish(ctl, topic, type, data, okText) {
  return ctl.req('pub', { topic, type, data }).then(() => { if (okText) toast(okText, 'good'); })
    .catch((e) => toast(e.message, 'error'));
}

/// Short help paragraph (dimmed text).
export function hint(text) {
  return h('p', { class: 'muted' }, text);
}
