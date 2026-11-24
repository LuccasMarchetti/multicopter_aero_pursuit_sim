// Joystick in the browser (Gamepad API), keyboard or virtual touch sticks -> sensor_msgs/Joy on /joy.
// Default layout: that of the joy game_controller_node (SDL), which drone_joy_teleop expects:
//   eixos 0 LX, 1 LY, 2 RX, 3 RY, 4 LT, 5 RT (esquerda/cima positivos; gatilho 0 -> -1)
//   buttons 0 A, 1 B, 2 X, 3 Y, 4 BACK, 5 GUIDE, 6 START, 7 LS, 8 RS, 9 LB, 10 RB,
//          11 cima, 12 baixo, 13 esquerda, 14 direita
// The server zeroes the command if the packets stop for 0.5 s (connection drop).

import { h, button, select, section, slider, stat, toggle, fmt, Sparkline } from './ui.js';

const STD_TO_SDL_BUTTONS = [0, 1, 2, 3, 8, 16, 9, 10, 11, 4, 5, 12, 13, 14, 15];
const AXIS_NAMES = ['LX', 'LY', 'RX', 'RY', 'LT', 'RT'];
const BUTTON_NAMES = ['A', 'B', 'X', 'Y', 'Back', 'Guide', 'Start', 'LS', 'RS', 'LB', 'RB', '↑', '↓', '←', '→'];

// Keyboard: WASD = right stick (forward/sideways), R/F = up/down, Q/E = yaw,
// [ ] , . PageUp PageDown = previous/next vehicle (LB/RB), Z/X = triggers (speed).
const KEYMAP = {
  axes: { KeyW: [3, 1], KeyS: [3, -1], KeyA: [2, 1], KeyD: [2, -1], KeyR: [1, 1], KeyF: [1, -1], KeyQ: [0, 1], KeyE: [0, -1], KeyZ: [4, -1], KeyX: [5, -1] },
  buttons: { PrevVehicle: 9, NextVehicle: 10, Space: 0 },
};

/// Logical key identifier. The brackets go by the character (`e.key`): on the
/// ABNT2 a tecla "[" tem e.code BracketRight e a "]" Backslash.
function keyId(e) {
  if (e.key === '[' || e.key === '{' || ['Comma', 'PageUp'].includes(e.code)) return 'PrevVehicle';
  if (e.key === ']' || e.key === '}' || ['Period', 'PageDown'].includes(e.code)) return 'NextVehicle';
  return e.code;
}

/// Virtual joystick (touch / mouse): two thumbsticks and a few buttons floating over the whole
/// interface (any tab), for phones and tablets. Same axes as a gamepad in the SDL layout:
/// left stick = up/down (axis 1) and yaw (axis 0), right stick = forward (axis 3) and sideways
/// (axis 2), left/up positive. Buttons: LB/RB = previous/next vehicle, LT/RT = slower/faster.
function virtualJoystick(onClose) {
  const state = { axes: [0, 0, 0, 0, 0, 0], buttons: new Array(15).fill(0) };
  const stick = (axX, axY, caption) => {
    const knob = h('div', { class: 'vj-knob' });
    const pad = h('div', { class: 'vj-pad' }, knob);
    let id = null;
    const set = (nx, ny) => {
      knob.style.transform = `translate(${nx * 50}%, ${ny * 50}%)`;
      // Screen y grows downwards; "up"/"left" are positive on the joystick.
      state.axes[axX] = Math.abs(nx) < 0.06 ? 0 : -nx;
      state.axes[axY] = Math.abs(ny) < 0.06 ? 0 : -ny;
    };
    const move = (e) => {
      if (e.pointerId !== id) return;
      const r = pad.getBoundingClientRect();
      let nx = (e.clientX - (r.left + r.width / 2)) / (r.width / 2);
      let ny = (e.clientY - (r.top + r.height / 2)) / (r.height / 2);
      const n = Math.hypot(nx, ny);
      if (n > 1) { nx /= n; ny /= n; }
      set(nx, ny);
    };
    const release = (e) => { if (e.pointerId === id) { id = null; set(0, 0); pad.classList.remove('active'); } };
    pad.addEventListener('pointerdown', (e) => {
      if (id !== null) return;
      id = e.pointerId;
      pad.setPointerCapture(id);
      pad.classList.add('active');
      move(e);
      e.preventDefault();
    });
    pad.addEventListener('pointermove', move);
    pad.addEventListener('pointerup', release);
    pad.addEventListener('pointercancel', release);
    pad.addEventListener('lostpointercapture', release);
    return { el: h('div', { class: 'vj-stick' }, pad, h('div', { class: 'vj-cap' }, caption)), reset: () => set(0, 0) };
  };
  // Held while pressed (the teleop reacts to the press edge or to the hold, like a gamepad).
  const hold = (text, title, apply) => {
    const b = h('button', { class: 'small vj-btn', title }, text);
    const on = (v) => (e) => { apply(v); b.classList.toggle('on', v); e.preventDefault(); };
    b.addEventListener('pointerdown', on(true));
    for (const ev of ['pointerup', 'pointercancel', 'pointerleave']) b.addEventListener(ev, on(false));
    return b;
  };
  const left = stick(0, 1, 'up/down · yaw');
  const right = stick(2, 3, 'forward/back · sideways');
  const vehicle = h('span', { class: 'vj-vehicle', title: 'Commanded drone (Drones tab)' });
  const close = h('button', { class: 'small vj-btn', title: 'Close the virtual joystick' }, '×');
  close.addEventListener('click', () => onClose());
  const bar = h('div', { class: 'vj-bar' },
    hold('◀', 'Previous vehicle (LB)', (v) => { state.buttons[9] = v ? 1 : 0; }),
    vehicle,
    hold('▶', 'Next vehicle (RB)', (v) => { state.buttons[10] = v ? 1 : 0; }),
    hold('speed −', 'Decrease speed (LT)', (v) => { state.axes[4] = v ? -1 : 0; }),
    hold('speed +', 'Increase speed (RT)', (v) => { state.axes[5] = v ? -1 : 0; }),
    close);
  const root = h('div', { id: 'vjoy', hidden: true }, left.el, bar, right.el);
  document.body.append(root);
  return {
    state,
    root,
    setVehicle: (text) => { vehicle.textContent = text; },
    show: (v) => {
      root.hidden = !v;
      if (!v) {
        left.reset(); right.reset();
        state.buttons.fill(0); state.axes[4] = state.axes[5] = 0;
      }
    },
  };
}

function dead(v, dz) {
  if (Math.abs(v) < dz) return 0;
  return Math.sign(v) * (Math.abs(v) - dz) / (1 - dz);
}

export function joystickPanel(ctl) {
  let enabled = false;
  let keyboard = false;
  let touch = false;
  let rate = 50;
  let deadzone = 0.08;
  let layout = 'sdl';
  let padIndex = null;
  let seq = 0;
  let timer = null;
  const keys = new Set();
  const rttLine = new Sparkline(120, { color: '#81c784' });
  const rtt = stat('Command RTT');
  const src = stat('Source');
  const sent = stat('Sent');
  let sentCount = 0;
  const sendTimes = new Map();

  const padSel = select({ label: 'Control', options: [['', '(none detected: press a button)']], onChange: (v) => { padIndex = v === '' ? null : Number(v); } });
  const axesBox = h('div', { class: 'axes' });
  const btnBox = h('div', { class: 'buttons' });
  const axisBars = AXIS_NAMES.map((n) => {
    const fill = h('div', { class: 'fill' });
    const val = h('span', { class: 'val' }, '0');
    axesBox.append(h('div', { class: 'axis' }, h('span', { class: 'n' }, n), h('div', { class: 'bar' }, fill), val));
    return { fill, val };
  });
  const btnEls = BUTTON_NAMES.map((n) => { const b = h('span', { class: 'btn' }, n); btnBox.append(b); return b; });

  const refreshPads = () => {
    const pads = [...navigator.getGamepads()].filter(Boolean);
    padSel.setOptions(pads.length ? pads.map((p) => [String(p.index), `${p.index}: ${p.id.slice(0, 48)}`]) : [['', '(none detected: press a button)']]);
    if (pads.length && (padIndex === null || !pads.some((p) => p.index === padIndex))) padIndex = pads[0].index;
    if (padIndex !== null) padSel.set(String(padIndex));
  };
  window.addEventListener('gamepadconnected', refreshPads);
  window.addEventListener('gamepaddisconnected', refreshPads);
  window.addEventListener('keydown', (e) => {
    if (!keyboard || e.ctrlKey || e.altKey || e.metaKey) return;
    if (['INPUT', 'TEXTAREA', 'SELECT'].includes(document.activeElement.tagName)) return;
    if (e.target.closest && e.target.closest('#spectator')) return;   // free-camera WASD
    const id = keyId(e);
    if (KEYMAP.axes[id] || KEYMAP.buttons[id] !== undefined) { keys.add(id); e.preventDefault(); }
  });
  window.addEventListener('keyup', (e) => { keys.delete(keyId(e)); keys.delete(e.code); });
  window.addEventListener('blur', () => keys.clear());

  function read() {
    let axes = [0, 0, 0, 0, 0, 0];
    let buttons = new Array(15).fill(0);
    let source = null;
    const pad = padIndex !== null ? navigator.getGamepads()[padIndex] : null;
    if (pad && pad.connected) {
      source = pad.id.slice(0, 30);
      if (layout === 'raw') {
        axes = pad.axes.map((a) => dead(a, deadzone));
        buttons = pad.buttons.map((b) => (b.pressed ? 1 : 0));
      } else {
        const a = pad.axes;
        const b = pad.buttons;
        axes = [-dead(a[0] || 0, deadzone), -dead(a[1] || 0, deadzone), -dead(a[2] || 0, deadzone),
          -dead(a[3] || 0, deadzone), -(b[6] ? b[6].value : 0), -(b[7] ? b[7].value : 0)];
        buttons = STD_TO_SDL_BUTTONS.map((i) => (b[i] && b[i].pressed ? 1 : 0));
      }
    }
    if (keyboard && keys.size) {
      source = source ? source + ' + teclado' : 'teclado';
      for (const k of keys) {
        const ax = KEYMAP.axes[k];
        if (ax) axes[ax[0]] = Math.max(-1, Math.min(1, axes[ax[0]] + ax[1]));
        const bt = KEYMAP.buttons[k];
        if (bt !== undefined) buttons[bt] = 1;
      }
    } else if (keyboard && !source) {
      source = 'teclado';
    }
    if (touch) {
      source = source ? source + ' + toque' : 'touch';
      vj.state.axes.forEach((v, i) => { axes[i] = Math.max(-1, Math.min(1, (axes[i] || 0) + v)); });
      vj.state.buttons.forEach((v, i) => { if (v) buttons[i] = 1; });
    }
    return { axes, buttons, source };
  }

  function tick() {
    const { axes, buttons, source } = read();
    axes.slice(0, 6).forEach((v, i) => {
      const bar = axisBars[i];
      if (!bar) return;
      bar.fill.style.width = `${Math.abs(v) * 50}%`;
      bar.fill.style.left = v >= 0 ? '50%' : `${50 - Math.abs(v) * 50}%`;
      bar.val.textContent = v.toFixed(2);
    });
    buttons.slice(0, 15).forEach((v, i) => btnEls[i] && btnEls[i].classList.toggle('on', !!v));
    src.set(source || 'none', source ? 'good' : 'bad');
    if (!enabled || !source) return;
    seq += 1;
    const t = performance.now();
    const msg = { op: 'joy', axes, buttons, seq: seq % 10 === 0 ? seq : undefined };
    if (msg.seq) sendTimes.set(seq, t);
    if (ctl.send(msg)) { sentCount += 1; sent.set(String(sentCount)); }
  }

  ctl.on('tel', (m) => {
    if (m.teleop && touch) vj.setVehicle(String(m.teleop.data || '').replace(/\s*\(.*$/, ''));
  });

  ctl.on('joy.ack', (m) => {
    const t0 = sendTimes.get(m.seq);
    if (t0 === undefined) return;
    sendTimes.delete(m.seq);
    const v = performance.now() - t0;
    rtt.set(fmt(v, 0, 'ms'), v < 60 ? 'good' : v < 150 ? 'ok' : 'bad');
    rttLine.push(v);
  });

  const restart = () => { clearInterval(timer); timer = setInterval(tick, 1000 / rate); };
  restart();
  setInterval(refreshPads, 2000);

  const warn = h('div', { class: 'muted' });
  const updateWarn = () => {
    warn.textContent = (keyboard || touch) && !enabled ? '⚠ Keyboard/virtual joystick on, but sending is off: nothing reaches the simulator.' : '';
  };
  const setEnabled = (v) => {
    enabled = v;
    if (!v) ctl.send({ op: 'joy', axes: [0, 0, 0, 0, 0, 0], buttons: new Array(15).fill(0) });
    updateWarn();
  };
  const enableTg = toggle({ label: 'Send commands to the simulator (/joy)', value: false, onChange: setEnabled });
  const keyboardTg = toggle({ label: 'Teclado (WASD, R/F, Q/E, Z/X; [ ] , . PgUp/PgDn troca o drone)', value: false, onChange: (v) => {
    keyboard = v;
    if (v && !enabled) { enableTg.set(true); setEnabled(true); }   // turning the keyboard on also turns sending on
    if (!v) keys.clear();
    updateWarn();
  } });
  // Virtual joystick: toggle here and in the header (🎮), handy on a phone.
  const headerBtn = h('button', { class: 'small', id: 'vjoy-btn', title: 'Virtual joystick (touch): drives the drone selected in the Drones tab' }, '🎮');
  const setTouch = (v) => {
    touch = v;
    vj.show(v);
    headerBtn.classList.toggle('active', v);
    if (touchTg.get() !== v) touchTg.set(v);
    if (v && !enabled) { enableTg.set(true); setEnabled(true); }   // like the keyboard: turns sending on
    updateWarn();
  };
  const vj = virtualJoystick(() => setTouch(false));
  const touchTg = toggle({ label: 'On-screen virtual joystick (touch; also via 🎮 in the header)', value: false, onChange: (v) => setTouch(v) });
  headerBtn.addEventListener('click', () => setTouch(!touch));
  const top = document.getElementById('top');
  if (top) top.insertBefore(headerBtn, document.getElementById('toggle-side'));

  return h('div', {},
    section('Sending', enableTg, warn,
      h('p', { class: 'muted' }, 'In the Simulation tab use "Joystick: browser" so it does not conflict with a local USB gamepad. ' +
        'Which drone receives the commands: Drones tab (or LB/RB / [ ]).'),
      src, rtt, rttLine.canvas, sent),
    section('Input', padSel, keyboardTg, touchTg,
      select({ label: 'Layout', options: [['sdl', 'game_controller_node (teleop default)'], ['raw', 'Raw (as delivered by the browser)']], value: 'sdl', onChange: (v) => { layout = v; } }),
      slider({ label: 'Dead zone', min: 0, max: 0.4, step: 0.01, value: deadzone, digits: 2, onChange: (v) => { deadzone = v; } }),
      slider({ label: 'Send rate', min: 10, max: 120, step: 5, value: rate, unit: 'Hz', digits: 0, live: false, onChange: (v) => { rate = v; restart(); } })),
    section('Control state', axesBox, btnBox),
    section('Mapping (drone_joy_teleop)',
      h('ul', { class: 'muted' },
        h('li', {}, 'Right stick: forward/back and sideways'), h('li', {}, 'Left stick: up/down and yaw'),
        h('li', {}, 'LB/RB (keyboard [ ] , . PgUp/PgDn): previous/next vehicle — X4 → X3 → Tiltrotor → X5 → pursuer'),
        h('li', {}, 'LT/RT (keyboard Z/X): decrease/increase speed'),
        h('li', {}, 'Virtual joystick: left stick = up/down and yaw, right = forward/back and sideways; ◀ ▶ switch the drone, speed −/+ = LT/RT'),
        h('li', {}, 'Released sticks publish nothing: the scenario/evader keeps commanding the drone'),
        h('li', {}, 'Fine mapping: Parameters tab → /drone_joy_teleop'))));
}
