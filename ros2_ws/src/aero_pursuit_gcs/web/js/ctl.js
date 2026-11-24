// Control connection (/ws/ctl): requests with replies, server pushes,
// clock synchronization (NTP style) and RTT/jitter measurement.

const wallNow = () => performance.timeOrigin + performance.now();

export class Ctl {
  constructor() {
    this.handlers = new Map();
    this.pending = new Map();
    this.nextId = 1;
    this.ws = null;
    this.connected = false;
    this.offset = 0;              // server clock - local clock [ms]
    this.rtt = null;
    this.jitter = null;
    this.rttHist = [];
    this.samples = [];            // (rtt, offset) recentes
    this.failures = 0;
    this.connect();
    setInterval(() => this.ping(), 500);
  }

  url() {
    return (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws/ctl';
  }

  connect() {
    const ws = new WebSocket(this.url());
    this.ws = ws;
    ws.onopen = () => {
      this.connected = true;
      this.failures = 0;
      this.emit('open', {});
      this.ping();
    };
    ws.onmessage = (ev) => this.onMessage(ev.data);
    ws.onclose = async () => {
      const was = this.connected;
      this.connected = false;
      for (const [, p] of this.pending) p.reject(new Error('connection lost'));
      this.pending.clear();
      if (was) this.emit('close', {});
      this.failures += 1;
      if (this.failures >= 2) {
        // 401 on upgrade => session expired: back to login.
        try {
          const r = await fetch('/', { redirect: 'manual' });
          if (r.type === 'opaqueredirect' || r.status === 302) { location.href = '/login'; return; }
        } catch (e) { /* server down */ }
      }
      setTimeout(() => this.connect(), Math.min(5000, 500 * this.failures));
    };
  }

  on(op, fn) {
    if (!this.handlers.has(op)) this.handlers.set(op, []);
    this.handlers.get(op).push(fn);
  }

  emit(op, msg) {
    for (const fn of this.handlers.get(op) || []) {
      try { fn(msg); } catch (e) { console.error(op, e); }
    }
  }

  onMessage(text) {
    let m;
    try { m = JSON.parse(text); } catch (e) { return; }
    if (m.op === 'result') {
      const p = this.pending.get(m.id);
      if (p) {
        this.pending.delete(m.id);
        clearTimeout(p.timer);
        m.ok ? p.resolve(m.data) : p.reject(new Error(m.error));
      }
      return;
    }
    if (m.op === 'pong') { this.onPong(m); return; }
    this.emit(m.op, m);
  }

  send(obj) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(obj));
      return true;
    }
    return false;
  }

  /// Request with a reply (Promise). Rejects on server error or timeout.
  req(op, args = {}, timeout = 10000) {
    return new Promise((resolve, reject) => {
      const id = this.nextId++;
      const timer = setTimeout(() => {
        this.pending.delete(id);
        reject(new Error(`${op}: tempo esgotado`));
      }, timeout);
      this.pending.set(id, { resolve, reject, timer });
      if (!this.send({ op, id, ...args })) {
        clearTimeout(timer);
        this.pending.delete(id);
        reject(new Error('desconectado'));
      }
    });
  }

  ping() {
    if (this.connected) this.send({ op: 'ping', t: wallNow() });
  }

  onPong(m) {
    const t3 = wallNow();
    const rtt = t3 - m.t;
    if (rtt < 0 || rtt > 60000) return;
    // Offset from the recent sample with the lowest RTT: the least affected by queueing.
    this.samples.push([rtt, m.ts - (m.t + rtt / 2)]);
    if (this.samples.length > 20) this.samples.shift();
    const best = this.samples.reduce((a, b) => (b[0] < a[0] ? b : a));
    this.offset = best[1];
    if (this.rtt !== null) {
      const d = Math.abs(rtt - this.rtt);
      this.jitter = this.jitter === null ? d : this.jitter * 0.8 + d * 0.2;
    }
    this.rtt = rtt;
    this.rttHist.push(rtt);
    if (this.rttHist.length > 120) this.rttHist.shift();
    this.emit('rtt', { rtt, jitter: this.jitter, offset: this.offset });
  }

  /// Current time on the server clock [ms since the epoch].
  serverNow() { return wallNow() + this.offset; }
}
