// Network tab: connection quality (with history), remote access (Cloudflare tunnel,
// rede local, token) e sistema (CPU/GPU/RTF).

import { h, button, section, slider, stat, toggle, textInput, toast, fmt, grade, Sparkline } from './ui.js';

async function copy(text) {
  try { await navigator.clipboard.writeText(text); toast('Copied', 'good'); } catch (e) { prompt('Copy:', text); }
}

export function networkPanel(ctl, state, video) {
  const s = {
    rtt: stat('Control RTT'), jitter: stat('Jitter'), lat: stat('Video: capture → screen'),
    p95: stat('Video: p95'), enc: stat('· encoding'), net: stat('· network + queue'), dec: stat('· decoding + drawing'),
    sim: stat('· Gazebo → streamer (sim time)'), fps: stat('Frames shown'), kbps: stat('Video bandwidth'),
    drops: stat('Drops (server)'), ack: stat('Delivery delay (ack)'), queue: stat('Decoder queue'),
  };
  const sparks = { rtt: new Sparkline(120, { color: '#81c784' }), lat: new Sparkline(120, { color: '#4fc3f7' }), kbps: new Sparkline(120, { color: '#ffb74d' }) };

  ctl.on('rtt', ({ rtt, jitter }) => {
    s.rtt.set(fmt(rtt, 0, 'ms'), grade(rtt, 60, 150));
    s.jitter.set(fmt(jitter, 1, 'ms'), grade(jitter, 10, 30));
    sparks.rtt.push(rtt);
  });
  video.onStats((st, vs) => {
    s.lat.set(fmt(st.latency, 0, 'ms'), grade(st.latency, 120, 250));
    s.p95.set(fmt(st.latencyP95, 0, 'ms'), grade(st.latencyP95, 180, 350));
    s.enc.set(fmt(st.enc, 1, 'ms'));
    s.net.set(fmt(st.net, 1, 'ms'));
    s.dec.set(fmt(st.dec, 1, 'ms'));
    s.sim.set(fmt(st.simAge, 1, 'ms'));
    s.fps.set(`${fmt(st.drawFps, 1)} of ${fmt(st.recvFps, 1)} received`);
    s.kbps.set(fmt(st.kbps / 1000, 2, 'Mbit/s'));
    s.queue.set(String(st.queue), st.queue > 2 ? 'bad' : '');
    sparks.lat.push(st.latency);
    sparks.kbps.push(st.kbps / 1000);
    if (vs) {
      s.drops.set(`${fmt(vs.drop_fps, 1)}/s (total ${vs.drops_total})`, vs.drop_fps > 0 ? 'bad' : 'good');
      s.ack.set(`${fmt(vs.ack_ms, 0, 'ms')} (p95 ${fmt(vs.ack_p95_ms, 0)}) | ${vs.inflight} in flight`, grade(vs.ack_p95_ms, 150, 400));
    }
  });

  // Acesso.
  const tState = stat('Tunnel');
  const tUrl = h('a', { target: '_blank', class: 'mono' });
  const token = h('code', { class: 'secret' }, '••••••');
  let tokenValue = '', shown = false;
  const lanBox = h('div', { class: 'mono' });
  const namedToken = textInput({ label: 'Named tunnel token (optional, fixed URL)', type: 'password',
    onChange: (v) => ctl.req('tunnel.config', { named_token: v }).catch(() => {}) });
  const hostname = textInput({ label: 'Named tunnel hostname', placeholder: 'gcs.yourdomain.com',
    onChange: (v) => ctl.req('tunnel.config', { hostname: v }).catch(() => {}) });
  const autostart = toggle({ label: 'Start the tunnel together with the GCS', onChange: (v) => ctl.req('tunnel.config', { autostart: v }).catch(() => {}) });
  const tStart = button('Start tunnel', () => ctl.req('tunnel.start').catch((e) => toast(e.message, 'error')), 'primary');
  const tStop = button('Stop', () => ctl.req('tunnel.stop').catch(() => {}), 'danger');
  const tMsg = h('div', { class: 'muted' });

  ctl.on('tunnel.status', (t) => {
    const labels = { stopped: 'stopped', installing: 'installing', starting: 'connecting', running: 'active', error: 'error' };
    tState.set(labels[t.state] || t.state, t.state === 'running' ? 'good' : t.state === 'error' ? 'bad' : '');
    tUrl.textContent = t.url || '';
    tUrl.href = t.url || '#';
    tMsg.textContent = t.message + (t.installed ? '' : ' (cloudflared is downloaded on first use)');
    tStart.disabled = t.state === 'running' || t.state === 'starting' || t.state === 'installing';
    hostname.set(t.hostname);
    autostart.set(t.autostart);
  });
  const helloHandler = (hello) => {
    tokenValue = hello.token;
    lanBox.innerHTML = '';
    lanBox.append(h('div', {}, hello.urls.local + '  (this computer)'));
    for (const u of hello.urls.lan) lanBox.append(h('div', {}, u));
    localNoAuth.set(hello.local_no_auth);
    telHz.set(hello.telemetry_hz);
  };
  const localNoAuth = toggle({ label: 'No token on this machine (localhost)', onChange: (v) => ctl.req('settings.set', { local_no_auth: v }) });
  const telHz = slider({ label: 'Telemetry rate (TF, state)', min: 5, max: 60, step: 1, value: 30, unit: 'Hz', digits: 0, live: false,
    onChange: (v) => ctl.req('settings.set', { telemetry_hz: v }) });

  // Sistema.
  const sys = { cpu: stat('CPU'), mem: stat('Memory'), gpu: stat('GPU'), vram: stat('VRAM'), rtf: stat('Real time (RTF)') };
  ctl.on('sys', (m) => {
    sys.cpu.set(`${fmt(m.cpu, 0)}% of ${m.cores} cores | load ${fmt(m.load, 1)}`, grade(m.cpu, 60, 85));
    sys.mem.set(`${fmt(m.mem_used_gb, 1)} / ${fmt(m.mem_total_gb, 1)} GB`);
    sys.gpu.set(m.gpu !== undefined ? `${m.gpu}%` : 'n/d', grade(m.gpu, 70, 90));
    sys.vram.set(m.vram_used_gb !== undefined ? `${fmt(m.vram_used_gb, 1)} / ${fmt(m.vram_total_gb, 1)} GB` : 'n/d');
  });
  ctl.on('tel', (m) => { if (m.rtf !== undefined && m.rtf !== null) sys.rtf.set(fmt(m.rtf, 2), grade(m.rtf, 0.95, 0.7, true)); });

  return {
    el: h('div', {},
      section('Connection quality', s.rtt, s.jitter, sparks.rtt.canvas, s.lat, s.p95, sparks.lat.canvas, s.enc, s.net, s.dec, s.sim,
        s.fps, s.kbps, sparks.kbps.canvas, s.drops, s.ack, s.queue),
      section('Remote access (Cloudflare Tunnel, no VPN)', tState, tUrl,
        h('div', { class: 'row wrap' }, tStart, tStop,
          button('Copy link', () => tUrl.textContent && copy(tUrl.textContent), 'small'),
          button('Copy link with token', () => tUrl.textContent && copy(`${tUrl.textContent}/?token=${tokenValue}`), 'small')),
        tMsg, autostart, h('details', {}, h('summary', {}, 'Named tunnel (fixed URL)'),
          h('p', { class: 'muted' }, 'Create a tunnel at one.dash.cloudflare.com (free account + domain), point it to http://localhost:8080 and paste the token here.'),
          namedToken, hostname)),
      section('Local network', lanBox, h('p', { class: 'muted' }, 'Self-signed HTTPS: accept the browser warning the first time (WebCodecs and Gamepad require HTTPS outside localhost).')),
      section('Access token', h('div', { class: 'row wrap' }, token,
        button('Show', () => { shown = !shown; token.textContent = shown ? tokenValue : '••••••'; }, 'small'),
        button('Copy', () => copy(tokenValue), 'small'),
        button('Generate new', async () => {
          if (!confirm('Generate a new token? Old links and remote sessions stop working.')) return;
          const r = await ctl.req('token.regen');
          tokenValue = r.token;
          token.textContent = shown ? tokenValue : '••••••';
          toast('New token generated', 'good');
        }, 'small danger')), localNoAuth),
      section('Server', telHz),
      section('System', sys.cpu, sys.mem, sys.gpu, sys.vram, sys.rtf)),
    onHello: helloHandler,
  };
}
