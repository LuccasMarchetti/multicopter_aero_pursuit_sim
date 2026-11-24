// Video tab: source, codec, resolution, fps, bitrate, rate control, latency x
// smoothness and overlays. Everything applied live in the encoder.

import { h, button, select, section, slider, stat, toggle, fmt, debounce } from './ui.js';

async function codecSupport() {
  const out = { h264: false, h265: false, mjpeg: true };
  if (typeof VideoDecoder === 'undefined') return out;
  for (const [k, codec] of [['h264', 'avc1.640033'], ['h265', 'hev1.1.6.L153.B0']]) {
    try { out[k] = (await VideoDecoder.isConfigSupported({ codec, optimizeForLatency: true })).supported; } catch (e) { /* */ }
  }
  return out;
}

export function streamPanel(ctl, state, video) {
  const pending = {};
  const flush = debounce(() => {
    const cfg = { ...pending };
    for (const k of Object.keys(pending)) delete pending[k];
    ctl.req('stream.set', { cfg }).catch(() => {});
  }, 120);
  const set = (k) => (v) => { pending[k] = v; flush(); };

  const topic = select({ label: 'Source', options: ['/camera', '/camera/segmented_bb', '/depth_camera'], onChange: set('topic') });
  const codec = select({ label: 'Codec', options: [['h264', 'H.264'], ['h265', 'H.265 / HEVC'], ['mjpeg', 'MJPEG']], onChange: (v) => { set('codec')(v); vis(); } });
  const backend = select({ label: 'Encoder', options: [['auto', 'Automatic (GPU if available)'], ['hardware', 'GPU (VA-API or NVENC)'], ['software', 'CPU']], onChange: set('backend') });
  const resText = h('span', { class: 'muted' });
  const scale = slider({ label: 'Resolution', min: 10, max: 100, step: 1, value: 100, unit: '%', digits: 0, live: false,
    onChange: (v) => set('scale')(v / 100) });
  const fps = slider({ label: 'Max FPS (0 = no limit)', min: 0, max: 60, step: 1, value: 30, digits: 0, live: false, onChange: set('fps') });
  const bitrate = slider({ label: 'Bitrate (cap)', min: 0.3, max: 50, step: 0.1, value: 8, unit: 'Mbit/s', digits: 1, live: false,
    onChange: (v) => set('bitrate')(Math.round(v * 1000)) });
  const adaptive = toggle({ label: 'Adaptive bitrate (follows link bandwidth)', onChange: set('adaptive') });
  const minBitrate = slider({ label: 'Minimum bitrate (adaptive)', min: 0.1, max: 10, step: 0.1, value: 0.5, unit: 'Mbit/s', digits: 1, live: false,
    onChange: (v) => set('min_bitrate')(Math.round(v * 1000)) });
  const rc = select({ label: 'Rate control', options: [['cbr', 'CBR (constant)'], ['vbr', 'VBR (variable)'], ['cqp', 'CQP (fixed quality)']], onChange: (v) => { set('rc')(v); vis(); } });
  const qp = slider({ label: 'QP (menor = melhor)', min: 10, max: 51, step: 1, value: 26, digits: 0, live: false, onChange: set('qp') });
  const jpegQ = slider({ label: 'JPEG quality', min: 10, max: 100, step: 1, value: 85, digits: 0, live: false, onChange: set('jpeg_quality') });
  const gop = slider({ label: 'Keyframe interval (frames)', min: 1, max: 120, step: 1, value: 30, digits: 0, live: false, onChange: set('gop'),
    title: 'Smaller: recovers faster from losses; larger: fewer bits per frame' });
  const inflight = slider({ label: 'Frames in flight (latency ↔ smoothness)', min: 1, max: 8, step: 1, value: 2, digits: 0, live: false, onChange: set('max_inflight'),
    title: '1-2: lowest latency (drops early); 4-8: smoother on unstable links' });
  const dmin = slider({ label: 'Min depth', min: 0, max: 10, step: 0.1, value: 0.2, unit: 'm', digits: 1, live: false, onChange: set('depth_min') });
  const dmax = slider({ label: 'Max depth', min: 1, max: 100, step: 0.5, value: 20, unit: 'm', digits: 1, live: false, onChange: set('depth_max') });
  const depthBox = h('div', {}, dmin, dmax);

  const vis = () => {
    const c = codec.get(), r = rc.get();
    qp.style.display = c !== 'mjpeg' && r === 'cqp' ? '' : 'none';
    jpegQ.style.display = c === 'mjpeg' ? '' : 'none';
    rc.style.display = c === 'mjpeg' ? 'none' : '';
    bitrate.style.display = c === 'mjpeg' || r === 'cqp' ? 'none' : '';
    adaptive.style.display = bitrate.style.display;
    minBitrate.style.display = bitrate.style.display;
    gop.style.display = c === 'mjpeg' ? 'none' : '';
    backend.style.display = c === 'mjpeg' ? 'none' : '';
    depthBox.style.display = topic.get().includes('depth') ? '' : 'none';
  };
  topic.input.addEventListener('change', vis);

  const encInfo = stat('Encoder');
  const encOut = stat('Output');
  const encIn = stat('Streamer input');
  const encTime = stat('Encoding time');
  const effBr = stat('Effective bitrate');
  const viewers = stat('Viewers');

  const updateRes = () => {
    const s = video.size;
    if (s) resText.textContent = `${Math.round(s.srcW * scale.querySelector('input').value / 100)}×${Math.round(s.srcH * scale.querySelector('input').value / 100)} de ${s.srcW}×${s.srcH}`;
  };
  scale.querySelector('input').addEventListener('input', updateRes);

  ctl.on('stream.state', (s) => {
    if ((s.channel || 'main') !== 'main') return;   // free camera: configured in the Gazebo view
    const c = s.cfg;
    topic.set(c.topic); codec.set(c.codec); backend.set(c.backend); scale.set(Math.round(c.scale * 100));
    fps.set(c.fps); bitrate.set(c.bitrate / 1000); adaptive.set(c.adaptive); minBitrate.set(c.min_bitrate / 1000);
    rc.set(c.rc); qp.set(c.qp); jpegQ.set(c.jpeg_quality); gop.set(c.gop); inflight.set(c.max_inflight);
    dmin.set(c.depth_min); dmax.set(c.depth_max);
    vis(); updateRes();
    if (!s.connected) encInfo.set('streamer offline', 'bad');
    else if (s.error) encInfo.set(s.error, 'bad');
    else if (s.info && s.info.codec) encInfo.set(`${s.info.codec} (${Number(s.info.hardware) ? 'GPU' : 'CPU'})`, 'good');
    else encInfo.set(s.viewers ? 'starting...' : 'idle (no viewer)');
    if (s.info && s.info.width) encOut.set(`${s.info.width}×${s.info.height} of ${s.info.src_width}×${s.info.src_height}`);
    encIn.set(`${fmt(s.in_fps, 1)} fps | ${fmt(s.in_kbps / 1000, 2)} Mbit/s`);
    encTime.set(fmt(s.encode_ms, 1, 'ms'), s.encode_ms === null ? '' : s.encode_ms < 20 ? 'good' : s.encode_ms < 40 ? 'ok' : 'bad');
    effBr.set(c.codec === 'mjpeg' || c.rc === 'cqp' ? '—' : `${fmt(s.effective_bitrate / 1000, 2)} Mbit/s`);
    viewers.set(String(s.viewers));
  });

  codecSupport().then((sup) => {
    codec.setOptions([['h264', 'H.264' + (sup.h264 ? '' : ' (not supported)'), !sup.h264],
      ['h265', 'H.265 / HEVC' + (sup.h265 ? '' : ' (not supported by this browser)'), !sup.h265], ['mjpeg', 'MJPEG']]);
    if (state.stream) codec.set(state.stream.cfg.codec);
  });

  // Image sources available in the graph.
  const refreshTopics = async () => {
    try {
      const topics = await ctl.req('topics');
      const imgs = topics.filter((t) => t.types.includes('sensor_msgs/msg/Image')).map((t) => t.name);
      const base = ['/camera', '/camera/segmented_bb', '/depth_camera'];
      topic.setOptions([...new Set([...base, ...imgs])]);
      if (state.stream) topic.set(state.stream.cfg.topic);
    } catch (e) { /* */ }
  };
  ctl.on('open', refreshTopics);

  const ov = video.options;
  const enable = toggle({ label: 'Receive video in this browser', value: true, onChange: (v) => video.setEnabled(v) });

  return h('div', {},
    section('Stream state', encInfo, encOut, encIn, encTime, effBr, viewers),
    section('Source and codec', enable, h('div', { class: 'row' }, topic, button('↻', refreshTopics, 'small', 'Refresh topics')), codec, backend, depthBox),
    section('Quality', scale, resText, fps, rc, bitrate, adaptive, minBitrate, qp, jpegQ, gop),
    section('Latency', inflight, button('Request keyframe', () => { video.requestKey(); ctl.req('stream.set', { cfg: {} }).catch(() => {}); }, 'small')),
    section('Overlay',
      toggle({ label: 'Tracker bounding box', value: ov.bbox, onChange: (v) => { ov.bbox = v; } }),
      toggle({ label: 'Center crosshair', value: ov.crosshair, onChange: (v) => { ov.crosshair = v; } }),
      toggle({ label: 'HUD (mode, fps, latency)', value: ov.hud, onChange: (v) => { ov.hud = v; } })));
}
