// Video worker: binary WebSocket -> WebCodecs (H.264/H.265) or JPEG -> OffscreenCanvas.
// Off the main thread: 3D/DOM rendering does not delay decoding or the ack.
//
// 64-byte header (aero_pursuit/streaming/frame_packet.hpp), little-endian.

const HDR = 64;
const CODECS = { 1: 'h264', 2: 'h265', 3: 'mjpeg' };
const DEFAULT_CODEC = { 1: 'avc1.640033', 2: 'hev1.1.6.L153.B0' };
const now = () => performance.timeOrigin + performance.now();

let canvas = null, ctx = null, ws = null, url = '';
let offset = 0;              // server clock - local
let enabled = true;
let codecStrings = {};       // id -> string announced by the streamer
let decoder = null, decoderKey = '', needKey = true, lastKeyReq = 0;
let lastDrawnSeq = -1;
const meta = new Map();      // timestamp (µs) -> frame metadata
let stats = newStats();
let lastSize = '';
let wasSupported = null;
const support = {};          // codec string -> true | false | 'checking'
const failsNoOutput = {};    // codec -> consecutive failures without any decoded frame
let lastUnsupportedMsg = 0;

const NAMES = { 1: 'H.264', 2: 'H.265' };
function unsupported(codecId, codec) {
  // Repeated every 3 s so the warning stays visible while the codec does not change.
  const t = performance.now();
  if (t - lastUnsupportedMsg < 3000) return;
  lastUnsupportedMsg = t;
  const hint = codecId === 2
    ? 'Chrome only decodes H.265 with GPU video acceleration; use H.264 or MJPEG in the Video tab'
    : 'use another codec in the Video tab';
  post({ type: 'error', text: `This browser cannot decode ${NAMES[codecId] || codec} (${codec}): ${hint}` });
}

function checkSupport(codec) {
  support[codec] = 'checking';
  VideoDecoder.isConfigSupported({ codec, optimizeForLatency: true })
    .then((r) => { support[codec] = !!r.supported; })
    .catch(() => { support[codec] = false; })
    .finally(() => requestKey());
}

function newStats() {
  return { frames: 0, bytes: 0, drawn: 0, skipped: 0, decErrors: 0, lat: [], net: [], enc: [], dec: [], sim: [] };
}

function connect() {
  ws = new WebSocket(url);
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => {
    post({ type: 'conn', connected: true });
    ws.send(JSON.stringify({ op: 'enable', on: enabled }));
    needKey = true;
  };
  ws.onclose = () => {
    post({ type: 'conn', connected: false });
    resetDecoder();
    setTimeout(connect, 1000);
  };
  ws.onmessage = (ev) => {
    if (typeof ev.data === 'string') {
      try { post({ type: 'vstats', data: JSON.parse(ev.data) }); } catch (e) { /* */ }
      return;
    }
    onPacket(ev.data);
  };
}

function post(m) { self.postMessage(m); }

function requestKey() {
  const t = performance.now();
  if (t - lastKeyReq > 300 && ws && ws.readyState === 1) {
    lastKeyReq = t;
    ws.send(JSON.stringify({ op: 'key' }));
  }
}

function resetDecoder() {
  if (decoder && decoder.state !== 'closed') { try { decoder.close(); } catch (e) { /* */ } }
  decoder = null;
  decoderKey = '';
  needKey = true;
  meta.clear();
}

function onPacket(buf) {
  const dv = new DataView(buf);
  if (buf.byteLength < HDR || dv.getUint32(0, true) !== 0x31564741 || dv.getUint8(4) !== 1) return;
  const arrival = now();
  const key = (dv.getUint8(5) & 1) === 1;
  const codecId = dv.getUint8(6);
  const w = dv.getUint32(8, true), hgt = dv.getUint32(12, true);
  const srcW = dv.getUint32(16, true), srcH = dv.getUint32(20, true);
  const seq = dv.getUint32(24, true), len = dv.getUint32(28, true);
  const recvMs = Number(dv.getBigInt64(40, true)) / 1e6;
  const encMs = Number(dv.getBigInt64(48, true)) / 1e6;
  const simAgeMs = Number(dv.getBigInt64(56, true)) / 1e6;
  // Immediate ack: the server measures the queue delay and does not send more than the link can take.
  ws.send(`{"op":"ack","seq":${seq}}`);
  stats.frames += 1;
  stats.bytes += buf.byteLength;

  const sizeKey = `${w}x${hgt}/${srcW}x${srcH}/${codecId}`;
  if (sizeKey !== lastSize) {
    lastSize = sizeKey;
    post({ type: 'size', w, h: hgt, srcW, srcH, codec: CODECS[codecId] });
  }
  const m = { seq, arrival, recvMs, encMs, simAgeMs, key };
  const data = new Uint8Array(buf, HDR, len);

  if (codecId === 3) { decodeJpeg(data, m); return; }

  if (typeof VideoDecoder === 'undefined') {
    if (wasSupported !== false) {
      wasSupported = false;
      post({ type: 'error', text: 'WebCodecs unavailable (use HTTPS or localhost, or the MJPEG codec)' });
    }
    return;
  }
  const codec = codecStrings[codecId] || DEFAULT_CODEC[codecId];
  const want = `${codec}`;
  if (!decoder || decoder.state === 'closed' || decoderKey !== want) {
    if (support[codec] === undefined) checkSupport(codec);
    if (support[codec] === 'checking') { stats.skipped += 1; return; }
    if (support[codec] === false) { stats.skipped += 1; unsupported(codecId, codec); return; }
    if (!key) { stats.skipped += 1; requestKey(); return; }
    resetDecoder();
    decoder = new VideoDecoder({
      output: (frame) => { failsNoOutput[codec] = 0; draw(frame, meta.get(frame.timestamp)); },
      error: (e) => {
        stats.decErrors += 1;
        // isConfigSupported() may say "yes" and configure fail later (e.g. GPU acceleration
        // advertised but unavailable). Consecutive failures without any frame = not supported.
        failsNoOutput[codec] = (failsNoOutput[codec] || 0) + 1;
        if (e.name === 'NotSupportedError' || failsNoOutput[codec] >= 2) {
          support[codec] = false; unsupported(codecId, codec); resetDecoder(); return;
        }
        // An isolated error (e.g. a configuration change in the middle of the GOP) resolves itself at the
        // next keyframe; only warns if it repeats.
        if (stats.decErrors > 1) post({ type: 'error', text: `decodificador: ${e.message}` });
        resetDecoder();
        requestKey();
      },
    });
    try {
      decoder.configure({ codec, optimizeForLatency: true, hardwareAcceleration: 'no-preference' });
    } catch (e) {
      post({ type: 'error', text: `codec ${codec} not supported: ${e.message}` });
      resetDecoder();
      return;
    }
    decoderKey = want;
  }
  // Late decoder: drops until the next keyframe instead of accumulating latency.
  if (decoder.decodeQueueSize > 2 && !key) { needKey = true; }
  if (needKey && !key) { stats.skipped += 1; requestKey(); return; }
  needKey = false;
  const ts = seq * 1000;
  meta.set(ts, m);
  if (meta.size > 120) meta.delete(meta.keys().next().value);
  try {
    decoder.decode(new EncodedVideoChunk({ type: key ? 'key' : 'delta', timestamp: ts, data }));
  } catch (e) {
    stats.decErrors += 1;
    resetDecoder();
    requestKey();
  }
}

async function decodeJpeg(data, m) {
  try {
    const bmp = await createImageBitmap(new Blob([data], { type: 'image/jpeg' }));
    if (m.seq < lastDrawnSeq && lastDrawnSeq - m.seq < 1e6) { bmp.close(); stats.skipped += 1; return; }
    draw(bmp, m);
  } catch (e) {
    stats.decErrors += 1;
  }
}

function draw(img, m) {
  const w = img.displayWidth || img.width, hgt = img.displayHeight || img.height;
  if (canvas.width !== w || canvas.height !== hgt) { canvas.width = w; canvas.height = hgt; }
  ctx.drawImage(img, 0, 0);
  img.close();
  if (!m) return;
  lastDrawnSeq = m.seq;
  const t = now();
  stats.drawn += 1;
  // All times on the server clock (offset estimated by the control ping).
  stats.lat.push(t + offset - m.recvMs);
  stats.net.push(m.arrival + offset - m.encMs);
  stats.enc.push(m.encMs - m.recvMs);
  stats.dec.push(t - m.arrival);
  if (Number.isFinite(m.simAgeMs) && Math.abs(m.simAgeMs) < 1e5) stats.sim.push(m.simAgeMs);
}

function median(a) { if (!a.length) return null; const s = [...a].sort((x, y) => x - y); return s[s.length >> 1]; }
function p95(a) { if (!a.length) return null; const s = [...a].sort((x, y) => x - y); return s[Math.min(s.length - 1, Math.floor(s.length * 0.95))]; }

let lastReport = performance.now();
setInterval(() => {
  const t = performance.now();
  const dt = (t - lastReport) / 1000;
  lastReport = t;
  post({
    type: 'stats',
    recvFps: stats.frames / dt,
    drawFps: stats.drawn / dt,
    kbps: stats.bytes * 8 / 1000 / dt,
    skipped: stats.skipped,
    decErrors: stats.decErrors,
    latency: median(stats.lat), latencyP95: p95(stats.lat),
    net: median(stats.net), enc: median(stats.enc), dec: median(stats.dec), simAge: median(stats.sim),
    queue: decoder ? decoder.decodeQueueSize : 0,
  });
  stats = newStats();
}, 500);

self.onmessage = (ev) => {
  const m = ev.data;
  if (m.type === 'init') {
    canvas = m.canvas;
    ctx = canvas.getContext('2d', { alpha: false, desynchronized: true });
    url = m.url;
    connect();
  } else if (m.type === 'offset') {
    offset = m.value;
  } else if (m.type === 'enable') {
    enabled = m.on;
    if (ws && ws.readyState === 1) ws.send(JSON.stringify({ op: 'enable', on: enabled }));
    if (!enabled) resetDecoder();
  } else if (m.type === 'codecs') {
    codecStrings = m.codecs;
  } else if (m.type === 'key') {
    needKey = true;
    requestKey();
  }
};
