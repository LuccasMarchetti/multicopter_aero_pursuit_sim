// Session recording: the browser records this GCS tab (video, 3D, HUD, panels) with
// getDisplayMedia + MediaRecorder and downloads the file when it stops. Nothing runs on the
// server: the same in Gazebo and Isaac Sim. The browser asks which tab/window to share (the
// current tab comes first); recording needs a secure context (https or localhost), like the
// gamepad. The simulation data of the same run is in datasets/<session> (session_recorder).

import { h, toast } from './ui.js';

const TYPES = ['video/mp4;codecs=avc1', 'video/webm;codecs=vp9', 'video/webm;codecs=vp8', 'video/webm'];

export function recorderButton(state) {
  let rec = null, stream = null, chunks = [], t0 = 0, timer = null;
  const btn = h('button', { class: 'small', title: 'Record this GCS tab as a video' }, '⏺ Record');

  function label() {
    const s = Math.floor((Date.now() - t0) / 1000);
    btn.textContent = `⏹ ${String(Math.floor(s / 60)).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;
  }

  function fileName(ext) {
    const d = new Date();
    const p = (n) => String(n).padStart(2, '0');
    const sim = (state.sim && state.sim.backend) || 'gcs';
    return `aero_${sim}_${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}.${ext}`;
  }

  function save() {
    const type = rec.mimeType || 'video/webm';
    const blob = new Blob(chunks, { type });
    const a = h('a', { href: URL.createObjectURL(blob), download: fileName(type.includes('mp4') ? 'mp4' : 'webm') });
    document.body.append(a);
    a.click();
    setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 2000);
    toast(`Recording saved (${(blob.size / 1e6).toFixed(1)} MB)`, 'good');
  }

  function stop() {
    if (rec && rec.state !== 'inactive') rec.stop();
    if (stream) stream.getTracks().forEach((t) => t.stop());
    clearInterval(timer);
    btn.classList.remove('danger');
    btn.textContent = '⏺ Record';
  }

  async function start() {
    if (!navigator.mediaDevices || !navigator.mediaDevices.getDisplayMedia) {
      toast('Recording unavailable: use https://<ip>:8443 or localhost (secure context)', 'error', 6000);
      return;
    }
    try {
      stream = await navigator.mediaDevices.getDisplayMedia({
        video: { frameRate: 30, displaySurface: 'browser' }, audio: false,
        preferCurrentTab: true, selfBrowserSurface: 'include' });
    } catch (e) {
      toast(`Recording cancelled (${e.message})`, 'error');
      return;
    }
    const mimeType = TYPES.find((t) => window.MediaRecorder && MediaRecorder.isTypeSupported(t)) || '';
    chunks = [];
    rec = new MediaRecorder(stream, { mimeType, videoBitsPerSecond: 8e6 });
    rec.ondataavailable = (e) => { if (e.data.size) chunks.push(e.data); };
    rec.onstop = save;
    stream.getVideoTracks()[0].addEventListener('ended', stop);   // the browser's "Stop sharing"
    rec.start(1000);
    t0 = Date.now();
    btn.classList.add('danger');
    label();
    timer = setInterval(label, 1000);
    toast('Recording: click ⏹ to stop and save', 'good');
  }

  btn.addEventListener('click', () => (rec && rec.state === 'recording' ? stop() : start()));
  return btn;
}
