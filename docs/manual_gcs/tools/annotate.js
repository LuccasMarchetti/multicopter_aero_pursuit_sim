// Draws numbered callouts with arrows over real DOM elements (manual screenshots).
window.__find = (q) => {
  const fake = (x, y, w, h) => ({ getBoundingClientRect: () => ({ left: x, top: y, right: x + w, bottom: y + h, width: w, height: h }) });
  if (q.rect) return fake(...q.rect);
  if (q.obj !== undefined) {
    const sc = window.gcs.scene; const cv = sc.renderer.domElement.getBoundingClientRect();
    let v;
    if (q.obj === 'estimate') v = sc.estimate.position.clone();
    else if (q.obj === 'origin') v = sc.estimate.position.clone().set(0, 0, 0);
    else if (q.obj === 'path' || q.obj === 'los') {
      const line = q.obj === 'path' ? sc.pathLine : sc.los; const a = line.geometry.attributes.position.array;
      const n = line.geometry.drawRange.count; if (!line.visible || n < 2) return null;
      const i = q.obj === 'path' ? n - 1 : 0, j = q.obj === 'path' ? n - 1 : 1;
      v = sc.estimate.position.clone().set((a[i*3]+a[j*3])/2, (a[i*3+1]+a[j*3+1])/2, (a[i*3+2]+a[j*3+2])/2);
    } else { const o = sc.objects[q.obj]; if (!o) return null; v = o.position.clone(); }
    v.project(sc.camera);
    if (v.z > 1) return null;
    const x = cv.left + (v.x + 1) / 2 * cv.width, y = cv.top + (1 - v.y) / 2 * cv.height, s = q.size || 14;
    return fake(x - s, y - s, 2 * s, 2 * s);
  }
  if (q.hud) {   // HUD lines (video.js): 12px monospace at the image corner, 17 px per line
    const d = window.gcs.state.detection || {}; const r = document.getElementById('overlay').getBoundingClientRect();
    const iw = d.image_width || 16, ih = d.image_height || 9, k = Math.min(r.width / iw, r.height / ih);
    const ox = r.left + (r.width - iw * k) / 2, oy = r.top + (r.height - ih * k) / 2;
    const c = document.createElement('canvas').getContext('2d'); c.font = '12px monospace';
    const w = Math.max(c.measureText('TRACKING | asmc | d=5.7 m').width, c.measureText('1920x1080 h264 | 14 fps | 43 ms').width) + 8;
    return fake(ox + 6, oy + 6, w, 33);
  }
  if (q.det) {   // tracker box from the telemetry, mapped through the letterbox of the video
    const d = window.__det || window.gcs.state.detection; const r = document.getElementById('overlay').getBoundingClientRect();
    if (!d || !d.image_width) return null;
    const k = Math.min(r.width / d.image_width, r.height / d.image_height);
    const ox = r.left + (r.width - d.image_width * k) / 2, oy = r.top + (r.height - d.image_height * k) / 2;
    return fake(ox + d.x * k, oy + d.y * k - 18, d.width * k, d.height * k + 18);   // includes the label
  }
  if (q.overlay) {   // bounding box of pixels of a colour in the video overlay canvas
    const c = document.getElementById('overlay'); const r = c.getBoundingClientRect();
    const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
    let x0 = 1e9, y0 = 1e9, x1 = -1, y1 = -1;
    for (let y = 0; y < c.height; y++) for (let x = 0; x < c.width; x++) {
      const k = (y * c.width + x) * 4;
      if (d[k + 3] > 200 && q.overlay.some(([R, G, B]) => Math.abs(d[k] - R) < 12 && Math.abs(d[k + 1] - G) < 12 && Math.abs(d[k + 2] - B) < 12)) {
        x0 = Math.min(x0, x); y0 = Math.min(y0, y); x1 = Math.max(x1, x); y1 = Math.max(y1, y);
      }
    }
    if (x1 < 0) return null;
    const sx = r.width / c.width, sy = r.height / c.height;
    return fake(r.left + x0 * sx, r.top + y0 * sy, (x1 - x0) * sx, (y1 - y0) * sy);
  }
  if (q.sel) {
    const all = [...document.querySelectorAll(q.sel)].filter((e) => e.getClientRects().length);
    return all[q.nth || 0] || null;
  }
  const root = q.within ? document.querySelector(q.within) : document.body;
  if (!root) return null;
  let best = null, area = Infinity;
  for (const e of root.querySelectorAll(q.tag || '*')) {
    const r = e.getBoundingClientRect();
    if (!r.width || !r.height) continue;
    const t = (e.textContent || '').trim().replace(/\s+/g, ' ');
    const ok = q.exact ? t === q.text : t.startsWith(q.text);
    if (!ok) continue;
    if (r.width * r.height < area) { best = e; area = r.width * r.height; }
  }
  if (best && q.up) for (let i = 0; i < q.up && best.parentElement; i++) best = best.parentElement;
  return best;
};
window.__annotate = (items, draw = true) => {
  const shapes = [];
  document.getElementById('__ann')?.remove();
  const NS = 'http://www.w3.org/2000/svg';
  const svg = document.createElementNS(NS, 'svg');
  svg.id = '__ann';
  Object.assign(svg.style, { position: 'fixed', left: 0, top: 0, width: '100vw', height: '100vh', pointerEvents: 'none', zIndex: 2147483647 });
  svg.innerHTML = `<defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
    <path d="M0,0 L10,5 L0,10 z" fill="#ff3d00"/></marker>
    <filter id="sh"><feDropShadow dx="0" dy="1" stdDeviation="1.5" flood-color="#000" flood-opacity="0.8"/></filter></defs>`;
  const missing = [];
  const W = window.innerWidth, H = window.innerHeight;
  items.forEach((it, i) => {
    const el = window.__find(it);
    if (!el) { missing.push(it.n ?? i + 1); return; }
    const r = el.getBoundingClientRect();
    const pad = it.pad ?? 3;
    const x0 = r.left - pad, y0 = r.top - pad, x1 = r.right + pad, y1 = r.bottom + pad;
    const shape = { n: it.n ?? i + 1, rect: it.box !== false ? [x0, y0, x1, y1] : null, badge: null, arrow: null };
    shapes.push(shape);
    if (it.box !== false) {
      const rect = document.createElementNS(NS, 'rect');
      Object.entries({ x: x0, y: y0, width: x1 - x0, height: y1 - y0, rx: 5, fill: 'rgba(255,61,0,0.08)', stroke: '#ff3d00', 'stroke-width': 2.5, filter: 'url(#sh)' })
        .forEach(([k, v]) => rect.setAttribute(k, v));
      svg.append(rect);
    }
    const d = it.dist ?? 56;
    const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    let bx, by, ax, ay;
    switch (it.side || 'left') {
      case 'left': bx = x0 - d; by = cy + (it.dy || 0); ax = x0; ay = Math.min(Math.max(by, y0 + 4), y1 - 4); break;
      case 'right': bx = x1 + d; by = cy + (it.dy || 0); ax = x1; ay = Math.min(Math.max(by, y0 + 4), y1 - 4); break;
      case 'top': bx = cx + (it.dx || 0); by = y0 - d; ay = y0; ax = Math.min(Math.max(bx, x0 + 4), x1 - 4); break;
      case 'gutter': bx = it.gx ?? 1122; by = cy + (it.dy || 0); ax = x0; ay = Math.min(Math.max(by, y0 + 4), y1 - 4); break;
      default: bx = cx + (it.dx || 0); by = y1 + d; ay = y1; ax = Math.min(Math.max(bx, x0 + 4), x1 - 4);
    }
    if (it.top) { by = y0 + 18; ay = by; }
    bx = Math.min(Math.max(bx, 16), W - 16); by = Math.min(Math.max(by, 16), H - 16);
    const len = Math.hypot(ax - bx, ay - by);
    shape.badge = [bx, by];
    if (len > 18) shape.arrow = [bx, by, ax, ay];
    if (len > 18) {
      const line = document.createElementNS(NS, 'line');
      const ux = (ax - bx) / len, uy = (ay - by) / len;
      Object.entries({ x1: bx + ux * 15, y1: by + uy * 15, x2: ax - ux * 2, y2: ay - uy * 2, stroke: '#ff3d00', 'stroke-width': 3, 'marker-end': 'url(#ah)', 'stroke-linecap': 'round' })
        .forEach(([k, v]) => line.setAttribute(k, v));
      svg.append(line);
    }
    const c = document.createElementNS(NS, 'circle');
    Object.entries({ cx: bx, cy: by, r: 14, fill: '#ff3d00', stroke: '#fff', 'stroke-width': 2, filter: 'url(#sh)' }).forEach(([k, v]) => c.setAttribute(k, v));
    svg.append(c);
    const t = document.createElementNS(NS, 'text');
    Object.entries({ x: bx, y: by + 5, 'text-anchor': 'middle', 'font-size': 15, 'font-weight': 700, 'font-family': 'sans-serif', fill: '#fff' }).forEach(([k, v]) => t.setAttribute(k, v));
    t.textContent = it.n ?? i + 1;
    svg.append(t);
  });
  if (draw) document.body.append(svg);
  return { missing, shapes };
};
