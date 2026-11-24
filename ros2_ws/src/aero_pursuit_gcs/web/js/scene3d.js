// 3D view (three.js, Z up as in ROS): drones from the TFs, trails,
// MPC predicted trajectory and target estimate.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/OrbitControls.js';

const DRONES = {
  x500_depth_0: { color: 0x4fc3f7, label: 'Pursuer', size: 0.55 },
  X3: { color: 0xffb74d, label: 'X3', size: 0.45 },
  X4: { color: 0xe57373, label: 'X4', size: 0.45 },
  Tiltrotor: { color: 0xba68c8, label: 'Tiltrotor', size: 0.6 },
  X5: { color: 0x81c784, label: 'X5', size: 0.5 },
};
const TRAIL = 900;
// Scenario drones: color by role.
const ROLE_COLORS = { target: 0xe57373, pursuer: 0x4fc3f7 };

function makeLabel(source, color) {
  const text = source;
  const c = document.createElement('canvas');
  c.width = 256; c.height = 64;
  const ctx = c.getContext('2d');
  ctx.font = 'bold 30px sans-serif';
  ctx.fillStyle = 'rgba(0,0,0,0.55)';
  const w = ctx.measureText(text).width + 20;
  ctx.fillRect((256 - w) / 2, 8, w, 46);
  ctx.fillStyle = '#' + color.toString(16).padStart(6, '0');
  ctx.textAlign = 'center';
  ctx.fillText(text, 128, 42);
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthTest: false }));
  sprite.scale.set(2, 0.5, 1);
  sprite.position.set(0, 0, 0.8);
  return sprite;
}

function makeDrone({ color, label, size }) {
  const g = new THREE.Group();
  const mat = new THREE.MeshStandardMaterial({ color, metalness: 0.2, roughness: 0.6 });
  g.add(new THREE.Mesh(new THREE.BoxGeometry(size * 0.5, size * 0.5, size * 0.18), mat));
  const armMat = new THREE.MeshStandardMaterial({ color: 0x444444 });
  for (let i = 0; i < 4; i++) {
    const a = Math.PI / 4 + (i * Math.PI) / 2;
    const arm = new THREE.Mesh(new THREE.BoxGeometry(size * 1.1, size * 0.07, size * 0.05), armMat);
    arm.rotation.z = a;
    g.add(arm);
    const rotor = new THREE.Mesh(new THREE.CylinderGeometry(size * 0.28, size * 0.28, 0.01, 20),
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.35 }));
    rotor.rotation.x = Math.PI / 2;
    rotor.position.set(Math.cos(a) * size * 0.55, Math.sin(a) * size * 0.55, size * 0.08);
    g.add(rotor);
  }
  // Nose (body +X, FLU) to show the heading.
  const nose = new THREE.Mesh(new THREE.ConeGeometry(size * 0.08, size * 0.3, 8), new THREE.MeshBasicMaterial({ color: 0xffffff }));
  nose.rotation.z = -Math.PI / 2;
  nose.position.x = size * 0.4;
  g.add(nose);
  g.add(makeLabel(label, color));
  return g;
}

/// Line with a preallocated position buffer (and line distances for dashed
/// materials). BufferGeometry.setFromPoints/computeLineDistances allocate a new GPU
/// buffer on every call: done per frame they leak buffers and make the line flicker.
function makeDynamicLine(material, capacity) {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(capacity * 3), 3).setUsage(THREE.DynamicDrawUsage));
  geo.setAttribute('lineDistance', new THREE.BufferAttribute(new Float32Array(capacity), 1).setUsage(THREE.DynamicDrawUsage));
  geo.setDrawRange(0, 0);
  const line = new THREE.Line(geo, material);
  line.frustumCulled = false;
  line.visible = false;
  line.userData.capacity = capacity;
  return line;
}

/// Writes up to `capacity` points into a line made by makeDynamicLine.
function setLinePoints(line, pts) {
  const n = Math.min(pts.length, line.userData.capacity);
  const pos = line.geometry.attributes.position;
  const dist = line.geometry.attributes.lineDistance;
  let d = 0;
  for (let i = 0; i < n; i++) {
    const p = pts[i];
    if (i > 0) d += p.distanceTo(pts[i - 1]);
    pos.array[i * 3] = p.x; pos.array[i * 3 + 1] = p.y; pos.array[i * 3 + 2] = p.z;
    dist.array[i] = d;
  }
  pos.needsUpdate = true;
  dist.needsUpdate = true;
  line.geometry.setDrawRange(0, n);
  return n;
}

function makeTrail(color) {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(TRAIL * 3), 3));
  geo.setDrawRange(0, 0);
  const line = new THREE.Line(geo, new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.6 }));
  line.frustumCulled = false;
  return { line, n: 0, last: null };
}

export class Scene3D {
  constructor(container, state) {
    this.container = container;
    this.state = state;
    // Logarithmic depth: near 0.05 m / far 2 km would leave little depth precision far
    // away and lines close to each other (trails on the ground, grid) would z-fight.
    this.renderer = new THREE.WebGLRenderer({ antialias: true, logarithmicDepthBuffer: true });
    this.renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    container.append(this.renderer.domElement);
    // WebGL context loss (busy GPU, driver restarted, tab in the background):
    // warns and lets three.js recreate the resources when the context comes back.
    this.notice = document.createElement('div');
    this.notice.className = 'scene-notice';
    this.notice.textContent = '3D view: WebGL context lost; trying to recover...';
    this.notice.style.display = 'none';
    container.append(this.notice);
    this.renderer.domElement.addEventListener('webglcontextlost', (e) => { e.preventDefault(); this.notice.style.display = 'block'; });
    this.renderer.domElement.addEventListener('webglcontextrestored', () => { this.notice.style.display = 'none'; });
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x111820);
    this.camera = new THREE.PerspectiveCamera(55, 1, 0.05, 2000);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(-12, -12, 10);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.target.set(0, 0, 2);

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x334455, 1.2));
    const sun = new THREE.DirectionalLight(0xffffff, 1.0);
    sun.position.set(10, -5, 20);
    this.scene.add(sun);
    // Grids are drawn first and write no depth: the world axes (on their centre
    // lines) and trails near the ground always win instead of z-fighting with them.
    // A dense grid out to the horizon aliases (moiré band that shimmers when the
    // camera moves): 1 m lines only nearby, 10 m lines further out.
    const fine = new THREE.GridHelper(60, 60, 0x4a6a85, 0x2c3d50);
    const coarse = new THREE.GridHelper(400, 40, 0x4a6a85, 0x24323f);
    [coarse, fine].forEach((grid, i) => {
      grid.rotation.x = Math.PI / 2;
      grid.material.depthWrite = false;
      grid.renderOrder = -2 + i;
      this.scene.add(grid);
    });
    this.scene.add(new THREE.AxesHelper(2));
    this.frameOrder = 10;   // renderOrder of the next TF axes (see object())

    this.objects = {};        // frame -> Object3D
    this.trails = {};
    this.follow = 'free';
    this.showTrails = true;
    this.showFrames = false;

    this.pathLine = makeDynamicLine(new THREE.LineBasicMaterial({ color: 0xfff176 }), 256);
    this.scene.add(this.pathLine);
    this.estimate = new THREE.Mesh(new THREE.SphereGeometry(0.25, 16, 12),
      new THREE.MeshBasicMaterial({ color: 0xff4081, transparent: true, opacity: 0.8 }));
    this.estimate.visible = false;
    this.scene.add(this.estimate);
    this.los = makeDynamicLine(new THREE.LineDashedMaterial({ color: 0xff4081, dashSize: 0.4, gapSize: 0.25 }), 2);
    this.scene.add(this.los);

    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    this.visible = true;
    const loop = () => {
      requestAnimationFrame(loop);
      if (this.visible && container.offsetParent !== null) this.render();
    };
    requestAnimationFrame(loop);
  }

  resize() {
    const r = this.container.getBoundingClientRect();
    if (r.width < 2 || r.height < 2) return;
    this.renderer.setSize(r.width, r.height, false);
    this.camera.aspect = r.width / r.height;
    this.camera.updateProjectionMatrix();
  }

  /// Appearance of a frame: known drone, running scenario drone or null.
  droneStyle(frame) {
    if (DRONES[frame]) return DRONES[frame];
    const drones = (this.state.scenario && this.state.scenario.drones) || [];
    const d = drones.find((x) => x.name === frame);
    if (!d) return null;
    return { color: ROLE_COLORS[d.role] || 0xb0bec5, label: frame, size: 0.5 };
  }

  object(frame) {
    let o = this.objects[frame];
    if (o && (o.userData.drone || !this.droneStyle(frame))) return o;
    if (o) { this.scene.remove(o); delete this.objects[frame]; }   // became a drone (scenario loaded)
    const style = this.droneStyle(frame);
    if (style) {
      o = makeDrone(style);
      o.userData.drone = true;
      const t = makeTrail(style.color);
      this.trails[frame] = t;
      this.scene.add(t.line);
    } else {
      o = new THREE.AxesHelper(0.4);
      // Frames sharing an origin (camera_link/camera_optical, base/base_enu) have
      // colinear axes: without depth test and with a fixed draw order per frame the
      // overlap is stable instead of z-fighting (flickering) as the camera moves.
      o.material.depthTest = false;
      o.renderOrder = this.frameOrder++;
      o.visible = this.showFrames;
    }
    this.objects[frame] = o;
    this.scene.add(o);
    return o;
  }

  /// Transforms a point from frame `frame` to the world using the received TFs.
  toWorld(frame, p) {
    const v = new THREE.Vector3(p[0], p[1], p[2]);
    if (!frame || frame === this.state.worldFrame) return v;
    const tf = this.state.tf[frame];
    if (!tf) return null;
    v.applyQuaternion(new THREE.Quaternion(tf[3], tf[4], tf[5], tf[6]));
    return v.add(new THREE.Vector3(tf[0], tf[1], tf[2]));
  }

  /// Removes from the scene a frame (drone + label + trail) that no longer exists.
  dropFrame(frame) {
    const o = this.objects[frame];
    if (o) {
      this.scene.remove(o);
      o.traverse((c) => { if (c.geometry) c.geometry.dispose(); if (c.material && c.material.map) c.material.map.dispose(); });
      delete this.objects[frame];
    }
    const tr = this.trails[frame];
    if (tr) { this.scene.remove(tr.line); tr.line.geometry.dispose(); delete this.trails[frame]; }
    delete this.lastSeen[frame];
    if (this.follow === frame) this.follow = 'free';
  }

  update() {
    const tf = this.state.tf || {};
    const now = performance.now();
    this.lastSeen = this.lastSeen || {};
    for (const frame of Object.keys(tf)) this.lastSeen[frame] = now;
    // Removed drones (unloaded scenario, deleted model): vanish after 1.5 s without a pose.
    for (const frame of Object.keys(this.objects)) {
      if (!(frame in tf) && now - (this.lastSeen[frame] || 0) > 1500) this.dropFrame(frame);
    }
    for (const [frame, t] of Object.entries(tf)) {
      const o = this.object(frame);
      o.position.set(t[0], t[1], t[2]);
      o.quaternion.set(t[3], t[4], t[5], t[6]);
      if (!o.userData.drone) o.visible = this.showFrames;
      const tr = this.trails[frame];
      if (tr && this.showTrails) {
        const last = tr.last;
        if (!last || Math.hypot(t[0] - last[0], t[1] - last[1], t[2] - last[2]) > 0.05) {
          const arr = tr.line.geometry.attributes.position.array;
          if (tr.n >= TRAIL) { arr.copyWithin(0, 3); tr.n = TRAIL - 1; }
          arr.set([t[0], t[1], t[2]], tr.n * 3);
          tr.n += 1;
          tr.last = [t[0], t[1], t[2]];
          tr.line.geometry.setDrawRange(0, tr.n);
          tr.line.geometry.attributes.position.needsUpdate = true;
        }
      }
      if (tr) tr.line.visible = this.showTrails;
    }
    const path = this.state.path;
    if (path && path.points && path.points.length > 1) {
      const pts = path.points.map((p) => this.toWorld(path.frame, p)).filter(Boolean);
      const n = setLinePoints(this.pathLine, pts);
      this.pathLine.visible = n > 1 && performance.now() - this.state.pathT < 2000;
    } else {
      this.pathLine.visible = false;
    }
    const est = this.state.estimate;
    if (est && performance.now() - this.state.estimateT < 1500) {
      const p = this.toWorld(est.frame, est.p);
      this.estimate.visible = !!p;
      if (p) {
        this.estimate.position.copy(p);
        const chaser = tf.x500_depth_0;
        if (chaser) {
          setLinePoints(this.los, [new THREE.Vector3(chaser[0], chaser[1], chaser[2]), p]);
          this.los.visible = true;
        }
      }
    } else {
      this.estimate.visible = false;
      this.los.visible = false;
    }
  }

  clearTrails() {
    for (const t of Object.values(this.trails)) { t.n = 0; t.last = null; t.line.geometry.setDrawRange(0, 0); }
  }

  render() {
    this.update();
    const target = this.follow !== 'free' && this.objects[this.follow];
    if (target) {
      const delta = target.position.clone().sub(this.controls.target);
      this.controls.target.add(delta);
      this.camera.position.add(delta);
    }
    this.controls.update();
    for (const fn of this.beforeRender || []) fn();
    this.renderer.render(this.scene, this.camera);
  }

  followOptions() {
    return [['free', 'Free camera'], ...Object.entries(DRONES).map(([k, v]) => [k, `Seguir ${v.label}`])];
  }
}
