// Scenario layer in the 3D view: predicted path of the target and the pursuer patrol
// (scenario_tool preview, the same code the scenario_runner flies). Paths are edited in the
// map window of the Scenarios tab (path_editor.js).

import * as THREE from 'three';

const ROLE = { target: 0xe57373, pursuer: 0x4fc3f7 };

export class ScenarioLayer {
  constructor(scene3d) {
    this.s3 = scene3d;
    this.group = new THREE.Group();
    scene3d.scene.add(this.group);
  }

  clearPreview() {
    for (const c of [...this.group.children]) { this.group.remove(c); c.geometry && c.geometry.dispose(); }
  }

  setPreview(pv) {
    this.clearPreview();
    if (!pv || !pv.ok) return;
    const line = (pts, color, opacity = 0.85, dashed = false) => {
      if (pts.length < 2) return;
      const geo = new THREE.BufferGeometry().setFromPoints(pts.map((p) => new THREE.Vector3(p[0], p[1], p[2])));
      const mat = dashed ? new THREE.LineDashedMaterial({ color, dashSize: 0.6, gapSize: 0.4, transparent: true, opacity })
        : new THREE.LineBasicMaterial({ color, transparent: true, opacity });
      const l = new THREE.Line(geo, mat);
      if (dashed) l.computeLineDistances();
      l.frustumCulled = false;
      this.group.add(l);
    };
    const marker = (p, color, size = 0.25) => {
      const m = new THREE.Mesh(new THREE.SphereGeometry(size, 10, 8), new THREE.MeshBasicMaterial({ color }));
      m.position.set(p[0], p[1], p[2]);
      this.group.add(m);
    };
    for (const d of pv.drones || []) {
      for (const path of d.paths) line(path, ROLE.target, 0.95);
      if (d.spawn) marker(d.spawn, ROLE.target, 0.18);
      if (d.start) marker(d.start, ROLE.target, 0.3);
    }
    const p = pv.pursuer || {};
    if (p.start) marker(p.start, ROLE.pursuer, 0.35);
    for (const path of p.paths || []) line(path, ROLE.pursuer, 0.8, true);
  }

  // Kept for callers of the old 3D editing.
  stopEditing() {}
}
