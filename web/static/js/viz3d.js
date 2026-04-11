/**
 * Scanner Debug Mode — 3D Axis Travel Visualization
 *
 * Renders an interactive Three.js scene showing:
 *   - Specimen at origin
 *   - Per-axis travel geometry (rings, rails, arcs)
 *   - Translucent reachable envelope
 *   - Camera marker at computed position
 *   - Color-coded legend
 *
 * Coordinate convention: Z-up (matches scanner math)
 */

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

// ── Role Classification ──────────────────────────────────────────────────────

const ROTATION_ROLES = new Set([
  "specimen_rotation",
  "turntable",
  "camera_orbit",
]);
const FOCUS_ROLES = new Set(["focus", "focus_rail"]);
const TILT_ROLES = new Set(["camera_tilt"]);
const POSITION_ROLES = new Set(["positioning", "bed_positioning"]);

function classifyRole(role) {
  if (ROTATION_ROLES.has(role)) return "rotation";
  if (FOCUS_ROLES.has(role)) return "focus";
  if (TILT_ROLES.has(role)) return "tilt";
  if (POSITION_ROLES.has(role)) return "positioning";
  return "unknown";
}

// ── Role → Color ─────────────────────────────────────────────────────────────

const ROLE_HEX = {
  rotation: 0x00ff88,
  focus: 0xff6644,
  tilt: 0x4488ff,
  positioning: 0xdddd44,
  unknown: 0xaaaacc,
};

const ROLE_CSS = {
  rotation: "#00ff88",
  focus: "#ff6644",
  tilt: "#4488ff",
  positioning: "#dddd44",
  unknown: "#aaaacc",
};

// ── Main Visualization Class ─────────────────────────────────────────────────

class ScannerViz {
  constructor(containerId) {
    this.container = document.getElementById(containerId);
    if (!this.container) return;

    this.axes = [];
    this.positions = {};
    this.axisGroups = {};
    this.envelope = null;
    this.cameraMarker = null;
    this.specimenMarker = null;
    this.effectiveRadius = 100;

    this._init();
    this._animate();

    // Hide fallback text once canvas is mounted
    const fallback = document.getElementById("viz-fallback");
    if (fallback) fallback.style.display = "none";
  }

  // ── Scene Setup ──────────────────────────────────────────────────────────

  _init() {
    const w = this.container.clientWidth || 600;
    const h = this.container.clientHeight || 400;

    // Scene
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0f0f1a);

    // Camera (Z-up)
    this.cam = new THREE.PerspectiveCamera(50, w / h, 0.1, 10000);
    this.cam.up.set(0, 0, 1);
    this.cam.position.set(200, -200, 150);
    this.cam.lookAt(0, 0, 0);

    // Renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(w, h);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.container.appendChild(this.renderer.domElement);

    // Orbit controls
    this.controls = new OrbitControls(this.cam, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.12;

    // Lighting
    this.scene.add(new THREE.AmbientLight(0x404060, 3));
    const dir = new THREE.DirectionalLight(0xffffff, 1.5);
    dir.position.set(200, -100, 300);
    this.scene.add(dir);

    // Grid (XY plane = floor)
    this._buildGrid(400, 20);

    // Origin axes arrows
    this._buildAxesArrows(50);

    // Specimen marker
    const specGeo = new THREE.SphereGeometry(4, 16, 16);
    const specMat = new THREE.MeshPhongMaterial({
      color: 0x00aaff,
      emissive: 0x002244,
    });
    this.specimenMarker = new THREE.Mesh(specGeo, specMat);
    this.scene.add(this.specimenMarker);

    // Camera marker (cone)
    const coneGeo = new THREE.ConeGeometry(4, 10, 8);
    const coneMat = new THREE.MeshPhongMaterial({
      color: 0xffaa00,
      emissive: 0x442200,
    });
    this.cameraMarker = new THREE.Mesh(coneGeo, coneMat);
    this.cameraMarker.visible = false;
    this.scene.add(this.cameraMarker);

    // Resize
    window.addEventListener("resize", () => this._onResize());
  }

  _buildGrid(size, divisions) {
    const step = size / divisions;
    const half = size / 2;
    const pts = [];
    for (let i = -half; i <= half; i += step) {
      pts.push(new THREE.Vector3(i, -half, 0), new THREE.Vector3(i, half, 0));
      pts.push(new THREE.Vector3(-half, i, 0), new THREE.Vector3(half, i, 0));
    }
    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    const mat = new THREE.LineBasicMaterial({ color: 0x1e1e38 });
    this.scene.add(new THREE.LineSegments(geo, mat));
  }

  _buildAxesArrows(len) {
    const axes = [
      { dir: [1, 0, 0], color: 0xff4444, label: "X" },
      { dir: [0, 1, 0], color: 0x44ff44, label: "Y" },
      { dir: [0, 0, 1], color: 0x4488ff, label: "Z" },
    ];
    for (const a of axes) {
      const d = new THREE.Vector3(...a.dir);
      this.scene.add(
        new THREE.ArrowHelper(d, new THREE.Vector3(), len, a.color, 6, 3),
      );
    }
  }

  // ── Profile Loading ──────────────────────────────────────────────────────

  setProfile(axes) {
    this.axes = axes;
    this.positions = {};
    for (const a of axes) this.positions[a.name] = 0;
    this._rebuildGeometry();
    this._updateLegend();
    this._fitCamera();
  }

  _rebuildGeometry() {
    // Remove old axis groups
    for (const g of Object.values(this.axisGroups)) this.scene.remove(g);
    this.axisGroups = {};
    if (this.envelope) {
      this.scene.remove(this.envelope);
      this.envelope = null;
    }

    // Determine effective radius
    const focusAxis = this.axes.find((a) => FOCUS_ROLES.has(a.role));
    this.effectiveRadius = focusAxis
      ? (focusAxis.min_limit + focusAxis.max_limit) / 2 || 80
      : 100;

    // Build per-axis geometry
    for (const axis of this.axes) {
      const cls = classifyRole(axis.role);
      const color = ROLE_HEX[cls];
      const group = new THREE.Group();
      group.userData = { axisName: axis.name, cls };

      switch (cls) {
        case "rotation":
          this._buildRotationRing(group, axis, color);
          break;
        case "focus":
          this._buildFocusRail(group, axis, color);
          break;
        case "tilt":
          this._buildTiltArc(group, axis, color);
          break;
        case "positioning":
          this._buildPositioningAxis(group, axis, color);
          break;
        default:
          if (axis.axis_type === "rotary")
            this._buildRotationRing(group, axis, color);
          else this._buildPositioningAxis(group, axis, color);
      }

      this.scene.add(group);
      this.axisGroups[axis.name] = group;
    }

    // Reachable envelope
    this._buildEnvelope();

    // Camera marker visibility
    this.cameraMarker.visible = this.axes.length > 0;
    this._updateCameraMarker();
  }

  // ── Axis Geometry Builders ────────────────────────────────────────────────

  _buildRotationRing(group, axis, color) {
    const r = this.effectiveRadius;
    const s = (axis.min_limit * Math.PI) / 180;
    const e = (axis.max_limit * Math.PI) / 180;
    const segs = 128;
    const pts = [];
    for (let i = 0; i <= segs; i++) {
      const a = s + (i / segs) * (e - s);
      pts.push(new THREE.Vector3(r * Math.cos(a), r * Math.sin(a), 0));
    }

    // Ring line
    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    const mat = new THREE.LineBasicMaterial({ color });
    group.add(new THREE.Line(geo, mat));

    // Dashed fill for full-circle rings
    if (axis.max_limit - axis.min_limit >= 360) {
      const ringGeo = new THREE.RingGeometry(r - 1, r + 1, 64);
      const ringMat = new THREE.MeshBasicMaterial({
        color,
        transparent: true,
        opacity: 0.08,
        side: THREE.DoubleSide,
      });
      group.add(new THREE.Mesh(ringGeo, ringMat));
    }

    // Start / end tick marks
    this._addTick(group, r, s, color);
    this._addTick(group, r, e, color);

    // Position marker
    this._addMarker(group, color);
  }

  _buildFocusRail(group, axis, color) {
    const mn = axis.min_limit;
    const mx = axis.max_limit;

    // Rail line
    const pts = [new THREE.Vector3(mn, 0, 0), new THREE.Vector3(mx, 0, 0)];
    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    const mat = new THREE.LineBasicMaterial({ color });
    group.add(new THREE.Line(geo, mat));

    // Tube for visual thickness
    const len = mx - mn;
    const tubeGeo = new THREE.CylinderGeometry(1.5, 1.5, len, 8);
    tubeGeo.rotateZ(-Math.PI / 2);
    tubeGeo.translate((mn + mx) / 2, 0, 0);
    const tubeMat = new THREE.MeshPhongMaterial({
      color,
      transparent: true,
      opacity: 0.3,
    });
    group.add(new THREE.Mesh(tubeGeo, tubeMat));

    // End caps
    for (const x of [mn, mx]) {
      const capGeo = new THREE.SphereGeometry(2, 8, 8);
      const capMat = new THREE.MeshPhongMaterial({ color });
      const cap = new THREE.Mesh(capGeo, capMat);
      cap.position.set(x, 0, 0);
      group.add(cap);
    }

    this._addMarker(group, color);
  }

  _buildTiltArc(group, axis, color) {
    const r = this.effectiveRadius;
    const s = (axis.min_limit * Math.PI) / 180;
    const e = (axis.max_limit * Math.PI) / 180;
    const segs = 64;
    const pts = [];

    // Arc in XZ plane (vertical)
    for (let i = 0; i <= segs; i++) {
      const elev = s + (i / segs) * (e - s);
      pts.push(new THREE.Vector3(r * Math.cos(elev), 0, r * Math.sin(elev)));
    }

    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    const mat = new THREE.LineBasicMaterial({ color });
    group.add(new THREE.Line(geo, mat));

    // End ticks (short lines perpendicular to arc)
    for (const ang of [s, e]) {
      const p = new THREE.Vector3(r * Math.cos(ang), 0, r * Math.sin(ang));
      const tangent = new THREE.Vector3(
        -Math.sin(ang),
        0,
        Math.cos(ang),
      ).multiplyScalar(6);
      const tickPts = [p.clone().sub(tangent), p.clone().add(tangent)];
      const tGeo = new THREE.BufferGeometry().setFromPoints(tickPts);
      group.add(new THREE.Line(tGeo, new THREE.LineBasicMaterial({ color })));
    }

    this._addMarker(group, color);
  }

  _buildPositioningAxis(group, axis, color) {
    // Determine spatial direction from axis name
    const n = axis.name.toUpperCase();
    let dir;
    if (n === "X" || n.startsWith("X")) dir = new THREE.Vector3(1, 0, 0);
    else if (n === "Y" || n.startsWith("Y")) dir = new THREE.Vector3(0, 1, 0);
    else if (n === "Z" || n.startsWith("Z")) dir = new THREE.Vector3(0, 0, 1);
    else dir = new THREE.Vector3(1, 0, 0);

    group.userData.direction = dir;

    const startPt = dir.clone().multiplyScalar(axis.min_limit);
    const endPt = dir.clone().multiplyScalar(axis.max_limit);

    // Line
    const geo = new THREE.BufferGeometry().setFromPoints([startPt, endPt]);
    const mat = new THREE.LineBasicMaterial({ color });
    group.add(new THREE.Line(geo, mat));

    // Tube
    const len = axis.max_limit - axis.min_limit;
    const mid = dir
      .clone()
      .multiplyScalar((axis.min_limit + axis.max_limit) / 2);
    const tubeGeo = new THREE.CylinderGeometry(1.2, 1.2, len, 8);
    // Align cylinder along direction
    const up = new THREE.Vector3(0, 1, 0);
    const quat = new THREE.Quaternion().setFromUnitVectors(up, dir);
    tubeGeo.applyQuaternion(quat);
    tubeGeo.translate(mid.x, mid.y, mid.z);
    const tubeMat = new THREE.MeshPhongMaterial({
      color,
      transparent: true,
      opacity: 0.25,
    });
    group.add(new THREE.Mesh(tubeGeo, tubeMat));

    // End caps
    for (const pt of [startPt, endPt]) {
      const capGeo = new THREE.SphereGeometry(2, 8, 8);
      const capMat = new THREE.MeshPhongMaterial({ color });
      const cap = new THREE.Mesh(capGeo, capMat);
      cap.position.copy(pt);
      group.add(cap);
    }

    this._addMarker(group, color);
  }

  // ── Helpers ──────────────────────────────────────────────────────────────

  _addMarker(group, color) {
    const geo = new THREE.SphereGeometry(3.5, 12, 12);
    const mat = new THREE.MeshPhongMaterial({
      color,
      emissive: color,
      emissiveIntensity: 0.4,
    });
    const m = new THREE.Mesh(geo, mat);
    m.name = "marker";
    group.add(m);
  }

  _addTick(group, radius, angle, color) {
    const inner = radius - 6;
    const outer = radius + 6;
    const pts = [
      new THREE.Vector3(inner * Math.cos(angle), inner * Math.sin(angle), 0),
      new THREE.Vector3(outer * Math.cos(angle), outer * Math.sin(angle), 0),
    ];
    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    group.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color })));
  }

  // ── Reachable Envelope ───────────────────────────────────────────────────

  _buildEnvelope() {
    const rotAxis = this.axes.find((a) => ROTATION_ROLES.has(a.role));
    const focusAxis = this.axes.find((a) => FOCUS_ROLES.has(a.role));
    const tiltAxis = this.axes.find((a) => TILT_ROLES.has(a.role));

    if (!rotAxis && !focusAxis && !tiltAxis) return;

    const envGroup = new THREE.Group();

    const rMin = focusAxis ? focusAxis.min_limit : this.effectiveRadius;
    const rMax = focusAxis ? focusAxis.max_limit : this.effectiveRadius;
    const azMin = rotAxis ? (rotAxis.min_limit * Math.PI) / 180 : 0;
    const azMax = rotAxis ? (rotAxis.max_limit * Math.PI) / 180 : 2 * Math.PI;
    const elMin = tiltAxis ? (tiltAxis.min_limit * Math.PI) / 180 : 0;
    const elMax = tiltAxis ? (tiltAxis.max_limit * Math.PI) / 180 : 0;

    // Outer surface
    this._addEnvelopeSurface(envGroup, rMax, azMin, azMax, elMin, elMax, 0.07);
    // Inner surface (when different)
    if (Math.abs(rMax - rMin) > 1) {
      this._addEnvelopeSurface(
        envGroup,
        rMin,
        azMin,
        azMax,
        elMin,
        elMax,
        0.04,
      );
    }

    this.envelope = envGroup;
    this.scene.add(envGroup);
  }

  _addEnvelopeSurface(parent, radius, azMin, azMax, elMin, elMax, opacity) {
    const elevRange = Math.abs(elMax - elMin);

    // Flat ring (no tilt)
    if (elevRange < 0.02) {
      const segs = 128;
      const pts = [];
      for (let i = 0; i <= segs; i++) {
        const a = azMin + (i / segs) * (azMax - azMin);
        pts.push(
          new THREE.Vector3(radius * Math.cos(a), radius * Math.sin(a), 0),
        );
      }
      const geo = new THREE.BufferGeometry().setFromPoints(pts);
      const mat = new THREE.LineBasicMaterial({
        color: 0x4488ff,
        transparent: true,
        opacity: opacity * 3,
      });
      parent.add(new THREE.Line(geo, mat));
      return;
    }

    // Spherical patch
    const azSegs = 48;
    const elSegs = 24;
    const vertices = [];
    const indices = [];

    for (let ei = 0; ei <= elSegs; ei++) {
      const el = elMin + (ei / elSegs) * (elMax - elMin);
      for (let ai = 0; ai <= azSegs; ai++) {
        const az = azMin + (ai / azSegs) * (azMax - azMin);
        vertices.push(
          radius * Math.cos(el) * Math.cos(az),
          radius * Math.cos(el) * Math.sin(az),
          radius * Math.sin(el),
        );
        if (ei < elSegs && ai < azSegs) {
          const a = ei * (azSegs + 1) + ai;
          const b = a + azSegs + 1;
          indices.push(a, b, a + 1);
          indices.push(a + 1, b, b + 1);
        }
      }
    }

    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(vertices, 3));
    geo.setIndex(indices);
    geo.computeVertexNormals();

    const mat = new THREE.MeshPhongMaterial({
      color: 0x4488ff,
      transparent: true,
      opacity,
      side: THREE.DoubleSide,
      depthWrite: false,
    });
    parent.add(new THREE.Mesh(geo, mat));

    // Wireframe outline
    const wireGeo = new THREE.WireframeGeometry(geo);
    const wireMat = new THREE.LineBasicMaterial({
      color: 0x4488ff,
      transparent: true,
      opacity: opacity * 0.5,
    });
    parent.add(new THREE.LineSegments(wireGeo, wireMat));
  }

  // ── Position Updates ─────────────────────────────────────────────────────

  updatePositions(positions) {
    for (const [name, pos] of Object.entries(positions)) {
      this.positions[name] = pos;
    }
    // Update all markers
    for (const name of Object.keys(positions)) {
      this._updateMarker(name);
    }
    this._updateCameraMarker();
  }

  updateAxisPosition(name, pos) {
    this.positions[name] = pos;
    this._updateMarker(name);
    this._updateCameraMarker();
  }

  _updateMarker(name) {
    const axis = this.axes.find((a) => a.name === name);
    const group = this.axisGroups[name];
    if (!axis || !group) return;

    const marker = group.getObjectByName("marker");
    if (!marker) return;

    const pos = this.positions[name] ?? 0;
    const cls = classifyRole(axis.role);

    switch (cls) {
      case "rotation": {
        const rad = (pos * Math.PI) / 180;
        const r = this.effectiveRadius;
        marker.position.set(r * Math.cos(rad), r * Math.sin(rad), 0);
        break;
      }
      case "focus": {
        marker.position.set(pos, 0, 0);
        break;
      }
      case "tilt": {
        const rad = (pos * Math.PI) / 180;
        const r = this.effectiveRadius;
        marker.position.set(r * Math.cos(rad), 0, r * Math.sin(rad));
        break;
      }
      case "positioning": {
        const dir = group.userData.direction;
        if (dir) {
          marker.position.copy(dir.clone().multiplyScalar(pos));
        }
        break;
      }
    }
  }

  _updateCameraMarker() {
    if (!this.cameraMarker || this.axes.length === 0) return;

    let azimuth = 0;
    let elevation = 0;
    let distance = this.effectiveRadius;
    const offset = new THREE.Vector3();

    for (const axis of this.axes) {
      const pos = this.positions[axis.name] ?? 0;
      const cls = classifyRole(axis.role);
      switch (cls) {
        case "rotation":
          azimuth = pos;
          break;
        case "focus":
          distance = pos || this.effectiveRadius;
          break;
        case "tilt":
          elevation = pos;
          break;
        case "positioning": {
          const n = axis.name.toUpperCase();
          if (n === "X" || n.startsWith("X")) offset.x = pos;
          else if (n === "Y" || n.startsWith("Y")) offset.y = pos;
          else if (n === "Z" || n.startsWith("Z")) offset.z = pos;
          break;
        }
      }
    }

    const azRad = (azimuth * Math.PI) / 180;
    const elRad = (elevation * Math.PI) / 180;

    const x = distance * Math.cos(elRad) * Math.cos(azRad) + offset.x;
    const y = distance * Math.cos(elRad) * Math.sin(azRad) + offset.y;
    const z = distance * Math.sin(elRad) + offset.z;

    this.cameraMarker.position.set(x, y, z);
    this.cameraMarker.visible = true;

    // Point cone toward specimen
    this.cameraMarker.lookAt(this.specimenMarker.position);
    this.cameraMarker.rotateX(Math.PI / 2);
  }

  // ── Legend ────────────────────────────────────────────────────────────────

  _updateLegend() {
    const container = document.getElementById("viz-legend");
    if (!container) return;
    container.innerHTML = "";

    const seen = new Set();
    for (const axis of this.axes) {
      if (seen.has(axis.name)) continue;
      seen.add(axis.name);

      const cls = classifyRole(axis.role);
      const cssColor = ROLE_CSS[cls];

      const item = document.createElement("div");
      item.className = "legend-item";
      item.innerHTML = `
                <div class="legend-swatch" style="background:${cssColor}"></div>
                <span>${axis.name}: ${axis.label}</span>
            `;
      container.appendChild(item);
    }

    // Fixed entries
    container.insertAdjacentHTML(
      "beforeend",
      `
            <div class="legend-item">
                <div class="legend-swatch" style="background:#00aaff"></div>
                <span>Specimen</span>
            </div>
            <div class="legend-item">
                <div class="legend-swatch" style="background:#ffaa00"></div>
                <span>Camera</span>
            </div>
        `,
    );
  }

  // ── Camera Fit ───────────────────────────────────────────────────────────

  _fitCamera() {
    let maxDim = 50;
    for (const axis of this.axes) {
      maxDim = Math.max(
        maxDim,
        Math.abs(axis.max_limit),
        Math.abs(axis.min_limit),
      );
    }

    const d = maxDim * 2;
    this.cam.position.set(d * 0.7, -d * 0.7, d * 0.5);
    this.cam.lookAt(0, 0, 0);
    this.controls.target.set(0, 0, 0);
    this.controls.update();
  }

  // ── Resize ───────────────────────────────────────────────────────────────

  _onResize() {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    if (w === 0 || h === 0) return;
    this.cam.aspect = w / h;
    this.cam.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }

  // ── Render Loop ──────────────────────────────────────────────────────────

  _animate() {
    requestAnimationFrame(() => this._animate());
    this.controls.update();
    this.renderer.render(this.scene, this.cam);
  }
}

// ── Bootstrap ────────────────────────────────────────────────────────────────

window.scannerViz = new ScannerViz("viz-container");
