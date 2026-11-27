// three.js view of one SimWorld: the scene's visual.glb (moving nodes follow their MuJoCo bodies), the G1 built from the
// MuJoCo mesh data, mouse drag of free objects / doors / drawers, and a follow or free orbit camera.
import * as THREE from '../../vendor/three/three.module.min.js';
import { GLTFLoader } from '../../vendor/three/addons/GLTFLoader.js';
import { OrbitControls } from '../../vendor/three/addons/OrbitControls.js';

const zup = (x, y, z) => new THREE.Vector3(x, z, -y);            // MuJoCo world (z up) -> three world (y up)
const fromThree = (v) => [v.x, -v.z, v.y];

export class WorldView {
  constructor(canvas, world, opts = {}) {
    this.canvas = canvas; this.w = world; this.opts = opts;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, opts.maxDpr || 1.5));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(opts.background ?? 0x15171c);
    this.root = new THREE.Group(); this.root.rotation.x = -Math.PI / 2; this.scene.add(this.root);
    this.root.updateMatrixWorld(true);
    this.camera = new THREE.PerspectiveCamera(opts.fov || 60, 1, 0.03, 200);
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x5a5550, 1.6));
    const sun = new THREE.DirectionalLight(0xffffff, 1.6); sun.position.set(3, 8, 4); this.scene.add(sun);
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true; this.controls.dampingFactor = 0.12; this.controls.maxPolarAngle = Math.PI * 0.495;
    this.controls.minDistance = 0.4; this.controls.maxDistance = 30;
    this.controls.addEventListener('start', () => { this._userT = 1e15; });
    this.controls.addEventListener('end', () => { this._userT = performance.now(); });
    this.mode = world.robot ? (opts.camMode || 'follow') : 'free';
    this.bodyNodes = [];   // {b, node, restInv (Matrix4), local (Matrix4)}
    this.pickables = [];
    this.hover = null; this.labels = new Map((world.meta.bodies || []).map((b) => [b.name, b]));
    this._tmp = new THREE.Matrix4(); this._tmp2 = new THREE.Matrix4();
  }

  async init() {
    const w = this.w; w._checkViews();
    // scene visuals
    const gltf = await new Promise((res, rej) => new GLTFLoader().parse(w.scene.glb.buffer.slice(w.scene.glb.byteOffset, w.scene.glb.byteOffset + w.scene.glb.byteLength), '', res, rej));
    this.gltf = gltf.scene;
    this.root.add(this.gltf);
    this.gltf.updateMatrixWorld(true);
    const maxAniso = this.renderer.capabilities.getMaxAnisotropy();
    this.gltf.traverse((o) => {
      if (!o.isMesh) return;
      for (const m of [].concat(o.material)) {
        if (m.map) m.map.anisotropy = Math.min(8, maxAniso);
        if (!m.isMeshBasicMaterial && !o.geometry.attributes.normal) o.geometry.computeVertexNormals();
        if (m.isMeshStandardMaterial && !m.metalnessMap) m.metalness = Math.min(m.metalness, 0.1);   // glTF default metal = 1 looks black without an env map
      }
    });
    const missing = [];
    for (const b of w.moving) {
      const name = w.bodyName(b);
      const node = name && this.gltf.getObjectByName(name);
      if (!node) { missing.push(name); continue; }
      this.gltf.attach(node);                       // flatten: node directly under the glTF root, world pose kept
      node.updateMatrix();
      const local = node.matrix.clone();
      const restInv = this._bodyMatrix(b, w.rest.xpos, w.rest.xmat, new THREE.Matrix4()).invert();
      node.matrixAutoUpdate = false;
      node.traverse((o) => {
        if (!o.isMesh) return;
        o.material = [].concat(o.material).map((m) => m.clone());
        if (o.material.length === 1) o.material = o.material[0];
        o.userData.body = b; this.pickables.push(o);
      });
      this.bodyNodes.push({ b, node, restInv, local, name });
    }
    this.ceiling = ((w.meta.nodes || {}).ceiling || []).map((n) => this.gltf.getObjectByName(n)).filter(Boolean);
    this.ceilZ = (w.meta.bounds ? w.meta.bounds[1][2] : 3) - 0.25;
    if (missing.length) console.warn(`[scanart] ${missing.length} moving bodies without a glb node:`, missing.slice(0, 20));
    // robot
    if (w.robot) this._buildRobot();
    // drag helpers
    const lm = new THREE.LineBasicMaterial({ color: 0xffd166, depthTest: false, transparent: true });
    this.dragLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), lm);
    this.dragLine.renderOrder = 10; this.dragLine.visible = false; this.scene.add(this.dragLine);
    const sg = new THREE.SphereGeometry(0.018, 12, 8);
    this.dragDot = new THREE.Mesh(sg, new THREE.MeshBasicMaterial({ color: 0xffd166, depthTest: false })); this.dragDot.renderOrder = 11;
    this.dragDot.visible = false; this.scene.add(this.dragDot);
    this._initCamera();
    this._bindPointer();
    this.resize();
    this.sync();
  }

  _bodyMatrix(b, xpos, xmat, out) {
    const r = xmat, o = 9 * b, p = 3 * b;
    return out.set(r[o], r[o + 1], r[o + 2], xpos[p], r[o + 3], r[o + 4], r[o + 5], xpos[p + 1], r[o + 6], r[o + 7], r[o + 8], xpos[p + 2], 0, 0, 0, 1);
  }

  _buildRobot() {
    const w = this.w, m = w.model;
    const vert = m.mesh_vert, face = m.mesh_face, vadr = m.mesh_vertadr, vnum = m.mesh_vertnum, fadr = m.mesh_faceadr, fnum = m.mesh_facenum;
    const ggroup = m.geom_group, gtype = m.geom_type, gdata = m.geom_dataid, gbody = m.geom_bodyid, rgba = m.geom_rgba;
    this.robotGeoms = [];
    const cache = new Map();
    for (let g = 0; g < m.ngeom; g++) {
      if (!w.robotBodies.has(gbody[g]) || ggroup[g] !== 1 || gtype[g] !== 7) continue;
      const id = gdata[g];
      let geo = cache.get(id);
      if (!geo) {
        geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.BufferAttribute(Float32Array.from(vert.subarray(3 * vadr[id], 3 * (vadr[id] + vnum[id]))), 3));
        geo.setIndex(new THREE.BufferAttribute(Uint32Array.from(face.subarray(3 * fadr[id], 3 * (fadr[id] + fnum[id]))), 1));
        geo.computeVertexNormals();
        cache.set(id, geo);
      }
      const c = rgba.subarray(4 * g, 4 * g + 4);
      const dark = c[0] < 0.4;
      const mat = new THREE.MeshStandardMaterial({ color: new THREE.Color(c[0], c[1], c[2]).convertSRGBToLinear(), metalness: dark ? 0.2 : 0.35, roughness: dark ? 0.6 : 0.38 });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.matrixAutoUpdate = false; mesh.userData.body = gbody[g]; mesh.userData.robot = true;
      this.root.add(mesh); this.robotGeoms.push({ g, mesh }); this.pickables.push(mesh);
    }
    // soft contact shadow under the robot
    const tex = new THREE.CanvasTexture((() => {
      const cv = document.createElement('canvas'); cv.width = cv.height = 64; const cx = cv.getContext('2d');
      const gr = cx.createRadialGradient(32, 32, 0, 32, 32, 32); gr.addColorStop(0, 'rgba(0,0,0,0.55)'); gr.addColorStop(1, 'rgba(0,0,0,0)');
      cx.fillStyle = gr; cx.fillRect(0, 0, 64, 64); return cv;
    })());
    this.blob = new THREE.Mesh(new THREE.PlaneGeometry(0.9, 0.9), new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false }));
    this.blob.renderOrder = 2; this.root.add(this.blob);
  }

  _initCamera() {
    const w = this.w, meta = w.meta; w._checkViews();
    if (w.robot) {
      const P = w.pelvisPose();
      const back = this.opts.followDist || 2.6;
      this.camera.position.copy(zup(P.x - back * Math.cos(P.yaw), P.y - back * Math.sin(P.yaw), P.z + 0.9));
      this.controls.target.copy(zup(P.x, P.y, P.z - 0.05));
      this.lastFollow = zup(P.x, P.y, P.z - 0.05);
    }
    if (!w.robot || this.mode === 'free') {
      const c = meta.camera || { pos: [3, 3, 2], target: [0, 0, 0.5] };
      if (!w.robot) { this.camera.position.copy(zup(...c.pos)); this.controls.target.copy(zup(...c.target)); }
    }
    this.controls.update();
  }
  // follow camera: the user orbits / zooms an offset around the robot; walls between the robot and that offset pull the
  // camera in (MuJoCo ray against the scene's collision geoms, group 3)
  _followCamera() {
    const tgt = this.controls.target, cam = this.camera.position;
    const now = performance.now(), dt = Math.min(0.1, (now - (this._tf || now)) / 1000); this._tf = now;
    if (this._desired) {
      // chase: while the robot walks and the user is not orbiting, swing the offset behind the robot
      const R = this.w.robot, moving = R && !R.fell && Math.hypot(R.cmd[0], R.cmd[1]) + Math.abs(R.cmd[2]) * 0.3 > 0.05;
      if (moving && now - (this._userT || 0) > 1500) {
        const d = fromThree(this._desired), r = Math.hypot(d[0], d[1]), yaw = this.w.pelvisPose().yaw;
        let az = Math.atan2(d[1], d[0]); const goal = yaw + Math.PI;
        az += Math.atan2(Math.sin(goal - az), Math.cos(goal - az)) * Math.min(1, 2.2 * dt);
        this._desired.copy(zup(r * Math.cos(az), r * Math.sin(az), d[2]));
      }
      cam.copy(tgt).add(this._desired);
    }
    this.controls.update();
    this._desired = cam.clone().sub(tgt);
    const L = this._desired.length();
    if (L < 1e-3) return;
    const dir = this._desired.clone().divideScalar(L), w = this.w, mj = w.mj;
    const p0 = fromThree(tgt), dz = fromThree(dir), start = 0.25;
    this._gid ??= new mj.IntBuffer(1);
    const free = (d) => {   // free distance from the robot along d (z up), scene collision geoms only
      const pnt = [p0[0] + d[0] * start, p0[1] + d[1] * start, p0[2] + d[2] * start];
      let hit = -1;
      try { hit = mj.mj_ray(w.model, w.data, pnt, d, [0, 0, 0, 1, 0, 0], true, -1, this._gid, null); } catch (e) { hit = -1; }
      return hit >= 0 ? hit + start : Infinity;
    };
    // a wall behind the robot: first try the same direction, then look from higher up (up to ~60 deg more)
    const el0 = Math.asin(Math.max(-1, Math.min(1, dz[2]))), az = Math.atan2(dz[1], dz[0]);
    let best = null;
    for (let k = 0; k <= 4; k++) {
      const el = Math.min(1.35, el0 + k * 0.22);
      const d = [Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el)];
      const dist = Math.min(L, free(d) - 0.12);
      if (!best || dist > best.dist + 0.05) best = { d, dist };
      if (dist >= Math.min(L, 1.6)) break;
    }
    if (best.dist < L || best.d !== dz) {
      const v = zup(best.d[0], best.d[1], best.d[2]);
      cam.copy(tgt).addScaledVector(v, Math.max(0.35, best.dist));
      this.camera.lookAt(tgt);
    }
  }

  setMode(mode) {
    this.mode = mode; this._desired = null;
    if (mode === 'free') {
      const c = this.w.meta.camera;
      if (c) { this.camera.position.copy(zup(...c.pos)); this.controls.target.copy(zup(...c.target)); }
    } else this._initCamera();
  }

  resize() {
    const r = this.canvas.getBoundingClientRect();
    const W = Math.max(2, Math.round(r.width)), H = Math.max(2, Math.round(r.height));
    if (W === this._w && H === this._h) return;
    this._w = W; this._h = H;
    this.renderer.setSize(W, H, false);
    this.camera.aspect = W / H;
    // portrait screens: keep about 60 deg horizontally
    const base = this.opts.fov || 60;
    this.camera.fov = W >= H ? base : Math.min(80, 2 * Math.atan(Math.tan(base * Math.PI / 360) / (W / H)) * 180 / Math.PI);
    this.camera.updateProjectionMatrix();
  }

  sync() {
    const w = this.w; w._checkViews();
    const v = w.v, T = this._tmp;
    for (const e of this.bodyNodes) {
      this._bodyMatrix(e.b, v.xpos, v.xmat, T);
      e.node.matrix.multiplyMatrices(T, e.restInv).multiply(e.local);
      e.node.matrixWorldNeedsUpdate = true;
    }
    if (this.robotGeoms) {
      const gx = v.geom_xpos, gm = v.geom_xmat;
      for (const { g, mesh } of this.robotGeoms) {
        const o = 9 * g, p = 3 * g;
        mesh.matrix.set(gm[o], gm[o + 1], gm[o + 2], gx[p], gm[o + 3], gm[o + 4], gm[o + 5], gx[p + 1], gm[o + 6], gm[o + 7], gm[o + 8], gx[p + 2], 0, 0, 0, 1);
        mesh.matrixWorldNeedsUpdate = true;
      }
      const P = w.pelvisPose();
      this.blob.position.set(P.x, P.y, (w.meta.floor_z || 0) + 0.004);
      const s = Math.max(0.3, 1.2 - Math.max(0, P.z - (w.meta.floor_z || 0) - 0.6));
      this.blob.scale.set(s, s, 1);
      if (this.mode === 'follow') {
        const tgt = zup(P.x, P.y, P.z - 0.05);
        this.controls.target.copy(tgt); this.lastFollow.copy(tgt);   // the camera keeps its offset (render())
      }
    }
    if (w.drag) {
      const p = zup(...w.dragPoint()), t = zup(...w.drag.target);
      const pos = this.dragLine.geometry.attributes.position;
      pos.setXYZ(0, p.x, p.y, p.z); pos.setXYZ(1, t.x, t.y, t.z); pos.needsUpdate = true;
      this.dragLine.visible = true; this.dragDot.visible = true; this.dragDot.position.copy(t);
    } else { this.dragLine.visible = false; this.dragDot.visible = false; }
  }

  render() {
    this.resize();
    if (this.mode === 'follow' && this.w.robot) this._followCamera();
    else { this._desired = null; this.controls.update(); }
    if (this.ceiling.length) { const show = this.camera.position.y < this.ceilZ; for (const c of this.ceiling) c.visible = show; }
    this.renderer.render(this.scene, this.camera);
  }

  // ---------- pointer: hover highlight + drag ----------
  _bindPointer() {
    const el = this.canvas, ray = new THREE.Raycaster(), ndc = new THREE.Vector2();
    const pick = (ev) => {
      const r = el.getBoundingClientRect();
      ndc.set(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1);
      ray.setFromCamera(ndc, this.camera);
      const hit = ray.intersectObjects(this.pickables, false)[0];
      return hit || null;
    };
    let plane = null, lastHover = 0;
    el.addEventListener('pointerdown', (ev) => {
      if (ev.button !== 0) return;
      const hit = pick(ev);
      if (!hit || hit.object.userData.body == null) return;
      const b = hit.object.userData.body;
      if (this.opts.noRobotDrag && hit.object.userData.robot) return;
      this.controls.enabled = false;
      el.setPointerCapture(ev.pointerId);
      const n = new THREE.Vector3(); this.camera.getWorldDirection(n);
      plane = new THREE.Plane().setFromNormalAndCoplanarPoint(n, hit.point);
      this.w.startDrag(b, fromThree(hit.point));
      this.opts.onDrag?.(b, true);
      ev.preventDefault();
    });
    el.addEventListener('pointermove', (ev) => {
      if (plane && this.w.drag) {
        const r = el.getBoundingClientRect();
        ndc.set(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1);
        ray.setFromCamera(ndc, this.camera);
        const q = new THREE.Vector3();
        if (ray.ray.intersectPlane(plane, q)) this.w.moveDrag(fromThree(q));
        return;
      }
      const now = performance.now();
      if (now - lastHover < 60) return; lastHover = now;
      const hit = pick(ev);
      this._setHover(hit ? hit.object.userData.body : null, ev);
    });
    const up = (ev) => {
      if (!plane) return;
      plane = null; this.w.endDrag(); this.controls.enabled = true;
      try { el.releasePointerCapture(ev.pointerId); } catch (e) { /* ignore */ }
      this.opts.onDrag?.(null, false);
    };
    el.addEventListener('pointerup', up); el.addEventListener('pointercancel', up);
    el.addEventListener('pointerleave', () => { if (!plane) this._setHover(null); });
  }

  _setHover(b, ev) {
    if (b === this.hover) { if (b != null) this.opts.onHover?.(this.hoverInfo(b), ev); return; }
    const set = (body, on) => {
      for (const o of this.pickables) {
        if (o.userData.body !== body) continue;
        for (const m of [].concat(o.material)) {
          if (m.isMeshBasicMaterial) { if (on) { m.userData.c0 = m.color.clone(); m.color.multiplyScalar(1.35); } else if (m.userData.c0) m.color.copy(m.userData.c0); }
          else if (m.emissive) m.emissive.setHex(on ? 0x332a10 : 0x000000);
        }
      }
    };
    if (this.hover != null) set(this.hover, false);
    this.hover = b;
    if (b != null) set(b, true);
    this.canvas.style.cursor = b != null ? 'grab' : '';
    this.opts.onHover?.(b != null ? this.hoverInfo(b) : null, ev);
  }
  hoverInfo(b) {
    const name = this.w.bodyName(b);
    if (this.w.robotBodies && this.w.robotBodies.has(b)) return { name, label: 'Unitree G1 (drag to push)', robot: true };
    const L = this.labels.get(name);
    const kind = L?.kind || (this.w.jointValue(b) == null ? 'free' : 'joint');
    const verb = kind === 'hinge' ? 'drag to open' : kind === 'slide' ? 'drag to pull out' : 'drag to move';
    return { name, label: `${L?.label || name.replace(/_inst\d+$/, '').replace(/_/g, ' ')} (${verb})`, kind };
  }

  dispose() { this.renderer.dispose(); }
}
