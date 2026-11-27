// One physics world: a scene package (physics.xml + meshes) in MuJoCo WASM, optionally with a Unitree G1 driven by
// its locomotion policy. No rendering here (see view.js).
import { G1Policy } from './policy.js';

const PFX = 'g1_';

// ---------- asset loading ----------
export class Progress {
  constructor(cb) { this.cb = cb; this.total = 0; this.done = 0; }
  async fetchBytes(url) {
    const r = await fetch(url);
    if (!r.ok) throw new Error(`HTTP ${r.status} for ${url}`);
    const len = +r.headers.get('content-length') || 0;
    this.total += len;
    if (!r.body || !len) { const b = new Uint8Array(await r.arrayBuffer()); this.done += b.length; if (!len) this.total += b.length; this.cb?.(this); return b; }
    const reader = r.body.getReader(); const chunks = []; let n = 0;
    for (;;) {
      const { done, value } = await reader.read();
      if (done) break;
      chunks.push(value); n += value.length; this.done += value.length; this.cb?.(this);
    }
    const out = new Uint8Array(n); let o = 0;
    for (const c of chunks) { out.set(c, o); o += c.length; }
    return out;
  }
  async fetchText(url) { return new TextDecoder().decode(await this.fetchBytes(url)); }
}

const fileRefs = (xml) => [...new Set([...xml.matchAll(/\sfile="([^"]+)"/g)].map((m) => m[1]))];

export async function loadScenePackage(base, prog) {
  const meta = JSON.parse(await prog.fetchText(base + 'meta.json'));
  const glbP = prog.fetchBytes(base + 'visual.glb');
  const xmlBytes = await prog.fetchBytes(base + 'physics.xml');
  let xml = new TextDecoder().decode(xmlBytes);
  const files = new Map();
  // physics_web.xml (pack_physics.py): the same model with every hull inlined as vertices, valid only if it was packed
  // from this very physics.xml (sha1 in its first line)
  let web = null;
  const refs0 = fileRefs(xml);
  if (refs0.length) { try { web = await prog.fetchText(base + 'physics_web.xml'); } catch (e) { web = null; } }
  if (web) {
    const m = web.match(/src-sha1=([0-9a-f]{40})/);
    let ok = !!m;
    if (ok && globalThis.crypto?.subtle) {
      const h = [...new Uint8Array(await crypto.subtle.digest('SHA-1', xmlBytes))].map((b) => b.toString(16).padStart(2, '0')).join('');
      ok = h === m[1];
    }
    if (ok) return { base, meta, xml: web, files, glb: await glbP, packed: true };
    console.warn('[demo] physics_web.xml is stale (packed from another physics.xml); loading the mesh files');
  }
  const dirs = dirAttrs(xml);
  const refs = fileRefs(xml);
  let k = 0;   // at most 8 requests in flight (thousands of hull files otherwise exhaust the browser)
  await Promise.all(Array.from({ length: Math.min(8, refs.length) }, async () => {
    while (k < refs.length) { const f = refs[k++]; files.set(f, await prog.fetchBytes(base + joinDir(dirs, f))); }
  }));
  return { base, meta, xml, files, glb: await glbP, packed: false };
}

export async function loadRobotPackage(base, prog) {
  const xml = await prog.fetchText(base + 'g1_12dof.xml');
  const files = new Map();
  await Promise.all(fileRefs(xml).map(async (f) => { files.set(f, await prog.fetchBytes(base + f)); }));
  const policyCfg = JSON.parse(await prog.fetchText(base + 'policy.json'));
  const w = await prog.fetchBytes(base + 'policy.bin');
  return { base, xml, files, policyCfg, policyW: new Float32Array(w.buffer, w.byteOffset, w.byteLength / 4) };
}

function dirAttrs(xml) {
  const m = xml.match(/<compiler\b[^>]*>/);
  const get = (a) => (m && (m[0].match(new RegExp(`\\s${a}="([^"]*)"`)) || [])[1]) || '';
  return { meshdir: get('meshdir'), texturedir: get('texturedir'), assetdir: get('assetdir') };
}
// path of a file attribute relative to physics.xml (only meshes / hfields / skins use meshdir, textures texturedir)
function joinDir(dirs, f) {
  const d = dirs.meshdir || dirs.assetdir || '';
  return d && !f.startsWith('/') ? d.replace(/\/?$/, '/') + f : f;
}

// ---------- XML composition ----------
export function composeXML(scene, robot) {
  const P = new DOMParser();
  const sd = P.parseFromString(scene.xml, 'application/xml');
  const err = sd.querySelector('parsererror');
  if (err) throw new Error('physics.xml: ' + err.textContent);
  const root = sd.documentElement;
  const child = (el, tag) => [...el.children].find((c) => c.tagName === tag);
  const ensure = (tag, before) => {
    let e = child(root, tag);
    if (!e) { e = sd.createElement(tag); before ? root.insertBefore(e, before) : root.appendChild(e); }
    return e;
  };
  const comp = child(root, 'compiler');
  const dirs = { meshdir: comp?.getAttribute('meshdir') || '', texturedir: comp?.getAttribute('texturedir') || '', assetdir: comp?.getAttribute('assetdir') || '' };
  for (const el of root.querySelectorAll('[file]')) {
    const f = el.getAttribute('file');
    const d = el.tagName === 'texture' ? (dirs.texturedir || dirs.assetdir) : (dirs.meshdir || dirs.assetdir);
    el.setAttribute('file', 'scene/' + (d && !f.startsWith('/') ? d.replace(/\/?$/, '/') : '') + f);
  }
  if (comp) { comp.removeAttribute('meshdir'); comp.removeAttribute('texturedir'); comp.removeAttribute('assetdir'); }
  const angleRad = comp?.getAttribute('angle') === 'radian';
  // physics options for the robot: 2 ms steps
  const opt = ensure('option', child(root, 'asset') || child(root, 'worldbody'));
  opt.setAttribute('timestep', '0.002');
  if (!robot) return new XMLSerializer().serializeToString(sd);

  // one inactive weld per free object of the scene: the assisted grasp switches one on at runtime (eq_active)
  const freeNames = [];
  for (const b of root.querySelectorAll('worldbody body')) {
    const n = b.getAttribute('name');
    if (n && [...b.children].some((c) => c.tagName === 'freejoint' || (c.tagName === 'joint' && c.getAttribute('type') === 'free'))) freeNames.push(n);
  }
  if (freeNames.length) {
    const eq = child(root, 'equality') || ensure('equality');
    for (const n of freeNames) {
      const w = sd.createElement('weld');
      w.setAttribute('name', PFX + 'grab_' + n); w.setAttribute('body1', PFX + 'pelvis'); w.setAttribute('body2', n); w.setAttribute('active', 'false');
      eq.appendChild(w);
    }
  }
  const rd = P.parseFromString(robot.xml, 'application/xml');
  const rr = rd.documentElement;
  for (const el of rr.querySelectorAll('[file]')) el.setAttribute('file', 'g1/' + el.getAttribute('file'));
  for (const a of ['name', 'mesh', 'joint', 'material', 'body']) {
    for (const el of rr.querySelectorAll(`[${a}]`)) el.setAttribute(a, PFX + el.getAttribute(a));
  }
  if (!angleRad) {   // the scene compiles in degrees: convert the robot's hinge ranges
    for (const j of rr.querySelectorAll('joint[range]')) {
      const t = j.getAttribute('type') || 'hinge';
      if (t === 'hinge' || t === 'ball') j.setAttribute('range', j.getAttribute('range').trim().split(/\s+/).map((v) => (+v * 180 / Math.PI).toFixed(6)).join(' '));
    }
  }
  // robot defaults -> its own class
  const rdef = child(rr, 'default');
  let sdef = child(root, 'default');
  if (!sdef) { sdef = sd.createElement('default'); root.insertBefore(sdef, child(root, 'asset') || child(root, 'worldbody')); }
  const cls = sd.createElement('default'); cls.setAttribute('class', PFX + 'main');
  if (rdef) for (const c of [...rdef.children]) cls.appendChild(sd.importNode(c, true));
  sdef.appendChild(cls);
  const asset = child(root, 'asset') || ensure('asset', child(root, 'worldbody'));
  const ra = child(rr, 'asset');
  if (ra) for (const c of [...ra.children]) asset.appendChild(sd.importNode(c, true));
  const wb = child(root, 'worldbody') || ensure('worldbody');
  const pelvis = child(child(rr, 'worldbody'), 'body');
  const sp = scene.meta.spawn || { pos: [0, 0], yaw: 0 };
  const fz = scene.meta.floor_z || 0;
  const h = robot.policyCfg.base_height || 0.793;
  pelvis.setAttribute('pos', `${sp.pos[0]} ${sp.pos[1]} ${fz + h}`);
  pelvis.setAttribute('quat', `${Math.cos(sp.yaw / 2)} 0 0 ${Math.sin(sp.yaw / 2)}`);
  pelvis.setAttribute('childclass', PFX + 'main');
  pelvis.setAttribute('sleep', 'never');
  wb.appendChild(sd.importNode(pelvis, true));
  const act = child(root, 'actuator') || ensure('actuator');
  const rac = child(rr, 'actuator');
  if (rac) for (const c of [...rac.children]) act.appendChild(sd.importNode(c, true));
  return new XMLSerializer().serializeToString(sd);
}

// ---------- the world ----------
export class SimWorld {
  constructor(mj, scene, robot, opts = {}) {
    this.mj = mj; this.scene = scene; this.robotPkg = robot; this.meta = scene.meta; this.opts = opts;
    this.xml = composeXML(scene, robot);
    const vfs = new mj.MjVFS();
    try {
      for (const [f, b] of scene.files) vfs.addBuffer('scene/' + joinDir(dirAttrs(scene.xml), f), b);
      if (robot) for (const [f, b] of robot.files) vfs.addBuffer('g1/' + f, b);
      vfs.addBuffer('model.xml', new TextEncoder().encode(this.xml));
      this.model = mj.MjModel.from_xml_path('model.xml', vfs);
    } finally { vfs.delete(); }
    this.data = new mj.MjData(this.model);
    const m = this.model;
    if (opts.sleep) m.opt.enableflags |= mj.mjtEnableBit.mjENBL_SLEEP.value;
    this.dt = m.opt.timestep;
    this.nbody = m.nbody;
    const OBJ = mj.mjtObj;
    this.id = (type, name) => mj.mj_name2id(m, OBJ[type].value, name);
    this.bodyName = (b) => mj.mj_id2name(m, OBJ.mjOBJ_BODY.value, b);
    this._views();
    // robot indices
    this.robot = null;
    if (robot) {
      const c = robot.policyCfg;
      const base = this.id('mjOBJ_JOINT', PFX + 'floating_base_joint');
      const pelvis = this.id('mjOBJ_BODY', PFX + 'pelvis');
      const jq = [], jd = [], act = [];
      for (const j of c.joints) {
        const jid = this.id('mjOBJ_JOINT', PFX + j);
        jq.push(this.v.jnt_qposadr[jid]); jd.push(this.v.jnt_dofadr[jid]);
        act.push(this.id('mjOBJ_ACTUATOR', PFX + j));
      }
      this.robot = {
        cfg: c, policy: new G1Policy(c, robot.policyW), pelvis, bq: this.v.jnt_qposadr[base], bd: this.v.jnt_dofadr[base],
        jq, jd, act, kps: Float64Array.from(c.kps), kds: Float64Array.from(c.kds), dflt: Float64Array.from(c.default_angles),
        action: new Float32Array(12), target: Float64Array.from(c.default_angles), obs: new Float32Array(c.num_obs),
        counter: 0, cmd: [0, 0, 0], cmdGoal: [0, 0, 0], fell: false, fellAt: 0, spawnQ: null,
        mode: 'walk', zeroT: 0, stand: null, standCount: 0,
        // stance PD: stiffer ankles (the policy's ankle kp of 40 is below m*g*h of the standing robot)
        skp: Float64Array.from([150, 150, 150, 200, 300, 150, 150, 150, 150, 200, 300, 150]),
        skd: Float64Array.from([4, 4, 4, 6, 6, 4, 4, 4, 4, 6, 6, 4]),
      };
      this.robotBodies = new Set();
      for (let b = 0; b < m.nbody; b++) if (this.v.body_rootid[b] === pelvis) this.robotBodies.add(b);
      const R = this.robot;
      R.feet = ['left_ankle_roll_link', 'right_ankle_roll_link'].map((n) => this.id('mjOBJ_BODY', PFX + n));
      R.footGeoms = R.feet.map((f) => { const out = []; for (let g = 0; g < m.ngeom; g++) if (m.geom_bodyid[g] === f && m.geom_type[g] === 2) out.push(g); return out; });
      // contact bits: scene geoms 1|2, robot geoms 1; a carried object gets 2 (touches the scene, not the robot)
      const ct = m.geom_contype, ca = m.geom_conaffinity;
      for (let g = 0; g < m.ngeom; g++) {
        if (this.robotBodies.has(m.geom_bodyid[g]) || (ct[g] === 0 && ca[g] === 0)) continue;
        ct[g] |= 2; ca[g] |= 2;
      }
    }
    this.mj.mj_forward(m, this.data);
    this._views();
    if (this.robot) {
      this.robot.spawnQ = Float64Array.from(this.v.qpos.subarray(this.robot.bq, this.robot.bq + 7));
      for (let i = 0; i < 12; i++) this.v.qpos[this.robot.jq[i]] = this.robot.dflt[i];
      this.mj.mj_forward(m, this.data);
    }
    // rest poses of every body (for the visual sync)
    this.rest = { xpos: Float64Array.from(this.v.xpos), xmat: Float64Array.from(this.v.xmat) };
    this.qpos0 = Float64Array.from(this.v.qpos);
    this.moving = [];
    for (let b = 1; b < m.nbody; b++) if (this.v.body_weldid[b] !== 0 && !(this.robotBodies && this.robotBodies.has(b))) this.moving.push(b);
    this.simTime = 0; this.simTarget = 0; this.drag = null;
    this.stats = { steps: 0, simSec: 0, wallMs: 0, rtf: 1, physMsPerStep: 0 };
    this.nav = null;   // set by Navigator
    this.onEvent = null;
    this._initFreeze(opts);
    this._initGrasp();
  }

  // ---- speed fallback for packages without MuJoCo sleeping: free objects far from the robot and from the mouse are
  // pinned (no contacts, pose held) and released when the robot or a dragged object comes within `activeRadius`.
  // Packages that start their objects asleep (<flag sleep="enable"/>, body sleep="init") do not need it: MuJoCo
  // skips sleeping trees and wakes them on contact or on an applied force (the mouse drag).
  _initFreeze(opts) {
    const m = this.model, v = this.v;
    this.free = [];
    for (let b = 1; b < m.nbody; b++) {
      if (this.robotBodies && this.robotBodies.has(b)) continue;
      if (v.body_jntnum[b] > 0 && v.jnt_type[v.body_jntadr[b]] === 0) this.free.push(b);
    }
    const sleeping = (m.opt.enableflags & this.mj.mjtEnableBit.mjENBL_SLEEP.value) !== 0;
    const auto = !sleeping && this.free.length > 25;
    this.freeze = { on: opts.freeze ?? auto, R: opts.activeRadius || 1.3, frozen: new Map(), geoms: new Map(), ct: null, ca: null, hot: [], tick: 0 };
    if (!this.freeze.on) return;
    const gb = m.geom_bodyid, set = new Set(this.free);
    this.freeze.ct = Int32Array.from(m.geom_contype); this.freeze.ca = Int32Array.from(m.geom_conaffinity);
    for (let g = 0; g < m.ngeom; g++) if (set.has(gb[g])) { if (!this.freeze.geoms.has(gb[g])) this.freeze.geoms.set(gb[g], []); this.freeze.geoms.get(gb[g]).push(g); }
    for (const b of this.free) this._freezeBody(b, true);
    this._updateFreeze(true);
  }
  _freezeBody(b, on) {
    const F = this.freeze, m = this.model, v = this.v;
    if (on === F.frozen.has(b)) return;
    const ct = m.geom_contype, ca = m.geom_conaffinity;
    for (const g of F.geoms.get(b) || []) { ct[g] = on ? 0 : F.ct[g]; ca[g] = on ? 0 : F.ca[g]; }
    const j = v.body_jntadr[b], qa = v.jnt_qposadr[j], da = v.jnt_dofadr[j];
    if (on) { for (let i = 0; i < 6; i++) v.qvel[da + i] = 0; F.frozen.set(b, { qa, da, q: Float64Array.from(v.qpos.subarray(qa, qa + 7)) }); }
    else F.frozen.delete(b);
  }
  _pinFrozen() {
    const v = this.v;
    for (const { qa, da, q } of this.freeze.frozen.values()) { v.qpos.set(q, qa); for (let i = 0; i < 6; i++) v.qvel[da + i] = 0; }
  }
  // wake objects near the robot / the dragged object / recent drags; pin far ones once they are at rest
  _updateFreeze(force = false) {
    const F = this.freeze; if (!F.on) return;
    if (!force && (++F.tick % 25)) return;
    const v = this.v, xp = v.xpos, qv = v.qvel, now = this.simTime;
    const centers = [];
    const P = this.robot ? this.pelvisPose() : null;
    if (P) centers.push([P.x, P.y, F.R]);
    if (this.drag) { const p = this.dragPoint(); centers.push([p[0], p[1], 0.9]); F.hot.push([p[0], p[1], now + 4]); }
    F.hot = F.hot.filter((h) => h[2] > now);
    for (const h of F.hot) centers.push([h[0], h[1], 0.9]);
    for (const b of this.free) {
      const x = xp[3 * b], y = xp[3 * b + 1];
      let near = this.drag && this.drag.b === b;
      for (const c of centers) if (!near && Math.hypot(x - c[0], y - c[1]) < c[2]) near = true;
      if (near) { this._freezeBody(b, false); continue; }
      if (F.frozen.has(b)) continue;
      const d = v.jnt_dofadr[v.body_jntadr[b]];
      const speed = Math.hypot(qv[d], qv[d + 1], qv[d + 2]) + 0.2 * Math.hypot(qv[d + 3], qv[d + 4], qv[d + 5]);
      if (speed < 0.03) this._freezeBody(b, true);
    }
  }

  // ---- geometry helpers ----
  geomsOf(b) {
    if (!this._geomIndex) {
      this._geomIndex = new Map(); const gb = this.model.geom_bodyid;
      for (let g = 0; g < this.model.ngeom; g++) { if (!this._geomIndex.has(gb[g])) this._geomIndex.set(gb[g], []); this._geomIndex.get(gb[g]).push(g); }
    }
    return this._geomIndex.get(b) || [];
  }
  // world points of a geom: hull vertices for meshes, box corners of the local AABB otherwise
  geomPoints(g, out = []) {
    const m = this.model, v = this.v, R = v.geom_xmat, p = v.geom_xpos, o = 9 * g, P = 3 * g;
    const push = (l0, l1, l2) => out.push([p[P] + R[o] * l0 + R[o + 1] * l1 + R[o + 2] * l2, p[P + 1] + R[o + 3] * l0 + R[o + 4] * l1 + R[o + 5] * l2, p[P + 2] + R[o + 6] * l0 + R[o + 7] * l1 + R[o + 8] * l2]);
    if (m.geom_type[g] === 7) {
      const id = m.geom_dataid[g], a = 3 * m.mesh_vertadr[id], n = m.mesh_vertnum[id], V = m.mesh_vert;
      for (let i = 0; i < n; i++) push(V[a + 3 * i], V[a + 3 * i + 1], V[a + 3 * i + 2]);
    } else {
      const A = m.geom_aabb, c = 6 * g;
      for (let s8 = 0; s8 < 8; s8++) push(A[c] + (s8 & 1 ? 1 : -1) * A[c + 3], A[c + 1] + (s8 & 2 ? 1 : -1) * A[c + 4], A[c + 2] + (s8 & 4 ? 1 : -1) * A[c + 5]);
    }
    return out;
  }
  bodyBox(b) {
    const min = [1e9, 1e9, 1e9], max = [-1e9, -1e9, -1e9];
    for (const g of this.geomsOf(b)) for (const q of this.geomPoints(g)) for (let k = 0; k < 3; k++) { min[k] = Math.min(min[k], q[k]); max[k] = Math.max(max[k], q[k]); }
    return { min, max };
  }

  // ---- assisted grasp. The 12-DoF G1 has no arm joints (its arms are fixed to the torso), so a small free object
  // (<= 2 kg, <= 40 cm) within reach is moved kinematically to a carry pose in front of the hands and then held by a
  // MuJoCo weld to the pelvis that is switched on at runtime (eq_active); releasing switches it off and the object
  // falls / settles under physics. While carried it touches the scene but not the robot (contact bit 2 only).
  _initGrasp() {
    this.grasp = null;
    if (!this.robot) return;
    const m = this.model, v = this.v;
    const G = { held: null, anim: null, loose: [], eq: new Map(), small: new Map(), info: null, tick: 0,
      ct0: Int32Array.from(m.geom_contype), ca0: Int32Array.from(m.geom_conaffinity) };
    for (const b of this.free) {
      const e = this.id('mjOBJ_EQUALITY', PFX + 'grab_' + this.bodyName(b));
      if (e < 0) continue;
      G.eq.set(b, e);
      const mass = v.body_subtreemass[b], bb = this.bodyBox(b);
      const ext = Math.max(bb.max[0] - bb.min[0], bb.max[1] - bb.min[1], bb.max[2] - bb.min[2]);
      if (mass > 0 && mass <= 2.0 && ext <= 0.4) G.small.set(b, { mass, ext, r: Math.min(0.2, 0.5 * Math.max(bb.max[0] - bb.min[0], bb.max[1] - bb.min[1])) });
    }
    this.grasp = G;
  }
  _setCarryContacts(b, carried) {
    const G = this.grasp, ct = this.v.geom_contype, ca = this.v.geom_conaffinity;
    for (const g of this.geomsOf(b)) { ct[g] = carried ? 2 : G.ct0[g]; ca[g] = carried ? 2 : G.ca0[g]; }
  }
  _restoreContacts(all) { const G = this.grasp; for (const L of (all ? [...G.loose] : [])) this._setCarryContacts(L.b, false); if (all) G.loose = []; }
  holding() { const G = this.grasp; if (!G) return null; const b = G.held ?? (G.anim && G.anim.kind === 'pick' ? G.anim.b : null); return b; }
  // the nearest small object within reach (in front of the robot, horizontal distance <= 0.85 m from the pelvis)
  reachable(maxD = 0.85) {
    const G = this.grasp, R = this.robot; if (!G || R.fell) return null;
    const P = this.pelvisPose(), fz = this.meta.floor_z || 0, xi = this.v.xipos;
    let best = null, bd = 1e9;
    for (const [b] of G.small) {
      const dx = xi[3 * b] - P.x, dy = xi[3 * b + 1] - P.y, d = Math.hypot(dx, dy), h = xi[3 * b + 2] - fz;
      const fwd = (dx * Math.cos(P.yaw) + dy * Math.sin(P.yaw)) / Math.max(1e-6, d);
      if (d > maxD || h > 1.45 || h < -0.05 || (d > 0.3 && fwd < 0.2)) continue;
      if (d < bd) { bd = d; best = b; }
    }
    return best;
  }
  pick(b) {
    const G = this.grasp, R = this.robot;
    if (!G || R.fell || G.held != null || G.anim || !G.small.has(b)) return false;
    const v = this.v, P = this.pelvisPose();
    const qP = Array.from(v.xquat.subarray(4 * R.pelvis, 4 * R.pelvis + 4)), qO = Array.from(v.xquat.subarray(4 * b, 4 * b + 4));
    const j = v.body_jntadr[b];
    // keep the object's orientation relative to the robot; carry at chest height (above tables and counters)
    const rel = qmul(qconj(qP), qO);
    G.anim = { kind: 'pick', b, t0: this.simTime, dur: 0.7, c0: Array.from(v.xipos.subarray(3 * b, 3 * b + 3)), q0: qO, rel,
      carry: [0.24 + G.small.get(b).r, 0, 0.25], qa: v.jnt_qposadr[j], da: v.jnt_dofadr[j], ipos: Array.from(this.model.body_ipos.subarray(3 * b, 3 * b + 3)) };
    G.loose = G.loose.filter((L) => L.b !== b);
    this._setCarryContacts(b, true);
    this.nav?.cancel();
    this.onEvent?.('pick', b);
    return true;
  }
  // release what is held; place = [x, y, z] of a support point to put it on (null: let it drop where it is)
  release(place = null, instant = false) {
    const G = this.grasp; if (!G) return false;
    let b = G.held;
    if (G.anim) { b = G.anim.b; G.anim = null; }
    if (b == null) return false;
    const e = G.eq.get(b); this.v.eq_active[e] = 0; G.held = null;
    const v = this.v, j = v.body_jntadr[b];
    if (place && !instant) {
      const box = this.bodyBox(b), com = Array.from(v.xipos.subarray(3 * b, 3 * b + 3));
      const to = [place[0], place[1], place[2] + (com[2] - box.min[2]) + 0.012];
      G.anim = { kind: 'place', b, t0: this.simTime, dur: 0.6, c0: com, to, q0: Array.from(v.xquat.subarray(4 * b, 4 * b + 4)),
        qa: v.jnt_qposadr[j], da: v.jnt_dofadr[j], ipos: Array.from(this.model.body_ipos.subarray(3 * b, 3 * b + 3)) };
    } else G.loose.push({ b, t: this.simTime });
    this.onEvent?.('release', b);
    return true;
  }
  // where to put a held object: rays straight down in front of the robot onto a surface 0.2-1.4 m high (prefer the
  // geoms of `name`, a landmark's object); null if none
  findPlacement(name = null) {
    const G = this.grasp; if (!G || G.held == null) return null;
    const mj = this.mj, P = this.pelvisPose(), fz = this.meta.floor_z || 0;
    this._gidP ??= new mj.IntBuffer(1);
    // hits along lines fanning out ahead of the robot (straight first, then up to +-40 deg)
    const lines = [];
    for (const da of [0, 0.35, -0.35, 0.7, -0.7]) for (const sd of [0, 0.12, -0.12]) {
      const yaw = P.yaw + da, hs = [];
      for (let f = 0.3; f <= 1.1; f += 0.05) {
        const x = P.x + f * Math.cos(yaw) - sd * Math.sin(yaw), y = P.y + f * Math.sin(yaw) + sd * Math.cos(yaw);
        let hit = -1;
        try { hit = mj.mj_ray(this.model, this.data, [x, y, fz + 1.7], [0, 0, -1], [0, 0, 0, 1, 0, 0], true, G.held, this._gidP, null); } catch (e) { hit = -1; }
        if (hit < 0) continue;
        const z = fz + 1.7 - hit, g = this._gidP.GetView()[0];
        if (z < fz + 0.2 || z > fz + 1.4 || g < 0 || this.robotBodies.has(this.model.geom_bodyid[g])) continue;
        const gname = mj.mj_id2name(this.model, mj.mjtObj.mjOBJ_GEOM.value, g) || '';
        hs.push({ p: [x, y, z], mine: !!name && gname.startsWith(name), f });
      }
      lines.push(hs);
    }
    // a few centimetres in from the near edge of a flat run of hits (4th hit), on the named object if possible
    for (const want of name ? [true, false] : [false]) {
      for (const hs0 of lines) {
        const hs = hs0.filter((h) => !want || h.mine);
        if (hs.length < 2) continue;
        const k = Math.min(3, hs.length - 1);
        if (Math.abs(hs[k].p[2] - hs[0].p[2]) < 0.04) return hs[k].p;     // flat: not the edge of a backrest
      }
    }
    return null;
  }
  _graspStep() {
    const G = this.grasp, v = this.v, A = G.anim, R = this.robot;
    if (A) {
      const s = Math.min(1, (this.simTime - A.t0) / A.dur), e = s * s * (3 - 2 * s);
      let q, c;
      if (A.kind === 'pick') {
        const qP = Array.from(v.xquat.subarray(4 * R.pelvis, 4 * R.pelvis + 4)), pP = v.xpos.subarray(3 * R.pelvis, 3 * R.pelvis + 3);
        const ct = qrot(qP, A.carry).map((x, i) => x + pP[i]);
        const qT = qmul(qP, A.rel);
        q = qslerp(A.q0, qT, e);
        c = A.c0.map((x, i) => x + (ct[i] - x) * e + (i === 2 ? 0.1 * Math.sin(Math.PI * e) : 0));
      } else {
        q = A.q0; c = A.c0.map((x, i) => x + (A.to[i] - x) * e + (i === 2 ? 0.05 * Math.sin(Math.PI * e) : 0));
      }
      const off = qrot(q, A.ipos), pos = c.map((x, i) => x - off[i]);
      for (let i = 0; i < 3; i++) v.qpos[A.qa + i] = pos[i];
      for (let i = 0; i < 4; i++) v.qpos[A.qa + 3 + i] = q[i];
      for (let i = 0; i < 6; i++) v.qvel[A.da + i] = 0;
      if (s >= 1) {
        G.anim = null;
        if (A.kind === 'pick') {   // switch the weld on with the current relative pose
          const qP = Array.from(v.xquat.subarray(4 * R.pelvis, 4 * R.pelvis + 4)), pP = Array.from(v.xpos.subarray(3 * R.pelvis, 3 * R.pelvis + 3));
          const relp = qrot(qconj(qP), pos.map((x, i) => x - pP[i])), relq = qmul(qconj(qP), q);
          const e = G.eq.get(A.b), d = v.eq_data, o = (this.mj.mjNEQDATA || 11) * e;   // 12 values per equality in MuJoCo 4.0
          d[o] = 0; d[o + 1] = 0; d[o + 2] = 0; d[o + 3] = relp[0]; d[o + 4] = relp[1]; d[o + 5] = relp[2];
          d[o + 6] = relq[0]; d[o + 7] = relq[1]; d[o + 8] = relq[2]; d[o + 9] = relq[3]; d[o + 10] = 1;
          v.eq_active[e] = 1; G.held = A.b;
          this.onEvent?.('held', A.b);
        } else G.loose.push({ b: A.b, t: this.simTime });
      }
    }
    // released objects touch the robot again once they are clear of it
    if (G.loose.length && (++G.tick % 25) === 0) {
      const P = this.pelvisPose(), xi = v.xipos;
      G.loose = G.loose.filter((L) => {
        const d = Math.hypot(xi[3 * L.b] - P.x, xi[3 * L.b + 1] - P.y);
        if (d > 0.6 || this.simTime - L.t > 6) { this._setCarryContacts(L.b, false); return false; }
        return true;
      });
    }
  }

  activeCount() { return this.freeze.on ? this.free.length - this.freeze.frozen.size : this.free.length; }
  awakeTrees() { try { return this.data.ntree_awake; } catch (e) { return -1; } }

  _views() {
    const m = this.model, d = this.data;
    this.v = {
      qpos: d.qpos, qvel: d.qvel, ctrl: d.ctrl, xpos: d.xpos, xmat: d.xmat, xquat: d.xquat, xipos: d.xipos,
      geom_xpos: d.geom_xpos, geom_xmat: d.geom_xmat, xfrc: d.xfrc_applied, subtree_com: d.subtree_com,
      eq_active: d.eq_active, eq_data: m.eq_data, geom_contype: m.geom_contype, geom_conaffinity: m.geom_conaffinity,
      jnt_qposadr: m.jnt_qposadr, jnt_dofadr: m.jnt_dofadr, body_rootid: m.body_rootid, body_weldid: m.body_weldid,
      body_subtreemass: m.body_subtreemass, body_mass: m.body_mass, body_jntnum: m.body_jntnum, body_jntadr: m.body_jntadr,
      jnt_type: m.jnt_type, geom_bodyid: m.geom_bodyid,
    };
  }
  // WASM memory growth (e.g. a second world) detaches every cached typed-array view: re-create them when that happened
  _checkViews() { if (this.v.qpos.buffer.byteLength === 0 || this.v.xpos.length === 0) this._views(); }

  // ---- robot ----
  setCommand(vx, vy, wz) { if (this.robot) this.robot.cmdGoal = [vx, vy, wz]; }
  pelvisPose() {
    if (!this.robot) return null;
    this._checkViews();
    const q = this.v.qpos, b = this.robot.bq;
    const w = q[b + 3], z = q[b + 6], x = q[b + 4], y = q[b + 5];
    return { x: q[b], y: q[b + 1], z: q[b + 2], yaw: Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) };
  }
  resetRobot(pose) {
    const R = this.robot; if (!R) return;
    this._checkViews();
    const q = this.v.qpos, qv = this.v.qvel;
    const p = pose || null;
    for (let i = 0; i < 7; i++) q[R.bq + i] = R.spawnQ[i];
    if (p) { q[R.bq] = p.x; q[R.bq + 1] = p.y; q[R.bq + 3] = Math.cos(p.yaw / 2); q[R.bq + 4] = 0; q[R.bq + 5] = 0; q[R.bq + 6] = Math.sin(p.yaw / 2); }
    for (let i = 0; i < 12; i++) q[R.jq[i]] = R.dflt[i];
    for (let i = 0; i < 6; i++) qv[R.bd + i] = 0;
    for (let i = 0; i < 12; i++) qv[R.jd[i]] = 0;
    R.policy.reset(); R.action.fill(0); R.target.set(R.dflt); R.counter = 0; R.fell = false; R.cmd = [0, 0, 0]; R.cmdGoal = [0, 0, 0];
    R.mode = 'walk'; R.zeroT = 0; R.stand = null;
    if (this.grasp?.held != null || this.grasp?.anim) this.release(null, true);
    if (this.nav) this.nav.cancel();
    this.mj.mj_forward(this.model, this.data);
    this.onEvent?.('reset');
  }
  resetScene() {
    this._checkViews();
    if (this.grasp) { this.grasp.held = null; this.grasp.anim = null; this.grasp.loose = []; }
    this.mj.mj_resetData(this.model, this.data);
    this._checkViews();
    this.v.qpos.set(this.qpos0);
    if (this.grasp) this._restoreContacts(true);
    if (this.freeze.on) { for (const b of this.free) this._freezeBody(b, false); for (const b of this.free) this._freezeBody(b, true); this.freeze.hot = []; }
    this.mj.mj_forward(this.model, this.data);
    if (this.robot) this.resetRobot(); else this.onEvent?.('reset');
    this._updateFreeze(true);
  }

  _control() {
    const R = this.robot, v = this.v, c = R.cfg;
    const q = v.qpos, qv = v.qvel, o = R.obs;
    // command: navigation overrides the keyboard; rate limited
    const navOn = this.nav && this.nav.active;
    const goal = navOn ? this.nav.command(this.pelvisPose()) : R.cmdGoal;
    const lim = [0.04, 0.04, 0.08];
    for (let i = 0; i < 3; i++) R.cmd[i] += Math.max(-lim[i], Math.min(lim[i], goal[i] - R.cmd[i]));
    const zero = !navOn && Math.max(...goal.map(Math.abs)) < 0.02 && Math.max(...R.cmd.map(Math.abs)) < 0.02;
    const w = q[R.bq + 3], x = q[R.bq + 4], y = q[R.bq + 5], z = q[R.bq + 6];
    const gz = 1 - 2 * (w * w + z * z);
    if (R.mode === 'stand') {
      if (zero && !this._perturbed(gz)) { this._fallCheck(gz); return; }
      this._leaveStand();
    }
    o[0] = qv[R.bd + 3] * c.ang_vel_scale; o[1] = qv[R.bd + 4] * c.ang_vel_scale; o[2] = qv[R.bd + 5] * c.ang_vel_scale;
    o[3] = 2 * (-z * x + w * y); o[4] = -2 * (z * y + w * x); o[5] = gz;
    o[6] = R.cmd[0] * c.cmd_scale[0]; o[7] = R.cmd[1] * c.cmd_scale[1]; o[8] = R.cmd[2] * c.cmd_scale[2];
    for (let i = 0; i < 12; i++) {
      o[9 + i] = (q[R.jq[i]] - R.dflt[i]) * c.dof_pos_scale;
      o[21 + i] = qv[R.jd[i]] * c.dof_vel_scale;
      o[33 + i] = R.action[i];
    }
    const ph = ((R.counter * this.dt) % c.gait_period) / c.gait_period;
    o[45] = Math.sin(2 * Math.PI * ph); o[46] = Math.cos(2 * Math.PI * ph);
    const a = R.policy.act(o);
    for (let i = 0; i < 12; i++) { R.action[i] = a[i]; R.target[i] = a[i] * c.action_scale + R.dflt[i]; }
    // idle stance: the policy was trained with a gait clock and a foot-contact reward that make it step in place at
    // zero command; after 0.5 s of zero command, at a double-support instant of that clock, with the robot calm, hold
    // the pose with stiff ankles and a COM-over-feet feedback (see _standTarget)
    R.zeroT = zero ? R.zeroT + c.control_decimation * this.dt : 0;
    if (R.zeroT > 0.5 && this.opts.stance !== false && this._canStand(ph)) this._enterStand();
    this._fallCheck(gz);
  }
  _fallCheck(gz) {
    const R = this.robot, pz = this.v.qpos[R.bq + 2] - (this.meta.floor_z || 0);
    if (!R.fell && (pz < 0.45 || gz > -0.5)) {
      R.fell = true; R.fellAt = this.simTime; R.mode = 'walk'; R.stand = null; this.nav?.cancel('fell');
      if (this.grasp?.held != null) this.release(null, true);
      this.onEvent?.('fell');
    }
  }
  _canStand(ph) {
    const R = this.robot, v = this.v, qv = v.qvel;
    if (!((ph >= 0.5 && ph < 0.56) || ph < 0.06)) return false;          // double support of the gait clock
    if (Math.hypot(qv[R.bd], qv[R.bd + 1]) > 0.15 || Math.hypot(qv[R.bd + 3], qv[R.bd + 4], qv[R.bd + 5]) > 0.4) return false;
    const fz = R.footGeoms.map((gs) => Math.min(...gs.map((g) => v.geom_xpos[3 * g + 2])));
    return Math.abs(fz[0] - fz[1]) < 0.012;                              // both feet down
  }
  _comError() {   // COM minus the middle of the two feet, in the heading frame
    const R = this.robot, v = this.v, P = this.pelvisPose();
    const cx = Math.cos(P.yaw), sy = Math.sin(P.yaw);
    let fx = 0, fy = 0;
    for (const f of R.feet) { fx += v.xpos[3 * f] + v.xmat[9 * f] * 0.035; fy += v.xpos[3 * f + 1] + v.xmat[9 * f + 3] * 0.035; }
    const ex = v.subtree_com[3 * R.pelvis] - fx / 2, ey = v.subtree_com[3 * R.pelvis + 1] - fy / 2;
    return [ex * cx + ey * sy, -ex * sy + ey * cx];
  }
  _perturbed(gz) {   // a push or a lean the stance cannot hold: step again (the policy was trained with pushes)
    const R = this.robot, qv = this.v.qvel;
    if (Math.hypot(qv[R.bd], qv[R.bd + 1]) > 0.3 || gz > -Math.cos(10 * Math.PI / 180)) return true;
    const [ex, ey] = this._comError();
    return Math.abs(ex) > 0.07 || Math.abs(ey) > 0.09;
  }
  _enterStand() {
    const R = this.robot, q = this.v.qpos;
    R.stand = { hold: Float64Array.from(R.jq.map((a) => q[a])), from: Float64Array.from(R.target), blend: 0, pe: null };
    R.mode = 'stand'; R.standCount++;
    this.onEvent?.('stand');
  }
  _leaveStand() {
    const R = this.robot, q = this.v.qpos;
    R.mode = 'walk'; R.stand = null; R.zeroT = 0;
    R.policy.reset(); R.action.fill(0); R.counter = 0;
    for (let i = 0; i < 12; i++) R.target[i] = q[R.jq[i]];
    this.onEvent?.('walk');
  }
  // stance target each physics step: the held joint pose, ankles shifted to keep the COM over the feet
  _standTarget() {
    const R = this.robot, S = R.stand, dt = this.dt;
    S.blend = Math.min(1, S.blend + dt / 0.3);
    const [ex, ey] = this._comError();
    const dex = S.pe ? (ex - S.pe[0]) / dt : 0, dey = S.pe ? (ey - S.pe[1]) / dt : 0; S.pe = [ex, ey];
    const KB = 2, KBD = 0.2, clamp = (v, a) => Math.max(-a, Math.min(a, v));
    const dp = clamp(KB * ex + KBD * dex, 0.3), dr = clamp(-(KB * ey + KBD * dey), 0.2);
    for (let i = 0; i < 12; i++) {
      let t = S.hold[i];
      if (i === 4 || i === 10) t += dp;
      if (i === 5 || i === 11) t += dr;
      R.target[i] = S.from[i] + (t - S.from[i]) * S.blend;
    }
  }

  _physStep() {
    const v = this.v, R = this.robot;
    if (R) {
      const q = v.qpos, qv = v.qvel, ctrl = v.ctrl;
      if (R.mode === 'stand') this._standTarget();
      const b = R.mode === 'stand' ? R.stand.blend : 0;
      for (let i = 0; i < 12; i++) {
        const kp = R.fell ? 0 : R.kps[i] + (R.skp[i] - R.kps[i]) * b, kd = R.kds[i] + (R.skd[i] - R.kds[i]) * b;
        ctrl[R.act[i]] = (R.target[i] - q[R.jq[i]]) * kp - qv[R.jd[i]] * kd;
      }
      if (this.grasp && (this.grasp.anim || this.grasp.loose.length)) this._graspStep();
    }
    if (this.drag) this._applyDrag();
    this._updateFreeze();
    this.mj.mj_step(this.model, this.data);
    if (this.freeze.on && this.freeze.frozen.size) this._pinFrozen();
    this.simTime += this.dt;
    if (R) {
      R.counter++;
      if (R.counter % R.cfg.control_decimation === 0 && !R.fell) this._control();
    }
  }

  // advance by dtWall seconds of simulated time, within budgetMs of compute (the rest is dropped = slow motion)
  advance(dtWall, budgetMs = 12) {
    this._checkViews();
    this.simTarget += dtWall;
    const t0 = performance.now(); let n = 0; let lag = false;
    while (this.simTime + 0.5 * this.dt <= this.simTarget) {
      this._physStep(); n++;
      if ((n & 7) === 0 && performance.now() - t0 > budgetMs) { lag = true; break; }
    }
    const ms = performance.now() - t0;
    if (lag) this.simTarget = this.simTime;
    const s = this.stats; s.steps += n; s.simSec += n * this.dt; s.wallMs += ms;
    s.physMsPerStep = n ? 0.9 * s.physMsPerStep + 0.1 * (ms / n) : s.physMsPerStep;
    s.lagging = lag;
    return n;
  }

  // ---- mouse drag: a critically damped spring from a point of body b to a target (world, z up) ----
  startDrag(b, worldPoint) {
    this._checkViews();
    const v = this.v, R = v.xmat, p = v.xpos;
    const d = [worldPoint[0] - p[3 * b], worldPoint[1] - p[3 * b + 1], worldPoint[2] - p[3 * b + 2]];
    const r = R.subarray(9 * b, 9 * b + 9);
    const local = [r[0] * d[0] + r[3] * d[1] + r[6] * d[2], r[1] * d[0] + r[4] * d[1] + r[7] * d[2], r[2] * d[0] + r[5] * d[1] + r[8] * d[2]];
    const robot = !!(this.robotBodies && this.robotBodies.has(b));
    const mass = robot ? 8 : Math.max(0.2, this.v.body_subtreemass[b]);
    this.drag = { b, local, target: worldPoint.slice(), prev: worldPoint.slice(), mass, fmax: robot ? 120 : null };
  }
  moveDrag(target) { if (this.drag) this.drag.target = target.slice(); }
  endDrag() {
    if (!this.drag) return;
    const o = 6 * this.drag.b; for (let i = 0; i < 6; i++) this.v.xfrc[o + i] = 0;
    this.drag = null;
  }
  dragPoint() {
    if (!this.drag) return null;
    this._checkViews();
    const { b, local } = this.drag, r = this.v.xmat, p = this.v.xpos, o = 9 * b;
    return [p[3 * b] + r[o] * local[0] + r[o + 1] * local[1] + r[o + 2] * local[2],
      p[3 * b + 1] + r[o + 3] * local[0] + r[o + 4] * local[1] + r[o + 5] * local[2],
      p[3 * b + 2] + r[o + 6] * local[0] + r[o + 7] * local[1] + r[o + 8] * local[2]];
  }
  _applyDrag() {
    const D = this.drag, b = D.b, p = this.dragPoint();
    const w = 2 * Math.PI * 2.5, k = w * w, cdamp = 2 * w;
    const F = [0, 0, 0];
    for (let i = 0; i < 3; i++) {
      const vel = (p[i] - D.prev[i]) / this.dt;
      F[i] = D.mass * (k * (D.target[i] - p[i]) - cdamp * vel);
    }
    const fmax = D.fmax ?? D.mass * 40 + 150, fn = Math.hypot(F[0], F[1], F[2]);
    if (fn > fmax) for (let i = 0; i < 3; i++) F[i] *= fmax / fn;
    D.prev = p;
    const c = this.v.xipos, r = [p[0] - c[3 * b], p[1] - c[3 * b + 1], p[2] - c[3 * b + 2]];
    const xf = this.v.xfrc, o = 6 * b;
    xf[o] = F[0]; xf[o + 1] = F[1]; xf[o + 2] = F[2];
    xf[o + 3] = r[1] * F[2] - r[2] * F[1]; xf[o + 4] = r[2] * F[0] - r[0] * F[2]; xf[o + 5] = r[0] * F[1] - r[1] * F[0];
  }

  // joint value of a part body (hinge angle / slide distance), null for free bodies
  jointValue(b) {
    this._checkViews();
    if (this.v.body_jntnum[b] < 1) return null;
    const j = this.v.body_jntadr[b], t = this.v.jnt_type[j];
    if (t !== 2 && t !== 3) return null;   // 2 slide, 3 hinge
    return this.v.qpos[this.v.jnt_qposadr[j]];
  }

  dispose() { this.data.delete(); this.model.delete(); }
}

// quaternions [w, x, y, z]
function qmul(a, b) {
  return [a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3], a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1], a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]];
}
function qconj(a) { return [a[0], -a[1], -a[2], -a[3]]; }
function qrot(q, v) { const r = qmul(qmul(q, [0, v[0], v[1], v[2]]), qconj(q)); return [r[1], r[2], r[3]]; }
function qslerp(a, b, t) {
  let d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  if (d < 0) { b = b.map((x) => -x); d = -d; }
  if (d > 0.9995) { const r = a.map((x, i) => x + (b[i] - x) * t); const n = Math.hypot(...r); return r.map((x) => x / n); }
  const th = Math.acos(d), s0 = Math.sin((1 - t) * th) / Math.sin(th), s1 = Math.sin(t * th) / Math.sin(th);
  return a.map((x, i) => s0 * x + s1 * b[i]);
}
