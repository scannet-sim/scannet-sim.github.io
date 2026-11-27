// Text navigation: landmark / object matching, a 2D occupancy grid rasterised from the CURRENT poses of every collision
// geom (static, free and jointed bodies) at leg-to-torso height, a clearance (distance) map, Dijkstra with a clearance
// cost, and a path follower that turns the path into velocity commands for the locomotion policy (slows down near
// obstacles and at the goal, replans when it stalls or leaves the path, ends facing the target).

const STOP = new Set(['go', 'to', 'the', 'a', 'an', 'walk', 'move', 'please', 'near', 'toward', 'towards', 'next', 'robot',
  'g1', 'and', 'of', 'at', 'in', 'on', 'my', 'over', 'there', 'head', 'navigate', 'find', 'reach', 'approach', 'by', 'up', 'can',
  'you', 'pick', 'grab', 'take', 'get', 'fetch', 'bring', 'carry', 'put', 'place', 'it', 'down', 'onto', 'into', 'drop', 'lift', 'set', 'that', 'this', 'one']);
const toks = (s) => (s || '').toLowerCase().replace(/[_\-]/g, ' ').replace(/[^a-z0-9 ]/g, ' ').split(/\s+/)
  .filter((t) => t && !/^\d+$/.test(t) && !/^inst\d*$/.test(t));
const stem = (t) => t.replace(/(es|s)$/, '');

// best label match of `text` among items [{label, name, ...}] (null if nothing matches)
export function matchLabel(text, items) {
  const q = toks(text).filter((t) => !STOP.has(t)).map(stem);
  if (!q.length) return null;
  let best = null, bs = 0;
  for (const L of items || []) {
    const lt = [...toks(L.label), ...toks(L.name)].map(stem);
    let s = 0;
    for (const t of q) {
      if (lt.includes(t)) s += 3;
      else if (t.length >= 3 && lt.some((u) => u.startsWith(t) || t.startsWith(u) || (u.length >= 4 && t.includes(u)))) s += 1;
    }
    s -= 0.01 * lt.length;
    if (s > bs) { bs = s; best = L; }
  }
  return bs >= 1 ? best : null;
}
export const matchLandmark = matchLabel;

export class Navigator {
  constructor(world, opts = {}) {
    this.w = world; this.res = opts.res || 0.05;
    this.hard = opts.hard ?? 0.24;      // cells closer than this to an obstacle are blocked (robot half width ~0.2 m)
    this.soft = opts.soft ?? 0.6;       // closer than this cost extra: paths keep clear of furniture
    this.active = false; this.status = 'idle'; this.path = null; this.goal = null; this.onStatus = null;
    world.nav = this;
  }

  // occupancy at the current poses; skips the robot and what it carries
  buildGrid() {
    const w = this.w, m = w.model, fz = w.meta.floor_z || 0;
    w._checkViews();
    const B = w.meta.bounds || [[-6, -6, 0], [6, 6, 3]];
    const x0 = B[0][0] - 0.5, y0 = B[0][1] - 0.5, x1 = B[1][0] + 0.5, y1 = B[1][1] + 0.5;
    const r = this.res, nx = Math.ceil((x1 - x0) / r), ny = Math.ceil((y1 - y0) / r);
    const occ = new Uint8Array(nx * ny);
    const gb = m.geom_bodyid, ct = m.geom_contype, ca = m.geom_conaffinity, gt = m.geom_type;
    const held = w.holding ? w.holding() : null;
    const zlo = fz + 0.06, zhi = fz + 1.4;
    for (let g = 0; g < m.ngeom; g++) {
      const b = gb[g];
      if ((w.robotBodies && w.robotBodies.has(b)) || b === held) continue;
      if (ct[g] === 0 && ca[g] === 0) continue;
      if (gt[g] === 0) continue;                                     // the floor plane
      if (gt[g] === 1) { this._hfield(g, occ, nx, ny, x0, y0, fz); continue; }
      const pts = w.geomPoints(g);
      let zmin = 1e9, zmax = -1e9;
      for (const q of pts) { zmin = Math.min(zmin, q[2]); zmax = Math.max(zmax, q[2]); }
      if (zmax < zlo || zmin > zhi) continue;
      fillHull(convexHull(pts.map((q) => [q[0], q[1]])), occ, nx, ny, x0, y0, r);
    }
    this.grid = { occ, nx, ny, x0, y0, r };
    this.dist = distanceField(occ, nx, ny, r);
    return this.grid;
  }

  _hfield(g, occ, nx, ny, x0, y0, fz) {
    const m = this.w.model, d = this.w.data;
    const h = m.geom_dataid[g], nr = m.hfield_nrow[h], nc = m.hfield_ncol[h];
    const sz = m.hfield_size.subarray(4 * h, 4 * h + 4), adr = m.hfield_adr[h], data = m.hfield_data;
    const p = d.geom_xpos.subarray(3 * g, 3 * g + 3), r = this.res;
    for (let iy = 0; iy < ny; iy++) for (let ix = 0; ix < nx; ix++) {
      const X = x0 + (ix + 0.5) * r - p[0], Y = y0 + (iy + 0.5) * r - p[1];
      if (Math.abs(X) > sz[0] || Math.abs(Y) > sz[1]) continue;
      const c = Math.min(nc - 1, Math.max(0, Math.round((X / sz[0] + 1) / 2 * (nc - 1))));
      const rr = Math.min(nr - 1, Math.max(0, Math.round((Y / sz[1] + 1) / 2 * (nr - 1))));
      const z = p[2] + data[adr + rr * nc + c] * sz[2];
      if (z > fz + 0.08 || z < fz - 0.06) occ[iy * nx + ix] = 1;
    }
  }

  cell(x, y) { const G = this.grid; return [Math.floor((x - G.x0) / G.r), Math.floor((y - G.y0) / G.r)]; }
  inside(ix, iy) { const G = this.grid; return ix >= 0 && iy >= 0 && ix < G.nx && iy < G.ny; }
  clearance(x, y) { const [ix, iy] = this.cell(x, y); return this.inside(ix, iy) ? this.dist[iy * this.grid.nx + ix] : 0; }
  free(ix, iy) { return this.inside(ix, iy) && this.dist[iy * this.grid.nx + ix] >= this.hard; }

  // Dijkstra from the robot over the free cells (cost = length x (1 + clearance penalty)); goal = the reached cell
  // closest to (gx, gy) (so goals inside furniture or its margin still get the nearest reachable spot)
  plan(sx, sy, gx, gy, maxOff = 1.2) {
    const G = this.grid, nx = G.nx, N = G.nx * G.ny, D = this.dist;
    let s = this.cell(sx, sy);
    if (!this.free(...s)) s = this._escape(s);
    if (!s) return null;
    const gs = new Float32Array(N).fill(Infinity), from = new Int32Array(N).fill(-1), closed = new Uint8Array(N);
    const pen = (j) => { const c = D[j]; return c >= this.soft ? 1 : 1 + 6 * ((this.soft - c) / (this.soft - this.hard)) ** 2; };
    const heap = new MinHeap(), si = s[1] * nx + s[0];
    gs[si] = 0; heap.push(si, 0);
    const nb = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1], [1, 1, 1.4142], [1, -1, 1.4142], [-1, 1, 1.4142], [-1, -1, 1.4142]];
    const [tx, ty] = this.cell(gx, gy);
    let best = -1, bd = Infinity;
    while (heap.size) {
      const i = heap.pop();
      if (closed[i]) continue; closed[i] = 1;
      const ix = i % nx, iy = (i / nx) | 0;
      const dg = Math.hypot(ix - tx, iy - ty);
      if (dg < bd - 1e-6) { bd = dg; best = i; }
      if (dg === 0) break;
      for (const [dx, dy, c] of nb) {
        const jx = ix + dx, jy = iy + dy;
        if (!this.free(jx, jy)) continue;
        if (dx && dy && (!this.free(ix + dx, iy) || !this.free(ix, iy + dy))) continue;
        const j = jy * nx + jx, ng = gs[i] + c * pen(j);
        if (ng < gs[j]) { gs[j] = ng; from[j] = i; heap.push(j, ng); }
      }
    }
    if (best < 0 || bd * G.r > maxOff) return null;
    const cells = [];
    for (let i = best; i !== -1; i = from[i]) cells.push([i % nx, (i / nx) | 0]);
    cells.reverse();
    // shortcut segments only where the straight line keeps a comfortable clearance
    const keep = Math.min(this.soft * 0.6, 0.36);
    const out = [cells[0]];
    let k = 0;
    while (k < cells.length - 1) {
      let j = Math.min(cells.length - 1, k + 40);
      while (j > k + 1 && !this.los(cells[k], cells[j], keep)) j--;
      out.push(cells[j]); k = j;
    }
    return out.map(([ix, iy]) => [G.x0 + (ix + 0.5) * G.r, G.y0 + (iy + 0.5) * G.r]);
  }
  // from a blocked cell to the nearest free one (the robot standing close to furniture)
  _escape(s) {
    for (let rad = 1; rad < 40; rad++) {
      let best = null, bd = 1e9;
      for (let dy = -rad; dy <= rad; dy++) for (let dx = -rad; dx <= rad; dx++) {
        if (Math.max(Math.abs(dx), Math.abs(dy)) !== rad || !this.free(s[0] + dx, s[1] + dy)) continue;
        const dd = dx * dx + dy * dy; if (dd < bd) { bd = dd; best = [s[0] + dx, s[1] + dy]; }
      }
      if (best) return best;
    }
    return null;
  }
  los(a, b, keep) {
    const n = Math.max(Math.abs(b[0] - a[0]), Math.abs(b[1] - a[1])) * 2, nx = this.grid.nx;
    for (let i = 0; i <= n; i++) {
      const t = i / Math.max(1, n), x = Math.round(a[0] + (b[0] - a[0]) * t), y = Math.round(a[1] + (b[1] - a[1]) * t);
      if (!this.inside(x, y)) return false;
      const c = this.dist[y * nx + x];
      // near the two ends the robot may already be close to things; in between it keeps `keep`
      const end = Math.min(i, n - i) / 2 * this.grid.r < 0.3;
      if (c < (end ? this.hard : keep)) return false;
    }
    return true;
  }

  // walk to (x, y); then face `face` ([x, y]) or turn to `yaw`; label for the status line
  goTo(x, y, opts = {}) {
    const P = this.w.pelvisPose();
    if (!P) return false;
    this.buildGrid();
    const path = this.plan(P.x, P.y, x, y, opts.maxOff ?? 1.2);
    this.goal = { x, y, yaw: opts.yaw ?? null, face: opts.face || null, label: opts.label || '', tol: opts.tol ?? 0.15, maxOff: opts.maxOff ?? 1.2 };
    if (!path) { this.active = false; this._set('no path'); return false; }
    this.path = path; this.idx = 1; this.active = true; this.replans = 0; this.turnT = null;
    this.prog = { t: this.w.simTime, x: P.x, y: P.y, best: Infinity };
    this.lastCheck = this.w.simTime;
    this._set('walking');
    return true;
  }
  replan() {
    const P = this.w.pelvisPose(), g = this.goal;
    this.buildGrid();
    const path = this.plan(P.x, P.y, g.x, g.y, g.maxOff);
    if (!path) return false;
    this.path = path; this.idx = 1; this.replans++;
    this.prog = { t: this.w.simTime, x: P.x, y: P.y, best: Infinity };
    return true;
  }
  cancel(why = 'idle') { if (this.active || why !== 'idle') { this.active = false; this._set(why); } }
  _set(s) { this.status = s; this.onStatus?.(s, this.goal); }

  command(P) {
    if (!this.active || !P) return [0, 0, 0];
    const path = this.path, last = path.length - 1;
    const dist = (q) => Math.hypot(q[0] - P.x, q[1] - P.y);
    const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
    const clamp = (v, a) => Math.max(-a, Math.min(a, v));
    while (this.idx < last && dist(path[this.idx]) < 0.4) this.idx++;
    const dg = dist(path[last]);
    // arrived at the end of the path: turn to face the target
    if (this.status === 'turning' || (this.idx >= last && dg < this.goal.tol)) {
      const g = this.goal;
      const yaw = g.face ? Math.atan2(g.face[1] - P.y, g.face[0] - P.x) : g.yaw;
      if (yaw == null) { this.active = false; this._set('arrived'); return [0, 0, 0]; }
      if (this.status !== 'turning') { this._set('turning'); this.turnT = this.w.simTime; }
      const e = wrap(yaw - P.yaw);
      if (Math.abs(e) < 0.1 || this.w.simTime - this.turnT > 5) { this.active = false; this._set('arrived'); return [0, 0, 0]; }
      return [0, 0, clamp(1.4 * e, 0.6)];
    }
    // progress / deviation checks: replan from the current poses when stalled or off the path
    const t = this.w.simTime;
    this.prog.best = Math.min(this.prog.best, dg);
    if (t - this.lastCheck > 1.0) {
      this.lastCheck = t;
      const seg = this._offPath(P);
      const stalled = t - this.prog.t > 3 && Math.hypot(P.x - this.prog.x, P.y - this.prog.y) < 0.15;
      if (stalled && dg < 0.7) {   // blocked close to the goal (e.g. a carried object touching the counter): good enough
        this.idx = last; this.goal.tol = Math.max(this.goal.tol, dg + 0.01); this._set('turning'); this.turnT = t; return [0, 0, 0];
      }
      if (stalled || seg > 0.45) {
        if (this.replans >= 5 || !this.replan()) { this.active = false; this._set('stuck'); return [0, 0, 0]; }
        return [0, 0, 0];
      }
      if (t - this.prog.t > 3) this.prog = { t, x: P.x, y: P.y, best: this.prog.best };
    }
    // pure pursuit on a look-ahead point; slow near obstacles and near the goal
    let tgt = path[this.idx];
    const la = 0.55;
    for (let k = this.idx; k <= last; k++) { tgt = path[k]; if (dist(path[k]) > la) break; }
    const e = wrap(Math.atan2(tgt[1] - P.y, tgt[0] - P.x) - P.yaw);
    if (Math.abs(e) > 0.7) return [0, 0, clamp(1.5 * e, 0.8)];
    const clr = this.clearance(P.x, P.y);
    const vmax = Math.min(0.7, 0.25 + 0.9 * Math.max(0, clr - this.hard), 0.15 + 0.6 * dg);
    const vx = Math.max(0.12, vmax) * Math.max(0, Math.cos(e));
    return [vx, 0, clamp(1.6 * e, 0.8)];
  }
  _offPath(P) {   // distance from the robot to the current path segment
    const p = this.path, i = Math.max(1, this.idx), a = p[i - 1], b = p[i];
    const dx = b[0] - a[0], dy = b[1] - a[1], L2 = dx * dx + dy * dy || 1;
    const t = Math.max(0, Math.min(1, ((P.x - a[0]) * dx + (P.y - a[1]) * dy) / L2));
    return Math.hypot(P.x - a[0] - t * dx, P.y - a[1] - t * dy);
  }
}

class MinHeap {
  constructor() { this.k = []; this.p = []; }
  get size() { return this.k.length; }
  push(k, p) {
    const K = this.k, P = this.p; let i = K.length; K.push(k); P.push(p);
    while (i > 0) { const j = (i - 1) >> 1; if (P[j] <= P[i]) break; [K[i], K[j]] = [K[j], K[i]]; [P[i], P[j]] = [P[j], P[i]]; i = j; }
  }
  pop() {
    const K = this.k, P = this.p, top = K[0], lk = K.pop(), lp = P.pop();
    if (K.length) {
      K[0] = lk; P[0] = lp; let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1; let m = i;
        if (l < K.length && P[l] < P[m]) m = l;
        if (r < K.length && P[r] < P[m]) m = r;
        if (m === i) break;
        [K[i], K[m]] = [K[m], K[i]]; [P[i], P[m]] = [P[m], P[i]]; i = m;
      }
    }
    return top;
  }
}

// distance (m) from every cell to the nearest occupied cell: two-pass chamfer (1, sqrt 2)
function distanceField(occ, nx, ny, r) {
  const D = new Float32Array(nx * ny);
  const INF = 1e9;
  for (let i = 0; i < D.length; i++) D[i] = occ[i] ? 0 : INF;
  const s2 = Math.SQRT2;
  for (let y = 0; y < ny; y++) for (let x = 0; x < nx; x++) {
    const i = y * nx + x; let d = D[i];
    if (x > 0) d = Math.min(d, D[i - 1] + 1);
    if (y > 0) { d = Math.min(d, D[i - nx] + 1); if (x > 0) d = Math.min(d, D[i - nx - 1] + s2); if (x < nx - 1) d = Math.min(d, D[i - nx + 1] + s2); }
    D[i] = d;
  }
  for (let y = ny - 1; y >= 0; y--) for (let x = nx - 1; x >= 0; x--) {
    const i = y * nx + x; let d = D[i];
    if (x < nx - 1) d = Math.min(d, D[i + 1] + 1);
    if (y < ny - 1) { d = Math.min(d, D[i + nx] + 1); if (x < nx - 1) d = Math.min(d, D[i + nx + 1] + s2); if (x > 0) d = Math.min(d, D[i + nx - 1] + s2); }
    D[i] = d;
  }
  for (let i = 0; i < D.length; i++) D[i] = Math.min(D[i], 1e4) * r;
  return D;
}

function convexHull(pts) {
  const p = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  for (const q of p) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], q) <= 0) lo.pop(); lo.push(q); }
  for (let i = p.length - 1; i >= 0; i--) { const q = p[i]; while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], q) <= 0) up.pop(); up.push(q); }
  return lo.slice(0, -1).concat(up.slice(0, -1));
}

function fillHull(h, occ, nx, ny, x0, y0, r) {
  if (h.length < 3) { for (const q of h) { const ix = Math.floor((q[0] - x0) / r), iy = Math.floor((q[1] - y0) / r); if (ix >= 0 && iy >= 0 && ix < nx && iy < ny) occ[iy * nx + ix] = 1; } return; }
  let xa = 1e9, xb = -1e9, ya = 1e9, yb = -1e9;
  for (const q of h) { xa = Math.min(xa, q[0]); xb = Math.max(xb, q[0]); ya = Math.min(ya, q[1]); yb = Math.max(yb, q[1]); }
  const i0 = Math.max(0, Math.floor((xa - x0) / r)), i1 = Math.min(nx - 1, Math.floor((xb - x0) / r));
  const j0 = Math.max(0, Math.floor((ya - y0) / r)), j1 = Math.min(ny - 1, Math.floor((yb - y0) / r));
  const n = h.length;
  for (let j = j0; j <= j1; j++) for (let i = i0; i <= i1; i++) {
    const X = x0 + (i + 0.5) * r, Y = y0 + (j + 0.5) * r;
    let inside = true;
    for (let k = 0; k < n && inside; k++) {
      const a = h[k], b = h[(k + 1) % n];
      const cr = (b[0] - a[0]) * (Y - a[1]) - (b[1] - a[1]) * (X - a[0]);
      const len = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1;
      if (cr / len < -0.5 * r) inside = false;     // half a cell of tolerance: thin walls still mark their cells
    }
    if (inside) occ[j * nx + i] = 1;
  }
}
