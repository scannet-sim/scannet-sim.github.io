// Interactive demo: ?scene=<id>[&twin=<id>][&robot=g1|none][&cam=follow|free][&sleep=1][&start=click][&ui=min]
import loadMujoco from '../../vendor/mujoco/mujoco.js';
import { Progress, loadScenePackage, loadRobotPackage, SimWorld } from './world.js';
import { WorldView } from './view.js';
import { Navigator, matchLabel } from './nav.js';

const Q = new URLSearchParams(location.search);
const SCENES = new URL('../assets/scenes/', import.meta.url.replace(/js\/app\.js.*$/, '')).href;
const ROBOT = new URL('../assets/g1/', import.meta.url.replace(/js\/app\.js.*$/, '')).href;
const sceneId = Q.get('scene') || '_test';
const twinId = Q.get('twin');
const withRobot = (Q.get('robot') || 'g1') !== 'none';
const $ = (s) => document.querySelector(s);
const ui = { loading: $('#loading'), bar: $('#loading .bar i'), msg: $('#loading .msg'), panes: $('#panes'), tip: $('#tip'),
  stats: $('#stats'), input: $('#cmd'), focus: $('#focus') };
if (Q.get('ui') === 'min') document.body.classList.add('min');
if (!withRobot) document.body.classList.add('norobot');

const state = { worlds: [], views: [], keys: new Set(), speed: 1, running: false, frames: 0, simAcc: 0, wallAcc: 0, fps: 0, rtf: 0 };
window.simdemo = { state, ready: null, setKeys: (ks) => { state.keys = new Set(ks); }, command: (t) => runCommand(t), grasp: () => graspToggle() };

function fmtMB(n) { return (n / 1048576).toFixed(1) + ' MB'; }

async function boot() {
  const prog = new Progress((p) => {
    const f = p.total ? p.done / p.total : 0;
    ui.bar.style.width = (100 * Math.min(1, f)).toFixed(1) + '%';
    ui.msg.textContent = `Loading physics engine, robot and scene… ${fmtMB(p.done)}${p.total ? ' / ' + fmtMB(p.total) : ''}`;
  });
  const ids = twinId ? [twinId, sceneId] : [sceneId];
  const mjP = loadMujoco({ locateFile: (f) => new URL('../vendor/mujoco/' + f, import.meta.url.replace(/js\/app\.js.*$/, '')).href });
  const robotP = withRobot ? loadRobotPackage(ROBOT, prog) : Promise.resolve(null);
  const scenesP = Promise.all(ids.map((id) => loadScenePackage(SCENES + id + '/', prog)));
  const [mj, robot, scenes] = await Promise.all([mjP, robotP, scenesP]);
  ui.msg.textContent = 'Compiling physics…';
  await new Promise((r) => setTimeout(r, 20));
  // a twin without landmarks borrows those of the main scene (same frame)
  const main = scenes[scenes.length - 1];
  for (const s of scenes) if (!(s.meta.landmarks || []).length && main.meta.landmarks) s.meta.landmarks = main.meta.landmarks;
  for (const s of scenes) {   // fallbacks for packages without spawn / camera
    const M = s.meta, L = (M.landmarks || []).find((l) => l.approach);
    if (!M.spawn) M.spawn = L ? { pos: [L.approach[0], L.approach[1]], yaw: L.approach[2] || 0 } : { pos: [0, 0], yaw: 0 };
    if (!M.camera && M.bounds) {
      const [a, b] = M.bounds, c = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
      M.camera = { pos: [c[0] - 0.55 * (b[0] - a[0]), c[1] - 0.55 * (b[1] - a[1]), b[2] + 1.2], target: [c[0], c[1], 0.6] };
    }
  }
  for (const s of scenes) {
    const w = new SimWorld(mj, s, robot, { sleep: Q.get('sleep') === '1', freeze: Q.has('freeze') ? Q.get('freeze') === '1' : undefined });
    if (withRobot) {
      const nav = new Navigator(w);
      nav.onStatus = (st, g) => setBadge(w, st, g);
    }
    w.onEvent = (e, b) => {
      if (e === 'fell') { setBadge(w, 'fell'); w.task = null; }
      if (e === 'reset') { setBadge(w, 'ready'); w.task = null; }
      if (e === 'pick') setBadge(w, `Picking up ${objLabel(w, b)}…`, null, 'go');
      if (e === 'held') setBadge(w, `Holding ${objLabel(w, b)} · assisted grasp`, null, 'ok');
      if (e === 'release') setBadge(w, `Put down ${objLabel(w, b)}`, null, 'ok');
    };
    state.worlds.push(w);
  }
  ui.panes.classList.toggle('twin', state.worlds.length > 1);
  for (const w of state.worlds) {
    const pane = document.createElement('div'); pane.className = 'pane';
    pane.innerHTML = `<canvas></canvas><div class="tag"></div><div class="badge"></div>`;
    pane.querySelector('.tag').textContent = w.meta.title || w.meta.id;
    pane.classList.add(w.meta.kind === 'scan' ? 'scan' : 'ours');
    ui.panes.appendChild(pane);
    const view = new WorldView(pane.querySelector('canvas'), w, {
      camMode: Q.get('cam') || 'follow',
      onHover: (info, ev) => showTip(info, ev),
      onDrag: () => hideTip(),
    });
    w.pane = pane; w.view = view;
    await view.init();
    state.views.push(view);
    setBadge(w, 'ready');
  }
  ui.loading.classList.add('done');
  makeChips();
  state.running = true;
  requestAnimationFrame(frame);
  return true;
}

function setBadge(w, st, g, cls0) {
  const b = w.pane?.querySelector('.badge'); if (!b) return;
  if (cls0) { b.textContent = st; b.className = 'badge ' + cls0; return; }
  const lab = g?.label ? ` · ${g.label}` : '';
  const T = { ready: ['', ''], idle: ['', ''], walking: ['Walking' + lab, 'go'], turning: ['Turning' + lab, 'go'], arrived: ['Arrived' + lab, 'ok'],
    stuck: ['Stuck' + lab, 'bad'], 'no path': ['No path' + lab, 'bad'], fell: ['Fell over · press R', 'bad'] };
  const [txt, cls] = T[st] || [st, ''];
  b.textContent = txt; b.className = 'badge ' + cls;
}

function showTip(info, ev) {
  if (!info || !ev) return hideTip();
  ui.tip.textContent = info.label; ui.tip.style.display = 'block';
  ui.tip.style.left = ev.clientX + 14 + 'px'; ui.tip.style.top = ev.clientY + 12 + 'px';
}
function hideTip() { ui.tip.style.display = 'none'; }

// ---------- keyboard / touch ----------
const KEYMAP = { KeyW: 'f', ArrowUp: 'f', KeyS: 'b', ArrowDown: 'b', KeyA: 'l', ArrowLeft: 'l', KeyD: 'r', ArrowRight: 'r', KeyQ: 'sl', KeyE: 'sr' };
window.addEventListener('keydown', (e) => {
  if (e.target === ui.input) { if (e.key === 'Enter') { runCommand(ui.input.value); ui.input.blur(); } return; }
  const k = KEYMAP[e.code];
  if (k) { state.keys.add(k); e.preventDefault(); for (const w of state.worlds) { w.task = null; w.nav?.cancel(); } }
  if (e.shiftKey) state.keys.add('fast');
  if (e.code === 'KeyR') { for (const w of state.worlds) w.resetRobot(); }
  if (e.code === 'KeyC') toggleCam();
  if (e.code === 'KeyG') graspToggle();
  if (e.code === 'Space') { e.preventDefault(); state.keys.clear(); for (const w of state.worlds) { w.nav?.cancel(); w.setCommand(0, 0, 0); } }
  if (e.code === 'Enter') { ui.input.focus(); e.preventDefault(); }
});
window.addEventListener('keyup', (e) => { const k = KEYMAP[e.code]; if (k) state.keys.delete(k); if (!e.shiftKey) state.keys.delete('fast'); });
window.addEventListener('blur', () => state.keys.clear());
for (const btn of document.querySelectorAll('[data-key]')) {
  const k = btn.dataset.key;
  const on = (e) => { e.preventDefault(); state.keys.add(k); for (const w of state.worlds) { w.task = null; w.nav?.cancel(); } btn.classList.add('on'); };
  const off = (e) => { e.preventDefault(); state.keys.delete(k); btn.classList.remove('on'); };
  btn.addEventListener('pointerdown', on); btn.addEventListener('pointerup', off); btn.addEventListener('pointerleave', off); btn.addEventListener('pointercancel', off);
}
$('#go')?.addEventListener('click', () => runCommand(ui.input.value));
$('#reset')?.addEventListener('click', () => { for (const w of state.worlds) w.resetRobot(); });
$('#resetall')?.addEventListener('click', () => { for (const w of state.worlds) w.resetScene(); });
$('#cam')?.addEventListener('click', toggleCam);
$('#grab')?.addEventListener('click', graspToggle);
for (const chip of document.querySelectorAll('.chips')) chip.addEventListener('click', (e) => { const t = e.target.closest('button'); if (t) { ui.input.value = t.textContent; runCommand(t.textContent); } });

function toggleCam() { for (const v of state.views) if (v.w.robot) v.setMode(v.mode === 'follow' ? 'free' : 'follow'); $('#cam') && ($('#cam').textContent = state.views[0]?.mode === 'follow' ? 'Free camera (C)' : 'Follow robot (C)'); }

function keyCommand() {
  const k = state.keys, fast = k.has('fast');
  const vx = (k.has('f') ? (fast ? 1.0 : 0.7) : 0) + (k.has('b') ? -0.45 : 0);
  const wz = (k.has('l') ? 0.9 : 0) + (k.has('r') ? -0.9 : 0);
  const vy = (k.has('sl') ? 0.4 : 0) + (k.has('sr') ? -0.4 : 0);
  return [vx, vy, wz];
}

// ---------- objects, assisted grasp, text tasks ----------
function objLabel(w, b) {
  const n = w.bodyName(b), L = (w.meta.bodies || []).find((x) => x.name === n);
  return (L && L.label) || n.replace(/_inst\d+$/, '').replace(/_/g, ' ');
}
function smallObjects(w) {
  return [...(w.grasp?.small.keys() || [])].map((b) => ({ b, name: w.bodyName(b), label: objLabel(w, b) }));
}
function nearestOf(w, items) {
  const P = w.pelvisPose(), xi = w.v.xipos;
  let best = null, bd = 1e9;
  for (const it of items) { const d = Math.hypot(xi[3 * it.b] - P.x, xi[3 * it.b + 1] - P.y); if (d < bd) { bd = d; best = it; } }
  return best;
}
function graspToggle() {
  for (const w of state.worlds) {
    if (!w.grasp || w.robot.fell) continue;
    w.task = null; w.nav?.cancel();
    if (w.holding() != null) { w.release(w.findPlacement(null)); continue; }
    const b = w.reachable();
    if (b == null) setBadge(w, 'Nothing small (≤ 2 kg, ≤ 40 cm) within reach', null, 'bad');
    else w.pick(b);
  }
}
// steps: {go: [x, y], face, label, maxOff} | {pick: b} | {put: landmark name or null}
function startTask(w, steps) { w.task = { steps, i: 0, started: false }; }
function runTasks() { for (const w of state.worlds) stepTask(w); }
function stepTask(w) {
  {
    const T = w.task; if (!T) return;
    const s = T.steps[T.i];
    if (!s) { w.task = null; return; }
    const fail = (msg) => { setBadge(w, msg, null, 'bad'); w.task = null; };
    if (!T.started) {
      T.started = true; T.t0 = w.simTime;
      if (s.go && !w.nav.goTo(s.go[0], s.go[1], { face: s.face, label: s.label, maxOff: s.maxOff ?? 1.2 })) return fail(`No path · ${s.label}`);
      if (s.pick != null) {
        const b = w.reachable(0.95) === s.pick || reachOk(w, s.pick) ? s.pick : null;
        if (b == null || !w.pick(b)) return fail(`Out of reach · ${objLabel(w, s.pick)}`);
      }
      if (s.put !== undefined) { if (w.holding() == null) return fail('Not holding anything'); w.release(w.findPlacement(s.put)); }
      return;
    }
    if (s.go) {
      if (w.nav.status === 'arrived') { T.i++; T.started = false; }
      else if (!w.nav.active) w.task = null;           // stuck / no path / cancelled: the badge says why
    } else if (s.pick != null) {
      if (w.grasp.held === s.pick) { T.i++; T.started = false; } else if (!w.grasp.anim) fail('Could not pick it up');
    } else if (!w.grasp.anim) { T.i++; T.started = false; }
  }
}
function reachOk(w, b) {
  const P = w.pelvisPose(), xi = w.v.xipos;
  return Math.hypot(xi[3 * b] - P.x, xi[3 * b + 1] - P.y) < 0.95 && xi[3 * b + 2] - (w.meta.floor_z || 0) < 1.45;
}
function landmarkStep(w, L) {
  const a = L.approach || [L.pos[0], L.pos[1]];
  return { go: [a[0], a[1]], face: L.pos, label: L.label || L.name };
}
// to put something on a landmark: walk to the reachable spot closest to its centre (the edge of the table / counter)
function surfaceStep(w, L) { return { go: [L.pos[0], L.pos[1]], face: L.pos, label: L.label || L.name, maxOff: 3.0 }; }

function runCommand(text) {
  text = (text || '').trim();
  if (!text || !withRobot) return;
  const t = text.toLowerCase();
  if (/^(stop|halt|wait)\b/.test(t)) { for (const w of state.worlds) { w.task = null; w.nav?.cancel(); } return; }
  if (/^(reset|restart)\b/.test(t)) { for (const w of state.worlds) w.resetRobot(); return; }
  for (const w of state.worlds) {
    if (w.robot.fell) w.resetRobot();
    w.task = null; w.nav.cancel();
    if (/\b(back|home|start)\b/.test(t) && !/\b(bring|put|place)\b/.test(t)) {
      const s = w.meta.spawn; startTask(w, [{ go: [s.pos[0], s.pos[1]], face: [s.pos[0] + Math.cos(s.yaw), s.pos[1] + Math.sin(s.yaw)], label: 'start' }]); continue;
    }
    // "put it on the dining table" / "put it down"
    if (/\b(put|place|drop|set)\b/.test(t) && w.holding() != null) {
      const dest = t.split(/\b(?:on|onto|in|into|at|to)\b/).slice(1).join(' ');
      const L = dest ? matchLabel(dest, w.meta.landmarks) : null;
      startTask(w, L ? [surfaceStep(w, L), { put: L.name }] : [{ put: null }]);
      continue;
    }
    // "pick up the book" / "bring the backpack to the sofa"
    if (/\b(pick|grab|take|get|fetch|lift|bring|carry)\b/.test(t)) {
      const [objText, destText] = t.split(/\bto\b|\bonto\b|\bon\b/);
      const items = smallObjects(w), m = matchLabel(objText, items);
      if (!m) { setBadge(w, `No small object (≤ 2 kg, ≤ 40 cm) called "${objText.replace(/^(pick up|pick|grab|take|get|fetch|lift|bring|carry)\s*/, '').replace(/^(up\s+)?(the|a|an)\s+/, '').trim()}"`, null, 'bad'); continue; }
      const it = nearestOf(w, items.filter((x) => x.label === m.label)) || m;
      const xi = w.v.xipos, ob = [xi[3 * it.b], xi[3 * it.b + 1]];
      const steps = [];
      if (w.holding() != null) steps.push({ put: null });
      steps.push({ go: ob, face: ob, label: it.label, maxOff: 0.85 }, { pick: it.b });
      const L = destText ? matchLabel(destText, w.meta.landmarks) : null;
      if (L) steps.push(surfaceStep(w, L), { put: L.name });
      startTask(w, steps);
      continue;
    }
    const L = matchLabel(text, w.meta.landmarks);
    if (L) { startTask(w, [landmarkStep(w, L)]); continue; }
    const o = matchLabel(text, smallObjects(w));     // "go to the backpack"
    if (o) { const it = nearestOf(w, smallObjects(w).filter((x) => x.label === o.label)) || o; const ob = [w.v.xipos[3 * it.b], w.v.xipos[3 * it.b + 1]]; startTask(w, [{ go: ob, face: ob, label: it.label, maxOff: 0.9 }]); continue; }
    setBadge(w, `Don't know "${text}"`, null, 'bad');
  }
}

// suggestion chips: two landmarks, the nearest small object, and a surface to put it on
function makeChips() {
  const box = document.getElementById('chips'), w = state.worlds[state.worlds.length - 1];
  if (!box || !w || !withRobot) return;
  box.innerHTML = '';
  const add = (txt) => { const b = document.createElement('button'); b.textContent = txt; box.appendChild(b); };
  const lms = w.meta.landmarks || [];
  for (const L of lms.slice(0, 2)) add(`go to the ${L.label || L.name}`);
  const o = nearestOf(w, smallObjects(w).filter((x) => w.v.xipos[3 * x.b + 2] - (w.meta.floor_z || 0) > 0.3));
  const surf = lms.find((L) => /table|counter|desk|shelf|cabinet/.test(L.label || ''));
  if (o) add(surf ? `bring the ${o.label} to the ${surf.label}` : `pick up the ${o.label}`);
}

// ---------- main loop ----------
let last = performance.now(), statT = performance.now();
function frame(now) {
  const wall = Math.max(0, (now - last) / 1000); last = now;
  const dt = Math.min(0.1, wall);
  const cmd = keyCommand();
  runTasks();
  const budget = (wall > 0.05 ? 30 : 14) / Math.max(1, state.worlds.length);
  let sim = 0;
  for (const w of state.worlds) {
    if (w.robot && !(w.nav && w.nav.active)) w.setCommand(...cmd);
    const t0 = w.simTime; w.advance(dt * state.speed, budget); sim += w.simTime - t0;
  }
  for (const v of state.views) { v.sync(); v.render(); }
  state.frames++; state.simAcc += sim / Math.max(1, state.worlds.length); state.wallAcc += wall;
  if (now - statT > 500) {
    state.fps = state.frames * 1000 / (now - statT); state.rtf = state.wallAcc ? state.simAcc / state.wallAcc : 0;
    state.frames = 0; state.simAcc = 0; state.wallAcc = 0; statT = now;
    const w = state.worlds[state.worlds.length - 1];
    if (ui.stats) ui.stats.textContent = `${state.fps.toFixed(0)} fps · ${(100 * state.rtf).toFixed(0)}% real time · ${w.stats.physMsPerStep.toFixed(2)} ms/step`;
    ui.focus?.classList.toggle('show', withRobot && !document.hasFocus());
  }
  requestAnimationFrame(frame);
}

function start() {
  window.simdemo.ready = boot().catch((e) => {
    console.error(e); ui.msg.textContent = 'Could not start the demo: ' + (e.message || e); ui.loading.classList.add('error'); throw e;
  });
}
if (Q.get('start') === 'click') {
  ui.loading.classList.add('click');
  ui.msg.textContent = 'Click to load the interactive demo (~15–40 MB)';
  ui.loading.addEventListener('click', () => { ui.loading.classList.remove('click'); start(); }, { once: true });
} else start();
