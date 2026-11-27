# ScanArt interactive demo (browser physics + Unitree G1)

Static, no build step, no CDN: MuJoCo 4.0.1 (WebAssembly, single-threaded) for physics, three.js r170 for rendering.
Works on GitHub Pages (no COOP/COEP headers needed).

## URL

`demo/index.html?scene=<id>[&robot=g1|none][&cam=follow|free][&start=click][&ui=min][&freeze=0|1]`

| param | meaning |
|---|---|
| `scene` | scene package in `../assets/scenes/<id>/` (default `_test`, a tiny synthetic room) |
| `robot` | `g1` (default): a Unitree G1 walks with its locomotion policy; `none`: scene explorer only |
| `cam` | `follow` (default with a robot; you can still orbit around it) or `free` (scene camera from `meta.json`) |
| `start=click` | show a "click to load" card instead of loading at once (for embedding in a page) |
| `ui=min` | hide the help line and the suggestion chips |
| `freeze` | force the far-object pinning fallback on/off (default: on only for packages without MuJoCo sleeping and with > 25 free objects) |
| `twin` | optional second package shown side by side, driven by the same keys (same frame as `scene`) |

Embed: `<iframe src="demo/index.html?scene=office_ours&start=click" allow="fullscreen"></iframe>` (keyboard works after a click in the frame).

## Controls

- **W/S** or **↑/↓** walk forward / back, **A/D** or **←/→** turn, **Q/E** side-step, **Shift** faster, **Space** stop
- **R** reset the robot, **C** follow / free camera, *Reset scene* restores every object
- **G** / *Pick up / put down*: assisted grasp of the nearest small free object (≤ 2 kg, ≤ 40 cm) within 0.85 m in front
  of the robot; G again puts it on the surface in front (or drops it)
- **Mouse drag** on a door, drawer, object (or the robot) pulls it with a spring; orbit / zoom elsewhere
- Text box (Enter to focus): "go to the fridge", "pick up the book", "put it on the dining table", "bring the apple to the
  counter", "put it down", "go back", "stop". Words are matched against the package's `landmarks` and the labels of the
  small free objects (nearest instance). Navigation: a 5 cm occupancy grid rasterised from the CURRENT poses of all
  collision geoms (static, free, jointed; robot and carried object excluded) between 6 cm and 1.4 m, a distance field
  (cells < 0.24 m from obstacles blocked, < 0.6 m penalised), Dijkstra to the reachable cell nearest the goal,
  clearance-keeping shortcuts, pure pursuit that slows near obstacles and the goal, replanning when stalled or off the
  path, and a final turn to face the target.
- Touch screens: on-screen arrow pad.

## Files

| path | size | what |
|---|---|---|
| `demo/index.html`, `demo/demo.css`, `demo/js/*.js` | 65 KB | page, runtime (`world.js` physics + G1 control, `view.js` rendering / picking, `nav.js` navigation, `policy.js` LSTM policy, `app.js` UI) |
| `vendor/mujoco/` | 11.5 MB (2.7 MB gzipped) | `@mujoco/mujoco` 4.0.1 `mujoco.js` + `mujoco.wasm`, Apache-2.0 |
| `vendor/three/` | 0.85 MB | three.js r170 module build + GLTFLoader, OrbitControls, BufferGeometryUtils, MIT |
| `assets/g1/` | 3.6 MB | G1 12-DoF MJCF + meshes (decimated 503k -> 72k triangles), policy weights `policy.bin` (31,404 float32) + `policy.json` |

## Runtime notes

- Scene package contract: `physics.xml` (+ `meshes/*.stl`), `visual.glb` (z up, nodes named like the moving MJCF bodies,
  world coordinates at rest), `meta.json` (`title`, `bounds`, `spawn`, `camera`, `bodies`, `landmarks`; `nodes.ceiling`
  is hidden when the camera is above the room).
- `physics_web.xml` (optional, preferred): the same model with every hull inlined as vertex lists (one ~0.2-0.7 MB
  gzipped file instead of thousands of STL requests). Made by `pack_physics.py <package dir>` (kept outside the site,
  a build script); its first line holds the sha1 of the `physics.xml`
  it was packed from and the runtime ignores it (loads `meshes/`, 8 requests at a time) when they differ. Re-run it
  after every change of a package; with it, `meshes/` need not be deployed.
- Speed: packages that start objects asleep (`<flag sleep="enable"/>`, body `sleep="init"`) cost ~0.15-0.3 ms per
  2 ms step with the G1 walking (the robot is `sleep="never"`); objects wake on contact or when dragged.
- Idle stance: the unitree_rl_gym policy has a 0.8 s gait clock in its observation and was trained with a foot-contact
  reward tied to that clock, so at zero command it steps in place. After 0.5 s of zero command, at a double-support
  instant of the clock with the robot calm (|v| < 0.15 m/s), the runtime holds the joint pose with stiffer PD gains
  (ankle kp 300: the policy's 40 is below m·g·h of the standing robot) plus a COM-over-feet feedback on the ankles;
  any command, a push (|v| > 0.3 m/s, tilt > 10°) or a COM outside the feet hands control back to the policy (LSTM
  state and gait clock reset), which steps to recover and stands again when calm.
- Assisted grasp: the 12-DoF model has no arm joints, so the grasp is honest about being assisted: the object is moved
  kinematically (0.7 s) to a carry pose in front of the hands and held by a MuJoCo weld to the pelvis, one inactive
  `<weld>` per free object is added when the XML is composed and switched with `eq_active` (relative pose written to
  `eq_data`). While carried it collides with the scene but not with the robot (contact bit 2); after release it
  falls / settles under physics. Carrying ≤ 2 kg was tested walking, standing and with pushes.
- Follow camera: orbit/zoom around the robot; it swings behind the robot while it walks and is pulled in / raised when
  a wall is between robot and camera (`mj_ray` on the scene collision geoms).
- The G1 MJCF is merged into the scene XML in the browser (names prefixed `g1_`, own default class, `sleep="never"`).
  Physics dt 2 ms, policy at 50 Hz (decimation 10), PD torques kp/kd from unitree_rl_gym `deploy_mujoco/configs/g1.yaml`.
- Moving visual nodes follow their bodies: `node = T_body(t) · T_body(rest)^-1 · node_rest`.
- `window.scanart` exposes `state.worlds` / `state.views`, `setKeys([...])`, `command(text)` for scripted tests.

## Credits / licences

- MuJoCo WebAssembly bindings, Google DeepMind, Apache-2.0 — `vendor/mujoco/LICENSE`
- three.js, MIT — `vendor/three/LICENSE`
- Unitree G1 model (`resources/robots/g1_description/g1_12dof.xml` + meshes) and pretrained locomotion policy
  (`deploy/pre_train/g1/motion.pt`) from unitree_rl_gym (github.com/unitreerobotics/unitree_rl_gym, commit 276801e),
  BSD-3-Clause, © Unitree Robotics — `assets/g1/LICENSE.unitree_rl_gym`. Meshes decimated and weights re-exported, no
  other change.
