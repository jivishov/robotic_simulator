// main.js (ES module)

const $ = (id) => document.getElementById(id);

const canvas = $("canvas");
const ctx = canvas.getContext("2d");
const consoleEl = $("console");
const readoutsEl = $("readouts");
const threeContainer = $("threeContainer");

const runBtn = $("runBtn");
const stopBtn = $("stopBtn");
const resetBtn = $("resetBtn");
const homeBtn = $("homeBtn");
const toggle3dBtn = $("toggle3dBtn");

const codeEl = $("code");
codeEl.value = `import arm, sim

arm.home()
arm.set_speed(0.7)

# Draw a square
arm.move_to(140, 40)
arm.line_to(140, 120, steps=40)
arm.line_to(60, 120, steps=40)
arm.line_to(60, 40, steps=40)
arm.line_to(140, 40, steps=40)

sim.sleep(0.5)
arm.home()
`;

function log(msg) {
  consoleEl.textContent += msg + "\n";
  consoleEl.scrollTop = consoleEl.scrollHeight;
}
function clearLog() { consoleEl.textContent = ""; }

// -------------------- Robot model (2R planar) --------------------
const model = {
  L1: 160,    // pixels
  L2: 120,    // pixels
  // joint limits in radians
  q1Min: deg2rad(-170), q1Max: deg2rad(170),
  q2Min: deg2rad(-170), q2Max: deg2rad(170),
  maxSpeed: deg2rad(120),  // rad/s (scaled by speedScale)
};

const state = {
  q: [deg2rad(0), deg2rad(0)],
  speedScale: 0.7,
  toolPath: [],
  busy: false,
  currentCmd: null, // {id,type,...}
  t: 0,
};

function deg2rad(d) { return (d * Math.PI) / 180; }
function rad2deg(r) { return (r * 180) / Math.PI; }
function clamp(x, a, b) { return Math.max(a, Math.min(b, x)); }

function fk(q) {
  const [q1, q2] = q;
  const x1 = model.L1 * Math.cos(q1);
  const y1 = model.L1 * Math.sin(q1);
  const x2 = x1 + model.L2 * Math.cos(q1 + q2);
  const y2 = y1 + model.L2 * Math.sin(q1 + q2);
  return { x1, y1, x2, y2 };
}

// Closed-form IK for 2R planar, elbow-down by default
function ik(x, y) {
  const L1 = model.L1, L2 = model.L2;
  const r2 = x*x + y*y;
  const c2 = (r2 - L1*L1 - L2*L2) / (2*L1*L2);

  if (c2 < -1 || c2 > 1) return { ok: false, reason: "unreachable" };

  const s2_pos = Math.sqrt(Math.max(0, 1 - c2*c2));
  const s2_neg = -s2_pos;

  const q2a = Math.atan2(s2_pos, c2); // elbow-up
  const q2b = Math.atan2(s2_neg, c2); // elbow-down

  const q1a = Math.atan2(y, x) - Math.atan2(L2*Math.sin(q2a), L1 + L2*Math.cos(q2a));
  const q1b = Math.atan2(y, x) - Math.atan2(L2*Math.sin(q2b), L1 + L2*Math.cos(q2b));

  const candA = [wrap(q1a), wrap(q2a)];
  const candB = [wrap(q1b), wrap(q2b)];

  const okA = withinLimits(candA);
  const okB = withinLimits(candB);
  if (okB) return { ok: true, q: candB, alt: candA, chosen: "elbow-down" };
  if (okA) return { ok: true, q: candA, alt: candB, chosen: "elbow-up" };
  return { ok: false, reason: "joint-limits" };
}

function wrap(a) {
  while (a > Math.PI) a -= 2*Math.PI;
  while (a < -Math.PI) a += 2*Math.PI;
  return a;
}
function withinLimits(q) {
  return (q[0] >= model.q1Min && q[0] <= model.q1Max && q[1] >= model.q2Min && q[1] <= model.q2Max);
}

// -------------------- Command execution --------------------
const cmdQueue = [];
let nextCmdId = 1;

function enqueue(cmd) {
  cmdQueue.push(cmd);
  tryStartNext();
}

function tryStartNext() {
  if (state.busy) return;
  const cmd = cmdQueue.shift();
  if (!cmd) return;
  startCommand(cmd);
}

function startCommand(cmd) {
  state.busy = true;
  state.currentCmd = cmd;
  state.t = 0;

  if (cmd.type === "HOME") {
    cmd.qStart = [...state.q];
    cmd.qTarget = [0, 0];
    cmd.duration = computeDuration(cmd.qStart, cmd.qTarget);
  } else if (cmd.type === "SET_JOINTS") {
    cmd.qStart = [...state.q];
    cmd.qTarget = [
      clamp(deg2rad(cmd.deg[0]), model.q1Min, model.q1Max),
      clamp(deg2rad(cmd.deg[1]), model.q2Min, model.q2Max),
    ];
    cmd.duration = computeDuration(cmd.qStart, cmd.qTarget);
  } else if (cmd.type === "MOVE_TO") {
    const sol = ik(cmd.x, cmd.y);
    if (!sol.ok) {
      finishCommand(cmd, { ok: false, error: sol.reason });
      return;
    }
    cmd.qStart = [...state.q];
    cmd.qTarget = sol.q;
    cmd.duration = computeDuration(cmd.qStart, cmd.qTarget);
    cmd.ik = sol;
  } else if (cmd.type === "SLEEP") {
    cmd.duration = Math.max(0, cmd.seconds);
  } else if (cmd.type === "LINE_TO") {
    // Expand into a series of MOVE_TO commands
    const ee = fk(state.q);
    const x0 = ee.x2, y0 = ee.y2;
    const steps = Math.max(2, cmd.steps | 0);
    for (let i = 1; i <= steps; i++) {
      const u = i / steps;
      enqueue({ id: `${cmd.id}:${i}`, type: "MOVE_TO", x: x0 + (cmd.x - x0)*u, y: y0 + (cmd.y - y0)*u, fromLine: true, parentId: cmd.id });
    }
    finishCommand(cmd, { ok: true, note: "expanded" });
    return;
  } else {
    finishCommand(cmd, { ok: false, error: "unknown-cmd" });
    return;
  }
}

function computeDuration(qStart, qTarget) {
  const dq1 = Math.abs(qTarget[0] - qStart[0]);
  const dq2 = Math.abs(qTarget[1] - qStart[1]);
  const dq = Math.max(dq1, dq2);
  const speed = model.maxSpeed * clamp(state.speedScale, 0.05, 1.0);
  return Math.max(0.08, dq / speed);
}

function update(dt) {
  if (state.busy && state.currentCmd) {
    const cmd = state.currentCmd;
    state.t += dt;

    if (cmd.type === "SLEEP") {
      if (state.t >= cmd.duration) finishCommand(cmd, { ok: true });
    } else if (cmd.type === "HOME" || cmd.type === "SET_JOINTS" || cmd.type === "MOVE_TO") {
      const u = clamp(state.t / cmd.duration, 0, 1);
      const s = smoothstep(u);
      state.q[0] = lerp(cmd.qStart[0], cmd.qTarget[0], s);
      state.q[1] = lerp(cmd.qStart[1], cmd.qTarget[1], s);

      const ee = fk(state.q);
      state.toolPath.push({ x: ee.x2, y: ee.y2 });

      if (u >= 1) finishCommand(cmd, { ok: true, ik: cmd.ik?.chosen });
    }
  }
}

function finishCommand(cmd, result) {
  state.busy = false;
  state.currentCmd = null;
  worker.postMessage({ type: "cmd_done", id: cmd.id, result });
  tryStartNext();
}

function lerp(a, b, t) { return a + (b - a) * t; }
function smoothstep(t) { return t*t*(3 - 2*t); }

// -------------------- Rendering (2D) --------------------
function draw() {
  // Light background
  ctx.fillStyle = "#f0f4f8";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // coordinate transform: base at bottom-left-ish, y up
  const base = { x: 140, y: canvas.height - 100 };

  // grid
  ctx.save();
  ctx.strokeStyle = "#c0c8d0";
  ctx.globalAlpha = 0.5;
  for (let x = 0; x < canvas.width; x += 40) { line(x, 0, x, canvas.height); }
  for (let y = 0; y < canvas.height; y += 40) { line(0, y, canvas.width, y); }
  ctx.restore();

  // axes
  ctx.save();
  ctx.strokeStyle = "#607080";
  ctx.lineWidth = 2;
  ctx.globalAlpha = 0.8;
  line(base.x, base.y, base.x + 250, base.y);
  line(base.x, base.y, base.x, base.y - 250);
  ctx.restore();

  // FK points
  const k = fk(state.q);
  const p0 = { x: base.x, y: base.y };
  const p1 = { x: base.x + k.x1, y: base.y - k.y1 };
  const p2 = { x: base.x + k.x2, y: base.y - k.y2 };

  // toolpath
  ctx.save();
  ctx.strokeStyle = "#e74c3c";
  ctx.lineWidth = 2;
  ctx.globalAlpha = 0.9;
  ctx.beginPath();
  for (let i = 0; i < state.toolPath.length; i++) {
    const tp = state.toolPath[i];
    const px = base.x + tp.x;
    const py = base.y - tp.y;
    if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
  }
  ctx.stroke();
  ctx.restore();

  // links
  ctx.save();
  ctx.strokeStyle = "#3498db";
  ctx.lineWidth = 10;
  ctx.lineCap = "round";
  line(p0.x, p0.y, p1.x, p1.y);
  line(p1.x, p1.y, p2.x, p2.y);
  ctx.restore();

  // joints
  ctx.save();
  ctx.fillStyle = "#2c3e50";
  circle(p0.x, p0.y, 12, true);
  circle(p1.x, p1.y, 10, true);
  ctx.fillStyle = "#e74c3c";
  circle(p2.x, p2.y, 6, true);
  ctx.restore();

  // readouts
  readoutsEl.textContent =
    `q1=${rad2deg(state.q[0]).toFixed(1)}°  q2=${rad2deg(state.q[1]).toFixed(1)}°  ` +
    `speed=${state.speedScale.toFixed(2)}  queue=${cmdQueue.length}  busy=${state.busy}`;
}

function line(x1,y1,x2,y2){
  ctx.beginPath(); ctx.moveTo(x1,y1); ctx.lineTo(x2,y2); ctx.stroke();
}
function circle(x,y,r,fill){
  ctx.beginPath(); ctx.arc(x,y,r,0,Math.PI*2);
  fill ? ctx.fill() : ctx.stroke();
}

// -------------------- Rendering (3D preview) --------------------
const threeState = {
  enabled: false,
  ready: false,
  THREE: null,
  renderer: null,
  scene: null,
  camera: null,
  link1: null,
  link2: null,
  joints: [],
  grid: null,
  path: null,
  lastPathLen: 0,
};

let threeModulePromise = null;

function ensureThree() {
  if (threeState.ready) return Promise.resolve();
  if (!threeModulePromise) {
    threeModulePromise = import("https://cdn.jsdelivr.net/npm/three@0.164/build/three.module.js");
  }
  return threeModulePromise.then((THREE) => {
    initThree(THREE);
  }).catch((err) => {
    log("[3D] failed to load Three.js: " + err);
  });
}

function initThree(THREE) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b1119);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio || 1);

  const camera = new THREE.PerspectiveCamera(45, 1, 1, 2000);
  camera.position.set(250, 200, 360);
  camera.lookAt(0, 0, 0);

  const light = new THREE.DirectionalLight(0xffffff, 1.0);
  light.position.set(120, 200, 200);
  scene.add(light);
  scene.add(new THREE.AmbientLight(0x708090, 0.65));

  const grid = new THREE.GridHelper(500, 10, 0x2a3b52, 0x1b2633);
  grid.rotation.x = Math.PI / 2; // place in XY plane
  scene.add(grid);

  const linkMaterial = new THREE.MeshStandardMaterial({ color: 0x6ba4ff, metalness: 0.05, roughness: 0.6 });
  const jointMaterial = new THREE.MeshStandardMaterial({ color: 0xe8eef5, metalness: 0.05, roughness: 0.45 });

  const link1 = new THREE.Mesh(new THREE.BoxGeometry(model.L1, 12, 12), linkMaterial);
  const link2 = new THREE.Mesh(new THREE.BoxGeometry(model.L2, 10, 10), linkMaterial);
  link1.castShadow = link2.castShadow = false;
  link1.receiveShadow = link2.receiveShadow = false;

  const joint0 = new THREE.Mesh(new THREE.SphereGeometry(10, 24, 16), jointMaterial);
  const joint1 = new THREE.Mesh(new THREE.SphereGeometry(8, 24, 16), jointMaterial);
  const joint2 = new THREE.Mesh(new THREE.SphereGeometry(7, 24, 16), jointMaterial);

  const pathMaterial = new THREE.LineBasicMaterial({ color: 0xb9e2ff, linewidth: 2 });
  const pathGeometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0)]);
  const path = new THREE.Line(pathGeometry, pathMaterial);

  scene.add(link1, link2, joint0, joint1, joint2, path);

  threeState.THREE = THREE;
  threeState.renderer = renderer;
  threeState.scene = scene;
  threeState.camera = camera;
  threeState.link1 = link1;
  threeState.link2 = link2;
  threeState.joints = [joint0, joint1, joint2];
  threeState.grid = grid;
  threeState.path = path;
  threeState.ready = true;
  threeState.lastPathLen = 0;

  threeContainer.textContent = "";
  threeContainer.appendChild(renderer.domElement);
  resizeThree();
}

function resizeThree() {
  if (!threeState.ready) return;
  const rect = threeContainer.getBoundingClientRect();
  const w = rect.width || 1;
  const h = rect.height || 1;
  threeState.renderer.setSize(w, h, false);
  threeState.camera.aspect = w / h;
  threeState.camera.updateProjectionMatrix();
}

function updateThreeScene() {
  if (!threeState.ready || !threeState.enabled) return;
  const { THREE, link1, link2, joints, path } = threeState;
  const k = fk(state.q);

  // base at origin, arm lies in XY plane
  const base = new THREE.Vector3(0, 0, 0);
  const p1 = new THREE.Vector3(k.x1, k.y1, 0);
  const p2 = new THREE.Vector3(k.x2, k.y2, 0);

  // link1 transform
  const q1 = state.q[0];
  link1.position.set(p1.x / 2, p1.y / 2, 0);
  link1.rotation.set(0, 0, q1);

  // link2 transform
  const q2 = state.q[0] + state.q[1];
  const link2Offset = new THREE.Vector3((model.L2 / 2) * Math.cos(q2), (model.L2 / 2) * Math.sin(q2), 0);
  link2.position.set(p1.x + link2Offset.x, p1.y + link2Offset.y, 0);
  link2.rotation.set(0, 0, q2);

  // joints
  joints[0].position.copy(base);
  joints[1].position.copy(p1);
  joints[2].position.copy(p2);

  // path
  if (state.toolPath.length !== threeState.lastPathLen) {
    threeState.lastPathLen = state.toolPath.length;
    const pts = state.toolPath.map((tp) => new THREE.Vector3(tp.x, tp.y, 0.5));
    if (pts.length === 0) pts.push(new THREE.Vector3(0, 0, 0));
    path.geometry.setFromPoints(pts);
  }

  threeState.renderer.render(threeState.scene, threeState.camera);
}

// -------------------- Worker (Python) --------------------
const worker = new Worker("./py_worker.js", { type: "classic" });

worker.onmessage = (ev) => {
  const msg = ev.data;
  if (!msg || !msg.type) return;

  if (msg.type === "print") log(msg.text);
  if (msg.type === "error") log(msg.text);

  if (msg.type === "enqueue_cmd") {
    enqueue(msg.cmd);
  }

  if (msg.type === "set_speed") {
    state.speedScale = clamp(msg.value, 0, 1);
  }

  if (msg.type === "reset_path") {
    state.toolPath = [];
    threeState.lastPathLen = 0;
  }
};

function hardResetSim() {
  cmdQueue.length = 0;
  state.busy = false;
  state.currentCmd = null;
  state.q = [0, 0];
  state.speedScale = 0.7;
  state.toolPath = [];
  threeState.lastPathLen = 0;
  worker.postMessage({ type: "sim_reset_done" });
  draw();
}

runBtn.onclick = () => {
  clearLog();
  state.toolPath = [];
  threeState.lastPathLen = 0;
  worker.postMessage({ type: "run", code: codeEl.value });
};

stopBtn.onclick = () => {
  cmdQueue.length = 0;
  state.busy = false;
  state.currentCmd = null;
  worker.postMessage({ type: "stop" });
  log("[stopped]");
};

resetBtn.onclick = () => {
  clearLog();
  hardResetSim();
  log("[reset]");
};

homeBtn.onclick = () => {
  state.toolPath = [];
  threeState.lastPathLen = 0;
  enqueue({ id: nextCmdId++, type: "HOME" });
};

toggle3dBtn.onclick = () => {
  if (threeState.enabled) {
    threeState.enabled = false;
    toggle3dBtn.textContent = "3D View (off)";
    if (threeState.renderer) {
      threeState.renderer.setAnimationLoop(null);
    }
    return;
  }
  toggle3dBtn.textContent = "3D View (loading...)";
  ensureThree().then(() => {
    if (!threeState.ready) {
      toggle3dBtn.textContent = "3D View (off)";
      return;
    }
    threeState.enabled = true;
    toggle3dBtn.textContent = "3D View (on)";
    resizeThree();
  });
};

window.addEventListener("resize", resizeThree);

// animation loop
let last = performance.now();
function tick(now) {
  const dt = Math.min(0.05, (now - last) / 1000);
  last = now;
  update(dt);
  draw();
  updateThreeScene();
  requestAnimationFrame(tick);
}
requestAnimationFrame(tick);
