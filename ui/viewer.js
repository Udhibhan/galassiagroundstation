// viewer.js - the 3D gimbal on the page. Loads /model/gimbal.glb (converted from "XY Dish Assembly.STEP") and moves
// the X and Y axes from the live angles (window.GS.gimbal.axes.x/y.angle). No internet needed: three.js is in /vendor.
//
// Model (ui/model/gimbal_meta.json): metres, +Y up, nodes BASE / XGROUP / YGROUP (YGROUP is a child of XGROUP).
//   X axis: xgroup.rotation.x          Y axis: ygroup.rotation.z (the physical 1.2 degree tilt is baked into rotation.x, kept)
// Mount frame (kinematics.py): U = dish normal at zero = +Y, E = X axis = +X, N = U x E = -Z.
//   +X tilts the dish toward South, +Y toward East.  The Y shaft is along +Z = -N, so +Y angle = rotation.z of -angle.
// NOT real time: the pose is applied only when a move has finished (or every 15 s during continuous tracking), and the scene
// is drawn once per change (or while you drag the camera). Nothing animates, so it costs almost nothing.
import * as THREE from "three";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";

const box = document.getElementById("v3d");
const msg = document.getElementById("v3d-msg");
const noteEl = document.getElementById("v3d-note");
const D2R = Math.PI / 180;

let renderer;
try {
  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: "high-performance" });
} catch (e) {
  msg.textContent = "The 3D view needs WebGL, which this browser does not offer. Everything else works.";
  throw e;
}
const MAX_Q = Math.min(window.devicePixelRatio || 1, 2);
let quality = MAX_Q;                                         // full resolution: the scene is static, so this is cheap
renderer.setPixelRatio(quality);
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 0.95;
box.prepend(renderer.domElement);
renderer.domElement.style.cssText = "position:absolute;inset:0;display:block;width:100%;height:100%;touch-action:none";

const scene = new THREE.Scene();
const pmrem = new THREE.PMREMGenerator(renderer);
scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;   // soft studio reflections on the metal
const camera = new THREE.PerspectiveCamera(32, 1, 0.01, 100);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.09;
controls.maxPolarAngle = Math.PI * 0.52;
controls.minDistance = 1.2; controls.maxDistance = 9;
scene.add(new THREE.HemisphereLight(0xffffff, 0x9aa7b3, 0.55));
const key = new THREE.DirectionalLight(0xffffff, 1.9); key.position.set(3, 5, 4); scene.add(key);
const rim = new THREE.DirectionalLight(0xcfe3ff, 0.9); rim.position.set(-4, 3, -3); scene.add(rim);

const E = new THREE.Vector3(1, 0, 0), U = new THREE.Vector3(0, 1, 0), N = new THREE.Vector3(0, 0, -1);
let xNode = null, yNode = null, sy = -1, centre = new THREE.Vector3(0.05, 0.7, 0), R = 1.2;
const cur = { x: 0, y: 0 };
const compassSprites = [];
let sky = null, sunDot = null, satDot = null, satPath = null, sunPath = null, ready = false;

function label(text, color) {
  const c = document.createElement("canvas"); c.width = c.height = 128;
  const g = c.getContext("2d"); g.font = "700 76px Inter, system-ui, Arial, sans-serif"; g.fillStyle = color; g.textAlign = "center"; g.textBaseline = "middle";
  g.fillText(text, 64, 68);
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthTest: false, transparent: true }));
  sp.renderOrder = 10;
  return sp;
}

// true az/el (degrees) -> point on the sky dome around the dish. The mount may be rotated from true north by yaw: az_mount = az_true - yaw.
function skyPoint(az, el, r) {
  const yaw = (window.GS && window.GS.mount) ? window.GS.mount.yaw : 0;
  const a = (az - yaw) * D2R, e = el * D2R;
  return E.clone().multiplyScalar(Math.sin(a) * Math.cos(e)).add(N.clone().multiplyScalar(Math.cos(a) * Math.cos(e)))
    .add(U.clone().multiplyScalar(Math.sin(e))).multiplyScalar(r).add(centre);
}

// --- make the model cheap to draw: ~280 meshes -> a handful, drop tiny screws, colour by what moves ---------------------------------
const PALETTE = { BASE: 0x5F6B77, XGROUP: 0x8795A2, YGROUP: 0xB4C0CB };      // fixed -> darker, moving -> lighter
function optimise(root) {
  root.updateMatrixWorld(true);
  const groups = ["BASE", "XGROUP", "YGROUP"].map(n => root.getObjectByName(n)).filter(Boolean);
  const nearest = o => { for (let p = o.parent; p; p = p.parent) if (groups.includes(p)) return p; return root; };
  const meshes = []; root.traverse(o => { if (o.isMesh) meshes.push(o); });
  const buckets = new Map();
  let dropped = 0;
  for (const m of meshes) {
    const geo = m.geometry; geo.computeBoundingSphere();
    if (geo.boundingSphere.radius * m.matrixWorld.getMaxScaleOnAxis() < 0.008) { dropped++; continue; }   // under ~1.6 cm across
    const g = nearest(m);
    let b = buckets.get(g.uuid); if (!b) buckets.set(g.uuid, b = { g, parts: [] });
    b.parts.push(m);
  }
  const v = new THREE.Vector3(), nm = new THREE.Matrix3();
  for (const { g, parts } of buckets.values()) {
    const inv = g.matrixWorld.clone().invert();
    let nv = 0, ni = 0;
    for (const m of parts) { nv += m.geometry.attributes.position.count; ni += m.geometry.index ? m.geometry.index.count : m.geometry.attributes.position.count; }
    const pos = new Float32Array(nv * 3), nor = new Float32Array(nv * 3), idx = new Uint32Array(ni);
    let vo = 0, io = 0;
    for (const m of parts) {
      const geo = m.geometry, P = geo.attributes.position, Nn = geo.attributes.normal;
      const rel = inv.clone().multiply(m.matrixWorld); nm.getNormalMatrix(rel);
      for (let i = 0; i < P.count; i++) {
        v.fromBufferAttribute(P, i).applyMatrix4(rel); pos.set([v.x, v.y, v.z], (vo + i) * 3);
        if (Nn) { v.fromBufferAttribute(Nn, i).applyMatrix3(nm).normalize(); nor.set([v.x, v.y, v.z], (vo + i) * 3); }
      }
      if (geo.index) for (let i = 0; i < geo.index.count; i++) idx[io++] = geo.index.getX(i) + vo;
      else for (let i = 0; i < P.count; i++) idx[io++] = vo + i;
      vo += P.count;
    }
    const mg = new THREE.BufferGeometry();
    mg.setAttribute("position", new THREE.BufferAttribute(pos, 3)); mg.setAttribute("normal", new THREE.BufferAttribute(nor, 3)); mg.setIndex(new THREE.BufferAttribute(idx, 1));
    mg.computeBoundingSphere();
    const mat = new THREE.MeshStandardMaterial({ color: PALETTE[g.name] || 0xaab4bd, metalness: 0.45, roughness: 0.46, envMapIntensity: 0.95 });
    g.add(new THREE.Mesh(mg, mat));                           // geometry is already in the group's local frame
  }
  for (const m of meshes) { m.removeFromParent(); m.geometry.dispose(); }
  console.log(`3D model: ${meshes.length} meshes -> ${buckets.size} draw calls (${dropped} tiny parts left out)`);
}

function softShadow(radius) {                                  // contact shadow under the base: a radial gradient, no shadow maps
  const c = document.createElement("canvas"); c.width = c.height = 256;
  const g = c.getContext("2d"), gr = g.createRadialGradient(128, 128, 8, 128, 128, 126);
  gr.addColorStop(0, "rgba(14,26,36,.55)"); gr.addColorStop(0.45, "rgba(14,26,36,.22)"); gr.addColorStop(1, "rgba(14,26,36,0)");
  g.fillStyle = gr; g.fillRect(0, 0, 256, 256);
  const m = new THREE.Mesh(new THREE.PlaneGeometry(radius * 2, radius * 2),
    new THREE.MeshBasicMaterial({ map: new THREE.CanvasTexture(c), transparent: true, depthWrite: false }));
  m.rotation.x = -Math.PI / 2;
  return m;
}

function beamMesh(len, radius) {                               // translucent cone that fades out along its length
  const g = new THREE.ConeGeometry(radius, len, 48, 1, true);
  g.rotateX(Math.PI); g.translate(0, len / 2, 0);              // apex at the origin, opening along +Y
  const mat = new THREE.ShaderMaterial({
    transparent: true, depthWrite: false, side: THREE.DoubleSide,
    uniforms: { c: { value: new THREE.Color(0x14b8d4) }, len: { value: len } },
    vertexShader: "varying float h; uniform float len; void main(){ h = position.y/len; gl_Position = projectionMatrix*modelViewMatrix*vec4(position,1.0); }",
    fragmentShader: "varying float h; uniform vec3 c; void main(){ gl_FragColor = vec4(c, 0.42*pow(1.0-h,1.3)); }",
  });
  const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(0, len * 1.1, 0)]),
    new THREE.LineBasicMaterial({ color: 0x0b9bb4, transparent: true, opacity: 0.8 }));
  const grp = new THREE.Group(); grp.add(new THREE.Mesh(g, mat), line);
  return grp;
}

let dirty = true, enabled = true;
const invalidate = () => { dirty = true; scheduleRender(); };

async function init() {
  let meta = {};
  try { meta = await (await fetch("/model/gimbal_meta.json", { cache: "no-store" })).json(); } catch (e) {}
  const gltf = await new Promise((res, rej) => new GLTFLoader().load("/model/gimbal.glb", res, undefined, rej));
  const root = gltf.scene;
  scene.add(root);
  xNode = root.getObjectByName("XGROUP");
  yNode = root.getObjectByName("YGROUP");
  if (!xNode || !yNode) msg.textContent = "The model has no XGROUP / YGROUP nodes: showing it static.";
  optimise(root);
  root.updateMatrixWorld(true);

  const bb = new THREE.Box3().setFromObject(root);
  bb.getCenter(centre);
  const size = bb.getSize(new THREE.Vector3());
  R = Math.max(size.x, size.z) * 0.85;

  if (yNode) {                                                 // the dish axis, attached to the Y group so it moves with the dish
    const dc = new THREE.Vector3(...(meta.dish_center_world || [0.04, 0.576, -0.008]));
    const beam = beamMesh(1.15, 0.075);
    const p0 = yNode.worldToLocal(dc.clone()), p1 = yNode.worldToLocal(dc.clone().add(U));
    beam.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), p1.sub(p0).normalize());
    beam.position.copy(yNode.worldToLocal(dc.clone()));
    yNode.add(beam);
  }

  // ground: contact shadow, faint grid and a compass ring (mount frame: N = -Z, E = +X; the letters are rotated by yaw)
  const gy = bb.min.y - 0.002;
  const shadow = softShadow(0.62); shadow.position.set(0.0, gy + 0.0005, 0.0); scene.add(shadow);
  const grid = new THREE.GridHelper(R * 2, 24, 0xbcc7d0, 0xdde4ea);
  grid.position.set(centre.x, gy, centre.z);
  grid.material.transparent = true; grid.material.opacity = 0.7; grid.material.depthWrite = false;
  scene.add(grid);
  const ring = new THREE.Mesh(new THREE.RingGeometry(R * 0.992, R, 160), new THREE.MeshBasicMaterial({ color: 0x8895a1, side: THREE.DoubleSide, transparent: true, opacity: 0.7, depthWrite: false }));
  ring.rotation.x = -Math.PI / 2; ring.position.copy(grid.position); scene.add(ring);
  // letters are TRUE compass directions: placed in updateSky() rotated by the mount yaw (az_mount = az_true - yaw), like the Sun
  for (const [t, az, c] of [["N", 0, "#C9302C"], ["E", 90, "#46576A"], ["S", 180, "#46576A"], ["W", 270, "#46576A"]]) {
    const s = label(t, c); s.scale.setScalar(0.17);
    s.userData.az = az; s.userData.base = grid.position.clone(); s.userData.r = R * 1.1;
    s.position.copy(grid.position); s.position.y += 0.05;
    s.visible = false;                                          // letters only appear once north is known (aligned)
    compassSprites.push(s);
    scene.add(s);
  }
  // Sun and satellite on a dome around the dish
  sky = new THREE.Group(); scene.add(sky);
  sunDot = new THREE.Mesh(new THREE.SphereGeometry(0.06, 32, 20), new THREE.MeshBasicMaterial({ color: 0xffb703 })); sky.add(sunDot);
  satDot = new THREE.Mesh(new THREE.OctahedronGeometry(0.055), new THREE.MeshBasicMaterial({ color: 0x0c6db0 })); sky.add(satDot);
  sunPath = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xffb703, transparent: true, opacity: 0.55 }));
  satPath = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x0c6db0, transparent: true, opacity: 0.85 }));
  sky.add(sunPath, satPath);

  frame();
  ready = true;
  msg.hidden = !!enabled;
  if (!enabled) applyEnabled();
}

function frame() {
  controls.target.copy(centre).setY(centre.y - 0.22);
  camera.position.set(centre.x + 2.35, centre.y + 0.95, centre.z + 3.0);
  camera.updateProjectionMatrix();
  controls.update();
}

controls.addEventListener("change", () => { dirty = true; scheduleRender(); });
try { enabled = localStorage.getItem("gs-3d") !== "off"; } catch (e) {}
const pauseBtn = document.getElementById("v3d-pause");
function applyEnabled() {
  renderer.domElement.style.visibility = enabled ? "visible" : "hidden";
  pauseBtn.textContent = enabled ? "3D on" : "3D off";
  if (!enabled) { msg.hidden = false; msg.textContent = "3D view is off (saves CPU/GPU). Press “3D off” to turn it back on."; } else if (ready) { msg.hidden = true; invalidate(); }
}
pauseBtn.onclick = () => { enabled = !enabled; try { localStorage.setItem("gs-3d", enabled ? "on" : "off"); } catch (e) {} applyEnabled(); };
document.getElementById("v3d-reset").onclick = () => { frame(); invalidate(); };

function setLine(line, pts) { line.geometry.dispose(); line.geometry = new THREE.BufferGeometry().setFromPoints(pts); }
let lastSun = 0, lastSatKey = "";
function updateSky() {
  const G = window.GS;
  const aligned = !!(G && G.mount);
  for (const c of compassSprites) c.visible = aligned;
  sky.visible = aligned;
  if (!aligned) return;
  const yawC = G.mount.yaw || 0;
  for (const c of compassSprites) {
    const a = (c.userData.az - yawC) * D2R;
    c.position.copy(c.userData.base).add(E.clone().multiplyScalar(Math.sin(a) * c.userData.r)).add(N.clone().multiplyScalar(Math.cos(a) * c.userData.r));
    c.position.y += 0.05;
  }
  if (G.sun && G.sun.el > -2) { sunDot.visible = true; sunDot.position.copy(skyPoint(G.sun.az, G.sun.el, R)); } else sunDot.visible = false;
  if (G.sun && G.sun.path && performance.now() - lastSun > 5000) {
    lastSun = performance.now();
    setLine(sunPath, G.sun.path.map(p => skyPoint(p[0], p[1], R)));
  }
  const S = G.sat;
  if (S && S.busy && S.az != null) {
    satDot.visible = true; satDot.position.copy(skyPoint(S.az, S.el, R));
    const k = (S.pass && S.pass.rise) + ":" + (S.track ? S.track.length : 0);
    if (S.track && k !== lastSatKey) { lastSatKey = k; setLine(satPath, S.track.filter(p => p[1] >= 0).map(p => skyPoint(p[0], p[1], R))); }
    satPath.visible = true;
  } else { satDot.visible = false; satPath.visible = false; }
}

// ---- update only when the motion is done -------------------------------------------------------------------------------------------
const shown = { x: null, y: null };                             // the pose currently drawn
let lastApply = 0, lastAligned = null, firstDone = false;
const idleAxes = G => ["x", "y"].every(k => G.gimbal.axes[k].phase === "idle" && !G.gimbal.axes[k].stepping);
const motionDone = G => idleAxes(G) && !G.gimbal.act && !["homing", "starting"].includes(G.gimbal.mode);
function setNote(txt, cls) { if (noteEl && (noteEl.dataset.k !== txt + cls)) { noteEl.dataset.k = txt + cls; noteEl.textContent = txt; noteEl.className = cls; } }

function apply(tx, ty) {
  shown.x = tx; shown.y = ty; lastApply = performance.now();
  const G = window.GS, fl = (G && G.view && G.view.flip) || {};
  if (xNode) xNode.rotation.x = (fl.x ? -1 : 1) * tx * D2R;
  if (yNode) yNode.rotation.z = sy * (fl.y ? -1 : 1) * ty * D2R;
  updateSky();
  invalidate();
}
function check() {                                              // 4 times a second: cheap, draws only if something changed
  const G = window.GS;
  if (!ready || !enabled || !G || !G.gimbal) return;
  const ax = G.gimbal.axes, tx = ax.x.angle != null ? ax.x.angle : 0, ty = ax.y.angle != null ? ax.y.angle : 0;
  const done = motionDone(G), tracking = !!(G.gimbal.act && G.gimbal.act.kind === "track");
  const aligned = !!G.mount;
  if (aligned !== lastAligned) { lastAligned = aligned; updateSky(); invalidate(); }
  if (shown.x === null || (done && (Math.abs(tx - shown.x) > 0.02 || Math.abs(ty - shown.y) > 0.02))) {
    firstDone = true; apply(tx, ty);
  } else if (tracking && performance.now() - lastApply > 15000) apply(tx, ty);
  else if (!tracking && performance.now() - lastApply > 60000) { updateSky(); invalidate(); lastApply = performance.now(); }   // the Sun creeps along
  const base = aligned ? `North known · yaw ${G.mount.yaw >= 0 ? "+" : ""}${G.mount.yaw.toFixed(1)}°` : "Not aligned to north yet";
  if (tracking) setNote("Tracking · view refreshes every 15 s", "moving");
  else if (!done) setNote("Moving… the view updates when the move ends", "moving");
  else setNote(base, aligned ? "on" : "");
}
setInterval(check, 250);

let rafPending = false;
function scheduleRender() {
  if (rafPending) return;
  rafPending = true;
  requestAnimationFrame(() => {
    rafPending = false;
    if (document.hidden || !ready || !enabled) return;
    const w = box.clientWidth, h = box.clientHeight;
    if (!w || !h) return;                                       // parked (Test tab)
    if (renderer.domElement.width !== Math.floor(w * renderer.getPixelRatio()) || renderer.domElement.height !== Math.floor(h * renderer.getPixelRatio())) {
      renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix();
    }
    controls.update();
    renderer.render(scene, camera);
    dirty = false;
  });
}
document.addEventListener("visibilitychange", () => { if (!document.hidden) invalidate(); });

window.addEventListener("gs-resize", invalidate);
new ResizeObserver(invalidate).observe(box);
applyEnabled();
init().catch(e => {
  console.error(e);
  msg.hidden = false;
  msg.textContent = "The 3D model could not be loaded (ui/model/gimbal.glb): " + (e && e.message ? e.message : e);
});
