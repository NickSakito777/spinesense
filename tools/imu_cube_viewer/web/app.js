import * as THREE from "./vendor/three.module.js";

const IMU_IDS = ["IMU0", "IMU1", "IMU2", "IMU3", "IMU4"];
const COLORS = {
  IMU0: 0xff6f61,
  IMU1: 0x29c285,
  IMU2: 0x64a6ff,
  IMU3: 0xf2b84b,
  IMU4: 0xb68cff,
};

const state = new Map();
const zero = new Map();
let paused = false;

const canvas = document.querySelector("#scene");
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.setClearColor(0x121210, 1);

const scene = new THREE.Scene();
scene.fog = new THREE.Fog(0x121210, 9, 18);

const camera = new THREE.PerspectiveCamera(42, 1, 0.1, 100);
camera.position.set(0, 4.1, 8.2);
camera.lookAt(0, 0.45, 0);

const hemi = new THREE.HemisphereLight(0xffffff, 0x28251f, 2.2);
scene.add(hemi);

const key = new THREE.DirectionalLight(0xffffff, 2.1);
key.position.set(3, 5, 4);
scene.add(key);

const grid = new THREE.GridHelper(12, 24, 0x55504a, 0x2a2824);
grid.position.y = -1.2;
scene.add(grid);

const cubes = new Map();
let scenePositions = spreadPositions(IMU_IDS.length, 2.1);

for (const [index, id] of IMU_IDS.entries()) {
  const group = new THREE.Group();
  group.position.set(scenePositions[index], 0, 0);
  scene.add(group);

  const cube = new THREE.Mesh(
    new THREE.BoxGeometry(1.22, 0.28, 1.22),
    [
      new THREE.MeshStandardMaterial({ color: COLORS[id], roughness: 0.42, metalness: 0.04 }),
      new THREE.MeshStandardMaterial({ color: COLORS[id], roughness: 0.42, metalness: 0.04 }),
      new THREE.MeshStandardMaterial({ color: 0xf2f2ea, roughness: 0.35, metalness: 0.03 }),
      new THREE.MeshStandardMaterial({ color: 0x2b2924, roughness: 0.55, metalness: 0.02 }),
      new THREE.MeshStandardMaterial({ color: 0x6bd3ff, roughness: 0.38, metalness: 0.03 }),
      new THREE.MeshStandardMaterial({ color: 0xf2b84b, roughness: 0.38, metalness: 0.03 }),
    ],
  );
  group.add(cube);

  const edges = new THREE.LineSegments(
    new THREE.EdgesGeometry(cube.geometry),
    new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.45 }),
  );
  group.add(edges);

  const axisGroup = new THREE.Group();
  axisGroup.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 0.95, 0xff4d4d, 0.16, 0.1));
  axisGroup.add(new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), 0.7, 0x28c782, 0.16, 0.1));
  axisGroup.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0), 0.95, 0x64a6ff, 0.16, 0.1));
  axisGroup.position.set(0, 0.34, 0);
  group.add(axisGroup);

  const text = makeLabel(id, COLORS[id]);
  text.position.set(0, -0.72, 0);
  scene.add(text);
  text.userData.followX = scenePositions[index];
  text.userData.followY = -1.82;

  cubes.set(id, {
    group,
    label: text,
    targetRoll: 0,
    targetPitch: 0,
    currentRoll: 0,
    currentPitch: 0,
  });
  zero.set(id, { roll: 0, pitch: 0 });
}

document.querySelector("#zeroButton").addEventListener("click", () => {
  for (const id of IMU_IDS) {
    const s = state.get(id);
    if (s) {
      zero.set(id, { roll: s.roll_deg, pitch: s.pitch_deg });
    }
  }
});

document.querySelector("#pauseButton").addEventListener("click", (event) => {
  paused = !paused;
  event.currentTarget.textContent = paused ? "Resume" : "Pause";
});

const serialState = document.querySelector("#serialState");
const bridgeMessage = document.querySelector("#bridgeMessage");
const readout = document.querySelector("#readout");

buildReadoutCards();

const source = new EventSource("/events");
source.onmessage = (event) => {
  const payload = JSON.parse(event.data);
  if (payload.type === "status") {
    setBridgeStatus(payload);
    return;
  }
  if (payload.type === "sample") {
    state.set(payload.imu, payload);
    updateCard(payload);
    if (!paused) {
      const z = zero.get(payload.imu) || { roll: 0, pitch: 0 };
      const cube = cubes.get(payload.imu);
      if (cube) {
        cube.targetRoll = THREE.MathUtils.degToRad(payload.roll_deg - z.roll);
        cube.targetPitch = THREE.MathUtils.degToRad(payload.pitch_deg - z.pitch);
      }
    }
  }
};

source.onerror = () => {
  serialState.textContent = "offline";
  serialState.className = "state-pill error";
  bridgeMessage.textContent = "SSE disconnected";
};

function setBridgeStatus(payload) {
  serialState.textContent = payload.serial || "status";
  serialState.className = `state-pill ${payload.serial || ""}`;
  bridgeMessage.textContent = payload.message || "";
}

function updateCard(sample) {
  const card = document.querySelector(`[data-imu="${sample.imu}"]`);
  if (!card) return;
  card.dataset.lastSeen = String(Date.now());
  const live = card.querySelector(".live");
  live.textContent = "LIVE";
  live.className = "live on";
  card.querySelector('[data-field="addr"]').textContent = sample.addr;
  card.querySelector('[data-field="roll"]').textContent = sample.roll_deg.toFixed(1);
  card.querySelector('[data-field="pitch"]').textContent = sample.pitch_deg.toFixed(1);
  card.querySelector('[data-field="g"]').textContent = sample.g_mag.toFixed(0);
}

setInterval(() => {
  const now = Date.now();
  for (const id of IMU_IDS) {
    const card = document.querySelector(`[data-imu="${id}"]`);
    const live = card.querySelector(".live");
    const lastSeen = Number(card.dataset.lastSeen || 0);
    const isLive = now - lastSeen < 900;
    live.textContent = isLive ? "LIVE" : "LOST";
    live.className = `live ${isLive ? "on" : "off"}`;
  }
}, 250);

function makeLabel(text, color) {
  const canvas2d = document.createElement("canvas");
  canvas2d.width = 512;
  canvas2d.height = 160;
  const ctx = canvas2d.getContext("2d");
  ctx.clearRect(0, 0, canvas2d.width, canvas2d.height);
  ctx.fillStyle = "#f4f4f1";
  ctx.font = "700 62px system-ui, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(text, 256, 70);
  ctx.fillStyle = `#${color.toString(16).padStart(6, "0")}`;
  ctx.fillRect(146, 124, 220, 10);
  const texture = new THREE.CanvasTexture(canvas2d);
  const material = new THREE.SpriteMaterial({ map: texture, transparent: true });
  const sprite = new THREE.Sprite(material);
  sprite.scale.set(1.45, 0.45, 1);
  return sprite;
}

function buildReadoutCards() {
  readout.replaceChildren(
    ...IMU_IDS.map((id) => {
      const card = document.createElement("article");
      card.className = "imu-card";
      card.dataset.imu = id;
      card.innerHTML = `
        <div class="card-head">
          <strong>${id}</strong>
          <span class="live">LOST</span>
        </div>
        <dl>
          <div><dt>addr</dt><dd data-field="addr">--</dd></div>
          <div><dt>roll</dt><dd data-field="roll">--</dd></div>
          <div><dt>pitch</dt><dd data-field="pitch">--</dd></div>
          <div><dt>|a| mg</dt><dd data-field="g">--</dd></div>
        </dl>
      `;
      return card;
    }),
  );
}

function spreadPositions(count, spacing) {
  const center = (count - 1) / 2;
  return Array.from({ length: count }, (_, index) => (index - center) * spacing);
}

function resize() {
  const width = window.innerWidth;
  const height = window.innerHeight;
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  applySceneLayout();
  camera.updateProjectionMatrix();
}

window.addEventListener("resize", resize);
resize();

function applySceneLayout() {
  const mobile = window.innerWidth <= 760;
  scenePositions = spreadPositions(IMU_IDS.length, mobile ? 0.82 : 2.1);
  camera.fov = mobile ? 64 : 48;
  camera.position.set(0, mobile ? 5.5 : 4.4, mobile ? 10.4 : 9.6);
  camera.lookAt(0, mobile ? 0.75 : 0.45, 0);
  grid.position.y = mobile ? -1.55 : -1.2;

  for (const [index, id] of IMU_IDS.entries()) {
    const cube = cubes.get(id);
    const scale = mobile ? 0.42 : 0.82;
    cube.group.position.x = scenePositions[index];
    cube.group.userData.baseY = mobile ? 0.7 : 0;
    cube.group.position.y = cube.group.userData.baseY;
    cube.group.scale.setScalar(scale);
    cube.label.userData.followX = scenePositions[index];
    cube.label.userData.followY = mobile ? -0.55 : -1.7;
    cube.label.scale.set(mobile ? 0.54 : 1.0, mobile ? 0.17 : 0.31, 1);
  }
}

function animate() {
  requestAnimationFrame(animate);
  const time = performance.now() * 0.001;
  for (const cube of cubes.values()) {
    cube.currentRoll = THREE.MathUtils.lerp(cube.currentRoll, cube.targetRoll, 0.2);
    cube.currentPitch = THREE.MathUtils.lerp(cube.currentPitch, cube.targetPitch, 0.2);
    cube.group.rotation.set(cube.currentRoll, cube.currentPitch, 0, "XYZ");
    cube.group.position.y = (cube.group.userData.baseY || 0) + Math.sin(time * 1.7 + cube.group.position.x) * 0.035;
    cube.label.position.x = cube.label.userData.followX;
    cube.label.position.y = cube.label.userData.followY;
    cube.label.position.z = 0;
  }
  renderer.render(scene, camera);
}

animate();
