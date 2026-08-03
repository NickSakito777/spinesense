import * as THREE from "./vendor/three.module.js";

// 5IMU spine chain, v0 (gyro-only). Board/IMU convention follows the existing
// code: U1..U5 bottom->top. IMU0=U1 bottom, IMU1=U2 low, IMU2=U3 mid,
// IMU3=U4 high, IMU4=U5 top. Recording body_chain is unchanged.
const SPINE_CHAIN = [
  { id: "IMU4", role: "top / U5" },
  { id: "IMU3", role: "high / U4" },
  { id: "IMU2", role: "mid / U3" },
  { id: "IMU1", role: "low / U2" },
  { id: "IMU0", role: "bottom / U1" },
];

// Kinematic chain bottom->top. Each joint is the upper IMU's orientation
// relative to the segment below it (adjacent relative, gyro-only integration).
const CHAIN_BOTTOM_UP = ["IMU0", "IMU1", "IMU2", "IMU3", "IMU4"];
const BOARD_BOTTOM_UP = ["U1", "U2", "U3", "U4", "U5"];
const CHAIN_JOINTS = [
  { upper: "IMU1", lower: "IMU0" },
  { upper: "IMU2", lower: "IMU1" },
  { upper: "IMU3", lower: "IMU2" },
  { upper: "IMU4", lower: "IMU3" },
];
// Focus joint drives the numeric twist readout (keeps the old IMU1<->IMU2 pair).
const FOCUS_JOINT_INDEX = 1;

const SEGMENT_COLORS = [0x2fbf71, 0x57c98a, 0x5da8ff, 0x8f8bff, 0xc46bff];

const AXES = {
  x: new THREE.Vector3(1, 0, 0),
  y: new THREE.Vector3(0, 1, 0),
  z: new THREE.Vector3(0, 0, 1),
};
const AXIS_MAPS = {
  xyz: [0, 1, 2],
  yxz: [1, 0, 2],
  zxy: [2, 0, 1],
  xzy: [0, 2, 1],
  yzx: [1, 2, 0],
  zyx: [2, 1, 0],
};
const COLORS = { pillar: 0xf2b84b };
const AUTO_ZERO_STILL_DPS = 1.2;
const AUTO_ZERO_HOLD_MS = 650;
const AUTO_ZERO_MAX_ABS_DEG = 22;
const AUTO_ZERO_RATE_IDLE = 1.25;
const AUTO_ZERO_RATE_NEUTRAL_PHASE = 4.0;

const SEGMENT_GAP = 0.62;
const SEGMENT_HEIGHT = 0.34;
const BASE_Y = -1.0;

const latest = new Map();
const cards = new Map();
let axisName = "z";
let axisMapName = "xyz";
let mode = "full3d";
let sign = 1;
let paused = false;
let autoZeroEnabled = true;
let biasCollector = null;
let sampleCount = 0;
let lastFrameT = null;
const qIdentity = new THREE.Quaternion();

// Per-joint integration state.
const joints = CHAIN_JOINTS.map((cfg, index) => ({
  ...cfg,
  index,
  qTarget: new THREE.Quaternion(),
  qRender: new THREE.Quaternion(),
  twistDeg: 0,
  bias: new THREE.Vector3(),
  relGyro: new THREE.Vector3(),
  relAxisDps: 0,
  relAxisBiasDps: 0,
  relGyroMagDps: 0,
  autoZeroStillMs: 0,
  autoZeroState: "auto on",
  pivot: null,
}));

const canvas = document.querySelector("#scene");
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.setClearColor(0x101114, 1);

const scene = new THREE.Scene();
scene.fog = new THREE.Fog(0x101114, 10, 20);

const camera = new THREE.PerspectiveCamera(46, 1, 0.1, 100);
camera.position.set(5.0, 3.4, 7.8);
camera.lookAt(0, 0.35, 0);

const hemi = new THREE.HemisphereLight(0xffffff, 0x252935, 2.2);
scene.add(hemi);

const key = new THREE.DirectionalLight(0xffffff, 2.4);
key.position.set(3, 5, 4);
scene.add(key);

const fill = new THREE.DirectionalLight(0x85b7ff, 1.0);
fill.position.set(-4, 2, -2);
scene.add(fill);

const grid = new THREE.GridHelper(8, 20, 0x565b64, 0x252a31);
grid.position.y = -1.4;
scene.add(grid);

// Build the 5-segment chain bottom->top as nested pivots.
const chainRoot = new THREE.Group();
scene.add(chainRoot);

const seg0 = new THREE.Group();
seg0.position.y = BASE_Y;
chainRoot.add(seg0);
seg0.add(makeSegment(SEGMENT_COLORS[0]));
addSegmentLabel(seg0, `${CHAIN_BOTTOM_UP[0]} ${BOARD_BOTTOM_UP[0]}`, SEGMENT_COLORS[0]);

let parentNode = seg0;
joints.forEach((joint, i) => {
  const pivot = new THREE.Group();
  pivot.position.y = SEGMENT_GAP;
  parentNode.add(pivot);

  const rod = new THREE.Mesh(
    new THREE.CylinderGeometry(0.06, 0.06, SEGMENT_GAP, 16),
    new THREE.MeshStandardMaterial({ color: COLORS.pillar, roughness: 0.4, metalness: 0.05 }),
  );
  rod.position.y = -SEGMENT_GAP / 2;
  pivot.add(rod);

  pivot.add(makeSegment(SEGMENT_COLORS[i + 1]));
  addSegmentLabel(pivot, `${CHAIN_BOTTOM_UP[i + 1]} ${BOARD_BOTTOM_UP[i + 1]}`, SEGMENT_COLORS[i + 1]);
  joint.pivot = pivot;
  parentNode = pivot;
});

const serialState = document.querySelector("#serialState");
const bridgeMessage = document.querySelector("#bridgeMessage");
const twistDegEl = document.querySelector("#twistDeg");
const relGyroEl = document.querySelector("#relGyro");
const biasValueEl = document.querySelector("#biasValue");
const pairCountEl = document.querySelector("#pairCount");
const zeroStatusEl = document.querySelector("#zeroStatus");
const needleEl = document.querySelector("#needle");
const readout = document.querySelector("#readout");
const biasButton = document.querySelector("#biasButton");
const tareButton = document.querySelector("#tareButton");
const autoZeroButton = document.querySelector("#autoZeroButton");
const pauseButton = document.querySelector("#pauseButton");
const signButton = document.querySelector("#signButton");
const mapSelect = document.querySelector("#mapSelect");
const startTestButton = document.querySelector("#startTestButton");
const abortTestButton = document.querySelector("#abortTestButton");
const testStateEl = document.querySelector("#testState");
const testPhaseEl = document.querySelector("#testPhase");
const testTimerEl = document.querySelector("#testTimer");
const testProgressEl = document.querySelector("#testProgress");
const recordPathEl = document.querySelector("#recordPath");
const staticDurationEl = document.querySelector("#staticDuration");
const startStaticButton = document.querySelector("#startStaticButton");

const RECORD_PHASES = [
  { name: "neutral", label: "HOLD NEUTRAL", duration: 8 },
  { name: "move", label: "MOVE SPINE", duration: 20 },
  { name: "return_neutral", label: "RETURN NEUTRAL", duration: 12 },
];
const PREP_SECONDS = 3;
let activeTest = null;
let audioContext = null;

buildReadoutCards();

for (const button of document.querySelectorAll(".mode-button")) {
  button.addEventListener("click", () => {
    mode = button.dataset.mode;
    document.querySelectorAll(".mode-button").forEach((item) => item.classList.toggle("active", item === button));
    resetIntegration();
  });
}

for (const button of document.querySelectorAll(".axis-button")) {
  button.addEventListener("click", () => {
    axisName = button.dataset.axis;
    document.querySelectorAll(".axis-button").forEach((item) => item.classList.toggle("active", item === button));
    resetIntegration();
  });
}

mapSelect.addEventListener("change", () => {
  axisMapName = mapSelect.value;
  resetIntegration();
});

signButton.addEventListener("click", () => {
  sign *= -1;
  signButton.textContent = sign > 0 ? "Sign +" : "Sign -";
  resetIntegration();
});

biasButton.addEventListener("click", () => {
  biasCollector = {
    startedAt: performance.now(),
    values: joints.map(() => []),
  };
  biasButton.textContent = "Hold still";
});

tareButton.addEventListener("click", () => {
  resetIntegration();
});

autoZeroButton.addEventListener("click", () => {
  autoZeroEnabled = !autoZeroEnabled;
  autoZeroButton.textContent = autoZeroEnabled ? "Auto Zero ON" : "Auto Zero OFF";
  autoZeroButton.classList.toggle("active", autoZeroEnabled);
  for (const joint of joints) {
    joint.autoZeroStillMs = 0;
    joint.autoZeroState = autoZeroEnabled ? "auto on" : "off";
  }
});

pauseButton.addEventListener("click", () => {
  paused = !paused;
  pauseButton.textContent = paused ? "Resume" : "Pause";
});

startTestButton.addEventListener("click", () => {
  startProtocolTest();
});

startStaticButton.addEventListener("click", () => {
  startStaticHold();
});

abortTestButton.addEventListener("click", () => {
  abortProtocolTest();
});

const source = new EventSource("/events");
source.onmessage = (event) => {
  const payload = JSON.parse(event.data);
  if (payload.type === "status") {
    setBridgeStatus(payload);
    return;
  }
  if (payload.type === "sample") {
    latest.set(payload.imu, payload);
    updateCard(payload);
    sampleCount += 1;
  }
};

source.onerror = () => {
  serialState.textContent = "offline";
  serialState.className = "state-pill error";
  bridgeMessage.textContent = "SSE disconnected";
};

function integrateChain(dt) {
  const axis = AXES[axisName];
  for (const joint of joints) {
    const up = latest.get(joint.upper);
    const lo = latest.get(joint.lower);
    if (!up || !lo) continue;

    const relRaw = new THREE.Vector3(
      up.gx_dps - lo.gx_dps,
      up.gy_dps - lo.gy_dps,
      up.gz_dps - lo.gz_dps,
    );
    collectBias(joint, relRaw);

    const mappedRaw = mapGyroVector(relRaw);
    const mappedBias = mapGyroVector(joint.bias);
    joint.relGyro.copy(mapGyroVector(relRaw.clone().sub(joint.bias)));
    if (mode === "full3d") {
      joint.relGyro.y *= sign;
    }
    joint.relGyroMagDps = joint.relGyro.length();
    joint.relAxisBiasDps = mappedBias.dot(axis) * sign;
    joint.relAxisDps = mappedRaw.dot(axis) * sign - joint.relAxisBiasDps;

    if (paused || dt <= 0) continue;
    if (mode === "full3d") {
      integrateQuaternion(joint.qTarget, joint.relGyro, dt);
      joint.twistDeg = twistFromQuaternion(joint.qTarget, axis);
    } else {
      joint.twistDeg += joint.relAxisDps * dt;
    }
    applyAutoZero(joint, dt, axis);
  }
}

function mapGyroVector(raw) {
  const order = AXIS_MAPS[axisMapName] || AXIS_MAPS.xyz;
  const values = [raw.x, raw.y, raw.z];
  return new THREE.Vector3(values[order[0]], values[order[1]], values[order[2]]);
}

function integrateQuaternion(q, omegaDps, dt) {
  const omegaRad = omegaDps.clone().multiplyScalar(Math.PI / 180);
  const angle = omegaRad.length() * dt;
  if (angle < 1e-9) return;
  const axis = omegaRad.normalize();
  const delta = new THREE.Quaternion().setFromAxisAngle(axis, angle);
  q.multiply(delta).normalize();
}

function twistFromQuaternion(q, axis) {
  const v = new THREE.Vector3(q.x, q.y, q.z);
  const projected = axis.clone().multiplyScalar(v.dot(axis));
  const qTwist = new THREE.Quaternion(projected.x, projected.y, projected.z, q.w);
  const normSq = qTwist.x * qTwist.x + qTwist.y * qTwist.y + qTwist.z * qTwist.z + qTwist.w * qTwist.w;
  if (normSq < 1e-12) return 0;
  qTwist.normalize();
  const signed = new THREE.Vector3(qTwist.x, qTwist.y, qTwist.z).dot(axis);
  let angle = 2 * Math.atan2(signed, qTwist.w);
  angle = ((angle + Math.PI) % (2 * Math.PI)) - Math.PI;
  return THREE.MathUtils.radToDeg(angle);
}

function collectBias(joint, relRaw) {
  if (!biasCollector) return;
  biasCollector.values[joint.index].push(relRaw.clone());
  const elapsed = (performance.now() - biasCollector.startedAt) / 1000;
  const remaining = Math.max(0, 3 - elapsed);
  biasButton.textContent = `Hold ${remaining.toFixed(1)}s`;
  if (elapsed >= 3) {
    for (const j of joints) {
      const vals = biasCollector.values[j.index];
      if (vals.length) j.bias.copy(averageVector(vals));
    }
    biasCollector = null;
    resetIntegration();
    biasButton.textContent = "Bias 3s";
  }
}

function resetIntegration() {
  for (const joint of joints) {
    joint.qTarget.identity();
    joint.qRender.identity();
    joint.twistDeg = 0;
    joint.relAxisDps = 0;
    joint.autoZeroStillMs = 0;
  }
}

function applyAutoZero(joint, dt, axis) {
  if (!autoZeroEnabled) {
    joint.autoZeroState = "off";
    joint.autoZeroStillMs = 0;
    return;
  }
  if (paused || dt <= 0) {
    joint.autoZeroState = paused ? "paused" : "auto on";
    return;
  }

  const neutralPhase = isNeutralProtocolPhase();
  const still = joint.relGyroMagDps <= AUTO_ZERO_STILL_DPS;
  const maxAbs = neutralPhase ? 95 : AUTO_ZERO_MAX_ABS_DEG;
  const nearNeutral = Math.abs(joint.twistDeg) <= maxAbs;

  if (!still || !nearNeutral) {
    joint.autoZeroStillMs = 0;
    joint.autoZeroState = still ? "hold angle" : "moving";
    return;
  }

  joint.autoZeroStillMs += dt * 1000;
  if (joint.autoZeroStillMs < AUTO_ZERO_HOLD_MS) {
    joint.autoZeroState = "still";
    return;
  }

  const rate = neutralPhase ? AUTO_ZERO_RATE_NEUTRAL_PHASE : AUTO_ZERO_RATE_IDLE;
  const alpha = 1 - Math.exp(-rate * dt);
  if (mode === "full3d") {
    joint.qTarget.slerp(qIdentity, alpha).normalize();
    joint.twistDeg = twistFromQuaternion(joint.qTarget, axis);
  } else {
    joint.twistDeg = THREE.MathUtils.lerp(joint.twistDeg, 0, alpha);
  }
  joint.autoZeroState = neutralPhase ? "zeroing neutral" : "zeroing";
}

function isNeutralProtocolPhase() {
  const label = activeTest?.phaseLabel || "";
  return label === "GET READY" || label === "HOLD NEUTRAL" || label === "RETURN NEUTRAL";
}

function averageVector(values) {
  if (!values.length) return new THREE.Vector3();
  const total = new THREE.Vector3();
  for (const value of values) {
    total.add(value);
  }
  return total.multiplyScalar(1 / values.length);
}

function setBridgeStatus(payload) {
  serialState.textContent = payload.serial || "status";
  serialState.className = `state-pill ${payload.serial || ""}`;
  bridgeMessage.textContent = payload.message || "";
}

async function startProtocolTest() {
  if (activeTest) return;
  await unlockAudio();
  resetIntegration();
  activeTest = {
    aborted: false,
    recording: false,
    recordStatus: null,
  };
  setTestUi("active", "GET READY", PREP_SECONDS, 0, "Preparing");
  startTestButton.disabled = true;
  startStaticButton.disabled = true;
  abortTestButton.disabled = false;
  recordPathEl.textContent = "Recording starts after get ready";
  beep(660, 0.12);

  try {
    const readyOk = await runCountdown("GET READY", PREP_SECONDS, 0, PREP_SECONDS);
    if (!readyOk) return;

    const phases = buildMarkerPhases();
    const status = await postJson("/api/record/start", {
      protocol: "spine5_u5_top_test",
      layout_preset: "spine5-u5-top",
      body_chain: [
        { role: "top", imu: "IMU4", board: "U5" },
        { role: "high", imu: "IMU3", board: "U4" },
        { role: "mid", imu: "IMU2", board: "U3" },
        { role: "low", imu: "IMU1", board: "U2" },
        { role: "bottom", imu: "IMU0", board: "U1" },
      ],
      phases,
      notes: "5IMU back-chain recording. Board order is U5 to U1 from top to bottom. Recording starts after the 3 second get-ready countdown. Use 0-8s for bias/tare.",
    });
    activeTest.recording = true;
    activeTest.recordStatus = status;
    recordPathEl.textContent = shortRecordText(status);
    resetIntegration();
    beep(880, 0.14);

    const totalRecordSeconds = RECORD_PHASES.reduce((sum, phase) => sum + phase.duration, 0);
    let elapsed = 0;
    for (const phase of RECORD_PHASES) {
      const ok = await runCountdown(phase.label, phase.duration, elapsed, totalRecordSeconds);
      if (!ok) return;
      elapsed += phase.duration;
      beep(880, 0.1);
    }

    const stopped = await postJson("/api/record/stop", { reason: "completed" });
    activeTest.recording = false;
    setTestUi("done", "DONE", 0, 100, "Done");
    recordPathEl.textContent = shortRecordText(stopped);
    beep(1040, 0.14);
    setTimeout(() => beep(1320, 0.14), 170);
  } catch (error) {
    setTestUi("abort", "ERROR", 0, 0, "error");
    recordPathEl.textContent = String(error.message || error);
    if (activeTest?.recording) {
      await safeStopRecording("error");
    }
  } finally {
    startTestButton.disabled = false;
    startStaticButton.disabled = false;
    abortTestButton.disabled = true;
    activeTest = null;
  }
}

async function startStaticHold() {
  if (activeTest) return;
  await unlockAudio();
  resetIntegration();
  const durationRaw = Number(staticDurationEl?.value || 120);
  const holdSeconds = Number.isFinite(durationRaw) && durationRaw > 0 ? durationRaw : 120;
  activeTest = {
    aborted: false,
    recording: false,
    recordStatus: null,
  };
  setTestUi("active", "GET READY", PREP_SECONDS, 0, "Preparing");
  startTestButton.disabled = true;
  startStaticButton.disabled = true;
  abortTestButton.disabled = false;
  recordPathEl.textContent = "Lay the chain flat, hands OFF. Recording starts after get ready.";
  beep(660, 0.12);

  try {
    const readyOk = await runCountdown("GET READY (HANDS OFF)", PREP_SECONDS, 0, PREP_SECONDS);
    if (!readyOk) return;

    const phases = [{ name: "static", label: "STATIC HOLD", start_s: 0, end_s: holdSeconds }];
    const status = await postJson("/api/record/start", {
      protocol: "static_hold_5imu",
      layout_preset: "spine5-u5-top",
      body_chain: [
        { role: "top", imu: "IMU4", board: "U5" },
        { role: "high", imu: "IMU3", board: "U4" },
        { role: "mid", imu: "IMU2", board: "U3" },
        { role: "low", imu: "IMU1", board: "U2" },
        { role: "bottom", imu: "IMU0", board: "U1" },
      ],
      phases,
      notes:
        `Static-hold drift test, ${holdSeconds}s. Lay the chain flat on the table, hands off the structure, cables and table. ` +
        "five_imu_fusion --auto-markers uses 0-8s as bias/tare and the remainder as the drift window. " +
        "Do NOT enable Auto Zero / closed-loop when measuring drift.",
    });
    activeTest.recording = true;
    activeTest.recordStatus = status;
    recordPathEl.textContent = shortRecordText(status);
    resetIntegration();
    beep(880, 0.14);

    const ok = await runCountdown("STATIC HOLD", holdSeconds, 0, holdSeconds);
    if (!ok) return;

    const stopped = await postJson("/api/record/stop", { reason: "completed" });
    activeTest.recording = false;
    setTestUi("done", "DONE", 0, 100, "Done");
    recordPathEl.textContent = shortRecordText(stopped);
    beep(1040, 0.14);
    setTimeout(() => beep(1320, 0.14), 170);
  } catch (error) {
    setTestUi("abort", "ERROR", 0, 0, "error");
    recordPathEl.textContent = String(error.message || error);
    if (activeTest?.recording) {
      await safeStopRecording("error");
    }
  } finally {
    startTestButton.disabled = false;
    startStaticButton.disabled = false;
    abortTestButton.disabled = true;
    activeTest = null;
  }
}

function abortProtocolTest() {
  if (!activeTest) return;
  activeTest.aborted = true;
  safeStopRecording("aborted");
  setTestUi("abort", "ABORTED", 0, 0, "aborted");
  recordPathEl.textContent = "Recording aborted";
  beep(220, 0.18);
}

async function runCountdown(label, duration, elapsedBefore, totalDuration) {
  const started = performance.now();
  if (activeTest) {
    activeTest.phaseLabel = label;
  }
  while (true) {
    if (!activeTest || activeTest.aborted) return false;
    const elapsed = (performance.now() - started) / 1000;
    const remaining = Math.max(0, duration - elapsed);
    const phaseProgress = Math.min(1, elapsed / duration);
    const totalProgress = totalDuration > 0
      ? ((elapsedBefore + Math.min(duration, elapsed)) / totalDuration) * 100
      : phaseProgress * 100;
    setTestUi("active", label, remaining, totalProgress, "active");
    if (remaining <= 0) return true;
    await sleep(80);
  }
}

function buildMarkerPhases() {
  let cursor = 0;
  return RECORD_PHASES.map((phase) => {
    const out = {
      name: phase.name,
      label: phase.label,
      start_s: cursor,
      end_s: cursor + phase.duration,
    };
    cursor += phase.duration;
    return out;
  });
}

function setTestUi(state, phase, remaining, progressPercent, stateText) {
  testStateEl.textContent = stateText;
  testStateEl.className = `test-state ${state}`;
  testPhaseEl.textContent = phase;
  testTimerEl.textContent = remaining > 0 ? `${remaining.toFixed(1)}s` : "--";
  testProgressEl.style.width = `${THREE.MathUtils.clamp(progressPercent, 0, 100)}%`;
}

async function safeStopRecording(reason) {
  try {
    await postJson("/api/record/stop", { reason });
  } catch {
    // Best effort: the page may be closing or the bridge may be gone.
  }
  if (activeTest) activeTest.recording = false;
}

async function postJson(path, payload) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload || {}),
  });
  const data = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(data.error || `${path} failed with ${response.status}`);
  }
  return data;
}

function shortRecordText(status) {
  if (!status?.log_path) return "Recording active";
  const log = status.log_path.split(/[\\/]/).pop();
  const markers = status.marker_path ? status.marker_path.split(/[\\/]/).pop() : "markers";
  const lines = Number(status.line_count || 0);
  return `${log} | ${markers} | ${lines} lines`;
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function unlockAudio() {
  if (!audioContext) {
    audioContext = new AudioContext();
  }
  if (audioContext.state === "suspended") {
    await audioContext.resume();
  }
}

function beep(freq = 880, duration = 0.12) {
  if (!audioContext) return;
  const osc = audioContext.createOscillator();
  const gain = audioContext.createGain();
  osc.frequency.value = freq;
  osc.type = "sine";
  gain.gain.setValueAtTime(0.0001, audioContext.currentTime);
  gain.gain.exponentialRampToValueAtTime(0.12, audioContext.currentTime + 0.01);
  gain.gain.exponentialRampToValueAtTime(0.0001, audioContext.currentTime + duration);
  osc.connect(gain);
  gain.connect(audioContext.destination);
  osc.start();
  osc.stop(audioContext.currentTime + duration + 0.02);
}

function buildReadoutCards() {
  readout.replaceChildren(
    ...SPINE_CHAIN.map(({ id, role }) => {
      const card = document.createElement("article");
      card.className = "imu-card";
      card.dataset.imu = id;
      card.innerHTML = `
        <div class="card-head">
          <strong>${id}</strong>
          <span class="live">LOST</span>
        </div>
        <dl>
          <div><dt>role</dt><dd data-field="role">${role}</dd></div>
          <div><dt>roll</dt><dd data-field="roll">--</dd></div>
          <div><dt>pitch</dt><dd data-field="pitch">--</dd></div>
          <div><dt>gyro z</dt><dd data-field="gz">--</dd></div>
        </dl>
      `;
      cards.set(id, card);
      return card;
    }),
  );
}

function updateCard(sample) {
  const card = cards.get(sample.imu);
  if (!card) return;
  card.dataset.lastSeen = String(Date.now());
  const live = card.querySelector(".live");
  live.textContent = "LIVE";
  live.className = "live on";
  card.querySelector('[data-field="roll"]').textContent = sample.roll_deg.toFixed(1);
  card.querySelector('[data-field="pitch"]').textContent = sample.pitch_deg.toFixed(1);
  card.querySelector('[data-field="gz"]').textContent = sample.gz_dps.toFixed(2);
}

setInterval(() => {
  const now = Date.now();
  for (const card of cards.values()) {
    const live = card.querySelector(".live");
    const lastSeen = Number(card.dataset.lastSeen || 0);
    const isLive = now - lastSeen < 1100;
    live.textContent = isLive ? "LIVE" : "LOST";
    live.className = `live ${isLive ? "on" : "off"}`;
  }
}, 250);

function makeSegment(color) {
  const group = new THREE.Group();
  const body = new THREE.Mesh(
    new THREE.BoxGeometry(1.15, SEGMENT_HEIGHT, 1.15),
    new THREE.MeshStandardMaterial({ color, roughness: 0.42, metalness: 0.04 }),
  );
  group.add(body);
  group.add(
    new THREE.LineSegments(
      new THREE.EdgesGeometry(body.geometry),
      new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.4 }),
    ),
  );
  const pointer = new THREE.Mesh(
    new THREE.BoxGeometry(0.1, 0.07, 0.7),
    new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0x111111, roughness: 0.28 }),
  );
  pointer.position.set(0, SEGMENT_HEIGHT * 0.5, -0.42);
  group.add(pointer);
  return group;
}

function addSegmentLabel(node, text, color) {
  const label = makeLabel(text, color);
  label.position.set(-1.4, 0, 0);
  label.scale.set(1.5, 0.26, 1);
  node.add(label);
}

function makeLabel(text, color) {
  const canvas2d = document.createElement("canvas");
  canvas2d.width = 768;
  canvas2d.height = 128;
  const ctx = canvas2d.getContext("2d");
  ctx.clearRect(0, 0, canvas2d.width, canvas2d.height);
  ctx.fillStyle = "#f4f4f1";
  ctx.font = "700 44px system-ui, sans-serif";
  ctx.textAlign = "left";
  ctx.textBaseline = "middle";
  ctx.fillText(text, 20, 58);
  ctx.fillStyle = `#${color.toString(16).padStart(6, "0")}`;
  ctx.fillRect(20, 100, 260, 8);
  const texture = new THREE.CanvasTexture(canvas2d);
  const material = new THREE.SpriteMaterial({ map: texture, transparent: true });
  const sprite = new THREE.Sprite(material);
  sprite.scale.set(2.3, 0.38, 1);
  return sprite;
}

function resize() {
  const width = window.innerWidth;
  const height = window.innerHeight;
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  const mobile = width <= 760;
  camera.fov = mobile ? 60 : 46;
  camera.position.set(mobile ? 4.6 : 5.0, mobile ? 3.6 : 3.4, mobile ? 9.2 : 7.8);
  camera.lookAt(0, 0.35, 0);
  chainRoot.scale.setScalar(mobile ? 0.82 : 0.95);
  camera.updateProjectionMatrix();
}

window.addEventListener("resize", resize);
resize();

function animate() {
  requestAnimationFrame(animate);
  const now = performance.now();
  const dt = lastFrameT == null ? 0 : Math.min(0.05, (now - lastFrameT) / 1000);
  lastFrameT = now;

  integrateChain(dt);

  for (const joint of joints) {
    if (mode === "full3d") {
      joint.qRender.slerp(joint.qTarget, 0.25);
    } else {
      const q = new THREE.Quaternion().setFromAxisAngle(
        AXES[axisName],
        THREE.MathUtils.degToRad(joint.twistDeg),
      );
      joint.qRender.slerp(q, 0.25);
    }
    if (joint.pivot) joint.pivot.quaternion.copy(joint.qRender);
  }

  const focus = joints[FOCUS_JOINT_INDEX];
  twistDegEl.textContent = focus.twistDeg.toFixed(1);
  relGyroEl.textContent = mode === "full3d"
    ? `${focus.relGyroMagDps.toFixed(2)} dps`
    : `${focus.relAxisDps.toFixed(2)} dps`;
  biasValueEl.textContent = `${focus.relAxisBiasDps.toFixed(2)} dps`;
  pairCountEl.textContent = String(sampleCount);
  zeroStatusEl.textContent = focus.autoZeroState;
  const needlePercent = THREE.MathUtils.clamp(50 + (focus.twistDeg / 60) * 50, 0, 100);
  needleEl.style.left = `${needlePercent}%`;

  renderer.render(scene, camera);
}

animate();
