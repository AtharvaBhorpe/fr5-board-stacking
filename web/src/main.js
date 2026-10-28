import "./style.css";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import URDFLoader from "urdf-loader";
const $ = (s) => document.querySelector(s);
let token =
  location.hash.slice(1) || localStorage.getItem("workcell-token") || "";
if (location.hash) {
  history.replaceState(null, "", location.pathname);
  localStorage.setItem("workcell-token", token);
}
let connected = false,
  lastEvent = null,
  state = null,
  robot = null,
  loadingRobot = false,
  voiceEnabled = false,
  audioContext = null,
  speechEpoch = 0,
  speechChain = Promise.resolve(),
  speaking = null;
let lastPoll = 0;
function notice(text) {
  $("#notice").textContent = text;
  $("#notice").hidden = false;
  clearTimeout(notice.timer);
  notice.timer = setTimeout(() => ($("#notice").hidden = true), 6500);
}
async function api(path, body) {
  const r = await fetch("/api/" + path, {
    method: body === undefined ? "GET" : "POST",
    headers: {
      "X-Workcell-Token": token,
      ...(body === undefined ? {} : { "Content-Type": "application/json" }),
    },
    body: body === undefined ? undefined : JSON.stringify(body),
    cache: "no-store",
    signal: AbortSignal.timeout(body ? 45000 : 5000),
  });
  if (!r.ok) {
    let data = await r.json().catch(() => ({}));
    throw new Error(data.error || `Request failed (${r.status})`);
  }
  return r;
}
const json = async (path, body) => (await api(path, body)).json();
async function pair() {
  if (!token) {
    try {
      token = (await (await fetch("/api/session")).json()).token || "";
    } catch {}
  }
  if (!token) {
    $("#login-dialog").showModal();
    return false;
  }
  localStorage.setItem("workcell-token", token);
  return true;
}
$("#login-form").onsubmit = (e) => {
  e.preventDefault();
  token = $("#token-input").value.trim();
  localStorage.setItem("workcell-token", token);
  $("#login-dialog").close();
  poll();
};
function stopSpeech() {
  speechEpoch++;
  if (speaking) {
    try {
      speaking.stop();
    } catch {}
    speaking = null;
  }
  speechChain = Promise.resolve();
}
async function unlockAudio() {
  audioContext ||= new AudioContext();
  await audioContext.resume();
  voiceEnabled = true;
  $("#sound").textContent = "Voice replies on · Tap to mute";
}
$("#sound").onclick = () => {
  if (voiceEnabled) {
    voiceEnabled = false;
    stopSpeech();
    $("#sound").textContent = "Enable voice replies";
  } else unlockAudio().catch((e) => notice(e.message));
};
function speak(text) {
  if (!voiceEnabled) return;
  const epoch = speechEpoch;
  speechChain = speechChain
    .then(async () => {
      if (epoch !== speechEpoch) return;
      const result = await json("speech", { text });
      if (epoch !== speechEpoch) return;
      const bytes = Uint8Array.from(atob(result.wav), (c) => c.charCodeAt(0));
      const buffer = await audioContext.decodeAudioData(bytes.buffer);
      if (epoch !== speechEpoch) return;
      await new Promise((resolve) => {
        const source = audioContext.createBufferSource();
        source.buffer = buffer;
        source.connect(audioContext.destination);
        source.onended = resolve;
        speaking = source;
        source.start();
      });
    })
    .catch((e) => notice("Voice reply: " + e.message));
}
async function command(text) {
  if (voiceEnabled) await audioContext.resume();
  await json("chat", { text });
}
$("#chat-form").onsubmit = async (e) => {
  e.preventDefault();
  const text = $("#chat-input").value.trim();
  if (!text) return;
  try {
    await command(text);
    $("#chat-input").value = "";
  } catch (e) {
    notice(e.message);
  }
};
$("#chat-input").onkeydown = (e) => {
  if (e.key === "Enter" && !e.shiftKey) {
    e.preventDefault();
    $("#chat-form").requestSubmit();
  }
};
for (const b of document.querySelectorAll("[data-variant]"))
  b.onclick = () =>
    command(`pick variant ${b.dataset.variant.at(-1).toUpperCase()}`).catch(
      (e) => notice(e.message),
    );
$("#sort").onclick = () =>
  ["a", "b", "c"].forEach((v) =>
    command(`move all variant ${v}`).catch((e) => notice(e.message)));
async function control(action) {
  if (action === "stop" || action === "clear") {
    stopSpeech();
    held = false;
    if (capturing) await stopTalk(null, true);
  }
  await json("control", { action });
}
$("#stop").onclick = () => control("stop").catch((e) => notice(e.message));
$("#confirm").onclick = () =>
  control("confirm").catch((e) => notice(e.message));
$("#clear").onclick = () => control("clear").catch((e) => notice(e.message));
for (const b of document.querySelectorAll("[data-reset]"))
  b.onclick = () => control(b.dataset.reset).catch((e) => notice(e.message));
let pairedLink = "";
$("#pair").onclick = async () => {
  try {
    const c = await json("connection");
    pairedLink = c.url + "/#" + token;
    $("#pair-url").textContent = c.url;
    $("#pair-dialog").showModal();
  } catch (e) {
    notice(e.message);
  }
};
$("#copy-link").onclick = () =>
  navigator.clipboard
    .writeText(pairedLink)
    .then(() => notice("Paired link copied. Treat it as a control credential."))
    .catch(() => notice("Clipboard unavailable"));
let installPrompt;
window.addEventListener("beforeinstallprompt", (e) => {
  e.preventDefault();
  installPrompt = e;
  $("#install").hidden = false;
});
$("#install").onclick = async () => {
  await installPrompt?.prompt();
  $("#install").hidden = true;
};
if ("serviceWorker" in navigator)
  navigator.serviceWorker
    .register("/sw.js")
    .catch((e) => notice("Offline shell unavailable: " + e.message));

const viewport = $("#viewport"),
  scene = new THREE.Scene();
scene.background = new THREE.Color("#e1e9ef");
const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 50);
camera.up.set(0, 0, 1);
let renderer;
try {
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  viewport.appendChild(renderer.domElement);
} catch (e) {
  notice("3D graphics unavailable: " + e.message);
}
const orbit = renderer ? new OrbitControls(camera, renderer.domElement) : null;
function home() {
  camera.position.set(1.2, -1.7, 1.6);
  orbit?.target.set(-0.25, 0.2, 0.25);
  orbit?.update();
}
home();
$("#home-view").onclick = home;
$("#top-view").onclick = () => {
  camera.position.set(-0.2, -0.3, 1.9);
  orbit?.target.set(-0.2, -0.3, 0);
  orbit?.update();
};
scene.add(new THREE.HemisphereLight(0xffffff, 0x708090, 2.5));
const light = new THREE.DirectionalLight(0xffffff, 3);
light.position.set(2, -3, 4);
scene.add(light);
const grid = new THREE.GridHelper(2, 20, 0xaabcc8, 0xc5d3dc);
grid.rotation.x = Math.PI / 2;
grid.position.z = -0.105;
scene.add(grid);
const base = new THREE.Mesh(
  new THREE.BoxGeometry(0.25, 0.25, 0.02),
  new THREE.MeshStandardMaterial({ color: 0xb8c7d2, roughness: 0.8 }),
);
base.position.z = -0.011;
scene.add(base);
const boardsGroup = new THREE.Group();
scene.add(boardsGroup);
new ResizeObserver(() => {
  if (!renderer) return;
  const r = viewport.getBoundingClientRect();
  renderer.setSize(r.width, r.height);
  camera.aspect = r.width / r.height;
  camera.updateProjectionMatrix();
}).observe(viewport);
function draw() {
  requestAnimationFrame(draw);
  orbit?.update();
  if (renderer) renderer.render(scene, camera);
}
draw();
async function loadRobot() {
  if (loadingRobot || robot) return;
  loadingRobot = true;
  try {
    const text = await (await api("robot")).text();
    if (!text) throw new Error("Robot description not ready");
    const loader = new URDFLoader();
    loader.packages = (name) => "/api/mesh/" + name;
    loader.loadMeshCb = (path, manager, material, done) => {
      fetch(path, { headers: { "X-Workcell-Token": token } })
        .then((r) => {
          if (!r.ok) throw new Error("Mesh unavailable: " + path);
          return r.arrayBuffer();
        })
        .then((buffer) => {
          const geometry = new STLLoader().parse(buffer);
          const mesh = new THREE.Mesh(
            geometry,
            new THREE.MeshStandardMaterial({
              color: 0xe4e8ec,
              metalness: 0.25,
              roughness: 0.55,
            }),
          );
          done(mesh);
        })
        .catch((e) => {
          done(null, e);
          notice(e.message);
        });
    };
    robot = loader.parse(text);
    scene.add(robot);
    window.workcellRobot = robot;
  } catch (e) {
    $("#robot-label").textContent = e.message;
  } finally {
    loadingRobot = false;
  }
}
function drawBoards(vision) {
  for (const c of [...boardsGroup.children]) {
    boardsGroup.remove(c);
    c.geometry.dispose();
    c.material.dispose();
  }
  const colors = { VARa: 0x7199c0, VARb: 0x9c86bc, VARc: 0x72a79b };
  for (const [key, dest] of [
    ["boards", false],
    ["destinations", true],
  ])
    for (const b of vision[key] || []) {
      if (!Number.isFinite(b.x) || !Number.isFinite(b.y)) continue;
      const sizes = {
        VARa: [0.21, 0.3],
        VARb: [0.15, 0.21],
        VARc: [0.105, 0.21],
      };
      const [w, h] = b.size_mm
        ? b.size_mm.map((v) => v / 1000)
        : sizes[b.variant] || [0.1, 0.1];
      const mesh = new THREE.Mesh(
        new THREE.BoxGeometry(w, h, 0.004),
        new THREE.MeshStandardMaterial({
          color: colors[b.variant] || 0xbbbbbb,
          transparent: dest,
          opacity: dest ? 0.3 : 0.9,
          roughness: 0.9,
        }),
      );
      mesh.position.set(b.x, b.y, -0.097);
      mesh.rotation.z = b.yaw || 0;
      boardsGroup.add(mesh);
    }
}
function appendEvent(e, play) {
  if (e.kind === "log") return;
  if (e.phase === "clear") $("#messages").replaceChildren();
  $(".chat-empty")?.remove();
  const div = document.createElement("div");
  div.className = "message " + (e.role || "system");
  const role = document.createElement("div");
  role.className = "role";
  role.textContent =
    e.role === "assistant" ? "Workcell" : e.role === "user" ? "You" : "System";
  const text = document.createElement("div");
  text.textContent = e.text;
  div.append(role, text);
  $("#messages").append(div);
  while ($("#messages").children.length > 200)
    $("#messages").firstElementChild.remove();
  $("#messages").scrollTop = $("#messages").scrollHeight;
  $("#chat-status").textContent = e.phase || "Ready";
  if (play && e.role === "assistant") speak(e.text);
}
let visionStamp = 0;
function update(s) {
  state = s;
  connected = true;
  lastPoll = Date.now();
  $("#connection").textContent = s.robot_fresh
    ? "Connected"
    : "Robot state stale";
  $("#connection-dot").classList.toggle("live", s.robot_fresh);
  $("#mode").textContent =
    s.panel.hardware === "SIMULATION"
      ? "Simulation · MoveIt mock hardware"
      : "Read-only · " + s.panel.hardware;
  $("#robot-label").textContent = s.robot_fresh
    ? "Live joint positions"
    : "Joint data stale";
  $("#controller").textContent = s.panel.phase;
  $("#gripper").textContent = s.gripper;
  $("#batch").textContent = s.batch.total
    ? `${s.batch.completed} / ${s.batch.total} · ${s.batch.phase}`
    : "No active batch";
  $("#task-detail").textContent = s.panel.detail || "Ready for a task";
  $("#confirm").hidden = !s.panel.waiting;
  $("#sort").disabled = !s.robot_fresh || !s.vision_fresh || s.panel.busy;
  $("#talk").disabled = !s.model?.asr_ready;
  if (!s.model?.asr_ready && !capturing) {
    $("#voice-status").textContent = s.model?.asr_error
      ? "Speech recognition unavailable. Typed chat is available."
      : "Parakeet is loading. Typed chat is available.";
  } else if ($("#voice-status").textContent.includes("Typed chat is available")) {
    $("#voice-status").textContent = "Microphone is off";
  }
  $("#model-status").textContent = s.model?.asr_ready
    ? "Local models ready"
    : s.model?.asr_error
      ? "Speech recognition unavailable · typed chat ready"
      : s.model?.ready
        ? "Parakeet loading · typed chat ready"
        : "Model service is starting";
  for (const b of document.querySelectorAll("[data-reset]"))
    b.disabled =
      s.panel.busy ||
      ["checking", "executing", "verifying"].includes(s.batch.phase);
  for (const b of document.querySelectorAll("[data-variant]")) {
    const v = b.dataset.variant,
      n = (s.vision.boards || []).filter((x) => x.variant === v).length;
    b.disabled = !!s.blocked[v];
    b.title = s.blocked[v] || "Pick and place this variant";
    b.querySelector("small").textContent = s.vision_fresh
      ? `${n} visible · ${(s.vision.destinations || []).some((x) => x.variant === v) ? "template ready" : "no template"}`
      : "Vision stale";
  }
  $("#vision-detail").textContent = s.vision_fresh
    ? (s.vision.live ? "Live camera" : "Simulated camera scene") +
      " · " +
      (s.vision.calibrated ? "Calibrated" : "Not calibrated")
    : "No fresh camera observations";
  if (robot) {
    for (const [name, value] of Object.entries(s.joints))
      robot.setJointValue(name, value);
  } else loadRobot();
  if (s.vision.stamp !== visionStamp) {
    visionStamp = s.vision.stamp;
    drawBoards(s.vision);
  }
  const logs = s.events.filter((e) => e.kind === "log");
  $("#logs").textContent = logs.map((e) => e.text).join("\n");
  $("#log-count").textContent = logs.length + " entries";
  const initial = lastEvent === null;
  for (const e of s.events)
    if (initial || e.id > lastEvent) appendEvent(e, !initial);
  lastEvent = s.events.at(-1)?.id ?? lastEvent;
  for (const key of ["overhead", "wrist"]) {
    const available = !!s.cameras[key];
    $("#" + key).parentElement.classList.toggle("available", available);
    $("#" + key + "-status").textContent = available ? "Receiving" : "No feed";
  }
}
async function poll() {
  try {
    update(await json("state"));
  } catch (e) {
    connected = false;
    $("#connection").textContent = "Disconnected";
    $("#connection-dot").classList.remove("live");
    $("#talk").disabled = true;
    $("#sort").disabled = true;
    for (const b of document.querySelectorAll("[data-variant],[data-reset]"))
      b.disabled = true;
    $("#robot-label").textContent = "Disconnected · last pose";
    drawBoards({});
    if (/paired/.test(e.message) && !$("#login-dialog").open)
      $("#login-dialog").showModal();
  }
  setTimeout(poll, 250);
}
async function images() {
  if (connected)
    for (const key of ["overhead", "wrist"]) {
      if (!state?.cameras[key]) continue;
      try {
        const blob = await (await api("image/" + key)).blob();
        const image = $("#" + key),
          old = image.dataset.url;
        image.src = URL.createObjectURL(blob);
        image.dataset.url = image.src;
        if (old) URL.revokeObjectURL(old);
      } catch {}
    }
  setTimeout(images, 250);
}

let held = false,
  capturing = false,
  captureStream = null,
  recorder = null,
  captureSource = null,
  captureContext = null,
  frames = [],
  timer = null,
  started = 0;
async function startTalk(e) {
  e?.preventDefault();
  if (!connected || held) return;
  held = true;
  try {
    stopSpeech();
    if (voiceEnabled) await audioContext.resume();
    $("#voice-status").textContent = "Allow microphone access to speak";
    captureStream = await navigator.mediaDevices.getUserMedia({
      audio: {
        channelCount: 1,
        echoCancellation: true,
        noiseSuppression: true,
      },
    });
    if (!held) {
      captureStream.getTracks().forEach((t) => t.stop());
      return;
    }
    captureContext = new AudioContext();
    await captureContext.audioWorklet.addModule("/recorder.js");
    captureSource = captureContext.createMediaStreamSource(captureStream);
    recorder = new AudioWorkletNode(captureContext, "pcm-recorder");
    frames = [];
    recorder.port.onmessage = (e) => frames.push(e.data);
    captureSource.connect(recorder);
    recorder.connect(captureContext.destination);
    capturing = true;
    started = Date.now();
    $("#talk").classList.add("recording");
    $("#talk-label").textContent = "Listening… release to send";
    $("#voice-status").textContent = "Microphone on";
    timer = setInterval(() => {
      const seconds = (Date.now() - started) / 1000;
      $("#mic-time").textContent = seconds.toFixed(1) + "s";
      if (seconds >= 20) stopTalk();
    }, 100);
  } catch (err) {
    held = false;
    captureStream?.getTracks().forEach((t) => t.stop());
    notice(err.message);
    $("#voice-status").textContent =
      "Microphone unavailable. Check permission and HTTPS trust.";
  }
}
async function stopTalk(e, discard = false) {
  e?.preventDefault();
  held = false;
  if (!capturing) return;
  capturing = false;
  clearInterval(timer);
  recorder.disconnect();
  captureSource.disconnect();
  captureStream.getTracks().forEach((t) => t.stop());
  const rate = captureContext.sampleRate;
  await captureContext.close();
  $("#talk").classList.remove("recording");
  $("#talk-label").textContent = "Hold to talk";
  $("#mic-time").textContent = "";
  $("#voice-status").textContent = discard
    ? "Recording cancelled"
    : "Transcribing locally…";
  if (discard) return;
  try {
    const total = frames.reduce((n, a) => n + a.length, 0);
    if (total < rate * 0.15)
      throw new Error("Hold the button while speaking, then release.");
    const samples = new Float32Array(total);
    let offset = 0;
    for (const chunk of frames) {
      samples.set(chunk, offset);
      offset += chunk.length;
    }
    const offline = new OfflineAudioContext(
        1,
        Math.ceil((total * 16000) / rate),
        16000,
      ),
      buffer = offline.createBuffer(1, total, rate);
    buffer.copyToChannel(samples, 0);
    const source = offline.createBufferSource();
    source.buffer = buffer;
    source.connect(offline.destination);
    source.start();
    const pcm = (await offline.startRendering()).getChannelData(0);
    const wav = new ArrayBuffer(44 + pcm.length * 2),
      view = new DataView(wav);
    const str = (i, s) =>
      [...s].forEach((c, n) => view.setUint8(i + n, c.charCodeAt(0)));
    str(0, "RIFF");
    view.setUint32(4, wav.byteLength - 8, true);
    str(8, "WAVEfmt ");
    view.setUint32(16, 16, true);
    view.setUint16(20, 1, true);
    view.setUint16(22, 1, true);
    view.setUint32(24, 16000, true);
    view.setUint32(28, 32000, true);
    view.setUint16(32, 2, true);
    view.setUint16(34, 16, true);
    str(36, "data");
    view.setUint32(40, pcm.length * 2, true);
    pcm.forEach((v, i) =>
      view.setInt16(44 + i * 2, Math.max(-1, Math.min(1, v)) * 32767, true),
    );
    let binary = "";
    for (const b of new Uint8Array(wav)) binary += String.fromCharCode(b);
    const result = await json("transcribe", { wav: btoa(binary) });
    $("#voice-status").textContent = result.text
      ? "Heard: " + result.text
      : "No speech recognized. Try again.";
  } catch (err) {
    notice(err.message);
    $("#voice-status").textContent = "Microphone is off";
  }
}
$("#talk").onpointerdown = (e) => {
  $("#talk").setPointerCapture(e.pointerId);
  startTalk(e);
};
$("#talk").onpointerup = stopTalk;
$("#talk").onpointercancel = () => {
  held = false;
  stopTalk(null, true);
};
$("#talk").onkeydown = (e) => {
  if ((e.code === "Space" || e.code === "Enter") && !e.repeat) startTalk(e);
};
$("#talk").onkeyup = (e) => {
  if (e.code === "Space" || e.code === "Enter") stopTalk(e);
};
window.addEventListener("blur", () => {
  held = false;
  if (capturing) stopTalk(null, true);
});
setInterval(() => {
  if (Date.now() - lastPoll > 2500) {
    for (const b of document.querySelectorAll(
      "[data-variant],[data-reset],#sort",
    ))
      b.disabled = true;
  }
}, 500);
await pair();
poll();
images();
