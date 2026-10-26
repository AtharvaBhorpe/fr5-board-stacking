/* Table Calibration page (decision-log V-11, V-15, V-19, V-21..V-24). Talks only to the calib_app JSON API. */
"use strict";
const $ = id => document.getElementById(id);
const esc = s => String(s).replace(/[&<>"']/g, c => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"}[c]));
const mm = v => v == null ? "–" : (v * 1000).toFixed(1);
const f1 = v => v == null ? "–" : (+v).toFixed(1);
let S = null, src = "robot", lastFit = null, clicks = [], expanded = null, busy = false;   // expanded: one tag id
const prev = {};                                   // last rendered JSON per section (re-render only on change)

/* ---------- errors from this page go to the app log (tests fail on them) ---------- */
function report(level, msg) { try { fetch("/api/log", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({level, msg})}); } catch (e) {} }
window.addEventListener("error", e => report("error", `${e.message} @ ${e.filename}:${e.lineno}`));
window.addEventListener("unhandledrejection", e => report("error", "promise: " + (e.reason && e.reason.stack || e.reason)));

/* ---------- toasts / api ---------- */
function toast(text, kind = "ok", ms = 6000) {
  const d = document.createElement("div"); d.className = "toast " + kind; d.textContent = text;
  $("toasts").appendChild(d); setTimeout(() => d.remove(), ms);
}
async function post(url, body = {}) {
  if (busy) return null;
  busy = true; document.body.classList.add("busy");                // buttons disabled while a request runs
  try {
    const r = await (await fetch(url, {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)})).json();
    if (!r.ok) toast(r.message || "error", "bad", 9000); else (r.warnings || []).forEach(w => toast(w, "warn", 11000));
    await refresh();
    return r;
  } catch (e) { toast("app not reachable: " + e, "bad"); return null; }
  finally { busy = false; document.body.classList.remove("busy"); }
}
function srcBody() {
  const b = {source: src};
  if (src !== "robot") {
    b.x = parseFloat($("ix").value); b.y = parseFloat($("iy").value);
    if (!isFinite(b.x) || !isFinite(b.y)) { toast(`enter x and y (${src === "pendant" ? "mm, pendant frame" : "m, base_link"})`, "bad"); return null; }
  }
  return b;
}

/* ---------- actions (also used by the self-test) ---------- */
const UI = {
  async identify() {
    if (S && S.refs.length && !confirm("Identify again? Records of tags that leave the set are dropped.")) return null;
    const r = await post("/api/refs/identify"); if (r && r.ok) toast(`${r.refs.length} reference tags: ${r.refs.join(", ")}`); return r;
  },
  async record(id) {
    const b = srcBody(); if (!b) return null; if (id != null) b.id = id;
    const r = await post("/api/record", b);
    if (r && r.ok) { toast(`tag ${r.tag}: record ${r.n}` + (r.spread_mm != null ? ` · spread ${r.spread_mm} mm` : "")); expanded = String(r.tag); prev.refs = null; render(); }
    return r;
  },
  async delRecord(id, i) { return post("/api/record/delete", {id, index: i}); },
  async removeRef(id) { if (!confirm(`Remove tag ${id} from the reference set (and its records)?`)) return null; return post("/api/refs/remove", {id}); },
  async fit() {
    const r = await post("/api/fit");
    if (r && r.ok) { lastFit = r.warnings || []; toast(`fit: rms ${r.calib.rms_mm} mm`); prev.fit = null; }
    return r;
  },
  async spot() {
    const b = srcBody(); if (!b) return null; const m = $("mov").value; if (m) b.id = +m;
    const r = await post("/api/verify/record", b); if (r && r.ok) toast(`spot ${r.summary.n}: error ${r.spot.err_mm} mm`, r.spot.err_mm <= S.config.verify.max_mm ? "ok" : "warn");
    return r;
  },
  async delSpot(i) { return post("/api/verify/delete", {index: i}); },
  async save() {
    const sm = S && S.verify.summary;
    if (!(sm && sm.passed) && !confirm("Verification not passed. Save anyway?")) return null;
    const r = await post("/api/save"); if (r && r.ok) toast("saved — commit config/calib.yaml after a good calibration", "ok", 9000); return r;
  },
  async reset() { if (!confirm("Start over? Clears reference tags, records and verification spots (the saved file stays).")) return null; lastFit = null; return post("/api/reset"); },
  setSource(v) {
    src = v; document.querySelectorAll("#src button").forEach(b => b.classList.toggle("on", b.dataset.v === v));
    $("xyIn").classList.toggle("hidden", v === "robot");
    $("ix").placeholder = v === "pendant" ? "x mm" : "x m"; $("iy").placeholder = v === "pendant" ? "y mm" : "y m";
  },
  toggleRow(id) { id = String(id); expanded = expanded === id ? null : id; prev.refs = null; render(); },
  async recordCovered() {                          // big dock button: record whatever the cup covers
    if (!S) return;
    if (S.cup_on.length === 1) return UI.record();
    const hiddenMov = S.moving.filter(i => S.tags[i] && !S.tags[i].visible);
    if (S.active && hiddenMov.length === 1) return UI.spot();
    toast(S.cup_on.length > 1 ? `several reference tags hidden (${S.cup_on.join(", ")}): use the row button` : "no tag is covered by the cup", "warn");
  },
  theme() {
    const r = document.documentElement, dark = r.dataset.theme ? r.dataset.theme === "dark" : matchMedia("(prefers-color-scheme: dark)").matches;
    r.dataset.theme = dark ? "light" : "dark"; try { localStorage.setItem("calib-theme", r.dataset.theme); } catch (e) {}
    draw();
  },
};
window.UI = UI;

/* ---------- geometry: image <-> screen (object-fit: contain) ---------- */
function fitRect() {
  const v = $("view").getBoundingClientRect(); if (!S || !S.camera.size) return null;
  const [W, H] = S.camera.size, s = Math.min(v.width / W, v.height / H);
  return {x: (v.width - W * s) / 2, y: (v.height - H * s) / 2, s, v};
}

/* ---------- overlay ---------- */
function draw() {
  const c = $("ov"), r = fitRect(); if (!r) return;
  const dpr = devicePixelRatio || 1; c.width = r.v.width * dpr; c.height = r.v.height * dpr;
  const g = c.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, r.v.width, r.v.height);
  const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
  const P = (u, v) => [r.x + u * r.s, r.y + v * r.s];
  // calibrated area
  if (S.active_tags.length > 2) {
    const h = hull(S.active_tags); g.beginPath(); h.forEach(([u, v], i) => { const [x, y] = P(u, v); i ? g.lineTo(x, y) : g.moveTo(x, y); });
    g.closePath(); g.fillStyle = css("--area"); g.fill(); g.setLineDash([7, 6]); g.strokeStyle = css("--ref"); g.globalAlpha = .7; g.lineWidth = 1.5; g.stroke(); g.setLineDash([]); g.globalAlpha = 1;
  }
  const draft = S.draft ? Object.fromEntries(S.draft.tags.map(t => [String(t.id), t])) : {};
  const label = (x, y, text, col) => {
    g.font = "600 13px system-ui, sans-serif"; const w = g.measureText(text).width + 10;
    g.fillStyle = "rgba(10,14,20,.78)"; g.beginPath(); g.roundRect(x, y - 18, w, 20, 6); g.fill();
    g.fillStyle = col; g.fillText(text, x + 5, y - 4);
  };
  for (const [id, t] of Object.entries(S.tags)) {
    if (t.u == null) continue;
    const col = css(t.role === "ref" ? "--ref" : t.role === "moving" ? "--mov" : "--un"), [x, y] = P(t.u, t.v);
    g.strokeStyle = col; g.lineWidth = 2.5;
    if (t.visible && t.corners) { g.beginPath(); t.corners.forEach(([u, v], i) => { const [a, b] = P(u, v); i ? g.lineTo(a, b) : g.moveTo(a, b); }); g.closePath(); g.stroke(); }
    else { g.setLineDash([4, 4]); g.beginPath(); g.arc(x, y, 16, 0, 7); g.stroke(); g.setLineDash([]); }
    if (S.cup_on.includes(+id)) { g.strokeStyle = css("--acc"); g.lineWidth = 3; g.beginPath(); g.arc(x, y, 24 + 3 * Math.sin(Date.now() / 200), 0, 7); g.stroke(); }
    const d = draft[id];                            // fit error ring: radius grows with the error
    if (d) { g.strokeStyle = d.err_mm < 1 ? css("--ok") : d.err_mm < 2 ? css("--warn") : css("--bad"); g.lineWidth = 2; g.beginPath(); g.arc(x, y, 8 + Math.min(d.err_mm, 10) * 3, 0, 7); g.stroke(); }
    const n = (t.records || []).length, sus = S.draft && S.draft.suspect == id;
    label(x + 18, y - 10, `${id}${n ? " · " + n + "×" : ""}${sus ? " ⚠" : ""}${t.visible ? "" : " hidden"}`, sus ? css("--bad") : "#fff");
  }
  S.verify.spots.forEach((p, i) => {                // verification spots (moving tag pixel at record time)
    if (p.u == null) return; const [x, y] = P(p.u, p.v), ok = p.err_mm <= S.config.verify.max_mm;
    g.fillStyle = ok ? css("--ok") : css("--bad"); g.beginPath(); g.arc(x, y, 5, 0, 7); g.fill();
    label(x + 8, y + 22, `#${i + 1} ${p.err_mm} mm`, ok ? css("--ok") : css("--bad"));
  });
  clicks.slice(0, 1).forEach(k => {
    const [x, y] = P(k.u, k.v); g.strokeStyle = "#ff4fd8"; g.lineWidth = 2;
    g.beginPath(); g.moveTo(x - 10, y); g.lineTo(x + 10, y); g.moveTo(x, y - 10); g.lineTo(x, y + 10); g.stroke();
  });
}
function hull(p) {
  p = p.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]); const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  for (const q of p) { while (lo.length > 1 && cr(lo[lo.length - 2], lo[lo.length - 1], q) <= 0) lo.pop(); lo.push(q); }
  for (const q of p.slice().reverse()) { while (up.length > 1 && cr(up[up.length - 2], up[up.length - 1], q) <= 0) up.pop(); up.push(q); }
  return lo.slice(0, -1).concat(up.slice(0, -1));
}

/* ---------- render (each section only when its data changed) ---------- */
function changed(k, data) { const j = JSON.stringify(data); if (prev[k] === j) return false; prev[k] = j; return true; }
const errBar = (e, lim) => `<span class="bar"><i style="width:${Math.min(100, e / lim * 50)}%;background:var(${e < lim / 2 ? "--ok" : e < lim ? "--warn" : "--bad"})"></i></span>${e} mm`;

function steps() {
  const s = S, refsDone = s.refs.length > 0, allRec = refsDone && s.refs.every(i => (s.tags[i] && s.tags[i].records || []).length >= 2);
  const ver = s.verify.summary.passed, saved = s.saved && !s.draft && s.saved.verification && s.saved.verification.passed;
  const done = {identify: refsDone, record: allRec || !!s.active, fit: !!s.active, verify: ver || !!saved, save: !!saved};
  let now = null; document.querySelectorAll("#steps li").forEach(li => {
    const d = done[li.dataset.k]; li.classList.toggle("done", d); const isNow = !d && !now; if (isNow) now = li.dataset.k; li.classList.toggle("now", isNow);
  });
}
function header() {
  const s = S, c = $("pCam");
  c.textContent = s.camera.status === "ok" ? `camera ${(s.camera.size || []).join("×")} · ${s.camera.fps} fps` : "camera: " + s.camera.status;
  c.className = "pill " + (s.camera.status === "ok" ? "ok" : "bad"); c.title = s.camera.device || "";
  $("camdev").textContent = s.camera.device || "";
  const rb = s.robot, p = rb && rb.pose, pr = $("pRob");
  if (!rb) { pr.textContent = "robot reader off"; pr.className = "pill warn"; }
  else { pr.textContent = rb.fresh ? `robot ${rb.rate_hz} Hz · ${p && p.manual_mode ? "manual" : "AUTO"}` : "robot: no data"; pr.className = "pill " + (rb.fresh ? (p && p.manual_mode ? "ok" : "warn") : "bad"); pr.title = rb.last_error || ""; }
  const cl = $("pCal"), sv = s.saved;
  cl.textContent = s.active === "draft" ? "draft fit · not saved" : sv ? `saved ${sv.date.slice(5, 16)} · ${sv.verification ? (sv.verification.passed ? "verified" : "NOT verified") : "not verified"}` : "not calibrated";
  cl.className = "pill " + (s.active === "draft" ? "warn" : sv && sv.verification && sv.verification.passed ? "ok" : sv ? "warn" : "bad");
}
function dock() {
  const s = S, rb = s.robot, p = rb && rb.fresh && rb.pose, cup = $("cupOn");
  const hiddenMov = s.moving.filter(i => s.tags[i] && !s.tags[i].visible);
  if (s.cup_on.length === 1) { cup.textContent = "tag " + s.cup_on[0]; cup.className = "cup"; }
  else if (s.cup_on.length > 1) { cup.textContent = s.cup_on.join(", ") + " ?"; cup.className = "cup warnc"; }
  else if (s.active && hiddenMov.length === 1) { cup.textContent = "spot · " + hiddenMov[0]; cup.className = "cup"; }
  else { cup.textContent = "no tag covered"; cup.className = "cup none"; }
  $("tcp").textContent = p ? `${mm(p.x)} / ${mm(p.y)} / ${mm(p.z)}` : (src === "robot" ? "no robot data" : "typed values");
  $("tcpSub").innerHTML = p ? `tilt <span class="${p.tilt_ok ? "" : "warnc"}">${f1(p.tilt_deg)}°</span> · ${p.manual_mode ? "manual" : '<span class="warnc">auto</span>'}${p.estop ? ' · <span class="badc">E-STOP</span>' : ""}` : "";
  const can = s.cup_on.length === 1 || (s.active && hiddenMov.length === 1);
  $("bRec").disabled = !can;
  $("bRec").textContent = s.cup_on.length === 1 ? `Record tag ${s.cup_on[0]}` : can ? "Record spot" : "Record";
}
function robotCard() {
  const rb = S.robot, p = rb && rb.pose;
  if (!changed("robot", rb)) return;
  $("robot").innerHTML = !rb ? '<span class="mute">reader off (--no-robot): use pendant or manual values</span>' : rb.fresh && p ? `<div class="kv">
    <b>tcp</b><span>${mm(p.x)} / ${mm(p.y)} / ${mm(p.z)} mm</span><b>tilt</b><span class="${p.tilt_ok ? "" : "warnc"}">${f1(p.tilt_deg)}°</span>
    <b>mode</b><span class="${p.manual_mode ? "okc" : "warnc"}">${p.manual_mode ? "manual" : "auto"} · state ${p.robot_state}${p.estop ? ' · <b class="badc">E-STOP</b>' : ""}</span>
    <b>flange check</b><span class="${p.flange_check_mm < 5 ? "okc" : "warnc"}">${f1(p.flange_check_mm)} mm <span class="mute">(our FK vs controller)</span></span>
    <b>joints</b><span class="mute small">${p.joints_deg.map(a => a.toFixed(1)).join("  ")}</span>
    <b>packets</b><span class="mute small">${rb.rate_hz} Hz · ${rb.packets} · checksum ok ${rb.checksum_ok} · framing-only ${rb.checksum_bad_framing_ok} · rejected ${rb.rejected}</span></div>`
    : `<div class="callout bad">${esc(rb.last_error || "no fresh data")}</div><p class="hint">Robot on, our bringup off, port 20004 reachable?</p>`;
}
function refsCard() {
  const s = S, data = {refs: s.refs, tags: s.refs.map(i => s.tags[i]), cup: s.cup_on, draft: s.draft && s.draft.tags, sus: s.draft && s.draft.suspect, exp: expanded, max: s.config.max_per_tag};
  if (!changed("refs", data)) return;
  const stable = Object.values(s.tags).filter(t => t.visible && t.stable).length;
  $("refHint").innerHTML = !s.refs.length ? `${stable} stable tag(s) in view. Place the <b>8 reference tags only</b> (no moving tag), wait until all are stable, then identify.`
    : `${s.refs.length} reference tags · ${s.refs.filter(i => (s.tags[i] && s.tags[i].records || []).length >= 2).length} with 2+ records · click a row for its records`;
  $("bId").classList.toggle("pri", !s.refs.length);
  if (!s.refs.length) { $("refs").innerHTML = ""; return; }
  const dt = s.draft ? Object.fromEntries(s.draft.tags.map(t => [t.id, t])) : {};
  let h = '<table><tr><th>tag</th><th>camera</th><th>records</th><th>spread</th><th>fit error</th><th></th></tr>';
  for (const id of s.refs) {
    const t = s.tags[id] || {records: []}, rs = t.records || [], d = dt[id], sus = s.draft && s.draft.suspect == id, cov = s.cup_on.includes(id);
    const cam = t.missing ? '<span class="chip bad">not seen</span>' : cov ? '<span class="chip acc">cup on it</span>' : t.visible ? (t.stable ? '<span class="chip ok">stable</span>' : '<span class="chip warn">unstable</span>') : '<span class="chip">hidden</span>';
    const dots = "●".repeat(rs.length) + `<span class="e">${"●".repeat(Math.max(0, s.config.max_per_tag - rs.length))}</span>`;
    h += `<tr class="tagrow${cov ? " covered" : ""}" onclick="UI.toggleRow(${id})"><td><span class="id${sus ? " sus" : ""}">${id}</span>${sus ? ' <span class="badc small">suspect: re-record</span>' : ""}</td><td>${cam}</td>
      <td><span class="dots">${dots}</span></td><td class="${t.spread_mm > s.config.spread_warn_mm ? "warnc" : ""}">${t.spread_mm == null ? "–" : t.spread_mm + " mm"}</td>
      <td>${d ? errBar(d.err_mm, 2) : "–"}</td>
      <td class="right"><button class="btn sm pri" onclick="event.stopPropagation();UI.record(${id})" ${rs.length >= s.config.max_per_tag ? "disabled" : ""}>Record</button><button class="x" title="remove from the reference set" onclick="event.stopPropagation();UI.removeRef(${id})">✕</button></td></tr>`;
    if (expanded === String(id)) rs.forEach((r, i) => {
      h += `<tr class="sub"><td>#${i + 1}</td><td colspan="4">x ${mm(r.x)} · y ${mm(r.y)} mm${r.z != null ? ` · z ${mm(r.z)}` : ""}${r.tilt_deg != null ? ` · tilt ${r.tilt_deg}°` : ""}${r.flange_check_mm != null ? ` · flange ${r.flange_check_mm} mm` : ""} · ${esc(r.source)} · ${new Date(r.time * 1000).toLocaleTimeString()}${r.moved ? ' · <b class="badc">tag moved: delete</b>' : ""}</td>
        <td class="right"><button class="x" title="delete this record" onclick="UI.delRecord(${id},${i})">✕</button></td></tr>`;
    });
  }
  $("refs").innerHTML = h + "</table>";
}
function fitCard() {
  const s = S, d = s.draft;
  $("c2").classList.toggle("off", !s.refs.length); $("bFit").disabled = !s.refs.length;
  if (!changed("fit", {d, sv: s.saved, lastFit})) return;
  if (!d) {
    $("fit").innerHTML = s.saved ? `<div class="callout">Using the saved calibration (${esc(s.saved.date)}, rms ${s.saved.rms_mm} mm, ${s.saved.tags} tags). Fit again to replace it.</div>`
      : '<p class="hint">Record every reference tag 2–3× (another orientation / approach each time), then fit.</p>';
    return;
  }
  const lens = d.lens_effect_mm < 1 ? "lens barely distorted" : `lens term corrects up to ${d.lens_effect_mm} mm`;
  $("fit").innerHTML = `<div class="kpis"><div class="kpi"><div class="k">rms</div><div class="v ${d.rms_mm <= 1 ? "okc" : d.rms_mm <= 2 ? "warnc" : "badc"}">${d.rms_mm} mm</div></div>
    <div class="kpi"><div class="k">without lens term</div><div class="v">${d.rms_homography_only_mm} mm</div></div>
    <div class="kpi"><div class="k">lens effect</div><div class="v">${d.lens_effect_mm} mm</div></div><div class="kpi"><div class="k">tags</div><div class="v">${d.tags.length}</div></div></div>
    <div class="callout ${d.suspect != null ? "bad" : "ok"}">${d.suspect != null ? `Tag <b>${d.suspect}</b> looks badly recorded: delete its records and record it again.` : `Fit consistent · ${lens} · k ${(+d.lens_k).toFixed(3)}`}</div>
    ${lastFit && lastFit.length ? `<div class="callout warn"><b>Check</b><ul>${lastFit.map(w => `<li>${esc(w)}</li>`).join("")}</ul></div>` : ""}`;
}
function verifyCard() {
  const s = S; $("c3").classList.toggle("off", !s.active); $("bVer").disabled = !s.active;
  const sel = $("mov"), cur = sel.value, opts = '<option value="">moving tag: auto</option>' + s.moving.map(i => `<option value="${i}" ${String(i) === cur ? "selected" : ""}>tag ${i}</option>`).join("");
  if (changed("movsel", opts)) sel.innerHTML = opts;
  if (!changed("verify", s.verify)) return;
  const sm = s.verify.summary;
  const broken = sm.n && (sm.mean_mm > sm.limits.mean_mm || sm.max_mm > sm.limits.max_mm);
  $("vsum").innerHTML = sm.n ? `<div class="kpis"><div class="kpi"><div class="k">result</div><div class="v verdict ${sm.passed ? "okc" : broken ? "badc" : "mute"}">${sm.passed ? "PASSED" : broken ? "not passed" : "in progress"}</div></div>
    <div class="kpi"><div class="k">spots</div><div class="v">${sm.n} <span class="mute small">/ ${sm.limits.min_spots}+</span></div></div>
    <div class="kpi"><div class="k">mean</div><div class="v ${sm.mean_mm <= sm.limits.mean_mm ? "okc" : "badc"}">${sm.mean_mm} <span class="mute small">≤ ${sm.limits.mean_mm}</span></div></div>
    <div class="kpi"><div class="k">max</div><div class="v ${sm.max_mm <= sm.limits.max_mm ? "okc" : "badc"}">${sm.max_mm} <span class="mute small">≤ ${sm.limits.max_mm}</span></div></div></div>
    ${sm.hint ? `<div class="callout warn">${esc(sm.hint)}</div>` : ""}` : `<p class="hint">${esc(sm.hint)}</p>`;
  $("spots").innerHTML = s.verify.spots.length ? '<table><tr><th>#</th><th>tag</th><th>predicted (mm)</th><th>recorded (mm)</th><th>error</th><th></th></tr>' + s.verify.spots.map((p, i) =>
    `<tr><td>${i + 1}</td><td><span class="id mov">${p.tag ?? "–"}</span></td><td>${mm(p.px)} / ${mm(p.py)}</td><td>${mm(p.x)} / ${mm(p.y)}</td>
     <td>${errBar(p.err_mm, s.config.verify.max_mm)}${p.inside ? "" : ' <span class="chip warn">outside</span>'}</td><td class="right"><button class="x" title="delete spot" onclick="UI.delSpot(${i})">✕</button></td></tr>`).join("") + "</table>" : "";
}
function saveCard() {
  const s = S, sv = s.saved; $("bSave").disabled = !s.active;
  if (!changed("save", {sv, e: s.saved_error})) return;
  $("saved").innerHTML = sv ? `<div class="kv"><b>saved</b><span>${esc(sv.date)} · ${sv.tags} tags · rms ${sv.rms_mm} mm · k ${(+sv.lens_k).toFixed(3)} · ${(sv.image_size || []).join("×")}</span>
    <b>verification</b><span class="${sv.verification && sv.verification.passed ? "okc" : "warnc"}">${sv.verification ? `${sv.verification.passed ? "passed" : "NOT passed"} · ${sv.verification.spots} spots · mean ${sv.verification.mean_mm} / max ${sv.verification.max_mm} mm` : "none"}</span></div>`
    : `<p class="hint">${esc(s.saved_error || "nothing saved yet")}</p>`;
}
function clicksCard() {
  if (!changed("clicks", clicks)) return;
  $("clicks").innerHTML = clicks.length ? "<table>" + clicks.map(k => k.d.ok ? `<tr><td class="mute small">(${k.u.toFixed(0)}, ${k.v.toFixed(0)})</td><td><b>${mm(k.d.x)} / ${mm(k.d.y)} mm</b></td><td>${k.d.inside ? '<span class="chip ok">inside</span>' : '<span class="chip warn">outside</span>'}</td><td class="mute small">${k.d.using}</td></tr>`
    : `<tr><td class="mute small">(${k.u.toFixed(0)}, ${k.v.toFixed(0)})</td><td colspan="3" class="badc">${esc(k.d.message)}</td></tr>`).join("") + "</table>" : prev.clicksEmpty || $("clicks").innerHTML;
}
function render() {
  if (!S) return;
  header(); steps(); dock(); robotCard(); refsCard(); fitCard(); verifyCard(); saveCard(); clicksCard(); draw();
}
async function refresh() {
  try { S = await (await fetch("/api/state")).json(); render(); }
  catch (e) { $("pCam").textContent = "app not reachable"; $("pCam").className = "pill bad"; }
}

/* ---------- wiring ---------- */
$("bId").onclick = () => UI.identify(); $("bFit").onclick = () => UI.fit(); $("bVer").onclick = () => UI.spot();
$("bSave").onclick = () => UI.save(); $("bReset").onclick = () => UI.reset(); $("bRec").onclick = () => UI.recordCovered(); $("theme").onclick = () => UI.theme();
document.querySelectorAll("#src button").forEach(b => b.onclick = () => UI.setSource(b.dataset.v));
$("view").addEventListener("click", async e => {
  const r = fitRect(); if (!r) return; const b = $("view").getBoundingClientRect();
  const u = (e.clientX - b.left - r.x) / r.s, v = (e.clientY - b.top - r.y) / r.s;
  if (u < 0 || v < 0 || u > S.camera.size[0] || v > S.camera.size[1]) return;
  const d = await (await fetch(`/api/pixel?u=${u}&v=${v}`)).json(); clicks = [{u, v, d}, ...clicks].slice(0, 6); render();
});
try { const t = localStorage.getItem("calib-theme"); if (t) document.documentElement.dataset.theme = t; } catch (e) {}
const img = $("img");
img.onload = () => { $("noimg").classList.add("hidden"); draw(); };
img.onerror = () => $("noimg").classList.remove("hidden");
if (location.search.includes("snap")) { const tick = () => img.src = "/snapshot?" + Date.now(); tick(); setInterval(tick, 1000); }
else img.src = "/stream";
addEventListener("resize", draw);
setInterval(refresh, 400); setInterval(() => { if (S && S.cup_on.length) draw(); }, 80); refresh();
if (location.search.includes("selftest")) { const s = document.createElement("script"); s.src = "/web/selftest.js?" + Date.now(); document.body.appendChild(s); }
