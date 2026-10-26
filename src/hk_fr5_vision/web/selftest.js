/* Browser self-test of the calibration page (loaded only with ?selftest). Clicks through the real UI against a test
   backend (synthetic camera image + fake controller), asks the harness (tests/web_ui_check.py) to change the image via
   "WAIT ..." log lines, reports "PASS / FAIL ..." lines and "DONE p/f" to /api/log. Truth values come in the URL hash. */
(async () => {
  "use strict";
  const T = JSON.parse(decodeURIComponent(location.hash.slice(1)));   // {refs: {id: [x, y]}, mov: id, spots: [[x, y]...]}
  window.confirm = () => true;                                         // dialogs auto-accepted in the self-test
  let pass = 0, fail = 0;
  const log = (level, msg) => fetch("/api/log", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({level, msg})});
  const ok = async (name, cond, extra = "") => { cond ? pass++ : fail++; await log("selftest", `${cond ? "PASS" : "FAIL"} ${name}${extra ? "  [" + extra + "]" : ""}`); };
  const sleep = ms => new Promise(r => setTimeout(r, ms));
  const until = async (fn, ms = 8000) => { const end = Date.now() + ms; while (Date.now() < end) { if (!busy) { await refresh(); try { if (!busy && fn()) return true; } catch (e) {} } await sleep(150); } return false; };
  const wait = async (what, fn, ms = 15000) => { await log("wait", what); return until(fn, ms); };
  const typeXY = (x, y) => { $("ix").value = x; $("iy").value = y; };
  const rowBtn = id => [...document.querySelectorAll("#refs tr.tagrow")].find(r => r.querySelector(".id").textContent == id);
  try {
    await until(() => S && Object.keys(S.tags).length >= 8);
    await ok("page loaded: 8 tags in the state, camera ok, step 1 'Identify' is current",
      S.camera.status === "ok" && document.querySelector("#steps li.now").dataset.k === "identify");
    await until(() => !$("noimg").classList.contains("hidden"), 5000);
    await ok("camera image shown (snapshot / stream loaded)", $("noimg").classList.contains("hidden"));

    $("bId").click(); await until(() => S.refs.length === 8);
    await ok("click 'Identify reference tags' -> 8 rows", S.refs.length === 8 && document.querySelectorAll("#refs tr.tagrow").length === 8);
    await ok("step 'Record' is current now", document.querySelector("#steps li.now").dataset.k === "record");

    UI.setSource("manual");
    await ok("source Manual shows the x / y inputs", !$("xyIn").classList.contains("hidden"));
    const ids = Object.keys(T.refs).map(Number), cover = ids[1];
    for (const id of ids) {
      for (let k = 0; k < 2; k++) {
        if (id === cover && k === 1) continue;                         // 2nd record of this tag via the big dock button
        const [x, y] = T.refs[id]; typeXY(x + (k ? 0.0006 : -0.0005), y + (k ? -0.0004 : 0.0005));
        rowBtn(id).querySelector("button.pri").click();
        await until(() => ((S.tags[id] || {}).records || []).length === k + 1, 5000);
      }
    }
    await ok("row Record buttons: 2 records per tag (1 for the tag kept for the dock button)",
      ids.every(id => S.tags[id].records.length === (id === cover ? 1 : 2)), ids.map(i => S.tags[i].records.length).join(","));

    // cover a tag (harness removes it from the image) -> dock shows it, the big Record button records it
    await wait(`cover ${cover}`, () => S.cup_on.length === 1 && S.cup_on[0] === cover);
    await until(() => $("cupOn").textContent === `tag ${cover}`, 3000);
    await ok(`covered tag ${cover}: dock says 'tag ${cover}', big button enabled`, $("cupOn").textContent === `tag ${cover}` && !$("bRec").disabled, $("cupOn").textContent);
    typeXY(T.refs[cover][0] + 0.0004, T.refs[cover][1]);
    $("bRec").click();
    await until(() => S.tags[cover].records.length === 2, 5000);
    await ok("big dock button 'Record tag N' records the covered tag", S.tags[cover].records.length === 2);
    await wait(`uncover ${cover}`, () => S.cup_on.length === 0);

    rowBtn(ids[0]).click(); await sleep(300);
    await ok("click a row: only its 2 numbered record sub-rows appear", document.querySelectorAll("#refs tr.sub").length === 2,
      String(document.querySelectorAll("#refs tr.sub").length));
    const subs = document.querySelectorAll("#refs tr.sub").length;
    document.querySelector("#refs tr.sub button.x").click(); await until(() => S.tags[ids[0]].records.length === 1, 4000);
    await ok("sub-row ✕ deletes that record", S.tags[ids[0]].records.length === 1);
    typeXY(T.refs[ids[0]][0], T.refs[ids[0]][1]); rowBtn(ids[0]).querySelector("button.pri").click();
    await until(() => S.tags[ids[0]].records.length === 2, 4000);

    $("bFit").click(); await until(() => S.draft, 6000); await sleep(400);
    const rms = S.draft && S.draft.rms_mm;
    await ok("Fit: KPIs shown, rms < 2 mm, no suspect, fit errors in the rows", S.draft && rms < 2 && S.draft.suspect == null
      && $("fit").textContent.includes("rms") && document.querySelectorAll("#refs .bar").length === 8, `rms ${rms}`);
    await ok("step 'Verify' is current, verify card enabled", document.querySelector("#steps li.now").dataset.k === "verify" && !$("bVer").disabled);

    // verification: the harness puts the moving tag at each spot in turn
    for (let k = 0; k < T.spots.length; k++) {
      await wait(`spot ${k}`, () => S.moving.includes(T.mov) && S.tags[T.mov].n >= 10 && S.tags[T.mov].epoch === k);
      const [x, y] = T.spots[k]; typeXY(x + 0.0003, y - 0.0003);
      $("bVer").click(); await until(() => S.verify.spots.length === k + 1, 5000);
    }
    await sleep(500);
    await ok(`verify ${T.spots.length} spots: PASSED shown`, S.verify.summary.passed && $("vsum").textContent.includes("PASSED"), JSON.stringify(S.verify.summary));
    await ok("step 'Save' is current", document.querySelector("#steps li.now").dataset.k === "save");

    $("bSave").click(); await until(() => S.saved && !S.draft, 6000); await sleep(400);
    await ok("Save: saved card shows the date + passed verification, header pill 'verified'",
      S.saved && $("saved").textContent.includes("passed") && $("pCal").textContent.includes("verified"), $("pCal").textContent);
    await ok("all 5 steps done", document.querySelectorAll("#steps li.done").length === 5);

    // click on the image at tag ids[0] -> its x / y
    const t0 = S.tags[ids[0]], r = fitRect(), b = $("view").getBoundingClientRect();
    $("view").dispatchEvent(new MouseEvent("click", {clientX: b.left + r.x + t0.u * r.s, clientY: b.top + r.y + t0.v * r.s, bubbles: true}));
    await until(() => clicks.length === 1, 4000); await sleep(300);
    const c = clicks[0] && clicks[0].d;
    await ok("click on the image: table x / y of that tag (< 3 mm), listed", c && c.ok && Math.hypot(c.x - T.refs[ids[0]][0], c.y - T.refs[ids[0]][1]) < 0.003
      && $("clicks").textContent.includes("mm"), c && `${c.x} ${c.y}`);

    const before = document.documentElement.dataset.theme || "system"; $("theme").click(); await sleep(200);
    await ok("theme toggle switches light / dark", (document.documentElement.dataset.theme || "system") !== before);
    $("theme").click();

    UI.setSource("robot");
    await ok("source Robot: inputs hidden, dock shows live tcp from the fake controller", $("xyIn").classList.contains("hidden") && /\d/.test($("tcp").textContent), $("tcp").textContent);
    await ok("robot card: flange check + packets shown", $("robot").textContent.includes("flange check") && $("robot").textContent.includes("checksum ok"));

    $("bReset").click(); await until(() => S.refs.length === 0, 5000);
    await ok("Start over: reference set cleared, saved calibration kept", S.refs.length === 0 && S.saved && document.querySelector("#steps li.now").dataset.k === "identify");
  } catch (e) {
    fail++; await log("selftest", "FAIL exception: " + (e && e.stack || e));
  }
  await log("selftest", `DONE ${pass}/${fail}`);
})();
