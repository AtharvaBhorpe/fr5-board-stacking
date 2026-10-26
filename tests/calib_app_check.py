"""Calibration app backend checks (V6): real app (subprocess) on a synthetic table image + fake controller (port
20004 stream), temp calib / session files. Covers tags, record (robot / pendant / manual), limits, fit, moved-tag
detection, reference tags (identify / remove, V-23), moving tag of any id (one / several), verification (pass, same spot,
fail hint), save -> calib.yaml (+ verification), pixel test, session restore,
errors, stream / snapshot.

    pixi run python3 tests/calib_app_check.py
"""
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import urllib.request

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import synth_table as st                      # noqa: E402
from fake_controller import FakeController    # noqa: E402
from hk_fr5_vision import calib, robot_pose   # noqa: E402

ok = bad = 0
def check(name, cond, extra=""):
    global ok, bad
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True)
    ok += bool(cond); bad += not cond

SHARE = get_package_share_directory("hk_fr5_vision")
APP = os.path.join(get_package_prefix("hk_fr5_vision"), "lib", "hk_fr5_vision", "calib_app")
PORT = 18080
tmp = tempfile.mkdtemp(prefix="calib_app_check_")
img = os.path.join(tmp, "table.png")
fc = FakeController(); fc.start()   # streams the 'ready' pose (j1 sweeping) at 100 Hz

cfg = yaml.safe_load(open(os.path.join(SHARE, "config", "calib_app.yaml")))
cfg.update(calib_file=os.path.join(tmp, "calib.yaml"), robot_config=os.path.join(tmp, "robot.yaml"),
           session_file=os.path.join(tmp, "session.json"), port=PORT)
cfg["camera"]["name"] = "synthetic"
cfg["tags"]["verify_seen_s"] = 3.0   # removed tags stop counting as 'moving' quickly (test speed)
yaml.safe_dump(cfg, open(os.path.join(tmp, "app.yaml"), "w"))
rcfg = yaml.safe_load(open(os.path.join(SHARE, "config", "robot.yaml")))
rcfg.update(robot_ip="127.0.0.1", state_port=fc.port, reconnect_period=0.3)
yaml.safe_dump(rcfg, open(cfg["robot_config"], "w"))
open(cfg["calib_file"], "w").write("version: 1\ncalibrated: false\n")

def get(path):
    with urllib.request.urlopen(f"http://127.0.0.1:{PORT}{path}", timeout=5) as r:
        return json.loads(r.read())
def post(path, body=None):
    req = urllib.request.Request(f"http://127.0.0.1:{PORT}{path}", data=json.dumps(body or {}).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.loads(r.read())
def start():
    p = subprocess.Popen([APP, "--config", os.path.join(tmp, "app.yaml"), "--image", img], stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True, start_new_session=True)
    for _ in range(100):
        try:
            get("/api/state"); return p
        except OSError:
            time.sleep(0.1)
    raise RuntimeError("app did not start: " + (p.stdout.read() if p.poll() is not None else ""))
def stop(p):
    os.killpg(p.pid, signal.SIGINT)
    try:
        return p.communicate(timeout=5)[0]
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL); return p.communicate()[0] + " [KILLED]"
def wait_tags(ids, epoch=None, timeout=5):
    end = time.time() + timeout
    while time.time() < end:
        t = get("/api/state")["tags"]
        if all(str(i) in t and t[str(i)].get("n", 0) >= 10 and (epoch is None or t[str(i)]["epoch"] == epoch[i])
               for i in ids):
            return t
        time.sleep(0.1)
    return get("/api/state")["tags"]

rng = np.random.default_rng(2)
v_spot = (-0.40, 0.10)
MOV = 37   # moving tag: any id that is not a reference tag (V-23)
st.render({**st.CALIB, 6: v_spot}, img)   # tag 6 lies around by mistake when the references are identified
p = start()
try:
    t = wait_tags(list(st.CALIB) + [6])
    s = get("/api/state")
    check("9 tags seen, stable, unassigned before identify, camera size 1920x1080",
          all(str(i) in t and t[str(i)]["stable"] for i in list(st.CALIB) + [6]) and t["6"]["role"] == "unassigned"
          and s["camera"]["size"] == [1920, 1080], f"tags {sorted(t)}, cam {s['camera']}")
    r = post("/api/record", {"id": 14, "source": "manual", "x": 0, "y": 0})
    check("record before identify: refused", not r["ok"] and "identify" in r["message"], r.get("message"))
    r = post("/api/refs/identify")
    check("identify with 9 tags in view: 9 refs, warning (8 recommended)", r["ok"] and len(r["refs"]) == 9 and r["warnings"],
          str(r))
    r = post("/api/refs/remove", {"id": 6})
    t = get("/api/state")["tags"]
    check("remove tag 6 from the refs: 8 refs, tag 6 now 'moving', 14 'ref'", r["ok"] and sorted(r["refs"]) == sorted(st.CALIB)
          and t["6"]["role"] == "moving" and t["14"]["role"] == "ref", str(r))
    st.render(st.CALIB, img); time.sleep(cfg["tags"]["seen_s"] + 0.5)   # tag 6 no longer visible
    r = post("/api/refs/identify")
    check("identify again with exactly the 8 tags (6 removed from the table): 8 refs, no warning",
          r["ok"] and sorted(r["refs"]) == sorted(st.CALIB) and not r["warnings"], str(r))
    pt = st.to_px(*st.CALIB[14])
    check("tag 14 pixel average = synthetic truth (< 0.5 px)", math.dist((t["14"]["u"], t["14"]["v"]), pt) < 0.5)

    # robot source: fake controller pose (ready, tcp z +364 mm -> 'cup not on the table' warning)
    time.sleep(0.5)
    r = post("/api/record", {"id": 14, "source": "robot"})
    sr = get("/api/state")["robot"]
    check("record from robot (port 20004): accepted, z warning, values = our FK of the streamed joints",
          r["ok"] and any("not on the table" in w for w in r["warnings"]) and sr["connected"] and sr["fresh"],
          f"{r.get('warnings')} {r.get('message', '')}")
    rec = get("/api/state")["tags"]["14"]["records"][0]
    js = sr["pose"]["joints_deg"]
    _, T = robot_pose.fk([math.radians(a) for a in js], robot_pose.load_config(cfg["robot_config"]))
    check("robot record x / y close to FK of the stream (j1 sweeps: within 10 mm)",
          math.dist((rec["x"], rec["y"]), (T[0, 3], T[1, 3])) < 0.010 and rec["source"] == "robot")
    post("/api/record/delete", {"id": 14, "index": 0})
    check("delete record", "14" not in get("/api/state")["tags"] or not get("/api/state")["tags"]["14"]["records"])

    r = post("/api/record", {"id": 14, "source": "pendant", "x": -47.83, "y": -836.6})
    rec = get("/api/state")["tags"]["14"]["records"][0]
    check("pendant values: mm, x / y negated -> base_link m; warning about the flange",
          r["ok"] and abs(rec["x"] - 0.04783) < 1e-9 and abs(rec["y"] - 0.8366) < 1e-9 and r["warnings"])
    post("/api/record/delete", {"id": 14})

    # manual records: 2 per tag, 1 mm noise
    for i, (x, y) in st.CALIB.items():
        for _ in range(2):
            r = post("/api/record", {"id": i, "source": "manual", "x": x + rng.normal(0, 0.001), "y": y + rng.normal(0, 0.001)})
    check("16 manual records (2 per tag) accepted", r["ok"] and sum(len(v["records"]) for v in get("/api/state")["tags"].values()) == 16)
    post("/api/record", {"id": 0, "source": "manual", "x": st.CALIB[0][0], "y": st.CALIB[0][1]})
    r = post("/api/record", {"id": 0, "source": "manual", "x": 0.0, "y": 0.0})
    check("4th record of a tag refused (max 3)", not r["ok"] and "delete one first" in r["message"], r.get("message"))
    for body, text in [({"id": 6, "source": "manual", "x": 0, "y": 0}, "not a reference tag"),
                       ({"id": 42, "source": "manual", "x": 0, "y": 0}, "not a reference tag"),
                       ({"id": 13, "source": "manual", "x": "a", "y": 0}, "numbers"),
                       ({"id": 13, "source": "manual", "x": float("inf"), "y": 0}, "finite"),
                       ({"id": 13, "source": "teleport"}, "unknown source")]:
        try:
            r = post("/api/record", body)
        except ValueError:   # json.dumps(inf) -> Infinity: server sees bad JSON
            r = {"ok": False, "message": "finite (client)"}
        check(f"record refused: {text}", not r["ok"] and text in r["message"], r.get("message"))

    r = post("/api/fit")
    c = r.get("calib", {})
    check("fit: 8 tags, rms ~1 mm, lens_k ~0 (ideal lens), lens effect < 1 mm, draft active",
          r["ok"] and len(c["tags"]) == 8 and c["rms_mm"] < 2.0 and abs(c["lens_k"]) < 0.1 and c["lens_effect_mm"] < 1.0
          and get("/api/state")["active"] == "draft", f"rms {c.get('rms_mm')} k {c.get('lens_k')} effect {c.get('lens_effect_mm')}")
    q = get(f"/api/pixel?u={pt[0]}&v={pt[1]}")
    check("pixel test with the draft: corner tag 14 pixel -> its table x / y (< 3 mm), inside (on the outline)",
          q["ok"] and q["using"] == "draft" and math.dist((q["x"], q["y"]), st.CALIB[14]) < 0.003 and q["inside"], str(q))
    far = st.to_px(-0.85, 0.0)
    q = get(f"/api/pixel?u={far[0]}&v={far[1]}")
    check("pixel test: x -0.85 (beyond the tags) -> outside", q["ok"] and not q["inside"], str(q))

    # verification: moving tag (id 37, never seen before) at 5 spots, recorded at the truth + 0.5 mm noise
    r = post("/api/verify/record", {"source": "manual", "x": -0.10, "y": -0.60})   # cup elsewhere, tag 6 is gone
    check("removed tag 6 (seen < verify_seen_s ago) used as moving tag: spot far off -> 'is the cup on' warning",
          r["ok"] and any("is the cup on" in w for w in r["warnings"]), str(r.get("warnings", r.get("message"))))
    post("/api/verify/delete", {"all": True}); time.sleep(cfg["tags"]["verify_seen_s"] + 0.5)
    r = post("/api/verify/record", {"source": "manual", "x": 0, "y": 0})
    check("verify with no non-reference tag in view: refused", not r["ok"] and "no moving tag" in r["message"], r.get("message"))
    spots = [(-0.40, 0.10), (-0.55, -0.35), (-0.20, 0.50), (-0.10, -0.60), (-0.50, 0.55)]
    epoch = None
    for k, (x, y) in enumerate(spots):
        st.render({**st.CALIB, MOV: (x, y)}, img)
        if epoch is None:
            wait_tags([MOV]); epoch = get("/api/state")["tags"][str(MOV)]["epoch"]
        else:
            epoch += 1; wait_tags([MOV], {MOV: epoch})
        r = post("/api/verify/record", {"source": "manual", "x": x + rng.normal(0, 0.0005), "y": y + rng.normal(0, 0.0005)})
    sm = r["summary"]
    check("verification 5 spots with moving tag 37: passed (mean <= 2, max <= 4 mm), spot knows its tag",
          r["ok"] and sm["n"] == 5 and sm["passed"] and r["spot"]["tag"] == MOV, f"{sm} {r.get('message', '')}")
    r = post("/api/verify/record", {"source": "manual", "x": spots[-1][0], "y": spots[-1][1]})
    check("same spot again: warnings (tag not moved, spot too close)", r["ok"] and len(r["warnings"]) == 2, str(r["warnings"]))
    post("/api/verify/delete", {"index": 5})
    check("delete spot -> 5 left", get("/api/state")["verify"]["summary"]["n"] == 5)
    st.render({**st.CALIB, MOV: spots[-1], 9: (-0.25, -0.15)}, img); wait_tags([9])
    r = post("/api/verify/record", {"source": "manual", "x": 0, "y": 0})
    check("two non-reference tags in view: refused, asks to choose", not r["ok"] and "choose one" in r["message"], r.get("message"))
    r = post("/api/verify/record", {"id": 9, "source": "manual", "x": -0.25, "y": -0.15})
    check("choose moving tag 9 by id: accepted", r["ok"] and r["spot"]["tag"] == 9, r.get("message", ""))
    r = post("/api/verify/record", {"id": 14, "source": "manual", "x": 0, "y": 0})
    check("choosing a reference tag as the moving tag: refused", not r["ok"] and "reference tag" in r["message"], r.get("message"))
    post("/api/verify/delete", {"index": 5})
    st.render({**st.CALIB, MOV: spots[-1]}, img); time.sleep(cfg["tags"]["verify_seen_s"] + 0.5)   # tag 9 gone

    r = post("/api/save")
    saved = calib.load(cfg["calib_file"])
    check("save: calib.yaml valid, 8 tags, verification passed with 5 spots, no warning, .bak kept",
          r["ok"] and len(saved["tags"]) == 8 and saved["verification"]["passed"] and saved["verification"]["spots"] == 5
          and not r["warnings"] and os.path.exists(cfg["calib_file"] + ".bak"), str(r))
    s = get("/api/state")
    check("after save: active = saved, page shows its date + verification", s["active"] == "saved" and s["draft"] is None
          and s["saved"]["verification"]["passed"])

    # a calibration tag that moves after recording
    moved = dict(st.CALIB); moved[0] = (-0.6635 + 0.02, -0.5954)
    e0 = get("/api/state")["tags"]["0"]["epoch"]
    st.render({**moved, MOV: spots[-1]}, img); wait_tags([0], {0: e0 + 1})
    r = post("/api/fit")
    check("tag 0 moved after recording: flagged in state, fit skips it with a warning",
          get("/api/state")["tags"]["0"]["records"][0]["moved"] and any("tag 0 moved" in w for w in r["warnings"])
          and len(r["calib"]["tags"]) == 7, str(r.get("warnings")))

    # failing verification: recorded positions 4 mm off in the same direction -> systematic hint
    post("/api/verify/delete", {"all": True})
    ep = get("/api/state")["tags"][str(MOV)]["epoch"]
    for k, (x, y) in enumerate(spots):
        st.render({**moved, MOV: (x, y)}, img); ep += 1; wait_tags([MOV], {MOV: ep})
        r = post("/api/verify/record", {"source": "manual", "x": x + 0.004, "y": y})
    sm = r["summary"]
    check("verification 4 mm off the same way: failed, hint = systematic offset", not sm["passed"] and "same way" in sm["hint"],
          str(sm))
    r = post("/api/save")
    check("save without a passed verification: allowed, warns", r["ok"] and r["warnings"], str(r))
finally:
    out = stop(p)
check("app log: no traceback", "Traceback" not in out, out[-300:])

# session restore after a restart
p = start()
try:
    s = get("/api/state")
    check("restart: reference tags + records + verification restored from the session file",
          sorted(s["refs"]) == sorted(st.CALIB) and sum(len(v.get("records", [])) for v in s["tags"].values()) == 17
          and s["verify"]["summary"]["n"] == 5,
          f"{sum(len(v.get('records', [])) for v in s['tags'].values())} records, {s['verify']['summary']['n']} spots")
    for _ in range(50):   # first frame after the restart
        with urllib.request.urlopen(f"http://127.0.0.1:{PORT}/snapshot", timeout=5) as r:
            jpg = r.read()
        if jpg:
            break
        time.sleep(0.1)
    with urllib.request.urlopen(f"http://127.0.0.1:{PORT}/stream", timeout=5) as r:
        head = r.read(200)
    check("snapshot = JPEG, stream = multipart JPEG", jpg[:2] == b"\xff\xd8" and b"--frame" in head and b"image/jpeg" in head)
    r = post("/api/record/delete", {"all": True})
    check("delete all records", r["ok"] and not any(v.get("records") for v in get("/api/state")["tags"].values()))
finally:
    out = stop(p)
check("Ctrl+C: app exits cleanly", "[KILLED]" not in out and "Traceback" not in out, out[-200:])
fc.close(); shutil.rmtree(tmp)
print(f"{ok} passed, {bad} failed")
sys.exit(1 if bad else 0)
