"""Browser UI test of the calibration page (V7): the real app (synthetic table image + fake controller) and the real
page in headless Firefox running web/selftest.js (?selftest). The page clicks through identify, record (row buttons +
Space on a covered tag), delete, fit, verify (5 spots), save, image click, theme, robot source, start over; this
harness changes the camera image when the page asks ("wait" log lines). Fails on any FAIL line or browser error.
Also saves screenshots for a visual check.

    pixi run python3 tests/web_ui_check.py        (needs firefox; snap firefox works)
"""
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import urllib.parse
import urllib.request

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import synth_table as st                      # noqa: E402
from fake_controller import FakeController    # noqa: E402

FIREFOX = shutil.which("firefox")
if not FIREFOX:
    print("SKIP: firefox not found"); sys.exit(0)
SHARE = get_package_share_directory("hk_fr5_vision")
APP = os.path.join(get_package_prefix("hk_fr5_vision"), "lib", "hk_fr5_vision", "calib_app")
PORT = 18090
SNAP = os.path.isdir(os.path.expanduser("~/snap/firefox"))   # Ubuntu: /usr/bin/firefox is a wrapper for the snap
FF_HOME = os.path.expanduser("~/snap/firefox/common") if SNAP else tempfile.gettempdir()   # snap: no /tmp access
SHOTS = os.environ.get("HK_UI_SHOTS", os.path.join(FF_HOME, "hk_ui_shots"))
MOV, SPOTS = 37, [(-0.40, 0.10), (-0.55, -0.35), (-0.20, 0.50), (-0.10, -0.60), (-0.50, 0.55)]

tmp = tempfile.mkdtemp(prefix="web_ui_check_")
img = os.path.join(tmp, "table.png")
fc = FakeController(); fc.start()
cfg = yaml.safe_load(open(os.path.join(SHARE, "config", "calib_app.yaml")))
cfg.update(calib_file=os.path.join(tmp, "calib.yaml"), robot_config=os.path.join(tmp, "robot.yaml"),
           session_file=os.path.join(tmp, "session.json"), port=PORT)
cfg["tags"]["verify_seen_s"] = 3.0
yaml.safe_dump(cfg, open(os.path.join(tmp, "app.yaml"), "w"))
rcfg = yaml.safe_load(open(os.path.join(SHARE, "config", "robot.yaml")))
rcfg.update(robot_ip="127.0.0.1", state_port=fc.port, reconnect_period=0.3)
yaml.safe_dump(rcfg, open(cfg["robot_config"], "w"))
open(cfg["calib_file"], "w").write("version: 1\ncalibrated: false\n")
os.makedirs(SHOTS, exist_ok=True)

def get(path):
    with urllib.request.urlopen(f"http://127.0.0.1:{PORT}{path}", timeout=5) as r:
        return json.loads(r.read())
def firefox(url, profile, shot=None):
    pdir = os.path.join(FF_HOME, profile); shutil.rmtree(pdir, ignore_errors=True); os.makedirs(pdir)
    args = [FIREFOX, "--headless", "--new-instance", "--profile", pdir, "--window-size", "1600,1250"]
    if shot:
        try:
            subprocess.run(args + ["--screenshot", shot, url], cwd=FF_HOME, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL, timeout=60)
        except subprocess.TimeoutExpired:
            print(f"(screenshot {os.path.basename(shot)} timed out)")
        shutil.rmtree(pdir, ignore_errors=True); return None
    return subprocess.Popen(args + [url], cwd=FF_HOME, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
def killpg(p):
    try:
        os.killpg(p.pid, signal.SIGTERM); p.wait(timeout=10)
    except Exception:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except Exception:
            pass

st.render(st.CALIB, img)
app = subprocess.Popen([APP, "--config", os.path.join(tmp, "app.yaml"), "--image", img], stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, text=True, start_new_session=True)
results, errors, done, ff = [], [], None, None
try:
    for _ in range(100):
        try:
            get("/api/state"); break
        except OSError:
            time.sleep(0.1)
    end = time.time() + 10
    while time.time() < end and sum(1 for t in get("/api/state")["tags"].values() if t.get("stable")) < 8:
        time.sleep(0.2)
    truth = {"refs": {str(i): list(p) for i, p in st.CALIB.items()}, "mov": MOV, "spots": [list(p) for p in SPOTS]}
    url = f"http://127.0.0.1:{PORT}/?snap&selftest#" + urllib.parse.quote(json.dumps(truth))
    ff = firefox(url, "hk_selftest_profile")
    seen, end, covered = 0, time.time() + 150, None
    while time.time() < end and done is None:
        log = get("/api/log")["log"]
        for e in log[seen:]:
            lvl, msg = e["level"], e["msg"]
            if lvl == "wait":
                kind, arg = msg.split()
                if kind == "cover":
                    covered = int(arg); st.render({i: p for i, p in st.CALIB.items() if i != covered}, img)
                elif kind == "uncover":
                    st.render(st.CALIB, img)
                elif kind == "spot":
                    k = int(arg)
                    if k == 4:   # screenshot of the verify phase (4 spots done) in a 2nd browser
                        firefox(f"http://127.0.0.1:{PORT}/?snap", "hk_shot_profile", os.path.join(SHOTS, "verify.png"))
                    st.render({**st.CALIB, MOV: SPOTS[k]}, img)
            elif lvl == "selftest":
                print(msg, flush=True)
                if msg.startswith("DONE"):
                    done = msg
                else:
                    results.append(msg)
            elif lvl == "error":
                print("BROWSER ERROR " + msg, flush=True); errors.append(msg)
        seen = len(log)
        time.sleep(0.15)
    firefox(f"http://127.0.0.1:{PORT}/?snap", "hk_shot_profile", os.path.join(SHOTS, "after_reset.png"))
finally:
    if ff:
        killpg(ff)
    os.killpg(app.pid, signal.SIGINT)
    try:
        out = app.communicate(timeout=5)[0]
    except subprocess.TimeoutExpired:
        os.killpg(app.pid, signal.SIGKILL); out = app.communicate()[0]
    fc.close(); shutil.rmtree(tmp, ignore_errors=True)
    for prof in ("hk_selftest_profile", "hk_shot_profile"):
        shutil.rmtree(os.path.join(FF_HOME, prof), ignore_errors=True)

if not results and done is None:
    print("the page never reported: firefox did not start or the self-test did not load")
fails = [r for r in results if not r.startswith("PASS")]
print(f"page log errors: {len(errors)}; app traceback: {'Traceback' in out}; screenshots in {SHOTS}")
ok = done is not None and done.endswith("/0") and not fails and not errors and "Traceback" not in out
print(f"{len(results) - len(fails)} passed, {len(fails) + len(errors) + (done is None)} failed" + ("" if done else " (self-test did not finish)"))
sys.exit(0 if ok else 1)
