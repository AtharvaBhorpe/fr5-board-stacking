"""tools/vacuum_demo_ui.py without ROS / robot: a temp bridge dir (state.txt + cmd pipe read here), the page server
on a free port. Checks: page + status, 'Now' sends inject at once, 'Arm' fires DELAY s into a carry (not before, not
without a carry), cancel, a carry that ends before the delay does not fire, bridge not running -> message, Pause / Resume (TM-48).

    pixi run python3 tests/demo_ui_check.py
"""
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, "..", "tools", "vacuum_demo_ui.py")
ok = bad = 0


def check(name, cond, extra=""):
    global ok, bad
    ok, bad = ok + bool(cond), bad + (not cond)
    print(f"{'PASS' if cond else 'FAIL'} {name}{'  [' + str(extra) + ']' if extra else ''}", flush=True)


d = tempfile.mkdtemp(prefix="demo_ui_")
fifo, state = os.path.join(d, "cmd"), os.path.join(d, "state.txt")
os.mkfifo(fifo)
GOT = []


def reader():
    while True:
        with open(fifo) as f:
            for line in f:
                GOT.append((time.time(), line.strip()))


def set_state(skill="-", phase="-", gripper="IDLE", held=""):
    tail = f" j1 b1 drop_off_b phase {phase}" if skill != "-" else f"    phase {phase}"
    with open(state + ".tmp", "w") as f:
        f.write(f"12:00:00 CTRL RUNNING | skill {skill}{tail} | ready skills True motion True world True\n"
                f"WORLD vision_ok True held '{held}' gripper {gripper} obstacles 0 problems []\n")
    os.replace(state + ".tmp", state)


s = socket.socket(); s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
log = open(os.path.join(d, "ui.log"), "w")
p = subprocess.Popen([sys.executable, TOOL, "--port", str(port), "--bridge", d, "--delay", "0.5"], stdout=log,
                     stderr=subprocess.STDOUT)
base = f"http://127.0.0.1:{port}"


def get(path):
    return urllib.request.urlopen(base + path, timeout=2).read().decode()


def post(path):
    return urllib.request.urlopen(urllib.request.Request(base + path, data=b"", method="POST"), timeout=2).read().decode()


try:
    set_state()
    for _ in range(50):
        try:
            page = get("/"); break
        except OSError:
            time.sleep(0.1)
    st = json.loads(get("/status"))
    check("page served (button) + status JSON (gripper from state.txt)", "Simulate vacuum failure" in page
          and st["state"]["gripper"] == "IDLE" and st["armed"] is False, st)
    post("/now")                                                      # bridge not running yet: no reader
    st = json.loads(get("/status"))
    check("Now without a bridge (no reader on the pipe): clear message, nothing blocks", "bridge not running" in st["msg"],
          st["msg"])
    threading.Thread(target=reader, daemon=True).start(); time.sleep(0.3)
    post("/now"); time.sleep(0.3)
    check("Now: 'inject vacuum_loss' sent at once", [c for _, c in GOT] == ["inject vacuum_loss"], GOT)
    GOT.clear()
    post("/arm"); time.sleep(1.0)
    check("Arm without a carry: nothing sent, still armed", not GOT and json.loads(get("/status"))["armed"], GOT)
    set_state("place", "HOVER", "HOLDING", "b1"); t0 = time.time(); time.sleep(0.25)
    early = list(GOT)
    time.sleep(0.8)
    st = json.loads(get("/status"))
    check("Arm + carry (place HOVER, HOLDING): fires DELAY (0.5 s) into the carry, not before; disarmed after",
          not early and len(GOT) == 1 and 0.4 <= GOT[0][0] - t0 <= 0.9 and not st["armed"] and st["fired"] >= 2,
          f"{[(round(t - t0, 2), c) for t, c in GOT]} armed {st['armed']}")
    GOT.clear(); set_state()
    post("/arm"); set_state("place", "HOVER", "HOLDING", "b2"); time.sleep(0.2); set_state("place", "DESCEND", "HOLDING", "b2")
    time.sleep(0.8)
    check("Arm: a carry that ends before the delay does not fire; stays armed for the next one",
          not GOT and json.loads(get("/status"))["armed"], GOT)
    post("/disarm"); set_state("place", "HOVER", "HOLDING", "b3"); time.sleep(1.0)
    check("Cancel (disarm): no injection on the next carry", not GOT and not json.loads(get("/status"))["armed"], GOT)
    GOT.clear()
    post("/pause"); time.sleep(0.2); post("/resume"); time.sleep(0.3)
    check("TM-48 Pause / Resume buttons send 'pause' / 'resume' to the controller (bridge), in order",
          [c for _, c in GOT] == ["pause", "resume"] and 'id="pause"' in get("/") and 'id="resume"' in get("/"), GOT)
    with open(state, "w") as f:
        f.write("12:00:01 CTRL PAUSED PAUSED | skill -    phase - | ready skills True motion True world True\n"
                "WORLD vision_ok True held '' gripper IDLE obstacles 0 problems []\n")
    st = json.loads(get("/status"))
    check("TM-48 status shows the controller paused", st["state"]["paused"] is True, st["state"])
finally:
    p.terminate(); p.wait(timeout=5); log.close()
print(f"{ok} passed, {bad} failed")
sys.exit(1 if bad else 0)
