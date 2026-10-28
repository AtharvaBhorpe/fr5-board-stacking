"""Demo page -> REAL agent bridge -> controller, without robot / motion: a stand-in /task/control service records
every request. Checks that the page buttons arrive as the right Control commands (pause / resume / stop, whole
controller: job_id '') and that 'Simulate vacuum failure -> Now' makes the bridge start the injection tool.

    ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=82 pixi run python3 tests/demo_ui_bridge_check.py
"""
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request

import rclpy
from rclpy.executors import SingleThreadedExecutor

from hk_fr5_task_interfaces.srv import Control

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(HERE, "..", "tools")
ok = bad = 0


def check(name, cond, extra=""):
    global ok, bad
    ok, bad = ok + bool(cond), bad + (not cond)
    print(f"{'PASS' if cond else 'FAIL'} {name}{'  [' + str(extra) + ']' if extra else ''}", flush=True)


REQ, T_IN, T_OUT = [], [], []
rclpy.init()
n = rclpy.create_node("main_controller_standin", namespace="task")


def on_control(req, res):
    REQ.append((req.command, req.job_id, req.priority))
    T_IN.append(time.time())
    res.success, res.message = True, "ok (stand-in)"
    return res


n.create_service(Control, "/task/control", on_control)
ex = SingleThreadedExecutor(); ex.add_node(n)
threading.Thread(target=ex.spin, daemon=True).start()

d = tempfile.mkdtemp(prefix="demo_ui_bridge_")
bridge_dir = os.path.join(d, "bridge")
s = socket.socket(); s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
logs = {k: open(os.path.join(d, f"{k}.log"), "w") for k in ("bridge", "ui")}
procs = {"bridge": subprocess.Popen([sys.executable, os.path.join(TOOLS, "agent_bridge.py"), bridge_dir],
                                    stdout=logs["bridge"], stderr=subprocess.STDOUT)}
base = f"http://127.0.0.1:{port}"


def post(path):
    return urllib.request.urlopen(urllib.request.Request(base + path, data=b"", method="POST"), timeout=2).read()


def until(fn, secs):
    end = time.time() + secs
    while time.time() < end:
        if fn():
            return True
        time.sleep(0.05)
    return False


try:
    # the pipe exists before the bridge reads it: wait for its "bridge up" line (a press before that is lost)
    ev = os.path.join(bridge_dir, "events.txt")
    until(lambda: os.path.exists(ev) and "bridge up" in open(ev).read(), 30)
    procs["ui"] = subprocess.Popen([sys.executable, os.path.join(TOOLS, "vacuum_demo_ui.py"), "--port", str(port),
                                    "--bridge", bridge_dir], stdout=logs["ui"], stderr=subprocess.STDOUT)
    for _ in range(100):
        try:
            urllib.request.urlopen(base + "/status", timeout=1); break
        except OSError:
            time.sleep(0.1)
    for b in ("pause", "resume", "stop"):
        T_OUT.append(time.time())
        post(f"/{b}")
        until(lambda b=b: any(r[0] == b for r in REQ), 15)
    check("page Pause / Resume / Stop -> bridge -> /task/control: exactly 'pause', 'resume', 'stop', whole controller "
          "(job_id '')", REQ == [("pause", "", 0), ("resume", "", 0), ("stop", "", 0)],
          f"{REQ}; button press -> controller request {[round((i - o) * 1000) for o, i in zip(T_OUT, T_IN)]} ms")
    ev = os.path.join(bridge_dir, "events.txt")
    post("/now")
    started = until(lambda: "CMD inject vacuum_loss -> started" in open(ev).read(), 10)
    ran = until(lambda: "inject vacuum_loss:" in open(ev).read(), 20)    # the tool's own output (no gripper here)
    check("page Now -> bridge starts tools/inject_vacuum_loss.py (here refused / no gripper: nothing switched)",
          started and ran, [l.strip() for l in open(ev) if "inject" in l][-3:])
finally:
    for p in procs.values():
        p.terminate()
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
    for f in logs.values():
        f.close()
    ex.shutdown(); n.destroy_node(); rclpy.shutdown()
print(f"{ok} passed, {bad} failed")
sys.exit(1 if bad else 0)
