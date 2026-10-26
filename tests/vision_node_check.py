"""vision_transform node checks (V4): real node as a subprocess, temp calibration file, /vision/pixel_to_table.

    ROS_DOMAIN_ID=78 pixi run python3 tests/vision_node_check.py

Placeholder -> refused; new calibration picked up live; values = calib.apply; inside flag; every rejection; broken
file -> refused (old calibration NOT kept); recovery; deleted file; bad parameter; startup without the file; latency."""
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy

from ament_index_python.packages import get_package_prefix

from hk_fr5_vision import calib
from hk_fr5_vision_interfaces.srv import PixelToTable

ok = bad = 0
def check(name, cond, extra=""):
    global ok, bad
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True)
    ok += bool(cond); bad += not cond

PERIOD = 0.2
# lab homography (pixel -> m) as the true camera, 6 tags (same as calib_check)
H = np.diag([0.001, 0.001, 1.0]) @ np.array([[-0.0030423858564885085, -1.5204218890076686, 448.12361309443975],
                                              [-1.3693471816224512, 0.017101743089264865, 1368.8769844406138],
                                              [-1.2643839849879774e-06, 0.0002496750503579246, 1.0]])
HI = np.linalg.inv(H)
def to_px(x, y):
    q = HI @ np.array([x, y, 1.0]); return q[0] / q[2], q[1] / q[2]
WORLD = {14: (0.0478, 0.8366), 13: (0.0893, -0.8626), 0: (-0.6635, -0.5954), 15: (-0.6224, 0.6732),
         6: (-0.30, 0.30), 7: (-0.30, -0.30)}
C6 = calib.fit([{"id": i, "u": to_px(*p)[0], "v": to_px(*p)[1], "x": p[0], "y": p[1]} for i, p in WORLD.items()],
               (1920, 1080), "test")

rclpy.init(); n = rclpy.create_node("vision_node_test")
cli = n.create_client(PixelToTable, "/vision/pixel_to_table")
def call(u, v, timeout=3.0):
    f = cli.call_async(PixelToTable.Request(u=u, v=v)); end = time.time() + timeout
    while not f.done() and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.01)
    return f.result()
NODE = os.path.join(get_package_prefix("hk_fr5_vision"), "lib", "hk_fr5_vision", "vision_transform")
clean_exit = []
def start(*args):   # the node itself (not `ros2 run`), own process group: SIGINT reaches it, nothing left behind
    return subprocess.Popen([NODE, "--ros-args", *args], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            start_new_session=True)
def stop(p):
    os.killpg(p.pid, signal.SIGINT)   # = Ctrl+C
    try:
        out, _ = p.communicate(timeout=5); clean_exit.append(p.returncode)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL); out, _ = p.communicate(); clean_exit.append("killed")
    return out
def settle():
    time.sleep(PERIOD * 3)

centre, far = to_px(-0.30, 0.0), to_px(-0.90, 0.0)
with tempfile.TemporaryDirectory() as d:
    path = os.path.join(d, "calib.yaml")
    open(path, "w").write("version: 1\ncalibrated: false\n")
    p = start("-p", f"calib_file:={path}", "-p", f"reload_period:={PERIOD}")
    try:
        check("service advertised", cli.wait_for_service(timeout_sec=10))
        r = call([centre[0]], [centre[1]])
        check("placeholder: refused, says not calibrated", r and not r.success and "not calibrated" in r.message,
              r and r.message)

        calib.save(path, C6); settle()
        r = call([centre[0], far[0]], [centre[1], far[1]])
        ex, ey, ein = calib.apply(C6, [centre[0], far[0]], [centre[1], far[1]])
        check("new calibration picked up live; x / y = calib.apply, inside [True, False]",
              r and r.success and np.allclose(r.x, ex) and np.allclose(r.y, ey) and list(r.inside) == [True, False]
              and abs(r.x[0] + 0.30) < 1e-6 and abs(r.y[0]) < 1e-6, r and f"{r.message}; x {list(r.x)} y {list(r.y)}")

        for name, u, v, text in [("empty", [], [], "no points"), ("length mismatch", [1.0, 2.0], [1.0], "lengths differ"),
                                 ("NaN", [float("nan")], [1.0], "finite"), ("inf", [1.0], [float("inf")], "finite")]:
            r = call(u, v)
            check(f"rejects {name}", r and not r.success and text in r.message and not list(r.x), r and r.message)

        open(path, "w").write("version: 1\ncalibrated: true\nhomography: [[1, 2], [3, 4]]\n"); settle()
        r = call([centre[0]], [centre[1]])
        check("broken file: refused (old calibration not kept), reason given",
              r and not r.success and "bad" in r.message, r and r.message)

        calib.save(path, C6); settle()
        r = call([centre[0]], [centre[1]])
        check("good file again: recovers", r and r.success and abs(r.x[0] + 0.30) < 1e-6, r and r.message)

        os.remove(path); settle()
        r = call([centre[0]], [centre[1]])
        check("file deleted: refused, cannot read", r and not r.success and "cannot read" in r.message, r and r.message)

        calib.save(path, C6); settle()
        pts = [to_px(x, y) for x in np.linspace(-0.6, 0.0, 10) for y in np.linspace(-0.7, 0.7, 10)]
        t = time.time(); r = call([q[0] for q in pts], [q[1] for q in pts]); dt100 = (time.time() - t) * 1000
        t = time.time()
        for _ in range(50):
            call([centre[0]], [centre[1]])
        dt1 = (time.time() - t) / 50 * 1000
        check("latency: 1 point < 20 ms mean, 100 points in one call < 50 ms", r and r.success and dt1 < 20 and dt100 < 50,
              f"1 point {dt1:.1f} ms, 100 points {dt100:.1f} ms")
    finally:
        out = stop(p)
    check("node log: loaded / no calibration lines, no traceback",
          "calibration loaded" in out and "no calibration" in out and "Traceback" not in out)
    lines = [l for l in out.splitlines() if "no calibration" in l]
    check("each problem logged once (no log spam every reload_period)", len(lines) <= 4, f"{len(lines)} lines")

    # startup without the file, then the file appears
    missing = os.path.join(d, "later.yaml")
    p = start("-p", f"calib_file:={missing}", "-p", f"reload_period:={PERIOD}")
    try:
        cli.wait_for_service(timeout_sec=10)
        r = call([centre[0]], [centre[1]])
        check("startup without the file: runs, refuses", r and not r.success and "cannot read" in r.message, r and r.message)
        calib.save(missing, C6); settle()
        r = call([centre[0]], [centre[1]])
        check("file appears later: picked up", r and r.success, r and r.message)
    finally:
        stop(p)

# bad parameter -> exit 1 with a clear message
p = start("-p", "reload_period:=-1.0")
try:
    out, _ = p.communicate(timeout=15)
except subprocess.TimeoutExpired:
    p.kill(); out, _ = p.communicate()
check("reload_period -1 -> exit 1, clear error", p.returncode == 1 and "'reload_period' must be > 0" in out,
      f"rc {p.returncode}: {out.strip()[-120:]}")

check("Ctrl+C (SIGINT): node exits by itself within 5 s, rc 0", clean_exit == [0, 0], str(clean_exit))
n.destroy_node(); rclpy.shutdown()
print(f"{ok} passed, {bad} failed")
sys.exit(1 if bad else 0)
