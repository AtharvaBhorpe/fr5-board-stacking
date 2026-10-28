"""tools/inject_vacuum_loss.py without a robot: stand-in gripper node (parameters + /gripper/state), a local XML-RPC
server recording SetToolDO, a stand-in fake_world (force_vacuum). Checks the real sequence (vacuum OFF before
blow-off ON, blow-off OFF at the end, also when the controller fails), the refusal when not HOLDING, the sim path.

    ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=82 pixi run python3 tests/inject_check.py
"""
import os
import subprocess
import sys
import tempfile
import threading
import time
from xmlrpc.server import SimpleXMLRPCServer

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile

from hk_fr5_interfaces.msg import GripperState

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "..", "tools", "inject_vacuum_loss.py")
ok = bad = 0


def check(name, cond, extra=""):
    global ok, bad
    ok, bad = ok + bool(cond), bad + (not cond)
    print(f"{'PASS' if cond else 'FAIL'} {name}{'  [' + str(extra) + ']' if extra else ''}", flush=True)


CALLS, FAIL = [], {"blow_on": False}


def set_tool_do(i, status, smooth, block):
    CALLS.append((round(time.time(), 2), i, status))
    if FAIL["blow_on"] and i == 1 and status == 1:
        return 14                                     # controller error while switching blow-off on
    return 0


srv = SimpleXMLRPCServer(("127.0.0.1", 0), logRequests=False, allow_none=True)
srv.register_function(set_tool_do, "SetToolDO")
PORT = srv.server_address[1]
threading.Thread(target=srv.serve_forever, daemon=True).start()

rclpy.init()
grip = rclpy.create_node("gripper_controller", namespace="gripper")
for k, v in dict(use_fake_hardware=False, robot_ip="127.0.0.1", xmlrpc_port=PORT, vacuum_do=0, blow_off_do=1,
                 rpc_timeout=0.5).items():
    grip.declare_parameter(k, v)
fw = rclpy.create_node("fake_world", namespace="task")
fw.declare_parameter("force_vacuum", "")
FV = []


def on_fw_params(ps):
    FV.extend((round(time.time(), 2), p.value) for p in ps)
    return SetParametersResult(successful=True)


fw.add_on_set_parameters_callback(on_fw_params)
pub = grip.create_publisher(GripperState, "/gripper/state", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
ex = SingleThreadedExecutor(); ex.add_node(grip); ex.add_node(fw)
threading.Thread(target=ex.spin, daemon=True).start()
log_dir = tempfile.mkdtemp(prefix="inject_check_")


def state(s):
    pub.publish(GripperState(state=s))
    time.sleep(0.3)


def run(*args, name="run"):
    """The tool as its own process, output into a log file (never stdout=PIPE read at the end)."""
    path = os.path.join(log_dir, f"{name}.log")
    with open(path, "w") as f:
        t0 = time.time()
        rc = subprocess.run([sys.executable, SCRIPT, *args], stdout=f, stderr=subprocess.STDOUT, timeout=30).returncode
    return rc, time.time() - t0, open(path).read()


try:
    state(GripperState.IDLE)
    rc, _, out = run("1.0", name="refused")
    check("not HOLDING: refused, nothing switched", rc != 0 and "refused" in out and not CALLS, out.strip()[-120:])

    state(GripperState.HOLDING)
    rc, dt, out = run(name="real")                      # no SECS: the default (1.0 s, user)
    seq = [(i, s) for _, i, s in CALLS]
    on = next((t for t, i, s in CALLS if (i, s) == (1, 1)), None)
    off = next((t for t, i, s in CALLS if (i, s) == (1, 0)), None)
    check("real: vacuum DO0 OFF, then blow-off DO1 ON, then DO1 OFF (never DO0 ON)",
          rc == 0 and seq == [(0, 0), (1, 1), (1, 0)], seq)
    check("real: blow-off on for the default 1.0 s (+- 0.3)", on is not None and off is not None and abs(off - on - 1.0) < 0.3,
          f"{(off or 0) - (on or 0):.2f} s")

    CALLS.clear(); FAIL["blow_on"] = True
    rc, _, out = run("1.0", name="real_fail")
    seq = [(i, s) for _, i, s in CALLS]
    check("real: controller error on blow-off ON -> still blow-off OFF at the end, exit != 0",
          rc != 0 and seq[:1] == [(0, 0)] and seq[-1] == (1, 0), seq)
    FAIL["blow_on"] = False

    CALLS.clear()
    grip.set_parameters([rclpy.parameter.Parameter("use_fake_hardware", value=True)])
    rc, dt, out = run("1.0", name="sim")
    vals = [v for _, v in FV]
    check("sim: fake_world force_vacuum 'off' then '' after SECS, no XML-RPC",
          rc == 0 and vals == ["off", ""] and not CALLS and abs(FV[1][0] - FV[0][0] - 1.0) < 0.4, f"{FV} calls {CALLS}")

    rc, _, out = run("0.05", name="bad_secs")
    check("SECS out of range refused", rc != 0 and "SECS" in out, out.strip()[-80:])
finally:
    ex.shutdown(); srv.shutdown()
    grip.destroy_node(); fw.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
print(f"{ok} passed, {bad} failed")
sys.exit(1 if bad else 0)
