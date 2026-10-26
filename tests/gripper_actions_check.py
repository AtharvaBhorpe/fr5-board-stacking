"""Runtime test of grip / release actions (step 20c): fake mode + real code path vs mock XML-RPC."""
from testenv import MOCK, SHELL, GRIPPER_CFG, LOG_DIR  # noqa: F401 (chdir to the log dir)
import math, os, signal, subprocess, sys, time, xmlrpc.client
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from action_msgs.msg import GoalStatus
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as P, ParameterValue, ParameterType
from hk_fr5_interfaces.action import Grip, Release
from hk_fr5_interfaces.msg import GripperState as S, GripperErrorCode as C
from ament_index_python.packages import get_package_share_directory
CFG = os.path.join(get_package_share_directory("hk_fr5_gripper"), "config", "gripper.yaml")
PORT = 20916
ok = 0
def check(name, cond, extra=""):
    global ok
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True); ok += bool(cond)
    if not cond: raise SystemExit(1)
rclpy.init(); node = rclpy.create_node("gripper_action_test")
msgs = []
node.create_subscription(S, "/gripper/state", msgs.append, QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL))
grip_c, rel_c = ActionClient(node, Grip, "/gripper/grip"), ActionClient(node, Release, "/gripper/release")
par_c = node.create_client(SetParameters, "/gripper/gripper_controller/set_parameters")
def spin(sec):
    end = time.monotonic() + sec
    while time.monotonic() < end: rclpy.spin_once(node, timeout_sec=0.01)
def until(fut, timeout=10.0):
    end = time.monotonic() + timeout
    while not fut.done() and time.monotonic() < end: rclpy.spin_once(node, timeout_sec=0.01)
    return fut.result()
def send(client, goal, fb=None):
    h = until(client.send_goal_async(goal, feedback_callback=(lambda m: fb.append(m.feedback)) if fb is not None else None))
    return h
def run(client, goal, fb=None):
    t = time.monotonic(); h = send(client, goal, fb); res = until(h.get_result_async())
    return res.status, res.result, time.monotonic() - t
def set_fake_vacuum(v):
    req = SetParameters.Request(parameters=[P(name="fake_vacuum", value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=v))])
    until(par_c.call_async(req))
def last(): spin(0.15); return msgs[-1]
def start(*args):
    return subprocess.Popen(["ros2", "run", "hk_fr5_gripper", "gripper_controller", "--ros-args", "--params-file", CFG, *args],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
def stop(p):
    os.killpg(p.pid, signal.SIGINT); return p.communicate(timeout=10)[0]
def wait_servers():
    assert grip_c.wait_for_server(timeout_sec=10) and rel_c.wait_for_server(timeout_sec=10) and par_c.wait_for_service(timeout_sec=10)

# ---------------- fake mode ----------------
p = start(); wait_servers(); spin(0.3)
fb = []
st, r, dt = run(grip_c, Grip.Goal(), fb)
m = last()
check("grip: SUCCESS, time_to_vacuum ~0.5-0.7 s, feedback sent", st == GoalStatus.STATUS_SUCCEEDED and r.success and r.error_code.val == C.SUCCESS
      and 0.5 <= r.time_to_vacuum <= 0.7 and len(fb) >= 5, f"{r.message}; ttv {r.time_to_vacuum:.3f}; {len(fb)} fb")
check("grip: state HOLDING, pump on, blow-off off, vacuum", m.state == S.HOLDING and m.vacuum_on and not m.blow_off_on and m.vacuum_achieved)
st, r, dt = run(grip_c, Grip.Goal())
check("grip while HOLDING: SUCCESS at once, nothing switched (W-5)", r.success and r.time_to_vacuum == 0.0 and dt < 0.3, f"{r.message}; {dt:.3f} s")
n0 = len(msgs)
st, r, dt = run(rel_c, Release.Goal())
m = last(); seq = msgs[n0:]
check("release: SUCCESS after ~1.0 s", st == GoalStatus.STATUS_SUCCEEDED and r.success and 0.95 <= dt <= 1.3, f"{r.message}; {dt:.2f} s")
check("release: RELEASING with blow-off on + pump off seen, then IDLE all off",
      any(x.state == S.RELEASING and x.blow_off_on and not x.vacuum_on for x in seq) and m.state == S.IDLE
      and not (m.vacuum_on or m.blow_off_on or m.vacuum_achieved), m.message)
st, r, dt = run(rel_c, Release.Goal(blow_off_time=0.3))
check("release with nothing held + blow_off_time 0.3: SUCCESS ~0.3 s (W-6)", r.success and 0.28 <= dt <= 0.6, f"{dt:.2f} s")
set_fake_vacuum(False)
st, r, dt = run(grip_c, Grip.Goal(timeout=1.0))
m = last()
check("grip, no vacuum, timeout 1.0: NO_VACUUM ~1.0 s, pump off, IDLE", st == GoalStatus.STATUS_ABORTED and r.error_code.val == C.NO_VACUUM
      and not r.success and r.time_to_vacuum == -1.0 and 0.95 <= dt <= 1.3 and m.state == S.IDLE and not m.vacuum_on, f"{r.message}; {dt:.2f} s")
st, r, dt = run(grip_c, Grip.Goal())
check("grip, no vacuum, default timeout: NO_VACUUM ~3.0 s (grip_timeout)", r.error_code.val == C.NO_VACUUM and 2.95 <= dt <= 3.4, f"{dt:.2f} s")
for goal, name in ((Grip.Goal(timeout=-1.0), "timeout -1"), (Grip.Goal(timeout=math.nan), "timeout nan")):
    st, r, dt = run(grip_c, goal)
    check(f"grip {name}: INVALID_GOAL", r.error_code.val == C.INVALID_GOAL and st == GoalStatus.STATUS_ABORTED, r.message)
st, r, dt = run(rel_c, Release.Goal(blow_off_time=-0.5))
check("release blow_off_time -0.5: INVALID_GOAL", r.error_code.val == C.INVALID_GOAL, r.message)
# cancel grip
h = send(grip_c, Grip.Goal(timeout=5.0)); spin(0.5); until(h.cancel_goal_async()); res = until(h.get_result_async()); m = last()
check("cancel grip: CANCELED, CANCELLED code, pump off, IDLE (W-3)", res.status == GoalStatus.STATUS_CANCELED and res.result.error_code.val == C.CANCELLED
      and not m.vacuum_on and m.state == S.IDLE, res.result.message)
# release preempts grip
h = send(grip_c, Grip.Goal(timeout=5.0)); spin(0.4)
st, r, dt = run(rel_c, Release.Goal(blow_off_time=0.3)); g = until(h.get_result_async()); m = last()
check("release during grip: grip PREEMPTED, release SUCCESS, all off (W-7)", g.result.error_code.val == C.PREEMPTED and g.status == GoalStatus.STATUS_ABORTED
      and r.success and m.state == S.IDLE and not (m.vacuum_on or m.blow_off_on), f"{g.result.message} | {r.message}")
# grip preempts release
set_fake_vacuum(True)
h = send(rel_c, Release.Goal(blow_off_time=2.0)); spin(0.3)
n0 = len(msgs); st, r, dt = run(grip_c, Grip.Goal()); rl = until(h.get_result_async()); m = last()
check("grip during release: release PREEMPTED, grip SUCCESS, blow-off off, HOLDING", rl.result.error_code.val == C.PREEMPTED and r.success
      and m.state == S.HOLDING and m.vacuum_on and not m.blow_off_on, f"{rl.result.message} | {r.message}")
# cancel release
h = send(rel_c, Release.Goal(blow_off_time=3.0)); spin(0.4); t = time.monotonic(); until(h.cancel_goal_async()); res = until(h.get_result_async()); m = last()
check("cancel release: CANCELED, blow-off off at once, IDLE", res.status == GoalStatus.STATUS_CANCELED and res.result.error_code.val == C.CANCELLED
      and not (m.vacuum_on or m.blow_off_on) and m.state == S.IDLE and time.monotonic() - t < 0.5, res.result.message)
# PART_LOST then recover via grip
st, r, dt = run(grip_c, Grip.Goal()); set_fake_vacuum(False); spin(0.4); m = last()
check("vacuum lost while HOLDING: PART_LOST, pump on (W-4)", m.state == S.PART_LOST and m.vacuum_on, m.message)
set_fake_vacuum(True); st, r, dt = run(grip_c, Grip.Goal()); m = last()
check("grip from PART_LOST (pump still on): SUCCESS after 2 fresh debounce reads, HOLDING (clears)",
      r.success and 0.09 <= r.time_to_vacuum <= 0.2 and m.state == S.HOLDING, r.message)
# shutdown during blow-off
send(rel_c, Release.Goal(blow_off_time=3.0)); spin(0.3)
out = stop(p)
check("Ctrl+C during release: both pumps off, clean exit", "vacuum pump (DO0) off" in out and "blow-off pump (DO1) off" in out
      and "Traceback" not in out and p.returncode == 0, f"rc={p.returncode}")

# ---------------- real code path vs mock ----------------
REAL = ("-p", "use_fake_hardware:=false", "-p", "robot_ip:=127.0.0.1", "-p", f"xmlrpc_port:={PORT}")
def mock_up():
    m = subprocess.Popen([sys.executable, "-I", MOCK, str(PORT), "0", "0"], stdout=subprocess.PIPE, text=True); m.stdout.readline(); return m
mock = mock_up(); rpc = xmlrpc.client.ServerProxy(f"http://127.0.0.1:{PORT}/RPC2"); rpc._auto_vac(True)
msgs.clear(); p = start(*REAL); wait_servers(); spin(0.3)
def sets(): return [tuple(c[1:]) for c in rpc._log() if c[0] == "SetToolDO"]
st, r, dt = run(grip_c, Grip.Goal())
check("real grip: SetToolDO(1,0) then (0,1), SUCCESS", sets() == [(1, 0), (0, 1)] and r.success, f"{sets()}; {r.message}")
st, r, dt = run(rel_c, Release.Goal())
check("real release: SetToolDO(0,0), (1,1), (1,0), SUCCESS", sets()[2:] == [(0, 0), (1, 1), (1, 0)] and r.success, f"{sets()[2:]}")
rpc._auto_vac(False)
st, r, dt = run(grip_c, Grip.Goal(timeout=0.5))
check("real grip no vacuum: (1,0), (0,1), then (0,0), NO_VACUUM", sets()[5:] == [(1, 0), (0, 1), (0, 0)] and r.error_code.val == C.NO_VACUUM, f"{sets()[5:]}")
rpc._set_err(14); st, r, dt = run(grip_c, Grip.Goal()); m = last()
check("real grip, controller error: CONTROLLER_ERROR result + state", r.error_code.val == C.CONTROLLER_ERROR and m.state == S.CONTROLLER_ERROR
      and "14" in r.message, r.message)
rpc._set_err(0); rpc._auto_vac(True)
st, r, dt = run(grip_c, Grip.Goal()); m = last()
check("real: new grip clears CONTROLLER_ERROR -> SUCCESS, HOLDING", r.success and m.state == S.HOLDING, r.message)
st, r, dt = run(rel_c, Release.Goal(blow_off_time=0.2))
mock.terminate(); mock.wait(); spin(0.3)
st, r, dt = run(grip_c, Grip.Goal()); m = last()
check("real grip, robot unreachable: COMM_FAILURE at once", r.error_code.val == C.COMM_FAILURE and dt < 1.0 and m.state == S.COMM_ERROR, f"{r.message}; {dt:.2f} s")
mock = mock_up(); rpc = xmlrpc.client.ServerProxy(f"http://127.0.0.1:{PORT}/RPC2")
m = None
for _ in range(40):
    spin(0.1)
    if msgs[-1].state != S.COMM_ERROR: m = msgs[-1]; break
check("real: robot back -> COMM_ERROR clears by itself to IDLE", m is not None and m.state == S.IDLE, m and m.message)
h = send(grip_c, Grip.Goal(timeout=2.0)); spin(0.3); mock.terminate(); mock.wait()
res = until(h.get_result_async()); m = last()
check("real: comm lost during grip -> COMM_FAILURE at timeout (not NO_VACUUM)", res.result.error_code.val == C.COMM_FAILURE and m.state == S.COMM_ERROR,
      res.result.message)
out = stop(p)
check("real: clean exit", p.returncode == 0 and "Traceback" not in out, f"rc={p.returncode}")
print(f"{ok} checks passed")
node.destroy_node(); rclpy.shutdown()
