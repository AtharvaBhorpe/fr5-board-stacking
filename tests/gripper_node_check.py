"""Runtime test of gripper_controller (step 20b, start reset + interlock H-7 / H-8): fake mode + real code path vs
mock XML-RPC server."""
from testenv import MOCK, SHELL, GRIPPER_CFG, LOG_DIR  # noqa: F401 (chdir to the log dir)
import os, signal, subprocess, sys, time, xmlrpc.client
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from hk_fr5_interfaces.msg import GripperState as S, GripperErrorCode as C
from hk_fr5_interfaces.action import Grip
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory
CFG = os.path.join(get_package_share_directory("hk_fr5_gripper"), "config", "gripper.yaml")
PORT = 20914
ok = 0
def check(name, cond, extra=""):
    global ok
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True); ok += bool(cond)
    if not cond: raise SystemExit(1)

rclpy.init(); node = rclpy.create_node("gripper_test")
msgs = []
node.create_subscription(S, "/gripper/state", msgs.append,
                         QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
def wait_for(pred, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if msgs and pred(msgs[-1]): return msgs[-1]
    return msgs[-1] if msgs else None
grip_c = ActionClient(node, Grip, "/gripper/grip")
def until(fut, timeout=10.0):
    end = time.monotonic() + timeout
    while not fut.done() and time.monotonic() < end: rclpy.spin_once(node, timeout_sec=0.01)
    return fut.result()
def grip(timeout=1.0):
    grip_c.wait_for_server(timeout_sec=10.0)
    return until(until(grip_c.send_goal_async(Grip.Goal(timeout=timeout))).get_result_async()).result
def start(*args):
    return subprocess.Popen(["ros2", "run", "hk_fr5_gripper", "gripper_controller", "--ros-args",
                             "--params-file", CFG, *args], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
def stop(p):
    os.killpg(p.pid, signal.SIGINT); out = p.communicate(timeout=10)[0]; return p.returncode, out

# 1. bad configs -> exit 1 with a clear message
p = subprocess.run(["ros2", "run", "hk_fr5_gripper", "gripper_controller"], capture_output=True, text=True, timeout=20)
check("no params file -> exit 1", p.returncode == 1 and "missing" in p.stdout, p.stdout.strip())
p = subprocess.run(["ros2", "run", "hk_fr5_gripper", "gripper_controller", "--ros-args", "--params-file", CFG,
                    "-p", "blow_off_do:=0"], capture_output=True, text=True, timeout=20)
check("vacuum_do == blow_off_do -> exit 1", p.returncode == 1 and "must differ" in p.stdout, p.stdout.strip())

# 2. fake mode
p = start()
m = wait_for(lambda m: True)
check("fake: start reset -> IDLE, outputs off", m and m.state == S.IDLE and "start:" in m.message and not (m.vacuum_on or m.blow_off_on or m.vacuum_achieved),
      m and m.message)
n0 = len(msgs); t = time.monotonic(); wait_for(lambda m: len(msgs) >= n0 + 10, 3.0)
rate = (len(msgs) - n0) / (time.monotonic() - t)
check(f"fake: state rate ~10 Hz", 8.0 < rate < 12.5, f"{rate:.1f} Hz")
rc, out = stop(p)
check("fake: Ctrl+C -> both pumps off logged", "vacuum pump (DO0) off" in out and "blow-off pump (DO1) off" in out, out.strip().splitlines()[-1])
check("fake: clean exit (rc 0, no rosout warning, no traceback)", rc == 0 and "Failed to publish" not in out and "Traceback" not in out, f"rc={rc}")

# 3. real code path vs mock: controller has DO0 on + DI0 = 1 at node start (board held)
msgs.clear()
def mock_up(do="1", di="1"):
    m = subprocess.Popen([sys.executable, "-I", MOCK, str(PORT), do, di], stdout=subprocess.PIPE, text=True)
    m.stdout.readline(); return m
def mock_down(m): m.terminate(); m.wait()
def sets(rpc): return [tuple(c[1:]) for c in rpc._log() if c[0] == "SetToolDO"]
mock = mock_up()
rpc = xmlrpc.client.ServerProxy(f"http://127.0.0.1:{PORT}/RPC2")
REAL = ("-p", "use_fake_hardware:=false", "-p", "robot_ip:=127.0.0.1", "-p", f"xmlrpc_port:={PORT}")
p = start(*REAL)
m = wait_for(lambda m: "start:" in m.message)
check("real: start reset -> IDLE, both DO published off (H-7)", m and m.state == S.IDLE and not m.vacuum_on
      and not m.blow_off_on, m and m.message)
check("real: start reset = SetToolDO(0, 0), (1, 0); controller DO mask 0", sets(rpc) == [(0, 0), (1, 0)] and rpc._do() == 0,
      f"{sets(rpc)}, mask {rpc._do()}")
rpc._auto_vac(True)
r = grip()
m = wait_for(lambda m: m.state == S.HOLDING)
check("real: grip -> interlock: SetToolDO(1, 0) before (0, 1) (H-8), HOLDING", r.success and sets(rpc)[2:] == [(1, 0), (0, 1)]
      and m.state == S.HOLDING and m.vacuum_on and not m.blow_off_on, f"{sets(rpc)[2:]}; {r.message}")
mock_down(mock)
m = wait_for(lambda m: m.state == S.COMM_ERROR)
check("real: server down while HOLDING -> COMM_ERROR", m.state == S.COMM_ERROR, m.message)
mock = mock_up("1", "1"); rpc = xmlrpc.client.ServerProxy(f"http://127.0.0.1:{PORT}/RPC2"); t = time.monotonic()
m = wait_for(lambda m: m.state != S.COMM_ERROR, 4.0)
check("real: server back, vacuum still there -> HOLDING by itself (W-13), nothing switched", m.state == S.HOLDING
      and not sets(rpc), f"{m.message}; {time.monotonic()-t:.2f} s")
mock_down(mock); wait_for(lambda m: m.state == S.COMM_ERROR)
mock = mock_up("1", "0"); rpc = xmlrpc.client.ServerProxy(f"http://127.0.0.1:{PORT}/RPC2")
m = wait_for(lambda m: m.state != S.COMM_ERROR, 4.0)
check("real: server back, vacuum gone -> PART_LOST", m.state == S.PART_LOST and not m.vacuum_achieved, m.message)
rpc._set_di(0, 1); time.sleep(0.3); rpc._set_di(0, 0)
rpc._set_err(14)
m = wait_for(lambda m: m.state == S.CONTROLLER_ERROR)
check("real: controller error code -> CONTROLLER_ERROR", m.state == S.CONTROLLER_ERROR and "14" in m.message, m.message)
time.sleep(1.0); rclpy.spin_once(node, timeout_sec=0.05)
rpc._set_err(0)
mock_down(mock); m = wait_for(lambda m: m.state == S.COMM_ERROR)
mock = mock_up("0", "0"); rpc = xmlrpc.client.ServerProxy(f"http://127.0.0.1:{PORT}/RPC2")
m = wait_for(lambda m: m.state != S.COMM_ERROR, 4.0)
check("real: CONTROLLER_ERROR -> COMM_ERROR -> back: CONTROLLER_ERROR kept (W-12), DO0 = last commanded (on)",
      m.state == S.CONTROLLER_ERROR and m.vacuum_on, m.message)
rc, out = stop(p)
check("real: Ctrl+C -> SetToolDO(0, 0) + SetToolDO(1, 0) (W-10)", sets(rpc) == [(0, 0), (1, 0)], str(sets(rpc)))
check("real: repeated controller error logged once", out.count("CONTROLLER_ERROR: GetToolDI") == 1, f"{out.count('CONTROLLER_ERROR: GetToolDI')}x")
check("real: no comm error log spam", out.count("COMM_ERROR") == 3, f"{out.count('COMM_ERROR')}x for 3 outages")
check("real: never called GetToolDO (H-2)", "GetToolDO" not in out)
check("real: clean exit", rc == 0 and "Traceback" not in out and "Failed to publish" not in out, f"rc={rc}")
print(out)
mock_down(mock)

# 4. start reset fails with a controller error code: goals refused, reset retried until it works (H-7)
msgs.clear()
mock = mock_up("1", "1"); rpc = xmlrpc.client.ServerProxy(f"http://127.0.0.1:{PORT}/RPC2"); rpc._set_err(14)
p = start(*REAL, "-p", "comm_retry_period:=0.5")
m = wait_for(lambda m: m.state == S.CONTROLLER_ERROR)
check("real: SetToolDO error at start -> CONTROLLER_ERROR", m and m.state == S.CONTROLLER_ERROR and "14" in m.message, m and m.message)
n = len(sets(rpc)); r = grip()
check("real: grip before the start reset -> COMM_FAILURE, nothing switched", r.error_code.val == C.COMM_FAILURE
      and "start reset" in r.message and len(sets(rpc)) - n <= 1, r.message)
rpc._set_err(0)
m = wait_for(lambda m: "start:" in m.message, 3.0)
check("real: error gone -> start reset retried -> IDLE, DO mask 0", m.state == S.IDLE and rpc._do() == 0, m.message)
rc, out = stop(p); mock_down(mock)
check("real: start-reset retry not log-spammed", out.count("CONTROLLER_ERROR: SetToolDO") == 1, f"{out.count('CONTROLLER_ERROR: SetToolDO')}x")

# 5. unreachable at start -> COMM_ERROR, goals refused; robot appears -> start reset runs (H-7 + W-13)
msgs.clear()
p = start(*REAL)
m = wait_for(lambda m: m.state == S.COMM_ERROR)
check("real: unreachable at start -> COMM_ERROR", m and m.state == S.COMM_ERROR, m and m.message)
r = grip()
check("real: grip while unreachable at start -> COMM_FAILURE (start reset not done)", r.error_code.val == C.COMM_FAILURE
      and "start reset" in r.message, r.message)
mock = mock_up("1", "1"); rpc = xmlrpc.client.ServerProxy(f"http://127.0.0.1:{PORT}/RPC2")
m = wait_for(lambda m: m.state != S.COMM_ERROR, 4.0)
check("real: robot appears -> start reset -> IDLE, SetToolDO(0, 0), (1, 0)", m.state == S.IDLE and "start:" in m.message
      and sets(rpc) == [(0, 0), (1, 0)], f"{m.message}; {sets(rpc)}")
mock_down(mock)
time.sleep(0.5); t = time.monotonic(); rc, out = stop(p)
check("real: unreachable at Ctrl+C -> 'NOT switched off' logged, exits", "NOT switched off" in out and time.monotonic() - t < 5,
      f"{time.monotonic()-t:.2f} s")
print(f"{ok} checks passed")
node.destroy_node(); rclpy.shutdown()
