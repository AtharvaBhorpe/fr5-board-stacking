"""Offline gripper I/O checks (step 20b): Debounce, parse_reply, FakeIo, XmlRpcIo against mock_fairino.py
(read/write, controller error, timeout, connection refused). No ROS graph needed."""
from testenv import MOCK, SHELL, GRIPPER_CFG, LOG_DIR  # noqa: F401 (chdir to the log dir)
import subprocess, sys, time, xmlrpc.client
from hk_fr5_gripper.gripper_io import Debounce, FakeIo, XmlRpcIo, CommError, ControllerError, parse_reply
ok = 0
def check(name, cond):
    global ok
    print(("PASS " if cond else "FAIL ") + name); ok += bool(cond)
    if not cond: sys.exit(1)
# Debounce
d = Debounce(2); check("debounce first read taken", d.update(False) and d.value is False)
check("one True read ignored", not d.update(True) and d.value is False)
check("glitch reset", not d.update(False) and not d.update(True) and d.value is False)
check("second True -> changes", d.update(True) and d.value is True)
d1 = Debounce(1); d1.update(False); check("reads=1 immediate", d1.update(True))
# parse_reply
check("parse [0, 1]", parse_reply("X", [0, 1]) == 1)
check("parse bare int", parse_reply("X", 5) == 5)
try: parse_reply("X", [14, 0]); check("controller error raised", False)
except ControllerError as e: check(f"controller error code 14 ({e})", e.code == 14)
for bad in ("abc", True, [0, "x"], None):
    try: parse_reply("X", bad); check(f"bad reply {bad!r}", False)
    except CommError: check(f"bad reply {bad!r} -> CommError", True)
# FakeIo
flag = [True]; f = FakeIo(0, 0, 0.2, lambda: flag[0])
check("fake: DI 0 with pump off", not f.get_di(0))
f.set_do(0, True); check("fake: DI 0 before delay", not f.get_di(0) and f.get_do(0))
time.sleep(0.25); check("fake: DI 1 after delay", f.get_di(0))
flag[0] = False; check("fake: fake_vacuum false -> DI 0", not f.get_di(0))
flag[0] = True; f.set_do(0, False); check("fake: pump off -> DI 0", not f.get_di(0))
f.set_do(1, True); check("fake: DO1 readback", f.get_do(1) and not f.get_do(0))
# XmlRpcIo vs mock
srv = subprocess.Popen([sys.executable, "-I", MOCK, "20913", "2", "1"], stdout=subprocess.PIPE, text=True)
srv.stdout.readline()
try:
    io = XmlRpcIo("127.0.0.1", 20913, 0.5)
    check("real: GetToolDO mask 2 -> DO0 off, DO1 on", not io.get_do(0) and io.get_do(1))
    check("real: GetToolDI DI0 = 1", io.get_di(0) and not io.get_di(1))
    io.set_do(0, True); io.set_do(1, False)
    check("real: SetToolDO + readback", io.get_do(0) and not io.get_do(1))
    m = xmlrpc.client.ServerProxy("http://127.0.0.1:20913/RPC2")
    check("real: calls logged", m._log()[-2:] == [["SetToolDO", 0, 1], ["SetToolDO", 1, 0]])
    m._set_err(14)
    try: io.set_do(0, False); check("real: SetToolDO error", False)
    except ControllerError as e: check(f"real: SetToolDO error -> ControllerError ({e})", e.code == 14)
    try: io.get_di(0); check("real: GetToolDI error", False)
    except ControllerError: check("real: GetToolDI error -> ControllerError", True)
    m._set_err(0); m._set_delay(1.0)
    t = time.monotonic()
    try: io.get_di(0); check("real: timeout", False)
    except CommError as e: check(f"real: slow server -> CommError after {time.monotonic()-t:.2f} s", time.monotonic() - t < 0.8)
    m._set_delay(0.0); time.sleep(1.1)
    check("real: works again after timeout", io.get_di(0))
finally:
    srv.terminate(); srv.wait()
t = time.monotonic()
try: XmlRpcIo("127.0.0.1", 20913, 0.5).get_di(0); check("refused", False)
except CommError as e: check(f"real: server down -> CommError in {time.monotonic()-t:.3f} s ({e})", True)
print(f"{ok} checks passed")
