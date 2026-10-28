"""Inject a vacuum failure while the robot holds a board (demo / test of the part-lost recovery). Separate from the
gripper node: our gripper_controller sees the vacuum signal drop like a real loss -> PART_LOST -> the skill stops,
the board counts as lost, the controller re-decides; a board that fell on the table is a new input board.

    pixi run python3 tools/inject_vacuum_loss.py [SECS] [--force]       (or: tc inject vacuum_loss [SECS])

Mode from the running gripper node's parameters (use_fake_hardware, robot_ip, xmlrpc_port, vacuum_do, blow_off_do):
  real: tool DO vacuum OFF -> blow-off ON for SECS (default 1.0) -> blow-off OFF (always, also on Ctrl+C / error).
        SWITCHES THE REAL GRIPPER: only on the user's command, a person at the e-stop.
  sim:  fake_world 'force_vacuum' = off for SECS (the fake vacuum signal drops).
Refused unless the gripper is HOLDING (--force skips that check). Never switches the vacuum ON; the gripper node's
own release afterwards switches both outputs off.
"""
import sys
import time
import xmlrpc.client

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile

from hk_fr5_interfaces.msg import GripperState

GRIPPER = "/gripper/gripper_controller"
FAKE_WORLD = "/task/fake_world"
DEFAULT_SECS = 1.0          # blow-off time: 1 s drops a board in flight (user, 2026-10-28)
PARAMS = ["use_fake_hardware", "robot_ip", "xmlrpc_port", "vacuum_do", "blow_off_do", "rpc_timeout"]


def say(text):
    print(f"{time.strftime('%H:%M:%S')} inject vacuum_loss: {text}", flush=True)


def call(n, srv, name, req, timeout=3.0):
    c = n.create_client(srv, name)
    if not c.wait_for_service(timeout_sec=timeout):
        raise SystemExit(f"inject vacuum_loss: {name} not available")
    f = c.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=timeout)
    if f.result() is None:
        raise SystemExit(f"inject vacuum_loss: {name}: no answer")
    return f.result()


def gripper_state(n, timeout=3.0):
    box = {}
    sub = n.create_subscription(GripperState, "/gripper/state", lambda m: box.update(m=m),
                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    end = time.time() + timeout
    while "m" not in box and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.destroy_subscription(sub)
    return box["m"].state if "m" in box else None


def gripper_params(n):
    vals = call(n, GetParameters, f"{GRIPPER}/get_parameters", GetParameters.Request(names=PARAMS)).values
    out = {}
    for name, v in zip(PARAMS, vals):
        out[name] = {ParameterType.PARAMETER_BOOL: v.bool_value, ParameterType.PARAMETER_INTEGER: v.integer_value,
                     ParameterType.PARAMETER_DOUBLE: v.double_value, ParameterType.PARAMETER_STRING: v.string_value
                     }.get(v.type)
        if out[name] is None:
            raise SystemExit(f"inject vacuum_loss: gripper parameter '{name}' not set")
    return out


def set_fake(n, value):
    p = Parameter(name="force_vacuum", value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value))
    r = call(n, SetParameters, f"{FAKE_WORLD}/set_parameters", SetParameters.Request(parameters=[p]))
    if not r.results[0].successful:
        raise SystemExit(f"inject vacuum_loss: fake_world force_vacuum={value!r} refused: {r.results[0].reason}")


def set_do(proxy, index, on):
    code = proxy.SetToolDO(index, 1 if on else 0, 0, 0)       # (id, status, smooth 0, block 0 = blocking)
    if code != 0:
        raise RuntimeError(f"SetToolDO({index}, {int(on)}) returned controller error {code}")


def real(p, secs):
    """Vacuum off FIRST, then blow-off (never both on); blow-off off in every case."""
    t = xmlrpc.client.Transport()
    t.timeout = p["rpc_timeout"]
    proxy = xmlrpc.client.ServerProxy(f"http://{p['robot_ip']}:{p['xmlrpc_port']}/RPC2", transport=t)
    set_do(proxy, p["vacuum_do"], False)
    say(f"real: vacuum DO{p['vacuum_do']} OFF")
    try:
        time.sleep(0.1)
        set_do(proxy, p["blow_off_do"], True)
        say(f"real: blow-off DO{p['blow_off_do']} ON for {secs:.1f} s")
        time.sleep(secs)
    finally:
        for k in range(3):
            try:
                set_do(proxy, p["blow_off_do"], False)
                say(f"real: blow-off DO{p['blow_off_do']} OFF")
                break
            except Exception as e:                            # noqa: BLE001 - retry, then report loudly
                say(f"real: blow-off OFF failed ({e}), retry {k + 1}/3")
                time.sleep(0.2)
        else:
            say("real: BLOW-OFF MAY STILL BE ON - switch it off (gripper release / pendant)")


def main():
    a = sys.argv[1:]
    force = "--force" in a
    a = [x for x in a if x != "--force"]
    secs = float(a[0]) if a else DEFAULT_SECS
    if not 0.2 <= secs <= 10.0:
        raise SystemExit("inject vacuum_loss: SECS must be 0.2 - 10")
    rclpy.init()
    n = rclpy.create_node("inject_vacuum_loss")
    try:
        st = gripper_state(n)
        if st != GripperState.HOLDING and not force:
            raise SystemExit(f"inject vacuum_loss: refused, gripper is {st or 'not publishing'} (not HOLDING); --force skips")
        p = gripper_params(n)
        if p["use_fake_hardware"]:
            say(f"sim: fake vacuum off for {secs:.1f} s (gripper {st})")
            set_fake(n, "off")
            try:
                t_end = time.time() + secs
                while time.time() < t_end:
                    rclpy.spin_once(n, timeout_sec=0.05)
            finally:
                set_fake(n, "")
                say("sim: fake vacuum back to normal")
        else:
            say(f"REAL robot {p['robot_ip']} (gripper {st})")
            real(p, secs)
        say("done")
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
