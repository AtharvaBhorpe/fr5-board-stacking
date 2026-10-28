"""Arm configuration check (decision-log H-11, sim). Shape rule (user): base -> tcp rises then dips or is almost
straight: the elbow (forearm_link) never sags more than TOL below the straight line shoulder (upperarm_link) ->
wrist (wrist1_link); checked during every move. move_to (OMPL) must keep ONE arm configuration
(shoulder front -165 <= j2 <= 10, elbow up j3 > 0, wrist down j5 < 0, |j1| <= 175 deg) and j1 must not swing round the back.
Part 1 replays the lab sequence of 2026-10-27 that flipped the elbow (x3); part 2 is a grid over the table
(z 0.05, sequential snake order + long jumps); part 3 = left <-> right rows, 4 round trips each, incl. pickup height
z -0.055. Unreachable grid points (-27) are listed, not failures."""
import math, time, rclpy
import tf2_ros
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from hk_fr5_interfaces.action import MoveTo
from scene_lib import launch, stop

ok = 0
def check(name, cond, extra=""):
    global ok
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True); ok += bool(cond)
    if not cond: raise SystemExit(1)

rclpy.init(); n = rclpy.create_node("joint_config_test")
J = {}
n.create_subscription(JointState, "/joint_states", lambda m: J.update(zip(m.name, m.position)), 10)
mt = ActionClient(n, MoveTo, "/motion/move_to")
tfb = tf2_ros.Buffer(); tf2_ros.TransformListener(tfb, n)
SHOULDER, ELBOW, WRIST = "upperarm_link", "forearm_link", "wrist1_link"   # frames at the j2, j3, j4 axes
TOL = 0.03   # m: 'almost straight' near the reach limit is allowed
DETOUR = 30  # deg extra j1 travel allowed (OMPL detours around obstacles, e.g. camera_post: up to 24 seen);
             # a flip / swing round the back is 100+ deg
def shape():
    """(elbow height above the shoulder-wrist line, elbow - wrist height) in m. Rule (user): base -> tcp rises then dips,
    or at most is almost straight = the elbow never sags below the straight shoulder-wrist line (a "V").
    A nearly straight rising arm at the reach limit is fine. None if no TF yet."""
    try:
        P = {}
        for f in (SHOULDER, ELBOW, WRIST):
            t = tfb.lookup_transform("base_link", f, rclpy.time.Time()).transform.translation
            P[f] = (t.x, t.y, t.z)
    except Exception:
        return None
    sh, el, wr = P[SHOULDER], P[ELBOW], P[WRIST]
    sw = [wr[i] - sh[i] for i in range(3)]; se = [el[i] - sh[i] for i in range(3)]
    k = max(0.0, min(1.0, sum(a * b for a, b in zip(se, sw)) / max(sum(a * a for a in sw), 1e-9)))
    return el[2] - (sh[2] + k * sw[2]), el[2] - wr[2]
d = math.degrees
def until(f, t=120):
    end = time.time() + t
    while not f.done() and time.time() < end: rclpy.spin_once(n, timeout_sec=0.005)
    return f.result()
def move(goal):
    """Execute; return result, joint min / max during the move (deg)."""
    lo, hi = {}, {}
    r = until(mt.send_goal_async(goal)).get_result_async(); t_shape = 0.0
    while not r.done():
        rclpy.spin_once(n, timeout_sec=0.005)
        for k, v in J.items(): lo[k] = min(lo.get(k, d(v)), d(v)); hi[k] = max(hi.get(k, d(v)), d(v))
        if time.time() - t_shape > 0.02:   # rise-then-dip shape, sampled during the move
            t_shape = time.time(); sh = shape()
            if sh: lo["rise"] = min(lo.get("rise", sh[0]), sh[0]); lo["dip"] = min(lo.get("dip", sh[1]), sh[1])
    for _ in range(20): rclpy.spin_once(n, timeout_sec=0.01)
    sh = shape()
    if sh: lo["rise"] = min(lo.get("rise", sh[0]), sh[0]); lo["dip"] = min(lo.get("dip", sh[1]), sh[1])
    return r.result().result, lo, hi
def shape_ok(lo): return lo.get("rise", -1) >= -TOL   # "rise" = elbow above the shoulder-wrist line
def shape_txt(lo): return f"elbow above shoulder-wrist line min {lo.get('rise', float('nan')) * 1000:+.0f} mm (elbow-wrist min {lo.get('dip', float('nan')) * 1000:+.0f} mm)"
def pose(x, y, z): return MoveTo.Goal(x=x, y=y, z=z, yaw=math.pi / 2, velocity_scaling=1.0)
def good_config(): return (d(J["j3"]) > 0 and d(J["j5"]) < 0 and abs(d(J["j1"])) <= 175.5
                           and -165.5 <= d(J["j2"]) <= 10.5)   # shoulder front (back branch: j2 ~ -197)

L = launch("use_gripper:=false", log="joint_config_launch.log"); time.sleep(18)
try:
    assert mt.wait_for_server(timeout_sec=20)
    lab = [(-0.3789, -0.3209, 0.10), (-0.4209, -0.0158, 0.06), (-0.2522, 0.2967, 0.06), (-0.3790, 0.7330, 0.03)]
    for trial in range(3):
        r, _, _ = move(MoveTo.Goal(named_target="ready", velocity_scaling=1.0))
        check(f"replay {trial}: ready", r.success, r.message)
        for x, y, z in lab:
            j1_0 = d(J["j1"]); r, lo, hi = move(pose(x, y, z))
            travel, net = hi["j1"] - lo["j1"], abs(d(J["j1"]) - j1_0)
            check(f"replay {trial}: ({x:+.3f}, {y:+.3f}) good config, j1 travel {travel:.0f} <= net {net:.0f} + {DETOUR}",
                  r.success and good_config() and travel <= net + DETOUR and hi["j3"] - lo["j3"] < 120 and shape_ok(lo),
                  f"{r.message}; {shape_txt(lo)}; j1 {d(J['j1']):+.0f} j3 {d(J['j3']):+.0f} j5 {d(J['j5']):+.0f}")
    xs, ys = [-0.85, -0.675, -0.5, -0.325, -0.15], [-0.9, -0.6, -0.3, 0.0, 0.3, 0.6, 0.9]
    pts = [(x, y) for i, x in enumerate(xs) for y in (ys if i % 2 == 0 else ys[::-1])]
    pts += [(-0.379, -0.321), (-0.379, 0.733), (-0.15, -0.9), (-0.15, 0.9)]   # long jumps across the table
    unreachable = []
    for x, y in pts:
        j1_0 = d(J["j1"]); r, lo, hi = move(pose(x, y, 0.05))
        if not r.success:
            unreachable.append((x, y, r.error_code.val)); continue
        travel, net = hi["j1"] - lo["j1"], abs(d(J["j1"]) - j1_0)
        check(f"grid ({x:+.3f}, {y:+.3f}): good config, no j1 detour, rise-then-dip", good_config() and travel <= net + DETOUR and shape_ok(lo),
              f"{shape_txt(lo)}; j1 {d(J['j1']):+.0f} (net {net:.0f}, travel {travel:.0f}) j3 {d(J['j3']):+.0f} j5 {d(J['j5']):+.0f}")
    print(f"INFO unreachable grid points ({len(unreachable)}): {unreachable}")
    # left <-> right across the table, large j1 motions, 4 round trips per row (OMPL is random)
    rows = [(-0.30, 0.55, 0.05), (-0.50, 0.55, 0.05), (-0.65, 0.45, 0.05),
            (-0.45, 0.60, -0.055), (-0.30, 0.40, -0.055)]   # x, |y|, z; -0.055 = tcp on a 6 mm board (pickup)
    for x, ay, z in rows:
        for rep in range(4):
            for y in (ay, -ay):
                j1_0 = d(J["j1"]); r, lo, hi = move(pose(x, y, z))
                travel, net = hi["j1"] - lo["j1"], abs(d(J["j1"]) - j1_0)
                check(f"left<->right x {x:+.2f} z {z:+.2f} rep {rep}: to y {y:+.2f} good config, j1 direct, elbow steady",
                      r.success and good_config() and travel <= net + DETOUR and hi["j3"] - lo["j3"] < 120 and shape_ok(lo),
                      f"{r.message}; {shape_txt(lo)}; j1 {d(J['j1']):+.0f} (net {net:.0f}, travel {travel:.0f}) j3 range "
                      f"{lo['j3']:+.0f}..{hi['j3']:+.0f} j5 {d(J['j5']):+.0f}")
    check("grid: unreachable only outside the pick / place area (none with |y| <= 0.6 and -0.675 <= x <= -0.325)",
          not [p for p in unreachable if abs(p[1]) <= 0.6 and -0.675 <= p[0] <= -0.325], str(unreachable))
finally:
    stop(L)
    print(f"{ok} checks passed")
    n.destroy_node(); rclpy.shutdown()
