"""Supervised single pick from /vision/boards. Run from the ROBOT workspace (needs hk_fr5_interfaces):

    cd fr5-board-stacking && pixi run python3 ../vision1-overhead/robot/pick_one.py --hover-only   # 1st: check position
    cd fr5-board-stacking && pixi run python3 ../vision1-overhead/robot/pick_one.py                # hover, Enter, pick

    cd fr5-board-stacking && pixi run python3 ../vision1-overhead/robot/pick_one.py --place        # pick -> template

Sequence: hover 10 cm above the board centre -> ENTER -> straight down to the board top -> grip -> lift 10 cm ->
attach -> ENTER -> detach -> back down -> release -> lift. The board ends where it started. Ctrl+C any time = stop.
--place: after the lift, carry it to the destination template of its variant (vision `destinations`: tag 1 / 2 / 16
+ team offsets), ENTER, down onto the template (+ --stack boards already there), release, lift.
--variant X covered by other boards (learned model only: vision `occluded`): the covering top board goes to the
--temp spot first (stacked there), vision looks again, repeat (max MAX_CLEAR), then X is picked.
"""
import argparse
import json
import math
import time

import rclpy
from rclpy.action import ActionClient
from std_msgs.msg import String
from hk_fr5_interfaces.action import Grip, MoveDelta, MoveTo, Release
from hk_fr5_interfaces.srv import AttachBoard, DetachBoard

SIZES = {"VARa": (0.295, 0.210), "VARb": (0.150, 0.210), "VARc": (0.135, 0.210)}  # board X, Y (m)
BOARD_H = 0.005
HOVER = 0.10  # m above the board top
TEMP_TOP_Z = -0.055   # top of a board lying on the table (temp spot)
TEMP_CLEARANCE = 0.30  # m from any destination template centre (biggest board half-diagonal + template)
MAX_CLEAR = 4          # boards moved to the temp spot before giving up

ap = argparse.ArgumentParser()
ap.add_argument("--variant", help="VARa / VARb / VARc (default: first board)")
ap.add_argument("--hover-only", action="store_true", help="only move above the board, no contact")
ap.add_argument("--press", type=float, default=4.0,
                help="mm to go down beyond the computed board top. Real robot 2026-10-27: cup stopped ~10 mm above "
                     "the board, but MoveIt's table (top -0.060, cup collision 5 mm short of tcp) refuses > ~4 mm. "
                     "Real fix: measure where the cup touches the table and correct table_top_z / tcp_z")
ap.add_argument("--place", action="store_true", help="place on the destination template instead of putting back")
ap.add_argument("--stack", type=int, default=0, help="--place: boards already lying on that template")
ap.add_argument("--temp", type=float, nargs=2, default=(-0.25, 0.10), metavar=("X", "Y"),
                help="base_link x y (m) of the free spot for boards that cover the requested one (left table)")
ap.add_argument("--speed", type=float, default=0.1, help="velocity_scaling, first runs <= 0.2")
args = ap.parse_args()
PRESS = args.press / 1000.0

rclpy.init()
node = rclpy.create_node("vision1_pick_one")
def wait(f): rclpy.spin_until_future_complete(node, f); return f.result()
def act(c, goal): c.wait_for_server(); return wait(wait(c.send_goal_async(goal)).get_result_async()).result
def call(c, req): c.wait_for_service(); return wait(c.call_async(req))
def check(step, r):
    print(f"  {step}: success={r.success} {r.message}")
    if not r.success:
        raise SystemExit(f"STOP at {step}: {r.message}")

move_to = ActionClient(node, MoveTo, "/motion/move_to")
move_delta = ActionClient(node, MoveDelta, "/motion/move_delta")
grip = ActionClient(node, Grip, "/gripper/grip")
release = ActionClient(node, Release, "/gripper/release")
attach = node.create_client(AttachBoard, "/scene/attach_board")
detach = node.create_client(DetachBoard, "/scene/detach_board")


def lin(d):
    """Straight vertical move (base frame, d > 0 = up). Near the base a fast LIN can exceed joint acceleration limits
    (Pilz PLANNING_FAILED): retry once at a third of the speed."""
    r = act(move_delta, MoveDelta.Goal(dz=d, frame=MoveDelta.Goal.FRAME_BASE, velocity_scaling=args.speed))
    if not r.success and r.error_code.val == -1:
        print(f"  LIN failed at speed {args.speed} ({r.message}), retrying slower")
        r = act(move_delta, MoveDelta.Goal(dz=d, frame=MoveDelta.Goal.FRAME_BASE, velocity_scaling=args.speed / 3))
    return r

# 0. Which robot is this? A leftover sim process once made the real robot load mock hardware (SUCCESS, no motion).
from rclpy.qos import DurabilityPolicy, QoSProfile
desc = []
node.create_subscription(String, "/robot_description", lambda m: desc.append(m.data),
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
t0 = time.time()
while not desc and time.time() - t0 < 5.0:
    rclpy.spin_once(node, timeout_sec=0.5)
n_desc = node.count_publishers("/robot_description")
if n_desc > 1:
    raise SystemExit(f"{n_desc} robot stacks publish /robot_description (sim + robot?): stop all but one")
hw = "none (robot not started?)" if not desc else ("MOCK (simulation, arm will NOT move)"
                                                    if "mock_components" in desc[-1] else "REAL FR5")
print(f"robot hardware: {hw}")

# A run that stopped half-way leaves the board model attached: every plan would then fail. Clear it.
call(detach, DetachBoard.Request(keep_as_obstacle=False))

# 1. Vision
msgs = []
node.create_subscription(String, "/vision/boards", lambda m: msgs.append(json.loads(m.data)), 1)


def fresh_vision(after=0.0):
    """Newest /vision/boards message taken after time `after` (the scene changed: wait for a new frame)."""
    print("waiting for /vision/boards ...")
    msgs.clear()
    while not msgs or msgs[-1]["stamp"] < after:
        rclpy.spin_once(node, timeout_sec=1.0)
    msg = msgs[-1]
    age = time.time() - msg["stamp"]
    print(f"vision source {msg.get('source')!r}, message age {age:.2f} s")
    if abs(age) > 2.0:
        raise SystemExit(f"vision message is {age:.1f} s old: is the vision node frozen, or are clocks different?")
    if not msg.get("calibrated"):
        raise SystemExit("vision has no calibration: no metric poses, refusing to move")
    return msg


def tool_yaw(b):
    # Suction cup is round: board yaw or +pi, whichever is closest to home yaw +pi/2 -> least wrist rotation
    return min((b["yaw"], b["yaw"] + math.pi), key=lambda a: abs(math.remainder(a - math.pi / 2, 2 * math.pi)))


def reachable(b, z_top):
    """Plan only (no motion): hover pose and contact pose must both have a plan. Far corner of the pick table
    (> ~0.9 m from the base) is out of the FR5's reach."""
    for z in (z_top + HOVER, z_top - PRESS):
        r = act(move_to, MoveTo.Goal(x=b["x"], y=b["y"], z=z, yaw=tool_yaw(b), plan_only=True))
        if not r.success:
            print(f"  skip {b['variant']} at ({b['x']:.3f}, {b['y']:.3f}), {math.hypot(b['x'], b['y']):.2f} m "
                  f"from base: {r.message}")
            return False
    return True


def transfer(b, to=None, what="put it back down"):
    """Pick board b; put it at to = (x, y, top z, tool yaw) or back where it was. ENTER before every descent."""
    print(f"board {b['variant']} at x={b['x']:.4f} y={b['y']:.4f} top z={b['z']:.3f} "
          f"yaw={math.degrees(b['yaw']):.1f} deg, size {b['size_mm']} mm, inside_calibration={b['inside_calibration']}")
    yaw = tool_yaw(b)
    # Hover above it (OMPL, avoids the static workcell)
    check("hover", act(move_to, MoveTo.Goal(x=b["x"], y=b["y"], z=b["z"] + HOVER, yaw=yaw,
                                            velocity_scaling=args.speed)))
    if args.hover_only:
        print("hover-only: check the cup is above the board centre (measure the offset in mm if not).")
        raise SystemExit(0)
    input(">>> cup centred above the board? ENTER = go down and grip, Ctrl+C = stop ")
    # Down (LIN), grip, lift (LIN, rule 3), attach
    print(f"  going down {(HOVER + PRESS) * 1000:.0f} mm")
    check("down", lin(-(HOVER + PRESS)))
    g = act(grip, Grip.Goal())
    print(f"  grip: success={g.success} {g.message} time_to_vacuum={g.time_to_vacuum:.2f}s")
    if not g.success:
        lin(HOVER + PRESS)
        raise SystemExit("NO_VACUUM: lifted without board")
    # Lift BEFORE attaching: with --press the attached board model would start inside the table and MoveIt
    # refuses every plan. The lift is straight up, nothing to avoid.
    check("lift", lin(HOVER + PRESS))
    lx, ly = SIZES.get(b["variant"]) or SIZES[b.get("model_class", "VARa")]  # UNKWN: model's guess, collision only
    call(attach, AttachBoard.Request(length=lx, width=ly, height=BOARD_H, yaw=0.0))  # tool yaw = board yaw (mod pi)
    if to:
        x, y, z_top, tyaw = to
        # Carry (OMPL avoids the workcell, held board attached)
        check("carry", act(move_to, MoveTo.Goal(x=x, y=y, z=z_top + HOVER, yaw=tyaw, velocity_scaling=args.speed)))
    input(f">>> ENTER = {what} and release ")
    # Detach the board MODEL first (the vacuum still holds it): with --press it would end inside the table and
    # MoveIt refuses the descent. Straight down, only the carry above needed it.
    call(detach, DetachBoard.Request(keep_as_obstacle=False))
    check("down again", lin(-(HOVER + PRESS)))
    r = act(release, Release.Goal())
    print(f"  release: success={r.success} {r.message}")
    check("lift empty", lin(HOVER + PRESS))
    return time.time()


def blocker(msg, target):
    """Top board to move away so `target` (occluded) gets free: breadth-first over covered_by, nearest first."""
    top = {b["id"]: b for b in msg["boards"]}
    occ = {o["id"]: o for o in msg.get("occluded", [])}
    seen, queue = {target["id"]}, list(target.get("covered_by", []))
    while queue:
        i = queue.pop(0)
        if i in seen:
            continue
        seen.add(i)
        if i in top:
            return top[i]
        queue += occ.get(i, {}).get("covered_by", [])
    return None


place_z = lambda d: d["support_z"] + (args.stack + 1) * BOARD_H  # cup tip = top of the placed board
msg = fresh_vision()
moved = 0  # boards put on the temp spot in this run (stacked there)
for _ in range(MAX_CLEAR + 1):
    boards = [b for b in msg["boards"] if args.variant in (None, b["variant"])]
    dests = {d["variant"]: d for d in msg.get("destinations", [])}
    if args.place:
        print(f"destinations seen: {sorted(dests) or 'none'}")
        if args.variant and args.variant not in dests:
            raise SystemExit("--place: no board whose destination template (tag) is visible")
        boards = [b for b in boards if b["variant"] in dests]
        if not boards and not args.variant:
            raise SystemExit("--place: no board whose destination template (tag) is visible")
    b = next((b for b in boards if reachable(b, b["z"]) and
              (not args.place or reachable(dests[b["variant"]], place_z(dests[b["variant"]])))), None)
    if b is not None:
        break
    hidden = [o for o in msg.get("occluded", []) if args.variant and o["variant"] == args.variant]
    if not hidden:
        if boards:
            raise SystemExit("no reachable board (and destination): move one closer to the robot")
        raise SystemExit(f"no pickable board (variant={args.variant}); occluded: "
                         f"{[(o['id'], o['variant']) for o in msg.get('occluded', [])]}; rejected: {msg.get('rejected')}")
    # 2. Requested board is covered: move the board on top of it to the temp spot, look again
    t = hidden[0]
    top = blocker(msg, t)
    if top is None:
        raise SystemExit(f"{t['variant']} (id {t['id']}) is covered, but not by a pickable board "
                         f"(hand / arm / unknown blob?): covered_by {t.get('covered_by')}")
    if moved >= MAX_CLEAR:
        raise SystemExit(f"moved {moved} boards to the temp spot, {args.variant} still covered: stop")
    tz = TEMP_TOP_Z + moved * BOARD_H
    spot = {"variant": "temp", "x": args.temp[0], "y": args.temp[1], "yaw": top["yaw"]}
    near = [d["variant"] for d in dests.values() if math.hypot(d["x"] - spot["x"], d["y"] - spot["y"]) < TEMP_CLEARANCE]
    if near:
        raise SystemExit(f"temp spot {args.temp} is within {TEMP_CLEARANCE} m of template(s) {near}: pass --temp X Y")
    if not reachable(top, top["z"]) or not reachable(spot, tz):
        raise SystemExit(f"cannot clear: covering board {top['variant']} (id {top['id']}) or temp spot unreachable")
    print(f"--- {t['variant']} (id {t['id']}) is under {top['variant']} (id {top['id']}): moving it to the temp spot "
          f"{args.temp} (board {moved + 1} there)")
    done_at = transfer(top, (spot["x"], spot["y"], tz, tool_yaw(top)), "put it on the temp spot")
    moved += 1
    msg = fresh_vision(after=done_at + 1.0)

if args.place:
    d = dests[b["variant"]]
    print(f"  destination {d['variant']} (tag {d['tag_id']}) at x={d['x']:.4f} y={d['y']:.4f} "
          f"yaw={math.degrees(d['yaw']):.1f} deg, place top z={place_z(d):.3f}, median of {d.get('frames', 1)} "
          f"frames, seen {d['age']} s ago")
    transfer(b, (d["x"], d["y"], place_z(d), tool_yaw(d)), "put it down on its template")
else:
    transfer(b)
print("done")
rclpy.shutdown()
