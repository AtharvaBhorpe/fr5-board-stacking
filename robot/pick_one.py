"""Supervised single pick from /vision/boards. Run from the ROBOT workspace (needs hk_fr5_interfaces):

    cd fr5-board-stacking && pixi run python3 ../vision1-overhead/robot/pick_one.py --hover-only   # 1st: check position
    cd fr5-board-stacking && pixi run python3 ../vision1-overhead/robot/pick_one.py                # hover, Enter, pick

Sequence: hover 10 cm above the board centre -> ENTER -> straight down to the board top -> grip -> attach -> lift
10 cm -> ENTER -> back down -> release -> detach -> lift. The board ends where it started. Ctrl+C any time = stop.
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

ap = argparse.ArgumentParser()
ap.add_argument("--variant", help="VARa / VARb / VARc (default: first board)")
ap.add_argument("--hover-only", action="store_true", help="only move above the board, no contact")
ap.add_argument("--speed", type=float, default=0.1, help="velocity_scaling, first runs <= 0.2")
args = ap.parse_args()

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

# 1. One fresh vision message with a calibrated board
msgs = []
node.create_subscription(String, "/vision/boards", lambda m: msgs.append(json.loads(m.data)), 1)
print("waiting for /vision/boards ...")
while not msgs:
    rclpy.spin_once(node, timeout_sec=1.0)
msg = msgs[-1]
age = time.time() - msg["stamp"]
print(f"vision source {msg.get('source')!r}, message age {age:.2f} s")
if abs(age) > 2.0:
    raise SystemExit(f"vision message is {age:.1f} s old: is the vision node frozen, or are clocks different?")
if not msg.get("calibrated"):
    raise SystemExit("vision has no calibration: no metric poses, refusing to move")
boards = [b for b in msg["boards"] if args.variant in (None, b["variant"])]
if not boards:
    raise SystemExit(f"no pickable board (variant={args.variant}); rejected: {msg.get('rejected')}")
def tool_yaw(b):
    # Suction cup is round: board yaw or +pi, whichever is closest to home yaw +pi/2 -> least wrist rotation
    return min((b["yaw"], b["yaw"] + math.pi), key=lambda a: abs(math.remainder(a - math.pi / 2, 2 * math.pi)))


def reachable(b):
    """Plan only (no motion): hover pose and contact pose must both have a plan. Far corner of the pick table
    (> ~0.9 m from the base) is out of the FR5's reach."""
    for z in (b["z"] + HOVER, b["z"]):
        r = act(move_to, MoveTo.Goal(x=b["x"], y=b["y"], z=z, yaw=tool_yaw(b), plan_only=True))
        if not r.success:
            print(f"  skip {b['variant']} at ({b['x']:.3f}, {b['y']:.3f}), {math.hypot(b['x'], b['y']):.2f} m "
                  f"from base: {r.message}")
            return False
    return True


b = next((b for b in boards if reachable(b)), None)
if b is None:
    raise SystemExit("no reachable board: move one closer to the robot")
yaw = tool_yaw(b)
print(f"board {b['variant']} at x={b['x']:.4f} y={b['y']:.4f} top z={b['z']:.3f} yaw={math.degrees(b['yaw']):.1f} deg, "
      f"size {b['size_mm']} mm, inside_calibration={b['inside_calibration']}")

# 2. Hover above it (OMPL, avoids the static workcell)
check("hover", act(move_to, MoveTo.Goal(x=b["x"], y=b["y"], z=b["z"] + HOVER, yaw=yaw, velocity_scaling=args.speed)))
if args.hover_only:
    print("hover-only: check the cup is above the board centre (measure the offset in mm if not).")
    raise SystemExit(0)
input(">>> cup centred above the board? ENTER = go down and grip, Ctrl+C = stop ")

# 3. Down (LIN), grip, attach, lift (LIN, rule 3)
check("down", lin(-HOVER))
g = act(grip, Grip.Goal())
print(f"  grip: success={g.success} {g.message} time_to_vacuum={g.time_to_vacuum:.2f}s")
if not g.success:
    lin(HOVER)
    raise SystemExit("NO_VACUUM: lifted without board")
lx, ly = SIZES[b["variant"]]
call(attach, AttachBoard.Request(length=lx, width=ly, height=BOARD_H, yaw=0.0))  # tool yaw = board yaw (mod pi)
check("lift", lin(HOVER))
input(">>> board lifted. ENTER = put it back down and release ")

# 4. Put it back where it was
check("down again", lin(-HOVER))
r = act(release, Release.Goal())
print(f"  release: success={r.success} {r.message}")
call(detach, DetachBoard.Request(keep_as_obstacle=False))
check("lift empty", lin(HOVER))
print("done")
rclpy.shutdown()
