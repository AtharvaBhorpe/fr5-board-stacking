"""Offline checks of the task_main_controller node's pure parts (T5c-1, CL-37): World.msg -> core dict (the core decides
the same on the message as on the world snapshot), Job.msg <-> core job, SkillCode names, skill goals, config. No ROS
graph (messages only), ~2 s.

    pixi run python3 tests/main_node_check.py
"""
import math
import os
import sys
import tempfile

from builtin_interfaces.msg import Time

from action_msgs.msg import GoalStatus

from hk_fr5_task.main_controller import (CODE_NAMES, code_name, dict_to_job, goal_for, job_to_dict, load_config,
                                         result_code, world_to_dict)
from hk_fr5_task.main_core import LOOKUP_CODES, MOTION_CODES, MainCore
from hk_fr5_task_interfaces.msg import Board, Job, Obstacle, SkillCode, World as WorldMsg, Zone

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from main_sim import CD, MCFG, Sim                                   # noqa: E402

ok = bad = 0
def check(name, cond, extra=""):
    global ok, bad
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True)
    ok += bool(cond); bad += not cond

def to_msg(snap, t):
    """Exactly like world_model._publish."""
    m = WorldMsg(vision_ok=snap["vision_ok"], vision_age=snap["vision_age"], held_board_id=snap["held_board_id"],
                 gripper_state="IDLE")
    m.header.stamp = Time(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))
    m.boards = [Board(**b) for b in snap["boards"]]
    m.zones = [Zone(**z) for z in snap["zones"]]
    m.obstacles = [Obstacle(**o) for o in snap["obstacles"]]
    return m

def same(a, b):
    if isinstance(a, float) and isinstance(b, float):
        return (math.isnan(a) and math.isnan(b)) or abs(a - b) < 1e-9
    return a == b

# 1 SkillCode names
consts = {n: getattr(SkillCode, n) for n in dir(type(SkillCode)) if n.isupper() and not n.startswith("_")
          and isinstance(getattr(SkillCode, n), int)}
check("SkillCode: every constant has its name (value -> name), 23 codes", len(CODE_NAMES) == len(consts) == 23
      and all(CODE_NAMES[v] == n for n, v in consts.items()), str(len(CODE_NAMES)))
check("code_name: 0 -> SUCCESS, 30 -> GRIP_FAILED, unknown value -> INTERNAL_ERROR",
      code_name(0) == "SUCCESS" and code_name(30) == "GRIP_FAILED" and code_name(999) == "INTERNAL_ERROR")
used = set(LOOKUP_CODES) | set(MOTION_CODES) | {"SUCCESS", "OBSTACLE", "TIMEOUT", "RELEASE_FAILED", "ALREADY_HOLDING",
       "CANCELLED", "GRIP_FAILED", "LOST_ON_LIFT", "LOST_IN_TRANSIT", "NOT_HOLDING", "TARGET_OCCUPIED", "TARGET_UNKNOWN",
       "STACK_FULL", "NOT_VERIFIED"}
check("every code name the core reacts to exists in SkillCode (no typo)", used <= set(CODE_NAMES.values()),
      str(used - set(CODE_NAMES.values())))

S_OK, S_CAN, S_AB = GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED
check("result_code (N2): SUCCEEDED + success + SUCCESS -> SUCCESS; a default-filled result (code 0, success false) is "
      "never SUCCESS", result_code(S_OK, True, 0) == "SUCCESS" and result_code(S_OK, False, 0) == "INTERNAL_ERROR"
      and result_code(S_AB, False, 0) == "INTERNAL_ERROR" and result_code(GoalStatus.STATUS_UNKNOWN, True, 0) == "INTERNAL_ERROR")
check("result_code: aborted / cancelled keep the skill's code; cancelled without a code -> CANCELLED; success claimed "
      "with a failure code -> INTERNAL_ERROR", result_code(S_AB, False, 30) == "GRIP_FAILED"
      and result_code(S_CAN, False, 1) == "CANCELLED" and result_code(S_CAN, False, 0) == "CANCELLED"
      and result_code(S_OK, True, 30) == "INTERNAL_ERROR")

# 2 World.msg -> core dict
sim = Sim()
sim.frames(1.0)
snap = sim.world.snapshot(sim.t)
w = world_to_dict(to_msg(snap, sim.t))
check("world_to_dict: stamp = header stamp (s), flags + held board copied", abs(w["stamp"] - sim.t) < 1e-6
      and w["vision_ok"] == snap["vision_ok"] and w["held_board_id"] == snap["held_board_id"], f"{w['stamp']} vs {sim.t}")
check("world_to_dict: every board / zone / obstacle field equal to the world snapshot (incl. NaN top_z)",
      all(same(a[k], b[k]) for kind in ("boards", "zones", "obstacles") for a, b in zip(w[kind], snap[kind]) for k in a)
      and all(len(w[k]) == len(snap[k]) for k in ("boards", "zones", "obstacles"))
      and any(math.isnan(b["top_z"]) for b in w["boards"]))
c1, c2 = MainCore(MCFG), MainCore(MCFG)
job = {"type": "process", "select_variant": "red", "zone": "processing_1", "to": "drop_off_1", "count": 2, "priority": 10}
c1.add_job(dict(job), snap, sim.t); c2.add_job(dict(job), w, sim.t)
s2 = dict(snap, stamp=sim.t)
a1, a2 = c1.next_action(s2, sim.t, sim.tcp), c2.next_action(w, sim.t, sim.tcp)
check("the core decides the same on the converted message as on the snapshot", a1 == a2 and a1 is not None, f"{a1} / {a2}")

# 3 Job.msg <-> core job
req = Job(type="transfer", select_variant="white_small", to="drop_off_2", count=3, max_attempts=5, priority=7,
          source="dashboard", state="DONE", delivered=9)
d = job_to_dict(req)
check("job_to_dict: only the caller's fields (no state / counters taken from the request)",
      d == {"type": "transfer", "select_variant": "white_small", "select_board": "", "from_zone": "", "zone": "",
            "to": "drop_off_2", "count": 3, "max_attempts": 5, "priority": 7, "source": "dashboard"}, str(d))
core = MainCore(MCFG)
okj, msg, jid = core.add_job(d, snap, sim.t)
st = core.status(sim.t)["jobs"][0]
jm = dict_to_job(st)
check("dict_to_job: core status entry -> Job.msg (id, state, counters, message)", okj and jm.id == jid
      and jm.state == "QUEUED" and jm.count == 3 and jm.max_attempts == 5 and jm.priority == 7 and jm.delivered == 0
      and jm.source == "dashboard" and "transfer" not in jm.message, f"{jm.id} {jm.state} {jm.count}")

# 4 skill goals
g = {k: goal_for(a) for k, a in {"pick": {"skill": "pick", "board": "b3"}, "place": {"skill": "place", "target": "drop_off_1"},
     "release_safe": {"skill": "release_safe", "target": "fault"}, "home": {"skill": "home"},
     "wait_clear": {"skill": "wait_clear"}}.items()}
check("goal_for: pick board id (skill decides contact search), place / release_safe target, default speeds",
      g["pick"][1].board_id == "b3" and not g["pick"][1].contact_search and g["pick"][1].velocity_scaling == 0.0
      and g["place"][1].target == "drop_off_1" and g["release_safe"][1].target == "fault"
      and g["wait_clear"][1].timeout == 0.0 and all(g[k][0] == k for k in g))
try:
    goal_for({"skill": "dance"}); check("goal_for rejects an unknown skill", False)
except ValueError as e:
    check("goal_for rejects an unknown skill", "dance" in str(e))

# 5 config
cfg = load_config(os.path.join(CD, "main.yaml"), os.path.join(CD, "world.yaml"))
check("load_config: main.yaml + variants from world.yaml, skill limits for every skill",
      set(cfg["variants"]) == {"VARa", "VARb", "VARc"}
      and set(cfg["skill_timeout"]) == {"pick", "place", "home", "wait_clear", "release_safe"}
      and cfg["world_timeout"] > 0 and cfg["tcp_period"] > 0)
bad_cfg = os.path.join(tempfile.mkdtemp(), "main_bad.yaml")
with open(os.path.join(CD, "main.yaml")) as f, open(bad_cfg, "w") as g2:
    g2.write("\n".join(l for l in f.read().split("\n") if not l.startswith("tick:")))
try:
    load_config(bad_cfg, os.path.join(CD, "world.yaml")); check("load_config: a missing key is reported", False)
except KeyError as e:
    check("load_config: a missing key is reported with the file and key", "tick" in str(e), str(e))
os.remove(bad_cfg)

print(f"{ok} passed, {bad} failed")
sys.exit(1 if bad else 0)
