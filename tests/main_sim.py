"""Shared simulation for the main-controller offline checks (main_core_check, main_core_regress_check).

The real world_core + fake_core on a SIMULATED clock with nominal skill durations (pick 6 s, place 8 s, ...).
Skills behave like skill_server: before moving they check the cup and the world (ALREADY_HOLDING, BOARD_NOT_FOUND,
NOT_HOLDING, TARGET_UNKNOWN, TARGET_OCCUPIED / STACK_FULL); release_safe: NOT_HOLDING if the world holds a board
but the cup is empty (CL-32), nothing held at all -> nothing to do (SUCCESS).
Realism: the robot mask (CL-31) is built like the world-model node (band base -> tcp + the held board's circle) and the
camera does not see boards under the real arm (a narrower band, `occlude`); the camera sees the held board at the tcp;
the tcp follows the skills (at the board / zone during a skill, home after place / release_safe, at hover after a failed
pick). Layout rule CL-34: no scenario board / zone under the arm mask at home (ValueError). Options: world-event lag, hidden zones, blind camera, a timeline of scheduled changes, a physical cup (`cup`)
that can disagree with the world, take_off_cup() / operator_clear(zone) (operator). Failures are injected per skill
(`inject`, or `chaos(action)` for fuzz runs); 'CODE+held' = the pick fails after the board is on the cup (A2);
'CODE+done' = the skill did its work but returns CODE (e.g. NOT_VERIFIED: placed, not confirmed).
Checker after every step: invariants (pre_violations / state_violations), loop detector (failures) and progress
detector (repeated successes, no change for many actions).

MAIN_CORE=<path to a main_core.py> runs the checks against another core version (e.g. an old one).
"""
import copy
import importlib.util
import os

import yaml

from hk_fr5_task import mask as M
from hk_fr5_task import vision1
from hk_fr5_task.fake_core import FakeWorld
from hk_fr5_task.skill_math import zone_accepts
from hk_fr5_task.world_core import World, rect_corners

if os.environ.get("MAIN_CORE"):
    _spec = importlib.util.spec_from_file_location("main_core_under_test", os.environ["MAIN_CORE"])
    _mod = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_mod)
    MainCore = _mod.MainCore
else:
    from hk_fr5_task.main_core import MainCore

HERE = os.path.dirname(os.path.abspath(__file__))
CD = os.path.join(HERE, "..", "src", "hk_fr5_task", "config")
WCFG = yaml.safe_load(open(os.path.join(CD, "world.yaml")))
WCFG.update(yaml.safe_load(open(os.path.join(HERE, "test_variants.yaml"))))   # placeholder test variants (V1, TM-20)
MCFG = yaml.safe_load(open(os.path.join(CD, "main.yaml")))
MCFG["variants"] = list(WCFG["variants"])
SCFG = yaml.safe_load(open(os.path.join(CD, "skills.yaml")))
DUR = {"pick": 6.0, "place": 8.0, "home": 3.0, "wait_clear": 2.0, "release_safe": 10.0}
VISION_PERIOD = 0.2              # s: one Vision 1 frame per sim step (~5 Hz like the real Vision 1, user)
REFUSE = 0.2                     # s: a skill refusing before it moves
PRECHECK = ("BOARD_NOT_FOUND", "BOARD_MOVED", "VISION_STALE", "ALREADY_HOLDING", "TARGET_UNKNOWN", "TARGET_OCCUPIED",
            "STACK_FULL")      # injected: skill_server raises these before it moves (no tcp change). An injected
#                                NOT_HOLDING = silent drop during the carry (D7); real NOT_HOLDING refusals come from _refuse
FINAL = ("DONE", "CANCELLED")
ZONES = [{"name": "processing_1", "kind": "processing", "x": -0.50, "y": 0.45, "yaw": 0.0, "length": 0.20, "width": 0.16},
         {"name": "processing_2", "kind": "processing", "x": -0.50, "y": -0.45, "yaw": 0.0, "length": 0.20, "width": 0.16},
         {"name": "drop_off_1", "kind": "drop_off", "x": -0.15, "y": 0.55, "yaw": 0.0, "length": 0.24, "width": 0.20},
         {"name": "drop_off_2", "kind": "drop_off", "x": -0.15, "y": -0.55, "yaw": 0.0, "length": 0.24, "width": 0.20}]
for _z in ZONES:
    _z["top_z"] = WCFG["table_z"]  # logic tests: no sheet (V2 / V4 sheet heights: world_core_check / skill_math_check)
# the fault zone is never a Vision 1 zone: world.yaml fixed_zones (CL-15 / CL-39)
BOARDS = [{"variant": "red", "x": -0.45, "y": 0.22, "yaw": 0.2}, {"variant": "red", "x": -0.62, "y": 0.20, "yaw": 0.0},
          {"variant": "white_small", "x": -0.30, "y": -0.30, "yaw": -0.1, "count": 3}]   # clear of the park-pose arm band (TM-25)
OPERATOR = {"take_after": 1.5, "return_after": 3.0}
JOB_STATES = ("QUEUED", "RUNNING", "WAITING", "PAUSED", "DONE", "CANCELLED")      # Job.msg
FINAL_ITEMS = ("delivered", "fault", "lost", "skipped")
SAME_FAIL_MAX = 5              # loop detector: identical failing actions in a row
SAME_OK_MAX = 2                # progress detector: identical successful actions in a row
NO_PROGRESS = 12               # ... actions in a row without any change of jobs / boards / zones
ARM_R, OCCLUDE_R, MASK_EXTRA = 0.12, 0.05, 0.03     # mask band (node default), real arm (hides less), held margin
FAIL_WINDOW = (60.0, 20)       # ... and at most 20 failures in any 60 s of simulated time
ALL = []                       # every Sim created (the invariant summary at the end of a test file)


def can_take(w, name, fault):
    """Independent CL-28 rule: can zone `name` take a board now (processing: seen + EMPTY, drop-off: seen + not FULL,
    fault zone always)."""
    if name == fault:
        return True
    z = next((z for z in w["zones"] if z["name"] == name), None)
    if z is None or (not z["fixed"] and z["age"] > MCFG["fresh"]):
        return False
    return z["state"] == "EMPTY" if z["kind"] == "processing" else z["state"] != "FULL"


def pick_target(core, a):
    j = core.job(a.get("job", ""))
    if j is None:
        return None
    st = a.get("stage")
    return {"to_zone": j["zone"], "to_out": j["to"], "to_target": j["to"], "to_fault": core.c["fault_zone"]}.get(st)


def pre_violations(core, a, w):
    """Rules for an action the core just chose, against the world it decided on."""
    out = []
    fault = core.c["fault_zone"]
    if a["skill"] == "pick" and hasattr(core, "_target_ok"):          # CL-28 (cores from step 2 on)
        tg = pick_target(core, a)
        if tg and not can_take(w, tg, fault):
            out.append(f"pick {a.get('board')} for {tg}, which cannot take a board now")
    if a["skill"] == "place" and hasattr(core, "_target_ok") and not can_take(w, a.get("target"), fault):
        out.append(f"place into {a.get('target')}, which cannot take a board now")
    if core.halted:
        out.append(f"{a['skill']} chosen while HALTED")
    if not w["vision_ok"]:
        out.append(f"{a['skill']} chosen while vision is stale")
    if a["skill"] == "pick" and w["held_board_id"]:
        out.append(f"pick {a.get('board')} while the world holds {w['held_board_id']}")
    if a["skill"] == "place":
        z = next((z for z in w["zones"] if z["name"] == a.get("target")), None)
        if z is not None and z["kind"] == "processing" and z["state"] != "EMPTY":
            out.append(f"place into {z['name']} while it is {z['state']}")
    return out


def state_violations(core):
    """Rules for the core's bookkeeping (after every result)."""
    out, owner = [], {}
    for j in core.jobs:
        if j["state"] not in JOB_STATES:
            out.append(f"{j['id']}: unknown state {j['state']}")
        if j["type"] == "home":
            continue
        items = j["_items"]
        for b, st in items.items():
            if st not in FINAL_ITEMS:
                if b in owner:
                    out.append(f"{b} in flight for {owner[b]} and {j['id']}")
                owner[b] = j["id"]
        final = sum(1 for st in items.values() if st in FINAL_ITEMS)
        counted = sum(j.get(k, 0) for k in ("delivered", "to_fault", "lost", "skipped"))
        if "attempts" in j and (j["attempts"] != len(items) or counted != final):
            out.append(f"{j['id']}: counters attempts {j['attempts']} / items {len(items)}, outcomes {counted} / final {final}")
        if j["count"] > 0 and j.get("delivered", 0) > j["count"]:           # 0 = all, -1 = continuous (TM-39)
            out.append(f"{j['id']}: delivered {j['delivered']} > count {j['count']}")
        if j.get("max_attempts") and j["attempts"] > j["max_attempts"]:
            out.append(f"{j['id']}: attempts {j['attempts']} > max_attempts {j['max_attempts']}")
        if j["state"] in FINAL and any(o == j["id"] for o in core.claims.values()):
            out.append(f"{j['id']} is {j['state']} but still claims boards")
        stale = [b for b, o in core.claims.items() if o == j["id"] and b not in items]
        if len(stale) > 1:
            out.append(f"{j['id']} claims boards it never picked: {stale}")
        if j["state"] == "DONE" and any(st not in FINAL_ITEMS for st in items.values()):
            out.append(f"{j['id']} is DONE with a board still in flight: {items}")
    return out


class Sim:
    """World + fake world + simulated skills on a simulated clock."""

    def __init__(self, boards=BOARDS, wcfg=None, mcfg=None, return_variant="same", lag=0.0, operator=None, mask=True):
        op = {**OPERATOR, **(operator or {}), "return_variant": return_variant}
        scn = {"zones": ZONES, "boards": copy.deepcopy(boards), "noise_xy": 0.0, "operator": op}
        home_arm = [M.capsule((0.0, 0.0), tuple(MCFG["home_xy"]), ARM_R)]          # CL-34 layout rule
        for bd in boards:
            v = WCFG["variants"][bd["variant"]]
            if M.hits(home_arm, rect_corners(bd["x"], bd["y"], bd.get("yaw", 0.0), v["length"], v["width"])):
                raise ValueError(f"layout: board at ({bd['x']}, {bd['y']}) lies under the arm at home (CL-34)")
        for z in ZONES:
            if M.hits(home_arm, rect_corners(z["x"], z["y"], z["yaw"], z["length"], z["width"])):
                raise ValueError(f"layout: zone {z['name']} lies under the arm at home (CL-34)")
        self.fake = FakeWorld(scn, WCFG["variants"])
        self.world = World(wcfg or WCFG)
        self.core = MainCore(mcfg or MCFG)
        self.t, self.tcp = 0.0, tuple(MCFG["home_xy"])
        self.inject, self.seq = [], []
        self.cup = ""                  # what is physically on the suction cup
        self.lag = lag                 # s: a skill's world events reach the world this long after the skill result
        self.later = []                # (t, event) delayed world events
        self.hide = set()              # zone names the camera does not see
        self.blind = False             # camera delivers nothing (-> vision_ok false)
        self.timeline = []             # (t, fn(sim)) scheduled changes
        self.starts = []               # (t, skill, vision_ok, world held) for every started action
        self.violations = []           # (t, text) broken invariants
        self.fail_times = []           # times of failed actions (loop detector)
        self._same = (None, 0)         # (last failing action, repeats)
        self._same_ok = (None, 0)      # (last successful action, repeats)
        self._sig = (None, 0)          # (progress signature, actions without a change)
        self.mask = mask               # CL-31 robot mask + occlusion + held board visible
        self.old_release_safe = False  # True: release_safe as before CL-32 (SUCCESS 'nothing held' with an empty cup)
        self.expect_bad = False        # True: this scenario shows a violation on purpose (left out of the summary)
        self.chaos = None              # fuzz: fn(action) -> failure code to inject, or None
        self.fault_drops = 0           # boards put into the fault zone (a void: the world forgets them, CL-39)
        self.skip_homes = 0            # TM-36: places that stayed over the zone (no home move)
        ALL.append(self)
        self.frames(0.5)

    # ---------- world ----------
    def frames(self, secs):
        end = self.t + secs
        while self.t < end - 1e-9:
            self.t = round(self.t + VISION_PERIOD, 6)
            for item in [x for x in self.timeline if x[0] <= self.t]:
                self.timeline.remove(item); item[1](self)
            for item in [x for x in self.later if x[0] <= self.t]:
                self.later.remove(item); self.world.event(self.t, item[1])
            frame = self.fake.frame(self.t, self.t)          # always: the operator keeps going
            if self.blind:
                continue
            frame["zones"] = [z for z in frame["zones"] if z["name"] not in self.hide]
            _, raw, problems = vision1.parse(frame)          # the real adapter path (V3 / V5)
            assert not problems, problems
            arm = [M.capsule((0.0, 0.0), self.tcp, OCCLUDE_R)]
            dets = []
            for d in raw:
                if self.mask and d["cls"].startswith("board/") and M.hits(arm, d["corners"]):
                    continue                                 # under the real arm: the camera cannot see it
                dets.append(d)
            if self.mask and self.cup and self.cup in self.world.boards:   # the overhead camera sees the held board
                v = WCFG["variants"].get(self.world.boards[self.cup].get("variant", ""), {"length": 0.1, "width": 0.07})
                dets.append({"cls": f"board/{self.world.boards[self.cup].get('variant', 'red')}", "id": "held",
                             "score": 0.9, "corners": rect_corners(self.tcp[0], self.tcp[1], 0.0, v["length"], v["width"])})
            self.world.update(self.t, dets, self._mask() if self.mask else None)

    def _mask(self):
        """Like world_model._mask (transitional, CL-31): band base -> tcp; the held board's circle while holding."""
        r = 0.0
        hb = self.world.boards.get(self.world.held) if self.world.held else None
        if hb is not None:
            v = WCFG["variants"].get(hb.get("variant", ""))
            l, w = (v["length"], v["width"]) if v else (hb.get("length", 0.0), hb.get("width", 0.0))
            r = (l * l + w * w) ** 0.5 / 2 + MASK_EXTRA
        return M.band(self.tcp, ARM_R, r)

    def operator_clear(self, zone):
        """The operator removes every board in a zone (physically) and resets it in the world."""
        z = next(z for z in self.fake.zones if z["name"] == zone)
        self.fake.boards = [b for b in self.fake.boards if self.fake.zone_at(b["x"], b["y"]) is not z]
        self.world.event(self.t, {"type": "reset_zone", "zone": zone})

    def take_off_cup(self):
        """The operator takes the board off the cup (no skill running): physically gone, the world is not told."""
        if self.cup:
            self.fake.event(self.t, {"type": "dropped"})
            self.cup = ""

    def wev(self, ev, delay=0.0):
        """A world event from a skill; with lag it reaches the world `delay` s later."""
        if delay > 0:
            self.later.append((self.t + delay, ev))
        else:
            self.world.event(self.t, ev)

    def wd(self):
        s = self.world.snapshot(self.t)
        s["stamp"] = self.t
        return s

    def zone(self, name):
        return next(z for z in self.wd()["zones"] if z["name"] == name)

    def board(self, bid):
        return next((b for b in self.wd()["boards"] if b["id"] == bid), None)

    # ---------- skills ----------
    def _refuse(self, a, w):
        """skill_server's checks before it moves: a SkillCode name, or ''."""
        skill = a["skill"]
        if skill == "pick":
            if self.cup or w["held_board_id"]:
                return "ALREADY_HOLDING"
            b = self.board(a["board"])
            if b is None or b["location"] == "held":           # TM-36: boards the world still holds count
                return "BOARD_NOT_FOUND"
        if skill == "release_safe" and w["held_board_id"] and not self.cup and not self.old_release_safe:
            return "NOT_HOLDING"                                 # CL-32: the world holds a board, the cup is empty
        if skill == "place" or (skill == "release_safe" and self.cup and w["held_board_id"]):
            if not self.cup or not w["held_board_id"]:
                return "NOT_HOLDING"
            target = a.get("target") or self.core.c["fault_zone"]
            z = next((z for z in w["zones"] if z["name"] == target), None)
            if z is None or (not z["fixed"] and z["age"] > SCFG["fresh"]):
                return "TARGET_UNKNOWN"
            return zone_accepts(SCFG, z)
        return ""

    def run(self, a):
        """Execute one action: (code, message). Effects in the middle of the skill duration."""
        skill, b = a["skill"], a.get("board", "")
        w = self.wd()
        why = self._refuse(a, w)
        if why:
            self.frames(REFUSE)
            return why, "refused (sim)"
        if self.chaos is not None:                           # fuzz: random failure for this action
            c = self.chaos(a)
            if c:
                self.inject.insert(0, (skill, None, c))
        for k, (s, bb, code) in enumerate(self.inject):
            if s == skill and bb in (None, b) and code in PRECHECK:      # the skill refuses before it moves
                self.inject.pop(k)
                self.frames(REFUSE)
                return code, "injected (refused before moving)"
        pre = DUR[skill] if self.lag else DUR[skill] / 2   # with lag: effects at the end, world events lag later
        target = a.get("target") or self.core.c["fault_zone"]
        if skill == "pick" and self.board(b):                # the arm goes there (hover / descend)
            self.tcp = (self.board(b)["x"], self.board(b)["y"])
        elif skill in ("place", "release_safe") and self.cup and w["held_board_id"]:
            self.tcp = (self.zone(target)["x"], self.zone(target)["y"])
        elif skill == "home":
            self.tcp = tuple(MCFG["home_xy"])
        ret = "SUCCESS"
        for k, (s, bb, code) in enumerate(self.inject):
            if s == skill and bb in (None, b) and code.endswith("+done"):  # the work is done, the code says otherwise
                self.inject.pop(k)
                ret = code[:-5]
                break
            if s == skill and bb in (None, b):
                self.inject.pop(k)
                self.frames(pre)
                if code.endswith("+held"):                           # A2: fails after the board is on the cup
                    code = code[:-5]
                    bd = self.board(b)
                    self.fake.event(self.t, {"type": "picked", "x": bd["x"], "y": bd["y"]})
                    self.wev({"type": "picked", "board": b}, self.lag); self.cup = b
                elif code == "LOST_ON_LIFT" and skill == "pick":         # sealed + lifted, fell back onto the table:
                    self.wev({"type": "dropped", "board": b}, self.lag)   # the skill drops it from the world
                elif code in ("LOST_IN_TRANSIT", "LOST_ON_LIFT"):          # the skill tells the world
                    self.fake.event(self.t, {"type": "dropped"}); self.wev({"type": "dropped", "board": self.cup}, self.lag)
                    self.cup = ""
                elif code == "NOT_HOLDING":                              # silent drop: the world is NOT told (D7)
                    self.fake.event(self.t, {"type": "dropped"}); self.cup = ""
                self.frames(DUR[skill] - pre)
                return code, "injected"
        delay = self.lag
        self.frames(pre)
        if skill == "pick":
            bd = self.board(b)
            if not any(abs(fb["x"] - bd["x"]) < 0.05 and abs(fb["y"] - bd["y"]) < 0.05 for fb in self.fake.boards):
                self.frames(DUR[skill] - pre)                # TM-36: the remembered board is gone: the contact
                return "GRIP_FAILED", "no vacuum (nothing there)"   # search finds nothing -> no seal
            self.fake.event(self.t, {"type": "picked", "x": bd["x"], "y": bd["y"]})
            self.wev({"type": "picked", "board": b}, delay)
            self.cup = b
        elif skill in ("place", "release_safe") and self.cup and w["held_board_id"]:
            target = a.get("target") or self.core.c["fault_zone"]
            z = self.zone(target)
            hb = w["held_board_id"]
            variant = self.board(hb)["variant"] if self.board(hb) else "red"
            self.fake.event(self.t, {"type": "placed", "x": z["x"], "y": z["y"], "yaw": z["yaw"], "variant": variant})
            self.wev({"type": "placed", "board": hb, "zone": target, "x": z["x"], "y": z["y"], "yaw": z["yaw"]}, delay)
            self.cup = ""
            if z["kind"] == "fault":
                self.fault_drops += 1
        if skill == "place" and a.get("skip_home"):         # TM-36: release + retract, the arm stays over the zone
            self.frames(min(1.0, DUR[skill] - pre))
            self.skip_homes += 1
            return ret, "ok (no home)"
        if skill in ("place", "release_safe"):
            self.frames(min(1.0, DUR[skill] - pre))         # release + lift, then the skill's home move
            self.tcp = tuple(MCFG["home_xy"])
            self.frames(max(0.0, DUR[skill] - pre - 1.0))
            return ret, "ok"
        self.frames(DUR[skill] - pre)
        return ret, "ok"

    # ---------- loop ----------
    def step(self):
        """One decision: run the next action (True) or let 0.2 s pass (False)."""
        w = self.wd()
        a = self.core.next_action(w, self.t, self.tcp)
        if a is None:
            self.frames(0.2)
            return False
        self.starts.append((self.t, a["skill"], w["vision_ok"], w["held_board_id"]))
        self._bad(pre_violations(self.core, a, w))
        self.core.started(a)
        code, msg = self.run(a)
        self.seq.append((a["skill"], a.get("job", ""), a.get("board", ""), a.get("target", a.get("stage", "")), code))
        self.core.on_result(a, code, msg, self.t)
        self._bad(state_violations(self.core))
        self._loops(self.seq[-1])
        for ev in getattr(self.core, "take_world_events", lambda: [])():
            self.wev(ev, self.lag)                     # the node publishes them on /task/events
        self.frames(0.1)                               # a world message newer than the result (CL-23)
        return True

    def _bad(self, texts):
        self.violations += [(round(self.t, 1), x) for x in texts]

    def _signature(self):
        """What counts as progress: job states + board stages, held board, problem boards, zone states / stacks."""
        w = self.world
        return (tuple((j["id"], j["state"], tuple(sorted(j["_items"].items()))) for j in self.core.jobs), w.held,
                tuple(sorted(self.core.problem)), self.core.halted,
                tuple((z["name"], z["state"], z["stack_count"], z["board_id"]) for z in w.zones.values()))

    def _loops(self, entry):
        sig = self._signature()
        n = self._sig[1] + 1 if self._sig[0] == sig else 0
        self._sig = (sig, n)
        if n == NO_PROGRESS:
            self._bad([f"no progress: {n} actions without any change (last {entry})"])
        if entry[-1] == "SUCCESS":
            self._same = (None, 0)
            k = self._same_ok[1] + 1 if self._same_ok[0] == entry else 1
            self._same_ok = (entry, k)
            if k == SAME_OK_MAX + 1:
                self._bad([f"loop: {entry} succeeded {k}x in a row"])
            return
        self._same_ok = (None, 0)
        n = self._same[1] + 1 if self._same[0] == entry else 1
        self._same = (entry, n)
        if n == SAME_FAIL_MAX + 1:
            self._bad([f"loop: {entry} failed {n}x in a row"])
        self.fail_times.append(self.t)
        recent = [x for x in self.fail_times if x > self.t - FAIL_WINDOW[0]]
        if len(recent) == FAIL_WINDOW[1] + 1:
            self._bad([f"loop: {len(recent)} failures within {FAIL_WINDOW[0]:.0f} s"])

    def idle(self):
        return all(j["state"] in FINAL for j in self.core.jobs) and not self.core.recovery

    def quiet(self):
        """Nothing can change any more without a person: HALTED, or every job DONE / CANCELLED / PAUSED with nothing
        on the cup and no recovery queued."""
        if self.core.halted:
            return True
        return (all(j["state"] in FINAL or j["_paused"] for j in self.core.jobs) and not self.core.recovery
                and not self.world.held and not self.core.paused)

    def loop(self, max_t=400.0, until=None, max_actions=200):
        """Run decisions. until=None: stop when quiet() (returns True if all jobs are DONE / CANCELLED)."""
        while self.t < max_t and len(self.seq) < max_actions:
            if until is not None and until(self):
                return True
            if not self.step() and until is None and self.quiet():
                break
        return until is None and all(j["state"] in FINAL for j in self.core.jobs)

    def add(self, **job):
        return self.core.add_job(job, self.wd(), self.t)


def short(seq):
    return [f"{s}:{j}" + (f"->{tg}" if s == "place" else "") for s, j, b, tg, c in seq if c == "SUCCESS"]


def jobs(sim):
    return {j["id"]: j for j in sim.core.status(sim.t)["jobs"]}


def events(sim, text):
    """All logged core events (t, level, text) containing `text` (the full log, not only the last 50)."""
    return [e for e in sim.core.events if text in e[2]]


def invariant_summary():
    """(name, ok, extra) for the end of a test file: the invariants over every scenario run in it."""
    bad = [(i, t, x) for i, sim in enumerate(ALL) if not sim.expect_bad for t, x in sim.violations]
    acts = sum(len(sim.seq) for sim in ALL)
    return (f"invariants held after every step in all {len(ALL)} scenarios ({acts} actions)", not bad,
            "; ".join(f"sim {i} t={t}: {x}" for i, t, x in bad[:8]))
