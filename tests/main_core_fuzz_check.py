"""Seeded fuzz runs of the main controller core (T5b step 5): random layouts (CL-34), jobs, priorities, failure codes,
commands at random times, zone flicker, camera dropouts, world lag. A simple operator reacts ONLY to what the core
tells them (alerts, PAUSED jobs, HALTED), after a 5 s delay - so a missing / wrong alert shows up as a run that
never settles. Every run must keep the invariants (main_sim checker) and settle before T_MAX: all jobs DONE /
CANCELLED with nothing held, or HALTED after the operator's 3 resumes, or jobs left PAUSED after 5 resumes each.

    pixi run python3 tests/main_core_fuzz_check.py                 # seeds 0..19
    pixi run python3 tests/main_core_fuzz_check.py --seeds 100     # more
    pixi run python3 tests/main_core_fuzz_check.py --seed 17 -v    # one seed with its action trace
"""
import argparse
import math
import random
import re
import sys
import time
import traceback

from main_sim import ARM_R, FINAL, MCFG, WCFG, ZONES, Sim, invariant_summary, jobs
from hk_fr5_task import mask as M
from hk_fr5_task.world_core import in_rect, rect_corners

T_MAX = 900.0                   # s simulated
REACT = 5.0                     # s operator reaction time
CODES = {"pick": [("GRIP_FAILED", 3), ("OBSTACLE", 2), ("PLAN_FAILED", 1), ("MOTION_FAILED", 1), ("BOARD_NOT_FOUND", 2),
                  ("LOST_ON_LIFT", 1), ("OBSTACLE+held", 1)],
         "place": [("OBSTACLE", 2), ("MOTION_FAILED", 1), ("LOST_IN_TRANSIT", 1), ("NOT_HOLDING", 1),
                   ("NOT_VERIFIED+done", 2), ("TARGET_UNKNOWN", 1), ("STACK_FULL", 1), ("RELEASE_FAILED", 0.3)],
         "home": [("MOTION_FAILED", 1), ("OBSTACLE", 1)],
         "wait_clear": [("TIMEOUT", 1)],
         "release_safe": [("MOTION_FAILED", 1), ("OBSTACLE", 1)]}
ZNAMES = [z["name"] for z in ZONES]
STATS = {}                      # coverage over all seeds: what really happened


def stat(key, n=1):
    STATS[key] = STATS.get(key, 0) + n


def layout(rng):
    home_arm = [M.capsule((0.0, 0.0), tuple(MCFG["home_xy"]), ARM_R + 0.01)]
    boards, tries = [], 0
    n = rng.randint(3, 6)
    while len(boards) < n and tries < 3000:
        tries += 1
        v = rng.choice(list(WCFG["variants"]))
        x, y, yaw = rng.uniform(-0.75, -0.15), rng.uniform(-0.65, 0.65), rng.uniform(-0.5, 0.5)
        c = rect_corners(x, y, yaw, WCFG["variants"][v]["length"], WCFG["variants"][v]["width"])
        if any(abs(py) < 0.08 for px, py in c):                       # robot profile strip
            continue
        if any(in_rect(px, py, z, WCFG["zone_margin"] + 0.02) for z in ZONES for px, py in c + [[x, y]]):
            continue
        if M.hits(home_arm, c) or any(math.hypot(x - b["x"], y - b["y"]) < 0.22 for b in boards):
            continue
        boards.append({"variant": v, "x": x, "y": y, "yaw": yaw, "count": rng.choice([1, 1, 1, 2, 3])})
    return boards


def random_job(rng, variants):
    typ = rng.choices(["process", "transfer", "home"], [4, 4, 1])[0]
    if typ == "home":
        return {"type": "home"}
    count = rng.choice([0, 1, 1, 2, 3])
    job = {"type": typ, "select_variant": rng.choice(variants + [""]), "count": count, "priority": rng.randint(0, 10),
           "max_attempts": rng.choice([0, 0, count + 1, count + 2]) if count else 0}
    if typ == "process":
        job.update(zone=rng.choice(["processing_1", "processing_2"]), to=rng.choice(["drop_off_1", "drop_off_2", ""]))
    else:
        job.update(to=rng.choice(["drop_off_1", "drop_off_2"]))
    return job


class Operator:
    """Reacts after REACT s to: alerts naming a zone to reset / a job to cancel, PAUSED jobs, HALTED."""

    def __init__(self, sim, rng):
        self.sim, self.rng = sim, rng
        self.seen = 0                       # core events read so far
        self.todo = []                      # (t, fn)
        self.resumes, self.halts, self.own_pause = {}, 0, set()
        self.waiting_halt = self.waiting = False

    def later(self, fn, delay=REACT):
        self.todo.append((self.sim.t + delay, fn))

    def tick(self):
        sim, core = self.sim, self.sim.core
        for item in [x for x in self.todo if x[0] <= sim.t]:
            self.todo.remove(item); item[1]()
        for t, level, text in core.events[self.seen:]:
            if level != "error":
                continue
            if "reset the zone" in text:
                z = self._zone_in(text)
                if z:
                    stat("operator clears a zone (alert)")
                    self.later(lambda z=z: sim.operator_clear(z))
            if "placed, not taken for processing" in text:      # processing missed (e.g. arm covered the zone):
                z = self._zone_in(text)                          # take it out again and put it back, visibly
                if z:
                    stat("operator redoes the processing (LOADED alert)")
                    ret = sim.fake.s["operator"]["return_after"]
                    self.later(lambda z=z, ret=ret: sim.fake.pending.extend(
                        [(sim.t, "take", z), (sim.t + ret, "return", z)]))
            m = re.search(r"cancel (j\d+)", text)
            if m and "or put" in text:
                stat("operator cancels a job (alert)")
                self.later(lambda jid=m.group(1): core.control("cancel_job", jid, 0, sim.t))
        self.seen = len(core.events)
        for j in core.jobs:                 # a job the core paused: fix the reason, resume (max 5x)
            if j["state"] == "PAUSED" and j["id"] not in self.own_pause and not any(x[2:] == ("resume", j["id"]) for x in self.todo):
                if self.resumes.get(j["id"], 0) < 5:
                    self.resumes[j["id"]] = self.resumes.get(j["id"], 0) + 1
                    stat("operator resumes a PAUSED job")
                    z = self._zone_in(j["message"]) if "FULL" in j["message"] else None
                    def fix(j=j, z=z):
                        if z:
                            sim.operator_clear(z)
                        if j["state"] == "PAUSED":
                            core.control("resume", j["id"], 0, sim.t)
                    self.todo.append((sim.t + REACT, fix, "resume", j["id"]))
        if core.halted and not self.waiting_halt and self.halts < 3:
            self.halts += 1
            stat("HALTED (operator resumes)")
            self.waiting_halt = True
            def unhalt():
                sim.take_off_cup()          # whatever is on the cup, then resume
                core.control("resume", "", 0, sim.t)
                self.waiting_halt = False
            self.later(unhalt)

    def _zone_in(self, text):
        hits = [(text.find(n), n) for n in ZNAMES if n in text]
        return min(hits)[1] if hits else None

    def busy(self):
        return bool(self.todo) or self.waiting_halt


def run_seed(seed, verbose=False):
    rng = random.Random(seed)
    boards = layout(rng)
    wcfg = dict(WCFG)
    ret = "same"
    if rng.random() < 0.2:
        wcfg["expanded_detection"] = True
        ret = rng.choice(["same", "nonvariant"])
    op_cfg = {"take_after": rng.uniform(1.0, 15.0), "return_after": rng.uniform(2.0, 10.0)}
    sim = Sim(boards=boards, wcfg=wcfg, return_variant=ret, lag=rng.choice([0.0, 0.0, 0.3]), operator=op_cfg)
    global LAST
    LAST = sim                              # for debugging one seed
    p_fail = rng.uniform(0.10, 0.35)
    injected = []
    def chaos(a):
        if rng.random() >= p_fail:
            return None
        names, weights = zip(*CODES.get(a["skill"], [("", 0)]))
        if not sum(weights):
            return None
        c = rng.choices(names, weights)[0]
        injected.append(c); stat(f"inject {a['skill']} {c}")
        return c
    sim.chaos = chaos
    variants = sorted({b["variant"] for b in boards})
    added = []
    for _ in range(rng.randint(2, 4)):
        ok, msg, jid = sim.add(**random_job(rng, variants))
        added.append((ok, msg)); stat("job added" if ok else "job rejected")
    op = Operator(sim, rng)
    if rng.random() < 0.3:                  # one more job later
        t_add = rng.uniform(10, 120)
        sim.timeline.append((t_add, lambda s: added.append(s.add(**random_job(rng, variants))[:2])))
    for _ in range(rng.randint(1, 5)):      # commands at random times
        t_c = rng.uniform(5, 200)
        kind = rng.choice(["pause", "stop", "cancel", "prio", "job_pause", "home"])
        def cmd(s, kind=kind):
            core, js = s.core, [j["id"] for j in s.core.jobs]
            stat(f"command {kind}" + (" (holding)" if s.world.held else ""))
            if kind == "pause":
                core.control("pause", "", 0, s.t); op.later(lambda: core.control("resume", "", 0, s.t), rng.uniform(2, 10))
            elif kind == "stop":
                core.control("stop", "", 0, s.t)
                if rng.random() < 0.5 and s.world.held:
                    stat("operator takes the board off the cup (stop)")
                    op.later(s.take_off_cup, 1.0)
                op.later(lambda: core.control("resume", "", 0, s.t), rng.uniform(3, 10))
            elif kind == "cancel" and js:
                core.control("cancel_job", rng.choice(js), 0, s.t)
            elif kind == "prio" and js:
                core.control("set_priority", rng.choice(js), rng.randint(0, 20), s.t)
            elif kind == "job_pause" and js:
                jid = rng.choice(js)
                if core.control("pause", jid, 0, s.t)[0]:
                    op.own_pause.add(jid)
                    def res(jid=jid):
                        op.own_pause.discard(jid); core.control("resume", jid, 0, s.t)
                    op.later(res, rng.uniform(3, 15))
            elif kind == "home":
                core.control("home", "", 0, s.t)
        sim.timeline.append((t_c, cmd))
    for _ in range(rng.randint(0, 3)):      # zone flicker
        t_h, zname, dur = rng.uniform(5, 250), rng.choice(ZNAMES), rng.uniform(0.5, 3.0)
        sim.timeline += [(t_h, lambda s, z=zname: s.hide.add(z)), (t_h + dur, lambda s, z=zname: s.hide.discard(z))]
    for _ in range(rng.randint(0, 2)):      # camera dropouts
        t_b, dur = rng.uniform(5, 250), rng.uniform(0.5, 4.0)
        sim.timeline += [(t_b, lambda s: setattr(s, "blind", True)), (t_b + dur, lambda s: setattr(s, "blind", False))]

    settled, why = False, ""
    last_held = ""
    while sim.t < T_MAX and len(sim.seq) < 600:
        held = sim.world.held
        if held and held != last_held:                       # event-driven disturbances while carrying
            r = rng.random()
            lp = getattr(sim.core, "_last_pick", None) or {}
            j = sim.core.job(lp.get("job", "")) if lp.get("board") == held else None
            target = {"to_zone": j["zone"], "to_out": j["to"], "to_target": j["to"]}.get(lp.get("stage")) if j else None
            if r < 0.15:
                stat("disturbance: stop while holding")
                sim.core.control("stop", "", 0, sim.t)
                if rng.random() < 0.5:
                    stat("operator takes the board off the cup (stop)")
                    op.later(sim.take_off_cup, 1.0)
                op.later(lambda: sim.core.control("resume", "", 0, sim.t), rng.uniform(3, 8))
            elif r < 0.27 and target:
                stat("disturbance: target zone not seen while holding")
                dur = rng.uniform(15, 25)
                sim.hide.add(target)
                sim.timeline.append((sim.t + dur, lambda s, z=target: s.hide.discard(z)))
            elif r < 0.33 and target and target.startswith("processing"):
                stat("disturbance: unknown board put into the processing target while holding")
                z = next(z for z in ZONES if z["name"] == target)
                sim.fake._add("white_small", z["x"] + 0.01, z["y"], 0.0)
        last_held = held
        op.tick()
        t0 = sim.t
        moved = sim.step()
        if not moved:                                        # parking question: where does the idle arm wait?
            away = math.dist(sim.tcp, MCFG["home_xy"]) > MCFG["home_tol"]
            if away:
                stat("time idle away from home (s)", round(sim.t - t0, 1))
                if sim.world.held:
                    stat("time HOLDING idle away from home (s)", round(sim.t - t0, 1))
                band = [M.capsule((0.0, 0.0), sim.tcp, ARM_R)]
                if any(z["kind"] == "processing" and M.hits(band, rect_corners(z["x"], z["y"], z["yaw"], z["length"], z["width"]))
                       for z in ZONES):
                    stat("time a processing zone covered while idle (s)", round(sim.t - t0, 1))
        if not moved and not op.busy() and not sim.timeline and sim.quiet():
            if sim.core.halted and op.halts >= 3:
                settled, why = True, "HALTED after 3 resumes"; break
            if not sim.core.halted:
                left = [j for j in sim.core.jobs if j["state"] not in FINAL]
                if not left:
                    settled, why = True, "all jobs DONE / CANCELLED"; break
                if all(op.resumes.get(j["id"], 0) >= 5 for j in left):
                    settled, why = True, "PAUSED after 5 resumes"; break
    J = jobs(sim)
    for key in ("delivered", "to_fault", "lost", "skipped"):
        stat(f"boards {key}", sum(j[key] for j in J.values() if j["type"] != "home"))
    stat("actions", len(sim.seq))
    stat("alerts / errors logged", sum(1 for e in sim.core.events if e[1] == "error"))
    summary = (f"{len(J)} jobs ({', '.join(f'{k} {v}' for k, v in sorted(_count(J).items()))}), {len(sim.seq)} actions, "
               f"{len(injected)} failures injected, {op.halts} halts, t={sim.t:.0f} s")
    if verbose:
        print(f"  layout {boards}\n  jobs {[a for a in added]}")
        for x in sim.seq:
            print("   ", x)
        for e in sim.core.events[-25:]:
            print("    ev", round(e[0], 1), e[1], e[2][:160])
        for x in sim.violations:
            print("    VIOLATION", x)
        print(f"    operator cfg {sim.fake.s.get('operator')}, pending {sim.fake.pending}")
        for t, x in sim.fake.log:
            print("    fake", round(t, 1), x)
        for t, x in sim.world.log:
            if x.startswith("zone"):
                print("    world", round(t, 1), x)
    ok = settled and not sim.violations and not sim.world.held
    return ok, summary + (f" - {why}" if settled else " - DID NOT SETTLE") + \
        (f" - {len(sim.violations)} violations: {sim.violations[:2]}" if sim.violations else "")


def _count(J):
    out = {}
    for j in J.values():
        out[j["state"]] = out.get(j["state"], 0) + 1
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("-v", action="store_true")
    a = ap.parse_args()
    seeds = [a.seed] if a.seed is not None else range(a.start, a.start + a.seeds)
    ok = bad = 0
    t0 = time.time()
    for sd in seeds:
        try:
            good, text = run_seed(sd, a.v)
        except Exception as e:                               # noqa: BLE001 - a crash is this seed's failure
            tb = traceback.extract_tb(e.__traceback__)[-3:]
            good, text = False, f"EXCEPTION {type(e).__name__}: {e} @ " + " <- ".join(f"{f.name}:{f.lineno}" for f in reversed(tb))
        print(("PASS " if good else "FAIL ") + f"seed {sd}: {text}", flush=True)
        ok += good; bad += not good
    print("coverage: " + ", ".join(f"{k} {v}" for k, v in sorted(STATS.items())))
    name, good, extra = invariant_summary()
    print(("PASS " if good else "FAIL ") + name + (f"  [{extra}]" if extra else ""))
    ok += good; bad += not good
    print(f"total {time.time() - t0:.0f} s")
    print(f"{ok} passed, {bad} failed")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
