"""Offline checks of hk_fr5_task.main_core (T5b): the real world_core + fake_core on a SIMULATED clock with nominal
skill durations (pick 6 s, place 8 s, ...), so the simulated operator's processing overlaps exactly as on the robot.
Failures are injected per skill (simulation: tests/main_sim.py). Covers the user's scheduling example (CL-22), every failure policy, commands,
CL-23 / CL-24 / CL-25 / CL-26, claims, validation. No ROS, a few seconds.

    pixi run python3 tests/main_core_check.py
"""
import copy
import math
import sys

from main_sim import invariant_summary, BOARDS, MCFG, WCFG, Sim, jobs, short

ok = bad = 0
def check(name, cond, extra=""):
    global ok, bad
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True)
    ok += bool(cond); bad += not cond

# 1 the user's scheduling example (CL-22)
sim = Sim()
ok1, _, j1 = sim.add(type="process", select_variant="red", zone="processing_1", to="drop_off_1", count=2, priority=10)
ok2, _, j2 = sim.add(type="transfer", select_variant="white_small", to="drop_off_2", count=0, priority=5)
done = sim.loop()
want = ["pick:j1", "place:j1->processing_1", "pick:j2", "place:j2->drop_off_2", "pick:j1", "place:j1->drop_off_1",
        "pick:j1", "place:j1->processing_1", "pick:j2", "place:j2->drop_off_2", "pick:j1", "place:j1->drop_off_1",
        "pick:j2", "place:j2->drop_off_2"]
check("user example: exact order (T1 into processing, T2 meanwhile, processed board back -> T1 first, repeat)",
      done and short(sim.seq) == want, " | ".join(short(sim.seq)))
J = jobs(sim)
check("user example: j1 DONE 2/2, j2 DONE 3 (all), drop-offs 2 + 3, processing_1 EMPTY, nothing held",
      J[j1]["state"] == "DONE" and J[j1]["delivered"] == 2 and J[j2]["state"] == "DONE" and J[j2]["delivered"] == 3
      and sim.zone("drop_off_1")["stack_count"] == 2 and sim.zone("drop_off_2")["stack_count"] == 3
      and sim.zone("processing_1")["state"] == "EMPTY" and sim.wd()["held_board_id"] == "",
      f"j1 {J[j1]['state']} {J[j1]['delivered']}, j2 {J[j2]['state']} {J[j2]['delivered']}")
p1 = [b for s, j, b, tg, c in sim.seq if s == "pick" and j == j1]
check("processed boards keep their identity: the same id picked into and out of processing", p1[0] == p1[1] and p1[2] == p1[3], str(p1))
check("j2 'all' finished only after end_wait without a candidate (CL-24)", "all matching boards handled" in J[j2]["message"])
check("controller IDLE at the end, no problem boards", sim.core.state() == "IDLE" and not sim.core.problem)

# 2 priority change at runtime
sim = Sim()
_, _, a1 = sim.add(type="transfer", select_variant="red", to="drop_off_1", count=2, priority=10)
_, _, a2 = sim.add(type="transfer", select_variant="white_small", to="drop_off_2", count=1, priority=1)
sim.loop(until=lambda s: len(s.seq) >= 2)
sim.core.control("set_priority", a2, 20, sim.t)
sim.loop()
check("set_priority: the raised job runs at the next skill boundary (never mid-skill)",
      short(sim.seq)[:4] == ["pick:j1", "place:j1->drop_off_1", "pick:j2", "place:j2->drop_off_2"], " | ".join(short(sim.seq)))

# 3 nearest board to the current tcp (CL-26)
sim = Sim()
_, _, a = sim.add(type="transfer", select_variant="red", to="drop_off_1", count=1)
first = sim.core.next_action(sim.wd(), sim.t, (-0.62, 0.20))
reds = {b["id"]: (b["x"], b["y"]) for b in sim.wd()["boards"] if b["variant"] == "red"}
check("nearest red board to the current tcp chosen", first and math.dist(reds[first["board"]], (-0.62, 0.20)) < 0.01, str(first))

# 4 failure policies
sim = Sim(); _, _, a = sim.add(type="process", select_variant="red", zone="processing_1", to="drop_off_1", count=2)
sim.inject = [("pick", None, "GRIP_FAILED")]
sim.loop()
J = jobs(sim)[a]
check("GRIP_FAILED: board skipped (problem), the other red processed; count 2 not reachable -> DONE partial, 1 delivered + 1 skipped",
      J["state"] == "DONE" and J["delivered"] == 1 and J["skipped"] == 1 and J["attempts"] == 2 and "partial" in J["message"]
      and len(sim.core.problem) == 1, str(J))

sim = Sim(); _, _, a = sim.add(type="transfer", select_variant="white_small", to="drop_off_2", count=0)
sim.inject = [("place", None, "LOST_IN_TRANSIT")]
sim.loop()
J = jobs(sim)[a]
codes = [(s, c) for s, j, b, tg, c in sim.seq]
check("LOST_IN_TRANSIT: board counted lost, recovery home, the rest delivered (2 of the stack)",
      J["state"] == "DONE" and J["lost"] == 1 and J["delivered"] == 2 and J["attempts"] == 3 and ("home", "SUCCESS") in codes, str(codes))

sim = Sim(); _, _, a = sim.add(type="transfer", select_variant="red", to="drop_off_1", count=1)
sim.inject = [("pick", None, "OBSTACLE")]
sim.loop()
check("OBSTACLE: wait_clear, then re-decided from the visible boards (CL-33) -> a red picked + delivered, claim not kept",
      [(s, c) for s, j, b, tg, c in sim.seq][:3] == [("pick", "OBSTACLE"), ("wait_clear", "SUCCESS"), ("pick", "SUCCESS")]
      and jobs(sim)[a]["delivered"] == 1 and not sim.core.claims, " | ".join(f"{s}:{b}:{c}" for s, j, b, tg, c in sim.seq))

sim = Sim(); _, _, a = sim.add(type="transfer", select_variant="red", to="drop_off_1", count=1)
sim.inject = [("pick", None, "PLAN_FAILED")]
sim.loop()
check("PLAN_FAILED once: home, retry -> success", [(s, c) for s, j, b, tg, c in sim.seq][:3] ==
      [("pick", "PLAN_FAILED"), ("home", "SUCCESS"), ("pick", "SUCCESS")])

sim = Sim(); _, _, a = sim.add(type="transfer", select_variant="red", to="drop_off_1", count=2)
sim.inject = [("pick", None, "MOTION_FAILED"), ("home", None, "MOTION_FAILED"), ("home", None, "MOTION_FAILED")]   # a successful
#   recovery home in between would correctly reset the 'in a row' streak
sim.loop(until=lambda s: s.core.halted != "" and s.core.next_action(s.wd(), s.t, s.tcp) is None)
check("3 motion failures in a row -> HALTED, no actions", sim.core.state() == "HALTED", sim.core.halted)
sim.core.control("resume", "", 0, sim.t)
sim.loop()
check("resume after HALTED: the job continues to DONE", jobs(sim)[a]["state"] == "DONE", str(jobs(sim)[a]))

w = copy.deepcopy(WCFG); w["max_stack"] = 1
sim = Sim(wcfg=w); _, _, a = sim.add(type="transfer", select_variant="white_small", to="drop_off_2", count=0)
sim.loop(until=lambda s: jobs(s)[a]["state"] == "PAUSED")
check("drop-off FULL before the next pick: job PAUSED with a message, nothing held",
      jobs(sim)[a]["state"] == "PAUSED" and "FULL" in jobs(sim)[a]["message"] and sim.wd()["held_board_id"] == "")
sim.world.event(sim.t, {"type": "reset_zone", "zone": "drop_off_2"})
sim.core.control("resume", a, 0, sim.t)
sim.loop(until=lambda s: jobs(s)[a]["state"] == "PAUSED" and jobs(s)[a]["delivered"] >= 2, max_t=sim.t + 120)
check("operator emptied it + resume: exactly one more board delivered, then FULL again -> PAUSED again (max_stack 1)",
      jobs(sim)[a]["delivered"] == 2 and jobs(sim)[a]["state"] == "PAUSED" and sim.zone("drop_off_2")["stack_count"] == 1,
      f"{jobs(sim)[a]['state']} done {jobs(sim)[a]['delivered']}, stack {sim.zone('drop_off_2')['stack_count']}")

sim = Sim(); _, _, a = sim.add(type="transfer", select_variant="red", to="drop_off_1", count=1)
sim.inject = [("place", None, "STACK_FULL")]
sim.loop()
J = jobs(sim)[a]
check("STACK_FULL on place (holding): board to the fault zone, job PAUSED", J["state"] == "PAUSED" and J["to_fault"] == 1
      and sim.fault_drops == 1 and sim.wd()["held_board_id"] == "", str(J))

sim = Sim(); _, _, a = sim.add(type="transfer", select_variant="red", to="drop_off_1", count=1)
sim.inject = [("place", None, "NOT_VERIFIED+done")]           # placed, but the camera did not confirm it
sim.loop()
check("NOT_VERIFIED: counted delivered, alert logged", jobs(sim)[a]["delivered"] == 1
      and any("not confirmed" in e["text"] for e in sim.core.status(sim.t)["events"]))

sim = Sim(); _, _, a = sim.add(type="transfer", select_variant="red", to="drop_off_1", count=2)
sim.inject = [("pick", None, "BOARD_NOT_FOUND")] * 3
sim.loop()
check("BOARD_NOT_FOUND 3x on one board: skipped after 2 retries, the other red delivered",
      jobs(sim)[a]["delivered"] == 1 and len(sim.core.problem) == 1, str(jobs(sim)[a]))

# 5 processing FAULT (CL-19 expanded detection: a plain board comes back) -> fault zone
w = copy.deepcopy(WCFG); w["expanded_detection"] = True
sim = Sim(wcfg=w, return_variant="same")
_, _, a = sim.add(type="process", select_variant="red", zone="processing_1", to="drop_off_1", count=1)
sim.loop()
J = jobs(sim)[a]
check("not processed (plain board back): picked out into the fault zone; the 2nd red tried too (same) -> DONE partial, 0 delivered, 2 to fault",
      J["state"] == "DONE" and J["delivered"] == 0 and J["to_fault"] == 2 and "partial" in J["message"]
      and sim.fault_drops == 2, " | ".join(short(sim.seq)))

# 6 commands: pause (holding), stop (holding), cancel_job (holding), per-job pause / resume
sim = Sim(); _, _, a = sim.add(type="transfer", select_variant="red", to="drop_off_1", count=2)
sim.loop(until=lambda s: s.wd()["held_board_id"] != "")
sim.core.control("pause", "", 0, sim.t)
sim.loop(until=lambda s: s.core.next_action(s.wd(), s.t, s.tcp) is None and s.wd()["held_board_id"] == "", max_t=sim.t + 60)
n = len(sim.seq); sim.frames(5.0)
check("pause while holding: the held board is placed first, then nothing more",
      sim.seq[-1][0] == "place" and sim.core.next_action(sim.wd(), sim.t, sim.tcp) is None and len(sim.seq) == n)
sim.core.control("resume", "", 0, sim.t); sim.loop()
check("resume: the job finishes", jobs(sim)[a]["state"] == "DONE" and jobs(sim)[a]["delivered"] == 2)

sim = Sim(); _, _, a = sim.add(type="transfer", select_variant="red", to="drop_off_1", count=2)
sim.loop(until=lambda s: s.wd()["held_board_id"] != "")
sim.core.control("stop", "", 0, sim.t)
check("stop while holding: no action until resume (board stays on the cup)", sim.core.next_action(sim.wd(), sim.t, sim.tcp) is None)
sim.core.control("resume", "", 0, sim.t)
a1 = sim.core.next_action(sim.wd(), sim.t, sim.tcp)
check("resume after stop: the held board goes to the fault zone (release_safe, CL-25)", a1 and a1["skill"] == "release_safe", str(a1))
sim.core.started(a1); c, m = sim.run(a1); sim.core.on_result(a1, c, m, sim.t); sim.frames(0.1); sim.loop()
check("... then the job continues: 1 delivered, 1 to fault (no 3rd red -> DONE partial)", jobs(sim)[a]["delivered"] == 1
      and jobs(sim)[a]["to_fault"] == 1 and jobs(sim)[a]["state"] == "DONE",
      str(jobs(sim)[a]))

sim = Sim()
_, _, a = sim.add(type="transfer", select_variant="red", to="drop_off_1", count=2, priority=5)
_, _, b = sim.add(type="transfer", select_variant="white_small", to="drop_off_2", count=1, priority=1)
sim.loop(until=lambda s: s.wd()["held_board_id"] != "")
sim.core.control("cancel_job", a, 0, sim.t)
sim.loop()
check("cancel_job while holding: its board placed first, job CANCELLED, the other job runs",
      jobs(sim)[a]["state"] == "CANCELLED" and jobs(sim)[a]["delivered"] == 1 and jobs(sim)[b]["state"] == "DONE",
      f"{jobs(sim)[a]['state']} {jobs(sim)[a]['delivered']} / {jobs(sim)[b]['state']}")

sim = Sim()
_, _, a = sim.add(type="transfer", select_variant="red", to="drop_off_1", count=2, priority=5)
_, _, b = sim.add(type="transfer", select_variant="white_small", to="drop_off_2", count=1, priority=1)
sim.core.control("pause", a, 0, sim.t)
sim.loop(until=lambda s: jobs(s)[b]["state"] == "DONE")
check("pause one job: the other job runs, the paused one does nothing", jobs(sim)[a]["state"] == "PAUSED"
      and jobs(sim)[a]["delivered"] == 0 and jobs(sim)[b]["state"] == "DONE")
sim.core.control("resume", a, 0, sim.t); sim.loop()
check("resume that job: it finishes", jobs(sim)[a]["state"] == "DONE" and jobs(sim)[a]["delivered"] == 2)

# 7 CL-23, CL-24, claims, process without 'to', zone alerts, validation
sim = Sim(); _, _, a = sim.add(type="transfer", select_variant="red", to="drop_off_1", count=1)
act = sim.core.next_action(sim.wd(), sim.t, sim.tcp); sim.core.started(act); c, m = sim.run(act); sim.core.on_result(act, c, m, sim.t)
stale = sim.wd(); stale["stamp"] = sim.t - 0.05
check("CL-23: a world older than the last result -> no decision", sim.core.next_action(stale, sim.t, sim.tcp) is None)

sim = Sim(); _, _, a = sim.add(type="transfer", select_variant="red", to="drop_off_1", count=0)
sim.loop(until=lambda s: jobs(s)[a]["delivered"] == 2)
t_last = sim.t
sim.loop()
check("CL-24: 'all' ends end_wait s after the last candidate", jobs(sim)[a]["state"] == "DONE"
      and sim.t - t_last >= MCFG["end_wait"] - 0.25, f"{sim.t - t_last:.1f} s")

sim = Sim()
_, _, a = sim.add(type="transfer", select_variant="red", to="drop_off_1", count=0, priority=2)
_, _, b = sim.add(type="transfer", select_variant="red", to="drop_off_2", count=0, priority=1)
sim.loop()
picked = [bb for s, j, bb, tg, c in sim.seq if s == "pick" and c == "SUCCESS"]
check("claims: two jobs on the same variant never pick the same board; both reds delivered once",
      len(picked) == len(set(picked)) == 2 and sim.zone("drop_off_1")["stack_count"] + sim.zone("drop_off_2")["stack_count"] == 2)

sim = Sim(); _, _, a = sim.add(type="process", select_variant="red", zone="processing_2", count=1)
sim.loop()
check("process without 'to': DONE when processed, board stays in the zone (DONE)", jobs(sim)[a]["state"] == "DONE"
      and jobs(sim)[a]["delivered"] == 1 and sim.zone("processing_2")["state"] == "DONE")

w = copy.deepcopy(WCFG); w["unknown_after"] = 0.5
sim = Sim(wcfg=w, boards=BOARDS + [{"variant": "red", "x": -0.50, "y": 0.45, "yaw": 0.0}])
sim.frames(1.0); sim.core.next_action(sim.wd(), sim.t, sim.tcp); sim.core.next_action(sim.wd(), sim.t, sim.tcp)
alerts = [e for e in sim.core.status(sim.t)["events"] if "not placed by the robot" in e["text"] and "remove it and reset the zone" in e["text"]]
check("unowned FAULT in a processing zone (unknown board): one verbose alert (CL-26 / CL-29)", len(alerts) == 1, str(alerts))

sim = Sim()
for job, text in [({"type": "dance"}, "unknown job type"), ({"type": "transfer", "select_variant": "blue", "to": "drop_off_1"}, "unknown variant"),
                  ({"type": "process", "zone": "drop_off_1"}, "processing zone"), ({"type": "process", "zone": "processing_1", "to": "processing_2"}, "'to' must be"),
                  ({"type": "transfer", "to": "nowhere"}, "target zone"), ({"type": "transfer", "to": "drop_off_1", "count": -2}, "count must be >= -1"),
                  ({"type": "transfer", "to": "drop_off_1", "max_attempts": -1}, "max_attempts >= 0"),
                  ({"type": "transfer", "to": "drop_off_1", "count": 3, "max_attempts": 2}, "max_attempts must be >= count"),
                  ({"type": "transfer", "to": "drop_off_1", "select_board": "b99"}, "not in the world"),
                  ({"type": "transfer", "to": "drop_off_1", "select_board": "b1", "count": 2}, "count must be 0 or 1"),
                  ({"type": "clear_zone", "from_zone": "drop_off_1", "to": "drop_off_1"}, "same zone"),
                  ({"type": "clear_zone", "from_zone": "fault", "to": "drop_off_1"}, "fault zone is a void"),
                  ({"type": "transfer", "from_zone": "fault", "to": "drop_off_1"}, "fault zone is a void")]:
    okk, msg, _ = sim.core.add_job(job, sim.wd(), sim.t)
    check(f"add_job rejects: {text}", not okk and text in msg, msg)
for cmd, jid, text in [("dance", "", "unknown command"), ("cancel_job", "", "needs a job_id"), ("pause", "j99", "unknown job")]:
    okk, msg = sim.core.control(cmd, jid, 0, sim.t)
    check(f"control rejects: {text}", not okk and text in msg, msg)

check(*invariant_summary())
print(f"{ok} passed, {bad} failed")
sys.exit(1 if bad else 0)
