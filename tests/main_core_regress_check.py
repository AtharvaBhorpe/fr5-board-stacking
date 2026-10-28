"""Regression checks for the main-core review bugs (decision-log TM-16): one scenario per bug, built so that the bug
shows (an endless loop, a wrong target, a missing halt / alert) and the fix makes it pass. Simulation: main_sim.py.

    pixi run python3 tests/main_core_regress_check.py
    MAIN_CORE=/path/to/old/main_core.py pixi run python3 tests/main_core_regress_check.py   # prove the bugs on an old core
(review 1 checks fail on the pre-review core, review 2 checks on the review-1 core)
"""
import copy
import sys
import traceback

from main_sim import invariant_summary, MCFG, WCFG, Sim, events, jobs, short

ok = bad = 0
def check(name, cond, extra=""):
    global ok, bad
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True)
    ok += bool(cond); bad += not cond

def codes(sim):
    return [(s, c) for s, j, b, tg, c in sim.seq]

def fails(sim, code):
    return sum(1 for s, j, b, tg, c in sim.seq if c == code)

def cnt(j, key):
    return j.get(key, "n/a")

def held(sim):
    return sim.wd()["held_board_id"] != ""

def red_transfer(sim, count=2):
    return sim.add(type="transfer", select_variant="red", to="drop_off_1", count=count)[2]

def case(name):
    """Run one scenario; a crash of the core counts as a failed check (the old core may raise)."""
    def deco(fn):
        try:
            fn()
        except Exception as e:                               # noqa: BLE001 - report and go on with the next case
            check(f"{name}: no exception", False, f"{type(e).__name__}: {e} @ {traceback.extract_tb(e.__traceback__)[-1].lineno}")
        return fn
    return deco


@case("D3")
def d3():
    for wait in (0.1, 5.0):                                  # resume right away / after a while
        sim = Sim(); a = red_transfer(sim)
        sim.core.control("stop", "", 0, sim.t)               # nothing on the cup
        sim.frames(wait)
        sim.loop(max_t=sim.t + wait + 0.5, until=lambda s: False)       # paused ticks
        sim.core.control("resume", "", 0, sim.t)
        sim.loop(max_t=sim.t + 120)
        places = [(s, tg) for s, j, b, tg, c in sim.seq if s in ("place", "release_safe")]
        check(f"D3 stop while NOT holding, resume after {wait} s: boards go to their target, none to the fault zone",
              places and all(p == ("place", "drop_off_1") for p in places) and cnt(jobs(sim)[a], "to_fault") in (0, "n/a"),
              str(places))


@case("D3b")
def d3b():
    sim = Sim(); a = red_transfer(sim)
    act = sim.core.next_action(sim.wd(), sim.t, sim.tcp)
    sim.core.started(act)
    sim.core.control("stop", "", 0, sim.t)               # during the pick; the pick still succeeds
    c, m = sim.run(act); sim.core.on_result(act, c, m, sim.t); sim.frames(1.0)
    none_paused = sim.core.next_action(sim.wd(), sim.t, sim.tcp) is None
    sim.core.control("resume", "", 0, sim.t)
    nxt = sim.core.next_action(sim.wd(), sim.t, sim.tcp)
    check("D3b stop DURING a pick that still succeeds: held until resume, then release_safe to the fault zone (CL-25)",
          act["skill"] == "pick" and c == "SUCCESS" and none_paused and nxt is not None and nxt["skill"] == "release_safe",
          f"{c}, paused-none {none_paused}, next {nxt}")


@case("D7")
def d7():
    sim = Sim(); a = red_transfer(sim)
    sim.inject = [("place", None, "NOT_HOLDING")]           # board falls off unseen; the world still says 'held'
    sim.loop(max_t=200)
    J = jobs(sim)[a]
    check("D7 NOT_HOLDING (silent drop): no place loop, the world drops the board, the job goes on",
          fails(sim, "NOT_HOLDING") == 1 and not held(sim) and cnt(J, "lost") == 1 and J["state"] == "DONE",
          f"NOT_HOLDING x{fails(sim, 'NOT_HOLDING')}, {J['state']}, lost {cnt(J, 'lost')}")


@case("E3")
def e3():
    sim = Sim(); a = red_transfer(sim, 1)
    sim.inject = [("place", None, "OBSTACLE")]
    sim.loop()
    check("E3 OBSTACLE during a place (holding): wait_clear first, then the place again",
          codes(sim)[:4] == [("pick", "SUCCESS"), ("place", "OBSTACLE"), ("wait_clear", "SUCCESS"), ("place", "SUCCESS")],
          str(codes(sim)[:5]))


@case("D5")
def d5():
    sim = Sim(wcfg=dict(WCFG, fixed_zones=[])); a = red_transfer(sim, 1)    # no configured fault zone fallback
    sim.loop(until=held)
    sim.hide = {"drop_off_1", "fault"}                      # target AND fault zone not seen
    sim.frames(MCFG["fresh"] + 0.5)                          # older than 'fresh'
    sim.loop(max_t=sim.t + 120, max_actions=60)
    check("D5 holding, target and fault zone unknown: 3 tries at the target, 3 at the fault zone, then HALTED (no loop)",
          sim.core.halted != "" and fails(sim, "TARGET_UNKNOWN") <= 6 and held(sim),
          f"halted '{sim.core.halted}', TARGET_UNKNOWN x{fails(sim, 'TARGET_UNKNOWN')}")


@case("P")
def p():
    sim = Sim(); a = red_transfer(sim, 1)
    sim.loop(until=held)
    sim.hide = {"drop_off_1"}                               # zone flicker: back 1 s later
    sim.frames(MCFG["fresh"] + 0.5)                          # older than 'fresh'
    sim.timeline.append((sim.t + 1.0, lambda s: s.hide.clear()))
    sim.loop(max_t=sim.t + 120)
    J = jobs(sim)[a]
    check("P target briefly not seen (flicker): the core waits (no place try), then delivers there, job not paused",
          fails(sim, "TARGET_UNKNOWN") == 0 and J["state"] == "DONE" and sim.zone("drop_off_1")["stack_count"] == 1
          and sim.fault_drops == 0, f"{codes(sim)} {J['state']}")
    sim = Sim(); a = red_transfer(sim, 1)
    sim.inject = [("place", None, "TARGET_UNKNOWN")]         # skill and core disagree once
    sim.loop(max_t=120)
    J = jobs(sim)[a]
    check("P skill reports the target unknown once (core sees it): retried after retry_wait, delivered there",
          codes(sim)[:3] == [("pick", "SUCCESS"), ("place", "TARGET_UNKNOWN"), ("place", "SUCCESS")] and J["state"] == "DONE"
          and sim.zone("drop_off_1")["stack_count"] == 1, f"{codes(sim)} {J['state']}")


@case("P/R1")
def p_r1():
    sim = Sim(); a = red_transfer(sim, 1)
    sim.inject = [("place", None, "TARGET_UNKNOWN")] * 3     # skill keeps refusing, the core sees the zone
    sim.loop(max_t=120, until=lambda s: s.seq and not held(s) and s.seq[-1][0] == "place" and s.seq[-1][-1] == "SUCCESS")
    J = jobs(sim)[a]
    check("P/R1 skill refuses the target 3x: job PAUSED, board into the fault zone (fresh tries there, not HALTED)",
          fails(sim, "TARGET_UNKNOWN") == 3 and J["state"] == "PAUSED" and not sim.core.halted
          and sim.fault_drops == 1 and not held(sim),
          f"TARGET_UNKNOWN x{fails(sim, 'TARGET_UNKNOWN')}, {J['state']}, halted '{sim.core.halted}'")


@case("CL-28 hold")
def hold_unseen():
    sim = Sim(); a = red_transfer(sim, 1)
    sim.loop(until=held)
    sim.hide = {"drop_off_1"}                               # target gone for good while holding, fault zone seen
    sim.frames(MCFG["fresh"] + 0.5)                          # older than 'fresh' -> the core sees it as not seen
    t_held = sim.t
    sim.loop(max_t=sim.t + 120, until=lambda s: not held(s) or s.core.halted)
    J, al = jobs(sim)[a], events(sim, "holding board")
    t_place = next((x[0] for x in sim.starts if x[1] == "place"), None)
    check("CL-28 holding, target not seen: no place try there, alert after 10 s, fault zone after 10 s more, job PAUSED",
          fails(sim, "TARGET_UNKNOWN") == 0 and len(al) == 1 and al[0][0] - t_held >= MCFG["hold_alert_after"]
          and t_place is not None and t_place - t_held >= MCFG["hold_alert_after"] + MCFG["hold_fault_after"]
          and J["state"] == "PAUSED" and not sim.core.halted and sim.fault_drops == 1 and not held(sim),
          f"{codes(sim)}, {len(al)} alerts, place at +{(t_place or 0) - t_held:.1f} s, {J['state']}")


@case("H")
def h():
    sim = Sim(); a = red_transfer(sim)
    t0 = sim.t + 4.0                                         # camera down in the middle of the first pick
    sim.timeline += [(t0, lambda s: setattr(s, "blind", True)), (t0 + 6.0, lambda s: setattr(s, "blind", False))]
    sim.loop(max_t=200)
    blind_starts = [x for x in sim.starts if not x[2]]
    check("H camera down for 6 s: no action started while vision is stale, one alert, the job finishes",
          not blind_starts and len(events(sim, "no fresh Vision 1 data")) == 1 and jobs(sim)[a]["state"] == "DONE"
          and cnt(jobs(sim)[a], "delivered") == 2,
          f"{len(blind_starts)} blind starts, {len(events(sim, 'no fresh Vision 1 data'))} alerts, {jobs(sim)[a]['state']}")


@case("D6")
def d6():
    sim = Sim(); a = red_transfer(sim, 1)
    sim.inject = [("place", None, "RELEASE_FAILED")]
    sim.loop(max_t=100, until=lambda s: s.core.halted != "" or s.idle())
    check("D6 RELEASE_FAILED: HALTED (a human checks the gripper), nothing else runs",
          sim.core.halted != "" and "release" in sim.core.halted and codes(sim)[-1] == ("place", "RELEASE_FAILED"),
          f"halted '{sim.core.halted}', {codes(sim)}")


@case("U")
def u():
    sim = Sim(lag=0.5)                                       # world events reach the world 0.5 s after the result
    _, _, j1 = sim.add(type="process", select_variant="red", zone="processing_1", to="drop_off_1", count=2, priority=10)
    _, _, j2 = sim.add(type="transfer", select_variant="white_small", to="drop_off_2", count=0, priority=5)
    sim.loop()
    want = ["pick:j1", "place:j1->processing_1", "pick:j2", "place:j2->drop_off_2", "pick:j1", "place:j1->drop_off_1",
            "pick:j1", "place:j1->processing_1", "pick:j2", "place:j2->drop_off_2", "pick:j1", "place:j1->drop_off_1",
            "pick:j2", "place:j2->drop_off_2"]
    bad_codes = [c for s, j, b, tg, c in sim.seq if c != "SUCCESS"]
    check("U world lags 0.5 s behind the skill results: no re-pick / ALREADY_HOLDING / NOT_HOLDING, the user example unchanged",
          not bad_codes and short(sim.seq) == want, f"{bad_codes} | {' | '.join(short(sim.seq))}")


@case("V")
def v():
    sim = Sim(); a = red_transfer(sim)
    sim.cup = "something"                                    # on the cup at start, unknown to the world
    sim.loop(max_t=100, max_actions=30, until=lambda s: s.core.halted != "")
    check("V gripper holds something the world does not know: HALTED after the first ALREADY_HOLDING",
          sim.core.halted != "" and "ALREADY_HOLDING" in sim.core.halted and fails(sim, "ALREADY_HOLDING") == 1,
          f"halted '{sim.core.halted}', ALREADY_HOLDING x{fails(sim, 'ALREADY_HOLDING')}")


@case("E2")
def e2():
    sim = Sim(); a = red_transfer(sim, 1)
    sim.inject = [("pick", None, "OBSTACLE"), ("wait_clear", None, "TIMEOUT"), ("wait_clear", None, "TIMEOUT")]
    sim.loop()
    check("E2 wait_clear TIMEOUT: keeps waiting (no move with the hand there), one alert, then the pick",
          codes(sim)[:5] == [("pick", "OBSTACLE"), ("wait_clear", "TIMEOUT"), ("wait_clear", "TIMEOUT"),
                             ("wait_clear", "SUCCESS"), ("pick", "SUCCESS")] and len(events(sim, "still not clear")) == 1,
          f"{codes(sim)[:6]}, {len(events(sim, 'still not clear'))} alerts")


@case("B3")
def b3():
    w = copy.deepcopy(WCFG); w["process_timeout"] = 10.0
    sim = Sim(wcfg=w, operator={"return_after": 1000.0})    # the board never comes back
    _, _, a = sim.add(type="process", select_variant="red", zone="processing_1", to="drop_off_1", count=1)
    sim.loop(max_t=60, until=lambda s: False)
    J = jobs(sim)[a]
    check("B3 processing overdue: shown in the job message + one alert",
          "overdue" in J["message"] and len(events(sim, "not back yet")) == 1,
          f"'{J['message']}', {len(events(sim, 'not back yet'))} alerts")


@case("R6")
def r6():
    sim = Sim(operator={"return_after": 1000.0})
    _, _, a = sim.add(type="process", select_variant="red", zone="processing_1", to="drop_off_1", count=1)
    sim.loop(max_t=100, until=lambda s: s.zone("processing_1")["state"] == "PROCESSING")
    sim.world.event(sim.t, {"type": "reset_zone", "zone": "processing_1"})     # operator clears the zone
    sim.loop(max_t=sim.t + 20, until=lambda s: False)
    J = jobs(sim)[a]
    check("R6 our board no longer tracked in its processing zone: one alert + job message, no new board put in",
          len(events(sim, "is no longer tracked")) == 1 and "no longer in" in J["message"]
          and sum(1 for s, j, b, tg, c in sim.seq if s == "place" and tg == "processing_1") == 1,
          f"'{J['message']}', {len(events(sim, 'is no longer tracked'))} alerts, {short(sim.seq)}")


# ---------- review 2 (decision-log TM-16, CL-28..CL-32): fail on the review-1 core, pass on the current one ----------
def process_job(sim, **kw):
    return sim.add(type="process", select_variant="red", zone="processing_1", to="drop_off_1", count=1, **kw)[2]


@case("A1")
def a1():
    for old in (True, False):
        sim = Sim(); a = red_transfer(sim)
        sim.old_release_safe, sim.expect_bad = old, old
        sim.loop(until=held)
        sim.core.control("stop", "", 0, sim.t)
        sim.take_off_cup()                                   # the operator takes the board off during the stop
        sim.frames(1.0)
        sim.core.control("resume", "", 0, sim.t)
        sim.loop(max_t=sim.t + 120, max_actions=40)
        rs = [c for s, j, b, tg, c in sim.seq if s == "release_safe"]
        if old:
            check("A1 old release_safe (SUCCESS with an empty cup): the progress detector catches the success loop",
                  any("succeeded" in x or "no progress" in x for _, x in sim.violations), f"{rs[:6]}, {sim.violations[:2]}")
        else:
            J = jobs(sim)[a]
            check("A1 stop -> board taken off the cup -> resume: release_safe NOT_HOLDING once, board lost, job goes on",
                  rs == ["NOT_HOLDING"] and cnt(J, "lost") == 1 and cnt(J, "delivered") == 1 and not held(sim),
                  f"{rs}, lost {cnt(J, 'lost')}, delivered {cnt(J, 'delivered')}")


@case("A2")
def a2():
    sim = Sim(); a = red_transfer(sim, 1)
    sim.inject = [("pick", None, "OBSTACLE+held")]          # a hand during the lift, the board is already on the cup
    sim.loop()
    J = jobs(sim)[a]
    check("A2 pick fails after the board is on the cup: its job takes it over and places it at the target (not the fault zone)",
          sim.zone("drop_off_1")["stack_count"] == 1 and sim.fault_drops == 0 and cnt(J, "delivered") == 1
          and J["attempts"] == 1, f"{codes(sim)}, {J['message']}")


@case("A3")
def a3():
    sim = Sim(operator={"return_after": 3.0})
    a = process_job(sim)
    sim.loop(until=lambda s: s.zone("processing_1")["state"] == "DONE")
    sim.inject = [("pick", None, "GRIP_FAILED")]            # the processed board cannot be picked out
    sim.loop(max_t=sim.t + 30, until=lambda s: False)
    al = events(sim, "remove it by hand and reset the zone")
    check("A3/CL-29 processed board skipped (GRIP_FAILED): one alert 'skipped - remove it by hand and reset the zone'",
          len(al) == 1 and "processing_1 DONE" in al[0][2] and "skipped" in al[0][2], str([e[2] for e in al]))
    sim = Sim()
    sim.add(type="transfer", select_variant="red", to="processing_2", count=1)    # nobody takes it out afterwards
    sim.loop(max_t=80, until=lambda s: False)
    al = events(sim, "no active job will take it")
    check("A3/CL-29 board processed in a zone no active job owns: one alert 'no active job will take it'",
          len(al) == 1 and "processing_2 DONE" in al[0][2], str([e[2] for e in al]))


@case("A4/CL-30")
def a4():
    bs = [{"variant": "red", "x": -0.30, "y": 0.30, "yaw": 0.0},          # both input boards, both seen with the arm
          {"variant": "white_small", "x": -0.17, "y": 0.17, "yaw": 0.0}]    # at home (above the profile, TM-25); the
    #                                                                       white one lies on the base -> red line: under the arm
    #                                                                       while it hovers over the red
    sim = Sim(boards=bs)
    _, _, ar = sim.add(type="transfer", select_variant="red", to="drop_off_1", count=1, priority=10)
    _, _, aw = sim.add(type="transfer", select_variant="white_small", to="drop_off_2", count=0, priority=1)
    sim.inject = [("pick", None, "GRIP_FAILED")]            # the red fails, the arm stays at hover over it
    sim.loop()
    J = jobs(sim)[aw]
    check("A4/CL-30 'all' job whose board is hidden by the arm at hover: home first, then it is delivered (not DONE early)",
          cnt(J, "delivered") == 1 and ("home", "SUCCESS") in codes(sim),
          f"{codes(sim)}, {J['message']}")


@case("A5/CL-28")
def a5():
    sim = Sim(operator={"take_after": 10.0, "return_after": 3.0})
    a = process_job(sim)
    sim.loop(until=lambda s: s.wd()["held_board_id"] == "" and s.zone("processing_1")["state"] == "LOADED")
    sim.world.zones["drop_off_1"]["stack_count"] = WCFG["max_stack"]          # the drop-off fills up meanwhile
    sim.loop(max_t=sim.t + 40)
    J = jobs(sim)[a]
    check("A5/CL-28 processed board DONE, drop-off FULL: not picked out, job PAUSED, the board stays in the zone",
          J["state"] == "PAUSED" and "FULL" in J["message"] and sim.zone("processing_1")["state"] == "DONE"
          and sim.fault_drops == 0 and not any(tg == "to_out" for s, j, b, tg, c in sim.seq),
          f"{codes(sim)}, {J['state']} '{J['message']}'")
    sim = Sim(operator={"take_after": 10.0, "return_after": 3.0})
    a = process_job(sim)
    sim.loop(until=lambda s: s.wd()["held_board_id"] == "" and s.zone("processing_1")["state"] == "LOADED")
    sim.hide = {"drop_off_1"}
    sim.timeline.append((sim.t + 25.0, lambda s: s.hide.clear()))
    sim.loop(max_t=sim.t + 80)
    J = jobs(sim)[a]
    al = events(sim, "drop_off_1 not seen")
    check("A5/CL-28 drop-off not seen when the board is DONE: waits (alert after 10 s), picks it out once seen, delivered",
          len(al) == 1 and cnt(J, "delivered") == 1 and sim.fault_drops == 0, f"{codes(sim)}, {len(al)} alerts")


@case("A6")
def a6():
    sim = Sim(); a = red_transfer(sim)
    sim.inject = [("place", None, "LOST_IN_TRANSIT")]       # -> recovery home queued
    sim.loop(until=lambda s: s.seq and s.seq[-1][-1] == "LOST_IN_TRANSIT")
    sim.core.control("stop", "", 0, sim.t)                  # before the recovery home starts
    n = len(sim.seq)
    sim.loop(max_t=sim.t + 5, until=lambda s: False)
    still = len(sim.seq) == n
    sim.core.control("resume", "", 0, sim.t)
    sim.loop(max_t=sim.t + 60, until=lambda s: len(s.seq) > n)
    check("A6 recovery home queued, then 'stop': nothing moves until resume, then the home runs first",
          still and sim.seq[n][0] == "home", f"still {still}, after resume {sim.seq[n:n + 2]}")


@case("A7")
def a7():
    sim = Sim(); a = red_transfer(sim, 1)
    sim.inject = [("pick", None, "MOTION_FAILED"), ("home", None, "OBSTACLE")]
    sim.loop()
    check("A7 recovery home hits OBSTACLE: wait_clear, then the home again (kept), then the pick",
          codes(sim)[:5] == [("pick", "MOTION_FAILED"), ("home", "OBSTACLE"), ("wait_clear", "SUCCESS"), ("home", "SUCCESS"),
                             ("pick", "SUCCESS")], str(codes(sim)[:6]))


@case("A8")
def a8():
    sim = Sim(operator={"take_after": 60.0})
    a = process_job(sim)
    sim.loop(until=lambda s: s.zone("processing_1")["state"] == "LOADED")
    sim.core.control("pause", "", 0, sim.t)
    okc = sim.core.control("cancel_job", a, 0, sim.t)
    check("A8 cancel while paused (board in a processing zone, nothing on the cup): CANCELLED at once",
          okc[0] and jobs(sim)[a]["state"] == "CANCELLED", f"{okc}, {jobs(sim)[a]['state']}")


@case("A9/CL-28")
def a9():
    sim = Sim(operator={"take_after": 60.0})
    a = process_job(sim)
    sim.loop(until=held)
    sim.fake._add("white_small", -0.50, 0.45, 0.0)            # someone puts a board into processing_1 during the carry
    sim.frames(WCFG["unknown_after"] + 0.5)                                          # (the carry takes time: unknown_after -> FAULT)
    t0 = sim.t
    sim.loop(max_t=sim.t + 60, until=lambda s: not held(s))
    J, al = jobs(sim)[a], events(sim, "holding board")
    t_place = next((x[0] for x in sim.starts if x[1] == "place"), None)
    check("A9/CL-28 holding for a processing zone that turns busy (FAULT): alert, fault zone after 20 s, job NOT paused",
          len(al) == 1 and t_place is not None and t_place - t0 >= MCFG["hold_alert_after"] + MCFG["hold_fault_after"]
          and sim.fault_drops == 1 and J["state"] != "PAUSED" and not held(sim),
          f"{codes(sim)}, {len(al)} alerts, place at +{(t_place or 0) - t0:.1f} s, {J['state']}")


@case("CL-29 LOADED")
def loaded():
    sim = Sim(operator={"take_after": 60.0})
    a = process_job(sim)
    sim.loop(max_t=70, until=lambda s: False)
    al = events(sim, "placed, not taken for processing")
    check("CL-29 processing zone LOADED > 30 s: one alert 'placed, not taken for processing'", len(al) == 1
          and "processing_1 LOADED" in al[0][2], str([e[2] for e in al]))


@case("CL-29 re-arm")
def rearm():
    sim = Sim()
    sim.fake._add("white_small", -0.50, 0.45, 0.0)            # an unknown board in processing_1
    sim.loop(max_t=sim.t + 4, until=lambda s: False)
    sim.fake.boards = [b for b in sim.fake.boards if sim.fake.zone_at(b["x"], b["y"]) is None]   # taken away
    sim.loop(max_t=sim.t + 4, until=lambda s: False)          # zone EMPTY again; the FAULT auto-clear forgets the board (TM-33)
    sim.fake._add("white_small", -0.50, 0.45, 0.0)            # put back: the same situation again
    sim.loop(max_t=sim.t + 4, until=lambda s: False)
    al = events(sim, "remove it and reset the zone")
    check("CL-29 an alert re-arms when its situation ends: the same unknown board back in the zone alerts again",
          len(al) == 2 and all(e[2].startswith("processing_1 FAULT: board ") and "unknown board" in e[2] for e in al),
          str([e[2][:70] for e in al]))


@case("CL-35 park")
def park():
    sim = Sim(operator={"take_after": 60.0})
    a = process_job(sim)
    sim.loop(until=held)                                     # arm at hover over the picked board
    sim.fake._add("white_small", -0.50, 0.45, 0.0)            # processing_1 turns busy during the carry
    sim.frames(WCFG["unknown_after"] + 0.5)
    sim.loop(max_t=sim.t + 60, until=lambda s: not held(s))
    after = [(s, c) for s, j, b, tg, c in sim.seq][1:3]
    t_home = next((x[0] for x in sim.starts if x[1] == "home"), None)
    t_place = next((x[0] for x in sim.starts if x[1] == "place"), None)
    check("CL-35 holding, target busy, arm away from home: park at home first, then the hold timer (fault zone 20 s later)",
          after == [("home", "SUCCESS"), ("place", "SUCCESS")] and t_home is not None and t_place is not None
          and t_place - t_home >= MCFG["hold_alert_after"] + MCFG["hold_fault_after"] and sim.fault_drops == 1,
          f"{codes(sim)}, home at {t_home}, place at {t_place}")


@case("CL-31")
def held_visible():
    sim = Sim()                                              # the camera sees the held board (sim default)
    sim.add(type="transfer", select_variant="white_small", to="drop_off_2", count=0)
    sim.loop()
    check("CL-31 held board seen by the camera at the tcp: no phantom board created (ids <= physical boards)",
          sim.world._next - 1 <= 5 and sim.zone("drop_off_2")["stack_count"] == 3, f"ids {sim.world._next - 1}")

# ---------- TM-36: no home between a place and a queued pick; hidden boards from the world's memory ----------
from hk_fr5_task import mask as _M
from hk_fr5_task.world_core import rect_corners as _rc
from main_sim import OCCLUDE_R as _OR, ZONES as _Z
_D1 = next(z for z in _Z if z["name"] == "drop_off_1")
_HIDDEN = (-0.10, 0.37)                                  # on the arm's line base -> drop_off_1: hidden while it hovers there


def _hidden_under_arm(x, y, l=0.10, w=0.07):
    return _M.hits([_M.capsule((0.0, 0.0), (_D1["x"], _D1["y"]), _OR)], _rc(x, y, 0.0, l, w))


def _between(sim, first, second):
    """Skills started between the first and the second index of the SUCCESS picks / places in sim.seq."""
    return [s for s, j, b, tg, c in sim.seq[first + 1:second]]


@case("TM-36 visible")
def tm36_visible():
    sim = Sim(boards=[{"variant": "red", "x": -0.45, "y": 0.22, "yaw": 0.2}, {"variant": "red", "x": -0.62, "y": 0.20, "yaw": 0.0}])
    a = red_transfer(sim, 2)
    sim.loop(max_t=200)
    sk = [x[0] for x in sim.seq]
    i1 = sk.index("place"); i2 = sk.index("pick", i1)
    check("TM-36 next board visible: place (no home) -> pick directly; the last place goes home",
          "home" not in _between(sim, i1, i2) and sim.skip_homes == 1 and jobs(sim)[a]["state"] == "DONE"
          and cnt(jobs(sim)[a], "delivered") == 2, f"{sk}, skip_homes {sim.skip_homes}")
    check("TM-36 the deferred camera check confirms both places (no 'not confirmed' warning)",
          not events(sim, "not confirmed by the camera") and not sim.core.unverified)


@case("TM-36 hidden")
def tm36_hidden():
    assert _hidden_under_arm(*_HIDDEN), "layout: the second board must be under the arm over drop_off_1"
    sim = Sim(boards=[{"variant": "red", "x": -0.45, "y": 0.22, "yaw": 0.2},
                      {"variant": "red", "x": _HIDDEN[0], "y": _HIDDEN[1], "yaw": 0.0}])
    a = red_transfer(sim, 2)
    sim.loop(max_t=200, until=lambda s: s.skip_homes >= 1)          # first board placed, arm over drop_off_1
    hid = [b for b in sim.wd()["boards"] if b["location"] == "input"]
    hidden_now = len(hid) == 1 and hid[0]["age"] > MCFG["fresh"] - 1.0
    sim.loop(max_t=200)
    sk = [x[0] for x in sim.seq]
    i1 = sk.index("place"); i2 = sk.index("pick", i1)
    check("TM-36 next board hidden by the arm: it stays in the world and is picked directly (no home first)",
          len(hid) == 1 and "home" not in _between(sim, i1, i2) and cnt(jobs(sim)[a], "delivered") == 2,
          f"hidden board {hid and (hid[0]['id'], round(hid[0]['age'], 1))}, unseen now {hidden_now}, {sk}")


@case("TM-36 priority")
def tm36_priority():
    assert _hidden_under_arm(*_HIDDEN), "layout"
    sim = Sim(boards=[{"variant": "red", "x": -0.45, "y": 0.22, "yaw": 0.2},
                      {"variant": "white_small", "x": _HIDDEN[0], "y": _HIDDEN[1], "yaw": 0.0},
                      {"variant": "white_large", "x": -0.45, "y": -0.25, "yaw": 0.0}])
    hi = sim.add(type="transfer", select_variant="red", to="drop_off_1", count=1, priority=10)[2]
    me = sim.add(type="transfer", select_variant="white_small", to="drop_off_2", count=1, priority=5)[2]
    lo = sim.add(type="transfer", select_variant="white_large", to="drop_off_2", count=1, priority=1)[2]
    sim.loop(max_t=300)
    picks = [j for s, j, b, tg, c in sim.seq if s == "pick" and c == "SUCCESS"]
    sk = [x[0] for x in sim.seq]
    i1 = sk.index("place")
    check("TM-36 high / medium / low: after the high place the hidden MEDIUM board goes before the visible LOW one, "
          "directly (no home)", picks == [hi, me, lo] and sk[i1 + 1] == "pick", f"picks {picks}, skills {sk}")


@case("TM-36 gone")
def tm36_gone():
    sim = Sim(boards=[{"variant": "red", "x": -0.45, "y": 0.22, "yaw": 0.2},
                      {"variant": "red", "x": _HIDDEN[0], "y": _HIDDEN[1], "yaw": 0.0}])
    a = red_transfer(sim, 2)
    sim.loop(max_t=200, until=lambda s: s.skip_homes >= 1)
    sim.fake.boards = [b for b in sim.fake.boards if abs(b["x"] - _HIDDEN[0]) > 0.03 or abs(b["y"] - _HIDDEN[1]) > 0.03]
    sim.loop(max_t=200)                                               # removed by hand while hidden by the arm
    J = jobs(sim)[a]
    check("TM-36 a remembered board removed while hidden: the pick finds nothing (GRIP_FAILED), it is skipped, the "
          "job ends partial (no crash, no loop)", fails(sim, "GRIP_FAILED") >= 1 and cnt(J, "skipped") == 1
          and cnt(J, "delivered") == 1 and J["state"] == "DONE", f"{codes(sim)}, {J['state']} {J['message']}")


@case("TM-36 deferred verify")
def tm36_unconfirmed():
    sim = Sim(boards=[{"variant": "red", "x": -0.45, "y": 0.22, "yaw": 0.2}, {"variant": "red", "x": -0.62, "y": 0.20, "yaw": 0.0}])
    a = red_transfer(sim, 2)
    sim.loop(max_t=200, until=lambda s: s.skip_homes >= 1)            # first board placed, no home, not confirmed yet
    sim.fake.boards = [b for b in sim.fake.boards if abs(b["x"] - _D1["x"]) > 0.03 or abs(b["y"] - _D1["y"]) > 0.03]
    t0 = sim.t                                                        # taken away before the camera saw it
    sim.loop(max_t=t0 + MCFG["deferred_verify"] + 30, until=lambda s: s.t > t0 + MCFG["deferred_verify"] + 3)
    check("TM-36 deferred camera check: a skip_home place never seen in its zone -> one warning after "
          "deferred_verify, the job goes on", len(events(sim, "not confirmed by the camera within")) == 1
          and cnt(jobs(sim)[a], "delivered") == 2, str([e[2] for e in events(sim, "not confirmed")]))



@case("TM-39 continuous")
def tm39_continuous():
    sim = Sim(boards=[{"variant": "white_small", "x": -0.30, "y": -0.30, "yaw": -0.1, "count": 3}])   # no red at first
    ok_a, _, a = sim.add(type="process", select_variant="red", zone="processing_1", to="drop_off_1", count=-1, priority=10)
    _, _, b = sim.add(type="transfer", select_variant="white_small", to="drop_off_2", count=0, priority=1)
    sim.loop(max_t=400, until=lambda s: jobs(s)[b]["state"] == "DONE")
    t0 = sim.t
    sim.loop(max_t=t0 + MCFG["partial_after"] + MCFG["end_wait"] + 20)            # idle at home, no red for > 50 s
    Ja = jobs(sim)[a]
    check("TM-39 count -1 (continuous) accepted; no matching board, robot idle past end_wait + partial_after: the job "
          "stays open (QUEUED / WAITING; the lower one ran meanwhile)", ok_a and Ja["state"] in ("QUEUED", "WAITING") and "continuous" in Ja["message"]
          and cnt(jobs(sim)[b], "delivered") == 3, f"{Ja['state']} {Ja['message']}, {short(sim.seq)}")
    for k, (x, y, yaw) in enumerate(((-0.45, 0.22, 0.2), (-0.62, 0.20, 0.0))):
        sim.fake._add("red", x, y, yaw)                                          # the operator adds a red board
        sim.loop(max_t=sim.t + 200, until=lambda s, k=k: cnt(jobs(s)[a], "delivered") == k + 1 and not s.core.current)
        Ja = jobs(sim)[a]
        check(f"TM-39 red board {k + 1} added later: processed -> drop_off_1, delivered {k + 1}, job still open",
              cnt(Ja, "delivered") == k + 1 and Ja["state"] in ("WAITING", "RUNNING"), f"{Ja['state']} {Ja['message']}")
    sim.core.control("cancel_job", a, 0, sim.t)
    sim.loop(max_t=sim.t + 20, until=lambda s: jobs(s)[a]["state"] == "CANCELLED")
    check("TM-39 cancel ends the continuous job", jobs(sim)[a]["state"] == "CANCELLED", jobs(sim)[a]["message"])
    w = sim.wd()
    bad1 = sim.core.add_job({"type": "transfer", "select_variant": "red", "to": "drop_off_1", "count": -2}, w, sim.t)
    bid = next(x["id"] for x in w["boards"])
    bad2 = sim.core.add_job({"type": "transfer", "select_board": bid, "to": "drop_off_2", "count": -1}, w, sim.t)
    check("TM-39 count -2 and select_board with count -1 rejected", not bad1[0] and not bad2[0], f"{bad1[1]} / {bad2[1]}")

check(*invariant_summary())
print(f"{ok} passed, {bad} failed")
sys.exit(1 if bad else 0)
