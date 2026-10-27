"""Main controller core (T5, control-loop.md sections 5-7, CL-21..CL-32). No ROS: the node passes the world (dict like
World.msg + 'stamp'), the time and the current tcp; this core decides the next skill and digests skill results.

    core = MainCore(cfg)
    ok, msg, job_id = core.add_job(job_dict, world, t)
    action = core.next_action(world, t, tcp_xy)      # None = nothing to do now
    core.started(action)                              # the node runs the skill ...
    core.on_result(action, code_name, message, t)    # ... and reports its result
    for ev in core.take_world_events(): publish on /task/events   (e.g. 'dropped' after NOT_HOLDING)
    core.control(command, job_id, priority, t)       # dashboard / LLM commands
    core.status(t)                                    # controller + jobs + events

Reactive (CL-21): every tick each job says its next action from the world as it is now. Priority (CL-22): higher
`priority` first, ties in queue order, switching only between skills. A held board is always placed by the job that
picked it. Board choice: nearest to the current tcp (CL-26). Counters per job (S1): attempts, delivered, to_fault,
lost, skipped; `count` = target number of delivered boards.

Tick order (each step may end the tick with "nothing now"):
  skill running (R7) -> world older than the last result (CL-23) -> world not yet showing the last result (U) ->
  ALREADY_HOLDING with nothing known held: HALT (V) -> HALTED -> vision stale: hold (H) -> zone alerts (CL-29) ->
  retry wait (P / R5) -> recovery home / wait_clear (E3: before the held board; while paused only the operator's
  home and wait_clear, A6) -> held board: place at its target when it can take the board (CL-28), else park at home
  (CL-35) and wait with the hold timer (alert, then fault zone); release_safe after 'stop' or without an owner ->
  cancelled jobs end (A8) ->
  paused -> active jobs by priority.
Picks (CL-28): only when the place target can take the board (processing zone seen + EMPTY, drop-off seen + not
FULL, fault zone always); FULL pauses the job, an unseen zone or a busy processing zone is waited for. A job without
a candidate moves home first (CL-30), then its end timer runs. A board left on the cup by a failed pick is taken
over by its job (A2).
Held board that cannot be put down (D5): the job's target is retried, then the job pauses and the board goes to the
fault zone; the fault zone is retried max_place_fail times, then HALT. 'stop' sends a held board to the fault zone on
resume; a pick that started after the stop proves nothing was held, which clears that (D3).
"""
import math

ACTIVE = ("QUEUED", "RUNNING", "WAITING")
FINAL_ITEMS = ("delivered", "fault", "lost", "skipped")
IN_HAND = ("to_zone", "to_out", "to_target", "to_fault")
LOOKUP_CODES = ("BOARD_NOT_FOUND", "BOARD_MOVED", "VISION_STALE")
MOTION_CODES = ("PLAN_FAILED", "MOTION_FAILED", "INTERNAL_ERROR", "SCENE_ERROR", "GRIPPER_ERROR")
COUNTER = {"delivered": "delivered", "fault": "to_fault", "lost": "lost", "skipped": "skipped"}


class MainCore:
    def __init__(self, cfg):
        self.c = cfg
        self.jobs = []                 # dicts (Job.msg fields + internal keys starting with _)
        self._seq = 0
        self.claims = {}               # board id -> job id
        self.problem = set()           # boards skipped for good
        self.recovery = []             # home / wait_clear, run before anything else that moves
        self.paused = False            # whole controller (pause / stop)
        self.stop_fault = False        # 'stop': a held board goes to the fault zone on resume (CL-25)
        self.halted = ""               # reason; only 'resume' clears it
        self.streak = 0                # motion / system failures in a row
        self.retries = {}              # (board, skill) -> count
        self.place_fail = {}           # board -> failed place attempts (D5)
        self.events = []               # (t, level, text)
        self.current = None            # action being executed
        self.world_after = -1.0        # CL-23: decide only on a world stamped after the last skill result
        self.expect = None             # U: {"held": board or "", "until": t} - world must show the last result
        self.hold_until = -1.0         # P: wait before retrying a place
        self.world_events = []         # events for the node to publish on /task/events (D7)
        self._stop_action = None       # D3: the action running when 'stop' came (None = none)
        self._already_holding = False  # V: pick refused with ALREADY_HOLDING
        self._vision_down = False      # H: one alert per camera outage (R3)
        self._alerted = set()          # one-time alerts (hold timer, clear timeout, ...)
        self._zone_sit = {}            # CL-29: zone -> current alert situation (re-armed when it ends)
        self._hold = {}                # CL-28: held board -> since when its target cannot take it
        self._wait_since = {}          # CL-28: (job, target) -> since when the target is not seen (no board held)
        self._last_pick = None         # A2: the last pick action started (its board may be on the cup after a failure)
        self._homed = False            # CL-30: the last move ended at home (home / place / release_safe succeeded)
        self.problem_why = {}          # board -> why it was skipped (CL-29 alerts)
        self.decide = lambda point, ctx, default: default    # LLM decision hook (T8)

    # ---------- helpers ----------
    def _log(self, t, level, text):
        self.events = (self.events + [(t, level, text)])[-300:]

    def _alert_once(self, key, t, level, text):
        if key not in self._alerted:
            self._alerted.add(key)
            self._log(t, level, text)

    def _halt(self, t, reason):
        self.halted = reason
        self._log(t, "error", f"HALTED: {reason}")

    def job(self, job_id):
        return next((j for j in self.jobs if j["id"] == job_id), None)

    @staticmethod
    def _zone(world, name):
        return next((z for z in world["zones"] if z["name"] == name), None)

    @staticmethod
    def _board(world, bid):
        return next((b for b in world["boards"] if b["id"] == bid), None)

    def _zone_usable(self, world, name):
        z = self._zone(world, name)
        return z is not None and (z.get("fixed", False) or z.get("age", 0.0) <= self.c["fresh"])

    def _active(self):
        jobs = [j for j in self.jobs if j["state"] in ACTIVE and not j["_paused"]]
        return sorted(jobs, key=lambda j: (-j["priority"], j["_seq"]))

    def _target_ok(self, world, name):
        """CL-28: (True, '') if a board can be placed in zone `name` now, else (False, reason). Fault zone: always."""
        if name == self.c["fault_zone"]:
            return True, ""
        z = self._zone(world, name)
        if z is None:
            return False, "not seen"
        if not (z.get("fixed", False) or z.get("age", 0.0) <= self.c["fresh"]):
            return False, f"not seen for {z.get('age', 0.0):.0f} s"
        if z["kind"] == "processing" and z["state"] != "EMPTY":
            return False, z["state"]
        if z["kind"] != "processing" and z["state"] == "FULL":
            return False, "FULL"
        return True, ""

    @staticmethod
    def _needs_person(why):
        """Target problems a person must fix (job pauses): FULL, zone not seen. A busy processing zone frees itself."""
        return why == "FULL" or why.startswith("not seen")

    @staticmethod
    def _todo(why):
        if why == "FULL":
            return "empty it and reset the zone"
        if why.startswith("not seen"):
            return "check that the camera sees the zone"
        return f"wait or free the zone (it is {why})"

    def _who(self, b):
        """Owner text of a board for alerts: 'j1', 'j1 paused', 'j1 cancelled', 'no job'."""
        j = next((j for j in reversed(self.jobs) if b in j["_items"]), None)
        if j is None:
            return "no job"
        if j["_paused"]:
            return f"{j['id']} paused"
        if j["state"] in ACTIVE:
            return j["id"]
        return f"{j['id']} {j['state'].lower()}"

    def _bdesc(self, world, b):
        bd = self._board(world, b)
        return f"board {b} ({bd['variant'] if bd else 'unknown'}, {self._who(b)})"

    def _at_home(self, tcp):
        hx, hy = self.c["home_xy"]
        return self._homed or math.hypot(tcp[0] - hx, tcp[1] - hy) <= self.c["home_tol"]

    @staticmethod
    def _in_hand(j):
        return any(st in IN_HAND for st in j["_items"].values())

    def take_world_events(self):
        ev, self.world_events = self.world_events, []
        return ev

    # ---------- jobs ----------
    def add_job(self, job, world, t=0.0):
        """Validate + queue. Returns (ok, message, id)."""
        typ = job.get("type", "")
        zones = {z["name"]: z["kind"] for z in world.get("zones", [])}
        def zone_ok(name, kinds):
            return name in zones and zones[name] in kinds
        v = job.get("select_variant", "")
        if typ not in ("process", "transfer", "clear_zone", "home"):
            return False, f"unknown job type '{typ}' (process | transfer | clear_zone | home)", ""
        if v and v not in self.c["variants"]:
            return False, f"unknown variant '{v}' (known: {', '.join(self.c['variants'])})", ""
        if job.get("count", 0) < 0 or job.get("max_attempts", 0) < 0:
            return False, "count and max_attempts must be >= 0 (0 = all / no limit)", ""
        if job.get("count", 0) and job.get("max_attempts", 0) and job["max_attempts"] < job["count"]:
            return False, "max_attempts must be >= count", ""
        if job.get("select_board") and self._board(world, job["select_board"]) is None:
            return False, f"board '{job['select_board']}' not in the world", ""
        if job.get("select_board") and job.get("count", 0) > 1:                              # A12
            return False, "select_board names one board: count must be 0 or 1", ""
        if typ == "process":
            if not zone_ok(job.get("zone", ""), ("processing",)):
                return False, f"process needs a known processing zone, got '{job.get('zone', '')}'", ""
            if job.get("to") and not zone_ok(job["to"], ("drop_off", "fault")):
                return False, f"'to' must be a known drop-off / fault zone, got '{job['to']}'", ""
        if typ == "transfer" and not zone_ok(job.get("to", ""), ("drop_off", "fault", "processing")):
            return False, f"transfer needs a known target zone 'to', got '{job.get('to', '')}'", ""
        if typ == "clear_zone":
            if job.get("from_zone", "") not in zones:
                return False, f"clear_zone needs a known 'from_zone', got '{job.get('from_zone', '')}'", ""
            if not zone_ok(job.get("to", ""), ("drop_off", "fault")):
                return False, f"clear_zone needs a known drop-off / fault zone 'to', got '{job.get('to', '')}'", ""
        if typ in ("transfer", "clear_zone") and job.get("from_zone") and job.get("from_zone") == job.get("to"):
            return False, "from_zone and to are the same zone", ""
        self._seq += 1
        j = {"id": f"j{self._seq}", "type": typ, "select_variant": v, "select_board": job.get("select_board", ""),
             "from_zone": job.get("from_zone", ""), "zone": job.get("zone", ""), "to": job.get("to", ""),
             "count": int(job.get("count", 0)), "max_attempts": int(job.get("max_attempts", 0)),
             "priority": int(job.get("priority", 0)), "source": job.get("source", ""), "state": "QUEUED",
             "attempts": 0, "delivered": 0, "to_fault": 0, "lost": 0, "skipped": 0, "message": "",
             "_seq": self._seq, "_items": {}, "_paused": False, "_cancel": False, "_none_since": None,
}
        self.jobs.append(j)
        self._log(t, "info", f"job {j['id']} added: {self.describe(j)}")
        return True, "queued", j["id"]

    @staticmethod
    def describe(j):
        sel = j["select_board"] or (j["select_variant"] or "any") + (f" from {j['from_zone']}" if j["from_zone"] else "")
        n = j["count"] or "all"
        lim = f", max {j['max_attempts']} attempts" if j["max_attempts"] else ""
        return {"process": f"process {n} x {sel} in {j['zone']}" + (f" -> {j['to']}" if j["to"] else ""),
                "transfer": f"transfer {n} x {sel} -> {j['to']}",
                "clear_zone": f"clear {j['from_zone']} -> {j['to']}", "home": "home"}[j["type"]] + f"{lim} (prio {j['priority']})"

    def _outcome(self, j, b, kind, t):
        """Final outcome of one board for its job (counted once) + claim released."""
        self.claims.pop(b, None)
        if j is None:
            return
        if j["_items"].get(b) in FINAL_ITEMS:
            return
        j["_items"][b] = kind
        j[COUNTER[kind]] += 1
        j["attempts"] = len(j["_items"])

    def _candidates(self, j, world):
        """Boards this job could pick now (not held, not claimed by another job, not problem, recently seen)."""
        out = []
        for b in world["boards"]:
            if b["location"] == "held" or b["id"] in self.problem or b["age"] > self.c["fresh"]:
                continue
            if self.claims.get(b["id"], j["id"]) != j["id"] or b["id"] in j["_items"]:
                continue
            if j["select_board"] and b["id"] != j["select_board"]:
                continue
            if j["select_variant"] and b["variant"] != j["select_variant"]:
                continue
            if j["type"] == "clear_zone" or j["from_zone"]:
                if b["location"] != "zone" or b["zone"] != j["from_zone"]:
                    continue
            elif b["location"] != "input":
                continue
            out.append(b)
        return out

    @staticmethod
    def _nearest(boards, tcp):
        return min(boards, key=lambda b: (math.hypot(b["x"] - tcp[0], b["y"] - tcp[1]), b["id"]))

    @staticmethod
    def _in_flight(j):
        return [b for b, st in j["_items"].items() if st not in FINAL_ITEMS and b != "home"]

    def _may_start(self, j):
        """A new board may be started (S1): target not reached incl. boards in flight, attempts left."""
        if j["count"] and j["delivered"] + len(self._in_flight(j)) >= j["count"]:
            return False
        if j["max_attempts"] and j["attempts"] >= j["max_attempts"]:
            return False
        return True

    def _job_action(self, j, world, t, tcp):
        """Next action of one job, or None (nothing now). May finish the job."""
        if j["type"] == "home":
            if j["_items"].get("home") is None:
                j["_items"]["home"] = "planned"
                return {"skill": "home", "job": j["id"]}
            return None
        if j["type"] == "process":
            z = self._zone(world, j["zone"])
            for b in [b for b, st in j["_items"].items() if st == "in_zone"]:     # the processed board first
                if z is not None and z["board_id"] == b and z["state"] == "DONE":
                    if not j["to"]:                              # no output target: delivered when processed
                        self._outcome(j, b, "delivered", t)
                        self._log(t, "info", f"{j['id']}: {b} processed (stays in {j['zone']})")
                        continue
                    ok, why = self._target_ok(world, j["to"])   # CL-28: only pick it out if 'to' can take it
                    if not ok:
                        return self._blocked(j, t, j["to"], why, world)
                    return {"skill": "pick", "job": j["id"], "board": b, "stage": "to_out"}
                if z is not None and z["board_id"] == b and z["state"] == "FAULT":
                    return {"skill": "pick", "job": j["id"], "board": b, "stage": "to_fault"}
                if z is not None and z["board_id"] != b:                         # R6: zone reset / cleared
                    j["message"] = f"waiting: {b} is no longer in {j['zone']} - cancel the job or put it back"
                    self._alert_once(("gone", j["zone"], b), t, "error",
                                     f"{j['zone']} {z['state']}: {self._bdesc(world, b)} placed by {j['id']} is no longer "
                                     f"tracked there (zone reset or cleared?) - cancel {j['id']} or put the board back")
                    continue
                if z is not None and "overdue" in z.get("message", ""):          # B3 (alert: _zone_alerts)
                    j["message"] = f"waiting: {j['zone']} {z['message']} - is {b} back inside the zone?"
            if self._in_flight(j) or not self._may_start(j):
                return self._maybe_done(j, t)
            ok, why = self._target_ok(world, j["zone"])         # CL-28
            if not ok:
                return self._blocked(j, t, j["zone"], why, world)
            return self._start(j, world, t, tcp, "to_zone")
        if self._in_flight(j) or not self._may_start(j):      # transfer / clear_zone
            return self._maybe_done(j, t)
        ok, why = self._target_ok(world, j["to"])             # CL-28
        if not ok:
            return self._blocked(j, t, j["to"], why, world)
        return self._start(j, world, t, tcp, "to_target")

    def _blocked(self, j, t, target, why, world):
        """CL-28: the place target cannot take a board, so nothing is picked. FULL -> job PAUSED (a person empties it);
        not seen -> wait, alert after hold_alert_after; busy processing zone -> wait (it frees itself)."""
        if why == "FULL":
            j["_paused"], j["state"] = True, "PAUSED"
            j["message"] = f"paused: {target} is FULL - empty it and reset the zone, then resume the job"
            self._log(t, "error", f"job {j['id']} paused: {target} FULL")
            return None
        j["message"] = f"waiting: {target} is {why}"
        if why.startswith("not seen"):
            since = self._wait_since.setdefault((j["id"], target), t)
            if t - since >= self.c["hold_alert_after"]:
                self._alert_once(("unseen", j["id"], target, since), t, "error",
                                 f"{target} not seen for {t - since:.0f} s: {j['id']} cannot start its next board - "
                                 "check that the camera sees the zone")
        else:
            self._wait_since.pop((j["id"], target), None)
        return None

    def _start(self, j, world, t, tcp, stage):
        for key in [k for k in self._wait_since if k[0] == j["id"]]:
            self._wait_since.pop(key)
        cands = self._candidates(j, world)
        if not cands:
            return self._no_candidate(j, t, tcp)
        j["_none_since"] = None
        b = self._nearest(cands, tcp)
        return {"skill": "pick", "job": j["id"], "board": b["id"], "stage": stage}

    def _no_candidate(self, j, t, tcp):
        """CL-24: count 0 (all) ends after end_wait s without a candidate; count n gives up after partial_after s.
        CL-30: only counted with the arm at home (its mask hides boards near it): else move home first."""
        if not self._at_home(tcp):
            j["_none_since"] = None
            self._log(t, "info", f"{j['id']}: no matching board seen - moving home for a clear camera view (CL-30)")
            return {"skill": "home", "job": "", "board": ""}
        if j["_none_since"] is None:
            j["_none_since"] = t
        wait = self.c["end_wait"] if not j["count"] else self.c["partial_after"]
        if t - j["_none_since"] >= wait:
            self._finish(j, t, "DONE", self._summary(j) + (" - all matching boards handled" if not j["count"] else
                                                           " - no more matching boards (partial)"))
            return None
        j["message"] = "no matching board right now"
        return None

    def _maybe_done(self, j, t):
        if self._in_flight(j):
            j["state"] = "WAITING"
            return None
        if j["count"] and j["delivered"] >= j["count"]:
            self._finish(j, t, "DONE", self._summary(j))
        elif j["max_attempts"] and j["attempts"] >= j["max_attempts"]:
            self._finish(j, t, "DONE", self._summary(j) + " - max attempts reached (partial)")
        else:
            j["state"] = "WAITING"
        return None

    @staticmethod
    def _summary(j):
        return (f"{j['delivered']} delivered" + (f" of {j['count']}" if j["count"] else "") +
                f", {j['to_fault']} to fault, {j['lost']} lost, {j['skipped']} skipped ({j['attempts']} attempts)")

    def _finish(self, j, t, state, message):
        j["state"], j["message"] = state, message
        for b in [b for b, owner in self.claims.items() if owner == j["id"]]:
            self.claims.pop(b)
        self._log(t, "info" if state == "DONE" else "warn", f"job {j['id']} {state}: {message}")

    def _target(self, j, stage):
        return {"to_zone": j["zone"], "to_out": j["to"], "to_target": j["to"], "to_fault": self.c["fault_zone"]}[stage]

    # ---------- scheduling ----------
    def next_action(self, world, t, tcp):
        """The next skill to run, or None. world: World-like dict (with 'stamp'); tcp: (x, y) of the tcp now."""
        if self.current is not None:                                         # one skill at a time (R7)
            return None
        if world.get("stamp", float("inf")) <= self.world_after:            # CL-23
            return None
        held = world.get("held_board_id", "")
        if self.expect is not None:                                          # U: world shows the last result?
            if held == self.expect["held"]:
                self.expect = None
            elif t < self.expect["until"]:
                return None
            else:
                self._log(t, "warn", f"world does not show the expected state (held '{self.expect['held']}', "
                                     f"world '{held}') - going on with the world as it is")
                self.expect = None
        if self._already_holding:                                            # V
            self._already_holding = False
            if not held:
                self._halt(t, "the gripper holds something the world does not know (ALREADY_HOLDING) - "
                              "remove it from the cup, then resume")
        if self.halted:
            return None
        if not world.get("vision_ok", True):                                  # H
            if not self._vision_down:
                self._vision_down = True
                self._log(t, "error", "no fresh Vision 1 data - holding (nothing moves until the camera is back)")
            return None
        if self._vision_down:
            self._vision_down = False
            self._log(t, "info", "Vision 1 data back - continuing")
        self._zone_alerts(world, t)
        if t < self.hold_until:                                              # P: retry wait
            return None
        if self.recovery:                                                    # E3: before the held board
            r = next((r for r in self.recovery                               # A6: while paused only the operator's
                      if not self.paused or r.get("cmd") or r["skill"] == "wait_clear"), None)   # home / wait_clear
            if r is not None:
                return dict(r)
        if held:
            return self._held_action(world, held, t, tcp)
        for j in [j for j in self.jobs if j["_cancel"] and j["state"] in ACTIVE + ("PAUSED",)]:   # A8: also while paused
            self._finish(j, t, "CANCELLED", "cancelled - " + self._summary(j))
        if self.paused:
            return None
        for j in self._active():
            a = self._job_action(j, world, t, tcp)
            if a is not None:
                j["state"] = "RUNNING"
                if a["skill"] == "pick":
                    for ob in [ob for ob, o in self.claims.items()           # a job keeps no stale claim on a board
                               if o == j["id"] and ob != a["board"] and ob not in j["_items"]]:   # it moved on from
                        self.claims.pop(ob)
                    self.claims[a["board"]] = j["id"]
                return a
            if j["state"] == "RUNNING":
                j["state"] = "WAITING"
        return None

    def _held_action(self, world, held, t, tcp):
        """A board on the cup: place it at its job's target, or put it into the fault zone (release_safe)."""
        owner = self.job(self.claims.get(held, ""))
        stage = owner["_items"].get(held) if owner else None
        lp = self._last_pick
        if stage not in IN_HAND and lp is not None and lp["board"] == held:  # A2: the pick reported a failure, but
            o = self.job(lp.get("job", ""))                                  # the board is on the cup: take it over
            if o is not None and o["state"] in ACTIVE + ("PAUSED",) and o["_items"].get(held) is None:
                owner, stage = o, lp["stage"]
                o["_items"][held] = stage
                o["attempts"] = len(o["_items"])
                self.claims[held] = o["id"]
                self._log(t, "warn", f"{o['id']}: {held} is on the cup although its pick reported a failure - "
                                     "placing it as picked")
        if self.paused and (self.stop_fault or not self.c["place_when_paused"]):
            return None                                                      # 'stop': hold until resume (CL-25)
        if self.place_fail.get(held, 0) >= self.c["max_place_fail"]:         # D5
            self._halt(t, f"cannot put down the held board {held} after {self.place_fail[held]} tries - "
                          "take it off the cup / check the zones, then resume")
            return None
        if self.stop_fault or owner is None or stage not in IN_HAND:
            return {"skill": "release_safe", "job": owner["id"] if owner else "", "board": held,
                    "target": self.c["fault_zone"]}
        target = self._target(owner, stage)
        ok, why = self._target_ok(world, target)                             # CL-28
        if ok:
            self._hold.pop(held, None)
            return {"skill": "place", "job": owner["id"], "board": held, "target": target}
        if not self._at_home(tcp):                                           # CL-35: park (= home) before waiting,
            self._hold.pop(held, None)                                       # so the camera sees the zones; the
            owner["message"] = f"holding {held}: {target} is {why} - parking at home to wait"   # timer starts there
            return {"skill": "home", "job": "", "board": ""}
        return self._hold_wait(owner, held, target, why, world, t)

    def _hold_wait(self, j, b, target, why, world, t):
        """CL-28 hold timer: the held board's target cannot take it. Wait; alert after hold_alert_after; after
        hold_fault_after more the board goes to the fault zone (FULL / not seen: the job pauses too)."""
        since = self._hold.setdefault(b, t)
        waited = t - since
        a1, a2 = self.c["hold_alert_after"], self.c["hold_fault_after"]
        j["message"] = f"holding {b}: waiting for {target} ({why}, {waited:.0f} s)"
        if waited >= a1:
            self._alert_once(("hold", b, target, since), t, "error",
                             f"holding {self._bdesc(world, b)} for {target}: {why} for {waited:.0f} s - {self._todo(why)}; "
                             f"the board goes to the fault zone in {max(0.0, a1 + a2 - waited):.0f} s")
        if waited < a1 + a2:
            return None
        self._hold.pop(b, None)
        if self._needs_person(why):
            self._pause_job(j, b, t, f"{target} {why} while holding {b} for {waited:.0f} s")
        else:
            j["_items"][b] = "to_fault"
            self.place_fail.pop(b, None)
            self._log(t, "error", f"{j['id']}: {target} still {why} after {waited:.0f} s - {b} goes to the fault zone")
        return {"skill": "place", "job": j["id"], "board": b, "target": self.c["fault_zone"]}

    def started(self, action):
        self.current = action
        if action["skill"] == "pick":
            self._last_pick = action                                         # A2
        if action["skill"] != "wait_clear":
            self._homed = False                                              # CL-30: the arm moves

    def _zone_alerts(self, world, t):
        """CL-29: one verbose alert per processing-zone situation, again only when the situation changes.
        Format: '<zone> <state>: board <id> (<variant>, <owner>) - <what happened> - <what to do>'."""
        flight = {b: j for j in self.jobs for b, st in j["_items"].items() if st not in FINAL_ITEMS and b != "home"}
        for z in world["zones"]:
            if z["kind"] != "processing":
                continue
            s = self._zone_situation(z, world, flight)
            key = s[0] if s else None
            if key != self._zone_sit.get(z["name"]):                        # a new situation (or it ended: re-arm,
                self._zone_sit[z["name"]] = key                              # so a recurrence alerts again)
                if s:
                    self._log(t, s[1], s[2])

    def _zone_situation(self, z, world, flight):
        """(key, level, text) of the alert-worthy situation of one processing zone, or None."""
        b, st = z["board_id"], z["state"]
        if st == "EMPTY" or not b:
            return None
        j = flight.get(b)
        active = j is not None and j["state"] in ACTIVE and not j["_paused"]
        head, who = f"{z['name']} {st}: {self._bdesc(world, b)}", self._who(b)
        level, msg = "error", z.get("message", "")
        if b in self.problem:
            text = f"{head} - skipped ({self.problem_why.get(b, 'problem board')}) - remove it by hand and reset the zone"
        elif st == "LOADED":
            if z.get("state_age", 0.0) < self.c["loaded_alert_after"]:
                return None
            text = f"{head} - placed, not taken for processing (since {z.get('state_age', 0.0):.0f} s) - " \
                   f"take it for processing" + (f", or cancel {j['id']}" if j is not None and (active or j["_paused"]) else "")
        elif st == "PROCESSING":
            if "overdue" not in msg:
                return None
            text = f"{head} - {msg}, not back yet - put it back inside the zone (within its margin)"
        elif st == "DONE":
            if active:
                return None                                                  # its job picks it out
            if j is not None and j["_paused"]:
                text = f"{head} - processed, but {j['id']} is paused ({j['message']}) - fix that, then resume {j['id']}"
            else:
                text = f"{head} - processed, but no active job will take it - add a clear_zone job, or remove it " \
                       f"by hand and reset the zone"
        elif st == "FAULT":
            if active:
                level, text = "info", f"{head} - {msg} - {j['id']} moves it to the fault zone"
            else:
                text = f"{head} - {msg} - remove it and reset the zone" + \
                       (f" (or resume {j['id']}, which moves it to the fault zone)" if j is not None and j["_paused"] else "")
        else:
            return None
        return ("zone", st, b, who, b in self.problem, msg), level, text

    # ---------- results ----------
    def on_result(self, action, code, message, t):
        """code: SkillCode name ('SUCCESS', 'GRIP_FAILED', ...). t = time of the result (CL-23)."""
        self.current = None
        self.world_after = t
        skill, j, b = action["skill"], self.job(action.get("job", "")), action.get("board", "")
        keep = code == "OBSTACLE" or (code == "CANCELLED" and not action.get("cmd"))   # A7: run it again later
        if action in self.recovery and not keep:
            self.recovery.remove(action)
        until = t + self.c["world_wait"]
        if code in ("SUCCESS", "NOT_VERIFIED") and skill in ("home", "place", "release_safe"):
            self._homed = True                                               # these end at home (CL-30)
        if code == "SUCCESS":
            self.streak = 0
            self.retries.pop((b, skill), None)
            if skill in ("place", "release_safe"):
                self._hold.pop(b, None)
            if skill == "pick":
                self.expect = {"held": b, "until": until}
                if self.stop_fault and action is not self._stop_action:     # D3: started after the stop, so
                    self.stop_fault = False                                  # nothing was on the cup at 'stop'
                if j is not None:
                    j["_items"][b] = action["stage"]
                    j["attempts"] = len(j["_items"])
                    self._log(t, "info", f"{j['id']}: picked {b} ({action['stage'].replace('_', ' ')})")
            elif skill == "place":
                self.expect = {"held": "", "until": until}
                self.place_fail.pop(b, None)
                if j is not None:
                    stage = j["_items"].get(b)
                    if stage == "to_zone":
                        j["_items"][b] = "in_zone"
                    else:
                        self._outcome(j, b, "fault" if stage == "to_fault" else "delivered", t)
                    self._log(t, "info", f"{j['id']}: placed {b} in {action['target']}")
                    if j["_cancel"]:
                        self._finish(j, t, "CANCELLED", "cancelled (held board placed first) - " + self._summary(j))
                    elif j["_paused"]:
                        j["state"] = "PAUSED"
            elif skill == "release_safe":
                self.expect = {"held": "", "until": until}
                self.stop_fault = False
                self.place_fail.pop(b, None)
                self._outcome(j, b, "fault", t)
                self._log(t, "warn", f"{b}: put down in the fault zone (release_safe)")
            elif skill == "home" and j is not None and j["type"] == "home":
                j["_items"]["home"] = "done"
                self._finish(j, t, "DONE", "at home")
            return
        # ---- failures (control-loop.md section 6, T5 decisions + review fixes) ----
        self._log(t, "warn", f"{skill} {b or ''} failed: {code} - {message}")
        if skill == "home" and j is not None and j["type"] == "home":
            j["_items"]["home"] = None                                       # retried (streak / halt apply)
        key = (b, skill)
        self.retries[key] = self.retries.get(key, 0) + 1
        if skill in ("place", "release_safe") and code not in ("OBSTACLE", "CANCELLED", "TARGET_OCCUPIED"):
            self.place_fail[b] = self.place_fail.get(b, 0) + 1              # D5: bounded
        if code == "OBSTACLE":
            if not any(r["skill"] == "wait_clear" for r in self.recovery):
                self.recovery.insert(0, {"skill": "wait_clear", "job": "", "board": ""})
            return                                                           # the same action comes again after
        if code == "TIMEOUT" and skill == "wait_clear":                      # E2: keep waiting, alert once
            self.recovery.insert(0, {"skill": "wait_clear", "job": "", "board": ""})
            self._alert_once(("clear", round(t // 60)), t, "error", "work area still not clear - remove the obstacle")
            return
        if code in ("RELEASE_FAILED",):                                      # D6
            self._halt(t, f"release of {b} failed - the board may still be on the cup; check the gripper, then resume")
            return
        if code == "ALREADY_HOLDING":                                        # V (or a world lag, U)
            self._already_holding = True
            if j is None or b not in j["_items"]:
                self.claims.pop(b, None)
            return
        if code in MOTION_CODES:
            self.streak += 1
            if self.streak >= self.c["halt_after"]:
                self._halt(t, f"{self.streak} motion / system failures in a row (last: {code}) - check the robot, then resume")
                return
            home_job = skill == "home" and j is not None and j["type"] == "home"   # retries itself (R4)
            if not home_job and not any(r["skill"] == "home" for r in self.recovery):
                self.recovery.append({"skill": "home", "job": "", "board": ""})
            if skill == "pick" and self.retries[key] > self.c["motion_retries"]:
                self._skip(j, b, t, f"{code} {self.retries[key]}x")
            return
        if code == "CANCELLED":
            if skill == "pick" and j is not None and b not in j["_items"]:
                self.claims.pop(b, None)                                     # chosen again later
            return
        if code in LOOKUP_CODES and skill == "pick":
            if self.retries[key] > self.c["lookup_retries"]:
                self._skip(j, b, t, f"{code} after {self.retries[key]} tries")
            else:
                self.claims.pop(b, None)
            return
        if code == "GRIP_FAILED" and skill == "pick":
            choice = self.decide("grip_failed", {"board": b, "options": ["skip", "retry"]}, "skip")
            if choice == "skip" or self.retries[key] > self.c["motion_retries"]:
                self._skip(j, b, t, "no vacuum (GRIP_FAILED)")
            else:
                self.claims.pop(b, None)
            return
        if code in ("LOST_ON_LIFT", "LOST_IN_TRANSIT", "NOT_HOLDING"):
            if code == "NOT_HOLDING":                                        # D7: the skill did not tell the world
                self.world_events.append({"type": "dropped", "board": b})
            self._outcome(j, b, "lost", t)
            self._hold.pop(b, None)
            self.expect = {"held": "", "until": until}
            self._log(t, "error", f"{b} lost ({code}): check the table / floor")
            if not any(r["skill"] == "home" for r in self.recovery):
                self.recovery.append({"skill": "home", "job": "", "board": ""})
            return
        if code == "TARGET_OCCUPIED":
            self.hold_until = t + self.c["retry_wait"]                     # no instant retry (R5)
            if j is not None:
                j["message"] = f"waiting: {action.get('target')} busy"
            return                                                           # still holding: placed when free
        if code in ("TARGET_UNKNOWN", "STACK_FULL") and skill in ("place", "release_safe"):
            target, n = action.get("target") or self.c["fault_zone"], self.place_fail.get(b, 0)
            if target != self.c["fault_zone"]:                               # the job's target (P / S2)
                if code == "TARGET_UNKNOWN" and n <= self.c["lookup_retries"]:
                    self.hold_until = t + self.c["retry_wait"]
                    if j is not None:
                        j["message"] = f"target {target} not seen - retry {n}"
                    return
                self._pause_job(j, b, t, f"{code} at {target}")              # board -> fault zone
                return
            if code == "TARGET_UNKNOWN" and n < self.c["max_place_fail"]:   # the fault zone itself (D5)
                self.hold_until = t + self.c["retry_wait"]
                return
            self._halt(t, f"cannot put the held board {b} into the fault zone '{target}' ({code} {n}x) - "
                          "take it off the cup / check the zone, then resume")
            return
        if code == "NOT_VERIFIED" and skill == "place":
            self._log(t, "warn", f"{b}: placed but not confirmed by the camera - continuing")
            return self.on_result(action, "SUCCESS", "not verified", t)
        self._log(t, "error", f"{skill}: unhandled result {code}")

    def _pause_job(self, j, b, t, why):
        if j is not None:
            j["_paused"], j["state"] = True, "PAUSED"
            j["message"] = f"paused: {why} - fix it, then resume the job"
            if b and j["_items"].get(b) in IN_HAND and j["_items"][b] != "to_fault":
                j["_items"][b] = "to_fault"                                  # the held board goes to the fault zone
                self.place_fail.pop(b, None)                                 # fresh tries there (R1), once (D5)
        self._log(t, "error", f"job {j['id'] if j else '?'} paused: {why}")

    def _skip(self, j, b, t, why):
        self.problem.add(b)
        self.problem_why[b] = why
        self._outcome(j, b, "skipped", t)
        self._log(t, "warn", f"{b} skipped for good: {why}")

    # ---------- commands ----------
    def control(self, command, job_id="", priority=0, t=0.0):
        j = self.job(job_id) if job_id else None
        if job_id and j is None:
            return False, f"unknown job '{job_id}'"
        if command == "pause":
            if j:
                if j["state"] not in ACTIVE:
                    return False, f"job {job_id} is {j['state']}"
                j["_paused"], j["state"] = True, "PAUSED"
            else:
                self.paused = True
        elif command == "resume":
            if j:
                if j["state"] not in ("PAUSED",) + ACTIVE:
                    return False, f"job {job_id} is {j['state']}"
                j["_paused"], j["state"], j["_none_since"] = False, "QUEUED", None
            else:
                self.paused = False
                if self.halted:
                    self._log(t, "info", f"resumed after halt: {self.halted}")
                self.halted, self.streak = "", 0
                self.place_fail.clear()
        elif command == "stop":
            self.paused = True
            self.stop_fault = True                                           # a held board -> fault zone on resume
            self._stop_action = self.current
        elif command == "cancel_job":
            if not j:
                return False, "cancel_job needs a job_id"
            if j["state"] not in ACTIVE + ("PAUSED",):
                return False, f"job {job_id} is already {j['state']}"
            self._cancel(j, t)
        elif command == "set_priority":
            if not j:
                return False, "set_priority needs a job_id"
            j["priority"] = int(priority)
        elif command == "clear_queue":
            for x in self.jobs:
                if x["state"] in ACTIVE + ("PAUSED",):
                    self._cancel(x, t)
        elif command == "home":
            if not (self.paused or self.halted or all(x["state"] not in ACTIVE for x in self.jobs)):
                return False, "home only while paused / halted / idle"
            if not any(r["skill"] == "home" for r in self.recovery):
                self.recovery.append({"skill": "home", "job": "", "board": "", "cmd": True})   # A6: runs while paused
            if self.halted:
                return True, "home queued (runs after resume)"
        else:
            return False, f"unknown command '{command}'"
        self._log(t, "info", f"command {command}" + (f" {job_id}" if job_id else "") +
                  (f" priority {priority}" if command == "set_priority" else ""))
        return True, "ok"

    def _cancel(self, j, t):
        """A8: cancel now, unless the job's board is on the cup or its skill is running (then CL-25: placed first)."""
        j["_cancel"], j["_paused"] = True, False
        running = self.current is not None and self.current.get("job") == j["id"]
        if not self._in_hand(j) and not running:
            self._finish(j, t, "CANCELLED", "cancelled - " + self._summary(j))

    # ---------- status ----------
    def state(self):
        if self.halted:
            return "HALTED"
        if self.current is not None:
            return "RECOVERING" if not self.current.get("job") else "RUNNING"
        if self.paused:
            return "PAUSED"
        if any(j["state"] in ACTIVE for j in self.jobs):
            return "WAITING"
        return "IDLE"

    def status(self, t):
        jobs = [{k: v for k, v in j.items() if not k.startswith("_")} for j in self.jobs]
        return {"state": self.state(), "halted": self.halted, "paused": self.paused, "current": self.current,
                "jobs": jobs, "problem_boards": sorted(self.problem), "recovery": [r["skill"] for r in self.recovery],
                "events": [{"t": round(e[0], 2), "level": e[1], "text": e[2]} for e in self.events[-50:]]}
