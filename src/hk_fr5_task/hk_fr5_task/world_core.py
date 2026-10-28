"""World model core (control-loop.md sections 2-3, 6; decision-log TM-8). No ROS: time is passed in, so the logic is
tested with simulated time (tests/world_core_check.py).

Input per Vision 1 frame: detections in base_link (hk_fr5_task.vision1 parses the /vision/boards JSON, V3 TM-22):
{"cls": "board/<variant>" | "zone/processing" | "zone/drop_off" | "obstacle/<label>", "id": "" / zone name,
"score": 0..1, "corners": [[x, y] x4], zones also "z" (sheet top -> base_z, V2)}. Zone kind "fault" only from the
config (fixed_zones).
Input from the skills: events {"type": "picked" | "placed" | "dropped" | "reset_zone", ...}.
Output: snapshot() -> plain dict with the World message layout.
"""
import math

import numpy as np

from hk_fr5_task import mask as M

NAN = float("nan")


def wrap_half(a):
    """Angle -> (-pi/2, pi/2] (a rectangle's long axis has no direction)."""
    a = math.fmod(a, math.pi)
    if a > math.pi / 2:
        a -= math.pi
    elif a <= -math.pi / 2:
        a += math.pi
    return a


def rect_from_corners(corners):
    """4 corners in order around the rectangle -> (x, y, yaw of the long side, length, width)."""
    c = np.asarray(corners, float).reshape(4, 2)
    x, y = c.mean(axis=0)
    e = [c[(i + 1) % 4] - c[i] for i in range(4)]
    a, b = (np.linalg.norm(e[0]) + np.linalg.norm(e[2])) / 2, (np.linalg.norm(e[1]) + np.linalg.norm(e[3])) / 2
    long_edge = e[0] - e[2] if a >= b else e[1] - e[3]      # average of the two opposite edges (same direction)
    yaw = wrap_half(math.atan2(long_edge[1], long_edge[0]))
    return float(x), float(y), yaw, float(max(a, b)), float(min(a, b))


def rect_corners(x, y, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(length / 2, width / 2), (-length / 2, width / 2), (-length / 2, -width / 2), (length / 2, -width / 2)]
    return [[x + c * u - s * v, y + s * u + c * v] for u, v in pts]


def in_rect(px, py, r, margin=0.0):
    """Point inside the rotated rect r (dict x, y, yaw, length, width) grown by margin."""
    c, s = math.cos(r["yaw"]), math.sin(r["yaw"])
    dx, dy = px - r["x"], py - r["y"]
    u, v = c * dx + s * dy, -s * dx + c * dy
    return abs(u) <= r["length"] / 2 + margin and abs(v) <= r["width"] / 2 + margin


class World:
    def __init__(self, cfg):
        self.c = cfg
        self.boards = {}            # id -> dict
        self.zones = {}             # name -> dict
        self.obstacles = []
        self.held = ""
        self.last_frame_t = None
        self._next = 1
        self.unknown_noted = {}      # TM-33: zone -> last note time of an ignored UNKWN detection
        self.log = []               # (t, text) state changes, for the dashboard / tests

    # ---------- helpers ----------
    def _note(self, t, text):
        self.log = (self.log + [(t, text)])[-200:]
        self.note_count = getattr(self, "note_count", 0) + 1         # TM-26: the node logs each note once

    def _variant(self, name):
        return self.c["variants"].get(name)

    def _thickness(self, variant):
        v = self._variant(variant)
        return v["thickness"] if v else NAN

    def _stack_thickness(self, variant):
        """Thickness a board adds to / removes from a stack: its variant's, else default_thickness (never NaN)."""
        th = self._thickness(variant)
        return th if th == th else self.c["default_thickness"]

    def _size_ok(self, variant, length, width):
        v = self._variant(variant)
        if not v:
            return False
        tol = self.c["size_tol"]
        return abs(length - v["length"]) <= tol and abs(width - v["width"]) <= tol

    def _zone(self, name, kind, t, fixed=False):
        z = self.zones.get(name)
        if z is None:
            z = self.zones[name] = {"name": name, "kind": kind, "state": "EMPTY" if kind == "processing" else "OK",
                                    "x": 0.0, "y": 0.0, "yaw": 0.0, "length": 0.0, "width": 0.0, "board_id": "",
                                    "stack_count": 0, "stack_top_z": self.c["table_z"], "base_z": self.c["table_z"],
                                    "fixed": fixed, "seen": t, "since": t, "message": "", "occ_since": None,
                                    "empty_since": None}
        return z

    @staticmethod
    def _set_base(z, base):
        """V2: zone surface = sheet top (Vision 1 top_z / config z). The stack moves with it."""
        if base != z["base_z"]:
            z["stack_top_z"] += base - z["base_z"]
            z["base_z"] = base

    def _set_state(self, z, state, t, message=""):
        if z["state"] != state or z["message"] != message:
            self._note(t, f"zone {z['name']}: {z['state']} -> {state}" + (f" ({message})" if message else ""))
        if z["state"] != state:
            z["since"] = t
        z["state"], z["message"] = state, message

    def zone_of(self, x, y):
        """Name of the zone (+ margin) containing the point, or ''. Processing zones win over others."""
        best = ""
        for z in self.zones.values():
            if z["length"] > 0 and in_rect(x, y, z, self.c["zone_margin"]):
                if z["kind"] == "processing" or not best:
                    best = z["name"]
        return best

    # ---------- Vision 1 frame ----------
    def update(self, t, dets, mask=None):
        """mask (CL-31, hk_fr5_task.mask): None, or {"arm": [shapes], "tool": [shapes]} on the table plane.
        tool (gripper + held board): board detections touching it are ignored. arm: explains absence only - a board
        detection touching it is used only if it matches a known board and shows its full size (no new boards, no
        cut-off boxes); known boards touching the mask are not dropped as lost; a processing zone touching it with no
        board seen keeps its state (a covered zone must not look empty)."""
        c = self.c
        self.last_frame_t = t
        self.mask = mask or {}
        dets = [d for d in dets if d.get("score", 1.0) >= c["min_score"]]
        # zones (detected) + fixed fallbacks
        seen_zones = set()
        for d in dets:
            if not d["cls"].startswith("zone/"):
                continue
            kind = d["cls"].split("/", 1)[1]
            name = d.get("id") or f"{kind}_{len([z for z in self.zones.values() if z['kind'] == kind]) + 1}"
            z = self._zone(name, kind, t)
            z["x"], z["y"], z["yaw"], z["length"], z["width"] = rect_from_corners(d["corners"])
            z["seen"], z["fixed"] = t, False
            self._set_base(z, float(d.get("z", c["table_z"])))
            seen_zones.add(name)
        for f in c.get("fixed_zones") or []:
            if f["name"] not in seen_zones and (f["name"] not in self.zones or self.zones[f["name"]]["fixed"]):
                z = self._zone(f["name"], f["kind"], t, fixed=True)
                z.update({k: f[k] for k in ("x", "y", "yaw", "length", "width")}, seen=t, fixed=True)
                self._set_base(z, float(f.get("z", c["table_z"])))
        # obstacles
        self.obstacles = []
        w = c["work_area"]
        for d in dets:
            if d["cls"].startswith("obstacle/"):
                x, y, yaw, l, wd = rect_from_corners(d["corners"])
                inside = (w["x"][0] - w["margin"] <= x <= w["x"][1] + w["margin"]
                          and w["y"][0] - w["margin"] <= y <= w["y"][1] + w["margin"])
                self.obstacles.append({"label": d["cls"].split("/", 1)[1], "x": x, "y": y, "yaw": yaw, "length": l,
                                       "width": wd, "in_work_area": inside, "seen": t})
        # boards: associate detections with known boards
        bdets = sorted((d for d in dets if d["cls"].startswith("board/")), key=lambda d: -d.get("score", 1.0))
        known = c["variants"]
        free = {i: b for i, b in self.boards.items() if b["location"] != "held"}
        matched = set()
        for d in bdets:
            x, y, yaw, l, wd = rect_from_corners(d["corners"])
            if M.hits(self.mask.get("tool"), d["corners"]):
                continue                                     # the robot's own board / gripper (CL-31)
            zf = self.zone_of(x, y)
            if zf and self.zones[zf]["kind"] == "fault":
                continue                                     # CL-39: the fault zone is a void - boards there do not exist
            under_arm = M.hits(self.mask.get("arm"), d["corners"])
            label = d["cls"].split("/", 1)[1]
            if zf and label not in known and not (c.get("zone_unknown_boards", False) or c["expanded_detection"]):
                # TM-33 (user): non-variant (UNKWN) detections inside zones are hands / merged blobs, not boards
                # (no printed boards yet): ignored, so no ghost zone boards. Logged for debugging (every 10 s / zone).
                if t - self.unknown_noted.get(zf, -1e9) >= 10.0:
                    self.unknown_noted[zf] = t
                    self._note(t, f"ignored a {label} detection in zone {zf} (zone_unknown_boards: false)")
                continue
            best, bd = None, None
            for i, b in free.items():
                if i in matched:
                    continue
                dist = math.hypot(b["x"] - x, b["y"] - y)
                same_track = d.get("id") and d.get("id") == b.get("track")
                if dist <= c["match_dist"] or (same_track and dist <= 3 * c["match_dist"]):
                    key = (round(dist / 0.01), -max(b.get("seen", 0.0), b.get("placed", -1.0)))   # within 1 cm:
                    #   the most recently placed / seen wins (top of a stack, not the hidden board under it)
                    if bd is None or key < bd:                          # (top board of a stack, not the hidden one)
                        best, bd = i, key
            if under_arm and not self._full_view(best, label, l, wd):
                continue                                     # CL-31: under the arm only full views of known boards
            if best is None:
                best = self._new_board_id(t, x, y)
            b = self.boards.setdefault(best, {"id": best, "location": "input", "zone": "", "first": t,
                                              "variant": "unknown", "processed": False})
            if label in known and label != b["variant"] and "placed" not in b:   # TM-33: a board the ROBOT
                # placed keeps the variant we know (white on white / red on red merge into one blob, trial 1 TM-O12);
                # CL-19: a non-variant label never replaces the original variant (kept through processing)
                if b["variant"] != "unknown":
                    self._note(t, f"board {best}: variant {b['variant']} -> {label}")
                b["variant"] = label
            b.update(label=label, x=x, y=y, yaw=yaw, length=l, width=wd, score=d.get("score", 1.0),
                     track=d.get("id", ""), seen=t, inside=d.get("inside", True),
                     size_ok=self._size_ok(label, l, wd) if label in known else label in c["nonvariant_labels"])
            if b["location"] != "held":
                zn = self.zone_of(x, y)
                b["location"], b["zone"] = ("zone", zn) if zn else ("input", "")
            matched.add(best)
        # boards not seen: drop input boards after lost_after (zone boards may be hidden under a stack / away)
        for i in [i for i, b in self.boards.items() if i not in matched]:
            b = self.boards[i]
            if self._masked(rect_corners(b.get("x", 0.0), b.get("y", 0.0), b.get("yaw", 0.0),
                                         b.get("length", 0.0), b.get("width", 0.0))):
                b["lost_ref"] = t                            # hidden by the robot: the loss timer restarts after it
                continue
            if b["location"] == "input" and t - max(b["seen"], b.get("lost_ref", 0.0)) > c["lost_after"]:
                self._note(t, f"board {i} lost (not seen {t - b['seen']:.1f} s)")
                del self.boards[i]
        self._processing(t, matched)
        for z in self.zones.values():
            if z["kind"] == "drop_off":                      # fault zone: always OK (CL-39, never FULL)
                self._set_state(z, "FULL" if z["stack_count"] >= c["max_stack"] else "OK", t)

    def _masked(self, corners):
        """Any part of the footprint under the robot (arm or tool), CL-31."""
        m = getattr(self, "mask", None) or {}
        return M.hits(m.get("arm"), corners) or M.hits(m.get("tool"), corners)

    def _full_view(self, best, label, length, width):
        """CL-31: a detection under the arm counts only if it matches a known board and shows its full size (the
        variant's nominal size, else the size last seen of that board)."""
        if best is None:
            return False
        v = self._variant(label)
        b = self.boards[best]
        ref = (v["length"], v["width"]) if v else (b.get("length", 0.0), b.get("width", 0.0))
        tol = self.c["size_tol"]
        return ref[0] > 0 and abs(length - ref[0]) <= tol and abs(width - ref[1]) <= tol

    def _new_board_id(self, t, x, y):
        """New board, unless it is the board coming back into a PROCESSING zone (keeps its identity)."""
        zn = self.zone_of(x, y)
        z = self.zones.get(zn)
        if z and z["kind"] == "processing" and z["state"] == "PROCESSING" and z["board_id"] in self.boards:
            self._note(t, f"board {z['board_id']} back in {zn}")
            return z["board_id"]
        i = f"b{self._next}"
        self._next += 1
        self._note(t, f"new board {i}" + (f" in {zn}" if zn else ""))
        return i

    def _processing(self, t, matched):
        c = self.c
        for z in self.zones.values():
            if z["kind"] != "processing":
                continue
            present = [i for i in matched if self.boards[i]["location"] == "zone" and self.boards[i]["zone"] == z["name"]]
            if not present and self._masked(rect_corners(z["x"], z["y"], z["yaw"], z["length"], z["width"])):
                z["occ_since"] = z["empty_since"] = None     # covered, nothing seen: freeze, debounce restarts after
                continue
            if present:
                z["empty_since"] = None
                z["occ_since"] = z["occ_since"] if z["occ_since"] is not None else t
            else:
                z["occ_since"] = None
                z["empty_since"] = z["empty_since"] if z["empty_since"] is not None else t
            st = z["state"]
            if st == "LOADED" and not present and t - z["empty_since"] >= c["gone_after"]:
                self._set_state(z, "PROCESSING", t)
            elif st == "PROCESSING":
                if present and t - z["occ_since"] >= c["back_after"]:
                    b = self.boards[present[0]]
                    if c["expanded_detection"] and b.get("label") not in c["nonvariant_labels"]:
                        self._set_state(z, "FAULT", t, f"not processed: plain {b.get('label')} board came back")
                    else:
                        b["processed"] = True                     # CL-19: processed = state, not a label
                        self._set_state(z, "DONE", t)
                    z["board_id"] = present[0]
                elif not present and t - z["since"] > c["process_timeout"]:
                    self._set_state(z, "PROCESSING", t, f"overdue (> {c['process_timeout']:.0f} s)")   # state_age has the time
            elif st == "EMPTY" and present and t - z["occ_since"] >= c["unknown_after"]:
                z["board_id"] = present[0]
                self._set_state(z, "FAULT", t, f"unknown board {present[0]} (not placed by the robot)")
            elif st == "FAULT" and not present and z["empty_since"] is not None and t - z["empty_since"] >= c["gone_after"]:
                z["board_id"] = ""
                self._set_state(z, "EMPTY", t, "")
                gone = [i for i, b in self.boards.items() if b["location"] == "zone" and b["zone"] == z["name"]]
                for i in gone:                               # TM-33: cleared by itself -> no stale boards left there
                    del self.boards[i]
                self._note(t, f"zone {z['name']}: FAULT cleared (seen empty {c['gone_after']:.0f} s)"
                              + (f", removed {', '.join(gone)}" if gone else ""))

    # ---------- skill events ----------
    def event(self, t, ev):
        typ, i = ev.get("type"), ev.get("board", "")
        if typ == "picked":
            b = self.boards.get(i)
            if b is None:
                raise ValueError(f"picked: unknown board '{i}'")
            z = self.zones.get(b["zone"]) if b["location"] == "zone" else None
            if z and z["kind"] == "processing":
                z["board_id"] = ""
                self._set_state(z, "EMPTY", t)
            elif z:                                          # exactly THIS board off the stack (mixed variants)
                z["stack_count"] = max(0, z["stack_count"] - 1)
                z["stack_top_z"] = (z["base_z"] if z["stack_count"] == 0 else
                                    max(z["base_z"], z["stack_top_z"] - self._stack_thickness(b["variant"])))
            b.update(location="held", zone="")
            self.held = i
            self._note(t, f"picked {i}")
        elif typ == "placed":
            b = self.boards.get(i)
            if b is None:
                raise ValueError(f"placed: unknown board '{i}'")
            zn = ev.get("zone", "")
            z = self.zones.get(zn)
            if z and z["kind"] == "fault":                   # CL-39: void - the board leaves the world
                del self.boards[i]
                self.held = ""
                self._note(t, f"placed {i} in {zn} (fault zone: removed from the world)")
                return
            b.update(location="zone" if zn else "input", zone=zn, placed=t)   # 'seen' only from the camera
            for k in ("x", "y", "yaw"):
                if k in ev:
                    b[k] = float(ev[k])
            self.held = ""
            if z and z["kind"] == "processing":
                z["board_id"] = i
                z["occ_since"], z["empty_since"] = t, None
                if z["state"] == "EMPTY":
                    self._set_state(z, "LOADED", t)
                else:
                    self._set_state(z, "FAULT", t, f"placed {i} into a busy zone ({z['state']})")
            elif z:
                z["stack_count"] += 1
                z["stack_top_z"] += self._stack_thickness(b["variant"])
            self._note(t, f"placed {i}" + (f" in {zn}" if zn else ""))
        elif typ == "dropped":
            self.boards.pop(i, None)
            self.held = ""
            self._note(t, f"dropped {i} (lost in transit)")
        elif typ == "reset_zone":
            z = self.zones.get(ev.get("zone", ""))
            if z is None:
                raise ValueError(f"reset_zone: unknown zone '{ev.get('zone')}'")
            z.update(board_id="", stack_count=0, stack_top_z=z["base_z"])
            self._set_state(z, "EMPTY" if z["kind"] == "processing" else "OK", t)
            gone = [i for i, b in self.boards.items() if b["location"] == "zone" and b["zone"] == z["name"]]
            for i in gone:                                   # the operator emptied it: no ghost boards stay behind
                del self.boards[i]                           # (a new board there must not take an old id)
            if gone:
                self._note(t, f"reset {z['name']}: removed {', '.join(gone)}")
        else:
            raise ValueError(f"unknown event type '{typ}'")

    # ---------- output ----------
    def snapshot(self, t):
        c = self.c
        boards = []
        for b in self.boards.values():
            if b["location"] == "input":
                top = NAN                                      # input stack height unknown -> contact search (CL-16)
            elif b["location"] == "zone" and self.zones.get(b["zone"], {}).get("kind") != "processing":
                top = self.zones[b["zone"]]["stack_top_z"]
            elif b["location"] == "zone" and b["zone"] in self.zones:   # processing: one board on the sheet (V2)
                top = self.zones[b["zone"]]["base_z"] + self._thickness(b.get("variant", ""))
            else:
                top = c["table_z"] + self._thickness(b.get("variant", ""))
            boards.append({"id": b["id"], "variant": b.get("variant", "unknown"), "label": b.get("label", ""),
                           "processed": b.get("processed", False), "location": b["location"],
                           "zone": b["zone"], "x": b.get("x", NAN), "y": b.get("y", NAN), "yaw": b.get("yaw", 0.0),
                           "length": b.get("length", 0.0), "width": b.get("width", 0.0),
                           "thickness": self._thickness(b.get("variant", "")), "top_z": top,
                           "score": b.get("score", 0.0), "inside": b.get("inside", True), "size_ok": b.get("size_ok", False),
                           "age": t - b.get("seen", t)})
        zones = [{**{k: z[k] for k in ("name", "kind", "state", "x", "y", "yaw", "length", "width", "board_id",
                                        "stack_count", "stack_top_z", "base_z", "fixed", "message")},
                  "age": 0.0 if z["fixed"] else t - z["seen"], "state_age": t - z["since"]} for z in self.zones.values()]
        obstacles = [{**{k: o[k] for k in ("label", "x", "y", "yaw", "length", "width", "in_work_area")},
                      "age": t - o["seen"]} for o in self.obstacles]
        age = NAN if self.last_frame_t is None else t - self.last_frame_t
        return {"vision_ok": self.last_frame_t is not None and age < c["stale_after"], "vision_age": age,
                "boards": sorted(boards, key=lambda b: (len(b["id"]), b["id"])), "zones": sorted(zones, key=lambda z: z["name"]),
                "obstacles": obstacles, "held_board_id": self.held}
