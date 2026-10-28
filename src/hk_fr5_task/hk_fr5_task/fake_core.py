"""Fake world core (decision-log TM-5 / TM-8, V5 TM-24): a scenario on the table rendered as Vision 1 frames, reacting
to the skills' events and simulating the operator (processing: take the board away, bring it / a processed one back)
and hands. No ROS; time is passed in.

Frame out = the /vision/boards JSON of the agreed schema (notes/vision1-changes.md): stamp, calibrated, boards[]
{id (per frame), variant, corners_xy, conf, inside_calibration}, zones[] {name, kind, x, y, yaw, age, size, top_z}
(drop_off / processing only: the fault zone is ours), + the optional obstacles[] {label, corners_xy} (LATER item,
TM-24). Corner noise in metres (noise_xy). Scenario zones: x, y, yaw, length, width (board area), top_z (sheet top).
"""
import math

import numpy as np

from hk_fr5_task import mask
from hk_fr5_task.world_core import rect_corners

VISION1_ZONE_KINDS = ("drop_off", "processing")


class FakeWorld:
    def __init__(self, scenario, variants, seed=0):
        self.s, self.variants = scenario, variants
        self.rng = np.random.default_rng(seed)
        self.zones = [dict(z) for z in scenario.get("zones", [])]
        self.boards = []
        self._n = 0
        for b in scenario.get("boards", []):
            self._add(b["variant"], b["x"], b["y"], b.get("yaw", 0.0), b.get("count", 1))
        self.held = None
        self.pending = []                       # (t, action, data) operator actions
        self.tcp = None                         # tcp (x, y) from TF (fake_world); None = no arm known
        self.free_since = {}                    # zone -> t since the arm has not covered it
        self.hands = [dict(h) for h in scenario.get("hands", [])]
        self.log = []

    def _add(self, variant, x, y, yaw, count=1):
        self._n += 1
        self.boards.append({"track": f"t{self._n}", "variant": variant, "x": x, "y": y, "yaw": yaw, "count": count,
                            "layers": [variant] * count})            # bottom -> top (mixed stacks: own thickness each)

    def _size(self, variant):
        v = self.variants.get(variant, {})
        if variant not in self.variants:                     # non-variant (printed) board, CL-19
            return self.s.get("nonvariant_size", [0.15, 0.10])
        return [v.get("length", 0.1), v.get("width", 0.07)]

    def zone_at(self, x, y):
        for z in self.zones:
            c, s = math.cos(z["yaw"]), math.sin(z["yaw"])
            u, w = c * (x - z["x"]) + s * (y - z["y"]), -s * (x - z["x"]) + c * (y - z["y"])
            if abs(u) <= z["length"] / 2 and abs(w) <= z["width"] / 2:
                return z
        return None

    def base_at(self, x, y, table_z):
        """Surface under the point without boards: the zone sheet top (zone top_z) inside a zone, else the table."""
        z = self.zone_at(x, y)
        return z.get("top_z", table_z) if z else table_z

    def surface_at(self, x, y, table_z):
        """Top surface z under the point: top of the board / stack whose footprint contains it (on the sheet if it
        lies in a zone), else the sheet / the table."""
        for b in self.boards:
            l, w = self._size(b["variant"])
            c, s = math.cos(b["yaw"]), math.sin(b["yaw"])
            u, v = c * (x - b["x"]) + s * (y - b["y"]), -s * (x - b["x"]) + c * (y - b["y"])
            if abs(u) <= l / 2 and abs(v) <= w / 2:
                return self.base_at(b["x"], b["y"], table_z) + sum(
                    self.variants.get(lv, {}).get("thickness", 0.005) for lv in b["layers"])
        return self.base_at(x, y, table_z)

    def event(self, t, ev):
        typ = ev.get("type")
        if typ == "picked":
            if not self.boards:
                raise ValueError("picked: no board on the table")
            b = min(self.boards, key=lambda b: math.hypot(b["x"] - ev["x"], b["y"] - ev["y"]))
            if math.hypot(b["x"] - ev["x"], b["y"] - ev["y"]) > 0.05:
                raise ValueError(f"picked: no board within 50 mm of ({ev['x']:.3f}, {ev['y']:.3f})")
            self.held = {"variant": b["variant"], "yaw": b["yaw"]}
            b["layers"].pop()
            b["count"] = len(b["layers"])
            if b["count"] <= 0:
                self.boards.remove(b)
            else:
                b["variant"] = b["layers"][-1]                 # the board below is now on top
            self.log.append((t, f"picked {b['variant']} at ({b['x']:.3f}, {b['y']:.3f}), {b['count']} left there"))
        elif typ == "placed":
            variant = ev.get("variant") or (self.held or {}).get("variant", next(iter(self.variants), ""))
            x, y, yaw = float(ev["x"]), float(ev["y"]), float(ev.get("yaw", 0.0))
            on = [b for b in self.boards if math.hypot(b["x"] - x, b["y"] - y) < 0.03]
            if on:                              # on a stack: the top board changes, one track stays
                on[0].update(variant=variant, x=x, y=y, yaw=yaw); on[0]["layers"].append(variant)
                on[0]["count"] = len(on[0]["layers"])
            else:
                self._add(variant, x, y, yaw)
            self.held = None
            z = self.zone_at(x, y)
            if z and z["kind"] == "processing":
                op = self.s.get("operator", {})
                self.pending.append((t + op.get("take_after", 1.5), "take", z["name"]))   # return: after the take
            self.log.append((t, f"placed {variant} at ({x:.3f}, {y:.3f})" + (f" in {z['name']}" if z else "")))
        elif typ == "dropped":                  # vacuum lost in flight: the board falls under the tcp
            self._drop(t)
        else:
            raise ValueError(f"unknown event type '{typ}'")

    def _drop(self, t):
        """Vacuum loss (TM-38): the held board lands under the tcp (small random offset / turn) if that is on a table in
        the camera area (drop_area), else it is gone (over the rail between the tables, off the table, tcp unknown).
        A board landing in a zone is a zone board (not re-picked); on the table it is a new input board."""
        held, self.held = self.held, None
        if held is None:
            return
        a = self.s.get("drop_area", {})
        if self.tcp is None:
            self.log.append((t, f"dropped {held['variant']}: gone (tcp unknown)"))
            return
        x = self.tcp[0] + float(self.rng.normal(0, a.get("scatter", 0.01)))
        y = self.tcp[1] + float(self.rng.normal(0, a.get("scatter", 0.01)))
        xr, yr = a.get("x", [-0.66, 0.09]), a.get("y", [-0.86, 0.84])
        if not (xr[0] <= x <= xr[1] and yr[0] <= y <= yr[1]) or abs(y) < a.get("rail_half_width", 0.08):
            self.log.append((t, f"dropped {held['variant']} at ({x:.3f}, {y:.3f}): gone (off the table / on the rail)"))
            return
        yaw = held.get("yaw", 0.0) + float(self.rng.normal(0, a.get("turn", 0.15)))
        self._add(held["variant"], x, y, yaw)
        z = self.zone_at(x, y)
        self.log.append((t, f"dropped {held['variant']}: landed at ({x:.3f}, {y:.3f})"
                            + (f" in {z['name']}" if z else " on the table")))

    def _covered(self, z):
        """The arm (band base -> tcp, + the held board) covers zone z in the camera view (same shape as the world
        model's mask, CL-31)."""
        if self.tcp is None:
            return False
        r = 0.0
        if self.held:
            l, w = self._size(self.held["variant"])
            r = math.hypot(l, w) / 2 + 0.03
        band = mask.band(self.tcp, self.s.get("operator", {}).get("arm_radius", 0.12), r)
        return mask.hits(band.get("arm"), rect_corners(z["x"], z["y"], z["yaw"], z["length"], z["width"])) or \
            mask.hits(band.get("tool"), rect_corners(z["x"], z["y"], z["yaw"], z["length"], z["width"]))

    def _operator(self, t):
        """TM-37: the operator follows the demo routine: takes the board only when the arm does not cover the zone,
        and puts it back return_after s later, only after the zone was in free view for free_view s (the world must
        see it empty, gone_after 2 s). Before, fixed times raced with the arm (no home after a place, TM-36)."""
        op = self.s.get("operator", {})
        mode = op.get("return_variant", "same")    # same | nonvariant (printed board, CL-19)
        for z in self.zones:
            if self._covered(z):
                self.free_since[z["name"]] = None
            elif self.free_since.get(z["name"]) is None:
                self.free_since[z["name"]] = t
        for item in [p for p in self.pending if p[0] <= t]:
            _, action, zname = item
            z = next(z for z in self.zones if z["name"] == zname)
            since = self.free_since.get(zname)
            if since is None or (action == "return" and t - since < op.get("free_view", 3.0)):
                continue                                      # the arm covers the zone: wait
            self.pending.remove(item)
            here = [b for b in self.boards if self.zone_at(b["x"], b["y"]) is z]
            if action == "take" and here:
                self.pending.append((t + op.get("return_after", 3.0), "return", zname))
                self._taken = getattr(self, "_taken", {}); self._taken[zname] = here[0]["variant"]
                self.boards.remove(here[0])
                self.log.append((t, f"operator took the board out of {zname}"))
            elif action == "return":
                old = getattr(self, "_taken", {}).pop(zname, next(iter(self.variants), ""))
                variant = self.s.get("nonvariant_label", "other") if mode == "nonvariant" else old
                self._add(variant, z["x"] + 0.004, z["y"] - 0.003, z["yaw"] + 0.05)   # never exactly where it was
                self.log.append((t, f"operator put a {variant} board back into {zname}"))

    def frame(self, t, stamp):
        """The Vision 1 frame at simulated time t (operator runs first); stamp = capture time to put in it."""
        self._operator(t)
        noise = self.s.get("noise_xy", 0.001)
        def corners(x, y, yaw, l, w):
            c = np.array(rect_corners(x, y, yaw, l, w)) + (self.rng.normal(0, noise, (4, 2)) if noise else 0)
            return [[round(float(px), 5), round(float(py), 5)] for px, py in c]
        boards = [{"id": k, "variant": b["variant"], "corners_xy": corners(b["x"], b["y"], b["yaw"], *self._size(b["variant"])),
                   "conf": 0.9, "inside_calibration": True} for k, b in enumerate(self.boards)]
        zones = [{"name": z["name"], "kind": z["kind"], "x": z["x"], "y": z["y"], "yaw": z["yaw"], "age": 0.0,
                  "size": [z["length"], z["width"]], "top_z": z["top_z"], "frames": 15}
                 for z in self.zones if z["kind"] in VISION1_ZONE_KINDS]
        obstacles = [{"label": h.get("label", "hand"), "corners_xy": corners(h["x"], h["y"], 0.0, h.get("length", 0.12),
                                                                           h.get("width", 0.09))}
                     for h in self.hands if h["t"] <= t < h["t"] + h["duration"]]
        return {"stamp": stamp, "source": "fake_world", "calibrated": True, "image_size": [1920, 1080],
                "boards": boards, "occluded": [], "rejected": [], "zones": zones, "obstacles": obstacles}
