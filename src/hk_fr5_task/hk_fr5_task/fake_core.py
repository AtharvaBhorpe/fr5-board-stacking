"""Fake world core (decision-log TM-5 / TM-8): a scenario on the table rendered as Vision 1 detections in REAL camera
pixels (inverse of the real calibration), reacting to the skills' events and simulating the operator (processing:
take the board away, bring it / a processed one back) and hands. No ROS; time is passed in.

Detections out (pixels, Vision 1 contract, project-plan P-1..P-3): {"cls", "id", "score", "u", "v", "theta", "sx", "sy"}
= Detection2D bbox (centre, rotation, size) + class + track id.
"""
import math

import numpy as np

from hk_fr5_task.world_core import rect_corners


class Camera:
    """base_link (x, y on the table) <-> pixel with a calib.yaml dict (hk_fr5_vision calib model V-18)."""

    def __init__(self, calib):
        self.c = calib
        self.H = np.array(calib["homography"], float)
        self.Hi = np.linalg.inv(self.H)
        self.k = float(calib["lens_k"])
        self.cen = np.array(calib["image_size"], float) / 2.0
        self.n = float(max(calib["image_size"]))

    def to_pixel(self, x, y):
        q = self.Hi @ np.array([x, y, 1.0])
        pu = q[:2] / q[2]                       # undistorted pixel; invert p_u = c + (p_d - c)(1 + k |p_d - c|^2 / n^2)
        d = pu - self.cen
        dd = d.copy()
        for _ in range(30):
            dd = d / (1.0 + self.k * float(np.sum((dd / self.n) ** 2)))
        p = self.cen + dd
        return float(p[0]), float(p[1])

    def to_table(self, u, v):
        d = np.array([u, v], float) - self.cen
        pu = self.cen + d * (1.0 + self.k * float(np.sum((d / self.n) ** 2)))
        q = self.H @ np.array([pu[0], pu[1], 1.0])
        return float(q[0] / q[2]), float(q[1] / q[2])


def bbox_from_pixels(pts):
    """4 pixel corners (in order) -> rotated box (u, v, theta of the 1st edge pair, size along it, size across)."""
    p = np.asarray(pts, float)
    u, v = p.mean(axis=0)
    e0, e1 = (p[1] - p[0] + p[2] - p[3]) / 2, (p[3] - p[0] + p[2] - p[1]) / 2
    return float(u), float(v), math.atan2(e0[1], e0[0]), float(np.linalg.norm(e0)), float(np.linalg.norm(e1))


def bbox_corners(u, v, theta, sx, sy):
    """Rotated box -> 4 pixel corners (same order as bbox_from_pixels expects)."""
    c, s = math.cos(theta), math.sin(theta)
    ax, ay = c * sx / 2, s * sx / 2
    bx, by = -s * sy / 2, c * sy / 2
    return [[u - ax - bx, v - ay - by], [u + ax - bx, v + ay - by], [u + ax + bx, v + ay + by], [u - ax + bx, v - ay + by]]


class FakeWorld:
    def __init__(self, scenario, camera, variants, seed=0):
        self.s, self.cam, self.variants = scenario, camera, variants
        self.rng = np.random.default_rng(seed)
        self.zones = [dict(z) for z in scenario.get("zones", [])]
        self.boards = []
        self._n = 0
        for b in scenario.get("boards", []):
            self._add(b["variant"], b["x"], b["y"], b.get("yaw", 0.0), b.get("count", 1))
        self.held = None
        self.pending = []                       # (t, action, data) operator actions
        self.hands = [dict(h) for h in scenario.get("hands", [])]
        self.log = []

    def _add(self, variant, x, y, yaw, count=1):
        self._n += 1
        self.boards.append({"track": f"t{self._n}", "variant": variant, "x": x, "y": y, "yaw": yaw, "count": count})

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

    def event(self, t, ev):
        typ = ev.get("type")
        if typ == "picked":
            if not self.boards:
                raise ValueError("picked: no board on the table")
            b = min(self.boards, key=lambda b: math.hypot(b["x"] - ev["x"], b["y"] - ev["y"]))
            if math.hypot(b["x"] - ev["x"], b["y"] - ev["y"]) > 0.05:
                raise ValueError(f"picked: no board within 50 mm of ({ev['x']:.3f}, {ev['y']:.3f})")
            self.held = {"variant": b["variant"]}
            b["count"] -= 1
            if b["count"] <= 0:
                self.boards.remove(b)
            self.log.append((t, f"picked {b['variant']} at ({b['x']:.3f}, {b['y']:.3f}), {b['count']} left there"))
        elif typ == "placed":
            variant = ev.get("variant") or (self.held or {}).get("variant", "white_small")
            x, y, yaw = float(ev["x"]), float(ev["y"]), float(ev.get("yaw", 0.0))
            on = [b for b in self.boards if math.hypot(b["x"] - x, b["y"] - y) < 0.03]
            if on:                              # on a stack: the top board changes, one track stays
                on[0].update(variant=variant, x=x, y=y, yaw=yaw); on[0]["count"] += 1
            else:
                self._add(variant, x, y, yaw)
            self.held = None
            z = self.zone_at(x, y)
            if z and z["kind"] == "processing":
                op = self.s.get("operator", {})
                self.pending.append((t + op.get("take_after", 1.5), "take", z["name"]))
                self.pending.append((t + op.get("take_after", 1.5) + op.get("return_after", 3.0), "return", z["name"]))
            self.log.append((t, f"placed {variant} at ({x:.3f}, {y:.3f})" + (f" in {z['name']}" if z else "")))
        elif typ == "dropped":
            self.held = None
        else:
            raise ValueError(f"unknown event type '{typ}'")

    def _operator(self, t):
        mode = self.s.get("operator", {}).get("return_variant", "same")    # same | nonvariant (printed board, CL-19)
        for item in [p for p in self.pending if p[0] <= t]:
            self.pending.remove(item)
            _, action, zname = item
            z = next(z for z in self.zones if z["name"] == zname)
            here = [b for b in self.boards if self.zone_at(b["x"], b["y"]) is z]
            if action == "take" and here:
                self._taken = getattr(self, "_taken", {}); self._taken[zname] = here[0]["variant"]
                self.boards.remove(here[0])
                self.log.append((t, f"operator took the board out of {zname}"))
            elif action == "return":
                old = getattr(self, "_taken", {}).pop(zname, "white_small")
                variant = self.s.get("nonvariant_label", "other") if mode == "nonvariant" else old
                self._add(variant, z["x"] + 0.004, z["y"] - 0.003, z["yaw"] + 0.05)   # never exactly where it was
                self.log.append((t, f"operator put a {variant} board back into {zname}"))

    def detections(self, t):
        self._operator(t)
        noise = self.s.get("noise_px", 0.3)
        out = []
        def add(cls, tid, x, y, yaw, l, w, score):
            pts = [self.cam.to_pixel(px, py) for px, py in rect_corners(x, y, yaw, l, w)]
            pts = np.array(pts) + (self.rng.normal(0, noise, (4, 2)) if noise else 0)
            u, v, th, sx, sy = bbox_from_pixels(pts)
            out.append({"cls": cls, "id": tid, "score": score, "u": u, "v": v, "theta": th, "sx": sx, "sy": sy})
        for z in self.zones:
            add(f"zone/{z['kind']}", z["name"], z["x"], z["y"], z["yaw"], z["length"], z["width"], 0.95)
        for b in self.boards:
            l, w = self._size(b["variant"])
            add(f"board/{b['variant']}", b["track"], b["x"], b["y"], b["yaw"], l, w, 0.9)
        for h in self.hands:
            if h["t"] <= t < h["t"] + h["duration"]:
                add(f"obstacle/{h.get('label', 'hand')}", "", h["x"], h["y"], 0.0, h.get("length", 0.12), h.get("width", 0.09), 0.8)
        return out
