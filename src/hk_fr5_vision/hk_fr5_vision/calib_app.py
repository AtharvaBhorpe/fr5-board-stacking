"""Calibration app backend (decision-log V-9, V-11, V-15, V-19..V-21). No ROS; READ-ONLY towards the robot.

Workflow (V-23): place the tags, "identify reference tags" (all visible + stable tags), robot in manual mode (our motion
bringup OFF), hand-guide the cup onto a tag, "record" 2-3x per tag (other orientation / approach), fit (lens term +
homography, draft), verify: any NON-reference tag is the moving tag, 5-6 spots, save -> calib.yaml (the
vision_transform node reloads it). Web page: <share>/web/index.html (V7); JSON API below.

    ros2 run hk_fr5_vision calib_app [--config <yaml>] [--image <file>] [--no-robot]
    -> http://127.0.0.1:8080
"""
import argparse
import asyncio
import collections
import json
import math
import os
import threading
import time

import cv2
import numpy as np
import yaml
from aiohttp import web
from ament_index_python.packages import get_package_share_directory

from hk_fr5_vision import calib, robot_pose

SHARE = get_package_share_directory("hk_fr5_vision")


def load_app_config(path):
    with open(path) as f:
        c = yaml.safe_load(f)
    for sec, keys in {"camera": ("device", "width", "height", "name", "jpeg_quality", "stream_fps"),
                      "tags": ("dictionary", "allowed_ids", "verify_seen_s", "min_samples", "window", "stable_px", "seen_s",
                               "moved_px", "moved_frames"),
                      "record": ("max_per_tag", "spread_warn_mm", "table_z", "table_z_tol", "max_age_s"),
                      "verify": ("mean_mm", "max_mm", "min_spots", "same_spot_mm")}.items():
        for k in keys:
            if k not in c.get(sec, {}):
                raise ValueError(f"{path}: missing '{sec}.{k}'")
    c["calib_file"] = c.get("calib_file") or os.path.join(SHARE, "config", "calib.yaml")
    c["robot_config"] = c.get("robot_config") or os.path.join(SHARE, "config", "robot.yaml")
    c["session_file"] = os.path.expanduser(c.get("session_file") or "~/.local/state/hk_fr5_vision/calib_session.json")
    return c


class TagTracker:
    """Per tag: running average of the centre pixel (kept while the cup hides the tag), spread, last seen, corners.
    A tag that moved (detections far from its average for several frames) restarts its average, epoch + 1."""

    def __init__(self, cfg):
        self.c = cfg
        self.lock = threading.Lock()
        self.t = {}

    def update(self, dets, now):
        with self.lock:
            for i, (centre, corners) in dets.items():
                s = self.t.setdefault(i, {"samples": collections.deque(maxlen=self.c["window"]), "seen": 0.0,
                                          "corners": None, "far": 0, "epoch": 0, "moved_at": None})
                s["seen"], s["corners"] = now, corners
                if len(s["samples"]) >= self.c["min_samples"]:
                    m = np.mean(s["samples"], axis=0)
                    if math.dist(m, centre) > self.c["moved_px"]:
                        s["far"] += 1
                        if s["far"] < self.c["moved_frames"]:
                            continue                    # outlier (or the start of a move): not averaged
                        s["samples"].clear(); s["epoch"] += 1; s["moved_at"] = now
                    s["far"] = 0
                s["samples"].append(centre)

    def get(self, i, now=None):
        """dict for tag i or None: u, v (average), std, n, visible, stable, epoch, corners, age."""
        now = time.monotonic() if now is None else now
        with self.lock:
            s = self.t.get(i)
            if not s or not s["samples"]:
                return None
            a = np.array(s["samples"])
            u, v = a.mean(axis=0)
            std = float(np.sqrt(np.sum(a.var(axis=0)))) if len(a) > 1 else float("inf")
            return {"u": float(u), "v": float(v), "std_px": std, "n": len(a), "epoch": s["epoch"],
                    "visible": now - s["seen"] < self.c["seen_s"], "age_s": now - s["seen"],
                    "stable": len(a) >= self.c["min_samples"] and std < self.c["stable_px"],
                    "corners": s["corners"], "moved_s_ago": None if s["moved_at"] is None else now - s["moved_at"]}

    def ids(self):
        with self.lock:
            return sorted(self.t)


class Camera(threading.Thread):
    """Frames from a V4L2 device (index or path) or an image file (tests; re-read when it changes). Detects tags,
    feeds the tracker, keeps the latest JPEG (no overlay: drawn in the browser)."""

    def __init__(self, cfg, tracker, image=None):
        super().__init__(daemon=True)
        self.c, self.tracker, self.image = cfg["camera"], tracker, image
        t = cfg["tags"]
        self.allowed = set(t["allowed_ids"]) or None
        p = cv2.aruco.DetectorParameters()          # tuned on 30 real C920 frames (pix2table, 2026-10-26)
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        p.adaptiveThreshWinSizeMax = 53
        p.minMarkerPerimeterRate = 0.01
        p.errorCorrectionRate = 0.6
        p.polygonalApproxAccuracyRate = 0.05
        p.adaptiveThreshWinSizeStep = 4
        p.minCornerDistanceRate = 0.02
        p.perspectiveRemovePixelPerCell = 8
        p.perspectiveRemoveIgnoredMarginPerCell = 0.2
        self.det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, t["dictionary"])), p)
        self.lock = threading.Lock()
        self.jpeg, self.size, self.status, self.fps = None, None, "starting", 0.0
        self.running = True

    def _open(self):
        d = self.c["device"]
        cap = cv2.VideoCapture(d if isinstance(d, str) else int(d), cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.c["width"])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.c["height"])
        return cap

    def run(self):
        cap, fails, mtime, frame, times = None, 0, None, None, collections.deque(maxlen=30)
        while self.running:
            if self.image:
                try:
                    m = os.stat(self.image).st_mtime_ns
                except OSError:
                    self.status = f"cannot read {self.image}"; time.sleep(0.5); continue
                if m != mtime:
                    frame, mtime = cv2.imread(self.image), m
                time.sleep(0.03)
                if frame is None:
                    self.status = f"cannot read {self.image}"; continue
                img = frame.copy()
            else:
                if cap is None:
                    cap = self._open()
                ok, img = cap.read()
                if not ok:
                    fails += 1
                    self.status = f"camera {self.c['device']}: no frame ({fails}), reconnecting"
                    time.sleep(0.2)
                    if fails % 10 == 0:
                        cap.release(); cap = None
                    continue
                fails = 0
            now = time.monotonic(); times.append(now)
            corners, ids, _ = self.det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
            dets = {}
            if ids is not None:
                for c, i in zip(corners, ids.flatten()):
                    if self.allowed is None or int(i) in self.allowed:
                        dets[int(i)] = (c[0].mean(axis=0), c[0].tolist())
            self.tracker.update(dets, now)
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self.c["jpeg_quality"]])
            with self.lock:
                if ok:
                    self.jpeg = buf.tobytes()
                self.size = (img.shape[1], img.shape[0])
                self.status = "ok"
                self.fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else 0.0
        if cap is not None:
            cap.release()


class App:
    """State + logic. Every change is written to the session file (records survive an app restart)."""

    def __init__(self, cfg, image=None, robot=True):
        self.c = cfg
        self.lock = threading.Lock()
        self.tracker = TagTracker(cfg["tags"])
        self.camera = Camera(cfg, self.tracker, image)
        self.rcfg = robot_pose.load_config(cfg["robot_config"])
        self.reader = robot_pose.RobotReader(self.rcfg) if robot else None
        self.records, self.draft, self.verify, self.refs = {}, None, {"for": None, "spots": []}, []
        self.saved, self.saved_error, self.saved_stamp = None, "", None
        self._load_session()
        self._reload_saved()

    def start(self):
        self.camera.start()
        if self.reader:
            self.reader.start()

    def stop(self):
        self.camera.running = False
        if self.reader:
            self.reader.stop()

    # ---- persistence ----
    def _load_session(self):
        try:
            with open(self.c["session_file"]) as f:
                s = json.load(f)
            self.records = {int(k): v for k, v in s.get("records", {}).items()}
            self.draft, self.verify = s.get("draft"), s.get("verify", {"for": None, "spots": []})
            self.refs = [int(i) for i in s.get("refs", [])]
        except (OSError, ValueError):
            pass

    def _save_session(self):
        p = self.c["session_file"]
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p + ".tmp", "w") as f:
            json.dump({"refs": self.refs, "records": self.records, "draft": self.draft, "verify": self.verify}, f)
        os.replace(p + ".tmp", p)

    def _reload_saved(self):
        try:
            st = os.stat(self.c["calib_file"]); stamp = (st.st_mtime_ns, st.st_size)
        except OSError:
            stamp = None
        if stamp == self.saved_stamp and stamp is not None:
            return
        self.saved_stamp = stamp
        try:
            self.saved, self.saved_error = calib.load(self.c["calib_file"]), ""
        except ValueError as e:
            self.saved, self.saved_error = None, str(e)

    def active(self):
        """(calibration used for verify / test, 'draft' | 'saved' | None)."""
        self._reload_saved()
        if self.draft:
            return self.draft, "draft"
        return (self.saved, "saved") if self.saved else (None, None)

    # ---- robot / manual pose ----
    def _pose(self, req):
        """(x, y, info dict, warnings) from the robot or from typed values; raises ValueError."""
        src, warn = req.get("source", "robot"), []
        if src in ("manual", "pendant"):
            try:
                x, y = float(req["x"]), float(req["y"])
            except (KeyError, TypeError, ValueError):
                raise ValueError("x and y must be numbers")
            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError("x and y must be finite numbers")
            if src == "pendant":   # pendant / web app: mm, x / y negated vs base_link; flange = cup x / y if vertical
                x, y = -x / 1000.0, -y / 1000.0
                warn.append("pendant values: x / y are the flange, equal to the cup only with the tool pointing straight down")
            return x, y, {"source": src}, warn
        if src != "robot":
            raise ValueError(f"unknown source '{src}' (robot | pendant | manual)")
        if not self.reader:
            raise ValueError("robot reader off (--no-robot): use pendant or manual values")
        st, age = self.reader.latest()
        if st is None or age > self.c["record"]["max_age_s"]:
            raise ValueError(f"no fresh robot pose ({self.reader.status()['last_error'] or 'stale'}): is the robot "
                             "connected and our bringup off?")
        p = robot_pose.tcp_from_state(st, self.rcfg)
        r = self.c["record"]
        if not p["manual_mode"]:
            warn.append("robot not in manual mode")
        if not p["tilt_ok"]:
            warn.append(f"tool tilted {p['tilt_deg']:.1f} deg (FK still gives the true cup position; check the cup sits flat)")
        if abs(p["z"] - r["table_z"]) > r["table_z_tol"]:
            warn.append(f"tcp z {p['z'] * 1000:.0f} mm, table at {r['table_z'] * 1000:.0f} mm: cup not on the table?")
        return p["x"], p["y"], {"source": "robot", "z": p["z"], "tilt_deg": round(p["tilt_deg"], 2),
                                "flange_check_mm": round(p["flange_check_mm"], 1)}, warn

    # ---- reference tags (V-23) ----
    def identify(self, _req=None):
        """Reference tags = all tags visible + stable now. Records of tags that stay in the set are kept."""
        now = time.monotonic()
        ids = [i for i in self.tracker.ids() if (t := self.tracker.get(i, now)) and t["visible"] and t["stable"]]
        if len(ids) < calib.MIN_TAGS:
            raise ValueError(f"only {len(ids)} stable tags visible ({ids}): need {calib.MIN_TAGS}+ (8 recommended)")
        warn = [] if len(ids) == 8 else [f"{len(ids)} reference tags (8 recommended; lens term needs 6+)"]
        with self.lock:
            dropped = [i for i in self.records if i not in ids]
            if dropped:
                warn.append(f"records of tags {dropped} dropped (not in the new set)")
            self.refs = ids
            self.records = {i: r for i, r in self.records.items() if i in ids}
            self.draft = None
            self._save_session()
        return {"ok": True, "refs": ids, "warnings": warn}

    def remove_ref(self, req):
        i = int(req["id"])
        with self.lock:
            if i not in self.refs:
                raise ValueError(f"tag {i} is not a reference tag")
            self.refs.remove(i); self.records.pop(i, None); self.draft = None
            self._save_session()
        return {"ok": True, "refs": self.refs}

    # ---- calibration records ----
    def record(self, req):
        i = int(req["id"])
        if i not in self.refs:
            raise ValueError(f"tag {i} is not a reference tag (identify the reference tags first)")
        tag = self.tracker.get(i)
        if not tag or tag["n"] < self.c["tags"]["min_samples"]:
            raise ValueError(f"tag {i}: not seen long enough by the camera (needs {self.c['tags']['min_samples']} detections)")
        x, y, info, warn = self._pose(req)
        with self.lock:
            recs = self.records.setdefault(i, [])
            if len(recs) >= self.c["record"]["max_per_tag"]:
                raise ValueError(f"tag {i} has {len(recs)} records already: delete one first")
            if recs and recs[0]["epoch"] != tag["epoch"]:
                warn.append(f"tag {i} moved since its earlier records: delete them")
            recs.append({"x": x, "y": y, "u": tag["u"], "v": tag["v"], "epoch": tag["epoch"], "time": time.time(), **info})
            self.draft = None                            # records changed -> the draft fit is outdated
            if self.verify["for"] == "draft":
                self.verify = {"for": None, "spots": []}
            self._save_session()
        return {"ok": True, "tag": i, "n": len(recs), "warnings": warn, "spread_mm": self._spread(i)}

    def delete_record(self, req):
        with self.lock:
            if req.get("all"):
                self.records = {}
            elif req.get("index") is None:
                self.records.pop(int(req["id"]), None)
            else:
                i, k = int(req["id"]), int(req["index"])
                recs = self.records.get(i, [])
                if not 0 <= k < len(recs):
                    raise ValueError(f"tag {i}: no record {k}")
                recs.pop(k)
                if not recs:
                    self.records.pop(i)
            self.draft = None
            self._save_session()
        return {"ok": True}

    def _spread(self, i):
        recs = self.records.get(i, [])
        if len(recs) < 2:
            return None
        m = np.mean([[r["x"], r["y"]] for r in recs], axis=0)
        return round(max(math.dist(m, (r["x"], r["y"])) for r in recs) * 2000, 2)   # ~ max distance apart

    def fit(self):
        size = self.camera.size
        if not size:
            raise ValueError("no camera image yet")
        pts, warn = [], []
        for i, recs in sorted((i, r) for i, r in self.records.items() if i in self.refs):
            tag = self.tracker.get(i)
            if tag is None:
                warn.append(f"tag {i}: not seen by the camera, skipped"); continue
            if any(r["epoch"] != tag["epoch"] for r in recs):
                warn.append(f"tag {i} moved after it was recorded: re-record it, skipped"); continue
            s = self._spread(i)
            if s is not None and s > self.c["record"]["spread_warn_mm"]:
                warn.append(f"tag {i}: records {s:.1f} mm apart (> {self.c['record']['spread_warn_mm']} mm): redo one")
            pts.append({"id": i, "u": tag["u"], "v": tag["v"], "x": float(np.mean([r["x"] for r in recs])),
                        "y": float(np.mean([r["y"] for r in recs]))})
        if len(pts) < 8:
            warn.append(f"{len(pts)} tags (8 recommended; lens term needs 6+)")
        c = calib.fit(pts, size, self.c["camera"]["name"])
        c["lens_effect_mm"] = round(calib.lens_effect_mm(c), 2)
        with self.lock:
            self.draft = c; self.verify = {"for": "draft", "spots": []}
            self._save_session()
        return {"ok": True, "calib": c, "warnings": warn}

    # ---- verification (V-19) ----
    def verify_record(self, req):
        c, which = self.active()
        if c is None:
            raise ValueError("no calibration to verify: fit first (or load a saved one)")
        if self.camera.size and list(self.camera.size) != list(c["image_size"]):
            raise ValueError(f"camera image {self.camera.size} differs from the calibration {c['image_size']}")
        vid, tag = self._moving_tag(req.get("id"))
        x, y, info, warn = self._pose(req)
        (px,), (py,), (inside,) = calib.apply(c, [tag["u"]], [tag["v"]])
        with self.lock:
            if self.verify["for"] != which:
                self.verify = {"for": which, "spots": []}
            spots = self.verify["spots"]
            if any(s.get("tag") == vid and s["epoch"] == tag["epoch"] for s in spots):
                warn.append(f"tag {vid} has not moved since the last spot")
            if any(math.dist((s["px"], s["py"]), (px, py)) * 1000 < self.c["verify"]["same_spot_mm"] for s in spots):
                warn.append("this spot is very close to an earlier one: spread the spots")
            err = math.dist((px, py), (x, y)) * 1000
            if err > 4 * self.c["verify"]["max_mm"]:
                warn.append(f"error {err:.0f} mm: is the cup on moving tag {vid}? (a removed tag counts for "
                            f"{self.c['tags']['verify_seen_s']:.0f} s) - delete this spot if not")
            spots.append({"px": px, "py": py, "x": x, "y": y, "err_mm": round(err, 2),
                          "dx_mm": round((x - px) * 1000, 2), "dy_mm": round((y - py) * 1000, 2), "inside": inside,
                          "epoch": tag["epoch"], "tag": vid, **info})
            self._save_session()
        return {"ok": True, "spot": spots[-1], "summary": self.verify_summary(), "warnings": warn}

    def _moving_tag(self, want=None):
        """The verification tag = a NON-reference tag seen recently (the cup may hide it now). (id, tracker dict)."""
        if not self.refs:
            raise ValueError("identify the reference tags first")
        now, t = time.monotonic(), self.c["tags"]
        cands = {i: g for i in self.tracker.ids() if i not in self.refs and (g := self.tracker.get(i, now))
                 and g["n"] >= t["min_samples"] and g["age_s"] < t["verify_seen_s"]}
        if want is not None:
            want = int(want)
            if want in self.refs:
                raise ValueError(f"tag {want} is a reference tag, not the moving tag")
            if want not in cands:
                raise ValueError(f"moving tag {want}: not seen long enough / recently by the camera")
            return want, cands[want]
        if not cands:
            raise ValueError("no moving tag: place a tag that is not a reference tag in view (any id)")
        if len(cands) > 1:
            raise ValueError(f"several non-reference tags {sorted(cands)}: choose one (id) or remove the others")
        return next(iter(cands.items()))

    def verify_delete(self, req):
        with self.lock:
            if req.get("all"):
                self.verify["spots"] = []
            else:
                k = int(req["index"])
                if not 0 <= k < len(self.verify["spots"]):
                    raise ValueError(f"no spot {k}")
                self.verify["spots"].pop(k)
            self._save_session()
        return {"ok": True, "summary": self.verify_summary()}

    def verify_summary(self):
        v, spots = self.c["verify"], self.verify["spots"]
        if not spots:
            return {"n": 0, "passed": False, "hint": f"record the verification tag at {v['min_spots']}+ spots"}
        e = np.array([s["err_mm"] for s in spots]); d = np.array([[s["dx_mm"], s["dy_mm"]] for s in spots])
        mean, mx = float(e.mean()), float(e.max())
        passed = len(spots) >= v["min_spots"] and mean <= v["mean_mm"] and mx <= v["max_mm"]
        hint = ""
        if not passed and len(spots) >= 3:
            shift = float(np.linalg.norm(d.mean(axis=0)))
            if shift > 0.7 * mean:
                hint = (f"errors point the same way (mean shift {shift:.1f} mm): systematic placement offset or a moved "
                        "camera, not the lens")
            else:
                far = [s for s in spots if not s["inside"]]
                hint = ("largest errors outside the tag area: add / move tags to cover it" if far and
                        max(s["err_mm"] for s in far) >= mx else "re-check the worst spot, then re-record suspect tags")
        elif len(spots) < v["min_spots"]:
            hint = f"{v['min_spots'] - len(spots)} more spot(s) needed"
        return {"n": len(spots), "mean_mm": round(mean, 2), "max_mm": round(mx, 2), "passed": passed, "hint": hint,
                "limits": {"mean_mm": v["mean_mm"], "max_mm": v["max_mm"], "min_spots": v["min_spots"]},
                "for": self.verify["for"]}

    # ---- save ----
    def save(self):
        c, which = self.active()
        if c is None:
            raise ValueError("nothing to save: fit first")
        out = {k: v for k, v in c.items() if k != "lens_effect_mm"}
        s = self.verify_summary() if self.verify["for"] == which else {"n": 0}
        if s["n"]:
            out["verification"] = {"date": time.strftime("%Y-%m-%d %H:%M:%S"), "spots": s["n"], "mean_mm": s["mean_mm"],
                                   "max_mm": s["max_mm"], "passed": s["passed"], "limits": s["limits"],
                                   "errors_mm": [sp["err_mm"] for sp in self.verify["spots"]]}
        elif "verification" in c and which == "saved":
            out["verification"] = c["verification"]
        path = calib.save(self.c["calib_file"], out)
        with self.lock:
            self.draft = None
            self.verify = {"for": "saved", "spots": self.verify["spots"] if self.verify["for"] == which else []}
            self._save_session()
        self._reload_saved()
        warn = [] if s.get("passed") or "verification" in out and out["verification"].get("passed") else \
            ["saved WITHOUT a passed verification"]
        return {"ok": True, "path": path, "warnings": warn}

    # ---- state for the page ----
    def state(self):
        now = time.monotonic()
        tags = {}
        for i in self.tracker.ids():
            t = self.tracker.get(i, now)
            if t is None or (not t["visible"] and t["n"] < self.c["tags"]["min_samples"]):
                continue                                 # short-lived ghost: not shown
            recs = self.records.get(i, [])
            t.update({"role": "ref" if i in self.refs else ("moving" if self.refs else "unassigned"),
                      "records": [{k: v for k, v in r.items() if k not in ("u", "v", "epoch")}
                                  | {"moved": r["epoch"] != t["epoch"]} for r in recs], "spread_mm": self._spread(i)})
            tags[i] = t
        for i in self.refs:
            if i not in tags:
                tags[i] = {"role": "ref", "missing": True, "records": self.records.get(i, []), "spread_mm": self._spread(i)}
        robot = None
        if self.reader:
            robot = self.reader.status()
            st, age = self.reader.latest()
            if st is not None:
                robot["pose"] = robot_pose.tcp_from_state(st, self.rcfg)
        c, which = self.active()
        with self.camera.lock:
            cam = {"status": self.camera.status, "size": self.camera.size, "fps": round(self.camera.fps, 1)}
        saved = None
        if self.saved:
            saved = {k: self.saved.get(k) for k in ("date", "rms_mm", "lens_k", "image_size", "camera", "verification")}
            saved["tags"] = len(self.saved["tags"])
        return {"camera": cam, "tags": tags, "refs": self.refs, "robot": robot, "draft": self.draft, "active": which,
                "active_tags": [[t["u"], t["v"]] for t in c["tags"]] if c else [],
                "saved": saved, "saved_error": self.saved_error, "verify": {"spots": self.verify["spots"],
                                                                            "summary": self.verify_summary()},
                "config": {"max_per_tag": self.c["record"]["max_per_tag"],
                           "spread_warn_mm": self.c["record"]["spread_warn_mm"], "verify": self.c["verify"]}}

    def pixel(self, u, v):
        c, which = self.active()
        if c is None:
            raise ValueError("no calibration (fit or save one first)")
        (x,), (y,), (inside,) = calib.apply(c, [u], [v])
        return {"ok": True, "x": x, "y": y, "inside": inside, "using": which}


def make_app(app):
    def handler(fn, parse=True):
        async def h(request):
            try:
                body = await request.json() if parse and request.can_read_body else {}
                res = await asyncio.get_running_loop().run_in_executor(None, fn, body) if parse else fn(request)
                return web.json_response(_jsonable(res))
            except (ValueError, KeyError, TypeError) as e:
                return web.json_response({"ok": False, "message": str(e)})
        return h

    async def index(_):
        p = os.path.join(SHARE, "web", "index.html")
        if os.path.exists(p):
            return web.FileResponse(p, headers={"Cache-Control": "no-store"})
        return web.Response(text="calib_app: page not installed (V7); API at /api/state", content_type="text/plain")

    async def stream(request):
        resp = web.StreamResponse(headers={"Content-Type": "multipart/x-mixed-replace; boundary=frame",
                                           "Cache-Control": "no-store"})
        await resp.prepare(request)
        period = 1.0 / app.c["camera"]["stream_fps"]
        try:
            while True:
                with app.camera.lock:
                    jpeg = app.camera.jpeg
                if jpeg:
                    await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
                await asyncio.sleep(period)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return resp

    async def snapshot(_):
        with app.camera.lock:
            jpeg = app.camera.jpeg
        return web.Response(body=jpeg or b"", content_type="image/jpeg", headers={"Cache-Control": "no-store"})

    def pixel(request):
        try:
            u, v = float(request.query["u"]), float(request.query["v"])
        except (KeyError, ValueError):
            raise ValueError("u and v must be numbers")
        return app.pixel(u, v)

    w = web.Application()
    w.add_routes([web.get("/", index), web.get("/stream", stream), web.get("/snapshot", snapshot),
                  web.get("/api/state", handler(lambda _: app.state(), parse=True)),
                  web.get("/api/pixel", handler(pixel, parse=False)),
                  web.post("/api/refs/identify", handler(app.identify)), web.post("/api/refs/remove", handler(app.remove_ref)),
                  web.post("/api/record", handler(app.record)), web.post("/api/record/delete", handler(app.delete_record)),
                  web.post("/api/fit", handler(lambda _: app.fit())),
                  web.post("/api/verify/record", handler(app.verify_record)),
                  web.post("/api/verify/delete", handler(app.verify_delete)),
                  web.post("/api/save", handler(lambda _: app.save()))])
    web_dir = os.path.join(SHARE, "web")
    if os.path.isdir(web_dir):
        w.add_routes([web.static("/web", web_dir)])
    return w


def _jsonable(o):
    if isinstance(o, dict):
        return {str(k): _jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_jsonable(v) for v in o]
    if isinstance(o, (np.floating, np.integer)):
        o = o.item()
    if isinstance(o, float) and not math.isfinite(o):
        return None
    return o


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=os.path.join(SHARE, "config", "calib_app.yaml"))
    ap.add_argument("--image", help="use this image file instead of the camera (re-read when it changes)")
    ap.add_argument("--no-robot", action="store_true", help="no port 20004 reader (pendant / manual values only)")
    ap.add_argument("--port", type=int, help="override the config port")
    ap.add_argument("--camera", help="override camera.device: index (2 = /dev/video2) or a device path")
    a = ap.parse_args()
    cfg = load_app_config(a.config)
    if a.camera is not None:
        cfg["camera"]["device"] = int(a.camera) if a.camera.isdigit() else a.camera
    app = App(cfg, a.image, not a.no_robot)
    app.start()
    port = a.port or cfg["port"]
    print(f"calib_app: http://{cfg['host']}:{port}  (calibration {cfg['calib_file']}, session {cfg['session_file']})",
          flush=True)
    try:
        web.run_app(make_app(app), host=cfg["host"], port=port, print=None)
    finally:
        app.stop()
