"""Vision 1 /vision/boards JSON -> world_core detections (V3, decision-log TM-22). Pure, no ROS.

Schema = the agreed change request notes/vision1-changes.md (NOT the older vision-branch output):
    stamp (capture time, Unix s), calibrated, boards[] {variant, corners_xy, conf?, inside_calibration},
    zones[] {name, kind, x, y, yaw, age, size [long, short], top_z}.
Optional (TM-24): obstacles[] {label, corners_xy} - the LATER item of the change request; Vision 1 does not send it
yet, the fake world does (hands), so the obstacle stop works the same way once it arrives.
Ignored: occluded[], rejected[], destinations[], image_size, source, boards[].id / u / v / corners / z / yaw / zone,
zones[].frames / tag_id / age (zone memory = Vision 1's job, C8: a listed zone counts as seen).
"""
from hk_fr5_task.world_core import rect_corners

ZONE_KINDS = ("drop_off", "processing")      # the fault zone is ours (config), never from Vision 1


def parse(data):
    """data: the decoded JSON dict. Returns (stamp, dets, problems).
    dets: world_core.update() detections; problems: one text per skipped board / zone (bad or missing field).
    Raises ValueError for a frame that cannot be used at all (not calibrated, no stamp)."""
    if not isinstance(data, dict):
        raise ValueError("not a JSON object")
    if not data.get("calibrated", False):
        raise ValueError("Vision 1 not calibrated (no base_link coordinates)")
    try:
        stamp = float(data["stamp"])
    except (KeyError, TypeError, ValueError):
        raise ValueError("no valid 'stamp'")
    dets, problems = [], []
    for k, b in enumerate(data.get("boards") or []):
        try:
            cs = [[float(p[0]), float(p[1])] for p in b["corners_xy"]]
            if len(cs) != 4:
                raise ValueError(f"{len(cs)} corners_xy (need 4)")
            conf = b.get("conf")
            dets.append({"cls": f"board/{b['variant']}", "id": "", "score": 1.0 if conf is None else float(conf),
                         "corners": cs, "inside": bool(b.get("inside_calibration", True))})
        except (KeyError, TypeError, ValueError, IndexError) as e:
            problems.append(f"board {k}: {type(e).__name__} {e}")
    for k, z in enumerate(data.get("zones") or []):
        try:
            name, kind = str(z["name"]), z["kind"]
            if not name:
                raise ValueError("empty name")
            if kind not in ZONE_KINDS:
                raise ValueError(f"kind '{kind}' (expected {' / '.join(ZONE_KINDS)})")
            length, width = (float(v) for v in z["size"])
            dets.append({"cls": f"zone/{kind}", "id": name, "score": 1.0, "inside": True, "z": float(z["top_z"]),
                         "corners": rect_corners(float(z["x"]), float(z["y"]), float(z["yaw"]), length, width)})
        except (KeyError, TypeError, ValueError) as e:
            problems.append(f"zone {z.get('name', k) if isinstance(z, dict) else k}: {type(e).__name__} {e}")
    for k, o in enumerate(data.get("obstacles") or []):
        try:
            cs = [[float(p[0]), float(p[1])] for p in o["corners_xy"]]
            if len(cs) != 4:
                raise ValueError(f"{len(cs)} corners_xy (need 4)")
            dets.append({"cls": f"obstacle/{o.get('label') or 'unknown'}", "id": "", "score": 1.0, "corners": cs})
        except (KeyError, TypeError, ValueError, IndexError) as e:
            problems.append(f"obstacle {k}: {type(e).__name__} {e}")
    return stamp, dets, problems
