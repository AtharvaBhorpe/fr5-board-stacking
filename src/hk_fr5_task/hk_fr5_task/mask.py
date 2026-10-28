"""Robot mask on the table plane (CL-31): where the overhead camera cannot see the table because the robot is in the way.
Pure geometry, base_link x / y, metres. No ROS.

A mask is {"arm": [shape, ...], "tool": [shape, ...]}:
  arm  - the robot links. Only EXPLAINS ABSENCE: a board not seen there is frozen (not lost), a processing zone with
         nothing seen there keeps its state. A detection there is still used if it matches a known board and shows
         its full size (world_core decides).
  tool - the HELD BOARD only: detections there are always ignored (the camera sees the robot's own board as a
         board). With nothing held the gripper is part of 'arm' (a board partly under it is a cut-off view).
A shape is {"kind": "capsule", "a": [x, y], "b": [x, y], "r": r}  (segment a-b grown by r; a == b: a circle)
        or {"kind": "poly", "pts": [[x, y], ...], "r": r}       (convex polygon grown by r).
"""
import math


def capsule(a, b, r):
    return {"kind": "capsule", "a": [float(a[0]), float(a[1])], "b": [float(b[0]), float(b[1])], "r": float(r)}


def poly(pts, r=0.0):
    return {"kind": "poly", "pts": [[float(x), float(y)] for x, y in pts], "r": float(r)}


def _seg_point(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    k = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + k * dx), py - (ay + k * dy))


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _segs_cross(p1, p2, q1, q2):
    d1, d2, d3, d4 = _cross(q1, q2, p1), _cross(q1, q2, p2), _cross(p1, p2, q1), _cross(p1, p2, q2)
    return (d1 > 0) != (d2 > 0) and (d3 > 0) != (d4 > 0)


def inside_poly(p, pts):
    """Point inside a convex polygon (either winding); on the edge counts as inside; zero area: never."""
    sign = 0
    n = len(pts)
    for i in range(n):
        c = _cross(pts[i], pts[(i + 1) % n], p)
        if abs(c) < 1e-15:
            continue
        s = 1 if c > 0 else -1
        if sign and s != sign:
            return False
        sign = s
    return sign != 0                     # all points on one line (zero-area polygon): not 'inside', distance decides


def _edges(pts):
    return [(pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))] if len(pts) > 2 else [(pts[0], pts[-1])]


def distance(pts_a, pts_b):
    """Distance between two convex point sets (polygon >= 3 points, segment = 2 points, point = 1). 0 if they overlap."""
    if len(pts_b) >= 3 and any(inside_poly(p, pts_b) for p in pts_a):
        return 0.0
    if len(pts_a) >= 3 and any(inside_poly(p, pts_a) for p in pts_b):
        return 0.0
    ea, eb = _edges(pts_a), _edges(pts_b)
    if any(_segs_cross(p1, p2, q1, q2) for p1, p2 in ea for q1, q2 in eb):
        return 0.0
    d = min(_seg_point(p[0], p[1], q1[0], q1[1], q2[0], q2[1]) for p in pts_a for q1, q2 in eb)
    return min(d, min(_seg_point(p[0], p[1], q1[0], q1[1], q2[0], q2[1]) for p in pts_b for q1, q2 in ea))


def _shape_pts(s):
    return [s["a"], s["b"]] if s["kind"] == "capsule" else s["pts"]


def touches(shape, pts):
    """The shape (grown by its r) touches / overlaps the convex polygon `pts` (e.g. a board's 4 corners) or point."""
    return distance(_shape_pts(shape), [list(p) for p in pts]) <= shape["r"]


def hits(shapes, pts):
    return any(touches(s, pts) for s in shapes or [])


def band(tcp_xy, arm_radius, held_radius=0.0, base=(0.0, 0.0)):
    """Transitional mask (until the link mask from TF, CL-31 / step 7): the CL-20 B band base -> tcp as 'arm'; while
    a board is held (held_radius > 0) a circle of that radius at the tcp as 'tool'."""
    return {"arm": [capsule(base, tcp_xy, arm_radius)],
            "tool": [capsule(tcp_xy, tcp_xy, held_radius)] if held_radius > 0 else []}
