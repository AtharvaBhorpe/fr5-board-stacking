"""Pure maths for the skills (T4, control-loop.md section 4). No ROS. base_link, metres / radians.

tcp orientation: MoveTo MODE_DOWN_YAW, R = Rz(yaw) * Rx(pi): tcp x axis = (cos yaw, sin yaw) on the table. A board
is symmetric under 180 deg, so the tcp yaw for a board is board_yaw or board_yaw + pi: we take the one closest to the
reference yaw (home pose 'ready' = +pi/2), so wrist 3 (j6) never has to twist far. With tcp yaw = board yaw (mod pi)
the board's long side lies along tcp x -> attach yaw 0.
"""
import math

NAN = float("nan")


def wrap_pi(a):
    """-> (-pi, pi]."""
    a = math.fmod(a + math.pi, 2 * math.pi)
    if a <= 0:
        a += 2 * math.pi
    return a - math.pi


def tcp_yaw_for(board_yaw, ref_yaw):
    """tcp yaw that aligns tcp x with the board's long side, closest to ref_yaw (180 deg symmetry)."""
    a, b = wrap_pi(board_yaw), wrap_pi(board_yaw + math.pi)
    return a if abs(wrap_pi(a - ref_yaw)) <= abs(wrap_pi(b - ref_yaw)) else b


def pick_heights(cfg, board):
    """(hover_z, descent_end_z, contact_search). Known top (zone boards): descend to top - press. Unknown top (input
    stacks, NaN, CL-16): fast to the search start above the tallest allowed input stack, then search slowly down to
    the table - press (the vacuum seal stops it)."""
    hover = cfg["hover_z"]
    if board.get("top_z") is None or math.isnan(board["top_z"]):
        start = cfg["table_z"] + cfg["max_input_stack_height"] + cfg["search_clear"]
        return hover, start, True
    return hover, board["top_z"] - cfg["press"], False


def search_end_z(cfg):
    """Lowest tcp z of a contact search: table top - press (an empty spot seals on the table)."""
    return cfg["table_z"] - cfg["press"]


def place_target(cfg, zone, thickness):
    """(x, y, zone_yaw, tcp_z) for placing a board of this thickness on a zone (dict like Zone.msg).
    processing: on the zone sheet (base_z, V4); drop_off: on top of the stack. tcp z = board top - place_press (TM-25).
    fault zone (CL-39, a void): released from the fixed height fault_drop_z (no descent, the pile does not matter)."""
    if zone["kind"] == "fault":
        return zone["x"], zone["y"], zone["yaw"], cfg["fault_drop_z"]
    surface = zone["base_z"] if zone["kind"] == "processing" else zone["stack_top_z"]
    return zone["x"], zone["y"], zone["yaw"], surface + thickness - cfg["place_press"]


def stack_box(cfg, zone, boards, skip=""):
    """W13 / CL-38: ONE collision box per drop-off / fault stack, rebuilt from the world (no per-board bookkeeping).
    zone: dict like Zone.msg; boards: dicts like Board.msg. Height = stack top (world: count x variant thickness) minus
    the board being picked (`skip`) minus stack_box_margin (the cup may touch the top board); footprint = the largest
    board on it + stack_box_pad per side, aligned with the zone (boards are placed along the zone yaw). The box stands on
    the table (it covers the zone sheet too); no boards above the sheet (base_z, V4) -> no box.
    Returns {"size": [x, y, z], "position": [x, y, z], "yaw"} or None (no box: processing / fault zone, empty stack)."""
    if zone["kind"] != "drop_off":                   # fault zone: a void, no box (CL-39)
        return None
    on = [b for b in boards if b["location"] == "zone" and b["zone"] == zone["name"]]
    top = zone["stack_top_z"]
    sk = next((b for b in on if b["id"] == skip), None)
    if sk is not None:
        top -= sk["thickness"] if sk["thickness"] == sk["thickness"] else cfg["default_thickness"]   # NaN-safe
    if top - zone["base_z"] <= 0.001:                # no board left on the sheet
        return None
    h = top - cfg["table_z"] - cfg["stack_box_margin"]
    if h <= 0.001:
        return None
    l = max([b["length"] for b in on if b["id"] != skip] or [cfg["stack_box_default"][0]])
    w = max([b["width"] for b in on if b["id"] != skip] or [cfg["stack_box_default"][1]])
    pad = cfg["stack_box_pad"]
    return {"size": [l + 2 * pad, w + 2 * pad, h], "position": [zone["x"], zone["y"], cfg["table_z"] + h / 2],
            "yaw": zone["yaw"]}


def zone_accepts(cfg, zone):
    """'' if a board may be placed in this zone now, else the SkillCode name of the reason."""
    if zone["kind"] == "processing":
        return "" if zone["state"] == "EMPTY" else "TARGET_OCCUPIED"
    if zone["kind"] == "fault":
        return ""                                    # CL-39: a void, never full
    if zone["state"] == "FULL" or zone["stack_count"] >= cfg["max_stack"]:
        return "STACK_FULL"
    return ""


def board_at(world_boards, x, y, tol, variant=None):
    """First board within tol of (x, y) (optionally of a variant) in a list of Board-like dicts, else None."""
    best = None
    for b in world_boards:
        d = math.hypot(b["x"] - x, b["y"] - y)
        if d <= tol and (variant is None or b["variant"] == variant) and (best is None or d < best[0]):
            best = (d, b)
    return best[1] if best else None
