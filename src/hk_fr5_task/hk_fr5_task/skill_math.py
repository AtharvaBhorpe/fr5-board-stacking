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
    processing: on the table; drop_off / fault: on top of the stack. tcp z = board top + gap (released just above)."""
    surface = cfg["table_z"] if zone["kind"] == "processing" else zone["stack_top_z"]
    return zone["x"], zone["y"], zone["yaw"], surface + thickness + cfg["place_gap"]


def zone_accepts(cfg, zone):
    """'' if a board may be placed in this zone now, else the SkillCode name of the reason."""
    if zone["kind"] == "processing":
        return "" if zone["state"] == "EMPTY" else "TARGET_OCCUPIED"
    if zone["state"] == "FULL" or zone["stack_count"] >= cfg["max_stack"]:
        return "STACK_FULL"
    return ""


def in_shadow(px, py, tcp_x, tcp_y, radius, base=(0.0, 0.0)):
    """Point inside the arm's shadow on the table (CL-20 B): a capsule of `radius` around the segment robot base ->
    tcp (seen from the overhead camera the arm covers roughly this band)."""
    bx, by = base
    dx, dy = tcp_x - bx, tcp_y - by
    L2 = dx * dx + dy * dy
    k = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((px - bx) * dx + (py - by) * dy) / L2))
    return math.hypot(px - (bx + k * dx), py - (by + k * dy)) <= radius


def board_at(world_boards, x, y, tol, variant=None):
    """First board within tol of (x, y) (optionally of a variant) in a list of Board-like dicts, else None."""
    best = None
    for b in world_boards:
        d = math.hypot(b["x"] - x, b["y"] - y)
        if d <= tol and (variant is None or b["variant"] == variant) and (best is None or d < best[0]):
            best = (d, b)
    return best[1] if best else None
