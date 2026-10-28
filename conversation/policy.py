"""Bounded intent and evidence checks, independent of ROS and model inference."""
import math
import re
from dataclasses import dataclass

VARIANTS = ("VARa", "VARb", "VARc")


@dataclass(frozen=True)
class Intent:
    kind: str
    variant: str = ""


def intent(text: str) -> Intent:
    text = re.sub(r"[.!?]+$", "", text.strip().lower()).strip()
    if text in {"stop", "stop robot", "stop the robot", "cancel", "cancel pick", "stop please",
                "cancel batch", "stop batch", "cancel queue", "stop sorting"}:
        return Intent("stop")
    if text in {"status", "robot status", "what is the status", "what's the status", "batch status", "queue status"}:
        return Intent("status")
    if text in {"clear chat", "clear conversation"}:
        return Intent("clear")
    match = re.fullmatch(
        r"(?:please |can you |could you )?(?:pick(?: up| and place)?|pick up and place) "
        r"(?:one |a |the )?(?:var\s*|variant\s+)([abc])(?: board)?(?: please)?", text)
    if match:
        return Intent("pick", "VAR" + match[1])
    # Questions and ambiguous commands may be discussed, but never generate motion.
    return Intent("chat")


def block_reason(scene: dict, state: dict, now: float, variant: str, allow_replay=False) -> str:
    if variant not in VARIANTS:
        return "Unsupported variant. Choose VARa, VARb, or VARc."
    try:
        ages = [now - float(obj["stamp"]) for obj in (scene, state)]
    except (KeyError, ValueError, TypeError):
        return "Waiting for timestamped vision and controller status."
    if any(not math.isfinite(age) or not 0 <= age <= 2 for age in ages):
        return "Vision or controller status is stale; no pick requested."
    hardware = state.get("hardware")
    if hardware not in {"SIMULATION", "REAL FR5"}:
        return "Robot hardware identity is not known."
    if not isinstance(state.get("busy"), bool):
        return "Controller activity status is invalid."
    if state.get("busy"):
        return "A pick is already running. I will not queue another."
    if scene.get("calibrated") is not True:
        return "The overhead camera is not calibrated."
    if scene.get("live") is not True and not (hardware == "SIMULATION" and allow_replay):
        return "This is replay or unverified camera data; real motion is disabled."
    boards, destinations = scene.get("boards"), scene.get("destinations")
    if not isinstance(boards, list) or not isinstance(destinations, list) or any(
        not isinstance(item, dict) for item in boards + destinations
    ):
        return "Vision data is invalid; no pick requested."
    if not any(b.get("variant") == variant for b in boards):
        return f"No freely pickable {variant} is detected. Check the scene before requesting it again."
    if not any(d.get("variant") == variant for d in destinations):
        return f"The destination for {variant} is unavailable."
    return ""


def visual_agreement(result: dict) -> bool:
    return result.get("target_visible") is True and result.get("conflict") is False


def boundary_reply(text: str):
    """Keep explanations of the command boundary as deterministic as dispatch itself."""
    words = text.strip().lower()
    if re.search(r"\b(why did|did you|have you)\b.*\b(pick|place|move|finish)", words):
        return "I cannot infer task history or a selection reason from the current image. Ask status for the current controller state, and check the task log for earlier actions."
    words = re.sub(r"^(please |can you |could you )", "", words)
    if re.match(r"^(don't|do not|never)\b", words):
        return "No pick requested. To cancel a running task, say stop or use the Stop button."
    if re.match(r"^(confirm|approve|continue)\b", words):
        return "No confirmation is needed. An accepted pick request runs through pick, placement, and release automatically. Say stop to cancel."
    if re.match(r"^(move|jog|release|grip|rotate|turn|set speed)\b", words):
        return "I support one pick by variant, status, scene questions, and stop. Use the existing robot controls for other actions."
    if re.match(r"^(pick|stack|grab)\b", words):
        if re.search(r"\b(two|three|four|five|all|both|[2-9]|\d{2,})\b|\band\b", words):
            return "Use 'pick two variant B boards' or 'sort all visible boards'. I execute one pick at a time."
        return "Which variant do you mean: A, B, or C? Say 'pick variant B', for example. No pick requested."
    return None
