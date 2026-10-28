"""Free-form request -> main-controller commands. Gemma (llama.cpp, conversation/llm.sh) proposes; every command is repaired
(variant / zone spelling, a missing processing zone) and then must pass the fixed grammar (policy.intent + arity +
known zones). The controller validates each job again. No ROS here: conversation/plan_check.py tests it offline.
"""
import json
import os
import re
import urllib.request

from policy import intent

LLM = os.environ.get("PLANNER_LLM", "http://127.0.0.1:8770/v1/chat/completions")  # any OpenAI-style chat API
MAX_COMMANDS = 16  # ponytail: one plan = at most 16 jobs; the controller queue itself has no limit
PROMPT = """You turn a spoken or typed request into jobs for the main controller of a robot cell that moves boards.
Reply with JSON only: {"reason": "<one or two short sentences>", "commands": ["<command>", ...]}

COMMANDS (exact syntax, one job per command, sent in this order):
  transfer VARIANT ZONE COUNT PRIO        input-table boards of VARIANT -> ZONE
  process VARIANT PZONE ZONE COUNT PRIO   board -> processing zone PZONE; once processed the controller moves it to ZONE
  clear FROM_ZONE TO_ZONE COUNT PRIO      boards already on zone FROM_ZONE -> TO_ZONE
The robot parks at home by itself when the queue is empty: "go home after" needs no command; never plan pause / resume /
home / stop (the user says those directly).
VARIANT: VARa = large / big / wide white board; VARb = small / narrow white board; VARc = red board; any = any board.
ZONE: a drop_off zone from STATE, or fault (= reject / scrap / bin / trash / bad parts). PZONE: a processing zone.
COUNT: number of boards the user says; 1 when not said; all / every / everything = 0 (never count the boards in
STATE yourself); -1 = keep going / continuously until cancelled.
PRIO: higher runs first. urgent / asap / first / high = 10, medium / normal = 5, low / last / when there is time = 1,
no priority word = 0 (never invent one). Each job gets only the priority said in its own part of the request:
"urgent: X, after that Y" = X 10, Y 0. Jobs with the same PRIO run in the order of the list.

RULES
- One command per variant + destination the user names; never merge, never skip a part of the request.
- "process" only for boards the user says to process / machine / treat; it already includes the move to ZONE, so never
  add a transfer for the same boards. No ZONE said for a processed board: use the zone named next, else the first
  drop_off zone.
- "first X, then Y, then Z" without priorities: keep that order in the list, PRIO 0.
- "empty / clear out / everything from ZONE (to / into TO)" = clear ZONE TO 0 (TO not said: ask, no commands).
- Zone names: "drop off a", "drop of a", "tray a", "area a", "Ablage A" = drop_off_a.
- Use only zones from STATE. Unsafe (jog, release vacuum, move joints, go faster) or impossible: no commands, say why.
"""

EXAMPLES = [  # few-shot as earlier chat turns (request, reason, commands): a small model copies these far better than rules
    ("move the red board to drop off a", "One red board (VARc) to drop_off_a.", ["transfer VARc drop_off_a 1 0"]),
    ("put all the large boards on drop off b", "All large white boards (VARa) to drop_off_b; COUNT 0 means all.",
     ["transfer VARa drop_off_b 0 0"]),
    ("two small white boards to tray B and all big ones to tray A",
     "Two VARb to drop_off_b, then every VARa to drop_off_a.",
     ["transfer VARb drop_off_b 2 0", "transfer VARa drop_off_a 0 0"]),
    ("process variant C then drop off A with high priority, variant B to drop off B medium priority, variant C to the "
     "fault zone low priority",
     "Red board through processing_1 to drop_off_a first (high), then VARb to drop_off_b (medium), then a red board "
     "to the fault zone (low).",
     ["process VARc processing_1 drop_off_a 1 10", "transfer VARb drop_off_b 1 5", "transfer VARc fault 1 1"]),
    ("first one A to drop off B, then process a red one and put it on A, after that throw a B into the bin",
     "Three jobs in the order you said: VARa to drop_off_b, a red board processed then to drop_off_a, a VARb to the "
     "fault zone.",
     ["transfer VARa drop_off_b 1 0", "process VARc processing_1 drop_off_a 1 0", "transfer VARb fault 1 0"]),
    ("urgent: process three large boards to drop off b. the small ones can go to drop off a whenever",
     "Three VARa through processing to drop_off_b at high priority; all VARb to drop_off_a at low priority.",
     ["process VARa processing_1 drop_off_b 3 10", "transfer VARb drop_off_a 0 1"]),
    ("could you bring two of the big ones over to tray b, and get rid of a red one",
     "Two VARa to drop_off_b; one red board to the fault zone (get rid of = fault).",
     ["transfer VARa drop_off_b 2 0", "transfer VARc fault 1 0"]),
    ("keep moving red boards to drop off b", "Red boards to drop_off_b continuously until you cancel.",
     ["transfer VARc drop_off_b -1 0"]),
    ("move everything from drop off a to drop off b", "Every board on drop_off_a to drop_off_b.",
     ["clear drop_off_a drop_off_b 0 0"]),
    ("Verarbeite eine rote Platte und lege sie auf Ablage B, dann zwei kleine weisse auf Ablage A",
     "Eine rote Platte über processing_1 auf drop_off_b, dann zwei VARb auf drop_off_a.",
     ["process VARc processing_1 drop_off_b 1 0", "transfer VARb drop_off_a 2 0"]),
    ("release the vacuum and move down 5 cm", "I do not jog the arm or switch the vacuum; only board jobs.", []),
]


def repair(command, zones):
    """Fix what a small model gets wrong in spelling, never in meaning: case, a missing processing zone."""
    a = command.strip().split()
    if not a:
        return ""
    a[0] = a[0].lower()
    a = [re.sub(r"(?i)^var([abc])$", lambda m: "VAR" + m[1].lower(), x) for x in a]
    a = [x.lower() if re.fullmatch(r"(?i)(drop_off_[a-z]|processing_\d+|fault)", x) else x for x in a]
    a = ["fault" if re.fullmatch(r"(?i)(fault_?zone|bin|trash|scrap|rejects?|waste|garbage|ausschuss|m(ü|ue)ll)", x) else x
         for x in a]
    proc = [z["name"] for z in zones if z.get("kind") == "processing"]
    if a[0] == "process" and len(a) >= 3 and a[2] not in proc and len(proc) == 1:
        a.insert(2, proc[0])  # "process VARc drop_off_a 1 10" -> "process VARc processing_1 drop_off_a 1 10"
    return " ".join(a)


def valid(command, zones):
    """Exact arity, only zones that exist (the controller checks variants, counts, everything else)."""
    names = {z["name"] for z in zones}
    kind = {z["name"]: z.get("kind") for z in zones}
    a = command.split()
    num = lambda xs: all(re.fullmatch(r"-?\d+", x) for x in xs)  # noqa: E731
    return ((a[0] == "transfer" and len(a) in (4, 5) and a[2] in names and kind[a[2]] != "processing" and num(a[3:])) or
            (a[0] == "process" and len(a) in (5, 6) and kind.get(a[2]) == "processing" and a[3] in names
             and num(a[4:])) or
            (a[0] == "clear" and len(a) in (3, 4, 5) and a[1] in names and a[2] in names and num(a[3:])))


PROCESS_WORD = re.compile(r"\b(process|machin|treat|verarbeit|bearbeit)\w*", re.I)
CLEAR_WORD = re.compile(r"\b(from|empty|clear|out of|von|leer|räum|raeum)\w*", re.I)
PRIO_WORD = re.compile(r"priorit|urgent|asap|as soon as possible|immediately|right away|\b(high|medium|normal|low)\b|"
                       r"whenever|when there is time|important|wichtig|dringend|sofort|\b(hoch|hohe[rn]?|mittel|niedrig)", re.I)


ALL = re.compile(r"\b(?:all|every|alle[ns]?)\b(?: the| of the| die| der)?(?: (?:variant|var|variante))? ?"
                 r"(a|b|c|large|big|wide|gro\w*|small|narrow|klein\w*|red|rot\w*)\b", re.I)
WORD_VAR = {"a": "VARa", "large": "VARa", "big": "VARa", "wide": "VARa", "gro": "VARa", "b": "VARb", "small": "VARb",
            "narrow": "VARb", "klein": "VARb", "c": "VARc", "red": "VARc", "rot": "VARc"}


# "process 3 red boards", "process variant C", "verarbeite eine rote Platte": the variant right after a process word
PROC_VAR = re.compile(r"(?i:\b(?:process\w*|machin\w*|treat\w*|verarbeit\w*|bearbeit\w*))(?:\W+\w+){0,3}?\W+"
                      r"(?:(?i:variante?|var)\W*(?P<l>[a-cA-C])\b|(?P<w>(?i:large|big|wide|gro\w*|small|narrow|klein\w*|red|rot\w*))\b"
                      r"|(?P<u>[ABC])\b)")


VAR_IN = re.compile(r"(?i:variante?|var)\W*(?P<l>[a-c])\b|\b(?P<w>large|big|wide|gro\w*|small|narrow|klein\w*|red|rot\w*)\b", re.I)


def clause_prio(variant, text):
    """True/False: the one phrase naming this variant has / has no priority word; None: not exactly one such phrase."""
    clauses = []
    for c in re.split(r"[,.;]|\band\b|\bund\b", text):  # a phrase naming no board ("that's urgent") joins the one before
        if clauses and not VAR_IN.search(c):
            clauses[-1] += " " + c
        else:
            clauses.append(c)
    hits = [c for c in clauses
            if any(("VAR" + m["l"].lower()) == variant if m["l"] else
                   WORD_VAR.get(m["w"].lower(), WORD_VAR.get(m["w"].lower()[:3], WORD_VAR.get(m["w"].lower()[:5]))) == variant
                   for m in VAR_IN.finditer(c))]
    return bool(PRIO_WORD.search(hits[0])) if len(hits) == 1 else None


def ground(commands, text):
    """Hold the model to what the user said (small models add things): no priority word in the request -> every PRIO 0;
    more process jobs than process words -> the later ones become plain transfers."""
    out, left = [], len(PROCESS_WORD.findall(text))
    said = [WORD_VAR.get(w.lower(), WORD_VAR.get(w.lower()[:3], WORD_VAR.get(w.lower()[:5]))) if w else "VAR" + (l or u).lower()
            for l, w, u in ((m["l"], m["w"], m["u"]) for m in PROC_VAR.finditer(text))]
    planned = [c.split()[1] for c in commands if c.startswith("process ")]
    for v in [v for v in said if v and v not in planned]:  # said "process" for v, model wrote a transfer: process it
        i = next((i for i, c in enumerate(commands) if c.split()[:2] == ["transfer", v] and c.split()[2] != "fault"), None)
        if i is not None:  # "process VAR ZONE ..." -> repair() inserts the processing zone
            commands = commands[:i] + ["process " + commands[i].split(" ", 1)[1]] + commands[i + 1:]
    for c in commands:
        a = c.split()
        if a[0] == "clear" and not CLEAR_WORD.search(text):
            continue  # boards ON a zone only when the user names a source zone; the rest is an input-table transfer
        if a[0] == "process":
            if left:
                left -= 1
            else:  # ponytail: counts words, not boards; "process A and C" needs two process words to process both
                a = ["transfer"] + a[1:2] + a[3:]
        alls = {WORD_VAR.get(m.lower(), WORD_VAR.get(m.lower()[:3], WORD_VAR.get(m.lower()[:5]))) for m in ALL.findall(text)}
        if a[0] in ("transfer", "process") and a[1] in alls:  # "all variant c" -> COUNT 0 (= all)
            a[3 if a[0] == "transfer" else 4] = "0"
        if (not PRIO_WORD.search(text) or clause_prio(a[1], text) is False) and a[0] in ("transfer", "process", "clear") and len(a) == {"transfer": 5, "process": 6, "clear": 5}[a[0]]:
            a[-1] = "0"
        out.append(" ".join(a))
    return out


def plan(text, zones, boards=(), jobs=(), timeout=90):
    """-> (reason, commands, rejected). zones: [{"name", "kind"}] incl. fault. Raises on no model / bad JSON."""
    # variants only, no counts: with counts a small model turns "all" into a number
    state = {"zones": [{"name": z["name"], "kind": z.get("kind")} for z in zones],
             "variants_on_table": sorted({b.get("variant") for b in boards if b.get("variant")}),
             "jobs": list(jobs)}
    req = urllib.request.Request(LLM, json.dumps({
        "messages": [{"role": "system", "content": PROMPT + "\nSTATE: " + json.dumps(state)}] +
                    [m for t, r, c in EXAMPLES for m in ({"role": "user", "content": t}, {"role": "assistant",
                     "content": json.dumps({"reason": r, "commands": c})})] +
                    [{"role": "user", "content": text}],
        "temperature": float(os.environ.get("PLANNER_TEMP", 0)), "max_tokens": 900, "response_format": {"type": "json_object"}}).encode(),
        {"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        out = json.loads(json.load(r)["choices"][0]["message"]["content"])
    reason = str(out.get("reason", ""))[:400]
    commands, rejected = [], []
    for c in [repair(c, zones) for c in ground([repair(str(c), zones) for c in out.get("commands", [])][:MAX_COMMANDS], text)]:
        i = intent(c, zones)
        (commands if i.kind == "command" and valid(i.command, zones) else rejected).append(i.command or c)
    return reason, commands, rejected


if __name__ == "__main__":  # offline self-check of repair / valid (no model)
    Z = [{"name": n, "kind": k} for n, k in [("drop_off_a", "drop_off"), ("drop_off_b", "drop_off"),
                                              ("processing_1", "processing"), ("fault", "fault")]]
    assert repair("process VARC Drop_Off_A 1 10", Z) == "process VARc processing_1 drop_off_a 1 10"
    assert repair("transfer vara drop_off_b 0 1", Z) == "transfer VARa drop_off_b 0 1"
    assert repair("transfer VARb trash 1 0", Z) == "transfer VARb fault 1 0"
    assert valid("transfer VARc fault 1 1", Z) and valid("transfer VARc drop_off_b -1 0", Z)
    assert not valid("transfer VARc processing_1 1", Z) and not valid("process VARb drop_off_b 1 5", Z)
    assert not valid("transfer VARa drop_off_x 1", Z) and not valid("transfer VARa drop_off_a one", Z)
    assert valid("clear drop_off_a drop_off_b 0 0", Z) and not valid("home", Z)  # control: direct commands only
    assert ground(["process VARa processing_1 drop_off_b 2 10"], "process two a to drop off b") == \
        ["process VARa processing_1 drop_off_b 2 0"]
    assert ground(["process VARc processing_1 drop_off_a 1 10", "process VARc processing_1 fault 1 1"],
                  "process C to A high priority, C to fault low priority") == \
        ["process VARc processing_1 drop_off_a 1 10", "transfer VARc fault 1 1"]
    assert ground(["transfer VARc drop_off_b 1 0", "transfer VARa drop_off_a 1 0"], "all variant c to b, one a to a") == \
        ["transfer VARc drop_off_b 0 0", "transfer VARa drop_off_a 1 0"]
    assert ground(["transfer VARc drop_off_b 2"], "alle roten Platten auf Ablage B") == ["transfer VARc drop_off_b 0"]
    assert ground(["clear drop_off_a drop_off_b 0 0"], "put all the red boards on drop off b") == []
    t = "Process 3 red boards to drop off A with high priority, then two big white boards to drop off B"
    assert [repair(c, Z) for c in ground(["transfer VARc drop_off_a 3 10", "transfer VARa drop_off_b 2 0"], t)] == \
        ["process VARc processing_1 drop_off_a 3 10", "transfer VARa drop_off_b 2 0"]
    assert ground(["transfer VARa drop_off_b 1 0"], "process a red board, then a big one to b") == ["transfer VARa drop_off_b 1 0"]
    assert ground(["transfer VARc drop_off_b 1 0"], "verarbeite eine rote Platte, Ablage B")[0].startswith("process VARc")
    assert ground(["transfer VARa drop_off_b 2 5", "transfer VARb fault 3 1"],
                  "red to A with high priority, then two big white boards to B, and scrap 3 small board with low priority") \
        == ["transfer VARa drop_off_b 2 0", "transfer VARb fault 3 1"]
    print("ok")
