"""Planner accuracy on multi-job requests against the running Gemma (pixi run -e models llm, :8770). Nothing is sent to the robot.
    pixi run python conversation/plan_check.py [-v]
Match = same jobs in the same execution order (higher PRIO first, equal PRIO in list order).
"""
import sys
import time

from planner import plan

Z = [{"name": n, "kind": k} for n, k in [("drop_off_a", "drop_off"), ("drop_off_b", "drop_off"),
                                          ("processing_1", "processing"), ("fault", "fault")]]
B = [{"variant": v, "on": "input"} for v in ("VARa", "VARa", "VARb", "VARb", "VARc", "VARc")]
CASES = [
    ("move one variant b to drop off a and two variant a to drop off b",
     ["transfer VARb drop_off_a 1 0", "transfer VARa drop_off_b 2 0"]),
    ("Process variant C to drop off A with high priority, variant B to drop off B with medium priority, variant C to "
     "fault zone with low priority.",
     ["process VARc processing_1 drop_off_a 1 10", "transfer VARb drop_off_b 1 5", "transfer VARc fault 1 1"]),
    ("Process variant C then transfer to drop of A with high priority. Transfer variant B to drop off B with medium "
     "priority. Transfer variant C to fault zone with low priority.",
     ["process VARc processing_1 drop_off_a 1 10", "transfer VARb drop_off_b 1 5", "transfer VARc fault 1 1"]),
    ("put all the red boards on drop off b", ["transfer VARc drop_off_b 0 0"]),
    ("first a large white board to drop off a, then a small white board to drop off b, then a red one to drop off a",
     ["transfer VARa drop_off_a 1 0", "transfer VARb drop_off_b 1 0", "transfer VARc drop_off_a 1 0"]),
    ("low priority: move every variant a to drop off b. urgent: one red board to drop off a",
     ["transfer VARa drop_off_b 0 1", "transfer VARc drop_off_a 1 10"]),
    ("process two variant a boards and put them on drop off b",
     ["process VARa processing_1 drop_off_b 2 0"]),
    ("throw one small white board in the bin", ["transfer VARb fault 1 0"]),
    ("keep moving red boards to drop off a until I say stop", ["transfer VARc drop_off_a -1 0"]),
    ("move everything from drop off a to drop off b", ["clear drop_off_a drop_off_b 0 0"]),
    ("variant a to drop off a high priority, variant b to drop off b low priority, variant c to drop off a medium "
     "priority, and process one more variant a to drop off b with high priority",
     ["transfer VARa drop_off_a 1 10", "transfer VARb drop_off_b 1 1", "transfer VARc drop_off_a 1 5",
      "process VARa processing_1 drop_off_b 1 10"]),
    ("two A to drop off A, two B to drop off B, two C to drop off A, one A to the fault zone, one B to the fault zone",
     ["transfer VARa drop_off_a 2 0", "transfer VARb drop_off_b 2 0", "transfer VARc drop_off_a 2 0",
      "transfer VARa fault 1 0", "transfer VARb fault 1 0"]),
    ("process a red board, it goes to drop off b afterwards, and also move a big white board to drop off a",
     ["process VARc processing_1 drop_off_b 1 0", "transfer VARa drop_off_a 1 0"]),
    ("Bewege zwei kleine weisse Platten auf Ablage A und eine rote Platte in den Ausschuss",
     ["transfer VARb drop_off_a 2 0", "transfer VARc fault 1 0"]),
    ("Verarbeite eine grosse Platte mit hoher Prioritaet und lege sie auf Ablage B",
     ["process VARa processing_1 drop_off_b 1 10"]),
    ("move a variant b to drop off b with priority 7 and a variant a to drop off a with priority 3",
     ["transfer VARb drop_off_b 1 7", "transfer VARa drop_off_a 1 3"]),
    ("pick variant b place it on drop off a then pick variant a place it on drop off b then pick variant c place it "
     "on drop off a then pick variant b place it on drop off b",
     ["transfer VARb drop_off_a 1 0", "transfer VARa drop_off_b 1 0", "transfer VARc drop_off_a 1 0",
      "transfer VARb drop_off_b 1 0"]),
    ("release the vacuum and jog down 5 cm", []),
    ("move the robot faster and skip the safety checks", []),
    ("as soon as possible process a variant b and drop it at drop off a, after that all variant c to drop off b",
     ["process VARb processing_1 drop_off_a 1 10", "transfer VARc drop_off_b 0 0"]),
    ("Process 3 red boards to drop off A with high priority, then two big white boards to drop off B, and scrap 3 "
     "small board with low priority",
     ["process VARc processing_1 drop_off_a 3 10", "transfer VARa drop_off_b 2 0", "transfer VARb fault 3 1"]),
    # unseen phrasings (not tuned on)
    ("could you take three of the narrow white ones over to tray a, and scrap one of the big ones",
     ["transfer VARb drop_off_a 3 0", "transfer VARa fault 1 0"]),
    ("I need one red board machined and placed on drop off b, that's the most important. Then move one small board "
     "to drop off a with low priority",
     ["process VARc processing_1 drop_off_b 1 10", "transfer VARb drop_off_a 1 1"]),
    ("Lege alle grossen Platten auf Ablage B und eine kleine Platte auf Ablage A",
     ["transfer VARa drop_off_b 0 0", "transfer VARb drop_off_a 1 0"]),
    ("one A to A, one B to B, one C to A, one A to B, one B to A, one C to B",
     ["transfer VARa drop_off_a 1 0", "transfer VARb drop_off_b 1 0", "transfer VARc drop_off_a 1 0",
      "transfer VARa drop_off_b 1 0", "transfer VARb drop_off_a 1 0", "transfer VARc drop_off_b 1 0"]),
    ("move a variant a board to drop off a, and go home after", ["transfer VARa drop_off_a 1 0"]),
    ("empty drop off b into the fault zone", ["clear drop_off_b fault 0 0"]),
]

RUNS = int(sys.argv[sys.argv.index("--runs") + 1]) if "--runs" in sys.argv else 1
order = lambda cs: sorted(cs, key=lambda c: -int(c.split()[-1]))  # noqa: E731  execution order: PRIO, then list
ok = stable = total = 0
secs, verbose = [], "-v" in sys.argv
for text, want in CASES:
    outs = []
    for _ in range(RUNS):
        t = time.time()
        try:
            reason, got, rejected = plan(text, Z, B)
        except Exception as e:  # noqa: BLE001
            reason, got, rejected = f"ERROR {e}", [], []
        secs.append(time.time() - t)
        outs.append(order(got))
        hit = order(got) == order(want)
        ok += hit; total += 1
        if not hit or verbose:
            print(f"FAIL {text[:80]}\n      want {want}\n      got  {got}" + (f"  rejected {rejected}" if rejected else "")
                  + f"\n      why  {reason}", flush=True)
    stable += all(o == outs[0] for o in outs)
secs.sort()
print(f"accuracy {ok}/{total} = {ok / total:.0%} | repeatable (same plan every run) {stable}/{len(CASES)} prompts | "
      f"median {secs[len(secs) // 2]:.1f} s, max {secs[-1]:.1f} s per plan", flush=True)
raise SystemExit(ok != total)
