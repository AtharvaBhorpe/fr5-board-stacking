"""Repeatable local inference/ASR measurements on synthetic speech and a recorded scene."""
import base64
import io
import json
import os
from pathlib import Path
import subprocess
import time
import urllib.request
import wave

import numpy as np
from policy import intent, visual_agreement

root = Path(__file__).resolve().parents[1]
image = base64.b64encode((root / "samples/05.jpg").read_bytes()).decode()
scene = {"source": "recorded sample 05", "live": False, "calibrated": True,
         "boards": [{"variant": "VARb"}, {"variant": "VARb"}, {"variant": "VARc"}],
         "destinations": [{"variant": v} for v in ["VARa", "VARb", "VARc"]]}


def post(path, body):
    req = urllib.request.Request("http://127.0.0.1:8766"+path, json.dumps(body).encode(),
                                 {"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=40) as response:
        return json.load(response)


rows = []
for index in range(30):
    variant = ["VARb", "VARc", None][index % 3]
    text = "pick " + variant if variant else "Describe the visible scene in one short sentence."
    start = time.monotonic()
    result = post("/ask", {"text": text, "variant": variant, "scene": scene,
                           "state": {"hardware": "SIMULATION"}, "images": [image]})
    rows.append({"kind": "vlm", "input": text, "seconds": time.monotonic()-start,
                 "agreement": visual_agreement(result) if variant else None, "result": result})
    print(json.dumps(rows[-1]), flush=True)

phrases = ["Pick variant A.", "Pick variant B.", "Pick variant C.", "Stop.", "Robot status.",
           "Do not pick variant B.", "Pick two variant A boards.", "Pick that white board.",
           "What do you see?", "Confirm descent."]
for index in range(30):
    phrase = phrases[index % len(phrases)]
    wav_data = subprocess.check_output(["espeak-ng", "--stdout", "-v", "en-us", "-s", "150", "--", phrase])
    with wave.open(io.BytesIO(wav_data), "rb") as wav:
        source_rate = wav.getframerate()
        raw = np.frombuffer(wav.readframes(1_000_000), np.int16)
    pcm = np.interp(np.arange(0, len(raw), source_rate / 16000), np.arange(len(raw)), raw).clip(-32768, 32767).astype(np.int16)
    buf = io.BytesIO()
    with wave.open(buf, "wb") as wav:
        wav.setnchannels(1); wav.setsampwidth(2); wav.setframerate(16000); wav.writeframes(pcm.tobytes())
    start = time.monotonic()
    result = post("/transcribe", {"wav": base64.b64encode(buf.getvalue()).decode()})
    expected, actual = intent(phrase), intent(result["text"])
    rows.append({"kind": "synthetic_asr", "input": phrase, "transcript": result["text"],
                 "seconds": time.monotonic()-start, "intent_matches": expected == actual,
                 "wrong_action": actual.kind == "pick" and actual != expected})
    print(json.dumps(rows[-1]), flush=True)

summary = {"vlm_turns": 30, "vlm_complete_p95_s": float(np.percentile([r["seconds"] for r in rows[:30]], 95)),
           "visual_agreement_passes": sum(r["agreement"] is True for r in rows[:30]),
           "visual_agreement_cases": 20,
           "synthetic_asr_turns": 30, "asr_p95_s": float(np.percentile([r["seconds"] for r in rows[30:]], 95)),
           "asr_intent_matches": sum(r["intent_matches"] for r in rows[30:]),
           "asr_wrong_actions": sum(r["wrong_action"] for r in rows[30:]),
           "limitations": "Synthetic speech, recorded scene. Not live microphone or full PTT-to-spoken-answer latency."}
destination = root / "conversation/results"
destination.mkdir(exist_ok=True)
(destination / "benchmark.json").write_text(json.dumps({"summary": summary, "cases": rows}, indent=2)+"\n")
print(json.dumps(summary, indent=2), flush=True)
