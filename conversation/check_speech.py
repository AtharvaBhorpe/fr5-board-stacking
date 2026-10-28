"""Audible local speech smoke test: playback, interruption and replacement; no microphone capture."""
import json
import argparse
from pathlib import Path
import time
import urllib.request

parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, default=Path(__file__).parent / 'results/phonon-live.json')
args = parser.parse_args()


def request(path="", body=None):
    req = urllib.request.Request("http://127.0.0.1:8766" + path,
        json.dumps(body).encode() if body is not None else None,
        {"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as response:
        return json.load(response)


def wait_for(predicate, timeout=10):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        status = request()["speech_output"]
        assert status["error"] is None, status
        if predicate(status):
            return status
        time.sleep(.02)
    raise AssertionError("Speech state timed out: " + str(status))


assert request()["speech_output"]["engine"] == "phonon"
request("/speak", {"text": "I'm now using Phonon for live speech. The robot is in simulation."})
wait_for(lambda s: s["playing"])
wait_for(lambda s: not s["active"])
request("/speak", {"text": "This longer sentence tests whether speech can be interrupted promptly. " * 5})
wait_for(lambda s: s["playing"])
started = time.monotonic()
request("/silence", {})
wait_for(lambda s: not s["active"] and not s["playing"], timeout=2)
stop_seconds = time.monotonic()-started
assert stop_seconds < 1, stop_seconds
request("/speak", {"text": "This sentence should be replaced before it finishes. " * 5})
wait_for(lambda s: s["playing"])
request("/speak", {"text": "Speech replacement works."})
wait_for(lambda s: s["playing"])
wait_for(lambda s: not s["active"])
result = {"passed": True, "engine": "phonon", "normal_playback": True,
          "stop_seconds": stop_seconds, "replacement": True,
          "microphone_captured": False, "status": request()["speech_output"]}
path = args.output
path.write_text(json.dumps(result, indent=2)+"\n")
print(json.dumps(result, indent=2))
