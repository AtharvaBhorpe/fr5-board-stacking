"""Replay saved synthetic command recordings through the live ASR, without robot actions."""
import argparse
import base64
import io
import json
from pathlib import Path
import time
import urllib.request
import wave

from policy import batch_request
from speech_input import normalize_voice_command

parser = argparse.ArgumentParser()
parser.add_argument('--port', type=int, default=8766)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
url = f'http://127.0.0.1:{args.port}'
health = json.load(urllib.request.urlopen(url + '/health'))
fixtures = Path(__file__).parent / 'results/voice-batch'
expected = ['transfer VARb drop_off_b 2', 'transfer VARa drop_off_a 3', None]  # 2: 'sort all' dropped
rows = []
for path in sorted(fixtures.glob('*.wav')):
    began = time.monotonic()
    request = urllib.request.Request(url + '/transcribe', json.dumps(
        {'wav': base64.b64encode(path.read_bytes()).decode()}).encode(),
        {'Content-Type': 'application/json'})
    result = json.load(urllib.request.urlopen(request, timeout=60))
    normalized = normalize_voice_command(result['text'])
    actual = batch_request(normalized)
    row = dict(fixture=path.name, transcript=result['text'], normalized=normalized, command=actual,
               seconds=round(time.monotonic()-began, 3),
               passed=actual == expected[int(path.name[0])])
    rows.append(row)
    print(json.dumps(row), flush=True)

data = io.BytesIO()
with wave.open(data, 'wb') as wav:
    wav.setnchannels(1)
    wav.setsampwidth(2)
    wav.setframerate(16000)
    wav.writeframes(bytes(64000))
request = urllib.request.Request(url + '/transcribe', json.dumps(
    {'wav': base64.b64encode(data.getvalue()).decode()}).encode(),
    {'Content-Type': 'application/json'})
silence = json.load(urllib.request.urlopen(request, timeout=60))
rows.append(dict(fixture='two_seconds_silence', transcript=silence['text'],
                 passed=not silence['text'].strip()))
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(dict(health=health, synthetic=True,
    human_microphone_test=False, cases=rows), indent=2) + '\n')
assert all(row['passed'] for row in rows), 'See saved ASR results'
