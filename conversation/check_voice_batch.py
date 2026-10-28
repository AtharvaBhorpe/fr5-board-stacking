"""Synthetic speech -> actual local ASR -> bounded batch parsing; no robot commands."""
import base64
import io
import json
from pathlib import Path
import urllib.request
import wave

import numpy as np
import ptts

from scheduler import batch_request
from speech import MODEL
from speech_input import normalize_voice_command

ptts.set_num_threads(4)
tts = ptts.TTS(MODEL, lang='en', device='cpu', quant='q8')
output = Path(__file__).parent / 'results/voice-batch'
output.mkdir(exist_ok=True)
rows = []
for index, phrase in enumerate(('Pick two variant B boards.', 'Pick three variant A boards.',
                                 'Sort all visible boards.')):
    with tts.stream(phrase, seed=42) as stream:
        audio = np.concatenate([chunk.copy() for chunk in stream])
    audio = np.interp(np.arange(round(len(audio)*16000/tts.sample_rate))*tts.sample_rate/16000,
                      np.arange(len(audio)), audio)
    for label, scale in (('normal', 1.0), ('quiet', .08)):
        data = io.BytesIO()
        with wave.open(data, 'wb') as wav:
            wav.setnchannels(1); wav.setsampwidth(2); wav.setframerate(16000)
            wav.writeframes((np.clip(audio*scale, -1, 1)*32767).astype('<i2').tobytes())
        path = output / f'{index}-{label}.wav'
        path.write_bytes(data.getvalue())
        request = urllib.request.Request('http://127.0.0.1:8766/transcribe',
            json.dumps({'wav': base64.b64encode(data.getvalue()).decode()}).encode(),
            {'Content-Type': 'application/json'})
        result = json.load(urllib.request.urlopen(request, timeout=30))
        normalized = normalize_voice_command(result['text'])
        passed = batch_request(normalized) == batch_request(phrase)
        row = dict(input=phrase, level=label, transcript=result['text'], normalized=normalized,
                   passed=passed, wav=str(path))
        rows.append(row); print(json.dumps(row), flush=True)
(output / 'asr.json').write_text(json.dumps({'synthetic': True, 'human_microphone_test': False,
                                          'cases': rows}, indent=2)+'\n')
assert all(r['passed'] for r in rows), 'See ASR evidence for failures'
